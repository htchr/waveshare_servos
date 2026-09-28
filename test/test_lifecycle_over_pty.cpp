// Full lifecycle over an openpty() fake bus; each case must set its own port (default: bench).
// See docs/development.md, "Keep tests off the bench".

#include <gmock/gmock.h>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <system_error>
#include <tuple>
#include <utility>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rcutils/logging.h"

#include "fake_servo_bus.hpp"
#include "test_helpers.hpp"

namespace
{

using ::testing::Contains;
using ::testing::ElementsAreArray;
using ::testing::HasSubstr;
using ::testing::IsEmpty;
using ::testing::Not;
using lifecycle_msgs::msg::State;

// Per-name declarations, never a using-directive: cpplint's build/namespaces rule forbids
// using-directives in sources as well as headers.
using waveshare_servos_test::FakeBus;
using waveshare_servos_test::FakeServo;
using waveshare_servos_test::Joint;
using waveshare_servos_test::LogCapture;
using waveshare_servos_test::command;
using waveshare_servos_test::kBenchName;
using waveshare_servos_test::kRegAcc;
using waveshare_servos_test::kRegGoalSpeed;
using waveshare_servos_test::kRegMode;
using waveshare_servos_test::process_has_serial_port_open;
using waveshare_servos_test::resource_manager_params;
using waveshare_servos_test::robot_description;
using waveshare_servos_test::sign_magnitude_decode;
using waveshare_servos_test::state;

// Driver defaults written out, not included, so the expected values do not come from the
// header the driver computes them with.
constexpr int kEncoderSteps = 4096;
constexpr double kNmPerKgfCm = 0.0980665;
constexpr double kCurrentPerCountA = 0.006;
constexpr double kTorqueConstantNmPerA = 0.8825985;
constexpr int kDefaultMaxSpeedCounts = 6000;
constexpr int kDefaultAccCounts = 150;
// the offset both position joints of these descriptions declare, as text and as the double
// hardware_interface::stod parses it to
constexpr char kOffsetText[] = "1.570796";
constexpr double kOffset = 1.570796;
constexpr char kPositionLimit[] = "1.570796";
// one whole turn of a position joint's command range maps to ticks 0..2048, so tick 1024 is
// joint zero and there is room either side of it
constexpr int kMidTick = 1024;

double rad_of_tick(int tick, double offset = kOffset, double sign = 1.0)
{
  return sign * (tick * 2.0 * M_PI / kEncoderSteps - offset);
}

// The driver's own goal expression (send_commands), reproduced: sign, then offset, then ticks.
double goal_steps_of(double command_rad, double sign = 1.0, double offset = kOffset)
{
  return (sign * command_rad + offset) * kEncoderSteps / (2.0 * M_PI);
}

std::vector<std::string> all_state_interfaces()
{
  return {
    state("position"), state("velocity"), state("effort"), state("current"), state("voltage"),
    state("temperature"), state("load"), state("status"), state("torque")};
}

// joint1/2: position servos 1/2; joint3/4: wheels 3/4. Wheel asserts use joint4: urdf_head
// limits joint1..3 to 0.2 rad/s and joint4 has no limit, so its commands arrive unmodified.
std::vector<Joint> four_servo_joints(const std::vector<std::string> & states)
{
  std::vector<Joint> joints;
  for (const std::string id : {"1", "2"}) {
    joints.push_back(
      Joint{
        "joint" + id, id, "pos", kOffsetText,
        {command("position", std::string("-") + kPositionLimit, kPositionLimit),
          command("velocity")},
        states, "", "", "", "", ""});
  }
  for (const std::string id : {"3", "4"}) {
    joints.push_back(
      Joint{"joint" + id, id, "vel", "", {command("velocity")}, states, "", "", "", "", ""});
  }
  return joints;
}

// one position joint on servo 1, with whatever state interfaces the case wants
Joint one_position_joint(const std::vector<std::string> & states)
{
  return Joint{
    "joint1", "1", "pos", kOffsetText,
    {command("position", std::string("-") + kPositionLimit, kPositionLimit)},
    states, "", "", "", "", ""};
}

// How many of this process's descriptors point at `path`, from /proc/self/fd. Immune to the
// descriptor the directory iterator itself holds, which a plain count of the directory is not.
size_t descriptors_on(const std::string & path)
{
  size_t count = 0;
  std::error_code ec;
  std::filesystem::directory_iterator it("/proc/self/fd", ec);
  if (ec) {
    return 0;
  }
  const std::filesystem::directory_iterator end;
  for (; it != end; it.increment(ec)) {
    if (ec) {
      break;
    }
    std::error_code link_ec;
    const std::filesystem::path target = std::filesystem::read_symlink(it->path(), link_ec);
    if (!link_ec && target.string() == path) {
      count++;
    }
  }
  return count;
}

size_t count_containing(const std::vector<std::string> & messages, const std::string & needle)
{
  return static_cast<size_t>(
    std::count_if(
      messages.begin(), messages.end(),
      [&needle](const std::string & message) {
        return message.find(needle) != std::string::npos;
      }));
}

// The frozen WARN of a refused acceleration write, matched literally wherever it is expected.
std::string acc_refused_for(int id)
{
  return "could not write the acceleration register of motor id '" + std::to_string(id) +
         "'; it will run unramped";
}

// One record of a SyncWritePosEx packet: ACC, goal position (sign-magnitude on bit 15, little
// endian), goal time, goal speed (src/SMS_STS.cpp:53-80).
struct GoalRecord
{
  int acc = 0;
  int position = 0;
  int time = 0;
  int speed = 0;
};

GoalRecord decode_goal_record(const std::vector<uint8_t> & record)
{
  GoalRecord goal;
  if (record.size() != 7) {
    return goal;
  }
  goal.acc = record[0];
  goal.position =
    sign_magnitude_decode(static_cast<uint16_t>(record[1] | (record[2] << 8)));
  goal.time = record[3] | (record[4] << 8);
  goal.speed = record[5] | (record[6] << 8);
  return goal;
}

// One wheel sync-write record: the goal speed only, sign-magnitude on bit 15. The wheel ACC
// is written separately. See docs/design.md, "Wheel acceleration".
int decode_speed_record(const std::vector<uint8_t> & record)
{
  if (record.size() != 2) {
    return std::numeric_limits<int>::min();
  }
  return sign_magnitude_decode(static_cast<uint16_t>(record[0] | (record[1] << 8)));
}

// A temp file, removed on scope exit: the only openable non-tty path a test can make.
class TempFile
{
public:
  explicit TempFile(const std::string & name)
  : path_(std::filesystem::temp_directory_path() / name)
  {
    const int fd = ::open(path_.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
    if (fd != -1) {
      ::close(fd);
      created_ = true;
    }
  }

  ~TempFile()
  {
    std::error_code ec;
    std::filesystem::remove(path_, ec);
  }

  TempFile(const TempFile &) = delete;
  TempFile & operator=(const TempFile &) = delete;
  TempFile(TempFile &&) = delete;
  TempFile & operator=(TempFile &&) = delete;

  std::string path() const {return path_.string();}

  // Whether the file exists. Assert it first: without it the case tests "no such port".
  bool ok() const {return created_;}

private:
  std::filesystem::path path_;
  bool created_ = false;
};

// Holds a state handle's writer lock, so the driver's non-blocking set_state() finds it busy.
// Derived because the mutex is a protected member of the loan.
class BusyStateHandle : public hardware_interface::LoanedStateInterface
{
public:
  explicit BusyStateHandle(hardware_interface::LoanedStateInterface && loaned)
  : hardware_interface::LoanedStateInterface(std::move(loaned)),
    lock_(state_interface_.get_mutex())
  {
  }

private:
  std::unique_lock<std::shared_mutex> lock_;
};

// the same, for a command handle, so the driver's non-blocking get_command() keeps its cache
class BusyCommandHandle : public hardware_interface::LoanedCommandInterface
{
public:
  explicit BusyCommandHandle(hardware_interface::LoanedCommandInterface && loaned)
  : hardware_interface::LoanedCommandInterface(std::move(loaned)),
    lock_(command_interface_.get_mutex())
  {
  }

private:
  std::unique_lock<std::shared_mutex> lock_;
};

}  // namespace

// The tests stay outside the anonymous namespace: cppcheck 2.13 reports a syntaxError for a
// TEST_F inside one.

// ---------------------------------------------------------------------------------------------
// The shared fixture: one pty, one fake bus, one resource manager.

class PtyFixture : public ::testing::Test
{
protected:
  void TearDown() override
  {
    // Check before rm_.reset(): shutdown closes the port and would hide a case that opened the
    // bench adapter. The pty (/dev/pts) matches none of the serial-port prefixes.
    EXPECT_FALSE(process_has_serial_port_open()) << "a case reached a real serial port";
    // Then the leak checks: after shutdown only the fixture's slave fd may remain on the port,
    // and no malformed packet may have gone out.
    rm_.reset();
    EXPECT_FALSE(process_has_serial_port_open());
    EXPECT_EQ(descriptors_on(fake_.port()), 1u) << "the component left the port open";
    EXPECT_EQ(fake_.bad_checksums(), 0u) << "the driver put a malformed packet on the wire";
    EXPECT_EQ(fake_.quiet_timeouts(), 0u) <<
      "wait_quiet() gave up; the sync-write assertions in this case are unreliable";
  }

  // The <hardware> param block: this fixture's own pty, plus whatever the case adds.
  std::string hardware_params(const std::string & extra = "") const
  {
    return "  <param name=\"port\">" + fake_.port() + "</param>\n" + extra;
  }

  // Returns success instead of asserting: ASSERT_* would exit only this helper. Callers use
  // ASSERT_TRUE(load(...)).
  [[nodiscard]] bool load(const std::string & urdf)
  {
    auto params = resource_manager_params(urdf);
    rm_ = std::make_unique<hardware_interface::ResourceManager>(params, false);
    return rm_->load_and_initialize_components(params);
  }

  // set_component_state takes the target by non-const reference, so name one.
  hardware_interface::return_type go(uint8_t id, const char * label)
  {
    rclcpp_lifecycle::State target(id, label);
    return rm_->set_component_state(kBenchName, target);
  }

  hardware_interface::return_type configure()
  {
    return go(State::PRIMARY_STATE_INACTIVE, hardware_interface::lifecycle_state_names::INACTIVE);
  }

  hardware_interface::return_type activate()
  {
    return go(State::PRIMARY_STATE_ACTIVE, hardware_interface::lifecycle_state_names::ACTIVE);
  }

  hardware_interface::return_type deactivate() {return configure();}

  hardware_interface::return_type cleanup()
  {
    return go(
      State::PRIMARY_STATE_UNCONFIGURED,
      hardware_interface::lifecycle_state_names::UNCONFIGURED);
  }

  hardware_interface::return_type shutdown()
  {
    return go(State::PRIMARY_STATE_FINALIZED, hardware_interface::lifecycle_state_names::FINALIZED);
  }

  void step_read(double period_seconds = 0.01)
  {
    const rclcpp::Duration period = rclcpp::Duration::from_seconds(period_seconds);
    // No wait_quiet(): read() is synchronous, so the fake has handled every request on return.
    std::ignore = rm_->read(now_, period);
    now_ = now_ + period;
  }

  void step_write(double period_seconds = 0.01)
  {
    std::ignore = rm_->write(now_, rclcpp::Duration::from_seconds(period_seconds));
    fake_.wait_quiet();
  }

  void step_cycle(double period_seconds = 0.01)
  {
    step_read(period_seconds);
    step_write(period_seconds);
  }

  double state_of(const std::string & key)
  {
    const auto loaned = rm_->claim_state_interface(key);
    return loaned.get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
  }

  double command_of(const std::string & key)
  {
    const auto loaned = rm_->claim_command_interface(key);
    return loaned.get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
  }

  void command_is(const std::string & key, double value)
  {
    auto loaned = rm_->claim_command_interface(key);
    EXPECT_TRUE(loaned.set_value(value)) << key;
  }

  // the last record this servo received at `address`, or an empty record if it received none
  std::vector<uint8_t> last_record(uint8_t id, uint8_t address) const
  {
    const FakeServo servo = fake_.snapshot(id);
    for (auto it = servo.sync_writes.rbegin(); it != servo.sync_writes.rend(); ++it) {
      if (it->first == address) {
        return it->second;
      }
    }
    return {};
  }

  GoalRecord last_goal(uint8_t id) const {return decode_goal_record(last_record(id, kRegAcc));}

  int last_speed(uint8_t id) const {return decode_speed_record(last_record(id, kRegGoalSpeed));}

  size_t record_count(uint8_t id) const {return fake_.snapshot(id).sync_writes.size();}

  std::vector<std::string> warnings() const {return log_.messages(RCUTILS_LOG_SEVERITY_WARN);}
  std::vector<std::string> errors() const {return log_.messages(RCUTILS_LOG_SEVERITY_ERROR);}
  std::vector<std::string> infos() const {return log_.messages(RCUTILS_LOG_SEVERITY_INFO);}
  std::vector<std::string> fatals() const {return log_.messages(RCUTILS_LOG_SEVERITY_FATAL);}

  // declared first, so it outlives the resource manager and the log capture
  FakeBus fake_;
  LogCapture log_;
  std::unique_ptr<hardware_interface::ResourceManager> rm_;
  rclcpp::Time now_{0, 0, RCL_STEADY_TIME};
};

// ---------------------------------------------------------------------------------------------
// LifecycleOverPty: four healthy servos, every state interface the driver serves.

class LifecycleOverPty : public PtyFixture
{
protected:
  void SetUp() override
  {
    fake_.add_servo(1, 0);
    fake_.add_servo(2, 0);
    fake_.add_servo(3, 1);
    fake_.add_servo(4, 1);
    fake_.set_position(1, kMidTick);
    fake_.set_position(2, kMidTick);
  }

  [[nodiscard]] bool load_four_servos(const std::string & extra_params = "")
  {
    return load(
      robot_description(
        kBenchName, four_servo_joints(all_state_interfaces()), hardware_params(extra_params)));
  }
};

TEST_F(LifecycleOverPty, configure_opens_the_pty_and_pings_every_servo)
{
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);

  for (uint8_t id = 1; id <= 4; id++) {
    EXPECT_GE(fake_.snapshot(id).pings, 1) << "servo " << static_cast<int>(id);
  }
  EXPECT_THAT(fatals(), IsEmpty());
  EXPECT_THAT(warnings(), Not(Contains(HasSubstr("unable to ping motor id"))));
  // the fixture's own slave descriptor, plus the bus's lock descriptor and the library's
  EXPECT_EQ(descriptors_on(fake_.port()), 3u);
}

TEST_F(LifecycleOverPty, configure_fails_when_the_port_is_already_locked)
{
  int other = ::open(fake_.port().c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  ASSERT_NE(other, -1) << std::strerror(errno);
  // Closed on every exit path, fatal assertion included: a descriptor that escaped into TearDown
  // would fail descriptors_on(port) == 1 there and blame the driver for the test's own leak.
  const std::unique_ptr<int, void (*)(int *)> guard(
    &other, [](int * fd) {
      if (*fd != -1) {
        ::close(*fd);
      }
    });
  ASSERT_EQ(::flock(other, LOCK_EX | LOCK_NB), 0) << std::strerror(errno);

  ASSERT_TRUE(load_four_servos());
  EXPECT_EQ(configure(), hardware_interface::return_type::ERROR);
  EXPECT_THAT(fatals(), Contains(HasSubstr("another process holds the lock on port")));
  // the refusal left nothing behind: the fixture's slave and this case's descriptor, no more
  EXPECT_EQ(descriptors_on(fake_.port()), 2u);
}

TEST_F(LifecycleOverPty, configure_fails_when_the_port_does_not_exist)
{
  const std::string missing = "/dev/waveshare_servos_no_such_port";
  ASSERT_TRUE(
    load(
      robot_description(
        kBenchName, four_servo_joints(all_state_interfaces()),
        "  <param name=\"port\">" + missing + "</param>\n")));

  EXPECT_EQ(configure(), hardware_interface::return_type::ERROR);
  EXPECT_THAT(fatals(), Contains(HasSubstr("port '" + missing + "' does not exist")));
  EXPECT_EQ(descriptors_on(missing), 0u);
  // and this case never went near the pty
  EXPECT_EQ(descriptors_on(fake_.port()), 1u);
}

TEST_F(LifecycleOverPty, cleanup_releases_the_port_so_configure_can_run_again)
{
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  EXPECT_EQ(descriptors_on(fake_.port()), 3u);

  ASSERT_EQ(cleanup(), hardware_interface::return_type::OK);
  EXPECT_EQ(descriptors_on(fake_.port()), 1u) << "only the pty's own slave is left";

  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  EXPECT_EQ(descriptors_on(fake_.port()), 3u);
  EXPECT_THAT(fatals(), IsEmpty());
}

TEST_F(LifecycleOverPty, activate_enables_torque_and_seeds_the_commands_from_the_measurement)
{
  fake_.set_position(1, 1500);
  fake_.set_position(2, 600);
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);

  const FakeServo before = fake_.snapshot(1);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  // a servo whose torque has been latched off accepts goal positions and quietly ignores them
  EXPECT_EQ(fake_.snapshot(1).torque_enable_writes, before.torque_enable_writes + 1);
  for (uint8_t id = 2; id <= 4; id++) {
    EXPECT_GE(fake_.snapshot(id).torque_enable_writes, 1) << static_cast<int>(id);
  }
  // the position commands start where the servos actually are, so nothing moves on activation
  EXPECT_DOUBLE_EQ(command_of("joint1/position"), rad_of_tick(1500));
  EXPECT_DOUBLE_EQ(command_of("joint2/position"), rad_of_tick(600));
  // and the wheels start stopped
  EXPECT_DOUBLE_EQ(command_of("joint3/velocity"), 0.0);
  EXPECT_DOUBLE_EQ(command_of("joint4/velocity"), 0.0);
}

TEST_F(LifecycleOverPty, read_publishes_every_declared_state_in_its_unit)
{
  fake_.set_feedback(1, 1500, -200, -300, 118, 41, 1, 250);
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  step_read();

  const double amps = 250 * kCurrentPerCountA;
  EXPECT_DOUBLE_EQ(state_of("joint1/position"), rad_of_tick(1500));
  EXPECT_DOUBLE_EQ(state_of("joint1/velocity"), -200 * 2.0 * M_PI / kEncoderSteps);
  EXPECT_DOUBLE_EQ(state_of("joint1/current"), amps);
  EXPECT_DOUBLE_EQ(state_of("joint1/effort"), amps * kTorqueConstantNmPerA);
  EXPECT_DOUBLE_EQ(state_of("joint1/voltage"), 118 * 0.1);
  EXPECT_DOUBLE_EQ(state_of("joint1/temperature"), 41.0);
  EXPECT_DOUBLE_EQ(state_of("joint1/load"), -300 / 1000.0);
  EXPECT_DOUBLE_EQ(state_of("joint1/status"), 0.0);
  EXPECT_NEAR(state_of("joint1/torque"), amps * kTorqueConstantNmPerA / kNmPerKgfCm, 1e-12);
}

TEST_F(LifecycleOverPty, the_torque_interface_reports_kg_cm_while_effort_reports_newton_metres)
{
  fake_.set_feedback(1, 1500, 0, 0, 118, 41, 0, 250);
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  step_read();

  const double effort = state_of("joint1/effort");
  const double torque = state_of("joint1/torque");
  ASSERT_GT(effort, 0.0);
  // the deprecated `torque` alias keeps kg cm, the unit of the original driver
  EXPECT_NEAR(torque / effort, 1.0 / kNmPerKgfCm, 1e-9);
}

TEST_F(LifecycleOverPty, configure_and_read_succeed_for_a_joint_that_declares_only_position)
{
  fake_.set_position(1, 1500);
  ASSERT_TRUE(
    load(
      robot_description(
        kBenchName, {one_position_joint({state("position")})}, hardware_params())));
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  step_read();

  EXPECT_DOUBLE_EQ(state_of("joint1/position"), rad_of_tick(1500));
  EXPECT_FALSE(rm_->state_interface_exists("joint1/velocity"));
  EXPECT_FALSE(rm_->state_interface_exists("joint1/status"));
  EXPECT_THAT(fatals(), IsEmpty());
}

TEST_F(LifecycleOverPty, a_joint_with_no_state_interface_still_takes_commands)
{
  fake_.set_position(1, kMidTick);
  ASSERT_TRUE(load(robot_description(kBenchName, {one_position_joint({})}, hardware_params())));
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  EXPECT_THAT(rm_->state_interface_keys(), IsEmpty());

  command_is("joint1/position", 0.25);
  step_write();

  const GoalRecord goal = last_goal(1);
  EXPECT_EQ(goal.position, static_cast<int>(std::lround(goal_steps_of(0.25))));
  EXPECT_THAT(fatals(), IsEmpty());
}

TEST_F(LifecycleOverPty, write_sends_the_commanded_goal_position_and_the_paced_speed)
{
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  step_read();

  {
    SCOPED_TRACE("a short move is paced to arrive as the next setpoint is written");
    const double measured = state_of("joint1/position");
    const double target = measured + 0.01;
    command_is("joint1/position", target);
    step_write(0.01);

    const double goal_steps = goal_steps_of(target);
    const double now_steps = goal_steps_of(measured);
    const GoalRecord goal = last_goal(1);
    EXPECT_EQ(goal.position, static_cast<int>(std::lround(goal_steps)));
    EXPECT_EQ(goal.time, 0);
    EXPECT_EQ(
      goal.speed,
      static_cast<int>(
        static_cast<uint16_t>(
          std::clamp(std::fabs(goal_steps - now_steps) / 0.01, 1.0, 6000.0))));
    EXPECT_GT(goal.speed, 1);
  }
  {
    SCOPED_TRACE("a move longer than one period saturates at the joint's speed ceiling");
    command_is("joint1/position", 1.5);
    step_write(0.01);
    EXPECT_EQ(last_goal(1).speed, kDefaultMaxSpeedCounts);
  }
}

TEST_F(LifecycleOverPty, a_zero_length_move_still_sends_a_goal_speed_of_one)
{
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  step_read();

  // exactly where the servo already is: the paced speed is 0, and 0 in the goal speed register
  // means "no speed limit", which is what made the servo lurch at the ends of every trajectory
  command_is("joint1/position", state_of("joint1/position"));
  step_write();

  EXPECT_EQ(last_goal(1).speed, 1);
}

TEST_F(LifecycleOverPty, inverted_flips_the_commanded_position_and_the_wheel_speed)
{
  const double position_command = 0.2;
  const double wheel_command = 1.0;
  for (const bool inverted : {false, true}) {
    SCOPED_TRACE(inverted ? "inverted=true" : "inverted=false");
    const double sign = inverted ? -1.0 : 1.0;
    std::vector<Joint> joints = four_servo_joints(all_state_interfaces());
    if (inverted) {
      joints[0].inverted = "true";
      joints[3].inverted = "true";
    }
    ASSERT_TRUE(load(robot_description(kBenchName, joints, hardware_params())));
    ASSERT_EQ(configure(), hardware_interface::return_type::OK);
    ASSERT_EQ(activate(), hardware_interface::return_type::OK);
    step_read();

    command_is("joint1/position", position_command);
    command_is("joint4/velocity", wheel_command);
    step_write();

    EXPECT_EQ(
      last_goal(1).position,
      static_cast<int>(std::lround(goal_steps_of(position_command, sign))));
    EXPECT_EQ(
      last_speed(4),
      static_cast<int>(std::lround(sign * wheel_command * kEncoderSteps / (2.0 * M_PI))));
    ASSERT_EQ(shutdown(), hardware_interface::return_type::OK);
    rm_.reset();
  }
}

TEST_F(LifecycleOverPty, the_acceleration_record_carries_the_per_joint_max_accel)
{
  // 20 acc and 100 speed counts, declared in rad/s^2 and rad/s (counts = value * 4096 / 2 pi,
  // /100 more for acc): the converted values must reach the wire, not the 6000/150 defaults.
  const double max_accel = 20.0 * 100.0 * 2.0 * M_PI / kEncoderSteps;
  const double max_speed = 100.0 * 2.0 * M_PI / kEncoderSteps;
  std::vector<Joint> joints = four_servo_joints(all_state_interfaces());
  joints[0].max_accel = std::to_string(max_accel);
  joints[0].max_speed = std::to_string(max_speed);
  ASSERT_TRUE(load(robot_description(kBenchName, joints, hardware_params())));
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  step_read();

  command_is("joint1/position", 1.5);
  command_is("joint2/position", 1.5);
  step_write();

  EXPECT_EQ(last_goal(1).acc, 20);
  EXPECT_EQ(last_goal(1).speed, 100) << "the declared max_speed, not the 6000-count default";
  EXPECT_EQ(last_goal(2).acc, kDefaultAccCounts);
  EXPECT_EQ(last_goal(2).speed, kDefaultMaxSpeedCounts);
}

// Golden bytes captured from the vendored SMS_STS::SyncWritePosEx before the wrapper built the
// record; they pin the wrapper's record builder to the vendored byte format.
TEST_F(LifecycleOverPty, the_position_record_matches_the_vendored_byte_format)
{
  // Both goals saturate the paced speed at 6000, so the bytes depend on the command only. The
  // 1.570796 rad offset keeps even a negative command at a positive tick.
  struct Golden
  {
    const char * trace;
    double command_rad;
    std::array<uint8_t, 7> record;  // {acc, pos lo, pos hi, time lo, time hi, speed lo, speed hi}
    int acc;
    int position;
    int speed;
  };
  const std::vector<Golden> goldens = {
    {"a positive goal", 0.5, {0x96, 0x46, 0x05, 0x00, 0x00, 0x70, 0x17}, 150, 1350, 6000},
    {"a negative goal", -0.5, {0x96, 0xba, 0x02, 0x00, 0x00, 0x70, 0x17}, 150, 698, 6000}};

  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  step_read();

  for (const Golden & golden : goldens) {
    SCOPED_TRACE(golden.trace);
    command_is("joint1/position", golden.command_rad);
    step_write();

    const std::vector<uint8_t> record = last_record(1, kRegAcc);
    EXPECT_THAT(record, ElementsAreArray(golden.record));
    // Bytes catch a byte-order or width change; the decoded fields name the field that drifted.
    const GoalRecord goal = decode_goal_record(record);
    EXPECT_EQ(goal.acc, golden.acc);
    EXPECT_EQ(goal.position, golden.position);
    EXPECT_EQ(goal.time, 0);
    EXPECT_EQ(goal.speed, golden.speed);
  }
  EXPECT_THAT(fatals(), IsEmpty());
}

TEST_F(LifecycleOverPty, write_sends_one_sync_write_per_cycle_for_the_position_joints)
{
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  const size_t before1 = record_count(1);
  const size_t before2 = record_count(2);
  const size_t before3 = record_count(3);
  for (int cycle = 0; cycle < 5; cycle++) {
    step_cycle();
  }

  EXPECT_EQ(record_count(1), before1 + 5);
  EXPECT_EQ(record_count(2), before2 + 5);
  EXPECT_EQ(record_count(3), before3 + 5);
  const FakeServo servo = fake_.snapshot(1);
  for (const auto & entry : servo.sync_writes) {
    EXPECT_EQ(entry.first, kRegAcc);
    EXPECT_EQ(entry.second.size(), 7u);
  }
  const FakeServo wheel = fake_.snapshot(3);
  for (const auto & entry : wheel.sync_writes) {
    EXPECT_EQ(entry.first, kRegGoalSpeed);
    EXPECT_EQ(entry.second.size(), 2u);
  }
}

// Fails if the wheel ACC write returns to the per-cycle path (vendored SyncWriteSpe: one acked
// register-41 write per wheel per cycle). See docs/design.md, "Wheel acceleration".
TEST_F(LifecycleOverPty, a_wheel_gets_no_per_cycle_acc_write)
{
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  const FakeServo before3 = fake_.snapshot(3);
  const FakeServo before4 = fake_.snapshot(4);
  for (int cycle = 0; cycle < 20; cycle++) {
    step_cycle();
  }

  // No addressed write of any kind: a wheel gets only its record in the broadcast sync write,
  // which is never acked (src/SCS.cpp:132,268).
  EXPECT_EQ(fake_.snapshot(3).writes, before3.writes);
  EXPECT_EQ(fake_.snapshot(4).writes, before4.writes);
  // it is still polled every cycle, so this is a silence on the write path alone
  EXPECT_EQ(fake_.snapshot(3).requests, before3.requests + 20);
}

// Wheel ACC edge 1 of 4: build_groups(), which on_configure reaches after the absent-servo
// gate.
TEST_F(LifecycleOverPty, configure_writes_each_wheel_acc_once)
{
  // max_accel="0" (no ramp) must be written, not skipped. Poison the register first: an
  // untouched register also reads 0.
  std::vector<Joint> joints = four_servo_joints(all_state_interfaces());
  joints[3].max_accel = "0";
  fake_.set_byte(4, kRegAcc, 99);
  ASSERT_TRUE(load(robot_description(kBenchName, joints, hardware_params())));
  const FakeServo before3 = fake_.snapshot(3);
  const FakeServo before4 = fake_.snapshot(4);

  ASSERT_EQ(configure(), hardware_interface::return_type::OK);

  EXPECT_EQ(fake_.snapshot(3).mem[kRegAcc], kDefaultAccCounts);
  EXPECT_EQ(fake_.snapshot(4).mem[kRegAcc], 0);
  // One addressed write per wheel: both are already in mode 1, so set_mode() leaves register
  // 33 alone and the ACC write is the only write on the wire.
  EXPECT_EQ(fake_.snapshot(3).writes, before3.writes + 1);
  EXPECT_EQ(fake_.snapshot(4).writes, before4.writes + 1);
  EXPECT_EQ(fake_.snapshot(3).mode_writes, 0);
  // and a position joint gets no addressed write: its ACC is byte 0 of every goal record
  EXPECT_EQ(fake_.snapshot(1).writes, 0);
  EXPECT_EQ(fake_.snapshot(2).writes, 0);
}

// Wheel ACC edge 2 of 4: the per-joint loop in on_activate.
TEST_F(LifecycleOverPty, activate_writes_the_acceleration_register_once_per_servo)
{
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  // Cleared after configure so only the activation's writes count. Count deltas: an activation
  // that recovers a servo writes register 41 twice (edges 1 and 2).
  fake_.set_byte(3, kRegAcc, 0);
  fake_.set_byte(4, kRegAcc, 0);
  const FakeServo before3 = fake_.snapshot(3);
  const FakeServo before4 = fake_.snapshot(4);

  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  EXPECT_EQ(fake_.snapshot(3).mem[kRegAcc], kDefaultAccCounts);
  EXPECT_EQ(fake_.snapshot(4).mem[kRegAcc], kDefaultAccCounts);
  // Two addressed writes: the torque enable and exactly one register-41 write. The fake keeps
  // no write order. See docs/design.md, "Wheel acceleration".
  EXPECT_EQ(fake_.snapshot(3).writes, before3.writes + 2);
  EXPECT_EQ(fake_.snapshot(3).torque_enable_writes, before3.torque_enable_writes + 1);
  EXPECT_EQ(fake_.snapshot(4).writes, before4.writes + 2);
  EXPECT_EQ(fake_.snapshot(4).torque_enable_writes, before4.torque_enable_writes + 1);

  const FakeServo activated = fake_.snapshot(3);
  for (int cycle = 0; cycle < 20; cycle++) {
    step_cycle();
  }
  EXPECT_EQ(fake_.snapshot(3).writes, activated.writes) << "20 cycles and not one addressed write";
  // meanwhile the position servo's acceleration keeps arriving for free, in byte 0 of its record
  EXPECT_EQ(last_goal(1).acc, kDefaultAccCounts);
  EXPECT_EQ(fake_.snapshot(1).mem[kRegAcc], kDefaultAccCounts);
}

TEST_F(LifecycleOverPty, activation_rewrites_the_wheel_acc)
{
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);
  // Power-cycle stand-in: register 41 is SRAM. The servo was never absent, so build_groups()
  // does not run and only edge 2 restores it.
  fake_.set_byte(3, kRegAcc, 0);

  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  EXPECT_EQ(fake_.snapshot(3).mem[kRegAcc], kDefaultAccCounts);
  EXPECT_THAT(infos(), Not(Contains(HasSubstr("answered on activation; adding it back"))));
}

TEST_F(LifecycleOverPty, a_failed_acc_write_warns_and_activation_still_succeeds)
{
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  // Gone between configure and activate, so present_ is true and the driver tries the write.
  // Not fatal: a wheel with a stale ramp still runs speed commands.
  fake_.set_absent(3, true);

  EXPECT_EQ(activate(), hardware_interface::return_type::OK);

  EXPECT_EQ(count_containing(warnings(), acc_refused_for(3)), 1u);
  EXPECT_EQ(count_containing(warnings(), acc_refused_for(4)), 0u);
}

// "Write the ACC once" is for wheels only: a position servo's ACC is byte 0 of every goal
// record, so a restart is repaired next cycle. Do not optimise this path.
TEST_F(LifecycleOverPty, a_position_joint_still_gets_its_acc_in_every_record)
{
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  const FakeServo before = fake_.snapshot(1);

  for (int cycle = 0; cycle < 5; cycle++) {
    SCOPED_TRACE("cycle " + std::to_string(cycle));
    fake_.set_byte(1, kRegAcc, 0);   // whatever a restart left behind
    step_cycle();
    EXPECT_EQ(last_goal(1).acc, kDefaultAccCounts);
    EXPECT_EQ(fake_.snapshot(1).mem[kRegAcc], kDefaultAccCounts);
  }
  EXPECT_EQ(fake_.snapshot(1).writes, before.writes) << "and it costs no addressed write";
}

// The driver passes an empty position group to the wrapper, which sends nothing. A fake
// servo cannot see an empty broadcast, so this checks that no position record arrives.
TEST_F(LifecycleOverPty, a_group_with_no_position_joints_sends_no_position_packet)
{
  std::vector<Joint> joints;
  for (const std::string id : {"3", "4"}) {
    joints.push_back(
      Joint{"joint" + id, id, "vel", "", {command("velocity")}, all_state_interfaces(),
        "", "", "", "", ""});
  }
  ASSERT_TRUE(load(robot_description(kBenchName, joints, hardware_params())));
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  const size_t before3 = record_count(3);
  const size_t before4 = record_count(4);
  for (int cycle = 0; cycle < 5; cycle++) {
    step_cycle();
  }

  EXPECT_EQ(record_count(3), before3 + 5);
  EXPECT_EQ(record_count(4), before4 + 5);
  for (const auto & entry : fake_.snapshot(3).sync_writes) {
    EXPECT_EQ(entry.first, kRegGoalSpeed) << "a position record reached a wheels-only bus";
    EXPECT_EQ(entry.second.size(), 2u);
  }
  EXPECT_THAT(errors(), IsEmpty());
}

TEST_F(LifecycleOverPty, deactivate_zeroes_the_wheel_speeds)
{
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  step_read();
  command_is("joint4/velocity", 1.0);
  step_write();
  ASSERT_NE(last_speed(4), 0);

  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);
  fake_.wait_quiet();

  EXPECT_EQ(last_speed(4), 0);
  EXPECT_EQ(last_speed(3), 0);
}

TEST_F(LifecycleOverPty, shutdown_from_active_stops_and_closes_the_port)
{
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  step_read();
  command_is("joint4/velocity", 1.0);
  step_write();
  ASSERT_NE(last_speed(4), 0);

  ASSERT_EQ(shutdown(), hardware_interface::return_type::OK);
  fake_.wait_quiet();

  EXPECT_EQ(last_speed(4), 0);
  EXPECT_EQ(descriptors_on(fake_.port()), 1u) << "only the pty's own slave is left";
}

TEST_F(LifecycleOverPty, a_joint_that_starts_outside_its_limits_is_held_there)
{
  // moved by hand with the torque off: tick 2500 is well past the 2048 that the +1.570796 rad
  // command limit maps to
  fake_.set_position(1, 2500);
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  EXPECT_THAT(warnings(), Contains(HasSubstr("joint 'joint1' starts at")));
  EXPECT_THAT(warnings(), Contains(HasSubstr("outside its limits")));

  // a command that is still outside the limits must not run it to the nearest one
  command_is("joint1/position", 3.0);
  step_write();

  EXPECT_EQ(last_goal(1).position, 2500);
}

TEST_F(LifecycleOverPty, shutdown_parks_each_position_joint_at_its_measured_position)
{
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  // it has moved since activation, and the controller still commands somewhere else
  fake_.set_position(1, 1300);
  fake_.set_position(2, 800);
  step_read();
  command_is("joint1/position", 1.4);
  step_write();

  ASSERT_EQ(shutdown(), hardware_interface::return_type::OK);
  fake_.wait_quiet();

  // the park goal is where the servo actually is, not the goal it had not reached yet
  EXPECT_EQ(last_goal(1).position, 1300);
  EXPECT_EQ(last_goal(2).position, 800);
}

TEST_F(LifecycleOverPty, a_servo_that_never_answered_a_feedback_read_gets_no_park_goal)
{
  // on the bus, answers pings, never answers the feedback read: it has no measured position, so
  // it has nowhere to be parked
  fake_.set_silent_feedback(1, true);
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);
  fake_.wait_quiet();

  const size_t silent_before = record_count(1);
  const size_t healthy_before = record_count(2);
  ASSERT_EQ(shutdown(), hardware_interface::return_type::OK);
  fake_.wait_quiet();

  EXPECT_EQ(record_count(1), silent_before) << "a made-up goal would have been sent";
  EXPECT_EQ(record_count(2), healthy_before + 1);
}

// The park compacts p_ids_ in place, so send_commands() must resize its goals every cycle;
// without that the answering servo gets a stale duplicate record.
TEST_F(LifecycleOverPty, the_pruned_park_writes_only_the_servos_that_answered)
{
  fake_.set_silent_feedback(1, true);
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);
  fake_.wait_quiet();

  const size_t silent_before = record_count(1);
  const size_t parked_before = record_count(2);
  const size_t wheel_before = record_count(3);
  ASSERT_EQ(shutdown(), hardware_interface::return_type::OK);
  fake_.wait_quiet();

  EXPECT_EQ(record_count(1), silent_before) << "a made-up goal would have been sent";
  EXPECT_EQ(record_count(2), parked_before + 1) <<
    "one position frame carrying exactly the pruned group, not a stale trailing record";
  // the velocity group is never pruned: each wheel still gets its stop, 00 00, no sign bit
  EXPECT_EQ(record_count(3), wheel_before + 1);
  EXPECT_EQ(last_speed(3), 0);
  EXPECT_EQ(last_speed(4), 0);
}

TEST_F(LifecycleOverPty, a_stale_sample_from_before_a_cleanup_is_not_used_as_a_park_position)
{
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  fake_.set_position(1, 1300);
  step_read();
  ASSERT_DOUBLE_EQ(state_of("joint1/position"), rad_of_tick(1300));
  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);
  ASSERT_EQ(cleanup(), hardware_interface::return_type::OK);

  // the sample above is from the previous session on this port; the servo answers pings but no
  // longer answers a feedback read
  fake_.set_silent_feedback(1, true);
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  const size_t stale_before = record_count(1);
  const size_t healthy_before = record_count(2);

  ASSERT_EQ(shutdown(), hardware_interface::return_type::OK);
  fake_.wait_quiet();

  EXPECT_EQ(record_count(1), stale_before) << "the pre-cleanup sample was used as a park goal";
  EXPECT_EQ(record_count(2), healthy_before + 1);
}

TEST_F(LifecycleOverPty, a_busy_command_handle_keeps_the_previous_command)
{
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  step_read();

  command_is("joint1/position", 0.25);
  step_write();
  const int accepted = last_goal(1).position;
  ASSERT_EQ(accepted, static_cast<int>(std::lround(goal_steps_of(0.25))));

  command_is("joint1/position", 0.75);
  {
    // the handle is busy for the whole of this write(): the driver's non-blocking get_command
    // fails and the joint keeps the command it had last cycle -- never NaN, never a jump
    const BusyCommandHandle busy(rm_->claim_command_interface("joint1/position"));
    step_write();
  }
  EXPECT_EQ(last_goal(1).position, accepted);

  // and it catches up on the next cycle, once the handle is free again
  step_write();
  EXPECT_EQ(last_goal(1).position, static_cast<int>(std::lround(goal_steps_of(0.75))));
}

TEST_F(LifecycleOverPty, a_busy_state_handle_keeps_the_previous_sample)
{
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  fake_.set_position(1, 1300);
  step_read();
  ASSERT_DOUBLE_EQ(state_of("joint1/position"), rad_of_tick(1300));

  fake_.set_position(1, 1400);
  {
    const BusyStateHandle busy(rm_->claim_state_interface("joint1/position"));
    step_read();
  }
  EXPECT_DOUBLE_EQ(state_of("joint1/position"), rad_of_tick(1300)) << "the sample was forced in";

  // the caches are published again every cycle, so it catches up on the next read
  step_read();
  EXPECT_DOUBLE_EQ(state_of("joint1/position"), rad_of_tick(1400));
}

TEST_F(LifecycleOverPty, read_after_activate_seeds_the_unwrapped_position)
{
  // A wheel unwraps by default: 4095 -> 12 must step forward 13 ticks, not back to 0.018 rad.
  // See docs/configuration.md, "Multi-turn wheel position".
  fake_.set_position(3, 4095);
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  step_read();

  // the first sample of an activation is the plain register reading, exactly what the driver
  // published before unwrapping existed
  EXPECT_DOUBLE_EQ(state_of("joint3/position"), rad_of_tick(4095, 0.0));
  EXPECT_NEAR(state_of("joint3/position"), 6.281651, 1e-6);

  fake_.set_position(3, 12);
  step_read();
  EXPECT_DOUBLE_EQ(state_of("joint3/position"), rad_of_tick(4108, 0.0));
  EXPECT_NEAR(state_of("joint3/position"), 6.301593, 1e-6);
  EXPECT_GT(state_of("joint3/position"), 2.0 * M_PI) << "it wrapped back into the first turn";
}

TEST_F(LifecycleOverPty, unwrap_false_on_a_wheel_reproduces_the_wrapping_position)
{
  // unwrap=false publishes the plain wrapping register; joint4 keeps the default and makes the
  // same moves, so the two are compared.
  std::vector<Joint> joints = four_servo_joints(all_state_interfaces());
  joints[2].unwrap = "false";
  fake_.set_position(3, 4095);
  fake_.set_position(4, 4095);
  ASSERT_TRUE(load(robot_description(kBenchName, joints, hardware_params())));
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  step_read();
  ASSERT_DOUBLE_EQ(state_of("joint3/position"), rad_of_tick(4095, 0.0));
  ASSERT_DOUBLE_EQ(state_of("joint4/position"), rad_of_tick(4095, 0.0));

  fake_.set_position(3, 12);
  fake_.set_position(4, 12);
  step_read();

  EXPECT_DOUBLE_EQ(state_of("joint3/position"), rad_of_tick(12, 0.0)) << "the wrapping register";
  EXPECT_DOUBLE_EQ(state_of("joint4/position"), rad_of_tick(4108, 0.0)) << "the unwrapped count";
  // and it goes on wrapping, turn after turn, rather than drifting by one revolution once
  for (const int tick : {2000, 3900, 100}) {
    fake_.set_position(3, tick);
    fake_.set_position(4, tick);
    step_read();
    EXPECT_DOUBLE_EQ(state_of("joint3/position"), rad_of_tick(tick, 0.0)) << "tick " << tick;
  }
  EXPECT_DOUBLE_EQ(state_of("joint4/position"), rad_of_tick(8292, 0.0)) << "two whole turns on";
}

// ---------------------------------------------------------------------------------------------
// Sync-read path: one INST_SYNC_READ per cycle, the activation probe, fallback, bus totals.

// The nine state interfaces of all four joints in a fixed order, to compare two transports
// value for value.
std::vector<double> all_states_of(
  const std::function<double(const std::string &)> & state_of)
{
  std::vector<double> values;
  for (int joint = 1; joint <= 4; joint++) {
    for (const char * const interface : {"position", "velocity", "effort", "current",
        "voltage", "temperature", "load", "status", "torque"})
    {
      values.push_back(state_of("joint" + std::to_string(joint) + "/" + interface));
    }
  }
  return values;
}

// Frozen INFO: the transport this activation chose.
std::string sync_read_announced(size_t servos)
{
  return "feedback for " + std::to_string(servos) +
         " servos travels in one sync read per cycle (INST_SYNC_READ)";
}

// Frozen fallback WARN, with the ids that answered FeedBack but not the burst.
std::string sync_read_fallback_for(const std::string & ids)
{
  return "sync read went unanswered by motor id(s) " + ids +
         "; falling back to one feedback read per servo for this activation";
}

TEST_F(LifecycleOverPty, read_uses_one_sync_read_for_every_present_servo)
{
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  // Baseline after activation: the probe burst and the seeding reads (one INST_READ per
  // joint) are not per-cycle traffic.
  const uint64_t bursts_before = fake_.sync_read_requests();
  std::array<FakeServo, 5> before{};
  for (uint8_t id = 1; id <= 4; id++) {
    before[id] = fake_.snapshot(id);
  }

  for (int cycle = 0; cycle < 10; cycle++) {
    fake_.set_position(1, kMidTick + cycle);
    step_read();
  }

  EXPECT_EQ(fake_.sync_read_requests(), bursts_before + 10) << "one burst per cycle, not per joint";
  for (uint8_t id = 1; id <= 4; id++) {
    SCOPED_TRACE("servo " + std::to_string(static_cast<int>(id)));
    const FakeServo servo = fake_.snapshot(id);
    EXPECT_EQ(servo.sync_reads, before[id].sync_reads + 10);
    // No addressed INST_READ on the real-time path; this also keeps bus totals at one
    // transaction per cycle.
    EXPECT_EQ(servo.reads, before[id].reads);
  }
  EXPECT_THAT(fake_.last_sync_read_ids(), ElementsAreArray(std::vector<uint8_t>{1, 2, 3, 4})) <<
    "the present servos, in URDF joint order";
  // and the burst really is where the states come from
  EXPECT_DOUBLE_EQ(state_of("joint1/position"), rad_of_tick(kMidTick + 9));
}

TEST_F(LifecycleOverPty, the_sync_read_path_is_announced_once_at_activate)
{
  // The transport is chosen once per activation, so it is logged once (per cycle would be 100
  // lines a second).
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  EXPECT_EQ(count_containing(infos(), sync_read_announced(4)), 1u);
  EXPECT_THAT(warnings(), Not(Contains(HasSubstr("falling back"))));

  for (int cycle = 0; cycle < 20; cycle++) {
    step_cycle();
  }
  EXPECT_EQ(count_containing(infos(), sync_read_announced(4)), 1u) << "once, not once per cycle";
}

TEST_F(LifecycleOverPty,
  read_falls_back_to_one_feedback_read_per_servo_when_sync_read_is_unanswered)
{
  // Firmware that accepts INST_SYNC_READ (0x82) but never answers is why FeedBack() is kept;
  // set_sync_read_supported(false) stands in for it.
  fake_.set_sync_read_supported(false);
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK) << "a silent burst is not a failure";

  EXPECT_EQ(count_containing(warnings(), sync_read_fallback_for("1, 2, 3, 4")), 1u);
  EXPECT_THAT(infos(), Not(Contains(HasSubstr("travels in one sync read"))));

  const uint64_t bursts_before = fake_.sync_read_requests();
  std::array<FakeServo, 5> before{};
  for (uint8_t id = 1; id <= 4; id++) {
    before[id] = fake_.snapshot(id);
  }
  fake_.set_position(1, kMidTick + 7);
  for (int cycle = 0; cycle < 10; cycle++) {
    step_read();
  }

  EXPECT_EQ(fake_.sync_read_requests(), bursts_before) << "not one more burst after the probe";
  for (uint8_t id = 1; id <= 4; id++) {
    SCOPED_TRACE("servo " + std::to_string(static_cast<int>(id)));
    EXPECT_EQ(fake_.snapshot(id).reads, before[id].reads + 10);
  }
  // and the states are still right, which is the half of the fallback that matters
  EXPECT_DOUBLE_EQ(state_of("joint1/position"), rad_of_tick(kMidTick + 7));
}

TEST_F(LifecycleOverPty, the_fallback_decision_is_taken_once_per_activation_and_retried_on_the_next)
{
  // Two bursts only: the probe and one retry, so one lost frame does not condemn sync read.
  // Re-probing every cycle would cost one io_timeout_ms per cycle.
  fake_.set_sync_read_supported(false);
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  const uint64_t before_activation = fake_.sync_read_requests();
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  for (int cycle = 0; cycle < 20; cycle++) {
    step_cycle();
  }
  EXPECT_EQ(fake_.sync_read_requests(), before_activation + 2);

  // The mode is not sticky across activations: firmware is not the only reason a burst can go
  // unanswered, so the next activation asks again rather than carrying the verdict forward.
  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);
  fake_.set_sync_read_supported(true);
  const uint64_t before_second = fake_.sync_read_requests();
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  EXPECT_EQ(count_containing(infos(), sync_read_announced(4)), 1u);

  const FakeServo before = fake_.snapshot(2);
  for (int cycle = 0; cycle < 5; cycle++) {
    step_read();
  }
  EXPECT_EQ(fake_.snapshot(2).sync_reads, before.sync_reads + 5);
  EXPECT_GT(fake_.sync_read_requests(), before_second);
}

TEST_F(LifecycleOverPty, feedback_mode_per_servo_reads_each_servo_and_sends_no_sync_read)
{
  // per_servo is the escape hatch and the bench A/B baseline: no INST_SYNC_READ at all, not
  // even the probe.
  ASSERT_TRUE(load_four_servos("  <param name=\"feedback_mode\">per_servo</param>\n"));
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  const FakeServo before = fake_.snapshot(2);

  for (int cycle = 0; cycle < 5; cycle++) {
    step_read();
  }

  EXPECT_EQ(fake_.sync_read_requests(), 0u);
  EXPECT_EQ(fake_.snapshot(2).sync_reads, 0);
  EXPECT_EQ(fake_.snapshot(2).reads, before.reads + 5);
  EXPECT_THAT(infos(), Not(Contains(HasSubstr("travels in one sync read"))));
}

TEST_F(LifecycleOverPty, feedback_mode_sync_read_fails_activation_when_the_bus_ignores_sync_read)
{
  // feedback_mode=sync_read pins the fast path: a bus that ignores sync read fails activation
  // instead of quietly falling back.
  fake_.set_sync_read_supported(false);
  ASSERT_TRUE(load_four_servos("  <param name=\"feedback_mode\">sync_read</param>\n"));
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);

  EXPECT_EQ(activate(), hardware_interface::return_type::ERROR);

  EXPECT_THAT(fatals(), Contains(HasSubstr("motor id(s) 1, 2, 3, 4")));
  EXPECT_THAT(warnings(), Not(Contains(HasSubstr("falling back"))));
}

TEST_F(LifecycleOverPty, a_sync_read_cycle_publishes_exactly_what_the_per_servo_path_publishes)
{
  // Both transports share decode(), so the states must match exactly. Exact only on the fake:
  // real servos dither voltage and temperature by +-1 LSB.
  fake_.set_feedback(1, kMidTick + 31, -400, 611, 122, 41, 1, -250);
  fake_.set_feedback(2, kMidTick - 17, 900, -33, 118, 39, 0, 77);
  fake_.set_feedback(3, 3000, -1, 1023, 125, 44, 1, 0);
  fake_.set_feedback(4, 77, 2, -1023, 119, 38, 0, 3);
  fake_.set_status(2, 0x20);
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  for (int cycle = 0; cycle < 5; cycle++) {
    step_read();
  }
  const auto reader = [this](const std::string & key) {return state_of(key);};
  const std::vector<double> sync_states = all_states_of(reader);
  ASSERT_GT(fake_.sync_read_requests(), 0u) << "this half did not run on the sync path";

  // A present wheel keeps its count across the cycle and no register changes, so the
  // unwrapped values must match exactly too.
  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);
  fake_.set_sync_read_supported(false);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  const uint64_t bursts = fake_.sync_read_requests();
  for (int cycle = 0; cycle < 5; cycle++) {
    step_read();
  }
  ASSERT_EQ(fake_.sync_read_requests(), bursts) << "this half did not run on the per-servo path";

  EXPECT_THAT(all_states_of(reader), ElementsAreArray(sync_states));
}

TEST_F(LifecycleOverPty, the_sync_read_path_publishes_exactly_what_the_per_servo_path_publishes)
{
  // Same claim through feedback_mode=per_servo, the arm the bench A/B comparison runs.
  fake_.set_feedback(1, kMidTick + 5, -120, 700, 121, 40, 1, -9);
  fake_.set_feedback(2, kMidTick - 9, 33, -12, 117, 37, 0, 4);
  fake_.set_feedback(3, 2047, -2048, 1023, 126, 45, 1, 1);
  fake_.set_feedback(4, 4095, 2048, -1, 118, 36, 0, -1);
  const auto reader = [this](const std::string & key) {return state_of(key);};

  ASSERT_TRUE(load_four_servos("  <param name=\"feedback_mode\">per_servo</param>\n"));
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  for (int cycle = 0; cycle < 3; cycle++) {
    step_read();
  }
  const std::vector<double> per_servo_states = all_states_of(reader);
  ASSERT_EQ(fake_.sync_read_requests(), 0u);
  // Destroyed explicitly rather than by the next load()'s assignment: the component holds the pty
  // exclusively, so the first resource manager has to be gone before the second configures.
  rm_.reset();

  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  for (int cycle = 0; cycle < 3; cycle++) {
    step_read();
  }
  ASSERT_GT(fake_.sync_read_requests(), 0u);

  EXPECT_THAT(all_states_of(reader), ElementsAreArray(per_servo_states));
}

TEST_F(LifecycleOverPty, a_timeout_below_the_sync_read_floor_is_warned_about_and_takes_the_fallback)
{
  // 2 ms is legal but below the 3 ms a 4-servo sync read needs: the value stays and the driver
  // reads per servo, with no burst. See docs/bus-timing.md, "Timeout floor".
  ASSERT_TRUE(load_four_servos("  <param name=\"io_timeout_ms\">2</param>\n"));
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  EXPECT_EQ(
    count_containing(
      warnings(),
      "io_timeout_ms 2 is below the 3 ms a sync read of 4 servos needs here; using one feedback "
      "read per servo"), 1u);
  const FakeServo before = fake_.snapshot(2);
  fake_.set_position(1, kMidTick + 12);
  for (int cycle = 0; cycle < 5; cycle++) {
    step_read();
  }

  EXPECT_EQ(fake_.sync_read_requests(), 0u) << "not even the probe";
  EXPECT_EQ(fake_.snapshot(2).reads, before.reads + 5);
  EXPECT_DOUBLE_EQ(state_of("joint1/position"), rad_of_tick(kMidTick + 12));
}

TEST_F(LifecycleOverPty, on_activate_and_stop_and_park_still_read_per_servo)
{
  // Activation and park read per servo (a one-id sync read measured 0.755 ms vs 0.750 ms for
  // FeedBack), so the probe is the only burst.
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);

  std::array<FakeServo, 5> before{};
  for (uint8_t id = 1; id <= 4; id++) {
    before[id] = fake_.snapshot(id);
  }
  const uint64_t bursts_before = fake_.sync_read_requests();
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  for (uint8_t id = 1; id <= 4; id++) {
    SCOPED_TRACE("activation, servo " + std::to_string(static_cast<int>(id)));
    EXPECT_EQ(fake_.snapshot(id).reads, before[id].reads + 1);
  }
  EXPECT_EQ(fake_.sync_read_requests(), bursts_before + 1) << "the probe, and nothing else";

  for (uint8_t id = 1; id <= 4; id++) {
    before[id] = fake_.snapshot(id);
  }
  const uint64_t bursts_after_activate = fake_.sync_read_requests();
  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);
  for (uint8_t id = 1; id <= 4; id++) {
    SCOPED_TRACE("park, servo " + std::to_string(static_cast<int>(id)));
    EXPECT_EQ(fake_.snapshot(id).reads, before[id].reads + 1);
  }
  EXPECT_EQ(fake_.sync_read_requests(), bursts_after_activate);
}

TEST_F(LifecycleOverPty, the_bus_totals_line_is_logged_once_per_activation)
{
  // One bus totals INFO per activation, once even for deactivate then shutdown. hil_gates.py
  // parses it. See docs/bus-timing.md, "Bus totals line".
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  for (int cycle = 0; cycle < 50; cycle++) {
    step_read();
  }
  EXPECT_THAT(infos(), Not(Contains(HasSubstr("bus totals:")))) << "not while the loop is running";

  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);

  // 50 reads, 50 transactions: the sync path is one transaction per cycle, whatever the servo
  // count.
  EXPECT_EQ(
    count_containing(
      infos(),
      "bus totals: transactions 50, failed 0 (0.0 per million), worst consecutive 0, dropped 0 "
      "[id1 0, id2 0, id3 0, id4 0]"), 1u);

  ASSERT_EQ(shutdown(), hardware_interface::return_type::OK);
  EXPECT_EQ(count_containing(infos(), "bus totals:"), 1u) << "once per activation, not per exit";
}

TEST_F(LifecycleOverPty, the_per_servo_path_counts_one_transaction_per_unicast_read)
{
  // Per-servo path: one transaction per unicast read (4 per cycle here), so its per-million
  // rate is comparable with the sync path's.
  ASSERT_TRUE(load_four_servos("  <param name=\"feedback_mode\">per_servo</param>\n"));
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  std::array<int, 5> before{};
  for (uint8_t id = 1; id <= 4; id++) {
    before[id] = fake_.snapshot(id).reads;
  }
  for (int cycle = 0; cycle < 5; cycle++) {
    step_read();
  }
  // 5 cycles x 4 answering servos = 20 round trips inside the counted window, measured at the
  // fake before the park read can add a fifth to each. That is the number the line must print.
  int round_trips = 0;
  for (uint8_t id = 1; id <= 4; id++) {
    round_trips += fake_.snapshot(id).reads - before[id];
  }
  ASSERT_EQ(round_trips, 20);

  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);

  EXPECT_EQ(
    count_containing(
      infos(),
      "bus totals: transactions 20, failed 0 (0.0 per million), worst consecutive 0, dropped 0 "
      "[id1 0, id2 0, id3 0, id4 0]"), 1u);
}

TEST_F(LifecycleOverPty, a_failed_unicast_read_is_one_failed_transaction_out_of_four)
{
  // Per-servo path: a silent servo fails only its own read, so 3 silent cycles are 3 of 12,
  // 250 000 per million.
  ASSERT_TRUE(load_four_servos("  <param name=\"feedback_mode\">per_servo</param>\n"));
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  fake_.set_silent_feedback(3, true);
  for (int cycle = 0; cycle < 3; cycle++) {
    step_read();
  }

  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);

  EXPECT_EQ(
    count_containing(
      infos(),
      "bus totals: transactions 12, failed 3 (250000.0 per million), worst consecutive 3, "
      "dropped 0 [id1 0, id2 0, id3 3, id4 0]"), 1u);
}

TEST_F(LifecycleOverPty, a_component_that_never_read_logs_no_bus_totals)
{
  // No cycle ran, so no bus totals line: an all-zero line is noise for grep-based gates.
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);

  EXPECT_THAT(infos(), Not(Contains(HasSubstr("bus totals:"))));
}


// ---------------------------------------------------------------------------------------------
// WaveshareServosAbsentServo: the allow_missing_servos configure gate (no servo on the bus).

class WaveshareServosAbsentServo : public PtyFixture
{
protected:
  // 30 ms (default 5) makes a missing-servo ping one known cost; max_read_fails 2 keeps the
  // drop case short. Sync read stays on; the dead-servo advisory WARN at load is tolerated.
  static constexpr int kIoTimeoutMs = 30;
  static constexpr int kMaxReadFails = 2;
  // The timing case's slow io timeout (4x) and ping attempts; 3 x 120 ms is the longest wait
  // in this file.
  static constexpr int kSlowIoTimeoutMs = 120;
  static constexpr int kPingAttempts = 3;
  static constexpr char kAllowMissing[] =
    "  <param name=\"allow_missing_servos\">true</param>\n";

  void SetUp() override
  {
    fake_.add_servo(1, 0);
    fake_.add_servo(2, 0);
    fake_.add_servo(3, 1);
    fake_.add_servo(4, 1);
    fake_.set_position(1, kMidTick);
    fake_.set_position(2, kMidTick);
  }

  // No allow_missing_servos param unless the case adds one: the gate's default must come from
  // the driver.
  [[nodiscard]] bool load_four_servos(const std::string & extra = "")
  {
    return load(
      robot_description(
        kBenchName, four_servo_joints(all_state_interfaces()),
        hardware_params(
          "  <param name=\"io_timeout_ms\">" + std::to_string(kIoTimeoutMs) + "</param>\n" +
          "  <param name=\"max_read_fails\">" + std::to_string(kMaxReadFails) + "</param>\n" +
          extra)));
  }

  // Io timeout and ping attempts set by the case: the timing case configures with two
  // timeouts.
  [[nodiscard]] bool load_four_servos_timed(int io_timeout_ms, int ping_attempts)
  {
    return load(
      robot_description(
        kBenchName, four_servo_joints(all_state_interfaces()),
        hardware_params(
          "  <param name=\"io_timeout_ms\">" + std::to_string(io_timeout_ms) + "</param>\n" +
          "  <param name=\"max_read_fails\">" + std::to_string(kMaxReadFails) + "</param>\n" +
          kAllowMissing +
          "  <param name=\"ping_attempts\">" + std::to_string(ping_attempts) + "</param>\n")));
  }

  // Wall-clock cost of one on_configure, in milliseconds.
  double configure_ms()
  {
    const auto started = std::chrono::steady_clock::now();
    const hardware_interface::return_type result = configure();
    const double elapsed =
      std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    EXPECT_EQ(result, hardware_interface::return_type::OK);
    return elapsed;
  }

  // the per-servo ping WARN, whose wording is frozen: the bench check greps for it
  static std::string skipped(int id)
  {
    const std::string name = "joint" + std::to_string(id);
    return "unable to ping motor id '" + std::to_string(id) + "'; joint '" + name +
           "' will be skipped on the bus";
  }
};

TEST_F(
  WaveshareServosAbsentServo,
  configure_fails_when_a_servo_is_missing_and_missing_is_not_allowed)
{
  // Two of them, so the FATAL has a list to join rather than a single name, and they are not
  // adjacent, so an off-by-one in the loop would show.
  fake_.set_absent(1, true);
  fake_.set_absent(3, true);
  ASSERT_TRUE(load_four_servos());

  EXPECT_EQ(configure(), hardware_interface::return_type::ERROR);

  EXPECT_EQ(
    count_containing(
      fatals(),
      "2 of 4 servos did not answer: id 1 (joint 'joint1'), id 3 (joint 'joint3'); refusing to "
      "configure because 'allow_missing_servos' is false"), 1u);
  // and the gate message is the only FATAL of the refusal path: bench scenario H5A expects
  // exactly one
  EXPECT_EQ(fatals().size(), 1u);
  // the per-servo ping WARN still fires once per missing servo, wording unchanged
  EXPECT_THAT(warnings(), Contains(HasSubstr(skipped(1))));
  EXPECT_THAT(warnings(), Contains(HasSubstr(skipped(3))));
  EXPECT_EQ(count_containing(warnings(), "unable to ping motor id"), 2u);
  // the servos that answered are named nowhere, and the permissive WARN is not logged as well
  EXPECT_THAT(fatals(), Not(Contains(HasSubstr("joint2"))));
  EXPECT_THAT(warnings(), Not(Contains(HasSubstr("continuing without"))));
}

TEST_F(WaveshareServosAbsentServo, configure_closes_the_port_before_returning_failure)
{
  fake_.set_absent(3, true);
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::ERROR);

  // Only the pty slave is left: a refused on_configure gets no on_error/on_cleanup, so it
  // must close the port itself.
  EXPECT_EQ(descriptors_on(fake_.port()), 1u);

  // and the port is free in fact, not merely uncounted: the same component takes it again the
  // moment the servo is back
  fake_.set_absent(3, false);
  EXPECT_EQ(configure(), hardware_interface::return_type::OK);
  EXPECT_EQ(descriptors_on(fake_.port()), 3u);
}

TEST_F(
  WaveshareServosAbsentServo,
  configure_leaves_no_descriptor_when_the_port_cannot_be_configured)
{
  // A plain file opens but is not a tty: ServoBus refuses it after ::open(), when an fd could
  // leak. A tcsetattr failure takes the same path but cannot be staged.
  const TempFile not_a_tty("waveshare_servos_not_a_tty_" + std::to_string(::getpid()));
  ASSERT_TRUE(not_a_tty.ok()) << "the harness could not create a plain file under the temp dir";
  ASSERT_TRUE(
    load(
      robot_description(
        kBenchName, four_servo_joints(all_state_interfaces()),
        "  <param name=\"port\">" + not_a_tty.path() + "</param>\n")));

  EXPECT_EQ(configure(), hardware_interface::return_type::ERROR);

  EXPECT_THAT(
    fatals(),
    Contains(HasSubstr("port '" + not_a_tty.path() + "' is not a serial device")));
  EXPECT_EQ(fatals().size(), 1u) << "one FATAL for one refusal";
  EXPECT_EQ(descriptors_on(not_a_tty.path()), 0u) << "the refusal left its own descriptor open";
  // and this case never went near the pty
  EXPECT_EQ(descriptors_on(fake_.port()), 1u);

  // A component refused at open must configure again on a working port without a cleanup.
  ASSERT_TRUE(load_four_servos());
  EXPECT_EQ(configure(), hardware_interface::return_type::OK);
  EXPECT_EQ(descriptors_on(fake_.port()), 3u);
}

TEST_F(WaveshareServosAbsentServo, configure_succeeds_and_warns_when_missing_is_allowed)
{
  fake_.set_absent(1, true);
  fake_.set_absent(3, true);
  ASSERT_TRUE(load_four_servos(kAllowMissing));

  EXPECT_EQ(configure(), hardware_interface::return_type::OK);

  EXPECT_THAT(fatals(), IsEmpty());
  EXPECT_EQ(
    count_containing(
      warnings(),
      "continuing without 2 of 4 servos because 'allow_missing_servos' is true: "
      "id 1 (joint 'joint1'), id 3 (joint 'joint3'); their joints mirror their commands into "
      "their states until the servos answer"), 1u);
  // one extra WARN and nothing else changes: the per-servo WARNs are still there and the port is
  // still held
  EXPECT_THAT(warnings(), Contains(HasSubstr(skipped(1))));
  EXPECT_THAT(warnings(), Contains(HasSubstr(skipped(3))));
  EXPECT_EQ(descriptors_on(fake_.port()), 3u);

  // and the servos that did answer are in the groups and driven
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  const size_t answered_before = record_count(2);
  step_cycle();
  EXPECT_EQ(record_count(2), answered_before + 1);
}

TEST_F(WaveshareServosAbsentServo, configure_does_not_write_the_mode_register_on_the_refusal_path)
{
  // Servo 3 is a wheel in mode 0: the gate runs before build_groups(), so a refused configure
  // never spends an EPROM write on register 33.
  fake_.set_byte(3, kRegMode, 0);
  fake_.set_absent(1, true);
  ASSERT_TRUE(load_four_servos());

  ASSERT_EQ(configure(), hardware_interface::return_type::ERROR);

  for (const uint8_t id : {2, 3, 4}) {
    SCOPED_TRACE("servo " + std::to_string(static_cast<int>(id)));
    const FakeServo servo = fake_.snapshot(id);
    EXPECT_EQ(servo.writes, 0);
    EXPECT_EQ(servo.mode_writes, 0);
    EXPECT_THAT(servo.sync_writes, IsEmpty());
  }
  EXPECT_EQ(static_cast<int>(fake_.snapshot(3).mem[kRegMode]), 0);

  // The control that proves the assertions above can fail: same description, same wheel in the
  // wrong mode, nothing missing -- and the EPROM write really does happen.
  fake_.set_absent(1, false);
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  EXPECT_EQ(fake_.snapshot(3).mode_writes, 1);
  EXPECT_EQ(static_cast<int>(fake_.snapshot(3).mem[kRegMode]), 1);
}

TEST_F(WaveshareServosAbsentServo, activate_re_adds_a_servo_that_answers_again)
{
  fake_.set_absent(3, true);
  ASSERT_TRUE(load_four_servos(kAllowMissing));
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  const FakeServo before = fake_.snapshot(3);
  ASSERT_EQ(before.pings, 0) << "an absent servo answers nothing, so it counts nothing";

  // it is back on the bus by the time the controllers are armed
  fake_.set_absent(3, false);
  fake_.set_position(3, 700);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  EXPECT_EQ(count_containing(infos(), "motor id '3' answered on activation; adding it back"), 1u);
  EXPECT_GE(fake_.snapshot(3).pings, 1);
  step_read();
  EXPECT_DOUBLE_EQ(state_of("joint3/position"), rad_of_tick(700, 0.0));

  // and it is polled every cycle from here on, like any servo that was there all along
  const FakeServo resumed = fake_.snapshot(3);
  for (int cycle = 0; cycle < 3; cycle++) {
    step_read();
  }
  EXPECT_EQ(fake_.snapshot(3).feedback_reads, resumed.feedback_reads + 3);
}

TEST_F(
  WaveshareServosAbsentServo,
  activate_succeeds_with_a_still_silent_servo_even_when_missing_is_not_allowed)
{
  // The gate is configure-time only: re-activating after a run-time drop must succeed even
  // with allow_missing_servos false.
  ASSERT_TRUE(load_four_servos());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  step_read();

  fake_.set_absent(3, true);
  for (int cycle = 0; cycle < kMaxReadFails; cycle++) {
    step_read();
  }
  ASSERT_THAT(errors(), Contains(HasSubstr("motor id '3' stopped answering")));

  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);
  EXPECT_EQ(activate(), hardware_interface::return_type::OK);

  EXPECT_THAT(fatals(), IsEmpty());
  EXPECT_THAT(infos(), Not(Contains(HasSubstr("answered on activation"))));
  // the joint is a phantom and the rest of the robot runs
  step_read();
  EXPECT_TRUE(std::isnan(state_of("joint3/status")));
  EXPECT_DOUBLE_EQ(state_of("joint1/position"), rad_of_tick(kMidTick));

  // A fresh configure is gated, so the two recovery routes differ.
  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);
  ASSERT_EQ(cleanup(), hardware_interface::return_type::OK);
  EXPECT_EQ(configure(), hardware_interface::return_type::ERROR);
  EXPECT_EQ(
    count_containing(
      fatals(),
      "1 of 4 servos did not answer: id 3 (joint 'joint3'); refusing to configure because "
      "'allow_missing_servos' is false"), 1u);
  // one FATAL in total for the refusal, as above: everything before it in this case succeeded
  EXPECT_EQ(fatals().size(), 1u);
}

TEST_F(WaveshareServosAbsentServo, read_mirrors_commands_for_a_dropped_servo)
{
  // The permissive WARN promises mirrored commands. Both ways to lose a servo take that
  // branch: servo 1 was never there, servo 2 is dropped mid-run.
  fake_.set_absent(1, true);
  ASSERT_TRUE(load_four_servos(kAllowMissing));
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  command_is("joint1/position", 0.3);
  step_read();
  EXPECT_DOUBLE_EQ(state_of("joint1/position"), 0.3);
  EXPECT_TRUE(std::isnan(state_of("joint1/status"))) << "no reading is not 'no fault'";
  EXPECT_DOUBLE_EQ(state_of("joint1/velocity"), 0.0);

  command_is("joint1/position", -0.4);
  step_read();
  EXPECT_DOUBLE_EQ(state_of("joint1/position"), -0.4);

  // and the same for a servo that answered at configure time and then went away
  fake_.set_absent(2, true);
  for (int cycle = 0; cycle < kMaxReadFails; cycle++) {
    step_read();
  }
  ASSERT_THAT(errors(), Contains(HasSubstr("motor id '2' stopped answering")));
  command_is("joint2/position", 0.15);
  step_read();
  EXPECT_DOUBLE_EQ(state_of("joint2/position"), 0.15);
  EXPECT_TRUE(std::isnan(state_of("joint2/status")));
}

TEST_F(WaveshareServosAbsentServo, ping_cost_is_bounded_by_ping_attempts_times_io_timeout)
{
  // A missing servo costs ping_attempts x io_timeout_ms at configure, not the stock 100 ms.
  // Counts first; the timing check is relative only (slow > 2 x fast), never absolute.
  fake_.set_absent(3, true);
  ASSERT_TRUE(load_four_servos_timed(kIoTimeoutMs, 1));
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  for (const uint8_t id : {1, 2, 4}) {
    EXPECT_EQ(fake_.snapshot(id).pings, 1) << "servo " << static_cast<int>(id);
  }

  std::array<FakeServo, 5> before{};
  for (const uint8_t id : {1, 2, 4}) {
    before[id] = fake_.snapshot(id);
  }
  ASSERT_TRUE(load_four_servos_timed(kIoTimeoutMs, kPingAttempts));
  const double fast_ms = configure_ms();
  for (const uint8_t id : {1, 2, 4}) {
    EXPECT_EQ(fake_.snapshot(id).pings, before[id].pings + 1) << static_cast<int>(id);
  }

  ASSERT_TRUE(load_four_servos_timed(kSlowIoTimeoutMs, kPingAttempts));
  const double slow_ms = configure_ms();

  if (fast_ms < kPingAttempts * kIoTimeoutMs * 0.5) {
    GTEST_SKIP() << kPingAttempts << " absent pings took " << fast_ms <<
      " ms and did not reach ping_attempts x io_timeout_ms; nothing here is measuring what it "
      "claims to";
  }
  EXPECT_GT(slow_ms, 2.0 * fast_ms) <<
    "io_timeout_ms " << kIoTimeoutMs << " cost " << fast_ms << " ms, io_timeout_ms " <<
    kSlowIoTimeoutMs << " cost " << slow_ms << " ms; the absent ping is not paying the "
    "configured io timeout";
}


TEST_F(WaveshareServosAbsentServo, the_absent_servo_is_never_named_in_a_sync_read)
{
  // An absent servo is never in the sync-read id list (a silent listed id costs io_timeout_ms
  // per cycle). Only sync_read_named can tell "not named" from "named and silent".
  fake_.set_absent(1, true);
  ASSERT_TRUE(load_four_servos(kAllowMissing));
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  command_is("joint1/position", 0.3);
  for (int cycle = 0; cycle < 10; cycle++) {
    step_read();
    EXPECT_THAT(fake_.last_sync_read_ids(), ElementsAreArray(std::vector<uint8_t>{2, 3, 4}));
  }

  EXPECT_EQ(fake_.snapshot(1).sync_read_named, 0);
  EXPECT_EQ(fake_.snapshot(1).requests, 0);
  // and the mirror branch is untouched by any of it
  EXPECT_DOUBLE_EQ(state_of("joint1/position"), 0.3);
  EXPECT_TRUE(std::isnan(state_of("joint1/status")));
  EXPECT_EQ(count_containing(infos(), "feedback for 3 servos travels in one sync read per cycle"),
    1u);
}

// ---------------------------------------------------------------------------------------------
// DropRecoverOverPty: a servo goes silent mid-run, on cue between two driver calls.

class DropRecoverOverPty : public PtyFixture
{
protected:
  static constexpr int kMaxReadFails = 8;
  static constexpr int kIoTimeoutMs = 30;

  void SetUp() override
  {
    fake_.add_servo(1, 0);
    fake_.add_servo(2, 0);
    fake_.add_servo(3, 1);
    fake_.add_servo(4, 1);
    fake_.set_position(1, kMidTick);
    fake_.set_position(2, kMidTick);
  }

  // A drop budget the silence cases cannot reach: 12 failed reads must give the
  // lost-revolutions WARN without a drop, because that WARN is time-driven.
  static constexpr int kSilenceMaxReadFails = 40;

  std::string drop_params(int max_read_fails = kMaxReadFails) const
  {
    return "  <param name=\"io_timeout_ms\">" + std::to_string(kIoTimeoutMs) + "</param>\n" +
           "  <param name=\"max_read_fails\">" + std::to_string(max_read_fails) + "</param>\n";
  }

  [[nodiscard]] bool load_four_servos(int max_read_fails = kMaxReadFails)
  {
    return load(
      robot_description(
        kBenchName, four_servo_joints(all_state_interfaces()),
        hardware_params(drop_params(max_read_fails))));
  }

  // configure, activate, and take one good reading from every servo
  void bring_up(int max_read_fails = kMaxReadFails)
  {
    ASSERT_TRUE(load_four_servos(max_read_fails));
    ASSERT_EQ(configure(), hardware_interface::return_type::OK);
    ASSERT_EQ(activate(), hardware_interface::return_type::OK);
    step_read();
  }

  // The gap WARN from the fixture's parameters: gap = max(period, io timeout) x (missed + 1),
  // possible = speed ceiling x gap. See docs/design.md, "Position unwrapper".
  static std::string lost_revolutions(int missed, int gaps)
  {
    const double cycle = std::max(0.01, kIoTimeoutMs / 1000.0);
    return lost_revolutions_over(cycle * (missed + 1), gaps);
  }

  // the same message for a gap that came from the clock rather than from a count of failed reads
  static std::string lost_revolutions_over(double gap, int gaps)
  {
    const double ceiling = kDefaultMaxSpeedCounts * 2.0 * M_PI / kEncoderSteps;
    std::array<char, 256> message{};
    std::snprintf(
      message.data(), message.size(),
      "joint 'joint3' went %.3f s without a reading; at %.2f rad/s it could have turned %.2f rad "
      "while it was silent, so its unwrapped position may be off by whole revolutions "
      "(%d such gaps)",
      gap, ceiling, ceiling * gap, gaps);
    return std::string(message.data());
  }

  // silence servo 3 and run it into the drop; the ERROR lands on the last of these reads
  void drop_servo_three()
  {
    fake_.set_silent_feedback(3, true);
    for (int cycle = 0; cycle < kMaxReadFails; cycle++) {
      step_read();
    }
  }

  static std::string read_failed_for(int id, int count)
  {
    return "read failed for motor id '" + std::to_string(id) + "' (" + std::to_string(count) +
           " in a row)";
  }

  static std::string dropped_after(int id, int attempts)
  {
    return "motor id '" + std::to_string(id) + "' stopped answering after " +
           std::to_string(attempts) +
           " attempts; dropping it from the read cycle until the hardware is re-activated";
  }
};

TEST_F(DropRecoverOverPty, a_silent_servo_raises_its_failure_count_and_warns_once)
{
  // 700 ticks: the absent branch and an untouched register both give 0, so only a nonzero
  // value proves the last good sample was held.
  fake_.set_position(3, 700);
  bring_up();
  const double last_good = state_of("joint3/position");
  ASSERT_DOUBLE_EQ(last_good, rad_of_tick(700, 0.0));
  fake_.set_silent_feedback(3, true);

  for (int cycle = 1; cycle <= 5; cycle++) {
    SCOPED_TRACE("silent cycle " + std::to_string(cycle));
    fake_.set_position(1, kMidTick + cycle);
    fake_.set_position(2, kMidTick + cycle);
    fake_.set_position(4, cycle);
    step_read();

    // the last good sample, not NaN and not the -1 tick a timed-out read returns
    EXPECT_DOUBLE_EQ(state_of("joint3/position"), last_good);
    EXPECT_DOUBLE_EQ(state_of("joint1/position"), rad_of_tick(kMidTick + cycle));
    EXPECT_DOUBLE_EQ(state_of("joint2/position"), rad_of_tick(kMidTick + cycle));
    EXPECT_DOUBLE_EQ(state_of("joint4/position"), rad_of_tick(cycle, 0.0));
  }

  EXPECT_EQ(count_containing(warnings(), "read failed for motor id '3'"), 1u);
  EXPECT_THAT(warnings(), Contains(HasSubstr(read_failed_for(3, 1))));
  EXPECT_THAT(errors(), IsEmpty());
}

TEST_F(DropRecoverOverPty, the_failure_warning_is_throttled_to_the_first_and_every_two_hundredth)
{
  // One joint, so nothing else can disturb the count; 2 ms makes 201 failures cost ~0.4 s,
  // and max_read_fails 1000 is out of reach.
  fake_.set_silent_feedback(1, true);
  ASSERT_TRUE(
    load(
      robot_description(
        kBenchName, {one_position_joint(all_state_interfaces())},
        hardware_params(
          "  <param name=\"io_timeout_ms\">2</param>\n"
          "  <param name=\"max_read_fails\">1000</param>\n"))));
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  for (int cycle = 0; cycle < 201; cycle++) {
    step_read();
  }

  std::vector<std::string> failures;
  for (const std::string & message : warnings()) {
    if (message.find("read failed for motor id '1'") != std::string::npos) {
      failures.push_back(message);
    }
  }
  ASSERT_EQ(failures.size(), 2u);
  EXPECT_THAT(failures[0], HasSubstr(read_failed_for(1, 1)));
  EXPECT_THAT(failures[1], HasSubstr(read_failed_for(1, 200)));
  EXPECT_THAT(errors(), Not(Contains(HasSubstr("stopped answering"))));
}

TEST_F(DropRecoverOverPty, a_silent_servo_is_dropped_at_max_read_fails_and_reported_once)
{
  bring_up();
  fake_.set_silent_feedback(3, true);

  for (int cycle = 1; cycle < kMaxReadFails; cycle++) {
    SCOPED_TRACE("failed read " + std::to_string(cycle));
    step_read();
    EXPECT_THAT(errors(), Not(Contains(HasSubstr("stopped answering"))));
  }
  // the drop test is `>=`, not `>`: the drop lands on the 8th failed read, not the 9th
  step_read();
  EXPECT_THAT(errors(), Contains(HasSubstr(dropped_after(3, kMaxReadFails))));
  EXPECT_EQ(count_containing(errors(), "stopped answering"), 1u);

  // it is reported once, not once per cycle
  for (int cycle = 0; cycle < 5; cycle++) {
    step_read();
  }
  EXPECT_EQ(count_containing(errors(), "stopped answering"), 1u);
  EXPECT_EQ(count_containing(warnings(), "read failed for motor id '3'"), 1u);

  // and the joint now takes read()'s absent branch
  EXPECT_TRUE(std::isnan(state_of("joint3/status")));
  for (const char * name : {"velocity", "effort", "current", "voltage", "temperature", "load",
      "torque"})
  {
    EXPECT_DOUBLE_EQ(state_of(std::string("joint3/") + name), 0.0) << name;
  }
}

TEST_F(DropRecoverOverPty, a_dropped_servo_gets_no_further_request_on_the_bus)
{
  bring_up();
  // A position servo and a wheel at once: after a drop, neither may get any request at all.
  fake_.set_silent_feedback(1, true);
  fake_.set_silent_feedback(3, true);
  for (int cycle = 0; cycle < kMaxReadFails; cycle++) {
    step_read();
  }
  ASSERT_THAT(errors(), Contains(HasSubstr(dropped_after(1, kMaxReadFails))));
  ASSERT_THAT(errors(), Contains(HasSubstr(dropped_after(3, kMaxReadFails))));

  const FakeServo dropped_position_before = fake_.snapshot(1);
  const FakeServo dropped_wheel_before = fake_.snapshot(3);
  const FakeServo healthy_before = fake_.snapshot(2);
  const FakeServo healthy_wheel_before = fake_.snapshot(4);
  for (int cycle = 0; cycle < 20; cycle++) {
    step_cycle();
  }
  const FakeServo dropped_position = fake_.snapshot(1);
  const FakeServo dropped_wheel = fake_.snapshot(3);

  // no ping, no addressed read, no addressed write: nothing at all
  EXPECT_EQ(dropped_position.requests, dropped_position_before.requests);
  EXPECT_EQ(dropped_position.feedback_reads, dropped_position_before.feedback_reads);
  EXPECT_EQ(dropped_position.pings, dropped_position_before.pings);

  // the wheel is polled no more either, and is never pinged
  EXPECT_EQ(dropped_wheel.feedback_reads, dropped_wheel_before.feedback_reads);
  EXPECT_EQ(dropped_wheel.reads, dropped_wheel_before.reads);
  EXPECT_EQ(dropped_wheel.pings, dropped_wheel_before.pings);
  // A dropped wheel gets no addressed write (no ACC edge reaches it); the broadcast sync
  // write still reaches it but is not a request.
  EXPECT_EQ(dropped_wheel.writes, dropped_wheel_before.writes);
  EXPECT_EQ(dropped_wheel.requests, dropped_wheel_before.requests);

  // meanwhile the healthy servos are polled every cycle
  EXPECT_EQ(fake_.snapshot(2).feedback_reads, healthy_before.feedback_reads + 20);
  EXPECT_EQ(fake_.snapshot(4).feedback_reads, healthy_wheel_before.feedback_reads + 20);
}

TEST_F(DropRecoverOverPty, a_dropped_servo_keeps_its_place_in_the_sync_write_groups)
{
  // Deliberate: a dropped servo stays in the sync-write groups (bytes of an unacked
  // broadcast). Do not "fix" it: that would change the packet contents.
  bring_up();
  drop_servo_three();
  ASSERT_THAT(errors(), Contains(HasSubstr(dropped_after(3, kMaxReadFails))));

  std::array<size_t, 5> before{};
  for (uint8_t id = 1; id <= 4; id++) {
    before[id] = record_count(id);
  }
  const FakeServo dropped_before = fake_.snapshot(3);
  for (int cycle = 0; cycle < 3; cycle++) {
    step_cycle();
  }

  for (uint8_t id = 1; id <= 4; id++) {
    EXPECT_EQ(record_count(id), before[id] + 3) << "servo " << static_cast<int>(id);
  }
  EXPECT_EQ(fake_.snapshot(3).sync_writes.back().first, kRegGoalSpeed);
  EXPECT_EQ(fake_.snapshot(3).sync_writes.back().second.size(), 2u);
  // No round trip: the sync write is an unacked broadcast to 0xfe, and a dropped servo is not
  // read, so it gets records but no request.
  EXPECT_EQ(fake_.snapshot(3).requests, dropped_before.requests);
  EXPECT_EQ(fake_.snapshot(3).feedback_reads, dropped_before.feedback_reads);
}

TEST_F(DropRecoverOverPty, a_dropped_servo_stops_costing_a_timeout_per_cycle)
{
  bring_up();
  fake_.set_silent_feedback(3, true);

  std::vector<double> failing_ms;
  for (int cycle = 0; cycle < kMaxReadFails; cycle++) {
    const auto started = std::chrono::steady_clock::now();
    step_read();
    failing_ms.push_back(
      std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count());
  }
  ASSERT_THAT(errors(), Contains(HasSubstr(dropped_after(3, kMaxReadFails))));

  const FakeServo dropped_before = fake_.snapshot(3);
  const FakeServo healthy_before = fake_.snapshot(2);
  std::vector<double> post_drop_ms;
  for (int cycle = 0; cycle < 20; cycle++) {
    const auto started = std::chrono::steady_clock::now();
    step_read();
    post_drop_ms.push_back(
      std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count());
  }

  // The counters are the primary assertion and they do not measure time at all: whatever the
  // machine is doing, a dropped servo is not polled and the healthy ones are.
  EXPECT_EQ(fake_.snapshot(3).feedback_reads, dropped_before.feedback_reads);
  EXPECT_EQ(fake_.snapshot(2).feedback_reads, healthy_before.feedback_reads + 20);
  // Same claim on the sync path: sync_read_named shows the dropped id is not named, which
  // would otherwise cost io_timeout_ms per cycle.
  EXPECT_EQ(fake_.snapshot(3).sync_read_named, dropped_before.sync_read_named);
  EXPECT_EQ(fake_.snapshot(2).sync_reads, healthy_before.sync_reads + 20);
  EXPECT_EQ(fake_.snapshot(4).sync_reads, fake_.snapshot(2).sync_reads);

  const auto median = [](std::vector<double> samples) {
      std::sort(samples.begin(), samples.end());
      return samples[samples.size() / 2];
    };
  const double median_failing = median(failing_ms);
  const double median_post_drop = median(post_drop_ms);
  if (median_failing < kIoTimeoutMs * 0.5) {
    GTEST_SKIP() << "the pre-drop median was " << median_failing <<
      " ms and did not reach io_timeout_ms; this machine is too loaded to time anything";
  }
  // Relative and ordinal only: no absolute wall-clock bound is asserted anywhere. A failing cycle
  // pays one io_timeout_ms for the silent servo; a post-drop cycle is three pty round trips.
  EXPECT_LT(median_post_drop * 4.0, median_failing) <<
    "post-drop median " << median_post_drop << " ms, failing median " << median_failing << " ms";
}

TEST_F(DropRecoverOverPty, deactivate_then_activate_re_pings_a_dropped_servo_and_adds_it_back)
{
  bring_up();
  drop_servo_three();
  ASSERT_THAT(errors(), Contains(HasSubstr(dropped_after(3, kMaxReadFails))));
  step_read();    // the cycle on which read() takes the absent branch for servo 3

  fake_.set_silent_feedback(3, false);
  const FakeServo before = fake_.snapshot(3);
  // every joint that stayed healthy, both position joints and the other wheel: id 4 shares servo
  // 3's mode and sync-write group, so a regression that re-pinged a whole group would show there
  std::array<FakeServo, 5> healthy_before{};
  for (const uint8_t id : {1, 2, 4}) {
    healthy_before[id] = fake_.snapshot(id);
  }
  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  EXPECT_EQ(count_containing(infos(), "motor id '3' answered on activation; adding it back"), 1u);
  const FakeServo after = fake_.snapshot(3);
  EXPECT_GE(after.pings, before.pings + 1);
  EXPECT_LE(after.pings, before.pings + 3) << "at most ping_attempts tries";
  // a regroup costs no EPROM write: set_mode only writes register 33 when it differs
  EXPECT_EQ(after.mode_writes, before.mode_writes);
  // only joints that are absent are re-pinged (the re-ping loop of on_activate)
  for (const uint8_t id : {1, 2, 4}) {
    EXPECT_EQ(fake_.snapshot(id).pings, healthy_before[id].pings) << static_cast<int>(id);
  }

  fake_.set_position(3, 700);
  step_read();
  EXPECT_DOUBLE_EQ(state_of("joint3/position"), rad_of_tick(700, 0.0));

  const FakeServo resumed = fake_.snapshot(3);
  for (int cycle = 0; cycle < 20; cycle++) {
    step_read();
  }
  EXPECT_EQ(fake_.snapshot(3).feedback_reads, resumed.feedback_reads + 20);
}

TEST_F(DropRecoverOverPty,
  a_servo_that_answers_pings_but_no_feedback_is_added_back_and_dropped_again)
{
  bring_up();
  drop_servo_three();
  ASSERT_EQ(count_containing(errors(), "stopped answering"), 1u);
  step_read();

  // it still answers pings, so the re-ping path takes it back; it still refuses the feedback read
  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  EXPECT_EQ(count_containing(infos(), "motor id '3' answered on activation; adding it back"), 1u);

  // read_fails_ was reset on the re-add, so the second drop takes the full eight cycles
  for (int cycle = 1; cycle < kMaxReadFails; cycle++) {
    SCOPED_TRACE("failed read after the re-add: " + std::to_string(cycle));
    step_read();
    EXPECT_EQ(count_containing(errors(), "stopped answering"), 1u);
  }
  step_read();
  EXPECT_EQ(count_containing(errors(), "stopped answering"), 2u);
  EXPECT_EQ(count_containing(errors(), dropped_after(3, kMaxReadFails)), 2u);
}

TEST_F(DropRecoverOverPty, a_servo_that_answers_again_before_max_read_fails_is_never_dropped)
{
  bring_up();
  fake_.set_silent_feedback(3, true);
  for (int cycle = 0; cycle < kMaxReadFails - 1; cycle++) {
    step_read();
  }
  EXPECT_THAT(errors(), Not(Contains(HasSubstr("stopped answering"))));

  // one good read resets the counter to 0
  fake_.set_silent_feedback(3, false);
  step_read();
  fake_.set_silent_feedback(3, true);
  for (int cycle = 0; cycle < kMaxReadFails - 1; cycle++) {
    step_read();
  }

  EXPECT_THAT(errors(), Not(Contains(HasSubstr("stopped answering"))));
  EXPECT_EQ(count_containing(warnings(), "read failed for motor id '3'"), 2u);
  EXPECT_EQ(count_containing(warnings(), read_failed_for(3, 1)), 2u);

  // and it is still polled: present_ never went false
  fake_.set_silent_feedback(3, false);
  fake_.set_position(3, 900);
  const FakeServo before = fake_.snapshot(3);
  step_read();
  EXPECT_EQ(fake_.snapshot(3).feedback_reads, before.feedback_reads + 1);
  EXPECT_DOUBLE_EQ(state_of("joint3/position"), rad_of_tick(900, 0.0));
}

// Wheel ACC edge 3 of 4: read recovery, which catches a power cycle during an activation.
TEST_F(DropRecoverOverPty, a_servo_that_missed_reads_and_came_back_gets_its_acceleration_rewritten)
{
  bring_up();
  const FakeServo before = fake_.snapshot(3);

  // Three misses, below the drop budget. A power cycle always shows as a missed read, so the
  // recovery edge rewrites the ACC (one 0.594 ms write).
  fake_.set_silent_feedback(3, true);
  for (int cycle = 0; cycle < 3; cycle++) {
    step_read();
  }
  fake_.set_byte(3, kRegAcc, 0);       // what the restart lost
  fake_.set_silent_feedback(3, false);
  step_read();

  EXPECT_EQ(fake_.snapshot(3).mem[kRegAcc], kDefaultAccCounts);
  EXPECT_EQ(fake_.snapshot(3).writes, before.writes + 1) << "one writeByte on the recovery edge";

  const FakeServo recovered = fake_.snapshot(3);
  for (int cycle = 0; cycle < 10; cycle++) {
    step_read();
  }
  EXPECT_EQ(fake_.snapshot(3).writes, recovered.writes) << "a healthy cycle pays nothing";
  EXPECT_THAT(errors(), Not(Contains(HasSubstr("stopped answering"))));
}

// Frozen refused-ACC WARN: an attempted, refused write warns once; a write never attempted
// does not warn.
TEST_F(DropRecoverOverPty, a_servo_that_refuses_the_acceleration_write_is_warned_about_once)
{
  bring_up();

  // Servo 3 leaves between deactivate and activate, so edge 2 writes into silence. SCS::Ack
  // returns 0 on failure, never -1, so write_acc tests != 0.
  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);
  fake_.set_absent(3, true);
  EXPECT_EQ(activate(), hardware_interface::return_type::OK);

  EXPECT_EQ(count_containing(warnings(), acc_refused_for(3)), 1u);

  // Never attempted: after the drop present_ is false, so build_groups() and
  // write_wheel_acceleration() skip it; re-activation adds no WARN and no byte.
  for (int cycle = 0; cycle < kMaxReadFails; cycle++) {
    step_read();
  }
  ASSERT_THAT(errors(), Contains(HasSubstr(dropped_after(3, kMaxReadFails))));
  const FakeServo dropped = fake_.snapshot(3);
  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  EXPECT_EQ(count_containing(warnings(), acc_refused_for(3)), 1u);
  EXPECT_EQ(fake_.snapshot(3).writes, dropped.writes);
}

TEST_F(DropRecoverOverPty, a_dropped_wheel_does_not_jump_back_to_its_activation_position)
{
  // The absent branch mirrors pos_cmds_, which for a wheel dates from activation; the drop
  // must refresh it or the wheel jumps back by its whole travel.
  bring_up();
  // spin it through two and a half revolutions, 1000 ticks a cycle, so the register wraps twice
  for (int cycle = 1; cycle <= 10; cycle++) {
    fake_.set_position(3, (cycle * 1000) % kEncoderSteps);
    step_read();
  }
  const double spun = state_of("joint3/position");
  ASSERT_DOUBLE_EQ(spun, rad_of_tick(10000, 0.0));
  ASSERT_GT(spun, 4.0 * M_PI) << "two whole revolutions on";

  drop_servo_three();
  ASSERT_THAT(errors(), Contains(HasSubstr(dropped_after(3, kMaxReadFails))));
  // the drop cycle itself still publishes the last good sample: the dropping branch sets
  // present_ and continues, so the absent branch runs from the next cycle
  ASSERT_DOUBLE_EQ(state_of("joint3/position"), spun);

  step_read();    // the first cycle down read()'s absent branch

  EXPECT_NEAR(state_of("joint3/position"), spun, 2.0 * M_PI / kEncoderSteps) <<
    "within one encoder step of the last unwrapped reading";
  EXPECT_GT(state_of("joint3/position"), 2.0 * M_PI) <<
    "not the activation position, and not a value inside one turn";
  // and it stays there: the mirror is continuous cycle after cycle, not only on the first one
  for (int cycle = 0; cycle < 3; cycle++) {
    step_read();
    EXPECT_NEAR(state_of("joint3/position"), spun, 2.0 * M_PI / kEncoderSteps);
  }
}

TEST_F(DropRecoverOverPty, a_component_cycle_keeps_a_present_wheels_multi_turn_count)
{
  // A deactivate/activate cycle keeps a present wheel's multi-turn count: a reset teleported
  // diff_drive odometry. See docs/configuration.md, "Multi-turn wheel position".
  bring_up();
  for (int cycle = 1; cycle <= 10; cycle++) {
    fake_.set_position(3, (cycle * 1000) % kEncoderSteps);
    fake_.set_position(4, (cycle * 1000) % kEncoderSteps);
    step_read();
  }
  const double spun = state_of("joint3/position");
  ASSERT_DOUBLE_EQ(spun, rad_of_tick(10000, 0.0));
  ASSERT_GT(spun, 4.0 * M_PI) << "two whole revolutions on";

  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  step_read();

  EXPECT_DOUBLE_EQ(state_of("joint3/position"), spun) <<
    "the count survives the cycle instead of dropping back by whole revolutions";
  EXPECT_DOUBLE_EQ(state_of("joint4/position"), spun);
  EXPECT_GT(state_of("joint3/position"), 4.0 * M_PI) << "not the raw wrapped register value";

  // and it keeps counting from there: the wheel turns another 1000 ticks after the cycle
  fake_.set_position(3, 11000 % kEncoderSteps);
  fake_.set_position(4, 11000 % kEncoderSteps);
  step_read();
  EXPECT_DOUBLE_EQ(state_of("joint3/position"), rad_of_tick(11000, 0.0));
  EXPECT_DOUBLE_EQ(state_of("joint4/position"), rad_of_tick(11000, 0.0));

  // the carried bridge is not reported as lost revolutions: read_fails_ is zeroed at the top of
  // on_activate, so the seeding sample is charged no missed cycles
  EXPECT_THAT(warnings(), Not(Contains(HasSubstr("may be off by whole revolutions"))));
}

TEST_F(DropRecoverOverPty, a_rejoining_wheel_starts_a_new_unwrapped_count)
{
  // A dropped wheel starts a new count (the drop reset it); joint4 stayed present and keeps
  // its count, as in a_component_cycle_keeps_a_present_wheels_multi_turn_count.
  bring_up();
  for (int cycle = 1; cycle <= 10; cycle++) {
    fake_.set_position(3, (cycle * 1000) % kEncoderSteps);
    fake_.set_position(4, (cycle * 1000) % kEncoderSteps);
    step_read();
  }
  ASSERT_DOUBLE_EQ(state_of("joint3/position"), rad_of_tick(10000, 0.0));
  ASSERT_DOUBLE_EQ(state_of("joint4/position"), rad_of_tick(10000, 0.0));
  drop_servo_three();
  ASSERT_THAT(errors(), Contains(HasSubstr(dropped_after(3, kMaxReadFails))));

  fake_.set_silent_feedback(3, false);
  fake_.set_position(3, 700);
  fake_.set_position(4, 900);
  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_EQ(count_containing(infos(), "motor id '3' answered on activation; adding it back"), 1u);
  step_read();

  EXPECT_DOUBLE_EQ(state_of("joint3/position"), rad_of_tick(700, 0.0));
  EXPECT_LT(state_of("joint3/position"), 2.0 * M_PI) << "a new count, inside one turn";
  EXPECT_GE(state_of("joint3/position"), 0.0);
  // joint4 bridges 10000 -> 900 the short way: 900 - (10000 % 4096) = -908, so 9092 ticks.
  // The drop starts a new count, not the activation.
  EXPECT_DOUBLE_EQ(state_of("joint4/position"), rad_of_tick(9092, 0.0)) <<
    "a servo that stayed present carries its count across the activation";
  EXPECT_GT(state_of("joint4/position"), 4.0 * M_PI) << "two whole revolutions still on it";
  // the re-seed is not a gap: PositionUnwrapper::bridged() is false on the seeding sample, so
  // nothing about lost revolutions is reported for it
  EXPECT_THAT(warnings(), Not(Contains(HasSubstr("may be off by whole revolutions"))));
}

TEST_F(DropRecoverOverPty, a_long_silence_that_does_not_reach_the_drop_warns_about_lost_revolutions)
{
  // Gap = (12 silent + 1) x 30 ms = 0.39 s; at 9.20 rad/s that is 3.59 rad > pi, so the WARN
  // fires. max_read_fails 40: no drop, because the WARN is time-driven.
  bring_up(kSilenceMaxReadFails);
  step_read();    // the count is bridged from here on, so a gap has something to be lost from

  fake_.set_silent_feedback(3, true);
  for (int cycle = 1; cycle <= 12; cycle++) {
    SCOPED_TRACE("silent cycle " + std::to_string(cycle));
    step_read();
    // note_unwrap_gap runs on the success path, so a gap can only be reported once it has ENDED
    EXPECT_THAT(warnings(), Not(Contains(HasSubstr("without a reading"))));
  }
  fake_.set_silent_feedback(3, false);
  step_read();

  EXPECT_EQ(count_containing(warnings(), "without a reading"), 1u);
  EXPECT_THAT(warnings(), Contains(HasSubstr(lost_revolutions(12, 1))));
  EXPECT_THAT(warnings(), Contains(HasSubstr("went 0.390 s without a reading")));
  EXPECT_THAT(warnings(), Contains(HasSubstr("at 9.20 rad/s it could have turned 3.59 rad")));
  EXPECT_THAT(errors(), IsEmpty()) << "12 failed reads is well inside max_read_fails 40";
}

TEST_F(DropRecoverOverPty,
  a_descheduled_cycle_with_no_failed_read_still_warns_about_lost_revolutions)
{
  // A 0.5 s deschedule with no failed read must still warn (4.60 rad possible). Fails if the
  // check needs missed > 0 or read() ignores its period.
  bring_up(kSilenceMaxReadFails);

  step_read(0.5);

  EXPECT_THAT(warnings(), Contains(HasSubstr(lost_revolutions_over(0.5, 1))));
  EXPECT_THAT(warnings(), Contains(HasSubstr("joint 'joint3' went 0.500 s without a reading")));
  EXPECT_EQ(count_containing(warnings(), "joint 'joint3' went"), 1u);
  // every unwrapping joint is judged on its own elapsed time, so the other wheel warns too and the
  // two position joints, which do not unwrap, never do
  EXPECT_THAT(warnings(), Contains(HasSubstr("joint 'joint4' went 0.500 s without a reading")));
  EXPECT_THAT(warnings(), Not(Contains(HasSubstr("joint 'joint1' went"))));
  EXPECT_THAT(warnings(), Not(Contains(HasSubstr("joint 'joint2' went"))));
  EXPECT_THAT(errors(), IsEmpty()) << "nothing failed to answer: this gap is the loop's own";
}

TEST_F(DropRecoverOverPty, a_short_silence_does_not_warn_about_lost_revolutions)
{
  // Negative case: 3 silent cycles = 0.12 s = 1.10 rad at the ceiling, under pi, so no WARN.
  bring_up(kSilenceMaxReadFails);
  step_read();

  fake_.set_silent_feedback(3, true);
  for (int cycle = 0; cycle < 3; cycle++) {
    step_read();
  }
  fake_.set_silent_feedback(3, false);
  step_read();

  EXPECT_THAT(warnings(), Not(Contains(HasSubstr("without a reading"))));
  EXPECT_THAT(warnings(), Not(Contains(HasSubstr("may be off by whole revolutions"))));
  EXPECT_THAT(errors(), IsEmpty());
  // and the ordinary healthy cycles of this case warned about nothing either: at one cycle of
  // silence a wheel at its ceiling covers 0.28 rad, so the predicate is nowhere near firing
  EXPECT_EQ(count_containing(warnings(), "without a reading"), 0u);
}

TEST_F(DropRecoverOverPty, a_silence_that_ended_at_the_activation_does_not_warn_after_it)
{
  // A silence that ended at activation must not be reported after it: on_activate clears
  // read_fails_, so the first read is charged no missed cycles.
  bring_up(kSilenceMaxReadFails);
  step_read();

  fake_.set_silent_feedback(3, true);
  for (int cycle = 1; cycle <= 12; cycle++) {
    step_read();
  }
  ASSERT_THAT(errors(), IsEmpty()) << "12 failed reads is well inside max_read_fails 40";
  ASSERT_THAT(warnings(), Not(Contains(HasSubstr("without a reading")))) <<
    "the gap has not ended yet";

  // it answers again, but the next thing that happens is a lifecycle transition, not a read
  fake_.set_silent_feedback(3, false);
  fake_.set_position(3, 700);
  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  step_read();

  EXPECT_THAT(warnings(), Not(Contains(HasSubstr("without a reading"))));
  EXPECT_THAT(warnings(), Not(Contains(HasSubstr("may be off by whole revolutions"))));
  // 700 either way: a count bridged from tick 0 and a fresh seed give the same value
  EXPECT_DOUBLE_EQ(state_of("joint3/position"), rad_of_tick(700, 0.0));
  // the next cycles are ordinary healthy ones too, so nothing arrives late either
  for (int cycle = 0; cycle < 3; cycle++) {
    step_read();
  }
  EXPECT_EQ(count_containing(warnings(), "without a reading"), 0u);
}

TEST_F(DropRecoverOverPty, a_healthy_cycle_is_not_charged_a_failed_reads_timeout)
{
  // A successful read is charged the period, not the timeout. At io_timeout_ms 400 (> 342 ms,
  // half a turn at the ceiling) the wrong rule would warn on every healthy cycle.
  ASSERT_TRUE(
    load(
      robot_description(
        kBenchName, four_servo_joints(all_state_interfaces()),
        hardware_params(
          "  <param name=\"io_timeout_ms\">400</param>\n"
          "  <param name=\"max_read_fails\">40</param>\n"))));
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  for (int cycle = 1; cycle <= 5; cycle++) {
    SCOPED_TRACE("healthy cycle " + std::to_string(cycle));
    fake_.set_position(3, cycle * 10);
    step_read();
    EXPECT_THAT(warnings(), Not(Contains(HasSubstr("without a reading"))));
  }
  EXPECT_THAT(errors(), IsEmpty());

  // and the timeout is still charged to the reads that really failed: one silent cycle at 400 ms
  // is a gap worth reporting, and this one is reported once it has ended
  fake_.set_silent_feedback(3, true);
  step_read();
  fake_.set_silent_feedback(3, false);
  step_read();
  EXPECT_EQ(count_containing(warnings(), "joint 'joint3' went"), 1u);
  EXPECT_THAT(warnings(), Contains(HasSubstr("joint 'joint3' went 0.800 s without a reading")));
}

TEST_F(DropRecoverOverPty, the_unwrap_accumulator_is_not_advanced_by_a_read_that_did_not_refresh)
{
  // A failed read never reaches unwrap_ticks(): the next delta is from the last real tick,
  // not from the stale value republished during the silence.
  fake_.set_position(3, 1000);
  bring_up(kSilenceMaxReadFails);
  step_read();
  const double last_good = state_of("joint3/position");
  ASSERT_DOUBLE_EQ(last_good, rad_of_tick(1000, 0.0));

  // the servo really moves during the silence, by less than the half revolution that would alias
  fake_.set_silent_feedback(3, true);
  fake_.set_position(3, 1300);
  for (int cycle = 1; cycle <= 12; cycle++) {
    SCOPED_TRACE("silent cycle " + std::to_string(cycle));
    step_read();
    // bit-identical, not "close": the interface is frozen, and freezing it must not feed the
    // accumulator a delta of zero twelve times over
    EXPECT_DOUBLE_EQ(state_of("joint3/position"), last_good);
  }
  fake_.set_silent_feedback(3, false);
  step_read();

  EXPECT_NEAR(
    state_of("joint3/position"), last_good + 300 * 2.0 * M_PI / kEncoderSteps, 1e-12);
  EXPECT_DOUBLE_EQ(state_of("joint3/position"), rad_of_tick(1300, 0.0));
  // the gap is long enough to be reported, and the position result is asserted independently of it
  EXPECT_EQ(count_containing(warnings(), "without a reading"), 1u);
  EXPECT_THAT(warnings(), Contains(HasSubstr(lost_revolutions(12, 1))));
}

// ---------------------------------------------------------------------------------------------
// Sync read with drops: the burst's id list stays current; a bad reply costs only its joint.

TEST_F(DropRecoverOverPty, a_dropped_servo_leaves_the_sync_read_id_list)
{
  // A dropped id must leave the sync-read list: a silent listed id costs one io_timeout_ms
  // per cycle. See docs/bus-timing.md, "Cost of a silent servo".
  bring_up();
  drop_servo_three();
  ASSERT_THAT(errors(), Contains(HasSubstr(dropped_after(3, kMaxReadFails))));

  const FakeServo dropped_before = fake_.snapshot(3);
  std::array<FakeServo, 5> healthy_before{};
  for (const uint8_t id : {1, 2, 4}) {
    healthy_before[id] = fake_.snapshot(id);
  }
  for (int cycle = 0; cycle < 10; cycle++) {
    step_read();
    EXPECT_THAT(fake_.last_sync_read_ids(), ElementsAreArray(std::vector<uint8_t>{1, 2, 4}));
  }

  // sync_read_named, not sync_reads: it is bumped BEFORE the absent check, so it is the only
  // counter that can tell "the driver did not name it" from "it was named and stayed silent".
  EXPECT_EQ(fake_.snapshot(3).sync_read_named, dropped_before.sync_read_named);
  EXPECT_EQ(fake_.snapshot(3).sync_reads, dropped_before.sync_reads);
  for (const uint8_t id : {1, 2, 4}) {
    SCOPED_TRACE("servo " + std::to_string(static_cast<int>(id)));
    EXPECT_EQ(fake_.snapshot(id).sync_reads, healthy_before[id].sync_reads + 10);
  }
}

TEST_F(DropRecoverOverPty, the_sync_read_id_list_is_rebuilt_the_moment_the_drop_fires)
{
  // The id list is rebuilt at the end of the dropping read() (later joints still index
  // r_slot_), so the very next cycle asks for three ids.
  bring_up();
  fake_.set_silent_feedback(3, true);
  for (int cycle = 0; cycle < kMaxReadFails - 1; cycle++) {
    step_read();
  }
  ASSERT_THAT(errors(), Not(Contains(HasSubstr(dropped_after(3, kMaxReadFails)))));
  ASSERT_THAT(fake_.last_sync_read_ids(), ElementsAreArray(std::vector<uint8_t>{1, 2, 3, 4}));

  step_read();                         // the cycle the ERROR lands on
  ASSERT_THAT(errors(), Contains(HasSubstr(dropped_after(3, kMaxReadFails))));

  step_read();                         // the very next one already asks for three ids
  EXPECT_THAT(fake_.last_sync_read_ids(), ElementsAreArray(std::vector<uint8_t>{1, 2, 4}));
}

TEST_F(DropRecoverOverPty, a_servo_that_stops_answering_the_sync_read_freezes_only_its_own_joint)
{
  // A short burst fails only the missing servo's slot (the bench decoded the others
  // 1200/1200); treating it as atomic would freeze all four joints.
  fake_.set_position(2, kMidTick);
  bring_up();
  const double frozen = state_of("joint2/position");
  ASSERT_DOUBLE_EQ(frozen, rad_of_tick(kMidTick));

  fake_.set_silent_feedback(2, true);
  fake_.set_position(1, kMidTick + 40);
  fake_.set_position(2, kMidTick + 40);
  fake_.set_position(3, 640);
  fake_.set_position(4, 641);
  step_read();

  EXPECT_DOUBLE_EQ(state_of("joint2/position"), frozen) << "its last good sample, not 0 and not -1";
  EXPECT_DOUBLE_EQ(state_of("joint1/position"), rad_of_tick(kMidTick + 40));
  EXPECT_DOUBLE_EQ(state_of("joint3/position"), rad_of_tick(640, 0.0));
  EXPECT_DOUBLE_EQ(state_of("joint4/position"), rad_of_tick(641, 0.0));
  // one joint's failure count moved, and only one: the others are observable through the WARN
  // their first failure would print, and through the drop that would follow it
  EXPECT_EQ(count_containing(warnings(), read_failed_for(2, 1)), 1u);
  EXPECT_EQ(count_containing(warnings(), "read failed for motor id"), 1u);
  EXPECT_THAT(errors(), IsEmpty());
}

TEST_F(DropRecoverOverPty, a_partial_reply_is_not_decoded_as_a_short_frame)
{
  // 10 bytes cut from an 84-byte burst leave servo 4 with 11 of its 21 bytes; the walker
  // needs a whole frame, so nothing is decoded.
  fake_.set_position(4, 900);
  bring_up();
  const double frozen = state_of("joint4/position");
  ASSERT_DOUBLE_EQ(frozen, rad_of_tick(900, 0.0));

  fake_.set_sync_read_truncate_bytes(10);
  fake_.set_position(1, kMidTick + 3);
  fake_.set_position(4, 1400);
  step_read();

  EXPECT_DOUBLE_EQ(state_of("joint4/position"), frozen);
  EXPECT_DOUBLE_EQ(state_of("joint1/position"), rad_of_tick(kMidTick + 3));
  EXPECT_EQ(count_containing(warnings(), read_failed_for(4, 1)), 1u);

  // and it recovers continuous with the last accepted tick: a failed cycle never reaches
  // unwrap_ticks(), so the 500-tick move is one delta
  fake_.set_sync_read_truncate_bytes(0);
  step_read();
  EXPECT_DOUBLE_EQ(state_of("joint4/position"), rad_of_tick(1400, 0.0));
  EXPECT_EQ(count_containing(warnings(), "read failed for motor id"), 1u);
}

TEST_F(DropRecoverOverPty, the_unwrapper_advances_only_for_servos_that_answered_the_burst)
{
  // Silent across 100 -> 2000 -> 3900: only arrived samples count, so the step reads as -296
  // ticks (result -196), not two +1900 steps (3900).
  fake_.set_position(3, 100);
  bring_up(kSilenceMaxReadFails);
  ASSERT_DOUBLE_EQ(state_of("joint3/position"), rad_of_tick(100, 0.0));

  fake_.set_silent_feedback(3, true);
  fake_.set_position(3, 2000);
  step_read();
  fake_.set_position(3, 3900);
  step_read();
  ASSERT_DOUBLE_EQ(state_of("joint3/position"), rad_of_tick(100, 0.0)) << "frozen while silent";

  fake_.set_silent_feedback(3, false);
  step_read();

  EXPECT_DOUBLE_EQ(state_of("joint3/position"), rad_of_tick(-196, 0.0));
}

TEST_F(DropRecoverOverPty, the_bus_totals_line_carries_the_failures_and_the_drop)
{
  // The per-id tail is part of the format: hil_gates.py parses it (BUS_TOTALS_TAIL) to name
  // the failing ids. See docs/bus-timing.md, "Bus totals line".
  bring_up();
  drop_servo_three();
  ASSERT_THAT(errors(), Contains(HasSubstr(dropped_after(3, kMaxReadFails))));
  for (int cycle = 0; cycle < 4; cycle++) {
    step_read();
  }

  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);

  // 13 cycles: 1 + 8 to the drop + 4. `failed` counts bursts, the tail counts per servo;
  // they differ only when one burst loses two frames.
  EXPECT_EQ(
    count_containing(
      infos(),
      "bus totals: transactions 13, failed 8 (615384.6 per million), worst consecutive 8, "
      "dropped 1 [id1 0, id2 0, id3 8, id4 0]"), 1u);
}

TEST_F(DropRecoverOverPty, re_activating_hands_a_still_silent_servo_its_whole_drop_budget_again)
{
  // on_activate must reset read_fails_: re-activation gives a still-silent servo its full
  // drop budget, as the drop ERROR promises.
  bring_up();
  fake_.set_silent_feedback(3, true);
  for (int cycle = 0; cycle < kMaxReadFails - 1; cycle++) {
    step_read();
  }
  ASSERT_THAT(errors(), IsEmpty()) << "one cycle short of the budget";

  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  for (int cycle = 0; cycle < kMaxReadFails - 1; cycle++) {
    step_read();
  }
  EXPECT_THAT(errors(), IsEmpty()) << "the budget started again with the activation";
  // Twice, once per window: the consecutive counter the throttle keys on restarted too, so the
  // second window's first failure is its own first and not the previous window's eighth.
  EXPECT_EQ(count_containing(warnings(), read_failed_for(3, 1)), 2u);

  step_read();
  EXPECT_THAT(errors(), Contains(HasSubstr(dropped_after(3, kMaxReadFails))));
}

TEST_F(DropRecoverOverPty, the_worst_consecutive_run_is_the_worst_run_of_ITS_OWN_activation)
{
  // `worst consecutive` is per activation: a servo silent across the transition starts a new
  // run, so the second window reports 1, not 4.
  bring_up();
  fake_.set_silent_feedback(3, true);
  for (int cycle = 0; cycle < 3; cycle++) {
    step_read();
  }
  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);
  EXPECT_EQ(
    count_containing(
      infos(),
      "bus totals: transactions 4, failed 3 (750000.0 per million), worst consecutive 3, "
      "dropped 0 [id1 0, id2 0, id3 3, id4 0]"), 1u);

  // Servo 3 is still silent, so its seeding feedback() fails too and the activation cannot clear
  // the count the way a successful seed does.
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  step_read();
  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);

  EXPECT_EQ(
    count_containing(
      infos(),
      "bus totals: transactions 1, failed 1 (1000000.0 per million), worst consecutive 1, "
      "dropped 0 [id1 0, id2 0, id3 1, id4 0]"), 1u);
}


// ---------------------------------------------------------------------------------------------
// StatusOverPty: synthetic status bytes; no bit meaning on the ST3025 is asserted.

class StatusOverPty : public PtyFixture
{
protected:
  static constexpr int kMaxReadFails = 8;

  void SetUp() override
  {
    fake_.add_servo(1, 0);
    fake_.add_servo(2, 0);
    fake_.add_servo(3, 1);
    fake_.add_servo(4, 1);
    fake_.set_position(1, kMidTick);
    fake_.set_position(2, kMidTick);
  }

  [[nodiscard]] bool load_four_servos(const std::string & extra = "")
  {
    return load(
      robot_description(
        kBenchName, four_servo_joints(all_state_interfaces()),
        hardware_params(
          "  <param name=\"io_timeout_ms\">30</param>\n"
          "  <param name=\"max_read_fails\">" + std::to_string(kMaxReadFails) + "</param>\n" +
          extra)));
  }

  void bring_up(const std::string & extra = "")
  {
    ASSERT_TRUE(load_four_servos(extra));
    ASSERT_EQ(configure(), hardware_interface::return_type::OK);
    ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  }
};

TEST_F(StatusOverPty, the_status_interface_carries_the_byte_the_servo_sent)
{
  bring_up();
  for (const uint8_t byte : {0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x24, 0x2a,
      0xff})
  {
    SCOPED_TRACE("status byte " + std::to_string(static_cast<int>(byte)));
    fake_.set_status(3, byte);
    step_read();
    // every value 0..255 is exact in a double, so this is an equality test, not a tolerance
    EXPECT_DOUBLE_EQ(state_of("joint3/status"), static_cast<double>(byte));
  }
}

TEST_F(StatusOverPty, every_status_bit_reaches_the_status_interface_unmodified)
{
  bring_up();
  for (int value = 0; value <= 255; value++) {
    SCOPED_TRACE("status byte " + std::to_string(value));
    fake_.set_status(3, static_cast<uint8_t>(value));
    step_read();
    EXPECT_DOUBLE_EQ(state_of("joint3/status"), static_cast<double>(value));
  }
}

TEST_F(StatusOverPty, a_set_status_byte_is_warned_about_with_its_decoded_names)
{
  bring_up();
  fake_.set_status(3, 0x24);
  step_read();

  EXPECT_EQ(count_containing(warnings(), "reports status"), 1u);
  EXPECT_THAT(
    warnings(), Contains(HasSubstr("motor id '3' reports status 0x24 (overheat, overload)")));
}

TEST_F(StatusOverPty, an_uncited_bit_is_reported_as_unverified_on_the_bus_too)
{
  // a bit with no known meaning shows as "meaning unverified" in the log too
  bring_up();
  fake_.set_status(3, 0x50);
  step_read();

  EXPECT_THAT(
    warnings(),
    Contains(
      HasSubstr(
        "motor id '3' reports status 0x50 (bit4 (meaning unverified), "
        "bit6 (meaning unverified))")));
}

TEST_F(StatusOverPty,
  the_status_byte_comes_from_the_feedback_reply_and_not_from_the_activation_ping)
{
  // Ping() writes the responder id into SCS::Error (src/SCS.cpp:261-262), so a driver that latched
  // Error outside feedback() would publish the servo id as a fault mask.
  bring_up();
  step_read();
  for (int joint = 1; joint <= 4; joint++) {
    const std::string key = "joint" + std::to_string(joint) + "/status";
    EXPECT_DOUBLE_EQ(state_of(key), 0.0) << key << " carries the servo id, not the status byte";
  }

  fake_.set_status(3, 0x20);
  step_read();
  EXPECT_DOUBLE_EQ(state_of("joint3/status"), 32.0);
}

TEST_F(StatusOverPty, a_changed_status_byte_is_warned_about_once)
{
  bring_up();
  fake_.set_status(3, 0x20);
  for (int cycle = 0; cycle < 5; cycle++) {
    step_read();
  }
  EXPECT_EQ(count_containing(warnings(), "reports status"), 1u);
  EXPECT_THAT(warnings(), Contains(HasSubstr("motor id '3' reports status 0x20 (overload)")));

  fake_.set_status(3, 0x24);
  step_read();
  EXPECT_EQ(count_containing(warnings(), "reports status"), 2u);

  for (int cycle = 0; cycle < 5; cycle++) {
    step_read();
  }
  EXPECT_EQ(count_containing(warnings(), "reports status"), 2u);
}

TEST_F(StatusOverPty, a_zero_status_byte_is_never_warned_about)
{
  bring_up();
  const FakeServo before = fake_.snapshot(3);
  for (int cycle = 0; cycle < 10; cycle++) {
    step_read();
  }

  EXPECT_THAT(warnings(), Not(Contains(HasSubstr("reports status"))));
  EXPECT_THAT(infos(), Not(Contains(HasSubstr("cleared its fault"))));
  EXPECT_EQ(fake_.snapshot(3).torque_enable_writes, before.torque_enable_writes);
}

TEST_F(StatusOverPty, a_cleared_status_byte_re_enables_torque_exactly_once)
{
  bring_up();
  fake_.set_status(3, 0x20);
  for (int cycle = 0; cycle < 3; cycle++) {
    step_read();
  }
  const FakeServo latched = fake_.snapshot(3);

  fake_.set_status(3, 0x00);
  for (int cycle = 0; cycle < 5; cycle++) {
    step_read();
  }

  EXPECT_EQ(count_containing(infos(), "motor id '3' cleared its fault; re-enabling torque"), 1u);
  EXPECT_EQ(fake_.snapshot(3).torque_enable_writes, latched.torque_enable_writes + 1);
}

// Wheel ACC edge 4 of 4: fault clear. A protection trip may clear SRAM with no missed read,
// so edge 3 does not cover it.
TEST_F(StatusOverPty, a_wheel_that_clears_its_fault_gets_its_acc_back)
{
  bring_up();
  fake_.set_status(3, 0x20);
  for (int cycle = 0; cycle < 3; cycle++) {
    step_read();
  }
  const FakeServo latched = fake_.snapshot(3);

  // Every read answers, so edge 3 never fires: this case sees edge 4 alone.
  fake_.set_byte(3, kRegAcc, 0);
  fake_.set_status(3, 0x00);
  step_read();

  EXPECT_EQ(fake_.snapshot(3).mem[kRegAcc], kDefaultAccCounts);
  EXPECT_EQ(fake_.snapshot(3).writes, latched.writes + 2) << "the torque enable and the ACC write";
  EXPECT_EQ(fake_.snapshot(3).torque_enable_writes, latched.torque_enable_writes + 1);

  const FakeServo cleared = fake_.snapshot(3);
  for (int cycle = 0; cycle < 5; cycle++) {
    step_read();
  }
  EXPECT_EQ(fake_.snapshot(3).writes, cleared.writes) << "an edge, not a state";
}

TEST_F(StatusOverPty, status_is_nan_for_a_servo_that_never_answered)
{
  // nothing on the bus for this joint at all: no reading is not "no fault"
  fake_.set_absent(3, true);
  bring_up("  <param name=\"allow_missing_servos\">true</param>\n");
  step_read();

  EXPECT_TRUE(std::isnan(state_of("joint3/status")));
  for (const char * name : {"position", "velocity", "effort", "current", "voltage", "temperature",
      "load", "torque"})
  {
    const std::string key = std::string("joint3/") + name;
    EXPECT_TRUE(std::isfinite(state_of(key))) << key;
  }
}

TEST_F(StatusOverPty, status_returns_to_nan_when_a_servo_is_dropped)
{
  bring_up();
  fake_.set_status(3, 0x20);
  step_read();
  ASSERT_DOUBLE_EQ(state_of("joint3/status"), 32.0);

  fake_.set_silent_feedback(3, true);
  for (int cycle = 0; cycle < kMaxReadFails; cycle++) {
    step_read();
  }
  ASSERT_THAT(errors(), Contains(HasSubstr("stopped answering")));
  // the drop itself only stops the polling; the next cycle is the one that takes the absent branch
  step_read();

  EXPECT_TRUE(std::isnan(state_of("joint3/status")));
}

TEST_F(StatusOverPty, a_fault_latched_before_a_drop_does_not_fire_a_clear_edge_after_reactivation)
{
  bring_up();
  fake_.set_status(3, 0x20);
  step_read();
  ASSERT_EQ(count_containing(warnings(), "reports status"), 1u);

  fake_.set_silent_feedback(3, true);
  for (int cycle = 0; cycle < kMaxReadFails; cycle++) {
    step_read();
  }
  ASSERT_THAT(errors(), Contains(HasSubstr("stopped answering")));
  step_read();   // the absent branch clears last_error_ and status_bytes_

  fake_.set_silent_feedback(3, false);
  fake_.set_status(3, 0x00);
  ASSERT_EQ(deactivate(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  for (int cycle = 0; cycle < 3; cycle++) {
    step_read();
  }

  EXPECT_THAT(infos(), Not(Contains(HasSubstr("cleared its fault"))));
  EXPECT_DOUBLE_EQ(state_of("joint3/status"), 0.0);
}


TEST_F(StatusOverPty, each_joints_status_byte_comes_from_its_own_frame)
{
  // A sync read's status byte is byte 4 of each servo's own frame, never SCS::Error
  // (rewritten per frame, unchanged for a silent id). See docs/design.md, "Feedback block".
  bring_up();
  const std::array<uint8_t, 4> bytes{0x01, 0x02, 0x20, 0x80};
  for (uint8_t id = 1; id <= 4; id++) {
    fake_.set_status(id, bytes[id - 1]);
  }

  step_read();

  ASSERT_GT(fake_.sync_read_requests(), 0u) << "this case did not run on the sync path";
  for (uint8_t id = 1; id <= 4; id++) {
    SCOPED_TRACE("servo " + std::to_string(static_cast<int>(id)));
    EXPECT_DOUBLE_EQ(
      state_of("joint" + std::to_string(static_cast<int>(id)) + "/status"),
      static_cast<double>(bytes[id - 1]));
  }
  // and the fault WARN names the right id for the right byte, which a rotation would scramble
  EXPECT_THAT(warnings(), Contains(HasSubstr("motor id '3' reports status 0x20")));
  EXPECT_THAT(warnings(), Contains(HasSubstr("motor id '4' reports status 0x80")));
}

TEST_F(StatusOverPty, a_cleared_fault_still_re_enables_torque_once_per_joint)
{
  // Two faults clear in one cycle: each edge must use its own status_bytes_ entry, not a
  // neighbour's.
  bring_up();
  fake_.set_status(1, 0x04);
  fake_.set_status(3, 0x20);
  for (int cycle = 0; cycle < 3; cycle++) {
    step_read();
  }
  const FakeServo latched_one = fake_.snapshot(1);
  const FakeServo latched_three = fake_.snapshot(3);
  ASSERT_EQ(count_containing(warnings(), "motor id '1' reports status 0x04"), 1u);
  ASSERT_EQ(count_containing(warnings(), "motor id '3' reports status 0x20"), 1u);

  fake_.set_status(1, 0x00);
  fake_.set_status(3, 0x00);
  for (int cycle = 0; cycle < 5; cycle++) {
    step_read();
  }

  EXPECT_EQ(count_containing(infos(), "motor id '1' cleared its fault; re-enabling torque"), 1u);
  EXPECT_EQ(count_containing(infos(), "motor id '3' cleared its fault; re-enabling torque"), 1u);
  EXPECT_EQ(fake_.snapshot(1).torque_enable_writes, latched_one.torque_enable_writes + 1);
  EXPECT_EQ(fake_.snapshot(3).torque_enable_writes, latched_three.torque_enable_writes + 1);
  // the two joints that never had a fault are not swept along by their neighbours' edges
  EXPECT_THAT(infos(), Not(Contains(HasSubstr("motor id '2' cleared"))));
  EXPECT_THAT(infos(), Not(Contains(HasSubstr("motor id '4' cleared"))));
}
