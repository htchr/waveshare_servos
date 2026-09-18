// The whole component lifecycle over a pseudo terminal (PHASE2_SPEC 10.2 - 10.5).
//
// The plugin is loaded through pluginlib from the ament index -- the installed artifact, as
// test_load_waveshare_servos loads it -- and driven by a ResourceManager, while a fake SMS/STS
// servo bus (test/fake_servo_bus.hpp) answers on the master side of an openpty() pair. Nothing
// here needs a servo, a USB adapter or the real bench port.
//
// Standing rule, mechanical and non-negotiable: every case that reaches on_configure passes an
// explicit <param name="port"> naming its own pty, and every case ends with
// EXPECT_FALSE(process_has_serial_port_open()) -- here in the fixture's TearDown, so a case
// cannot forget it, and before the resource manager is destroyed, so the shutdown that closes the
// port cannot hide what the case opened. The driver's default port is the bench adapter's, and the
// bench servos are on it; a case that forgot the param would drive real motors out of
// `colcon test`. That default
// is asserted from the driver's log line in test_load_waveshare_servos, never by configuring, and
// neither this file nor test/fake_servo_bus.hpp ever spells that device path out, so a grep for
// it over the two of them comes back empty.
//
// Three fixtures:
//   LifecycleOverPty    configure / activate / read / write / park, over four healthy servos
//   DropRecoverOverPty  PHASE2_SPEC 10.4: a servo that goes silent mid-run (D7(a)'s deterministic
//                       replacement for the bench connector pull)
//   StatusOverPty       PHASE2_SPEC 10.5: synthetic status bytes (D7(b)'s replacement for the
//                       held-wheel overload test)
//
// ResourceManager::set_component_state takes its target state by non-const lvalue reference, so
// every transition goes through go(), which names one.

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

// The hardware constants the driver defaults to, spelled out rather than included: this is an
// end-to-end test, so the numbers it expects have to be derived independently of the header the
// driver computes them with.
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

// joint1 and joint2 run servos 1 and 2 to a goal position; joint3 and joint4 are wheels on servos
// 3 and 4. joint4 is the wheel every command assertion uses: urdf_head gives joint1..joint3 a
// <limit velocity="0.2">, and test_helpers' joint4 is continuous with no limit at all, so a
// velocity command reaches the driver unmodified whatever the framework decides to enforce.
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

// One record of a SyncWriteSpe packet: the goal speed alone, sign-magnitude on bit 15
// (src/SMS_STS.cpp:122-280). The ACC byte of a wheel does not travel in this record; the library
// sends it as a separate addressed write to register 41.
int decode_speed_record(const std::vector<uint8_t> & record)
{
  if (record.size() != 2) {
    return std::numeric_limits<int>::min();
  }
  return sign_magnitude_decode(static_cast<uint16_t>(record[0] | (record[1] << 8)));
}

// A plain file under the system temp directory, removed however the case that made it ends. One
// case needs a port that opens but is not a tty; a file is the only such path a test can make for
// itself.
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

  // Whether the file is really there. A case that stages "openable but not a tty" with a file it
  // could not create would silently become a different case -- the driver would refuse the path
  // for not existing -- so the one case that uses this asserts it first, and a temp directory that
  // is not writable is reported as the harness failure it is.
  bool ok() const {return created_;}

private:
  std::filesystem::path path_;
  bool created_ = false;
};

// Holds the framework's own writer lock on one state handle for as long as it is in scope, so the
// driver's non-blocking set_state() finds it busy and leaves the previous sample in place. The
// handle's mutex is reachable only through the loan's protected member, hence the derived type.
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
    // The standing rule first, while the component is still alive and still holding whatever it
    // opened: destroying the resource manager shuts it down and closes the port, so a check placed
    // after the reset could never catch the one thing this rule exists to catch -- a case that
    // forgot <param name="port"> and configured against the bench adapter. Nothing in this file
    // legitimately holds a real serial port at this point: the fake bus is a /dev/pts slave, which
    // matches none of the prefixes process_has_serial_port_open looks for.
    EXPECT_FALSE(process_has_serial_port_open()) << "a case reached a real serial port";
    // Then the leak checks. Destroying the resource manager shuts every component down, which
    // parks the servos and closes the port; after that the fixture's own slave descriptor must be
    // the only one left on the port, and nothing malformed can ever have gone out on the wire.
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

  // Returns whether the component came up, rather than asserting: a gtest ASSERT_* only returns
  // from the function it appears in, so an assertion here would let the case carry on and call
  // configure() on a resource manager that has no `four_servos` component. Every call site says
  // ASSERT_TRUE(load(...)) instead, which really does stop the case.
  [[nodiscard]] bool load(const std::string & urdf)
  {
    auto params = resource_manager_params(urdf);
    rm_ = std::make_unique<hardware_interface::ResourceManager>(params, false);
    return rm_->load_and_initialize_components(params);
  }

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
    // No wait_quiet: every transaction read() makes is synchronous, so the responder has already
    // seen the request by the time rm_->read() returns -- it either answered it or deliberately
    // did not.
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
  // the deprecated alias keeps kg cm, frozen at the Phase 1 value (D2)
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
  // 20 acceleration counts and 100 speed counts, written as the rad/s^2 and rad/s the description
  // declares: counts = value * encoder_steps / (2 pi), with a further /100 for the acceleration
  // register (PHASE2_SPEC 5.4). This also discharges 5.4's owed coverage -- that the CONVERTED
  // values reach the wire, not the 6000/150 register defaults.
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
  // PHASE2_SPEC 8.4 and 8.8. joint3 is a wheel, so `unwrap` is on by its default. Its register sits
  // at the top of the turn when the component activates and wraps to 12 on the next sample: the
  // published position must step FORWARD by the 13 ticks the servo actually turned (jazzy.md:43's
  // bench observation), never back to 0.018 rad.
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
  // The opt-out, and the exact-equality pin of the chunk: a joint with unwrap=false publishes the
  // plain register reading, bit for bit what the previous chunk's driver published. joint4 keeps
  // the default and makes the same motion in the same cycles, so the two are compared against each
  // other and not only against a constant.
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
// WaveshareServosAbsentServo (PHASE2_SPEC 6): the allow_missing_servos configure-time gate.
//
// `absent` here, never `silent_feedback`: this is "no servo on the bus at all", the failure the
// configure-time gate exists for. The runtime drop of a servo that was there and went quiet is
// DropRecoverOverPty's subject, and the two must not be confused -- an absent servo answers no
// ping, so it never reaches the read path in the first place.

class WaveshareServosAbsentServo : public PtyFixture
{
protected:
  // 30 ms rather than the 20 ms default, so a ping to a servo that is not there costs one
  // generous, known unit; max_read_fails 2 so the one case that needs a runtime drop pays two of
  // them instead of fifty.
  static constexpr int kIoTimeoutMs = 30;
  static constexpr int kMaxReadFails = 2;
  // The timing case's second io timeout, four times the first, and the retry budget both of its
  // measurements are taken at. 120 ms is inside the parser's 1..1000 range (src/waveshare_servos
  // .cpp) and three attempts of it is the longest wait in this file: 360 ms, once.
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

  // Deliberately without <param name="allow_missing_servos"> unless a case asks for one: the
  // default the gate is armed with has to come from the driver, not from the description (D1).
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

  // The same four servos with the io timeout and the retry budget named by the case instead of
  // taken from the fixture's constant. Only the timing case needs this: what it measures is the
  // cost of a ping to a servo that is not there, and the only way to show that cost tracks
  // io_timeout_ms is to configure twice with two different values of it.
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

  // the Phase 1 per-servo ping WARN, whose wording is frozen: the bench check greps for it
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
  // and that gate message is the ONE FATAL of the refusal path (PHASE2_SPEC 6.3), not merely one
  // of several: the bench predicate H5A is worded "exactly one driver FATAL", so it is pinned
  // here, where the count is deterministic.
  EXPECT_EQ(fatals().size(), 1u);
  // the Phase 1 per-servo WARN still fires, once per missing servo, with its wording unchanged
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

  // Only the fixture's own pty slave is left: neither the library's descriptor nor the bus's lock
  // descriptor survived. Nothing else could have closed them -- a component refused out of
  // UNCONFIGURED gets neither on_error nor on_cleanup (PHASE2_SPEC 6.3), so on_configure itself
  // has to, before it returns.
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
  // A plain file is openable but is not a tty, so ServoBus refuses it AFTER its own ::open() has
  // already succeeded -- the class of open failure in which a descriptor exists to be leaked, and
  // the one a test can stage. (The tcsetattr refusal whose FATAL says "could not be configured"
  // verbatim needs a device that vanishes between open and tcsetattr, which no test can arrange;
  // it reaches the same uniform rule by the same path.)
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

  // The observable half of the uniform rule (PHASE2_SPEC 6.3), and the half a descriptor count on
  // a file cannot see: a component refused at open is left clean enough to configure again, on a
  // port that works, without a cleanup in between -- exactly what the sibling case
  // configure_closes_the_port_before_returning_failure asserts for the gate path.
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
  // Servo 3 is a wheel sitting in mode 0, so build_groups() -> set_mode() would unlock EPROM,
  // rewrite register 33 and lock it again. The gate runs BEFORE build_groups() precisely so a
  // configuration that is about to be refused never spends one of that cell's write cycles
  // (PHASE2_SPEC 6.1).
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
  // PHASE2_SPEC 6.2: the gate is a configure-time policy and there is deliberately no
  // activate-time one. This description sets allow_missing_servos nowhere, so the gate is armed --
  // and a deactivate/activate of a component that has already lost a servo must still come up, or
  // a robot that has just recovered two servos of three could not be re-armed.
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

  // The other half of 6.2: a fresh configure IS subject to the gate, so the two recovery routes
  // are not the same route.
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
  // What the permissive WARN promises: "their joints mirror their commands into their states
  // until the servos answer". Both ways of losing a servo take the same branch, so both are
  // checked -- servo 1 was never on the bus, servo 2 was and was dropped mid-run.
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
  // What this pins: a servo that is not on the bus costs ping_attempts timeouts at configure and
  // nothing more -- not one timeout per servo on the bus, and not the library's stock 100 ms,
  // which is what made an absent servo cost a tenth of a second of every control period before
  // the io timeout was put in force ahead of the first transaction.
  //
  // The primary assertion is a count and does not look at a clock at all: a servo that answers is
  // pinged exactly once whatever ping_attempts says, so the retry budget is spent only on the
  // servo that is missing.
  //
  // The timing check that follows holds ping_attempts at 3 and varies io_timeout_ms instead, so
  // what it compares are two structurally different costs -- 3 x 30 ms against 3 x 120 ms -- and
  // not one cost against a scaled copy of itself. It asserts no absolute wall-clock bound
  // (PHASE2_SPEC 10.3): both measurements come from this case on this machine, the fixed
  // configure overhead O is the same in both, and the claim is only that the slow one is more
  // than twice the fast one. With the per-attempt cost tracking io_timeout_ms that reads
  // O + 360 > 2 x (O + 90), true for any O below 180 ms (O is about 3 ms here). Let the absent
  // ping revert to the library's stock 100 ms and both configures cost the same O + 300, so the
  // assertion fails -- which is the regression named above and the only reason this case exists.
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

// ---------------------------------------------------------------------------------------------
// DropRecoverOverPty (PHASE2_SPEC 10.4): a servo that goes silent mid-run. D7(a)'s deterministic
// replacement for the bench connector pull -- a fake servo that stops answering on cue, between
// two driver calls, so "the servo goes silent on cycle N" is the same on every machine.

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

  // The gap-warning cases (PHASE2_SPEC 10.4 cases 11-13) need a retry budget their silence cannot
  // reach: twelve failed reads have to produce the "lost revolutions" WARN without dropping the
  // servo, which is the whole point of a warning driven by elapsed time rather than by the drop.
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

  // The gap WARN of PHASE2_SPEC 8.5, with every number computed from the fixture's own parameters
  // rather than guessed: `cycle` is max(last_period_, io_timeout_ms), last_period_ being 0.01 s
  // until a write() sets it and these cases never write; `gap` is that cycle times the failed reads
  // plus the one that recovered; `possible` is the joint's SPEED CEILING over that gap, which is
  // why the message says "may be". A cycle in which nothing failed is charged no timeout at all --
  // its gap is the elapsed period, which lost_revolutions_over() below states directly.
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
  // A position servo 3 could not have got by accident: read()'s absent branch publishes 0.0 for a
  // wheel (pos_cmds_ is NaN for a velocity joint, cpp:1224), which is also what an untouched
  // register file reads back, so a zero here could not tell "held the last good sample" apart from
  // "was dropped early". 700 ticks can only have come from a real feedback reply.
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
  // One joint only, so no healthy round trip can flake the count, and a 2 ms timeout so 201
  // failing reads cost about 0.4 s. max_read_fails is far out of reach: this case measures the
  // throttle and nothing else.
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

TEST_F(DropRecoverOverPty, a_silent_servo_is_dropped_at_max_read_fails_with_the_phase1_messages)
{
  bring_up();
  fake_.set_silent_feedback(3, true);

  for (int cycle = 1; cycle < kMaxReadFails; cycle++) {
    SCOPED_TRACE("failed read " + std::to_string(cycle));
    step_read();
    EXPECT_THAT(errors(), Not(Contains(HasSubstr("stopped answering"))));
  }
  // cpp:541 is `>=`, not `>`: the drop lands on the 8th failed read, not the 9th
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
  // Both kinds at once: servo 1 is a position joint, whose sync write is one broadcast packet,
  // and servo 3 is a wheel, whose SyncWriteSpe sends an addressed ACC write per servo before the
  // broadcast (src/SMS_STS.cpp:275). The claim under test -- a dropped servo costs no further bus
  // time -- holds for both, but only servo 1 goes completely silent on the wire.
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
  // What it still receives is SMS_STS::SyncWriteSpe's per-servo ACC write, one per write() cycle
  // and not the driver's doing: the library sends register 41 addressed before broadcasting the
  // speeds. It is acked, so it costs no timeout -- but it is a request, so this is asserted
  // exactly rather than glossed.
  EXPECT_EQ(dropped_wheel.writes, dropped_wheel_before.writes + 20);
  EXPECT_EQ(dropped_wheel.requests, dropped_wheel_before.requests + 20);

  // meanwhile the healthy servos are polled every cycle
  EXPECT_EQ(fake_.snapshot(2).feedback_reads, healthy_before.feedback_reads + 20);
  EXPECT_EQ(fake_.snapshot(4).feedback_reads, healthy_wheel_before.feedback_reads + 20);
}

TEST_F(DropRecoverOverPty, a_dropped_servo_keeps_its_place_in_the_sync_write_groups)
{
  // Phase 1 behaviour, deliberately preserved: the drop sets present_ false and nothing else.
  // build_groups() is not called and send_commands() never consults present_, so a dropped servo
  // stays in the sync-write groups and still receives its record. Do not "fix" this -- it would
  // change the packet contents and move the bench's goal_position_raw rows.
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
  // The records cost no addressed round trip of their own: a sync write is a broadcast to 0xfe and
  // is never acked. Servo 3 is a wheel, so the only addressed request it still sees is
  // SMS_STS::SyncWriteSpe's per-servo ACC write (src/SMS_STS.cpp:275), exactly one per write()
  // cycle -- so `requests` grows by the cycle count and by nothing else.
  EXPECT_EQ(fake_.snapshot(3).requests, dropped_before.requests + 3) <<
    "only SyncWriteSpe's per-servo ACC write, nothing from the sync write itself";
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
  // only joints that are absent are re-pinged (cpp:373-375)
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

  // one good read resets the counter to 0 (cpp:554)
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

TEST_F(DropRecoverOverPty, a_dropped_wheel_does_not_jump_back_to_its_activation_position)
{
  // PHASE2_SPEC 8.6, the blocker this chunk fixes. read()'s absent branch mirrors pos_cmds_ into
  // pos_states_, and a `vel` joint's pos_cmds_ was last written at ACTIVATION, because it declares
  // no position command interface for a controller to refresh. Without the fix a wheel unwrapped
  // several revolutions out publishes its activation position on the cycle after the drop: an
  // unbounded backward jump, strictly worse than the bounded 2 pi discontinuity unwrapping removes.
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
  // the drop cycle itself still publishes the last good sample: cpp:541-550 sets present_ and
  // continues, so the absent branch does not run until the next cycle
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

TEST_F(DropRecoverOverPty, a_rejoining_wheel_starts_a_new_unwrapped_count)
{
  // PHASE2_SPEC 8.4: on_activate resets the counter, so a wheel that rejoins reports its plain
  // register reading again rather than carrying an old count forward. The consumer that integrates
  // wheel position re-zeroes on hardware activation, exactly as it does on controller activation.
  // joint4 is the control: the same wheel motion, never silent, never dropped. It pins that the
  // reset is on_activate's and not the drop's -- a driver that only reset a REJOINING servo's
  // count would leave this one continuing from 10000 ticks across the same transition.
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
  EXPECT_DOUBLE_EQ(state_of("joint4/position"), rad_of_tick(900, 0.0)) <<
    "every activation starts a new count, dropped servo or not";
  // the re-seed is not a gap: PositionUnwrapper::bridged() is false on the seeding sample, so
  // nothing about lost revolutions is reported for it
  EXPECT_THAT(warnings(), Not(Contains(HasSubstr("may be off by whole revolutions"))));
}

TEST_F(DropRecoverOverPty, a_long_silence_that_does_not_reach_the_drop_warns_about_lost_revolutions)
{
  // PHASE2_SPEC 8.5 and 10.4 case 11, the warning the manual connector pull was going to produce.
  // Twelve silent cycles at a 30 ms failed-read cost is 0.39 s, and a wheel at its 9.20 rad/s
  // ceiling could have turned 3.59 rad in that time -- more than the half revolution the unwrapper
  // needs to be sure of the direction, so the count MAY now be short by whole revolutions.
  // max_read_fails is 40, so nothing is dropped: this is the warning path, and it is reachable
  // only because the warning is driven by elapsed time and not only by the drop rule.
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
  // The other half of PHASE2_SPEC 8.5, and the only case that fails if note_unwrap_gap is moved
  // inside an `if (missed > 0)` or if read() goes back to discarding its `period`: a control loop
  // that was descheduled for half a second produces a successful read with missed == 0, and the
  // wheel could still have turned 4.60 rad unseen. 12-30 ms overruns are recorded on this bench
  // (jazzy.md:39), so the elapsed-time half of the predicate is not hypothetical.
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
  // The negative side of the same predicate (PHASE2_SPEC 10.4 case 12), without which the throttle
  // of 8.5 could be satisfied by warning about every gap. Three silent cycles is 0.12 s, in which
  // the same wheel could have turned only 1.10 rad -- well under the half revolution that would
  // make the count ambiguous.
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
  // A gap is only evidence about the count it interrupted, and on_activate starts a new one
  // (PHASE2_SPEC 8.4). The failure count that the silence left behind must therefore not be
  // charged to the first read of the NEXT activation: the servo answered on activation, the
  // unwrapper was re-seeded from that answer, and nothing about the pre-activation silence can
  // have been lost from a count that did not exist yet. Without the reset the driver reports a
  // gap that is a whole activation old, in a message the bench check greps for.
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
  // and the new count really is a new count, seeded from the register the servo answered with
  EXPECT_DOUBLE_EQ(state_of("joint3/position"), rad_of_tick(700, 0.0));
  // the next cycles are ordinary healthy ones too, so nothing arrives late either
  for (int cycle = 0; cycle < 3; cycle++) {
    step_read();
  }
  EXPECT_EQ(count_containing(warnings(), "without a reading"), 0u);
}

TEST_F(DropRecoverOverPty, a_healthy_cycle_is_not_charged_a_failed_reads_timeout)
{
  // Every read of this case ANSWERED, so the elapsed time between two samples is the control
  // period, 10 ms, and not the read timeout: a timeout is what a failed round trip costs, and
  // there were none. io_timeout_ms is a legal 400 here, which is above the 342 ms at which the
  // ceiling alone reaches half a revolution, so a driver that charged the timeout to a successful
  // read would report "went 0.400 s without a reading" on every cycle of a healthy 100 Hz loop --
  // a statement that is simply untrue, and a counter that then re-warns every 200th cycle.
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
  // PHASE2_SPEC 8.9, made executable. A cycle whose read failed never calls unwrap_ticks() at all,
  // so the accumulator is advanced only by a sample that actually arrived: the next real delta is
  // measured against the last REAL raw tick, not against the stale one that was re-published
  // twelve times. The Phase 1 recordings show exactly this shape across a lifecycle transition --
  // a run of bit-identical samples and then one catch-up step -- and an accumulator seeded or
  // advanced from a stale sample would be wrong by exactly the motion that happened meanwhile.
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
// StatusOverPty (PHASE2_SPEC 10.5): synthetic status bytes. This proves the decode path end to
// end -- reply byte -> status_bytes_ -> the `status` interface -> the fault-edge WARN/INFO ->
// EnableTorque -- without claiming what any bit means on this firmware (PHASE2_SPEC 7.8). What
// these cases deliberately do NOT assert: that bit 5 is what an overloaded ST3025 raises, or that
// the packet byte equals register 65. Both are unverified.

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
  // the honesty rule of D7 reaches the operator's log, not only the header
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
  step_read();   // the absent branch clears last_error_ and status_bytes_ (PHASE2_SPEC 7.9)

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
