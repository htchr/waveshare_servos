// Load-time tests for the waveshare_servos/WaveshareServos plugin.
//
// The plugin is loaded from a URDF string through the ResourceManager (or the System wrapper) and
// initialized, but it is NEVER configured or activated: the port is still hard-coded to
// /dev/ttyACM0, and configure would open it and talk to real servos.
//
// Two groups of tests:
// - WaveshareServosLoad / WaveshareServosRejects pin the behavior of the pre-Jazzy driver at
//   init time (interfaces, values, on_init checks). They must pass before and after the Phase 1
//   API migration.
// - WaveshareServosJazzyApi checks what the migration is meant to change (framework-created
//   interface handles, the component logger). They fail on the pre-Jazzy driver.

#include <gmock/gmock.h>

#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <ostream>
#include <string>
#include <system_error>
#include <tuple>
#include <utility>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_params.hpp"
#include "hardware_interface/types/resource_manager_params.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rcutils/logging.h"
#include "ros2_control_test_assets/descriptions.hpp"

namespace
{

using ::testing::Contains;
using ::testing::ElementsAre;
using ::testing::ElementsAreArray;
using ::testing::HasSubstr;
using ::testing::IsEmpty;
using ::testing::Not;
using ::testing::UnorderedElementsAreArray;
using lifecycle_msgs::msg::State;

constexpr char kPlugin[] = "waveshare_servos/WaveshareServos";
constexpr char kRmLogger[] = "test_rm";

// ---------------------------------------------------------------------------------------------
// URDF building

struct Joint
{
  std::string name;
  std::string id;
  std::string type;
  std::string offset;  // no <param name="offset"> when empty
  std::vector<std::string> command_interfaces;  // XML fragments
  std::vector<std::string> state_interfaces;  // XML fragments
};

// <command_interface>, with min/max params when given
std::string command(
  const std::string & name, const std::string & min = "", const std::string & max = "")
{
  if (min.empty() && max.empty()) {
    return "<command_interface name=\"" + name + "\"/>";
  }
  std::string xml = "<command_interface name=\"" + name + "\">";
  if (!min.empty()) {
    xml += "<param name=\"min\">" + min + "</param>";
  }
  if (!max.empty()) {
    xml += "<param name=\"max\">" + max + "</param>";
  }
  return xml + "</command_interface>";
}

// <state_interface>, with an initial_value param when given
std::string state(const std::string & name, const std::string & initial_value = "")
{
  if (initial_value.empty()) {
    return "<state_interface name=\"" + name + "\"/>";
  }
  return "<state_interface name=\"" + name + "\"><param name=\"initial_value\">" +
         initial_value + "</param></state_interface>";
}

std::vector<std::string> servo_states()
{
  return {state("position"), state("velocity"), state("torque"), state("temperature")};
}

// joint1/joint2 of description/ros2_control/example.ros2_control.xacro
Joint example_position_joint(const std::string & name, const std::string & id)
{
  return Joint{
    name, id, "pos", "1.570796",
    {command("position", "-1.570796", "1.570796"), command("velocity", "-9.2", "9.2")},
    servo_states()};
}

// joint3 of the example xacro
Joint example_velocity_joint(const std::string & name, const std::string & id)
{
  return Joint{name, id, "vel", "", {command("velocity", "-9.2", "9.2")}, servo_states()};
}

// the three joints of the example xacro, same params, offsets and limits
std::vector<Joint> example_joints()
{
  return {
    example_position_joint("joint1", "1"), example_position_joint("joint2", "2"),
    example_velocity_joint("joint3", "3")};
}

// the four servos on the bench: two position joints and two wheels, velocity without limits
std::vector<Joint> bench_joints()
{
  std::vector<Joint> joints;
  for (const std::string id : {"1", "2"}) {
    joints.push_back(
      Joint{
        "joint" + id, id, "pos", "1.570796",
        {command("position", "-1.570796", "1.570796"), command("velocity")}, servo_states()});
  }
  for (const std::string id : {"3", "4"}) {
    joints.push_back(Joint{"joint" + id, id, "vel", "", {command("velocity")}, servo_states()});
  }
  return joints;
}

std::string joint_xml(const Joint & joint)
{
  std::string xml = "<joint name=\"" + joint.name + "\">";
  xml += "<param name=\"id\">" + joint.id + "</param>";
  xml += "<param name=\"type\">" + joint.type + "</param>";
  if (!joint.offset.empty()) {
    xml += "<param name=\"offset\">" + joint.offset + "</param>";
  }
  for (const auto & fragment : joint.command_interfaces) {
    xml += fragment;
  }
  for (const auto & fragment : joint.state_interfaces) {
    xml += fragment;
  }
  return xml + "</joint>\n";
}

// ros2_control_test_assets::urdf_head defines the revolute joints joint1..joint3. Every joint of a
// <ros2_control> block has to exist in the URDF, so a continuous joint4 is added for the second
// wheel.
constexpr char kUrdfJoint4[] =
  R"(
  <link name="wheel4_link"/>
  <joint name="joint4" type="continuous">
    <parent link="base_link"/>
    <child link="wheel4_link"/>
    <axis xyz="0 0 1"/>
  </joint>
)";

std::string robot_description(
  const std::string & hardware_name, const std::vector<Joint> & joints,
  const std::string & hardware_params = "", const std::string & extra_components = "")
{
  std::string urdf = std::string(ros2_control_test_assets::urdf_head) + kUrdfJoint4;
  urdf += "<ros2_control name=\"" + hardware_name + "\" type=\"system\">\n<hardware>\n";
  urdf += std::string("<plugin>") + kPlugin + "</plugin>\n" + hardware_params + "</hardware>\n";
  for (const auto & joint : joints) {
    urdf += joint_xml(joint);
  }
  return urdf + extra_components + "</ros2_control>\n" + ros2_control_test_assets::urdf_tail;
}

// the example's hardware block also carries demo params that the driver ignores
constexpr char kExampleHardwareParams[] =
  R"(
  <param name="example_param_hw_start_duration_sec">0</param>
  <param name="example_param_hw_stop_duration_sec">3.0</param>
  <param name="example_param_hw_slowdown">100</param>
)";
constexpr char kExampleName[] = "example_ws_ros2_control";
constexpr char kBenchName[] = "four_servos";

std::string example_description(
  const std::vector<Joint> & joints = example_joints(),
  const std::string & extra_components = "")
{
  return robot_description(kExampleName, joints, kExampleHardwareParams, extra_components);
}

std::string bench_description(const std::vector<Joint> & joints = bench_joints())
{
  return robot_description(kBenchName, joints);
}

const std::vector<std::string> kServoStateNames = {"position", "velocity", "torque", "temperature"};

std::vector<std::string> state_keys(const std::vector<std::string> & joint_names)
{
  std::vector<std::string> keys;
  for (const auto & joint : joint_names) {
    for (const auto & name : kServoStateNames) {
      keys.push_back(joint + "/" + name);
    }
  }
  return keys;
}

// ---------------------------------------------------------------------------------------------
// Log capture: every rcutils log record is kept and then passed on to the console handler.

struct LogRecord
{
  int severity;
  std::string logger;
  std::string message;
};

std::mutex g_log_mutex;
std::vector<LogRecord> g_log_records;
rcutils_logging_output_handler_t g_console_handler = nullptr;

void capture_log(
  const rcutils_log_location_t * location, int severity, const char * name,
  rcutils_time_point_value_t timestamp, const char * format, va_list * args)
{
  va_list size_args;
  va_copy(size_args, *args);
  const int length = std::vsnprintf(nullptr, 0, format, size_args);
  va_end(size_args);
  std::string message;
  if (length > 0) {
    std::vector<char> buffer(static_cast<size_t>(length) + 1);
    va_list print_args;
    va_copy(print_args, *args);
    std::vsnprintf(buffer.data(), buffer.size(), format, print_args);
    va_end(print_args);
    message.assign(buffer.data(), static_cast<size_t>(length));
  }
  {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    g_log_records.push_back(LogRecord{severity, name != nullptr ? name : "", message});
  }
  if (g_console_handler != nullptr) {
    g_console_handler(location, severity, name, timestamp, format, args);
  }
}

class LogCapture
{
public:
  LogCapture()
  {
    // The logging macros initialize rcutils logging on first use, which installs the console
    // handler over any handler set before. Initialize first so the capture stays in place.
    RCUTILS_LOGGING_AUTOINIT;
    std::lock_guard<std::mutex> lock(g_log_mutex);
    g_log_records.clear();
    g_console_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(&capture_log);
  }

  ~LogCapture()
  {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    rcutils_logging_set_output_handler(g_console_handler);
    g_console_handler = nullptr;
  }

  LogCapture(const LogCapture &) = delete;
  LogCapture & operator=(const LogCapture &) = delete;

  std::vector<LogRecord> records() const
  {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    return g_log_records;
  }

  std::vector<std::string> messages(int severity) const
  {
    std::vector<std::string> selected;
    for (const auto & record : records()) {
      if (record.severity == severity) {
        selected.push_back(record.message);
      }
    }
    return selected;
  }
};

// ---------------------------------------------------------------------------------------------
// Bus guard: SCSerial::begin() keeps the tty open, so a configure would leave a descriptor behind.

bool process_has_serial_port_open()
{
  for (const auto & entry : std::filesystem::directory_iterator("/proc/self/fd")) {
    std::error_code error;
    const std::string target = std::filesystem::read_symlink(entry.path(), error).string();
    if (error) {
      continue;
    }
    for (const char * prefix : {"/dev/ttyACM", "/dev/ttyUSB", "/dev/ttyTHS", "/dev/serial"}) {
      if (target.rfind(prefix, 0) == 0) {
        return true;
      }
    }
  }
  return false;
}

hardware_interface::ResourceManagerParams resource_manager_params(const std::string & urdf)
{
  hardware_interface::ResourceManagerParams params;
  params.robot_description = urdf;
  params.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  params.logger = rclcpp::get_logger(kRmLogger);
  return params;
}

// Loads and initializes the plugin through the System wrapper, the way the ResourceManager does,
// so the exported interface values can be read without configuring anything.
class InitializedSystem
{
public:
  explicit InitializedSystem(const std::string & urdf)
  : loader_("hardware_interface", "hardware_interface::SystemInterface"),
    system_(std::unique_ptr<hardware_interface::SystemInterface>(
        loader_.createUnmanagedInstance(kPlugin)))
  {
    const auto infos = hardware_interface::parse_control_resources_from_urdf(urdf);
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = infos.at(0);
    params.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    params.logger = rclcpp::get_logger("test_system");
    state_id_ = system_.initialize(params).id();
  }

  uint8_t state_id() const {return state_id_;}
  hardware_interface::System & system() {return system_;}

private:
  // declared first: the plugin library must stay loaded while the instance exists
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader_;
  hardware_interface::System system_;
  uint8_t state_id_ = State::PRIMARY_STATE_UNKNOWN;
};

}  // namespace

// The tests stay outside the anonymous namespace: cppcheck 2.13 reports a syntaxError for a
// TEST_F inside one.

// ---------------------------------------------------------------------------------------------
// Characterization: a valid description loads, initializes and exports the expected interfaces.

class WaveshareServosLoad : public ::testing::Test
{
protected:
  LogCapture logs_;
};

TEST_F(WaveshareServosLoad, example_three_joint_system_loads_unconfigured)
{
  auto params = resource_manager_params(example_description());
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));

  EXPECT_EQ(rm.system_components_size(), 1u);
  const auto & status = rm.get_components_status();
  ASSERT_EQ(status.count(kExampleName), 1u);
  EXPECT_EQ(status.at(kExampleName).plugin_name, kPlugin);
  EXPECT_EQ(status.at(kExampleName).state.id(), State::PRIMARY_STATE_UNCONFIGURED);

  EXPECT_THAT(
    rm.state_interface_keys(),
    UnorderedElementsAreArray(state_keys({"joint1", "joint2", "joint3"})));
  const std::vector<std::string> command_keys = {
    "joint1/position", "joint1/velocity", "joint2/position", "joint2/velocity", "joint3/velocity"};
  EXPECT_THAT(rm.command_interface_keys(), UnorderedElementsAreArray(command_keys));
  for (const auto & key : rm.state_interface_keys()) {
    EXPECT_EQ(rm.get_state_interface_data_type(key), "double") << key;
  }
  for (const auto & key : rm.command_interface_keys()) {
    EXPECT_EQ(rm.get_command_interface_data_type(key), "double") << key;
  }

  // on_init reports the position command limits it parsed, for the position joints only
  const auto info = logs_.messages(RCUTILS_LOG_SEVERITY_INFO);
  EXPECT_THAT(
    info, Contains(HasSubstr("joint 'joint1' position commands clamped to [-1.5708, 1.5708] rad")));
  EXPECT_THAT(
    info, Contains(HasSubstr("joint 'joint2' position commands clamped to [-1.5708, 1.5708] rad")));
  EXPECT_THAT(info, Not(Contains(HasSubstr("joint 'joint3' position commands clamped"))));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

TEST_F(WaveshareServosLoad, bench_four_joint_system_loads_unconfigured)
{
  auto params = resource_manager_params(bench_description());
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));

  EXPECT_EQ(rm.system_components_size(), 1u);
  const auto & status = rm.get_components_status();
  ASSERT_EQ(status.count(kBenchName), 1u);
  EXPECT_EQ(status.at(kBenchName).plugin_name, kPlugin);
  EXPECT_EQ(status.at(kBenchName).state.id(), State::PRIMARY_STATE_UNCONFIGURED);

  EXPECT_THAT(
    rm.state_interface_keys(),
    UnorderedElementsAreArray(state_keys({"joint1", "joint2", "joint3", "joint4"})));
  const std::vector<std::string> command_keys = {
    "joint1/position", "joint1/velocity", "joint2/position", "joint2/velocity", "joint3/velocity",
    "joint4/velocity"};
  EXPECT_THAT(rm.command_interface_keys(), UnorderedElementsAreArray(command_keys));
  for (const auto & key : rm.state_interface_keys()) {
    EXPECT_EQ(rm.get_state_interface_data_type(key), "double") << key;
  }
  for (const auto & key : rm.command_interface_keys()) {
    EXPECT_EQ(rm.get_command_interface_data_type(key), "double") << key;
  }

  const auto info = logs_.messages(RCUTILS_LOG_SEVERITY_INFO);
  EXPECT_THAT(
    info, Contains(HasSubstr("joint 'joint1' position commands clamped to [-1.5708, 1.5708] rad")));
  EXPECT_THAT(
    info, Contains(HasSubstr("joint 'joint2' position commands clamped to [-1.5708, 1.5708] rad")));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

// The resource manager keeps a component's interfaces in the order the component exports them.
// `ros2 control list_hardware_components` and every controller that claims all interfaces (the
// joint_state_broadcaster, so the per-joint order of /dynamic_joint_states) show them in that
// order. The pre-Jazzy driver exported them joint by joint in the order of the description: the
// four state interfaces in the order on_init requires, then the joint's command interfaces as
// listed.
TEST_F(WaveshareServosLoad, interfaces_are_listed_in_description_order)
{
  std::vector<Joint> reordered = bench_joints();
  // on_init accepts the command interfaces of a joint in any order
  std::swap(reordered[0].command_interfaces[0], reordered[0].command_interfaces[1]);
  const std::vector<std::pair<std::string, std::string>> cases = {
    {"example", example_description()},
    {"bench", bench_description()},
    {"bench, joint1 velocity command first", bench_description(reordered)}};
  for (const auto & [label, urdf] : cases) {
    SCOPED_TRACE(label);
    const auto infos = hardware_interface::parse_control_resources_from_urdf(urdf);
    const std::string & name = infos.at(0).name;
    std::vector<std::string> expected_states;
    std::vector<std::string> expected_commands;
    for (const auto & joint : infos.at(0).joints) {
      for (const auto & interface : joint.state_interfaces) {
        expected_states.push_back(joint.name + "/" + interface.name);
      }
      for (const auto & interface : joint.command_interfaces) {
        expected_commands.push_back(joint.name + "/" + interface.name);
      }
    }

    auto params = resource_manager_params(urdf);
    hardware_interface::ResourceManager rm(params, false);
    ASSERT_TRUE(rm.load_and_initialize_components(params));
    const auto & status = rm.get_components_status();
    ASSERT_EQ(status.count(name), 1u);
    EXPECT_THAT(status.at(name).state_interfaces, ElementsAreArray(expected_states));
    EXPECT_THAT(status.at(name).command_interfaces, ElementsAreArray(expected_commands));
  }
  EXPECT_FALSE(process_has_serial_port_open());
}

// The driver serves the interfaces of the <joint> elements and nothing else. A <gpio> or <sensor>
// in the same <ros2_control> block declares interfaces it never reads or writes, so the pre-Jazzy
// driver did not export them and the resource manager refused the description ("Discrepancy
// between robot description file (urdf) and actually exported HW interfaces"). The framework's
// default export would hand those handles out instead, and the component would load with
// interfaces that nothing ever updates.
TEST_F(WaveshareServosLoad, gpio_and_sensor_interfaces_are_refused_at_load)
{
  const std::vector<std::pair<std::string, std::string>> cases = {
    {"gpio state interface", "<gpio name=\"aux\"><state_interface name=\"led\"/></gpio>\n"},
    {"gpio command interface", "<gpio name=\"aux\"><command_interface name=\"led\"/></gpio>\n"},
    {"sensor state interface",
      "<sensor name=\"imu\"><state_interface name=\"orientation.x\"/></sensor>\n"}};
  for (const auto & [label, extra_components] : cases) {
    SCOPED_TRACE(label);
    auto params = resource_manager_params(example_description(example_joints(), extra_components));
    hardware_interface::ResourceManager rm(params, false);

    EXPECT_FALSE(rm.load_and_initialize_components(params));
    EXPECT_THAT(rm.state_interface_keys(), Not(Contains("aux/led")));
    EXPECT_THAT(rm.state_interface_keys(), Not(Contains("imu/orientation.x")));
    EXPECT_THAT(rm.command_interface_keys(), Not(Contains("aux/led")));
    EXPECT_THAT(
      logs_.messages(RCUTILS_LOG_SEVERITY_ERROR),
      Contains(HasSubstr("Discrepancy between robot description file (urdf) and actually "
      "exported HW interfaces")));
  }
  EXPECT_FALSE(process_has_serial_port_open());
}

// A command interface listed twice on one joint used to be exported twice, and the resource
// manager complained about it ("Tried to insert CommandInterface with already existing key"); the
// interfaces of the joints after it were then not imported, so the description was refused. The
// framework's interface map would merge the two entries and load the description without a word.
TEST_F(WaveshareServosLoad, a_command_interface_listed_twice_is_reported_at_load)
{
  {
    SCOPED_TRACE("duplicate on the last joint");
    std::vector<Joint> joints = example_joints();
    joints[2].command_interfaces.push_back(command("velocity"));
    auto params = resource_manager_params(example_description(joints));
    hardware_interface::ResourceManager rm(params, false);

    std::ignore = rm.load_and_initialize_components(params);
    EXPECT_THAT(
      logs_.messages(RCUTILS_LOG_SEVERITY_ERROR),
      Contains(HasSubstr("already existing key. Insert[joint3/velocity]")));
  }
  {
    SCOPED_TRACE("duplicate on the first joint");
    std::vector<Joint> joints = example_joints();
    joints[0].command_interfaces.push_back(command("velocity", "-9.2", "9.2"));
    auto params = resource_manager_params(example_description(joints));
    hardware_interface::ResourceManager rm(params, false);

    // the import stops at the duplicate, so the later joints have no interfaces and the
    // description is refused
    EXPECT_FALSE(rm.load_and_initialize_components(params));
    EXPECT_THAT(
      logs_.messages(RCUTILS_LOG_SEVERITY_ERROR),
      Contains(HasSubstr("already existing key. Insert[joint1/velocity]")));
  }
  EXPECT_FALSE(process_has_serial_port_open());
}

TEST_F(WaveshareServosLoad, interface_values_are_nan_before_configure)
{
  for (const std::string & urdf : {example_description(), bench_description()}) {
    const auto infos = hardware_interface::parse_control_resources_from_urdf(urdf);
    SCOPED_TRACE(infos.at(0).name);
    InitializedSystem initialized(urdf);
    ASSERT_EQ(initialized.state_id(), State::PRIMARY_STATE_UNCONFIGURED);

    std::vector<std::string> state_names;
    for (const auto & handle : initialized.system().export_state_interfaces()) {
      state_names.push_back(handle->get_name());
      const auto value = handle->get_optional();
      ASSERT_TRUE(value.has_value()) << handle->get_name();
      EXPECT_TRUE(std::isnan(*value)) << handle->get_name() << " = " << *value;
    }
    std::vector<std::string> command_names;
    for (const auto & handle : initialized.system().export_command_interfaces()) {
      command_names.push_back(handle->get_name());
      const auto value = handle->get_optional();
      ASSERT_TRUE(value.has_value()) << handle->get_name();
      EXPECT_TRUE(std::isnan(*value)) << handle->get_name() << " = " << *value;
    }

    std::vector<std::string> joint_names;
    std::vector<std::string> expected_commands;
    for (const auto & joint : infos.at(0).joints) {
      joint_names.push_back(joint.name);
      for (const auto & interface : joint.command_interfaces) {
        expected_commands.push_back(joint.name + "/" + interface.name);
      }
    }
    EXPECT_THAT(state_names, UnorderedElementsAreArray(state_keys(joint_names)));
    EXPECT_THAT(command_names, UnorderedElementsAreArray(expected_commands));
  }
  EXPECT_FALSE(process_has_serial_port_open());
}

TEST_F(WaveshareServosLoad, shutdown_from_unconfigured_never_opens_the_port)
{
  auto params = resource_manager_params(bench_description());
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));
  ASSERT_EQ(rm.get_components_status().at(kBenchName).state.id(),
      State::PRIMARY_STATE_UNCONFIGURED);

  EXPECT_TRUE(rm.shutdown_components());
  EXPECT_EQ(rm.get_components_status().at(kBenchName).state.id(), State::PRIMARY_STATE_FINALIZED);
  EXPECT_FALSE(process_has_serial_port_open());
}

// ---------------------------------------------------------------------------------------------
// Characterization: every check in on_init rejects its description with its own FATAL message.

namespace
{

struct Rejection
{
  std::string name;
  std::function<void(std::vector<Joint> &)> break_joints;
  std::string fatal_message;
};

void PrintTo(const Rejection & rejection, std::ostream * os)
{
  *os << rejection.name;
}

std::vector<Rejection> rejections()
{
  return {
    {"three_state_interfaces",
      [](std::vector<Joint> & j) {j[2].state_interfaces.pop_back();},
      "joint has the wrong number of state interfaces"},
    {"five_state_interfaces",
      [](std::vector<Joint> & j) {j[0].state_interfaces.push_back(state("effort"));},
      "joint has the wrong number of state interfaces"},
    {"position_state_not_first",
      [](std::vector<Joint> & j) {
        j[1].state_interfaces = {
          state("velocity"), state("position"), state("torque"), state("temperature")};
      },
      "a joint does not have the position state interface first"},
    {"velocity_state_not_second",
      [](std::vector<Joint> & j) {
        j[0].state_interfaces = {
          state("position"), state("torque"), state("velocity"), state("temperature")};
      },
      "a joint does not have the velocity state interface second"},
    {"torque_state_not_third",
      [](std::vector<Joint> & j) {
        j[2].state_interfaces = {
          state("position"), state("velocity"), state("effort"), state("temperature")};
      },
      "a joint does not have the torque state interface third"},
    {"temperature_state_not_fourth",
      [](std::vector<Joint> & j) {
        j[1].state_interfaces = {
          state("position"), state("velocity"), state("torque"), state("voltage")};
      },
      "a joint does not have the temperature state interface fourth"},
    {"no_command_interface",
      [](std::vector<Joint> & j) {j[2].command_interfaces.clear();},
      "a joint does not have a command interfaces"},
    {"effort_command_interface",
      [](std::vector<Joint> & j) {j[2].command_interfaces = {command("effort", "-1.0", "1.0")};},
      "a joint is using a command interface that isn't position or velocity"},
    {"type_not_pos_or_vel",
      [](std::vector<Joint> & j) {j[1].type = "position";},
      "a joint has the wrong type, it should be vel or pos"},
    {"position_min_not_a_number",
      [](std::vector<Joint> & j) {
        j[1].command_interfaces[0] = command("position", "abc", "1.570796");
      },
      "joint 'joint2' has a position min or max that is not a number"},
    {"position_max_not_a_number",
      [](std::vector<Joint> & j) {
        j[0].command_interfaces[0] = command("position", "-1.570796", "half_pi");
      },
      "joint 'joint1' has a position min or max that is not a number"},
    {"position_min_greater_than_max",
      [](std::vector<Joint> & j) {
        j[1].command_interfaces[0] = command("position", "2.0", "1.570796");
      },
      "joint 'joint2' has a position min greater than its max"},
    // std::stod accepts "nan", and a NaN limit would switch that side of the clamp off
    {"position_min_nan",
      [](std::vector<Joint> & j) {
        j[1].command_interfaces[0] = command("position", "nan", "1.570796");
      },
      "joint 'joint2' has a position min greater than its max"},
    {"position_max_nan",
      [](std::vector<Joint> & j) {
        j[0].command_interfaces[0] = command("position", "-1.570796", "nan");
      },
      "joint 'joint1' has a position min greater than its max"},
  };
}

}  // namespace

class WaveshareServosRejects : public ::testing::TestWithParam<Rejection>
{
protected:
  LogCapture logs_;
};

TEST_P(WaveshareServosRejects, on_init_rejects_the_description)
{
  std::vector<Joint> joints = example_joints();
  GetParam().break_joints(joints);
  auto params = resource_manager_params(example_description(joints));
  hardware_interface::ResourceManager rm(params, false);

  EXPECT_FALSE(rm.load_and_initialize_components(params));
  EXPECT_EQ(rm.system_components_size(), 0u);
  EXPECT_THAT(rm.get_components_status(), IsEmpty());
  EXPECT_THAT(rm.state_interface_keys(), IsEmpty());
  EXPECT_THAT(rm.command_interface_keys(), IsEmpty());
  // the driver stops at the first failed check, so exactly its message is logged
  EXPECT_THAT(
    logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), ElementsAre(HasSubstr(GetParam().fatal_message)));
  EXPECT_FALSE(process_has_serial_port_open());
}

INSTANTIATE_TEST_SUITE_P(
  OnInit, WaveshareServosRejects, ::testing::ValuesIn(rejections()),
  [](const ::testing::TestParamInfo<Rejection> & info) {return info.param.name;});

// The unmodified example description is accepted, so every rejection above comes from its one
// change and not from the rest of the description.
TEST_F(WaveshareServosLoad, unmodified_example_description_is_accepted)
{
  auto params = resource_manager_params(example_description());
  hardware_interface::ResourceManager rm(params, false);
  EXPECT_TRUE(rm.load_and_initialize_components(params));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
}

// ---------------------------------------------------------------------------------------------
// Jazzy API migration (Phase 1). These fail on the pre-Jazzy driver and pass once it is migrated.

class WaveshareServosJazzyApi : public ::testing::Test
{
protected:
  LogCapture logs_;
};

// Phase 1 item 2: the interfaces are the framework's handles built from the URDF, not handles
// over the driver's own arrays. Only framework handles honor <param name="initial_value">; the
// legacy export_state_interfaces() path leaves the value at the driver's NaN.
TEST_F(WaveshareServosJazzyApi, exported_state_interface_honors_initial_value)
{
  std::vector<Joint> joints = bench_joints();
  joints[1].state_interfaces[3] = state("temperature", "21.5");
  InitializedSystem initialized(bench_description(joints));
  ASSERT_EQ(initialized.state_id(), State::PRIMARY_STATE_UNCONFIGURED);

  bool found = false;
  for (const auto & handle : initialized.system().export_state_interfaces()) {
    const auto value = handle->get_optional();
    ASSERT_TRUE(value.has_value()) << handle->get_name();
    if (handle->get_name() == "joint2/temperature") {
      found = true;
      EXPECT_EQ(*value, 21.5);
    } else {
      EXPECT_TRUE(std::isnan(*value)) << handle->get_name() << " = " << *value;
    }
  }
  EXPECT_TRUE(found);
  EXPECT_FALSE(process_has_serial_port_open());
}

// Phase 1 item 3: the driver logs through get_logger(), the logger the ResourceManager gives the
// component ("<resource manager logger>.hardware_component.system.<ros2_control name>"), and no
// longer through rclcpp::get_logger("waveshare_servos").
TEST_F(WaveshareServosJazzyApi, on_init_logs_through_the_component_logger)
{
  const std::string component_logger =
    std::string(kRmLogger) + ".hardware_component.system." + kExampleName;
  {
    auto params = resource_manager_params(example_description());
    hardware_interface::ResourceManager rm(params, false);
    ASSERT_TRUE(rm.load_and_initialize_components(params));
  }
  {
    std::vector<Joint> joints = example_joints();
    joints[1].type = "position";
    auto params = resource_manager_params(example_description(joints));
    hardware_interface::ResourceManager rm(params, false);
    ASSERT_FALSE(rm.load_and_initialize_components(params));
  }

  std::vector<std::string> driver_loggers;
  for (const auto & record : logs_.records()) {
    if (
      record.message.find("position commands clamped") != std::string::npos ||
      record.message.find("wrong type, it should be vel or pos") != std::string::npos)
    {
      driver_loggers.push_back(record.logger);
    }
  }
  // joint1 and joint2 clamp lines from the valid description, joint1 clamp line and the FATAL
  // from the rejected one
  EXPECT_THAT(
    driver_loggers,
    ElementsAre(component_logger, component_logger, component_logger, component_logger));
}
