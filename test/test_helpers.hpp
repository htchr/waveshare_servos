// Shared helpers for the waveshare_servos tests: URDF building, log capture and plugin loading.
//
// Everything here is inline rather than in an anonymous namespace: an anonymous-namespace function
// a translation unit does not call produces -Wunused-function under the package-wide -Wall -Wextra.

#ifndef TEST_HELPERS_HPP_
#define TEST_HELPERS_HPP_

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <ostream>
#include <string>
#include <vector>

#include "hardware_interface/component_parser.hpp"
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
#include "test_support.hpp"

namespace waveshare_servos_test
{

inline constexpr char kPlugin[] = "waveshare_servos/WaveshareServos";
inline constexpr char kRmLogger[] = "test_rm";

// ---------------------------------------------------------------------------------------------
// URDF building

// The seven <joint><param> names the driver knows, as strings so a test can declare a malformed
// value. "" means "emit no <param> at all"; kEmptyParam means "emit the element with no text", the
// declared-but-empty case every parameter has to reject on its own.
//
// The five members after state_interfaces were appended, never inserted: all four builders below
// use positional aggregate initialization, so a member added in the middle would silently bind a
// std::vector<std::string> to a std::string.
struct Joint
{
  std::string name;
  std::string id;
  std::string type;
  std::string offset;  // no <param name="offset"> when empty
  std::vector<std::string> command_interfaces;  // XML fragments
  std::vector<std::string> state_interfaces;  // XML fragments
  std::string inverted;
  std::string max_speed;
  std::string max_accel;
  std::string unwrap;
  // raw <param> XML for names the driver does not know, emitted after the seven it does
  std::string extra_params;
};

// the sentinel that makes joint_xml() emit <param name="x"></param>
inline constexpr char kEmptyParam[] = "<empty>";

// <command_interface>, with min/max params when given
inline std::string command(
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

// <state_interface>, with an initial_value param when given and a data_type when given.
//
// data_type is an XML ATTRIBUTE, never a <param> (component_parser.cpp:237-250,433), and it
// defaults to "double" when it is left out.
inline std::string state(
  const std::string & name, const std::string & initial_value = "",
  const std::string & data_type = "")
{
  std::string opening = "<state_interface name=\"" + name + "\"";
  if (!data_type.empty()) {
    opening += " data_type=\"" + data_type + "\"";
  }
  if (initial_value.empty()) {
    return opening + "/>";
  }
  return opening + "><param name=\"initial_value\">" + initial_value + "</param>" +
         "</state_interface>";
}

// the four state interfaces the example xacro declares
inline std::vector<std::string> servo_states()
{
  return {state("position"), state("velocity"), state("effort"), state("temperature")};
}

// the four it declared before `effort` (N m) replaced the deprecated `torque` alias (kg cm, D2);
// the bench description of the HIL harness still carries these, so the alias stays exercised
inline std::vector<std::string> legacy_servo_states()
{
  return {state("position"), state("velocity"), state("torque"), state("temperature")};
}

// joint1/joint2 of description/ros2_control/example.ros2_control.xacro
inline Joint example_position_joint(const std::string & name, const std::string & id)
{
  return Joint{
    name, id, "pos", "1.570796",
    {command("position", "-1.570796", "1.570796"), command("velocity", "-9.2", "9.2")},
    servo_states(), "", "", "", "", ""};
}

// joint3/joint4 of the example xacro. Neither is `inverted`: the example's URDF is a serial
// chain, not a mirrored wheel pair (see the comment on joint4 in example.ros2_control.xacro).
inline Joint example_velocity_joint(const std::string & name, const std::string & id)
{
  return Joint{
    name, id, "vel", "", {command("velocity", "-9.2", "9.2")}, servo_states(), "", "", "", "",
    ""};
}

// the four joints of the example xacro, same params, offsets and limits. Four, not three,
// because the reference bench carries four servos (ids 1..4) and the shipped example drives all
// of them; joint3 and joint4 are the wheels.
inline std::vector<Joint> example_joints()
{
  return {
    example_position_joint("joint1", "1"), example_position_joint("joint2", "2"),
    example_velocity_joint("joint3", "3"), example_velocity_joint("joint4", "4")};
}

// the four servos on the bench: two position joints and two wheels, velocity without limits
inline std::vector<Joint> bench_joints()
{
  std::vector<Joint> joints;
  for (const std::string id : {"1", "2"}) {
    joints.push_back(
      Joint{
        "joint" + id, id, "pos", "1.570796",
        {command("position", "-1.570796", "1.570796"), command("velocity")}, servo_states(),
        "", "", "", "", ""});
  }
  for (const std::string id : {"3", "4"}) {
    joints.push_back(
      Joint{
        "joint" + id, id, "vel", "", {command("velocity")}, servo_states(), "", "", "", "",
        ""});
  }
  return joints;
}

// one <param>, or nothing when the value is empty; kEmptyParam emits the element with no text
inline std::string joint_param(const std::string & name, const std::string & value)
{
  if (value.empty()) {
    return "";
  }
  if (value == kEmptyParam) {
    return "<param name=\"" + name + "\"></param>";
  }
  return "<param name=\"" + name + "\">" + value + "</param>";
}

inline std::string joint_xml(const Joint & joint)
{
  std::string xml = "<joint name=\"" + joint.name + "\">";
  xml += joint_param("id", joint.id);
  xml += joint_param("type", joint.type);
  xml += joint_param("offset", joint.offset);
  xml += joint_param("inverted", joint.inverted);
  xml += joint_param("max_speed", joint.max_speed);
  xml += joint_param("max_accel", joint.max_accel);
  xml += joint_param("unwrap", joint.unwrap);
  xml += joint.extra_params;
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
inline constexpr char kUrdfJoint4[] =
  R"(
  <link name="wheel4_link"/>
  <joint name="joint4" type="continuous">
    <parent link="base_link"/>
    <child link="wheel4_link"/>
    <axis xyz="0 0 1"/>
  </joint>
)";

inline std::string robot_description(
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

// the port and baudrate the example xacro declares, both at the driver defaults
inline constexpr char kExampleHardwareParams[] =
  R"(
  <param name="port">/dev/ttyACM0</param>
  <param name="baudrate">1000000</param>
)";

// the demo params the example xacro used to carry: the driver knows none of them, so they pin the
// warn-and-ignore behaviour for unknown hardware params
inline constexpr char kLegacyHardwareParams[] =
  R"(
  <param name="example_param_hw_start_duration_sec">0</param>
  <param name="example_param_hw_stop_duration_sec">3.0</param>
  <param name="example_param_hw_slowdown">100</param>
)";
inline constexpr char kExampleName[] = "example_ws_ros2_control";
inline constexpr char kBenchName[] = "four_servos";

inline std::string example_description(
  const std::vector<Joint> & joints = example_joints(),
  const std::string & extra_components = "",
  const std::string & extra_hardware_params = "")
{
  return robot_description(
    kExampleName, joints, std::string(kExampleHardwareParams) + extra_hardware_params,
    extra_components);
}

inline std::string bench_description(const std::vector<Joint> & joints = bench_joints())
{
  return robot_description(kBenchName, joints);
}

inline const std::vector<std::string> kServoStateNames = {
  "position", "velocity", "effort", "temperature"};

inline std::vector<std::string> state_keys(const std::vector<std::string> & joint_names)
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
// The rejection table rows: one broken description, one expected FATAL.

struct Rejection
{
  std::string name;
  std::function<void(std::vector<Joint> &)> break_joints;
  std::string fatal_message;
  std::string hardware_params;  // "" for every joint-level row
};

inline void PrintTo(const Rejection & rejection, std::ostream * os)
{
  *os << rejection.name;
}

// ---------------------------------------------------------------------------------------------
// Log capture: every rcutils log record is kept and then passed on to the console handler.

struct LogRecord
{
  int severity;
  std::string logger;
  std::string message;
};

inline std::mutex g_log_mutex;
inline std::vector<LogRecord> g_log_records;
inline rcutils_logging_output_handler_t g_console_handler = nullptr;

inline void capture_log(
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
// Plugin loading

inline hardware_interface::ResourceManagerParams resource_manager_params(const std::string & urdf)
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
  uint8_t state_id_ = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
};

}  // namespace waveshare_servos_test

#endif  // TEST_HELPERS_HPP_
