// Load-time tests: the plugin is loaded and initialized from a URDF string but never configured,
// so no serial port is opened.

#include <gmock/gmock.h>

#include <cmath>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rcutils/logging.h"
#include "test_helpers.hpp"

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

// Per-name declarations, never a using-directive: cpplint's build/namespaces rule forbids
// using-directives everywhere, not only in headers.
using waveshare_servos_test::InitializedSystem;
using waveshare_servos_test::Joint;
using waveshare_servos_test::LogCapture;
using waveshare_servos_test::Rejection;
using waveshare_servos_test::bench_description;
using waveshare_servos_test::bench_joints;
using waveshare_servos_test::command;
using waveshare_servos_test::example_description;
using waveshare_servos_test::example_joints;
using waveshare_servos_test::example_velocity_joint;
using waveshare_servos_test::joint_param;
using waveshare_servos_test::joint_xml;
using waveshare_servos_test::kUrdfJoint4;
using waveshare_servos_test::kBenchName;
using waveshare_servos_test::kEmptyParam;
using waveshare_servos_test::kExampleName;
using waveshare_servos_test::kLegacyHardwareParams;
using waveshare_servos_test::kPlugin;
using waveshare_servos_test::kRmLogger;
using waveshare_servos_test::legacy_servo_states;
using waveshare_servos_test::process_has_serial_port_open;
using waveshare_servos_test::resource_manager_params;
using waveshare_servos_test::robot_description;
using waveshare_servos_test::state;
using waveshare_servos_test::state_keys;

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

TEST_F(WaveshareServosLoad, example_four_joint_system_loads_unconfigured)
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

  // on_init reports the position command limits it parsed, for the position joints only
  const auto info = logs_.messages(RCUTILS_LOG_SEVERITY_INFO);
  EXPECT_THAT(
    info, Contains(HasSubstr("joint 'joint1' position commands clamped to [-1.5708, 1.5708] rad")));
  EXPECT_THAT(
    info, Contains(HasSubstr("joint 'joint2' position commands clamped to [-1.5708, 1.5708] rad")));
  EXPECT_THAT(info, Not(Contains(HasSubstr("joint 'joint3' position commands clamped"))));
  EXPECT_THAT(info, Not(Contains(HasSubstr("joint 'joint4' position commands clamped"))));
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

// Interfaces are exported joint by joint in description order, the order that
// list_hardware_components and /dynamic_joint_states show.
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

// Only <joint> interfaces are served: a <gpio> or <sensor> in the block makes the load fail,
// instead of exporting handles that nothing updates.
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

// A command interface listed twice is exported twice, so the resource manager logs "already
// existing key" (and refuses the load when joints follow) instead of a silent merge.
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

std::vector<Rejection> rejections()
{
  return {
    // `id` is required: a whole number 1..253, unique within the <ros2_control> block.
    {"id_missing",
      [](std::vector<Joint> & j) {j[1].id = "";},
      "joint 'joint2' has no <param name=\"id\">; every joint needs the bus id of its servo "
      "(1..253)", ""},
    {"id_empty",
      [](std::vector<Joint> & j) {j[1].id = kEmptyParam;},
      "joint 'joint2' has no <param name=\"id\">", ""},
    {"id_not_a_number",
      [](std::vector<Joint> & j) {j[0].id = "one";},
      "joint 'joint1' has an id that is not a whole number: 'one'", ""},
    {"id_has_trailing_text",
      [](std::vector<Joint> & j) {j[0].id = "1.5";},
      "joint 'joint1' has an id that is not a whole number: '1.5'", ""},
    {"id_zero",
      [](std::vector<Joint> & j) {j[2].id = "0";},
      "joint 'joint3' has id 0, outside the range 1..253", ""},
    // 254 is the broadcast address the sync writes use and 255 is the packet header byte
    {"id_is_the_broadcast_id",
      [](std::vector<Joint> & j) {j[2].id = "254";},
      "joint 'joint3' has id 254, outside the range 1..253; 254 is the broadcast id the sync "
      "writes use and 255 is the packet header byte", ""},
    {"id_too_large",
      [](std::vector<Joint> & j) {j[1].id = "300";},
      "joint 'joint2' has id 300, outside the range 1..253", ""},
    {"id_duplicate",
      [](std::vector<Joint> & j) {j[2].id = "1";},
      "joint 'joint3' has id 1, which joint 'joint1' already uses; ids must be unique within a "
      "<ros2_control> block", ""},
    // stoi_generic parses through std::stol, which accepts '+' and leading zeros, so the
    // message prints the parsed number, not the declared text.
    {"id_duplicate_written_differently",
      [](std::vector<Joint> & j) {j[2].id = "+001";},
      "joint 'joint3' has id 1, which joint 'joint1' already uses; ids must be unique within a "
      "<ros2_control> block", ""},
    // State interfaces are free-form (any subset, any order). Rejected: an unknown name, a
    // data_type other than double, and a duplicate.
    {"unknown_state_interface",
      [](std::vector<Joint> & j) {j[0].state_interfaces.push_back(state("torqe"));},
      "joint 'joint1' declares the unsupported state interface 'torqe'; supported names are "
      "position, velocity, effort, current, voltage, temperature, load, status and the deprecated "
      "torque", ""},
    // register 66 (ReadMove) is deliberately not exported: adding an interface later is purely
    // additive, removing one is not
    {"moving_state_interface_is_not_served",
      [](std::vector<Joint> & j) {j[2].state_interfaces.push_back(state("moving"));},
      "joint 'joint3' declares the unsupported state interface 'moving'", ""},
    {"duplicate_state_interface",
      [](std::vector<Joint> & j) {j[1].state_interfaces.push_back(state("position"));},
      "joint 'joint2' declares the state interface 'position' more than once", ""},
    {"duplicate_torque_alias",
      [](std::vector<Joint> & j) {
        j[0].state_interfaces = {state("torque"), state("torque")};
      },
      "joint 'joint1' declares the state interface 'torque' more than once", ""},
    // data_type is an XML attribute defaulting to "double"; without this check the description
    // loads and set_state<double> throws out of read() much later
    {"state_interface_data_type_not_double",
      [](std::vector<Joint> & j) {j[1].state_interfaces[0] = state("position", "", "int16");},
      "joint 'joint2' declares the state interface 'position' with data_type 'int16'; only "
      "'double' is supported", ""},
    {"no_command_interface",
      [](std::vector<Joint> & j) {j[2].command_interfaces.clear();},
      "a joint does not have a command interfaces", ""},
    {"effort_command_interface",
      [](std::vector<Joint> & j) {j[2].command_interfaces = {command("effort", "-1.0", "1.0")};},
      "a joint is using a command interface that isn't position or velocity", ""},
    // 'position' is the interface name, not a `type` value (pos | vel): the likely typo.
    {"type_not_pos_or_vel",
      [](std::vector<Joint> & j) {j[1].type = "position";},
      "joint 'joint2' has type 'position'; it must be 'pos' or 'vel'", ""},
    {"type_empty",
      [](std::vector<Joint> & j) {j[1].type = kEmptyParam;},
      "joint 'joint2' has type ''; it must be 'pos' or 'vel'", ""},
    // a wheel (type vel) takes only a velocity command
    {"type_vel_with_a_position_command",
      [](std::vector<Joint> & j) {
        j[2].type = "vel";
        j[2].command_interfaces = {command("position"), command("velocity")};
      },
      "joint 'joint3' has type 'vel' but declares a position command interface; a velocity joint "
      "runs its servo in wheel mode and takes only <command_interface name=\"velocity\">", ""},
    // a velocity command only paces a position joint's move; it cannot replace the goal
    {"type_pos_without_a_position_command",
      [](std::vector<Joint> & j) {
        j[0].type = "pos";
        j[0].command_interfaces = {command("velocity")};
      },
      "joint 'joint1' has type 'pos' but declares no position command interface; a position joint "
      "needs <command_interface name=\"position\"> (a velocity command interface only paces the "
      "move)", ""},
    {"position_min_not_a_number",
      [](std::vector<Joint> & j) {
        j[1].command_interfaces[0] = command("position", "abc", "1.570796");
      },
      "joint 'joint2' has a position min or max that is not a number", ""},
    {"position_max_not_a_number",
      [](std::vector<Joint> & j) {
        j[0].command_interfaces[0] = command("position", "-1.570796", "half_pi");
      },
      "joint 'joint1' has a position min or max that is not a number", ""},
    {"position_min_greater_than_max",
      [](std::vector<Joint> & j) {
        j[1].command_interfaces[0] = command("position", "2.0", "1.570796");
      },
      "joint 'joint2' has a position min greater than its max", ""},
    // hardware_interface::stod rejects "nan" and "inf" outright, so a non-finite limit is reported
    // as an unparsable one and can no longer reach the min-greater-than-max check below it
    {"position_min_nan",
      [](std::vector<Joint> & j) {
        j[1].command_interfaces[0] = command("position", "nan", "1.570796");
      },
      "joint 'joint2' has a position min or max that is not a number", ""},
    {"position_max_nan",
      [](std::vector<Joint> & j) {
        j[0].command_interfaces[0] = command("position", "-1.570796", "nan");
      },
      "joint 'joint1' has a position min or max that is not a number", ""},
    // A bad offset shows only in the ticks the limits map to. joint1 (offset pi/2, limits
    // +-pi/2) is ticks [0, 2048]; each row below moves that window out of [0, 4095].
    {"offset_not_a_number",
      [](std::vector<Joint> & j) {j[0].offset = "half_pi";},
      "joint 'joint1' has an offset that is not a finite number: 'half_pi'", ""},
    {"offset_infinite",
      [](std::vector<Joint> & j) {j[0].offset = "inf";},
      "joint 'joint1' has an offset that is not a finite number: 'inf'", ""},
    {"offset_puts_the_lower_limit_below_tick_zero",
      [](std::vector<Joint> & j) {j[0].offset = "0.0";},
      "map to servo ticks [-1024, 1024], outside the servo's single-turn range [0, 4095]", ""},
    {"offset_puts_the_upper_limit_past_the_last_tick",
      [](std::vector<Joint> & j) {j[0].offset = "4.8";},
      "map to servo ticks [2105, 4153], outside the servo's single-turn range [0, 4095]", ""},
    {"offset_out_of_range_with_only_a_min",
      [](std::vector<Joint> & j) {
        j[0].command_interfaces[0] = command("position", "-1.570796", "");
        j[0].offset = "0.0";
      },
      "position command min -1.5708 rad with offset 0.0000 rad and inverted=false maps to servo "
      "tick -1024", ""},
    // with no finite limit at all it is joint zero itself that has to be reachable. joint3 has to
    // be made a position joint first: as a wheel it is not range checked at all.
    {"offset_out_of_range_without_limits",
      [](std::vector<Joint> & j) {
        j[2].type = "pos";
        j[2].command_interfaces = {command("position")};
        j[2].offset = "-0.1";
      },
      "maps its zero position to servo tick -65, outside the servo's single-turn range [0, 4095]",
      ""},
    // std::lround outside long's range is unspecified (LONG_MIN here); the row checks that the
    // printed tick keeps the sign of the offset.
    {"offset_past_the_range_of_a_long",
      [](std::vector<Joint> & j) {
        j[2].type = "pos";
        j[2].command_interfaces = {command("position")};
        j[2].offset = "1e300";
      },
      "maps its zero position to servo tick 4611686018427387904, outside the servo's single-turn "
      "range [0, 4095]", ""},
    // only 'true'/'false'; '1' and '0' are refused on purpose
    {"inverted_not_a_bool",
      [](std::vector<Joint> & j) {j[0].inverted = "yes";},
      "joint 'joint1' has inverted='yes'; it must be 'true' or 'false'", ""},
    // max_speed is rad/s in the joint frame. 0 is refused: std::clamp(speed, 1.0, 0.0) on the
    // write path would be undefined behaviour.
    {"max_speed_not_a_number",
      [](std::vector<Joint> & j) {j[0].max_speed = "fast";},
      "joint 'joint1' has a max_speed that is not a finite number: 'fast'", ""},
    {"max_speed_zero",
      [](std::vector<Joint> & j) {j[0].max_speed = "0";},
      "joint 'joint1' has max_speed 0 rad/s; it must be greater than 0", ""},
    {"max_speed_negative",
      [](std::vector<Joint> & j) {j[0].max_speed = "-1.0";},
      "it must be greater than 0", ""},
    {"max_speed_below_one_count",
      [](std::vector<Joint> & j) {j[0].max_speed = "0.0005";},
      "which is less than one encoder step per second", ""},
    // max_accel is rad/s^2. 0 is the explicit opt-out ("no acceleration limit"); a small positive
    // value that rounds to 0 counts would mean the same thing by accident, so it is refused.
    {"max_accel_not_a_number",
      [](std::vector<Joint> & j) {j[0].max_accel = "quick";},
      "joint 'joint1' has a max_accel that is not a finite number: 'quick'", ""},
    {"max_accel_negative",
      [](std::vector<Joint> & j) {j[0].max_accel = "-1.0";},
      "it must be 0 (no acceleration limit) or greater", ""},
    {"max_accel_rounds_to_zero_counts",
      [](std::vector<Joint> & j) {j[0].max_accel = "0.05";},
      "rounds to 0 acceleration-register counts", ""},
    // unwrap on a position joint is FATAL, not ignored: it would break the goal-speed pacing,
    // the limit check and the activation seed
    {"unwrap_not_a_bool",
      [](std::vector<Joint> & j) {j[2].unwrap = "yes";},
      "joint 'joint3' has unwrap='yes'; it must be 'true' or 'false'", ""},
    {"unwrap_true_on_a_pos_joint",
      [](std::vector<Joint> & j) {j[0].unwrap = "true";},
      "joint 'joint1' has unwrap=true with type pos; only a vel joint has a multi-turn position",
      ""},
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
  auto params =
    resource_manager_params(example_description(joints, "", GetParam().hardware_params));
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
// The <hardware><param> block, its validation and the resolved-configuration INFO line.

namespace
{

// The configuration line with no hardware param. Doubles print with %.7g (%g would round
// 0.8825985 to 0.882599).
constexpr char kDefaultConfigurationLine[] =
  "bus configuration: port '/dev/ttyACM0', 1000000 baud, protocol 'sms_sts', io timeout 5 ms, "
  "3 ping attempt(s), drop a servo after 50 consecutive read failures, allow_missing_servos false, "
  "feedback_mode 'auto', 4096 encoder steps per revolution, 0.006 A per current count, "
  "0.8825985 N m/A";

// The "bus configuration" INFO lines in log order: one per successful load, none on a reject.
std::vector<std::string> configuration_lines(const LogCapture & logs)
{
  std::vector<std::string> lines;
  for (const auto & message : logs.messages(RCUTILS_LOG_SEVERITY_INFO)) {
    if (message.rfind("bus configuration: ", 0) == 0) {
      lines.push_back(message);
    }
  }
  return lines;
}

// The io_timeout_ms WARNs (floor and ceiling) in log order. Selected by the parameter name,
// so a case can require "exactly one" without naming which.
std::vector<std::string> timeout_warnings(const LogCapture & logs)
{
  std::vector<std::string> selected;
  for (const auto & message : logs.messages(RCUTILS_LOG_SEVERITY_WARN)) {
    if (message.find("io_timeout_ms") != std::string::npos) {
      selected.push_back(message);
    }
  }
  return selected;
}

// The INFO for a defaulted timeout below the floor: it is raised, so a stock description on a
// large bus keeps sync read.
std::vector<std::string> raise_infos(const LogCapture & logs)
{
  std::vector<std::string> selected;
  for (const auto & message : logs.messages(RCUTILS_LOG_SEVERITY_INFO)) {
    if (message.rfind("io_timeout_ms raised", 0) == 0) {
      selected.push_back(message);
    }
  }
  return selected;
}

// `count` velocity joints, ids 1..count; the floor exceeds the 5 ms default only from 10 joints
// (6 ms; 9 ms at 20). Joints past joint4 are added to the URDF as continuous joints.
std::string many_joint_description(size_t count, const std::string & declared)
{
  std::string urdf = std::string(ros2_control_test_assets::urdf_head) + kUrdfJoint4;
  std::string block;
  for (size_t i = 1; i <= count; i++) {
    const std::string name = "joint" + std::to_string(i);
    if (i > 4) {
      urdf += "<link name=\"" + name + "_link\"/>\n<joint name=\"" + name +
        "\" type=\"continuous\">\n<parent link=\"base_link\"/>\n<child link=\"" + name +
        "_link\"/>\n<axis xyz=\"0 0 1\"/>\n</joint>\n";
    }
    block += joint_xml(example_velocity_joint(name, std::to_string(i)));
  }
  urdf += "<ros2_control name=\"" + std::string(kExampleName) + "\" type=\"system\">\n<hardware>\n";
  urdf += std::string("<plugin>") + kPlugin + "</plugin>\n" + declared + "</hardware>\n" + block;
  return urdf + "</ros2_control>\n" + ros2_control_test_assets::urdf_tail;
}

// The "is not used by this driver; ignoring it" WARNs, in log order.
std::vector<std::string> unknown_parameter_warnings(const LogCapture & logs)
{
  std::vector<std::string> warnings;
  for (const auto & message : logs.messages(RCUTILS_LOG_SEVERITY_WARN)) {
    if (message.find("is not used by this driver") != std::string::npos) {
      warnings.push_back(message);
    }
  }
  return warnings;
}

std::string hardware_param(const std::string & name, const std::string & value)
{
  return "<param name=\"" + name + "\">" + value + "</param>\n";
}

// The three hardware-param message templates, written independently of params::message().
std::string empty_param(const std::string & name, const std::string & expected)
{
  return "hardware parameter '" + name + "' is empty; expected " + expected;
}

std::string malformed(
  const std::string & name, const std::string & value, const std::string & expected)
{
  return "hardware parameter '" + name + "' is '" + value + "', which is not " + expected;
}

std::string out_of_range(
  const std::string & name, const std::string & value, const std::string & expected)
{
  return "hardware parameter '" + name + "' is '" + value + "', which is out of range; expected " +
         expected;
}

// A rejection row that changes only the <hardware> block; the joints are left alone.
Rejection hw_reject(
  const std::string & name, const std::string & param, const std::string & value,
  const std::string & fatal_message)
{
  return Rejection{
    name, [](std::vector<Joint> &) {}, fatal_message, hardware_param(param, value)};
}

std::vector<Rejection> hw_param_rejections()
{
  // The lower bound is 2 ms, and the message gives the reason in terms of the batched read.
  const std::string timeout =
    "an integer between 2 and 1000 (milliseconds); 1 ms is not enough for a batched feedback "
    "read, which needs about 0.48 ms plus 0.29 ms per servo";
  const std::string modes = "a known feedback mode; expected 'auto', 'sync_read' or 'per_servo'";
  const std::string attempts = "an integer between 1 and 10";
  const std::string fails = "an integer between 1 and 1000000";
  const std::string steps = "an even integer between 2 and 32768 (encoder steps per revolution)";
  const std::string current = "a number greater than 0 and at most 1 (amperes per current count)";
  const std::string torque = "a number greater than 0 and at most 100 (newton metres per ampere)";
  const std::string unsupported_rate =
    "; the servo library maps only 9600, 19200, 38400, 57600, 115200, 500000 and 1000000, and "
    "silently falls back to 115200 for anything else";
  const auto baudrate_set = [&unsupported_rate](const std::string & value) {
      return "hardware parameter 'baudrate' is '" + value + "'" + unsupported_rate;
    };
  return {
    hw_reject(
      "port_empty", "port", "",
      empty_param("port", "a device path such as '/dev/ttyACM0'")),
    hw_reject("baudrate_empty", "baudrate", "", empty_param("baudrate", "an integer")),
    hw_reject(
      "baudrate_not_an_integer", "baudrate", "1M", malformed("baudrate", "1M", "an integer")),
    hw_reject(
      "baudrate_fractional", "baudrate", "1000000.0",
      malformed("baudrate", "1000000.0", "an integer")),
    hw_reject("baudrate_unsupported_rate", "baudrate", "230400", baudrate_set("230400")),
    hw_reject("baudrate_negative", "baudrate", "-1000000", baudrate_set("-1000000")),
    hw_reject("baudrate_zero", "baudrate", "0", baudrate_set("0")),
    hw_reject("protocol_empty", "protocol", "", empty_param("protocol", "'sms_sts'")),
    hw_reject(
      "protocol_scscl_not_implemented", "protocol", "scscl",
      "hardware parameter 'protocol' is 'scscl'; only 'sms_sts' is implemented, the SCS/SCSCL "
      "series is not supported yet"),
    hw_reject(
      "protocol_unknown", "protocol", "feetech",
      malformed("protocol", "feetech", "a known protocol; expected 'sms_sts'")),
    // A misspelt feedback_mode is refused, not replaced by a default: the parameter exists to
    // pin the read path.
    hw_reject(
      "feedback_mode_empty", "feedback_mode", "",
      empty_param("feedback_mode", "'auto', 'sync_read' or 'per_servo'")),
    hw_reject(
      "feedback_mode_unknown", "feedback_mode", "syncread",
      malformed("feedback_mode", "syncread", modes)),
    hw_reject(
      "io_timeout_ms_zero", "io_timeout_ms", "0", out_of_range("io_timeout_ms", "0", timeout)),
    // 1 ms is refused: it fails most sync reads and passes some stale ones as fresh.
    // See docs/bus-timing.md, "Transaction timeout".
    hw_reject(
      "io_timeout_ms_one", "io_timeout_ms", "1", out_of_range("io_timeout_ms", "1", timeout)),
    hw_reject(
      "io_timeout_ms_above_max", "io_timeout_ms", "1001",
      out_of_range("io_timeout_ms", "1001", timeout)),
    hw_reject(
      "io_timeout_ms_not_an_integer", "io_timeout_ms", "20ms",
      malformed("io_timeout_ms", "20ms", timeout)),
    hw_reject(
      "ping_attempts_zero", "ping_attempts", "0", out_of_range("ping_attempts", "0", attempts)),
    hw_reject(
      "ping_attempts_above_max", "ping_attempts", "11",
      out_of_range("ping_attempts", "11", attempts)),
    hw_reject(
      "ping_attempts_not_an_integer", "ping_attempts", "three",
      malformed("ping_attempts", "three", attempts)),
    hw_reject(
      "max_read_fails_zero", "max_read_fails", "0", out_of_range("max_read_fails", "0", fails)),
    hw_reject(
      "max_read_fails_above_max", "max_read_fails", "1000001",
      out_of_range("max_read_fails", "1000001", fails)),
    hw_reject(
      "max_read_fails_not_an_integer", "max_read_fails", "50.0",
      malformed("max_read_fails", "50.0", fails)),
    hw_reject(
      "allow_missing_servos_numeric_one", "allow_missing_servos", "1",
      malformed("allow_missing_servos", "1", "'true' or 'false'")),
    hw_reject(
      "allow_missing_servos_not_a_bool", "allow_missing_servos", "yes",
      malformed("allow_missing_servos", "yes", "'true' or 'false'")),
    hw_reject("encoder_steps_one", "encoder_steps", "1", out_of_range("encoder_steps", "1", steps)),
    hw_reject(
      "encoder_steps_above_max", "encoder_steps", "32769",
      out_of_range("encoder_steps", "32769", steps)),
    hw_reject(
      "encoder_steps_odd", "encoder_steps", "4097", out_of_range("encoder_steps", "4097", steps)),
    hw_reject(
      "encoder_steps_not_an_integer", "encoder_steps", "4k",
      malformed("encoder_steps", "4k", steps)),
    hw_reject(
      "current_per_count_a_zero", "current_per_count_a", "0",
      out_of_range("current_per_count_a", "0", current)),
    hw_reject(
      "current_per_count_a_negative", "current_per_count_a", "-0.006",
      out_of_range("current_per_count_a", "-0.006", current)),
    hw_reject(
      "current_per_count_a_above_max", "current_per_count_a", "1.5",
      out_of_range("current_per_count_a", "1.5", current)),
    hw_reject(
      "current_per_count_a_not_a_number", "current_per_count_a", "six milliamps",
      malformed("current_per_count_a", "six milliamps", current)),
    hw_reject(
      "torque_constant_nm_per_a_zero", "torque_constant_nm_per_a", "0",
      out_of_range("torque_constant_nm_per_a", "0", torque)),
    hw_reject(
      "torque_constant_nm_per_a_negative", "torque_constant_nm_per_a", "-0.8825985",
      out_of_range("torque_constant_nm_per_a", "-0.8825985", torque)),
    hw_reject(
      "torque_constant_nm_per_a_not_a_number", "torque_constant_nm_per_a", "nine",
      malformed("torque_constant_nm_per_a", "nine", torque)),
  };
}

}  // namespace

TEST_F(WaveshareServosLoad, hardware_params_default_to_the_compiled_in_constants)
{
  // no <param> in <hardware>: every value is the compiled-in default
  auto params = resource_manager_params(robot_description(kExampleName, example_joints()));
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));

  EXPECT_THAT(configuration_lines(logs_), ElementsAre(kDefaultConfigurationLine));
  // the torque constant is 9.0 kgf cm/A expressed in N m/A and must never print as 0.882599
  EXPECT_THAT(std::string(kDefaultConfigurationLine), HasSubstr("0.8825985 N m/A"));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_THAT(unknown_parameter_warnings(logs_), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

TEST_F(WaveshareServosLoad, every_mapped_baudrate_is_accepted)
{
  // the seven rates SCSerial::setBaudRate maps; anything else silently becomes 115200 there
  for (const std::string rate :
    {"9600", "19200", "38400", "57600", "115200", "500000", "1000000"})
  {
    SCOPED_TRACE(rate);
    auto params = resource_manager_params(
      example_description(example_joints(), "", hardware_param("baudrate", rate)));
    hardware_interface::ResourceManager rm(params, false);
    EXPECT_TRUE(rm.load_and_initialize_components(params));
    EXPECT_THAT(
      logs_.messages(RCUTILS_LOG_SEVERITY_INFO),
      Contains(HasSubstr("bus configuration: port '/dev/ttyACM0', " + rate + " baud, ")));
  }
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

TEST_F(WaveshareServosLoad, explicit_hardware_params_appear_in_the_configuration_line)
{
  std::string declared = hardware_param("port", "/dev/ttyUSB1");
  declared += hardware_param("baudrate", "115200");
  declared += hardware_param("protocol", "sms_sts");
  // 7, not the default 5, so the line proves the value was used; 7 is between the 4-joint floor
  // (3 ms) and the 8 ms ceiling, so no timeout WARN fires.
  declared += hardware_param("io_timeout_ms", "7");
  declared += hardware_param("ping_attempts", "1");
  declared += hardware_param("max_read_fails", "7");
  declared += hardware_param("allow_missing_servos", "true");
  declared += hardware_param("feedback_mode", "sync_read");
  declared += hardware_param("encoder_steps", "1024");
  declared += hardware_param("current_per_count_a", "0.0065");
  declared += hardware_param("torque_constant_nm_per_a", "1.5");
  auto params = resource_manager_params(example_description(example_joints(), "", declared));
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));

  // exactly one line, and every one of the eleven values is the declared one. A repeated <param>
  // keeps the last value with no diagnostic from the parser, so this line is the user's only check
  EXPECT_THAT(
    configuration_lines(logs_),
    ElementsAre(
      "bus configuration: port '/dev/ttyUSB1', 115200 baud, protocol 'sms_sts', io timeout 7 ms, "
      "1 ping attempt(s), drop a servo after 7 consecutive read failures, allow_missing_servos "
      "true, feedback_mode 'sync_read', 1024 encoder steps per revolution, 0.0065 A per current "
      "count, 1.5 N m/A"));
  // All eleven names must also be in kKnownHardwareParams, or the driver warns that it ignores
  // a parameter that it in fact uses.
  EXPECT_THAT(unknown_parameter_warnings(logs_), IsEmpty());
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

TEST_F(WaveshareServosLoad, protocol_is_normalised_to_lower_case)
{
  auto params = resource_manager_params(
    example_description(example_joints(), "", hardware_param("protocol", "SMS_STS")));
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));

  EXPECT_THAT(configuration_lines(logs_), ElementsAre(HasSubstr("protocol 'sms_sts'")));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

TEST_F(WaveshareServosLoad, feedback_mode_is_normalised_to_lower_case)
{
  auto params = resource_manager_params(
    example_description(example_joints(), "", hardware_param("feedback_mode", "Per_Servo")));
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));

  EXPECT_THAT(configuration_lines(logs_), ElementsAre(HasSubstr("feedback_mode 'per_servo'")));
  // The name must also be in kKnownHardwareParams, or the load warns that it is ignored.
  EXPECT_THAT(unknown_parameter_warnings(logs_), IsEmpty());
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

// ---------------------------------------------------------------------------------------------
// io_timeout_ms against the sync-read floor and ceiling. See docs/bus-timing.md, "Timeout floor".

// The default must be at or above the floor and below the ceiling on the four-servo bench.
TEST_F(WaveshareServosLoad, the_default_io_timeout_is_five_milliseconds)
{
  auto params = resource_manager_params(robot_description(kExampleName, example_joints()));
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));

  EXPECT_THAT(configuration_lines(logs_), ElementsAre(HasSubstr("io timeout 5 ms")));
  // 5 ms clears min_io_timeout_ms(4) == 3 and stays under the 8 ms dead-servo ceiling, so a stock
  // four-joint description must load in silence.
  EXPECT_THAT(timeout_warnings(logs_), IsEmpty());
  EXPECT_THAT(raise_infos(logs_), IsEmpty());
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
}

// A user-set value below the floor under 'auto' warns and switches to per-servo reads, the
// one path measured reliable at every setting down to 1 ms.
TEST_F(WaveshareServosLoad, a_timeout_below_the_sync_read_floor_warns_with_the_joint_count)
{
  auto params = resource_manager_params(
    example_description(example_joints(), "", hardware_param("io_timeout_ms", "2")));
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));

  EXPECT_THAT(
    timeout_warnings(logs_),
    ElementsAre(
      "io_timeout_ms 2 is below the 3 ms a sync read of 4 servos needs here; using one feedback "
      "read per servo"));
  // The latch is visible in the configuration line: the user asked for nothing, got 'auto', and
  // the driver reports the transport it will actually use rather than the word it parsed.
  EXPECT_THAT(configuration_lines(logs_), ElementsAre(HasSubstr("feedback_mode 'per_servo'")));
  // A user-set value is never rewritten: it is also what a dead servo costs, the user's choice.
  EXPECT_THAT(configuration_lines(logs_), ElementsAre(HasSubstr("io timeout 2 ms")));
  EXPECT_THAT(raise_infos(logs_), IsEmpty());
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
}

// per_servo is never checked against the floor: one FeedBack waits for one 21-byte reply and
// works at every setting the range allows.
TEST_F(WaveshareServosLoad, feedback_mode_per_servo_is_never_judged_against_the_sync_read_floor)
{
  std::string declared = hardware_param("io_timeout_ms", "2");
  declared += hardware_param("feedback_mode", "per_servo");
  auto params = resource_manager_params(example_description(example_joints(), "", declared));
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));

  EXPECT_THAT(timeout_warnings(logs_), IsEmpty());
  EXPECT_THAT(raise_infos(logs_), IsEmpty());
  EXPECT_THAT(configuration_lines(logs_), ElementsAre(HasSubstr("io timeout 2 ms")));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
}

// sync_read with a user-set value below the floor has no degraded path: the load is refused.
TEST_F(WaveshareServosLoad, feedback_mode_sync_read_refuses_an_io_timeout_below_the_floor)
{
  std::string declared = hardware_param("io_timeout_ms", "2");
  declared += hardware_param("feedback_mode", "sync_read");
  auto params = resource_manager_params(example_description(example_joints(), "", declared));
  hardware_interface::ResourceManager rm(params, false);

  EXPECT_FALSE(rm.load_and_initialize_components(params));
  EXPECT_THAT(
    logs_.messages(RCUTILS_LOG_SEVERITY_FATAL),
    ElementsAre(
      "hardware parameter 'io_timeout_ms' is '2', below the 3 ms a sync read of 4 servos needs; "
      "feedback_mode is 'sync_read', which rules out the per-servo path that would survive it, so "
      "raise io_timeout_ms to at least 3 or use 'auto'"));
  EXPECT_THAT(configuration_lines(logs_), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

// A defaulted value is raised, not demoted: at 10 joints the floor exceeds the 5 ms default,
// and a stock description must keep sync read.
TEST_F(WaveshareServosLoad, a_defaulted_io_timeout_is_raised_to_the_floor_instead_of_demoting)
{
  auto params = resource_manager_params(many_joint_description(10, ""));
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));

  EXPECT_THAT(
    raise_infos(logs_),
    ElementsAre("io_timeout_ms raised from 5 to 6 ms for a sync read of 10 servos"));
  EXPECT_THAT(timeout_warnings(logs_), IsEmpty());
  // The raise is what the driver will use, so it is what the configuration line reports, and the
  // transport the user never chose is still 'auto'.
  EXPECT_THAT(configuration_lines(logs_), ElementsAre(HasSubstr("io timeout 6 ms")));
  EXPECT_THAT(configuration_lines(logs_), ElementsAre(HasSubstr("feedback_mode 'auto'")));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
}

// Above 8 ms the value is expensive, not wrong: a silent servo costs the timeout plus the 2 ms
// drain every cycle, and 100 Hz leaves 10 ms.
TEST_F(WaveshareServosLoad, a_timeout_above_eight_milliseconds_warns_about_a_dead_servo)
{
  auto params = resource_manager_params(
    example_description(example_joints(), "", hardware_param("io_timeout_ms", "20")));
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));

  EXPECT_THAT(timeout_warnings(logs_), ElementsAre(HasSubstr("a 100 Hz loop has 10 ms")));
  // An advisory about cost, not a verdict: the text must say that a slower loop is legitimate.
  EXPECT_THAT(timeout_warnings(logs_), ElementsAre(HasSubstr("legitimate")));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
}

// The ceiling is 8 ms and the check is strictly above it: with a 3 ms floor, 8 is silent and
// 9 warns.
TEST_F(WaveshareServosLoad, the_dead_servo_ceiling_is_eight_milliseconds_not_ten)
{
  auto at_the_ceiling = resource_manager_params(
    example_description(example_joints(), "", hardware_param("io_timeout_ms", "8")));
  hardware_interface::ResourceManager silent(at_the_ceiling, false);
  ASSERT_TRUE(silent.load_and_initialize_components(at_the_ceiling));
  EXPECT_THAT(timeout_warnings(logs_), IsEmpty());

  // The capture accumulates across both loads, so "exactly one" below also re-states that the
  // 8 ms arm contributed nothing.
  auto above_it = resource_manager_params(
    example_description(example_joints(), "", hardware_param("io_timeout_ms", "9")));
  hardware_interface::ResourceManager warned(above_it, false);
  ASSERT_TRUE(warned.load_and_initialize_components(above_it));
  EXPECT_THAT(timeout_warnings(logs_), ElementsAre(HasSubstr("a 100 Hz loop has 10 ms")));
  EXPECT_THAT(raise_infos(logs_), IsEmpty());
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
}

// No ceiling WARN for per_servo: only a burst waits for the whole reply stream; per-servo
// reads pay the timeout only for the silent joint.
TEST_F(WaveshareServosLoad, the_dead_servo_ceiling_is_never_raised_against_the_per_servo_path)
{
  std::string declared = hardware_param("io_timeout_ms", "20");
  declared += hardware_param("feedback_mode", "per_servo");
  auto params = resource_manager_params(example_description(example_joints(), "", declared));
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));

  EXPECT_THAT(timeout_warnings(logs_), IsEmpty());
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
}

// The floor uses the chunk size min(joints, 30), because longer id lists are split.
// 40 joints tells them apart: floor(30) = 12 ms, floor(40) would be 16.
TEST_F(WaveshareServosLoad, the_sync_read_floor_is_measured_against_one_chunk_not_every_joint)
{
  auto params = resource_manager_params(many_joint_description(40, ""));
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));

  EXPECT_THAT(
    raise_infos(logs_),
    ElementsAre("io_timeout_ms raised from 5 to 12 ms for a sync read of 30 servos"));
  // The raise lands on the floor, which is also the ceiling max(8, floor), so no ceiling WARN.
  EXPECT_THAT(timeout_warnings(logs_), IsEmpty());
  EXPECT_THAT(configuration_lines(logs_), ElementsAre(HasSubstr("io timeout 12 ms")));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
}

// Who set the value is checked before the mode: a defaulted value is raised even under
// 'sync_read'; only a user-written value can reach the FATAL.
TEST_F(WaveshareServosLoad, a_defaulted_timeout_is_raised_even_when_sync_read_is_pinned)
{
  auto params = resource_manager_params(
    many_joint_description(10, hardware_param("feedback_mode", "sync_read")));
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));

  EXPECT_THAT(
    raise_infos(logs_),
    ElementsAre("io_timeout_ms raised from 5 to 6 ms for a sync read of 10 servos"));
  EXPECT_THAT(timeout_warnings(logs_), IsEmpty());
  EXPECT_THAT(configuration_lines(logs_), ElementsAre(HasSubstr("feedback_mode 'sync_read'")));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
}

// The ceiling is max(8, floor), not a flat 8: at 20 joints the floor is 9 ms, and 9 is silent.
TEST_F(WaveshareServosLoad, the_two_timeout_warnings_never_contradict_each_other)
{
  auto params = resource_manager_params(
    many_joint_description(20, hardware_param("io_timeout_ms", "9")));
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));

  EXPECT_THAT(timeout_warnings(logs_), IsEmpty());
  EXPECT_THAT(raise_infos(logs_), IsEmpty());
  EXPECT_THAT(configuration_lines(logs_), ElementsAre(HasSubstr("io timeout 9 ms")));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
}

TEST_F(WaveshareServosLoad, unknown_hardware_param_warns_and_still_loads)
{
  // a typo of a known name is the case that matters: it must be visible, and it must not reject
  auto params = resource_manager_params(
    example_description(example_joints(), "", hardware_param("prot0col", "sms_sts")));
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));

  EXPECT_THAT(
    unknown_parameter_warnings(logs_),
    ElementsAre("hardware parameter 'prot0col' is not used by this driver; ignoring it"));
  // the typo left `protocol` at its default, which is what the configuration line reports
  EXPECT_THAT(configuration_lines(logs_), ElementsAre(kDefaultConfigurationLine));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

TEST_F(WaveshareServosLoad, unknown_hardware_params_are_warned_about_in_sorted_order)
{
  // hardware_parameters is an unordered_map, so only sorting makes the log reproducible
  std::string declared = hardware_param("zzz_last", "1");
  declared += kLegacyHardwareParams;
  declared += hardware_param("aaa_first", "1");
  auto params = resource_manager_params(example_description(example_joints(), "", declared));
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));

  EXPECT_THAT(
    unknown_parameter_warnings(logs_),
    ElementsAre(
      "hardware parameter 'aaa_first' is not used by this driver; ignoring it",
      "hardware parameter 'example_param_hw_slowdown' is not used by this driver; ignoring it",
      "hardware parameter 'example_param_hw_start_duration_sec' is not used by this driver; "
      "ignoring it",
      "hardware parameter 'example_param_hw_stop_duration_sec' is not used by this driver; "
      "ignoring it",
      "hardware parameter 'zzz_last' is not used by this driver; ignoring it"));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

TEST_F(WaveshareServosLoad, a_bad_hardware_param_is_reported_before_a_bad_joint)
{
  std::vector<Joint> joints = example_joints();
  joints[1].type = "position";  // the joint loop would reject this one
  auto params = resource_manager_params(
    example_description(joints, "", hardware_param("ping_attempts", "0")));
  hardware_interface::ResourceManager rm(params, false);

  EXPECT_FALSE(rm.load_and_initialize_components(params));
  EXPECT_THAT(
    logs_.messages(RCUTILS_LOG_SEVERITY_FATAL),
    ElementsAre(HasSubstr(out_of_range("ping_attempts", "0", "an integer between 1 and 10"))));
  EXPECT_THAT(configuration_lines(logs_), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

TEST_F(WaveshareServosLoad, hardware_param_rejection_never_opens_the_port)
{
  // the port is opened in on_configure and nowhere else, so even a description whose only fault is
  // a hardware parameter must leave no serial port open
  const std::vector<std::pair<std::string, std::string>> cases = {
    {"empty port", hardware_param("port", "")},
    {"unsupported baudrate", hardware_param("baudrate", "230400")},
    {"a valid port that is never opened", hardware_param("port", "/dev/ttyACM0") +
      hardware_param("io_timeout_ms", "0")}};
  for (const auto & [label, declared] : cases) {
    SCOPED_TRACE(label);
    auto params = resource_manager_params(example_description(example_joints(), "", declared));
    hardware_interface::ResourceManager rm(params, false);

    EXPECT_FALSE(rm.load_and_initialize_components(params));
    EXPECT_EQ(rm.system_components_size(), 0u);
    EXPECT_FALSE(process_has_serial_port_open());
  }
}

class WaveshareServosHwParamRejects : public ::testing::TestWithParam<Rejection>
{
protected:
  LogCapture logs_;
};

TEST_P(WaveshareServosHwParamRejects, on_init_rejects_the_description)
{
  auto params = resource_manager_params(
    example_description(example_joints(), "", GetParam().hardware_params));
  hardware_interface::ResourceManager rm(params, false);

  EXPECT_FALSE(rm.load_and_initialize_components(params));
  EXPECT_EQ(rm.system_components_size(), 0u);
  EXPECT_THAT(rm.get_components_status(), IsEmpty());
  EXPECT_THAT(rm.state_interface_keys(), IsEmpty());
  EXPECT_THAT(rm.command_interface_keys(), IsEmpty());
  // the first bad value wins, so exactly its message is logged
  EXPECT_THAT(
    logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), ElementsAre(HasSubstr(GetParam().fatal_message)));
  EXPECT_THAT(configuration_lines(logs_), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

INSTANTIATE_TEST_SUITE_P(
  HardwareParams, WaveshareServosHwParamRejects, ::testing::ValuesIn(hw_param_rejections()),
  [](const ::testing::TestParamInfo<Rejection> & info) {return info.param.name;});

// ---------------------------------------------------------------------------------------------
// The <joint><param> block: id (required), type, offset, inverted, max_speed, max_accel, unwrap.

namespace
{

// the joint-type summary line, once per successful on_init
std::string joint_summary(size_t joints, size_t position, size_t velocity)
{
  return "parsed " + std::to_string(joints) + " joints: " + std::to_string(position) +
         " position (mode 0), " + std::to_string(velocity) + " velocity (mode 1)";
}

// logged once per joint that reports a multi-turn position
std::string unwrap_line(const std::string & joint)
{
  return "joint '" + joint + "' reports an unwrapped, multi-turn position";
}

}  // namespace

class WaveshareServosJointParams : public ::testing::Test
{
protected:
  // loads a description and returns whether the resource manager accepted it
  bool loads(const std::string & urdf)
  {
    auto params = resource_manager_params(urdf);
    hardware_interface::ResourceManager rm(params, false);
    return rm.load_and_initialize_components(params);
  }

  LogCapture logs_;
};

// A joint that declares a velocity command interface and no position one is a wheel; everything
// else is a position joint. joint1 and joint2 declare both, so they stay position joints.
TEST_F(WaveshareServosJointParams, type_is_inferred_from_the_command_interfaces)
{
  std::vector<Joint> joints = example_joints();
  for (Joint & joint : joints) {
    joint.type = "";
  }
  ASSERT_TRUE(loads(example_description(joints)));

  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_INFO), Contains(joint_summary(4, 2, 2)));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

TEST_F(WaveshareServosJointParams, an_explicit_type_matching_the_command_interfaces_is_accepted)
{
  // the example xacro's own types: 'pos', 'pos', 'vel', exactly what the inference would produce
  ASSERT_TRUE(loads(example_description()));

  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_INFO), Contains(joint_summary(4, 2, 2)));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

// A position joint may also declare a velocity command (a pacing hint); the example xacro,
// the bench description and example_controllers.yaml rely on it.
TEST_F(WaveshareServosJointParams, a_position_joint_may_declare_both_command_interfaces)
{
  auto params = resource_manager_params(example_description());
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));

  EXPECT_THAT(rm.command_interface_keys(), Contains("joint1/position"));
  EXPECT_THAT(rm.command_interface_keys(), Contains("joint1/velocity"));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_INFO), Contains(joint_summary(4, 2, 2)));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
}

// 'true'/'false' in any case; absent is 'false'. joint1's limits are not symmetric, so a lost
// sign maps them to ticks [-1024, 0] and fails the load.
TEST_F(WaveshareServosJointParams, inverted_true_and_false_both_load)
{
  std::vector<Joint> joints = example_joints();
  joints[0].inverted = "true";
  joints[0].offset = "0.0";
  joints[0].command_interfaces[0] = command("position", "-1.570796", "0.0");
  joints[1].inverted = "False";
  ASSERT_TRUE(loads(example_description(joints)));

  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

// Same check, other direction: the tick window of asymmetric limits is the only load-time sign
// of `inverted` (test_lifecycle_over_pty.cpp tests the motion).
TEST_F(WaveshareServosJointParams, inverted_flips_the_tick_mapping_of_the_position_limits)
{
  std::vector<Joint> joints = example_joints();
  joints[0].offset = "0.0";
  joints[0].command_interfaces[0] = command("position", "0.0", "1.570796");
  // uninverted: [0, pi/2] rad is ticks [0, 1024], inside the single turn
  ASSERT_TRUE(loads(example_description(joints)));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());

  // inverted: the same two limits are ticks [-1024, 0], and tick -1024 does not exist
  joints[0].inverted = "true";
  EXPECT_FALSE(loads(example_description(joints)));

  EXPECT_THAT(
    logs_.messages(RCUTILS_LOG_SEVERITY_FATAL),
    ElementsAre(HasSubstr(
      "and inverted=true map to servo ticks [-1024, 0], outside the servo's single-turn range "
      "[0, 4095]")));
  EXPECT_FALSE(process_has_serial_port_open());
}

// The check is on the tick the limit maps to, not on a strict inequality in radians: tick 0 and
// tick encoder_steps - 1 are both reachable, and the example joint1's lower limit is tick 0.
TEST_F(WaveshareServosJointParams, an_offset_at_the_edge_of_the_servo_range_is_accepted)
{
  std::vector<Joint> joints = example_joints();
  joints[0].offset = "3.141593";
  joints[0].command_interfaces[0] = command("position", "-3.141593", "3.140059");
  ASSERT_TRUE(loads(example_description(joints)));

  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

TEST_F(WaveshareServosJointParams, an_offset_one_tick_past_the_edge_is_rejected)
{
  std::vector<Joint> joints = example_joints();
  joints[0].offset = "3.141593";
  joints[0].command_interfaces[0] = command("position", "-3.141593", "3.141593");
  EXPECT_FALSE(loads(example_description(joints)));

  EXPECT_THAT(
    logs_.messages(RCUTILS_LOG_SEVERITY_FATAL),
    ElementsAre(HasSubstr(
      "map to servo ticks [0, 4096], outside the servo's single-turn range [0, 4095]")));
  EXPECT_FALSE(process_has_serial_port_open());
}

// A wheel never gets a goal position, so its offset only shifts the position it reports and
// nothing about it has to fit inside one turn.
TEST_F(WaveshareServosJointParams, a_velocity_joint_is_not_range_checked)
{
  std::vector<Joint> joints = example_joints();
  joints[2].offset = "100.0";
  ASSERT_TRUE(loads(example_description(joints)));

  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

// Requiring finite limits outright would reject every ros2_control_test_assets snippet, and
// skipping silently is the failure this check exists to remove, so it warns.
TEST_F(WaveshareServosJointParams, position_limits_without_min_and_max_warn_instead_of_failing)
{
  std::vector<Joint> joints = example_joints();
  joints[0].command_interfaces[0] = command("position");
  ASSERT_TRUE(loads(example_description(joints)));

  EXPECT_THAT(
    logs_.messages(RCUTILS_LOG_SEVERITY_WARN),
    Contains(
      "joint 'joint1' has type 'pos' but no finite position command limits, so its offset cannot "
      "be checked against the servo's single-turn range [0, 4095] ticks; add <param name=\"min\"> "
      "and <param name=\"max\"> to its position command interface"));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

TEST_F(WaveshareServosJointParams, a_position_command_without_min_and_max_is_accepted)
{
  // backwards compatibility: a bare <command_interface name="position"/> still loads, and nothing
  // is clamped
  std::vector<Joint> joints = example_joints();
  joints[0].command_interfaces[0] = command("position");
  ASSERT_TRUE(loads(example_description(joints)));

  EXPECT_THAT(
    logs_.messages(RCUTILS_LOG_SEVERITY_INFO),
    Not(Contains(HasSubstr("joint 'joint1' position commands clamped"))));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

// Both are SI in the joint frame: 9.2038847 rad/s is the 6000-count default and 23.0097 rad/s^2 is
// the 150-count one, at 4096 steps per revolution.
TEST_F(WaveshareServosJointParams, max_speed_and_max_accel_are_accepted_per_joint)
{
  std::vector<Joint> joints = example_joints();
  joints[0].max_speed = "9.2038847";
  joints[0].max_accel = "23.0097";
  joints[2].max_speed = "1.0";
  joints[2].max_accel = "0";
  ASSERT_TRUE(loads(example_description(joints)));

  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_THAT(
    logs_.messages(RCUTILS_LOG_SEVERITY_WARN),
    Not(Contains(HasSubstr("above the largest"))));
  EXPECT_FALSE(process_has_serial_port_open());
}

// bit 15 of the goal speed field is the direction sign, so the magnitude cannot pass 32767
TEST_F(WaveshareServosJointParams, max_speed_above_the_register_is_capped_with_a_warning)
{
  std::vector<Joint> joints = example_joints();
  joints[0].max_speed = "60.0";
  ASSERT_TRUE(loads(example_description(joints)));

  EXPECT_THAT(
    logs_.messages(RCUTILS_LOG_SEVERITY_WARN),
    Contains(HasSubstr(
      "joint 'joint1' has max_speed 60 rad/s, above the largest the goal speed register can hold "
      "(50.2639 rad/s); using that instead")));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

// the acceleration register is a u8
TEST_F(WaveshareServosJointParams, max_accel_above_the_register_is_capped_with_a_warning)
{
  std::vector<Joint> joints = example_joints();
  joints[0].max_accel = "50.0";
  ASSERT_TRUE(loads(example_description(joints)));

  EXPECT_THAT(
    logs_.messages(RCUTILS_LOG_SEVERITY_WARN),
    Contains(HasSubstr(
      "joint 'joint1' has max_accel 50 rad/s^2, above the largest the acceleration register can "
      "hold (39.1165 rad/s^2); using that instead")));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

// Past long's range std::lround is unspecified (LONG_MIN here); such a value is too large,
// never "less than one count", so the cap WARN fires, not the FATAL.
TEST_F(WaveshareServosJointParams, limits_past_the_range_of_a_long_are_capped_not_called_too_small)
{
  std::vector<Joint> joints = example_joints();
  joints[0].max_speed = "1e17";
  joints[1].max_accel = "1e17";
  ASSERT_TRUE(loads(example_description(joints)));

  const auto warnings = logs_.messages(RCUTILS_LOG_SEVERITY_WARN);
  EXPECT_THAT(
    warnings,
    Contains(HasSubstr(
      "joint 'joint1' has max_speed 1e+17 rad/s, above the largest the goal speed register can "
      "hold (50.2639 rad/s); using that instead")));
  EXPECT_THAT(
    warnings,
    Contains(HasSubstr(
      "joint 'joint2' has max_accel 1e+17 rad/s^2, above the largest the acceleration register "
      "can hold (39.1165 rad/s^2); using that instead")));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

// An unknown <joint> param warns and is ignored, never rejected: `invert` for `inverted` must
// be visible, and documentation params are legal.
TEST_F(WaveshareServosJointParams, an_unknown_joint_param_warns_and_still_loads)
{
  std::vector<Joint> joints = example_joints();
  // joint.parameters is an unordered_map, so only sorting makes the log reproducible
  joints[0].extra_params = joint_param("zzz_last", "1") + joint_param("invert", "true");
  joints[2].extra_params = joint_param("max_sped", "1.0");
  ASSERT_TRUE(loads(example_description(joints)));

  EXPECT_THAT(
    unknown_parameter_warnings(logs_),
    ElementsAre(
      "joint 'joint1' parameter 'invert' is not used by this driver; ignoring it",
      "joint 'joint1' parameter 'zzz_last' is not used by this driver; ignoring it",
      "joint 'joint3' parameter 'max_sped' is not used by this driver; ignoring it"));
  // the typo left joint1 uninverted, which the example limits load fine with
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

// unwrap defaults to the resolved type: a wheel turns past a revolution, a position joint cannot
TEST_F(WaveshareServosJointParams, unwrap_defaults_to_on_for_vel_joints_only)
{
  ASSERT_TRUE(loads(example_description()));

  const auto info = logs_.messages(RCUTILS_LOG_SEVERITY_INFO);
  EXPECT_THAT(info, Contains(unwrap_line("joint3")));
  EXPECT_THAT(info, Not(Contains(unwrap_line("joint1"))));
  EXPECT_THAT(info, Not(Contains(unwrap_line("joint2"))));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

TEST_F(WaveshareServosJointParams, unwrap_false_turns_a_vel_joint_back_to_wrapping)
{
  // One capture for both sub-cases (LogCapture is not reentrant); neither may log the line,
  // so nothing needs clearing between them.
  for (const std::string declared : {"false", "False"}) {
    SCOPED_TRACE(declared);
    std::vector<Joint> joints = example_joints();
    joints[2].unwrap = declared;
    ASSERT_TRUE(loads(example_description(joints)));

    EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_INFO), Not(Contains(unwrap_line("joint3"))));
    EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  }
  EXPECT_FALSE(process_has_serial_port_open());
}

// ---------------------------------------------------------------------------------------------
// Free-form state interfaces: any subset of the nine (or none), any order; `torque` is kg cm.

namespace
{

// every state interface the driver serves, in StateKind order
const std::vector<std::string> kAllStateNames = {
  "position", "velocity", "effort", "current", "voltage", "temperature", "load", "status",
  "torque"};

std::vector<std::string> states_named(const std::vector<std::string> & names)
{
  std::vector<std::string> fragments;
  for (const std::string & name : names) {
    fragments.push_back(state(name));
  }
  return fragments;
}

// the deprecation WARN, once per joint that declares `torque`
std::string torque_deprecation(const std::string & joint)
{
  return "joint '" + joint + "' declares the deprecated state interface 'torque' (kg cm); it keeps "
         "working and keeps reporting kg cm, but declare 'effort' instead, which reports N m";
}

std::vector<std::string> deprecation_warnings(const LogCapture & logs)
{
  std::vector<std::string> selected;
  for (const auto & message : logs.messages(RCUTILS_LOG_SEVERITY_WARN)) {
    if (message.find("deprecated state interface") != std::string::npos) {
      selected.push_back(message);
    }
  }
  return selected;
}

}  // namespace

class WaveshareServosStateInterfaces : public ::testing::Test
{
protected:
  bool loads(const std::string & urdf)
  {
    auto params = resource_manager_params(urdf);
    hardware_interface::ResourceManager rm(params, false);
    return rm.load_and_initialize_components(params);
  }

  LogCapture logs_;
};

TEST_F(WaveshareServosStateInterfaces, any_subset_and_order_of_state_interfaces_is_accepted)
{
  const std::vector<std::vector<std::string>> subsets = {
    {"position"},
    {"status"},
    {"temperature", "position"},
    {"load", "voltage", "current"},
    {"velocity", "position", "temperature", "effort"},
    {"status", "load", "temperature", "voltage", "current", "effort", "velocity", "position"}};
  for (const auto & names : subsets) {
    SCOPED_TRACE(::testing::PrintToString(names));
    std::vector<Joint> joints = example_joints();
    joints[0].state_interfaces = states_named(names);
    EXPECT_TRUE(loads(example_description(joints)));
  }
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

TEST_F(WaveshareServosStateInterfaces, a_joint_with_no_state_interfaces_is_accepted)
{
  // a joint the driver only commands: nothing is published for it, and nothing is refused either
  std::vector<Joint> joints = example_joints();
  joints[1].state_interfaces.clear();
  auto params = resource_manager_params(example_description(joints));
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));

  for (const std::string & name : kAllStateNames) {
    EXPECT_THAT(rm.state_interface_keys(), Not(Contains("joint2/" + name)));
  }
  EXPECT_THAT(rm.state_interface_keys(), Contains("joint1/position"));
  EXPECT_THAT(rm.command_interface_keys(), Contains("joint2/position"));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

TEST_F(WaveshareServosStateInterfaces, all_supported_state_interfaces_can_be_declared_together)
{
  std::vector<Joint> joints = example_joints();
  joints[0].state_interfaces = states_named(kAllStateNames);
  const std::string urdf = example_description(joints);
  auto params = resource_manager_params(urdf);
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));

  for (const std::string & name : kAllStateNames) {
    const std::string key = "joint1/" + name;
    EXPECT_THAT(rm.state_interface_keys(), Contains(key));
    EXPECT_EQ(rm.get_state_interface_data_type(key), "double") << key;
  }
  // nothing is configured, so all nine are NaN, `status` included (consumers guard it with
  // std::isfinite)
  InitializedSystem initialized(urdf);
  ASSERT_EQ(initialized.state_id(), State::PRIMARY_STATE_UNCONFIGURED);
  for (const auto & handle : initialized.system().export_state_interfaces()) {
    const auto value = handle->get_optional();
    ASSERT_TRUE(value.has_value()) << handle->get_name();
    EXPECT_TRUE(std::isnan(*value)) << handle->get_name() << " = " << *value;
  }
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

// The export order stays the description order for any subset: three, two, none and the
// example's four state interfaces on the four joints.
TEST_F(
  WaveshareServosStateInterfaces, state_interfaces_are_exported_in_description_order_for_any_subset)
{
  std::vector<Joint> joints = example_joints();
  joints[0].state_interfaces = states_named({"status", "position", "load"});
  joints[1].state_interfaces = states_named({"temperature", "effort"});
  joints[2].state_interfaces.clear();
  auto params = resource_manager_params(example_description(joints));
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));

  const auto & status = rm.get_components_status();
  ASSERT_EQ(status.count(kExampleName), 1u);
  EXPECT_THAT(
    status.at(kExampleName).state_interfaces,
    ElementsAre(
      "joint1/status", "joint1/position", "joint1/load", "joint2/temperature", "joint2/effort",
      "joint4/position", "joint4/velocity", "joint4/effort", "joint4/temperature"));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

TEST_F(WaveshareServosStateInterfaces, torque_alias_warns_once_naming_the_joint_and_effort)
{
  std::vector<Joint> joints = example_joints();
  joints[0].state_interfaces = legacy_servo_states();
  joints[2].state_interfaces = legacy_servo_states();
  ASSERT_TRUE(loads(example_description(joints)));

  // one WARN per joint that declares it, in URDF order, and none for the joints that do not
  EXPECT_THAT(
    deprecation_warnings(logs_),
    ElementsAre(torque_deprecation("joint1"), torque_deprecation("joint3")));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

TEST_F(WaveshareServosStateInterfaces, effort_alone_produces_no_deprecation_warning)
{
  // the shipped example, which declares `effort` on all four joints
  ASSERT_TRUE(loads(example_description()));

  EXPECT_THAT(deprecation_warnings(logs_), IsEmpty());
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

// They are different interfaces in different units, not two spellings of one: a description
// migrating from the alias may publish both while its consumers move over.
TEST_F(WaveshareServosStateInterfaces, torque_and_effort_may_be_declared_on_the_same_joint)
{
  std::vector<Joint> joints = example_joints();
  joints[0].state_interfaces = states_named({"position", "velocity", "effort", "torque"});
  auto params = resource_manager_params(example_description(joints));
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));

  EXPECT_THAT(rm.state_interface_keys(), Contains("joint1/effort"));
  EXPECT_THAT(rm.state_interface_keys(), Contains("joint1/torque"));
  EXPECT_THAT(deprecation_warnings(logs_), ElementsAre(torque_deprecation("joint1")));
  EXPECT_THAT(logs_.messages(RCUTILS_LOG_SEVERITY_FATAL), IsEmpty());
  EXPECT_FALSE(process_has_serial_port_open());
}

TEST_F(WaveshareServosStateInterfaces, deprecation_warning_uses_the_component_logger)
{
  const std::string component_logger =
    std::string(kRmLogger) + ".hardware_component.system." + kExampleName;
  std::vector<Joint> joints = example_joints();
  joints[1].state_interfaces = legacy_servo_states();
  ASSERT_TRUE(loads(example_description(joints)));

  std::vector<std::string> loggers;
  for (const auto & record : logs_.records()) {
    if (record.message.find("deprecated state interface") != std::string::npos) {
      loggers.push_back(record.logger);
    }
  }
  EXPECT_THAT(loggers, ElementsAre(component_logger));
}

// ---------------------------------------------------------------------------------------------
// Jazzy API: framework-built interface handles and the component logger.

class WaveshareServosJazzyApi : public ::testing::Test
{
protected:
  LogCapture logs_;
};

// The interfaces are framework handles built from the URDF, so <param name="initial_value">
// applies; without it the value is NaN.
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

// The driver logs through get_logger():
// "<resource manager logger>.hardware_component.system.<ros2_control name>".
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
      record.message.find("; it must be 'pos' or 'vel'") != std::string::npos)
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
