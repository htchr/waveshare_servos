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
using waveshare_servos_test::joint_param;
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

std::vector<Rejection> rejections()
{
  return {
    // PHASE2_SPEC 5.1: `id` is required, and every one of these rows was undefined behaviour in
    // Phase 1 -- the missing-param case a measured SIGSEGV on the unguarded find("id")->second.
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
    // hardware_interface::stoi_generic parses through std::stol, which accepts a leading '+' and
    // leading zeros, so the declared text and the parsed number differ on exactly the path where
    // the parsed number exists. PHASE2_SPEC 5.1 writes the number.
    {"id_duplicate_written_differently",
      [](std::vector<Joint> & j) {j[2].id = "+001";},
      "joint 'joint3' has id 1, which joint 'joint1' already uses; ids must be unique within a "
      "<ros2_control> block", ""},
    // PHASE2_SPEC 7.2/7.3: the state interfaces are free-form -- any subset, any order -- so the
    // six fixed-order rows that used to live here all describe legal descriptions now. What is
    // left to reject is a name the driver does not serve, a non-double data_type and a duplicate.
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
    // PHASE2_SPEC 5.2. 'position' is the ros2_control interface name, not the enum token this
    // param takes, so it is the typo the message has to name.
    {"type_not_pos_or_vel",
      [](std::vector<Joint> & j) {j[1].type = "position";},
      "joint 'joint2' has type 'position'; it must be 'pos' or 'vel'", ""},
    {"type_empty",
      [](std::vector<Joint> & j) {j[1].type = kEmptyParam;},
      "joint 'joint2' has type ''; it must be 'pos' or 'vel'", ""},
    // row E8 of the PHASE2_SPEC 5.2 table: a wheel takes only a velocity command
    {"type_vel_with_a_position_command",
      [](std::vector<Joint> & j) {
        j[2].type = "vel";
        j[2].command_interfaces = {command("position"), command("velocity")};
      },
      "joint 'joint3' has type 'vel' but declares a position command interface; a velocity joint "
      "runs its servo in wheel mode and takes only <command_interface name=\"velocity\">", ""},
    // row E7 (D3): a velocity command interface only paces the move, it cannot replace the goal
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
    // PHASE2_SPEC 5.3: offset is a servo-frame constant, so a bad one is only visible as the tick
    // its joint limits map to. joint1 is offset 1.570796 rad with limits +-1.570796 -> ticks
    // [0, 2048]; every row below moves that window off one end of the servo's single turn.
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
    // An offset that is finite but astronomically large overflows the tick arithmetic. std::lround
    // of a double outside long's range is unspecified -- on this target it is LONG_MIN -- so an
    // enormous positive offset would otherwise be reported as an enormous negative tick. The
    // verdict is a rejection either way; what this row pins is that the number printed still has
    // the sign of the offset.
    {"offset_past_the_range_of_a_long",
      [](std::vector<Joint> & j) {
        j[2].type = "pos";
        j[2].command_interfaces = {command("position")};
        j[2].offset = "1e300";
      },
      "maps its zero position to servo tick 4611686018427387904, outside the servo's single-turn "
      "range [0, 4095]", ""},
    // PHASE2_SPEC 5.3: '1' and '0' are deliberately not accepted, so neither is anything else
    {"inverted_not_a_bool",
      [](std::vector<Joint> & j) {j[0].inverted = "yes";},
      "joint 'joint1' has inverted='yes'; it must be 'true' or 'false'", ""},
    // PHASE2_SPEC 5.4: max_speed is rad/s in the joint frame. A value of 0 would make
    // std::clamp(speed, 1.0, 0.0) undefined behaviour on the write path, so it is refused here.
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
    // PHASE2_SPEC 5.5 / 8.2: unwrapping a position joint would break the goal-speed pacing, the
    // limit check and the activation seed, so it is a FATAL rather than a silently ignored param
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
// Item 1: the <hardware><param> block, its validation and the resolved-configuration INFO line.

namespace
{

// The PHASE2_SPEC 4.5 line for a description that declares no hardware parameter at all: every
// value is the compiled-in default, and the two doubles print with %.7g (%g would round
// 0.8825985 to 0.882599).
constexpr char kDefaultConfigurationLine[] =
  "bus configuration: port '/dev/ttyACM0', 1000000 baud, protocol 'sms_sts', io timeout 20 ms, "
  "3 ping attempt(s), drop a servo after 50 consecutive read failures, allow_missing_servos false, "
  "4096 encoder steps per revolution, 0.006 A per current count, 0.8825985 N m/A";

// The configuration lines of PHASE2_SPEC 4.5, in the order they were logged. on_init logs exactly
// one per successful load and none at all when a value is rejected.
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

// The "ignoring it" warnings of PHASE2_SPEC 4.3, in the order they were logged.
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

// The three message templates of PHASE2_SPEC 4.2 a hardware parameter can reach, spelled out here
// independently of the driver's own params::message().
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
  const std::string timeout = "an integer between 1 and 1000 (milliseconds)";
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
    hw_reject(
      "io_timeout_ms_zero", "io_timeout_ms", "0", out_of_range("io_timeout_ms", "0", timeout)),
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

TEST_F(WaveshareServosLoad, hardware_params_default_to_the_phase1_constants)
{
  // no <param> at all in <hardware>: every value below is the compiled-in Phase 1 constant
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
  declared += hardware_param("io_timeout_ms", "5");
  declared += hardware_param("ping_attempts", "1");
  declared += hardware_param("max_read_fails", "7");
  declared += hardware_param("allow_missing_servos", "true");
  declared += hardware_param("encoder_steps", "1024");
  declared += hardware_param("current_per_count_a", "0.0065");
  declared += hardware_param("torque_constant_nm_per_a", "1.5");
  auto params = resource_manager_params(example_description(example_joints(), "", declared));
  hardware_interface::ResourceManager rm(params, false);
  ASSERT_TRUE(rm.load_and_initialize_components(params));

  // exactly one line, and every one of the ten values is the declared one. A repeated <param>
  // keeps the last value with no diagnostic from the parser, so this line is the user's only check
  EXPECT_THAT(
    configuration_lines(logs_),
    ElementsAre(
      "bus configuration: port '/dev/ttyUSB1', 115200 baud, protocol 'sms_sts', io timeout 5 ms, "
      "1 ping attempt(s), drop a servo after 7 consecutive read failures, allow_missing_servos "
      "true, 1024 encoder steps per revolution, 0.0065 A per current count, 1.5 N m/A"));
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
// Item 2: the <joint><param> block -- id, type inference, offset, inverted, max_speed, max_accel
// and unwrap (PHASE2_SPEC 5). Every one of the seven is optional except id, so the descriptions
// that loaded before this item still load.

namespace
{

// the summary line PHASE2_SPEC 5.2 asks for, once per successful on_init
std::string joint_summary(size_t joints, size_t position, size_t velocity)
{
  return "parsed " + std::to_string(joints) + " joints: " + std::to_string(position) +
         " position (mode 0), " + std::to_string(velocity) + " velocity (mode 1)";
}

// the PHASE2_SPEC 5.5 line, logged once per joint that reports a multi-turn position
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

// The velocity command interface of a position joint is only a pacing hint (the goal is always the
// position), and the example xacro, the bench description and example_controllers.yaml all rely on
// a position joint declaring both.
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

// Exactly 'true'/'false', case-insensitively, and absent is 'false'.
//
// joint1's limits are deliberately NOT symmetric about its offset, so this case is load-bearing
// for the sign and not only for the spelling: with offset 0 and limits [-pi/2, 0] the window is
// ticks [0, 1024] when 'true' reached JointConfig::sign and ticks [-1024, 0] when it did not, and
// the second is a FATAL. joint2 keeps the example limits, which are symmetric, because what it
// pins is the 'False' spelling parsing at all.
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

// The other direction of the same pin: one description, one parameter changed, opposite verdicts.
// `inverted` is the parameter that decides which way the servo turns, and the only load-time
// observable of the sign it produces is the tick window the position limits map to (the driver-
// level proof is the pty chunk's inverted_flips_the_commanded_position_and_the_wheel_speed), so
// an asymmetric window is what keeps a wrong-signed or dropped assignment red here.
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
// tick encoder_steps - 1 are both reachable, and the bench A/B description sits on tick 0.
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

// Both conversions round a double into an integer count, and a finite but astronomically large
// limit lands outside long's range, where std::lround is unspecified (LONG_MIN on this target).
// Such a value is "too large for the register", never "smaller than one count", so the cap WARN is
// the outcome and the FATAL below it must not fire.
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

// The <hardware> block already warns about a <param> name it does not know; a typo inside a
// <joint> block is the worse of the two, because `invert` instead of `inverted` is a servo that
// turns the wrong way with no diagnostic at all. Warn and ignore, never reject: a description may
// legitimately carry documentation params.
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
  // One capture for both sub-cases: LogCapture is not reentrant (a second one would chain the
  // handler to itself and recurse until the stack runs out), and neither sub-case may log the
  // line, so there is nothing to clear between them.
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
// Items 3 and 7: free-form state interfaces (PHASE2_SPEC 7.2, 7.3). Any subset of the nine, in any
// order, per joint; the empty set is legal; `torque` still works and still says kg cm (D2).

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

// the PHASE2_SPEC 3.4 deprecation WARN, once per joint that declares the alias
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
  // nothing has been configured, so every one of the nine is still NaN -- `status` included, which
  // is the value a consumer has to guard with std::isfinite (PHASE2_SPEC 3.5)
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

// The export order is what `ros2 control list_hardware_components -v` prints and what a controller
// claiming every interface (/dynamic_joint_states) sees, so it stays the description's order even
// when a joint declares three interfaces, another two, a third none and a fourth the whole
// set it was given.
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
