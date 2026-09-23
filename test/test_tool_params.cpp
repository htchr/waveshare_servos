// Tests for src/tool_params.{hpp,cpp} -- the Phase 6 tools' parameter parser (PHASE6_SPEC A, D.3).
//
// The parser is pure, so almost every case hands it a std::map<std::string, rclcpp::ParameterValue>
// built by hand: no rclcpp::init, no node, no bus. The last two cases are the exception on purpose.
// They pin the two rclcpp/rcl behaviours tool_main depends on and cannot test itself without a
// real context: that a node's own overrides arrive with their names untouched (so a stale name
// reaches the parser instead of vanishing), and that rcl keeps an override addressed to another
// node under that node's name (so it can be refused rather than silently dropped).
//
// Every refusal here is what keeps an old or mistyped command line off /dev/ttyACM0: an ignored
// `device_port` opens the default port, and an id narrowed to 8 bits can be the broadcast id.

#include <gmock/gmock.h>

#include <unistd.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "driver_defaults.hpp"
#include "rcl/arguments.h"
#include "rcl_yaml_param_parser/parser.h"
#include "rclcpp/rclcpp.hpp"
#include "tool_params.hpp"

namespace
{

// Per-name declarations, never a using-directive (cpplint build/namespaces).
using ::testing::AllOf;
using ::testing::ElementsAre;
using ::testing::HasSubstr;
using ::testing::IsEmpty;
using ::testing::Not;
using ::testing::StartsWith;
using rclcpp::ParameterValue;
using waveshare_servos::tools::ParseResult;
using waveshare_servos::tools::Tool;
using waveshare_servos::tools::foreign_override_nodes;
using waveshare_servos::tools::name_of;
using waveshare_servos::tools::parse_params;
using waveshare_servos::tools::usage;

using Overrides = std::map<std::string, ParameterValue>;

constexpr Tool kAllTools[] = {
  Tool::kScan, Tool::kSetId, Tool::kCalibrateMidpoint, Tool::kFactoryReset};

// The ids a tool needs, so a case about another parameter is not refused for a missing id (Q4).
Overrides with_ids(Tool tool, Overrides overrides)
{
  if (tool == Tool::kSetId) {
    overrides.emplace("start_id", ParameterValue(int64_t{200}));
    overrides.emplace("new_id", ParameterValue(int64_t{201}));
  } else if (tool == Tool::kCalibrateMidpoint || tool == Tool::kFactoryReset) {
    overrides.emplace("id", ParameterValue(int64_t{200}));
  }
  return overrides;
}

// The single error of a result that must have exactly one; "" (and a failure) otherwise.
std::string only_error(const ParseResult & result)
{
  EXPECT_FALSE(result.config.has_value()) << "an error and a config together";
  EXPECT_EQ(result.errors.size(), 1u);
  return result.errors.size() == 1 ? result.errors.front() : std::string();
}

}  // namespace

TEST(ToolParams, defaults_are_the_hardware_parameter_defaults)
{
  // An empty map is a bare `ros2 run waveshare_servos scan`: the hardware interface's defaults,
  // compared both with the shared constants and with the literals, so neither can drift alone.
  const ParseResult result = parse_params(Tool::kScan, {});
  ASSERT_TRUE(result.config.has_value());
  EXPECT_THAT(result.errors, IsEmpty());
  EXPECT_EQ(result.config->port, waveshare_servos::defaults::kPort);
  EXPECT_EQ(result.config->port, "/dev/ttyACM0");
  EXPECT_EQ(result.config->baudrate, waveshare_servos::defaults::kBaudrate);
  EXPECT_EQ(result.config->baudrate, 1000000);
  EXPECT_EQ(result.config->id, -1);
  EXPECT_EQ(result.config->start_id, -1);
  EXPECT_EQ(result.config->new_id, -1);
}

TEST(ToolParams, new_names_are_applied)
{
  const ParseResult scan = parse_params(
    Tool::kScan, {{"port", ParameterValue(std::string("/dev/ttyUSB3"))},
      {"baudrate", ParameterValue(int64_t{115200})}});
  ASSERT_TRUE(scan.config.has_value()) << ::testing::PrintToString(scan.errors);
  EXPECT_EQ(scan.config->port, "/dev/ttyUSB3");
  EXPECT_EQ(scan.config->baudrate, 115200);

  const ParseResult set_id = parse_params(
    Tool::kSetId, {{"port", ParameterValue(std::string("/dev/ttyUSB3"))},
      {"baudrate", ParameterValue(int64_t{500000})}, {"start_id", ParameterValue(int64_t{4})},
      {"new_id", ParameterValue(int64_t{253})}});
  ASSERT_TRUE(set_id.config.has_value()) << ::testing::PrintToString(set_id.errors);
  EXPECT_EQ(set_id.config->port, "/dev/ttyUSB3");
  EXPECT_EQ(set_id.config->baudrate, 500000);
  EXPECT_EQ(set_id.config->start_id, 4);
  EXPECT_EQ(set_id.config->new_id, 253);
  EXPECT_EQ(set_id.config->id, -1);

  const ParseResult calibrate = parse_params(
    Tool::kCalibrateMidpoint, {{"id", ParameterValue(int64_t{2})}});
  ASSERT_TRUE(calibrate.config.has_value()) << ::testing::PrintToString(calibrate.errors);
  EXPECT_EQ(calibrate.config->id, 2);
  EXPECT_EQ(calibrate.config->port, "/dev/ttyACM0") << "the other parameters keep their defaults";
  EXPECT_EQ(calibrate.config->baudrate, 1000000);
}

TEST(ToolParams, device_port_is_refused_and_names_port)
{
  // Ignoring the Phase 1 name would open the default port whatever the command line said. It is
  // refused even when the new name is given too: the command line is stale either way.
  for (const Tool tool : kAllTools) {
    SCOPED_TRACE(name_of(tool));
    const ParseResult alone = parse_params(
      tool, with_ids(tool, {{"device_port", ParameterValue(std::string("/dev/ttyUSB0"))}}));
    EXPECT_EQ(
      only_error(alone),
      "parameter 'device_port' was renamed to 'port', the name the hardware interface uses; "
      "refusing to run rather than ignore it and open the default port '/dev/ttyACM0'");
    const ParseResult both = parse_params(
      tool, with_ids(
        tool, {{"device_port", ParameterValue(std::string("/dev/ttyUSB0"))},
          {"port", ParameterValue(std::string("/dev/ttyUSB0"))}}));
    EXPECT_THAT(only_error(both), HasSubstr("renamed to 'port'"));
  }
}

TEST(ToolParams, baud_rate_is_refused_and_names_baudrate)
{
  for (const Tool tool : kAllTools) {
    SCOPED_TRACE(name_of(tool));
    const ParseResult alone = parse_params(
      tool, with_ids(tool, {{"baud_rate", ParameterValue(int64_t{115200})}}));
    EXPECT_EQ(
      only_error(alone),
      "parameter 'baud_rate' was renamed to 'baudrate'; refusing to run rather than ignore it "
      "and use the default 1000000");
    const ParseResult both = parse_params(
      tool, with_ids(
        tool, {{"baud_rate", ParameterValue(int64_t{115200})},
          {"baudrate", ParameterValue(int64_t{115200})}}));
    EXPECT_THAT(only_error(both), HasSubstr("renamed to 'baudrate'"));
  }
}

TEST(ToolParams, an_unknown_name_is_refused_and_the_accepted_names_listed)
{
  EXPECT_EQ(
    only_error(parse_params(Tool::kScan, {{"io_timeout_ms", ParameterValue(int64_t{20})}})),
    "parameter 'io_timeout_ms' is not a parameter of scan, which takes: baudrate, port; "
    "refusing to run rather than ignore it");
  EXPECT_EQ(
    only_error(parse_params(Tool::kSetId, with_ids(Tool::kSetId, {{"idd", ParameterValue(1)}}))),
    "parameter 'idd' is not a parameter of set_id, which takes: baudrate, new_id, port, "
    "start_id; refusing to run rather than ignore it");
  EXPECT_EQ(
    only_error(
      parse_params(
        Tool::kCalibrateMidpoint,
        with_ids(Tool::kCalibrateMidpoint, {{"start_id", ParameterValue(int64_t{1})}}))),
    "parameter 'start_id' is not a parameter of calibrate_midpoint, which takes: baudrate, id, "
    "port; refusing to run rather than ignore it");
}

TEST(ToolParams, a_double_is_not_an_integer)
{
  // `-p new_id:=201.0` used to abort the old binary (134); `baudrate:=1e6` is a double to rcl.
  EXPECT_EQ(
    only_error(parse_params(Tool::kScan, {{"baudrate", ParameterValue(1e6)}})),
    "parameter 'baudrate' is 1000000.0 (a double), which is not an integer");
  EXPECT_EQ(
    only_error(
      parse_params(
        Tool::kSetId, {{"start_id", ParameterValue(4.0)}, {"new_id", ParameterValue(int64_t{7})}})),
    "parameter 'start_id' is 4.0 (a double), which is not an integer");
  EXPECT_EQ(
    only_error(parse_params(Tool::kCalibrateMidpoint, {{"id", ParameterValue(2.5)}})),
    "parameter 'id' is 2.5 (a double), which is not an integer");
}

TEST(ToolParams, a_string_or_bool_is_not_an_integer)
{
  EXPECT_EQ(
    only_error(parse_params(Tool::kScan, {{"baudrate", ParameterValue(std::string("fast"))}})),
    "parameter 'baudrate' is 'fast' (a string), which is not an integer");
  EXPECT_EQ(
    only_error(parse_params(Tool::kCalibrateMidpoint, {{"id", ParameterValue(true)}})),
    "parameter 'id' is true (a bool), which is not an integer");
}

TEST(ToolParams, an_integer_is_not_a_port)
{
  // `port:=0` is an integer to rcl. Never coerced to "0": a string parameter takes strings only.
  EXPECT_EQ(
    only_error(parse_params(Tool::kScan, {{"port", ParameterValue(int64_t{0})}})),
    "parameter 'port' is 0 (an integer), which is not a string");
}

TEST(ToolParams, an_empty_port_is_refused)
{
  EXPECT_EQ(
    only_error(parse_params(Tool::kScan, {{"port", ParameterValue(std::string())}})),
    "parameter 'port' is empty; expected a device path such as '/dev/ttyACM0'");
}

TEST(ToolParams, unmapped_baudrates_are_refused_with_the_library_sentence)
{
  // 2^32 + 9600 narrows to 9600, a rate that IS mapped: the int range is checked first.
  for (const int64_t baudrate : {int64_t{230400}, int64_t{0}, (int64_t{1} << 32) + 9600}) {
    SCOPED_TRACE(baudrate);
    EXPECT_EQ(
      only_error(parse_params(Tool::kScan, {{"baudrate", ParameterValue(baudrate)}})),
      "parameter 'baudrate' is '" + std::to_string(baudrate) + "'; the servo library maps only "
      "9600, 19200, 38400, 57600, 115200, 500000 and 1000000, and silently falls back to 115200 "
      "for anything else");
  }
  for (const int64_t baudrate : {9600, 19200, 38400, 57600, 115200, 500000, 1000000}) {
    SCOPED_TRACE(baudrate);
    const ParseResult result = parse_params(
      Tool::kScan, {{"baudrate", ParameterValue(baudrate)}});
    ASSERT_TRUE(result.config.has_value()) << ::testing::PrintToString(result.errors);
    EXPECT_EQ(result.config->baudrate, baudrate);
  }
}

TEST(ToolParams, ids_are_range_checked_before_narrowing)
{
  // Narrowed to 8 bits, 254 is the broadcast id (every servo obeys it), 510 is 254 again, 255 a
  // header byte and 300 is 44. None of them may reach a config.
  const auto set_id = [](int64_t start_id, int64_t new_id) {
      return parse_params(
        Tool::kSetId, {{"start_id", ParameterValue(start_id)}, {"new_id", ParameterValue(new_id)}});
    };
  const std::string start_range = "expected an integer between 0 and 253";
  EXPECT_EQ(
    only_error(set_id(254, 7)),
    "parameter 'start_id' is 254, out of range; expected an integer between 0 and 253 (254 is "
    "the broadcast id, which every servo obeys and none answers)");
  for (const int64_t start_id : {int64_t{255}, int64_t{300}, int64_t{510}, int64_t{-1}}) {
    SCOPED_TRACE(start_id);
    EXPECT_THAT(
      only_error(set_id(start_id, 7)),
      AllOf(
        StartsWith("parameter 'start_id' is " + std::to_string(start_id) + ", out of range"),
        HasSubstr(start_range)));
  }
  const ParseResult zero = set_id(0, 7);
  ASSERT_TRUE(zero.config.has_value()) << "a servo left at id 0 must be reachable";
  EXPECT_EQ(zero.config->start_id, 0);

  EXPECT_EQ(
    only_error(set_id(7, 0)),
    "parameter 'new_id' is 0, out of range; expected an integer between 1 and 253 (the hardware "
    "interface accepts ids 1..253)");
  EXPECT_THAT(
    only_error(set_id(7, 254)), StartsWith("parameter 'new_id' is 254, out of range"));
  const ParseResult top = set_id(7, 253);
  ASSERT_TRUE(top.config.has_value());
  EXPECT_EQ(top.config->new_id, 253);

  for (const int64_t id : {int64_t{254}, int64_t{300}}) {
    SCOPED_TRACE(id);
    EXPECT_THAT(
      only_error(parse_params(Tool::kCalibrateMidpoint, {{"id", ParameterValue(id)}})),
      AllOf(StartsWith("parameter 'id' is " + std::to_string(id) + ", out of range"),
      HasSubstr(start_range)));
  }
}

TEST(ToolParams, set_id_requires_both_ids)
{
  // [Q4] The old defaults were 1, so a bare set_id or calibrate addressed servo 1.
  const ParseResult neither = parse_params(Tool::kSetId, {});
  EXPECT_FALSE(neither.config.has_value());
  EXPECT_THAT(
    neither.errors, ElementsAre(
      "parameter 'new_id' is missing; expected an integer between 1 and 253",
      "parameter 'start_id' is missing; expected an integer between 0 and 253"));
  EXPECT_EQ(
    only_error(parse_params(Tool::kSetId, {{"start_id", ParameterValue(int64_t{200})}})),
    "parameter 'new_id' is missing; expected an integer between 1 and 253");
  EXPECT_EQ(
    only_error(parse_params(Tool::kSetId, {{"new_id", ParameterValue(int64_t{201})}})),
    "parameter 'start_id' is missing; expected an integer between 0 and 253");
}

TEST(ToolParams, calibrate_requires_id)
{
  EXPECT_EQ(
    only_error(parse_params(Tool::kCalibrateMidpoint, {})),
    "parameter 'id' is missing; expected an integer between 0 and 253");
}

TEST(ToolParams, factory_reset_takes_a_required_id_and_the_port_parameters)
{
  // factory_reset (FACTORY_RESET_SPEC 2): calibrate's surface exactly -- id required and range
  // checked before narrowing, port and baudrate with the hardware parameters' names and defaults.
  EXPECT_EQ(
    only_error(parse_params(Tool::kFactoryReset, {})),
    "parameter 'id' is missing; expected an integer between 0 and 253");
  EXPECT_EQ(
    only_error(parse_params(Tool::kFactoryReset, {{"id", ParameterValue(int64_t{254})}})),
    "parameter 'id' is 254, out of range; expected an integer between 0 and 253 (254 is the "
    "broadcast id, which every servo obeys and none answers)");
  for (const int64_t id : {int64_t{255}, int64_t{510}, int64_t{-1}}) {
    SCOPED_TRACE(id);
    EXPECT_THAT(
      only_error(parse_params(Tool::kFactoryReset, {{"id", ParameterValue(id)}})),
      StartsWith("parameter 'id' is " + std::to_string(id) + ", out of range"));
  }
  const ParseResult zero = parse_params(Tool::kFactoryReset, {{"id", ParameterValue(int64_t{0})}});
  ASSERT_TRUE(zero.config.has_value()) << "a servo at id 0 can be reset";
  EXPECT_EQ(zero.config->id, 0);
  EXPECT_EQ(zero.config->port, "/dev/ttyACM0");
  EXPECT_EQ(zero.config->baudrate, 1000000);

  const ParseResult given = parse_params(
    Tool::kFactoryReset, {{"port", ParameterValue(std::string("/dev/ttyUSB3"))},
      {"baudrate", ParameterValue(int64_t{500000})}, {"id", ParameterValue(int64_t{4})}});
  ASSERT_TRUE(given.config.has_value()) << ::testing::PrintToString(given.errors);
  EXPECT_EQ(given.config->port, "/dev/ttyUSB3");
  EXPECT_EQ(given.config->baudrate, 500000);
  EXPECT_EQ(given.config->id, 4);
  EXPECT_EQ(given.config->start_id, -1);
  EXPECT_EQ(given.config->new_id, -1);

  EXPECT_EQ(
    only_error(
      parse_params(
        Tool::kFactoryReset,
        with_ids(Tool::kFactoryReset, {{"new_id", ParameterValue(int64_t{1})}}))),
    "parameter 'new_id' is not a parameter of factory_reset, which takes: baudrate, id, port; "
    "refusing to run rather than ignore it");
}

TEST(ToolParams, scan_takes_no_id)
{
  // scan always covers 0..253 [Q5]: an id given to it is an unknown name, not a narrowed scan.
  EXPECT_THAT(
    only_error(parse_params(Tool::kScan, {{"id", ParameterValue(int64_t{3})}})),
    StartsWith("parameter 'id' is not a parameter of scan"));
}

TEST(ToolParams, equal_ids_are_refused)
{
  EXPECT_EQ(
    only_error(
      parse_params(
        Tool::kSetId, {{"start_id", ParameterValue(int64_t{4})},
          {"new_id", ParameterValue(int64_t{4})}})),
    "start_id and new_id are both 4; nothing to change");
}

TEST(ToolParams, all_errors_are_reported_together)
{
  // One round trip per mistake would be five runs here; every error comes back at once, sorted
  // by the parameter it is about, and none of them yields a config.
  const ParseResult result = parse_params(
    Tool::kSetId, {{"start_id", ParameterValue(int64_t{254})},
      {"device_port", ParameterValue(std::string("/dev/ttyUSB0"))},
      {"foo", ParameterValue(int64_t{1})}, {"baudrate", ParameterValue(1.5)}});
  EXPECT_FALSE(result.config.has_value());
  ASSERT_EQ(result.errors.size(), 5u) << ::testing::PrintToString(result.errors);
  EXPECT_THAT(result.errors[0], StartsWith("parameter 'baudrate' is 1.5 (a double)"));
  EXPECT_THAT(result.errors[1], StartsWith("parameter 'device_port' was renamed"));
  EXPECT_THAT(result.errors[2], StartsWith("parameter 'foo' is not a parameter of set_id"));
  EXPECT_THAT(result.errors[3], StartsWith("parameter 'new_id' is missing"));
  EXPECT_THAT(result.errors[4], StartsWith("parameter 'start_id' is 254, out of range"));
}

TEST(ToolParams, overrides_for_another_node_are_refused)
{
  // rclcpp drops an override for another node name without a word, and the tool would then run
  // on the defaults. Only the two global patterns and this node's own name are let through.
  EXPECT_THAT(
    foreign_override_nodes(
      "/set_id", {{"/**", {"port"}}, {"/*", {"baudrate"}}, {"/set_id", {"port"}},
        {"set_id", {"start_id"}}}),
    IsEmpty());
  const std::vector<std::string> refused = foreign_override_nodes(
    "/set_id", {{"setid", {"port"}}, {"/scan", {"baudrate", "port"}}, {"/set_id/x", {"new_id"}},
      {"/set*", {"start_id"}}});
  ASSERT_EQ(refused.size(), 4u) << ::testing::PrintToString(refused);
  // in node-name order: "/scan" < "/set*" < "/set_id/x" < "setid"
  EXPECT_EQ(
    refused[0],
    "parameter override(s) 'baudrate', 'port' are addressed to node '/scan', but this tool's "
    "node is '/set_id'; rclcpp would drop them silently and use the defaults, so refusing to "
    "run. Give parameters without a node prefix: -p port:=...");
  EXPECT_THAT(refused[1], AllOf(HasSubstr("'start_id'"), HasSubstr("node '/set*'")));
  EXPECT_THAT(refused[2], AllOf(HasSubstr("'new_id'"), HasSubstr("node '/set_id/x'")));
  EXPECT_THAT(refused[3], AllOf(HasSubstr("'port'"), HasSubstr("node 'setid'")));
  for (const std::string & message : refused) {
    EXPECT_THAT(message, HasSubstr("this tool's node is '/set_id'"));
  }
}

TEST(ToolParams, a_single_star_override_is_accepted_only_for_a_top_level_node)
{
  // Review fix F11. rclcpp turns `/*` into the pattern (/\w+) and matches it against the WHOLE
  // fully qualified name, so it reaches /set_id and not /robot/set_id -- pinned against rclcpp
  // itself by ToolParamsRclcpp.a_single_star_override_misses_a_namespaced_node. `/**` reaches
  // both. A `-r __ns:=/robot` would otherwise let a `/*` port be dropped and the default used.
  EXPECT_THAT(foreign_override_nodes("/set_id", {{"/*", {"port"}}}), IsEmpty());
  EXPECT_THAT(foreign_override_nodes("/robot/set_id", {{"/**", {"port"}}}), IsEmpty());
  EXPECT_THAT(foreign_override_nodes("/robot/set_id", {{"/robot/set_id", {"port"}}}), IsEmpty());
  const std::vector<std::string> refused =
    foreign_override_nodes("/robot/set_id", {{"/*", {"port"}}});
  ASSERT_EQ(refused.size(), 1u) << ::testing::PrintToString(refused);
  EXPECT_EQ(
    refused[0],
    "parameter override(s) 'port' are addressed to node '/*', but this tool's node is "
    "'/robot/set_id'; rclcpp would drop them silently and use the defaults, so refusing to run. "
    "Give parameters without a node prefix: -p port:=...");
}

TEST(ToolParams, every_tool_has_a_name_and_its_usage_line)
{
  // The names are the executables' and the nodes' (A.1 step 4); the usage lines are printed with
  // every exit 64 and are the README's command lines.
  EXPECT_STREQ(name_of(Tool::kScan), "scan");
  EXPECT_STREQ(name_of(Tool::kSetId), "set_id");
  EXPECT_STREQ(name_of(Tool::kCalibrateMidpoint), "calibrate_midpoint");
  EXPECT_STREQ(name_of(Tool::kFactoryReset), "factory_reset");
  EXPECT_EQ(
    usage(Tool::kScan),
    "usage: ros2 run waveshare_servos scan [--ros-args -p port:=/dev/ttyACM0 -p "
    "baudrate:=1000000]");
  EXPECT_EQ(
    usage(Tool::kSetId),
    "usage: ros2 run waveshare_servos set_id --ros-args -p start_id:=<0..253> -p "
    "new_id:=<1..253> [-p port:=...] [-p baudrate:=...]");
  EXPECT_EQ(
    usage(Tool::kCalibrateMidpoint),
    "usage: ros2 run waveshare_servos calibrate_midpoint --ros-args -p id:=<0..253> "
    "[-p port:=...] [-p baudrate:=...]");
  EXPECT_EQ(
    usage(Tool::kFactoryReset),
    "usage: ros2 run waveshare_servos factory_reset --ros-args -p id:=<0..253> "
    "[-p port:=...] [-p baudrate:=...]");
}

namespace
{

// The two cases below need a live rclcpp context: they pin the library behaviour the parser's
// callers rely on. The context is created once for the suite and never installs signal handlers.
class ToolParamsRclcpp : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(
      0, nullptr, rclcpp::InitOptions().auto_initialize_logging(false),
      rclcpp::SignalHandlerOptions::None);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

}  // namespace

TEST_F(ToolParamsRclcpp, overrides_from_ros_args_contain_stale_names)
{
  // A.1 step 4 reads get_parameter_overrides() and drops every key the node itself declared. For
  // that to refuse `device_port`, rclcpp must hand the override over under its own name even
  // though nothing declares it -- and it must list use_sim_time, which rclcpp declares.
  const auto node = std::make_shared<rclcpp::Node>(
    "set_id", rclcpp::NodeOptions()
    .arguments({"--ros-args", "-p", "device_port:=/x", "-p", "use_sim_time:=false"})
    .start_parameter_services(false).enable_rosout(false));
  const std::map<std::string, ParameterValue> & all =
    node->get_node_parameters_interface()->get_parameter_overrides();
  ASSERT_EQ(all.count("device_port"), 1u);
  EXPECT_EQ(all.at("device_port").get_type(), rclcpp::ParameterType::PARAMETER_STRING);
  EXPECT_EQ(all.at("device_port").get<std::string>(), "/x");
  ASSERT_EQ(all.count("use_sim_time"), 1u);
  EXPECT_TRUE(node->has_parameter("use_sim_time")) << "rclcpp declares it itself";
  EXPECT_FALSE(node->has_parameter("device_port"));

  Overrides mine;
  for (const auto & entry : all) {
    if (!node->has_parameter(entry.first)) {
      mine.insert(entry);
    }
  }
  const ParseResult result = parse_params(Tool::kSetId, with_ids(Tool::kSetId, mine));
  EXPECT_FALSE(result.config.has_value());
  EXPECT_THAT(result.errors, ElementsAre(HasSubstr("'device_port' was renamed to 'port'")));
}

TEST_F(ToolParamsRclcpp, global_overrides_keep_their_node_name)
{
  // rclcpp would drop `-p setid:port:=/x` for a node called set_id. rcl still has it, under the
  // node name exactly as typed -- measured: "setid", with no leading slash -- and an unprefixed
  // rule under "/**". Both forms are what foreign_override_nodes() is written against.
  const char * argv[] = {"set_id", "--ros-args", "-p", "setid:port:=/x", "-p", "port:=/y"};
  auto context = std::make_shared<rclcpp::Context>();
  context->init(6, argv, rclcpp::InitOptions().auto_initialize_logging(false));
  rcl_params_t * params = nullptr;
  ASSERT_EQ(
    rcl_arguments_get_param_overrides(&context->get_rcl_context()->global_arguments, &params),
    RCL_RET_OK);
  ASSERT_NE(params, nullptr);
  std::map<std::string, std::vector<std::string>> keys_by_node;
  for (size_t n = 0; n < params->num_nodes; n++) {
    std::vector<std::string> & keys = keys_by_node[params->node_names[n]];
    for (size_t k = 0; k < params->params[n].num_params; k++) {
      keys.push_back(params->params[n].parameter_names[k]);
    }
  }
  rcl_yaml_node_struct_fini(params);
  context->shutdown("test done");

  EXPECT_EQ(
    keys_by_node, (std::map<std::string, std::vector<std::string>>{
    {"/**", {"port"}}, {"setid", {"port"}}}));
  const std::vector<std::string> refused = foreign_override_nodes("/set_id", keys_by_node);
  ASSERT_EQ(refused.size(), 1u) << ::testing::PrintToString(refused);
  EXPECT_THAT(refused[0], AllOf(HasSubstr("node 'setid'"), HasSubstr("'port'")));
  EXPECT_THAT(refused[0], Not(HasSubstr("'/**'")));
}

TEST_F(ToolParamsRclcpp, a_single_star_override_misses_a_namespaced_node)
{
  // Measured, not assumed (review fix F11): the same params file reaches a top-level node through
  // `/*` and does not reach that node under a namespace, while `/**` reaches both. This is the
  // rclcpp behaviour foreign_override_nodes() mirrors for `/*`.
  const std::string path = (std::filesystem::temp_directory_path() /
    ("test_tool_params_star_" + std::to_string(::getpid()) + ".yaml")).string();
  {
    std::ofstream yaml(path);
    yaml << "/*:\n  ros__parameters:\n    port: /dev/ttyUSB1\n"
      "/**:\n  ros__parameters:\n    baudrate: 57600\n";
  }
  const auto options = [&path]() {
      return rclcpp::NodeOptions().arguments({"--ros-args", "--params-file", path})
             .start_parameter_services(false).enable_rosout(false);
    };
  const auto top = std::make_shared<rclcpp::Node>("set_id", options());
  const auto nested = std::make_shared<rclcpp::Node>("set_id", "/robot", options());
  std::filesystem::remove(path);
  ASSERT_EQ(std::string(nested->get_fully_qualified_name()), "/robot/set_id");
  const auto & top_overrides = top->get_node_parameters_interface()->get_parameter_overrides();
  const auto & nested_overrides =
    nested->get_node_parameters_interface()->get_parameter_overrides();
  EXPECT_EQ(top_overrides.count("port"), 1u) << "/* reaches a node at the top level";
  EXPECT_EQ(nested_overrides.count("port"), 0u) << "and not the same node under /robot";
  EXPECT_EQ(top_overrides.count("baudrate"), 1u);
  EXPECT_EQ(nested_overrides.count("baudrate"), 1u) << "/** reaches both";
}
