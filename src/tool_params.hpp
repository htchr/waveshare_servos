// tool_params -- the Phase 6 tools' parameter surface: a pure parser over rclcpp's overrides.
//
// The tools stay `ros2 run ... --ros-args -p name:=value` executables, but they never call
// declare_parameter: a wrongly typed value would then be an uncaught throw, an unknown name would
// be ignored, and an old command line (`device_port`, `baud_rate`) would open the default port.
// The overrides rclcpp resolved for the tool's node are handed here instead, and every one of them
// is either applied or refused (PHASE6_SPEC A.1, R1). Pure: nothing here needs rclcpp::init, which
// is what lets test/test_tool_params.cpp build the maps by hand.
//
// Under src/ and not installed, like param_parsing.hpp. rclcpp is used for ParameterValue only.

#ifndef TOOL_PARAMS_HPP_
#define TOOL_PARAMS_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/parameter_value.hpp"

namespace waveshare_servos
{
namespace tools
{

enum class Tool : uint8_t {kScan, kSetId, kCalibrateMidpoint, kFactoryReset};

// "scan" | "set_id" | "calibrate_midpoint" | "factory_reset": the executable's name, and its
// node's.
const char * name_of(Tool tool) noexcept;

// What a tool runs with once its parameters were accepted. The ids a tool does not take stay -1;
// the ones it takes are in range (A.2), so narrowing them to uint8_t cannot reach 0xfe or 0xff.
struct ToolConfig
{
  std::string port;
  int baudrate = 0;
  int id = -1;
  int start_id = -1;
  int new_id = -1;
};

struct ParseResult
{
  std::optional<ToolConfig> config;
  std::vector<std::string> errors;
};

// Pure: no rclcpp::init needed. errors non-empty <=> config empty. ALL errors are collected, so a
// command line is fixed in one round, and they are sorted by the parameter they are about. A
// message carries no "<tool>: " prefix; the caller prints one line per error.
ParseResult parse_params(
  Tool tool, const std::map<std::string, rclcpp::ParameterValue> & overrides);

// Pure. One refusal message per node name that is not "/**", "/*" or node_fqn (a leading '/' is
// added to a name that lacks one). keys_by_node: node name -> the parameter names under it. The
// messages come in node-name order.
std::vector<std::string> foreign_override_nodes(
  const std::string & node_fqn,
  const std::map<std::string, std::vector<std::string>> & keys_by_node);

// The one-line usage printed on stderr with every exit 64 (A.2), without a newline.
std::string usage(Tool tool);

}  // namespace tools
}  // namespace waveshare_servos

#endif  // TOOL_PARAMS_HPP_
