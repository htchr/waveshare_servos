// tool_params: a pure parser over the overrides rclcpp resolved; each is applied or refused.
// See docs/design.md, "Tool structure".

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
// the ones it takes are in range, so narrowing them to uint8_t cannot reach 0xfe or 0xff.
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

// Pure. One refusal per node name that is not "/**", node_fqn, or "/*" for a node with no
// namespace (a leading '/' is added to a name that lacks one). keys_by_node: node name -> the
// parameter names under it. The messages come in node-name order.
std::vector<std::string> foreign_override_nodes(
  const std::string & node_fqn,
  const std::map<std::string, std::vector<std::string>> & keys_by_node);

// The one-line usage printed on stderr with every exit 64, without a newline.
std::string usage(Tool tool);

}  // namespace tools
}  // namespace waveshare_servos

#endif  // TOOL_PARAMS_HPP_
