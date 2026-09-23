#include "tool_params.hpp"

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "driver_defaults.hpp"
#include "servo_bus.hpp"

namespace waveshare_servos
{
namespace tools
{
namespace
{

using Overrides = std::map<std::string, rclcpp::ParameterValue>;

// One refusal and the parameter it is about: the errors are sorted by that name before they are
// returned, so the output does not depend on the order the checks happen to run in.
using Error = std::pair<std::string, std::string>;

// The names each tool accepts, sorted: the unknown-name message lists them as they stand here.
std::vector<std::string> accepted_names(Tool tool)
{
  switch (tool) {
    case Tool::kScan:
      return {"baudrate", "port"};
    case Tool::kSetId:
      return {"baudrate", "new_id", "port", "start_id"};
    case Tool::kCalibrateMidpoint:
      return {"baudrate", "id", "port"};
    case Tool::kFactoryReset:
      return {"baudrate", "id", "port"};
  }
  return {};
}

std::string joined(const std::vector<std::string> & items, const std::string & separator)
{
  std::string text;
  for (std::size_t i = 0; i < items.size(); i++) {
    text += (i == 0 ? "" : separator) + items[i];
  }
  return text;
}

// A double as a person typed it: 1e6 prints 1000000.0 and 4.0 prints 4.0, never "1e+06" or
// "4.000000", so the message shows why the value is not an integer.
std::string double_text(double value)
{
  std::ostringstream text;
  text << std::setprecision(15) << value;
  std::string out = text.str();
  if (out.find_first_not_of("-0123456789") == std::string::npos) {
    out += ".0";
  }
  return out;
}

// "<value> (<a type>)" for the type-mismatch message. Strings are quoted, so an empty one or one
// with spaces is still visible.
std::string value_and_type(const rclcpp::ParameterValue & value)
{
  switch (value.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return std::string(value.get<bool>() ? "true" : "false") + " (a bool)";
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return std::to_string(value.get<int64_t>()) + " (an integer)";
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return double_text(value.get<double>()) + " (a double)";
    case rclcpp::ParameterType::PARAMETER_STRING:
      return "'" + value.get<std::string>() + "' (a string)";
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
      return rclcpp::to_string(value) + " (a byte array)";
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      return rclcpp::to_string(value) + " (a bool array)";
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      return rclcpp::to_string(value) + " (an integer array)";
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      return rclcpp::to_string(value) + " (a double array)";
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      return rclcpp::to_string(value) + " (a string array)";
    case rclcpp::ParameterType::PARAMETER_NOT_SET:
      return "not set (no value)";
  }
  return rclcpp::to_string(value);
}

std::string not_a(const std::string & name, const rclcpp::ParameterValue & value, const char * what)
{
  return "parameter '" + name + "' is " + value_and_type(value) + ", which is not " + what;
}

// An id with its range, checked as int64 BEFORE any narrowing: 300 would otherwise become 44, 254
// the broadcast id and -1 the header byte 0xff (A.1, "Name policy").
void take_id(
  const Overrides & overrides, const std::string & name, int low, const char * range_note,
  int * out, std::vector<Error> * errors)
{
  const std::string range =
    "expected an integer between " + std::to_string(low) + " and " + std::to_string(limits::kIdMax);
  const auto it = overrides.find(name);
  if (it == overrides.end()) {
    errors->emplace_back(name, "parameter '" + name + "' is missing; " + range);
    return;
  }
  if (it->second.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
    errors->emplace_back(name, not_a(name, it->second, "an integer"));
    return;
  }
  const int64_t value = it->second.get<int64_t>();
  if (value < low || value > limits::kIdMax) {
    std::string note;
    if (range_note != nullptr) {
      note = std::string(" (") + range_note + ")";
    } else if (value == 254) {
      note = " (254 is the broadcast id, which every servo obeys and none answers)";
    }
    errors->emplace_back(
      name, "parameter '" + name + "' is " + std::to_string(value) + ", out of range; " + range +
      note);
    return;
  }
  *out = static_cast<int>(value);
}

}  // namespace

const char * name_of(Tool tool) noexcept
{
  // No `default:` label, so a new Tool without a name here is a -Wswitch warning.
  switch (tool) {
    case Tool::kScan:
      return "scan";
    case Tool::kSetId:
      return "set_id";
    case Tool::kCalibrateMidpoint:
      return "calibrate_midpoint";
    case Tool::kFactoryReset:
      return "factory_reset";
  }
  return "unknown";
}

ParseResult parse_params(Tool tool, const Overrides & overrides)
{
  std::vector<Error> errors;
  ToolConfig config;
  config.port = defaults::kPort;
  config.baudrate = defaults::kBaudrate;

  // Names first. The two Phase 1 names are refused even next to their replacement: a command line
  // that still says device_port was written for the old tool and may mean something else.
  const std::vector<std::string> accepted = accepted_names(tool);
  for (const auto & entry : overrides) {
    const std::string & name = entry.first;
    if (name == "device_port") {
      errors.emplace_back(
        name, "parameter 'device_port' was renamed to 'port', the name the hardware interface "
        "uses; refusing to run rather than ignore it and open the default port '" +
        std::string(defaults::kPort) + "'");
    } else if (name == "baud_rate") {
      errors.emplace_back(
        name, "parameter 'baud_rate' was renamed to 'baudrate'; refusing to run rather than "
        "ignore it and use the default " + std::to_string(defaults::kBaudrate));
    } else if (std::find(accepted.begin(), accepted.end(), name) == accepted.end()) {
      errors.emplace_back(
        name, "parameter '" + name + "' is not a parameter of " + name_of(tool) +
        ", which takes: " + joined(accepted, ", ") + "; refusing to run rather than ignore it");
    }
  }

  // Types are never coerced: `port:=0` is an integer to rcl and `baudrate:=1e6` a double, and
  // either is refused rather than converted into something the user did not type.
  const auto port = overrides.find("port");
  if (port != overrides.end()) {
    if (port->second.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      errors.emplace_back("port", not_a("port", port->second, "a string"));
    } else if (port->second.get<std::string>().empty()) {
      errors.emplace_back(
        "port", "parameter 'port' is empty; expected a device path such as '" +
        std::string(defaults::kPort) + "'");
    } else {
      config.port = port->second.get<std::string>();
    }
  }

  const auto baudrate = overrides.find("baudrate");
  if (baudrate != overrides.end()) {
    if (baudrate->second.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
      errors.emplace_back("baudrate", not_a("baudrate", baudrate->second, "an integer"));
    } else {
      // The int range first, for the driver's reason (waveshare_servos.cpp:197-199): 2^32 + 9600
      // narrowed to an int is 9600, a rate that IS mapped.
      const int64_t value = baudrate->second.get<int64_t>();
      if (value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max() ||
        !ServoBus::is_supported_baudrate(static_cast<int>(value)))
      {
        errors.emplace_back(
          "baudrate", "parameter 'baudrate' is '" + std::to_string(value) + "'; the servo "
          "library maps only 9600, 19200, 38400, 57600, 115200, 500000 and 1000000, and "
          "silently falls back to 115200 for anything else");
      } else {
        config.baudrate = static_cast<int>(value);
      }
    }
  }

  // The ids are required [Q4]: the old defaults of 1 made a bare run address servo 1.
  if (tool == Tool::kSetId) {
    const std::size_t before = errors.size();
    take_id(overrides, "start_id", limits::kIdMin0, nullptr, &config.start_id, &errors);
    take_id(
      overrides, "new_id", limits::kIdMin, "the hardware interface accepts ids 1..253",
      &config.new_id, &errors);
    if (errors.size() == before && config.start_id == config.new_id) {
      errors.emplace_back(
        "start_id", "start_id and new_id are both " + std::to_string(config.start_id) +
        "; nothing to change");
    }
  } else if (tool == Tool::kCalibrateMidpoint || tool == Tool::kFactoryReset) {
    take_id(overrides, "id", limits::kIdMin0, nullptr, &config.id, &errors);
  }

  ParseResult result;
  if (errors.empty()) {
    result.config = config;
    return result;
  }
  std::stable_sort(
    errors.begin(), errors.end(),
    [](const Error & a, const Error & b) {return a.first < b.first;});
  for (const Error & error : errors) {
    result.errors.push_back(error.second);
  }
  return result;
}

std::vector<std::string> foreign_override_nodes(
  const std::string & node_fqn,
  const std::map<std::string, std::vector<std::string>> & keys_by_node)
{
  // Only the two global patterns and the exact name: reproducing rclcpp's wildcard matching would
  // be a second implementation of it that could drift from the first (appendix J, #11). rcl keeps
  // `-p setid:port:=...` under "setid", with no leading slash, hence the normalisation. `/*` is
  // rclcpp's pattern (/\w+) matched against the whole name, so it reaches a node with ONE path
  // segment only: under a namespace (`-r __ns:=/robot`) rclcpp drops it, and so it is refused
  // here like any other name (review fix F11; ToolParamsRclcpp pins rclcpp's side).
  const bool top_level = node_fqn.find('/', 1) == std::string::npos;
  std::vector<std::string> refusals;
  for (const auto & entry : keys_by_node) {
    const std::string & name = entry.first;
    const std::string absolute = (!name.empty() && name.front() == '/') ? name : "/" + name;
    if (absolute == "/**" || (absolute == "/*" && top_level) || absolute == node_fqn) {
      continue;
    }
    std::vector<std::string> keys = entry.second;
    std::sort(keys.begin(), keys.end());
    for (std::string & key : keys) {
      key = "'" + key + "'";
    }
    refusals.push_back(
      "parameter override(s) " + (keys.empty() ? std::string("(none)") : joined(keys, ", ")) +
      " are addressed to node '" + name + "', but this tool's node is '" + node_fqn +
      "'; rclcpp would drop them silently and use the defaults, so refusing to run. Give "
      "parameters without a node prefix: -p port:=...");
  }
  return refusals;
}

std::string usage(Tool tool)
{
  switch (tool) {
    case Tool::kScan:
      return "usage: ros2 run waveshare_servos scan [--ros-args -p port:=/dev/ttyACM0 -p "
             "baudrate:=1000000]";
    case Tool::kSetId:
      return "usage: ros2 run waveshare_servos set_id --ros-args -p start_id:=<0..253> -p "
             "new_id:=<1..253> [-p port:=...] [-p baudrate:=...]";
    case Tool::kCalibrateMidpoint:
      return "usage: ros2 run waveshare_servos calibrate_midpoint --ros-args -p id:=<0..253> "
             "[-p port:=...] [-p baudrate:=...]";
    case Tool::kFactoryReset:
      return "usage: ros2 run waveshare_servos factory_reset --ros-args -p id:=<0..253> "
             "[-p port:=...] [-p baudrate:=...]";
  }
  return "";
}

}  // namespace tools
}  // namespace waveshare_servos
