// Validated getters over a ros2_control <param> block, plus the compiled-in defaults.
//
// Driver-owned and header-only, and under src/ on purpose: install(DIRECTORY include/ ...)
// (CMakeLists.txt) must not pick it up, because it is an implementation detail rather than part of
// the package's public interface (PHASE2_SPEC 3.7, 4.2).
//
// One helper serves both parameter blocks: HardwareInfo::hardware_parameters and
// ComponentInfo::parameters are the same type, std::unordered_map<std::string, std::string>
// (hardware_info.hpp:391 and :111), so the hardware parameters of PHASE2_SPEC 4 and the joint
// parameters of PHASE2_SPEC 5 read the same way and produce the same five message templates.
//
// Every getter reports a Status and leaves the caller's value untouched unless it returns kOk, so
// a compiled-in default survives an absent parameter and a bad value can never half-apply. The
// conversions are hardware_interface's, not std::sto*: hardware_interface::stod is
// locale-independent, rejects trailing characters and rejects a non-finite result, which matters
// because NaN compares false against both range bounds and would otherwise slip through.

#ifndef PARAM_PARSING_HPP_
#define PARAM_PARSING_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "driver_defaults.hpp"
#include "hardware_interface/lexical_casts.hpp"

namespace waveshare_servos
{
namespace params
{

using ParameterMap = std::unordered_map<std::string, std::string>;

// What a getter found. kDefaulted is the absent case: it is an error only for a required
// parameter (id, PHASE2_SPEC 5.1), which tests `st != kOk`, while an optional parameter tests
// `st != kOk && st != kDefaulted` and keeps its default.
enum class Status
{
  kOk,
  kDefaulted,
  kEmpty,
  kMalformed,
  kOutOfRange
};

namespace detail
{

// Values arriving from the URDF are already stripped by parse_parameters_from_xml
// (component_parser.cpp:366-368); stripping again keeps the getters honest when a test, or a
// future caller, injects a map directly.
inline std::string strip(const std::string & text)
{
  const auto first = text.find_first_not_of(" \t\n\v\f\r");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = text.find_last_not_of(" \t\n\v\f\r");
  return text.substr(first, last - first + 1);
}

// The prologue every getter shares: absent -> kDefaulted, blank -> kEmpty, otherwise kOk with the
// stripped text in `text`.
inline Status lookup(const ParameterMap & p, const std::string & name, std::string & text)
{
  const auto it = p.find(name);
  if (it == p.end()) {
    return Status::kDefaulted;
  }
  text = strip(it->second);
  if (text.empty()) {
    return Status::kEmpty;
  }
  return Status::kOk;
}

}  // namespace detail

// The declared value as the messages quote it back: stripped, and empty when the parameter is
// absent.
inline std::string raw(const ParameterMap & p, const std::string & name)
{
  const auto it = p.find(name);
  if (it == p.end()) {
    return "";
  }
  return detail::strip(it->second);
}

inline Status get_string(const ParameterMap & p, const std::string & name, std::string & value)
{
  std::string text;
  const Status st = detail::lookup(p, name, text);
  if (st != Status::kOk) {
    return st;
  }
  value = text;
  return Status::kOk;
}

inline Status get_bool(const ParameterMap & p, const std::string & name, bool & value)
{
  std::string text;
  const Status st = detail::lookup(p, name, text);
  if (st != Status::kOk) {
    return st;
  }
  bool parsed = false;
  try {
    // parse_bool accepts exactly "true"/"false", case-insensitively (lexical_casts.hpp:108), so
    // the XML habits 1/0 and yes/no are malformed here.
    parsed = hardware_interface::parse_bool(text);
  } catch (const std::out_of_range &) {
    // Unreachable through parse_bool today; caught first so the three getters read alike and a
    // future conversion cannot report an overflow as a malformed value.
    return Status::kOutOfRange;
  } catch (const std::invalid_argument &) {
    return Status::kMalformed;
  }
  value = parsed;
  return Status::kOk;
}

// `min` and `max` are inclusive. A value that is well formed but outside them is kOutOfRange, the
// same verdict std::stol's own overflow gets, so the caller needs one branch for both.
inline Status get_int(
  const ParameterMap & p, const std::string & name, int64_t min, int64_t max, int64_t & value)
{
  std::string text;
  const Status st = detail::lookup(p, name, text);
  if (st != Status::kOk) {
    return st;
  }
  int64_t parsed = 0;
  try {
    parsed = hardware_interface::stoi_generic<int64_t>(text);
  } catch (const std::out_of_range &) {
    // Order matters: std::out_of_range derives from std::logic_error, not from
    // std::invalid_argument, but catching the other way round would still be wrong the day one of
    // them starts deriving -- and reads as if malformed and out of range were the same thing.
    return Status::kOutOfRange;
  } catch (const std::invalid_argument &) {
    return Status::kMalformed;
  }
  if (parsed < min || parsed > max) {
    return Status::kOutOfRange;
  }
  value = parsed;
  return Status::kOk;
}

// `max` is inclusive; `min` is inclusive unless min_exclusive, in which case the predicate is
// literally `v > min`. There is deliberately no 1e-9 sentinel: the message ("a number greater
// than 0 and at most 1") and the predicate would then be able to drift apart.
inline Status get_double(
  const ParameterMap & p, const std::string & name, double min, double max, bool min_exclusive,
  double & value)
{
  std::string text;
  const Status st = detail::lookup(p, name, text);
  if (st != Status::kOk) {
    return st;
  }
  double parsed = 0.0;
  try {
    parsed = hardware_interface::stod(text);
  } catch (const std::out_of_range &) {
    return Status::kOutOfRange;
  } catch (const std::invalid_argument &) {
    // stod throws this for "1.0f" and "1,5" (trailing characters) and for "nan" and "inf"
    // (non-finite), so no non-finite value ever reaches the range check below.
    return Status::kMalformed;
  }
  if ((min_exclusive ? parsed <= min : parsed < min) || parsed > max) {
    return Status::kOutOfRange;
  }
  value = parsed;
  return Status::kOk;
}

// The five templates of PHASE2_SPEC 4.2, and the only place they are spelled out. `subject` is
// "hardware parameter '<name>'" or "joint '<joint>' parameter '<name>'"; `raw_value` is raw()
// above; `expected` completes the sentence ("an integer between 1 and 10"). Always logged as
// RCLCPP_FATAL(logger, "%s", msg.c_str()), never as a format string, so a value containing a '%'
// cannot corrupt the call.
inline std::string message(
  Status st, const std::string & subject, const std::string & raw_value,
  const std::string & expected)
{
  switch (st) {
    case Status::kDefaulted:
      return subject + " is missing; expected " + expected;
    case Status::kEmpty:
      return subject + " is empty; expected " + expected;
    case Status::kMalformed:
      return subject + " is '" + raw_value + "', which is not " + expected;
    case Status::kOutOfRange:
      return subject + " is '" + raw_value + "', which is out of range; expected " + expected;
    case Status::kOk:
      break;
  }
  return "";
}

}  // namespace params

}  // namespace waveshare_servos

#endif  // PARAM_PARSING_HPP_
