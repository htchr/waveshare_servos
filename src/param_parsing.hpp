// Validated getters over a ros2_control <param> map; each writes `value` only on kOk. Under src/
// so it is not installed. See docs/configuration.md, "Parameter values".

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

// What a getter found. kDefaulted (absent) is an error only for the required id; an optional
// parameter accepts it and keeps its default.
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

// ros2_control already strips values from the URDF; strip again for a map built directly (tests).
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
    // parse_bool accepts only "true" or "false", in any case: 1/0 and yes/no are malformed.
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
    // Caught before invalid_argument on purpose: an overflow is kOutOfRange, never kMalformed.
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
    // Jazzy's stod throws this for trailing characters ("1.0f"), "nan", "inf" and an overflow
    // ("1e999"), so no non-finite value reaches the range check below.
    return Status::kMalformed;
  }
  if ((min_exclusive ? parsed <= min : parsed < min) || parsed > max) {
    return Status::kOutOfRange;
  }
  value = parsed;
  return Status::kOk;
}

// The FATAL text for a rejected parameter: missing, empty, malformed or out of range ("" for kOk).
// `subject` names the parameter, `raw_value` is raw(), `expected` completes the sentence.
// Log it as RCLCPP_FATAL(logger, "%s", msg.c_str()), so a '%' in the value is harmless.
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
