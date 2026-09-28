// Unit tests for src/param_parsing.hpp: no URDF, bus or motors. A failed getter must leave the
// caller's value unchanged. See docs/configuration.md, "Parameter values".

#include <gmock/gmock.h>

#include <clocale>
#include <cstdint>
#include <limits>
#include <string>

#include "param_parsing.hpp"

namespace
{

// Per-name declarations, never a using-directive: cpplint's build/namespaces rule forbids
// using-directives outside a short std::*_literals whitelist, in sources as well as headers.
using waveshare_servos::defaults::kAllowMissingServos;
using waveshare_servos::defaults::kBaudrate;
using waveshare_servos::defaults::kPort;
using waveshare_servos::defaults::kTorqueConstantNmPerA;
using waveshare_servos::params::ParameterMap;
using waveshare_servos::params::Status;
using waveshare_servos::params::get_bool;
using waveshare_servos::params::get_double;
using waveshare_servos::params::get_int;
using waveshare_servos::params::get_string;
using waveshare_servos::params::message;
using waveshare_servos::params::raw;

constexpr double kInf = std::numeric_limits<double>::infinity();

// The locales tried by double_is_locale_independent. A stock ROS container has only C and C.utf8,
// which is why that case skips rather than fails.
const char * const kCommaDecimalLocales[] = {
  "de_DE.UTF-8", "de_DE.utf8", "fr_FR.UTF-8", "fr_FR.utf8", "es_ES.UTF-8", "es_ES.utf8"};

TEST(ParamParsing, missing_parameter_reports_defaulted_and_keeps_the_default)
{
  const ParameterMap absent;

  // An absent key returns kDefaulted and keeps the default from waveshare_servos::defaults.
  std::string port = kPort;
  EXPECT_EQ(get_string(absent, "port", port), Status::kDefaulted);
  EXPECT_EQ(port, "/dev/ttyACM0");

  int64_t baudrate = kBaudrate;
  EXPECT_EQ(get_int(absent, "baudrate", INT64_MIN, INT64_MAX, baudrate), Status::kDefaulted);
  EXPECT_EQ(baudrate, 1000000);

  bool allow_missing_servos = kAllowMissingServos;
  EXPECT_EQ(get_bool(absent, "allow_missing_servos", allow_missing_servos), Status::kDefaulted);
  EXPECT_FALSE(allow_missing_servos);

  double torque_constant = kTorqueConstantNmPerA;
  EXPECT_EQ(
    get_double(absent, "torque_constant_nm_per_a", 0.0, 100.0, true, torque_constant),
    Status::kDefaulted);
  EXPECT_EQ(torque_constant, 0.8825985);

  // raw() of an absent key is the empty string: message() interpolates it unconditionally.
  EXPECT_EQ(raw(absent, "port"), "");
}

TEST(ParamParsing, empty_parameter_reports_empty)
{
  const ParameterMap p{
    {"port", ""}, {"baudrate", ""}, {"allow_missing_servos", ""}, {"current_per_count_a", ""}};

  std::string port = "/dev/ttyACM0";
  EXPECT_EQ(get_string(p, "port", port), Status::kEmpty);
  EXPECT_EQ(port, "/dev/ttyACM0");

  int64_t baudrate = 1000000;
  EXPECT_EQ(get_int(p, "baudrate", INT64_MIN, INT64_MAX, baudrate), Status::kEmpty);
  EXPECT_EQ(baudrate, 1000000);

  bool allow_missing_servos = false;
  EXPECT_EQ(get_bool(p, "allow_missing_servos", allow_missing_servos), Status::kEmpty);
  EXPECT_FALSE(allow_missing_servos);

  double current_per_count_a = 0.006;
  EXPECT_EQ(
    get_double(p, "current_per_count_a", 0.0, 1.0, true, current_per_count_a), Status::kEmpty);
  EXPECT_EQ(current_per_count_a, 0.006);
}

TEST(ParamParsing, whitespace_only_parameter_reports_empty)
{
  const ParameterMap p{
    {"port", " "}, {"baudrate", "\t"}, {"allow_missing_servos", "\n \t"},
    {"current_per_count_a", "   "}, {"io_timeout_ms", "  20  "}};

  std::string port = "/dev/ttyACM0";
  EXPECT_EQ(get_string(p, "port", port), Status::kEmpty);
  EXPECT_EQ(port, "/dev/ttyACM0");

  int64_t baudrate = 1000000;
  EXPECT_EQ(get_int(p, "baudrate", INT64_MIN, INT64_MAX, baudrate), Status::kEmpty);

  bool allow_missing_servos = false;
  EXPECT_EQ(get_bool(p, "allow_missing_servos", allow_missing_servos), Status::kEmpty);

  double current_per_count_a = 0.006;
  EXPECT_EQ(
    get_double(p, "current_per_count_a", 0.0, 1.0, true, current_per_count_a), Status::kEmpty);

  // A padded value parses, and raw() returns the stripped text that the FATAL messages quote.
  int64_t io_timeout_ms = 0;
  EXPECT_EQ(get_int(p, "io_timeout_ms", 2, 1000, io_timeout_ms), Status::kOk);
  EXPECT_EQ(io_timeout_ms, 20);
  EXPECT_EQ(raw(p, "io_timeout_ms"), "20");
  EXPECT_EQ(raw(p, "allow_missing_servos"), "");
}

TEST(ParamParsing, non_numeric_double_reports_malformed)
{
  const ParameterMap p{{"current_per_count_a", "six milliamps"}, {"offset", "zero"}};

  double current_per_count_a = 0.006;
  EXPECT_EQ(
    get_double(p, "current_per_count_a", 0.0, 1.0, true, current_per_count_a), Status::kMalformed);
  EXPECT_EQ(current_per_count_a, 0.006);

  double offset = 0.0;
  EXPECT_EQ(get_double(p, "offset", -kInf, kInf, false, offset), Status::kMalformed);
  EXPECT_EQ(offset, 0.0);
}

TEST(ParamParsing, nan_and_inf_doubles_report_malformed)
{
  // hardware_interface::stod rejects a non-finite result, so these never reach the range check --
  // which matters, because NaN compares false against both bounds and would otherwise be accepted.
  const ParameterMap p{
    {"a", "nan"}, {"b", "NaN"}, {"c", "-nan"}, {"d", "inf"}, {"e", "-inf"}, {"f", "infinity"}};

  for (const char * name : {"a", "b", "c", "d", "e", "f"}) {
    double value = 1.0;
    EXPECT_EQ(get_double(p, name, -kInf, kInf, false, value), Status::kMalformed) << name;
    EXPECT_EQ(value, 1.0) << name;
  }
}

TEST(ParamParsing, double_with_trailing_characters_reports_malformed)
{
  // "1,5" is the comma-decimal spelling: rejected, not silently read as 1.
  const ParameterMap p{{"a", "1.0f"}, {"b", "0.006A"}, {"c", "1,5"}, {"d", "1.5 rad"}};

  for (const char * name : {"a", "b", "c", "d"}) {
    double value = 2.0;
    EXPECT_EQ(get_double(p, name, -kInf, kInf, false, value), Status::kMalformed) << name;
    EXPECT_EQ(value, 2.0) << name;
  }
}

TEST(ParamParsing, double_below_min_reports_out_of_range)
{
  const ParameterMap p{{"current_per_count_a", "-0.5"}, {"torque_constant_nm_per_a", "-1e-9"}};

  double current_per_count_a = 0.006;
  EXPECT_EQ(
    get_double(p, "current_per_count_a", 0.0, 1.0, true, current_per_count_a),
    Status::kOutOfRange);
  EXPECT_EQ(current_per_count_a, 0.006);

  // The same verdict with an inclusive minimum: below is below either way.
  double torque_constant = 0.8825985;
  EXPECT_EQ(
    get_double(p, "torque_constant_nm_per_a", 0.0, 100.0, false, torque_constant),
    Status::kOutOfRange);
  EXPECT_EQ(torque_constant, 0.8825985);
}

TEST(ParamParsing, double_at_an_exclusive_min_reports_out_of_range)
{
  const ParameterMap p{{"zero", "0"}, {"zero_point", "0.0"}, {"tiny", "1e-300"}};

  // The exclusive predicate is literally `v > min`: exactly the minimum is rejected ...
  for (const char * name : {"zero", "zero_point"}) {
    double value = 0.006;
    EXPECT_EQ(get_double(p, name, 0.0, 1.0, true, value), Status::kOutOfRange) << name;
    EXPECT_EQ(value, 0.006) << name;

    // ... and accepted the moment the same bound is inclusive.
    double inclusive = 0.006;
    EXPECT_EQ(get_double(p, name, 0.0, 1.0, false, inclusive), Status::kOk) << name;
    EXPECT_EQ(inclusive, 0.0) << name;
  }

  // No 1e-9 sentinel: the smallest positive value a user can write is in range.
  double tiny = 0.006;
  EXPECT_EQ(get_double(p, "tiny", 0.0, 1.0, true, tiny), Status::kOk);
  EXPECT_EQ(tiny, 1e-300);
}

TEST(ParamParsing, double_above_max_reports_out_of_range)
{
  const ParameterMap p{{"a", "1.5"}, {"b", "1.0"}, {"c", "100.0000001"}};

  double above = 0.006;
  EXPECT_EQ(get_double(p, "a", 0.0, 1.0, true, above), Status::kOutOfRange);
  EXPECT_EQ(above, 0.006);

  // The maximum is inclusive for every hardware parameter.
  double at_max = 0.006;
  EXPECT_EQ(get_double(p, "b", 0.0, 1.0, true, at_max), Status::kOk);
  EXPECT_EQ(at_max, 1.0);

  double torque_constant = 0.8825985;
  EXPECT_EQ(
    get_double(p, "c", 0.0, 100.0, true, torque_constant), Status::kOutOfRange);
  EXPECT_EQ(torque_constant, 0.8825985);
}

TEST(ParamParsing, double_is_locale_independent)
{
  const char * const previous = std::setlocale(LC_ALL, nullptr);
  const std::string saved = previous != nullptr ? previous : "C";

  const char * applied = nullptr;
  for (const char * candidate : kCommaDecimalLocales) {
    applied = std::setlocale(LC_ALL, candidate);
    if (applied != nullptr) {
      break;
    }
  }
  if (applied == nullptr) {
    // setlocale leaves the locale untouched when it fails, so there is nothing to restore.
    GTEST_SKIP() << "no comma-decimal locale is installed (tried de_DE.UTF-8, de_DE.utf8, "
      "fr_FR.UTF-8, fr_FR.utf8, es_ES.UTF-8, es_ES.utf8); a stock ROS image carries "
      "only C and C.utf8, so locale independence cannot be exercised here";
  }

  const ParameterMap p{{"dot", "1.5"}, {"comma", "1,5"}};

  // Under a comma-decimal locale the C-locale spelling must still parse ...
  double dot = 0.0;
  EXPECT_EQ(get_double(p, "dot", -kInf, kInf, false, dot), Status::kOk);
  EXPECT_DOUBLE_EQ(dot, 1.5);

  // ... and the locale's own spelling must still be rejected, not truncated to 1.
  double comma = 42.0;
  EXPECT_EQ(get_double(p, "comma", -kInf, kInf, false, comma), Status::kMalformed);
  EXPECT_DOUBLE_EQ(comma, 42.0);

  std::setlocale(LC_ALL, saved.c_str());
}

TEST(ParamParsing, fractional_integer_reports_malformed)
{
  // std::stol stops at the '.', leaving characters behind: malformed, never a silent truncation.
  const ParameterMap p{{"baudrate", "1000000.0"}, {"max_read_fails", "50.0"}, {"id", "4.5"}};

  for (const char * name : {"baudrate", "max_read_fails", "id"}) {
    int64_t value = 7;
    EXPECT_EQ(get_int(p, name, INT64_MIN, INT64_MAX, value), Status::kMalformed) << name;
    EXPECT_EQ(value, 7) << name;
  }
}

TEST(ParamParsing, integer_with_trailing_characters_reports_malformed)
{
  const ParameterMap p{
    {"io_timeout_ms", "20ms"}, {"encoder_steps", "4k"}, {"baudrate", "1M"},
    {"ping_attempts", "three"}};

  for (const char * name : {"io_timeout_ms", "encoder_steps", "baudrate", "ping_attempts"}) {
    int64_t value = 7;
    EXPECT_EQ(get_int(p, name, INT64_MIN, INT64_MAX, value), Status::kMalformed) << name;
    EXPECT_EQ(value, 7) << name;
  }
}

TEST(ParamParsing, integer_beyond_int64_reports_out_of_range)
{
  const ParameterMap p{
    {"over", "9223372036854775808"}, {"under", "-9223372036854775809"},
    {"max", "9223372036854775807"}, {"min", "-9223372036854775808"}};

  // std::out_of_range must be caught before std::invalid_argument, or this reads as malformed.
  for (const char * name : {"over", "under"}) {
    int64_t value = 7;
    EXPECT_EQ(get_int(p, name, INT64_MIN, INT64_MAX, value), Status::kOutOfRange) << name;
    EXPECT_EQ(value, 7) << name;
  }

  // The limits themselves are representable and pass.
  int64_t at_max = 0;
  EXPECT_EQ(get_int(p, "max", INT64_MIN, INT64_MAX, at_max), Status::kOk);
  EXPECT_EQ(at_max, INT64_MAX);

  int64_t at_min = 0;
  EXPECT_EQ(get_int(p, "min", INT64_MIN, INT64_MAX, at_min), Status::kOk);
  EXPECT_EQ(at_min, INT64_MIN);
}

TEST(ParamParsing, integer_below_min_reports_out_of_range)
{
  const ParameterMap p{
    {"io_timeout_ms", "0"}, {"ping_attempts", "-1"}, {"baudrate", "-1000000"},
    {"max_read_fails", "1000001"}};

  int64_t io_timeout_ms = 20;
  EXPECT_EQ(get_int(p, "io_timeout_ms", 2, 1000, io_timeout_ms), Status::kOutOfRange);
  EXPECT_EQ(io_timeout_ms, 20);

  // Pins get_int's inclusive bounds only; test_load_waveshare_servos pins the driver's 2 ms
  // io_timeout_ms floor.
  const ParameterMap one{{"io_timeout_ms", "1"}};
  int64_t one_ms = 20;
  EXPECT_EQ(get_int(one, "io_timeout_ms", 2, 1000, one_ms), Status::kOutOfRange);
  EXPECT_EQ(one_ms, 20);
  EXPECT_EQ(get_int(one, "io_timeout_ms", 1, 1000, one_ms), Status::kOk);
  EXPECT_EQ(one_ms, 1);

  int64_t ping_attempts = 3;
  EXPECT_EQ(get_int(p, "ping_attempts", 1, 10, ping_attempts), Status::kOutOfRange);
  EXPECT_EQ(ping_attempts, 3);

  // A negative baudrate is well formed here; the driver's mapped-baudrate check rejects it later.
  int64_t baudrate = 1000000;
  EXPECT_EQ(get_int(p, "baudrate", INT64_MIN, INT64_MAX, baudrate), Status::kOk);
  EXPECT_EQ(baudrate, -1000000);

  // The upper bound is checked by the same predicate.
  int64_t max_read_fails = 50;
  EXPECT_EQ(get_int(p, "max_read_fails", 1, 1000000, max_read_fails), Status::kOutOfRange);
  EXPECT_EQ(max_read_fails, 50);
}

TEST(ParamParsing, bool_accepts_true_and_false_in_any_case)
{
  const ParameterMap p{
    {"a", "true"}, {"b", "TRUE"}, {"c", "True"}, {"d", "tRuE"},
    {"e", "false"}, {"f", "FALSE"}, {"g", "False"}, {"h", " true "}};

  for (const char * name : {"a", "b", "c", "d", "h"}) {
    bool value = false;
    EXPECT_EQ(get_bool(p, name, value), Status::kOk) << name;
    EXPECT_TRUE(value) << name;
  }
  for (const char * name : {"e", "f", "g"}) {
    bool value = true;
    EXPECT_EQ(get_bool(p, name, value), Status::kOk) << name;
    EXPECT_FALSE(value) << name;
  }
}

TEST(ParamParsing, bool_rejects_one_and_zero)
{
  // hardware_interface::parse_bool accepts only true and false (any case): 1/0, yes/no and on/off
  // are malformed.
  const ParameterMap p{
    {"a", "1"}, {"b", "0"}, {"c", "yes"}, {"d", "no"}, {"e", "on"}, {"f", "off"}};

  for (const char * name : {"a", "b", "c", "d", "e", "f"}) {
    bool value = false;
    EXPECT_EQ(get_bool(p, name, value), Status::kMalformed) << name;
    EXPECT_FALSE(value) << name;
  }
}

TEST(ParamParsing, message_wording_for_each_status)
{
  const std::string subject = "hardware parameter 'io_timeout_ms'";
  const std::string expected = "an integer between 2 and 1000 (milliseconds)";

  // kDefaulted is an error only for a required parameter (the joint id); no hardware parameter
  // reaches it, so this case pins its wording.
  EXPECT_EQ(
    message(Status::kDefaulted, subject, "", expected),
    "hardware parameter 'io_timeout_ms' is missing; expected an integer between 2 and 1000 "
    "(milliseconds)");
  EXPECT_EQ(
    message(Status::kEmpty, subject, "", expected),
    "hardware parameter 'io_timeout_ms' is empty; expected an integer between 2 and 1000 "
    "(milliseconds)");
  EXPECT_EQ(
    message(Status::kMalformed, subject, "20ms", expected),
    "hardware parameter 'io_timeout_ms' is '20ms', which is not an integer between 2 and 1000 "
    "(milliseconds)");
  EXPECT_EQ(
    message(Status::kOutOfRange, subject, "0", expected),
    "hardware parameter 'io_timeout_ms' is '0', which is out of range; expected an integer "
    "between 2 and 1000 (milliseconds)");
  EXPECT_EQ(message(Status::kOk, subject, "20", expected), "");

  // A joint parameter reads the same way: the subject and the expectation are the only variables.
  EXPECT_EQ(
    message(
      Status::kOutOfRange, "joint 'joint1' parameter 'id'", "254",
      "an integer between 1 and 253"),
    "joint 'joint1' parameter 'id' is '254', which is out of range; expected an integer between "
    "1 and 253");
}

}  // namespace
