// Unit tests for include/position_unwrapper.hpp: integer arithmetic, no ROS, bus or motors.
// See docs/design.md, "Position unwrapper".

#include <gmock/gmock.h>

#include <cmath>
#include <cstdint>
#include <ostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "position_unwrapper.hpp"

namespace
{

// Per-name declarations, never a using-directive: cpplint's build/namespaces rule forbids
// using-directives outside a short std::*_literals whitelist, in sources as well as headers.
using waveshare_servos::PositionUnwrapper;

// the encoder of every servo on this bench, and of every row of the table below
constexpr int32_t kSteps = 4096;

// Raw ticks in, expected unwrapped ticks out. Anchor rows also set `rad`: independent literal
// radians that the converted count must match. The other rows check ticks only.
struct UnwrapRow
{
  std::string name;
  std::vector<int32_t> raw;
  std::vector<int64_t> ticks;
  std::vector<double> rad = {};
};

// gtest prints an unknown parameter type as a hex dump; print the row instead, so a failure in one
// of the twelve rows reads as the sequence it is.
void PrintTo(const UnwrapRow & row, std::ostream * os)
{
  *os << row.name << " raw {";
  for (size_t at = 0; at < row.raw.size(); at++) {
    *os << (at == 0 ? "" : ", ") << row.raw[at];
  }
  *os << "}";
}

std::vector<UnwrapRow> unwrap_table()
{
  return {
    {"seed", {100}, {100}},
    {"forward_no_wrap", {100, 200, 300}, {100, 200, 300}},
    {"wrap_forward", {4090, 4095, 3, 10}, {4090, 4095, 4099, 4106}},
    {"wrap_backward", {5, 0, 4092, 4085}, {5, 0, -4, -11}},
    // A bench wrap: 4095 -> 12 is +13 ticks (about 1.99 rad/s at 100 Hz). The radians are the
    // joint3/position values that test_lifecycle_over_pty checks.
    {"bench_wrap", {4095, 12}, {4095, 4108}, {6.2816513263917, 6.301593076634214}},
    {"half_revolution_forward_reads_backward", {0, 2048}, {0, -2048}, {0.0, -M_PI}},
    {"just_under_half_revolution_forward", {0, 2047}, {0, 2047}, {0.0, 3.1400586728019073}},
    {"just_over_half_revolution_reads_backward", {0, 2049}, {0, -2047}},
    {"multi_turn_firmware_is_followed", {4000, 4100, 4200, 5000}, {4000, 4100, 4200, 5000}},
    {"multi_turn_firmware_crosses_zero", {2, 1, 0, -1, -2}, {2, 1, 0, -1, -2}},
    {"wrapping_register_crosses_zero", {2, 1, 0, 4095, 4094}, {2, 1, 0, -1, -2}},
    // Known limitation: a true +3000 ticks (a 0.5 s gap at 9.2 rad/s) aliases to -1096.
    // See docs/configuration.md, "Multi-turn wheel position".
    {"aliased_gap", {0, 3000}, {0, -1096}, {0.0, -1.6812429435226628}},
  };
}

// the row of the table above with this name, so a case that speaks about one row cannot quietly
// stop describing it; a name that is not there is a malformed table, not a silent pass
const UnwrapRow & unwrap_table_row(const std::string & name)
{
  static const std::vector<UnwrapRow> table = unwrap_table();
  for (const UnwrapRow & row : table) {
    if (row.name == name) {
      return row;
    }
  }
  throw std::out_of_range("no table row named " + name);
}

// the radian value of an unwrapped count, derived from the table rather than tabulated beside it
double rad_of_ticks(int64_t ticks)
{
  return static_cast<double>(ticks) * 2.0 * M_PI / kSteps;
}

}  // namespace

// The tests stay outside the anonymous namespace: cppcheck 2.13 reports a syntaxError for a
// TEST_F inside one.

class PositionUnwrapperTable : public ::testing::TestWithParam<UnwrapRow>
{
};

TEST_P(PositionUnwrapperTable, unwraps_the_tick_sequence)
{
  const UnwrapRow & row = GetParam();
  ASSERT_EQ(row.raw.size(), row.ticks.size()) << "the table row is malformed";
  ASSERT_TRUE(row.rad.empty() || row.rad.size() == row.raw.size()) << "the table row is malformed";
  PositionUnwrapper unwrapper(kSteps);
  for (size_t sample = 0; sample < row.raw.size(); sample++) {
    SCOPED_TRACE("sample " + std::to_string(sample) + " raw " + std::to_string(row.raw[sample]));
    const int64_t ticks = unwrapper.update(row.raw[sample]);
    EXPECT_EQ(ticks, row.ticks[sample]);
    EXPECT_EQ(unwrapper.ticks(), row.ticks[sample]);
    EXPECT_EQ(unwrapper.last_raw(), row.raw[sample]);
    // Anchor rows: compare with independent literal radians, not the same count converted twice.
    if (!row.rad.empty()) {
      EXPECT_THAT(rad_of_ticks(ticks), ::testing::DoubleNear(row.rad[sample], 1e-9));
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
  Table, PositionUnwrapperTable, ::testing::ValuesIn(unwrap_table()),
  [](const ::testing::TestParamInfo<UnwrapRow> & info) {return info.param.name;});

TEST(PositionUnwrapper, the_two_representations_describe_the_same_travel)
{
  // A multi-turn firmware and a wrapping register that report one motion give the same deltas;
  // only the seed differs (8000 and 3904 are the same pose, one revolution apart).
  const std::vector<std::pair<std::vector<int32_t>, std::vector<int32_t>>> pairs = {
    {{2, 1, 0, -1, -2}, {2, 1, 0, 4095, 4094}},
    {{8000, 8100, 7900, 8000}, {3904, 4004, 3804, 3904}},
  };
  for (const auto & pair : pairs) {
    SCOPED_TRACE("firmware seed " + std::to_string(pair.first.front()));
    ASSERT_EQ(pair.first.size(), pair.second.size());
    PositionUnwrapper firmware(kSteps);
    PositionUnwrapper wrapping(kSteps);
    std::vector<int64_t> a;
    std::vector<int64_t> b;
    for (size_t at = 0; at < pair.first.size(); at++) {
      a.push_back(firmware.update(pair.first[at]));
      b.push_back(wrapping.update(pair.second[at]));
    }
    for (size_t at = 1; at < a.size(); at++) {
      SCOPED_TRACE("sample " + std::to_string(at));
      EXPECT_EQ(a[at] - a[at - 1], b[at] - b[at - 1]) << "the same unwrapped travel";
      EXPECT_THAT(
        rad_of_ticks(a[at]) - rad_of_ticks(a[at - 1]),
        ::testing::DoubleNear(rad_of_ticks(b[at]) - rad_of_ticks(b[at - 1]), 1e-9));
    }
    // and the seed is the only difference, whatever it is: a constant offset for the whole run
    const int64_t offset = a.front() - b.front();
    for (size_t at = 0; at < a.size(); at++) {
      EXPECT_EQ(a[at] - b[at], offset) << "sample " << at;
    }
  }
  // the first pair is the table's two rows, so the deltas above are the tabulated ones
  EXPECT_EQ(unwrap_table_row("multi_turn_firmware_crosses_zero").ticks, (std::vector<int64_t>{
    2, 1, 0, -1, -2}));
  EXPECT_EQ(
    unwrap_table_row("wrapping_register_crosses_zero").ticks,
    unwrap_table_row("multi_turn_firmware_crosses_zero").ticks);
}

TEST(PositionUnwrapper, seeds_from_the_first_raw_value)
{
  PositionUnwrapper unwrapper(kSteps);
  EXPECT_FALSE(unwrapper.seeded());
  EXPECT_FALSE(unwrapper.bridged());

  // The first sample seeds the count at the raw value, also near the top of the register.
  EXPECT_EQ(unwrapper.update(4000), 4000);
  EXPECT_TRUE(unwrapper.seeded());
  EXPECT_FALSE(unwrapper.bridged()) << "the seeding sample continues no existing count";

  EXPECT_EQ(unwrapper.update(4010), 4010);
  EXPECT_TRUE(unwrapper.bridged());
}

TEST(PositionUnwrapper, reset_reseeds_without_a_phantom_revolution)
{
  PositionUnwrapper unwrapper(kSteps);
  unwrapper.update(4000);
  unwrapper.update(100);            // one wrap forward
  ASSERT_EQ(unwrapper.ticks(), 4196);

  unwrapper.reset();
  EXPECT_FALSE(unwrapper.seeded());
  EXPECT_FALSE(unwrapper.bridged());
  EXPECT_EQ(unwrapper.ticks(), 0);

  // a re-seed, not a continuation: the count starts again at the register reading
  EXPECT_EQ(unwrapper.update(100), 100);
  EXPECT_FALSE(unwrapper.bridged());
  EXPECT_EQ(unwrapper.update(110), 110);
}

TEST(PositionUnwrapper, a_dropped_and_rejoined_servo_starts_a_new_count)
{
  // read() resets a dropped servo's count, so after it answers again it reports the raw register
  // value. See docs/configuration.md, "Multi-turn wheel position".
  PositionUnwrapper unwrapper(kSteps);
  for (const int32_t raw : {0, 2000, 4000, 1900, 3900, 1800}) {
    unwrapper.update(raw);
  }
  ASSERT_EQ(unwrapper.ticks(), 9992) << "two wraps and change, 2.44 revolutions";

  unwrapper.reset();
  EXPECT_EQ(unwrapper.update(1800), 1800) << "not 9992, and not 9992 + a delta";
  EXPECT_EQ(unwrapper.update(1850), 1850);
}

TEST(PositionUnwrapper, wrapped_delta_maps_into_the_half_open_half_revolution)
{
  // [-steps/2, +steps/2): exactly half a revolution reads as backward, so the mapping is a
  // function and not a coin toss.
  const std::vector<int64_t> deltas = {
    -4096, -2049, -2048, -2047, -1, 0, 1, 2047, 2048, 2049, 4096, 8192};
  const std::vector<int64_t> reduced = {
    0, 2047, -2048, -2047, -1, 0, 1, 2047, -2048, -2047, 0, 0};
  ASSERT_EQ(deltas.size(), reduced.size());
  for (size_t at = 0; at < deltas.size(); at++) {
    SCOPED_TRACE("delta " + std::to_string(deltas[at]));
    const int64_t r = PositionUnwrapper::wrapped_delta(deltas[at], kSteps);
    EXPECT_EQ(r, reduced[at]);
    EXPECT_GE(r, -kSteps / 2);
    EXPECT_LT(r, kSteps / 2);
    // and it really is the same physical delta: the two differ by whole revolutions
    EXPECT_EQ((deltas[at] - r) % kSteps, 0);
  }
}

TEST(PositionUnwrapper, steps_below_two_freezes_the_count)
{
  // Total, because this runs on the real-time path: a steps of 0 or 1 must not divide by zero and
  // must not reduce a delta onto nonsense. It freezes the count instead.
  EXPECT_EQ(PositionUnwrapper::wrapped_delta(37, 0), 0);
  EXPECT_EQ(PositionUnwrapper::wrapped_delta(37, 1), 0);
  EXPECT_EQ(PositionUnwrapper::wrapped_delta(-37, 0), 0);

  // and the setter clamps, so an instance can never hold such a value in the first place
  PositionUnwrapper unwrapper(kSteps);
  unwrapper.set_steps_per_revolution(0);
  EXPECT_EQ(unwrapper.steps_per_revolution(), 2);
  unwrapper.set_steps_per_revolution(-100);
  EXPECT_EQ(unwrapper.steps_per_revolution(), 2);
  unwrapper.set_steps_per_revolution(1024);
  EXPECT_EQ(unwrapper.steps_per_revolution(), 1024);

  // The constructor clamps like the setter, so both entry points agree. The driver cannot reach
  // this: on_init requires an even encoder_steps in [2, 32768].
  EXPECT_EQ(PositionUnwrapper(0).steps_per_revolution(), 2);
  EXPECT_EQ(PositionUnwrapper(1).steps_per_revolution(), 2);
  EXPECT_EQ(PositionUnwrapper(-5).steps_per_revolution(), 2);
  EXPECT_EQ(PositionUnwrapper(4096).steps_per_revolution(), 4096);
}

TEST(PositionUnwrapper, an_odd_step_count_still_reduces)
{
  // The driver requires an even encoder_steps, but wrapped_delta must hold for any value: at 4095
  // steps, 2047 is forward and 2048 is backward.
  EXPECT_EQ(PositionUnwrapper::wrapped_delta(2047, 4095), 2047);
  EXPECT_EQ(PositionUnwrapper::wrapped_delta(2048, 4095), -2047);
  EXPECT_EQ(PositionUnwrapper::wrapped_delta(-2047, 4095), -2047);
  EXPECT_EQ(PositionUnwrapper::wrapped_delta(-2048, 4095), 2047);
}

TEST(PositionUnwrapper, three_revolutions_at_the_top_speed_do_not_drift)
{
  // 60 ticks per cycle is 6000 steps/s at 100 Hz. All 205 samples (2.99 revolutions, two wraps)
  // are asserted, so a lost or extra tick fails at the sample where it occurs.
  PositionUnwrapper unwrapper(kSteps);
  for (int i = 0; i <= 204; i++) {
    const int32_t raw = static_cast<int32_t>((i * 60) % kSteps);
    const int64_t ticks = unwrapper.update(raw);
    ASSERT_EQ(ticks, static_cast<int64_t>(i) * 60) << "cycle " << i;
  }
  EXPECT_EQ(unwrapper.ticks(), 12240);
}

TEST(PositionUnwrapper, a_gap_longer_than_half_a_revolution_aliases)
{
  // Pins the known aliasing so nobody re-seeds after a gap: that makes the error the full travel,
  // not whole revolutions. See docs/design.md, "Position unwrapper".
  PositionUnwrapper unwrapper(kSteps);
  unwrapper.update(0);
  EXPECT_EQ(unwrapper.update(3000), -1096);
  EXPECT_EQ(3000 - (-1096), kSteps) << "the error is exactly one revolution, never a partial turn";

  // and one tick less is still correct: the envelope is half a revolution, not a fuzzy region
  PositionUnwrapper inside(kSteps);
  inside.update(0);
  EXPECT_EQ(inside.update(2047), 2047);
}

TEST(PositionUnwrapper, works_with_a_1024_step_encoder)
{
  // encoder_steps is a hardware parameter, so the rule may not assume 4096 anywhere.
  PositionUnwrapper unwrapper(1024);
  EXPECT_EQ(unwrapper.steps_per_revolution(), 1024);
  EXPECT_EQ(unwrapper.update(1020), 1020);
  EXPECT_EQ(unwrapper.update(2), 1026);
  EXPECT_EQ(unwrapper.update(5), 1029);
}

TEST(PositionUnwrapper, tolerates_the_full_sign_magnitude_range)
{
  // ReadPos is sign-magnitude on bit 15, so raw can be anywhere in [-32767, 32767]. Nothing may
  // overflow, and each step is still the reduced delta.
  PositionUnwrapper unwrapper(kSteps);
  EXPECT_EQ(unwrapper.update(0), 0);
  EXPECT_EQ(unwrapper.update(32767), -1);
  EXPECT_EQ(unwrapper.update(-32767), 1);
  EXPECT_EQ(unwrapper.update(32767), -1);
  EXPECT_EQ(unwrapper.last_raw(), 32767);

  // the extremes are also legal seeds
  PositionUnwrapper low(kSteps);
  EXPECT_EQ(low.update(-32767), -32767);
  PositionUnwrapper high(kSteps);
  EXPECT_EQ(high.update(32767), 32767);
}
