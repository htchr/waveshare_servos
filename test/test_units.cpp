// Unit tests for include/units.hpp + src/units.cpp.
//
// The unit under test is pure: no ros2_control, no rclcpp, no bus, no motors. Everything here is
// arithmetic over one FeedbackSample, so the whole suite runs with the adapter unplugged.
//
// Two invariants are worth naming, because they are the reason several of these cases exist:
// - decode() keeps the Phase 1 operator association, so a non-inverted joint is bit-identical to
//   the pre-Phase-2 driver (position_conversion_matches_the_legacy_expression asserts exact
//   equality, never a tolerance);
// - `inverted` flips exactly three quantities -- position, velocity and load -- and never the
//   current family, which is an unsigned magnitude on this firmware.

#include <gmock/gmock.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <new>
#include <string>
#include <vector>

#include "units.hpp"

namespace
{

// Per-name declarations, never a using-directive: cpplint's build/namespaces rule forbids
// using-directives outside a short std::*_literals whitelist, in sources as well as headers.
using waveshare_servos::FeedbackSample;
using waveshare_servos::FeedbackScales;
using waveshare_servos::JointStates;
using waveshare_servos::StateKind;
using waveshare_servos::accel_counts_from_rad_s2;
using waveshare_servos::accel_rad_s2_from_counts;
using waveshare_servos::amps_to_nm;
using waveshare_servos::counts_to_amps;
using waveshare_servos::decode;
using waveshare_servos::kLoadFullScale;
using waveshare_servos::kNmPerKgfCm;
using waveshare_servos::kStateKindCount;
using waveshare_servos::kVoltsPerCount;
using waveshare_servos::kgfcm_per_amp;
using waveshare_servos::rad_from_steps;
using waveshare_servos::raw_to_load_fraction;
using waveshare_servos::raw_to_volts;
using waveshare_servos::status_text;
using waveshare_servos::status_to_double;
using waveshare_servos::steps_from_rad;

// The sample every case decodes: its nine sources are distinct and non-zero, so a transposition
// inside decode() cannot hide behind two fields that happen to hold the same number.
constexpr int kPositionTicks = 1024;
constexpr int kSpeedTicks = 100;
constexpr int kLoadRaw = 250;
constexpr int kCurrentCounts = 50;
constexpr int kVoltageRaw = 120;
constexpr int kTemperatureRaw = 41;
constexpr uint8_t kStatusByte = 0x24;

FeedbackSample reference_sample()
{
  FeedbackSample sample{};
  sample.position_ticks = kPositionTicks;
  sample.speed_ticks = kSpeedTicks;
  sample.load_raw = kLoadRaw;
  sample.current_counts = kCurrentCounts;
  sample.voltage_raw = kVoltageRaw;
  sample.temperature_raw = kTemperatureRaw;
  sample.status = kStatusByte;
  return sample;
}

// The member each StateKind names. A switch, not an array, so a new enumerator is a compile error
// here rather than a silently unread field.
double state_value(const JointStates & v, StateKind kind)
{
  switch (kind) {
    case StateKind::kPosition: return v.position;
    case StateKind::kVelocity: return v.velocity;
    case StateKind::kEffort: return v.effort;
    case StateKind::kCurrent: return v.current;
    case StateKind::kVoltage: return v.voltage;
    case StateKind::kTemperature: return v.temperature;
    case StateKind::kLoad: return v.load;
    case StateKind::kStatus: return v.status;
    case StateKind::kTorque: return v.torque;
  }
  return std::nan("");
}

int set_bits(unsigned int value)
{
  int count = 0;
  for (int bit = 0; bit < 8; bit++) {
    if ((value & (1u << bit)) != 0u) {
      count++;
    }
  }
  return count;
}

int occurrences(const std::string & text, const std::string & needle)
{
  int count = 0;
  for (size_t at = text.find(needle); at != std::string::npos; at = text.find(needle, at + 1)) {
    count++;
  }
  return count;
}

}  // namespace

// The tests stay outside the anonymous namespace: cppcheck 2.13 reports a syntaxError for a
// TEST_F inside one.

// One fixture for the whole suite, so the three `inverted` cases share one sample and exactly two
// decodes: the same feedback read as a normal joint and as a mirrored one.
class Units : public ::testing::Test
{
public:
  Units()
  : sample_(reference_sample()),
    ref_(decode(sample_, sample_.position_ticks, scales_with_sign(1.0))),
    inv_(decode(sample_, sample_.position_ticks, scales_with_sign(-1.0)))
  {
  }

  // A non-zero offset on purpose: the flip must be an exact negation whatever the offset is.
  static FeedbackScales scales_with_sign(double sign)
  {
    FeedbackScales scales;
    scales.sign = sign;
    scales.offset = 0.5;
    return scales;
  }

  FeedbackSample sample_;
  JointStates ref_;
  JointStates inv_;
};

TEST_F(Units, position_ticks_convert_to_radians)
{
  const FeedbackScales k;
  EXPECT_DOUBLE_EQ(decode(reference_sample(), 1024, k).position, M_PI / 2.0);
  EXPECT_DOUBLE_EQ(decode(reference_sample(), 2048, k).position, M_PI);
  EXPECT_DOUBLE_EQ(decode(reference_sample(), -1024, k).position, -M_PI / 2.0);
  EXPECT_DOUBLE_EQ(decode(reference_sample(), 0, k).position, 0.0);
}

TEST_F(Units, position_conversion_matches_the_legacy_expression)
{
  const FeedbackScales k;
  ASSERT_EQ(k.encoder_steps, 4096);
  ASSERT_DOUBLE_EQ(k.sign, 1.0);
  ASSERT_DOUBLE_EQ(k.offset, 0.0);
  for (int t = -4096; t <= 4096; t++) {
    FeedbackSample sample = reference_sample();
    sample.position_ticks = t;
    // cpp:734 verbatim, with the Phase 1 operator association: never a precomputed rad per tick.
    EXPECT_EQ(decode(sample, t, k).position, t * 2 * M_PI / 4096) << "tick " << t;
  }

  // The loop above cannot fail for the association on its own: 4096 is a power of two, so
  // 2*M_PI/4096 is exact and the hoisted form `t * (2*M_PI/4096)` is bit-identical for every tick.
  // encoder_steps is any even value in [2, 32768] (PHASE2_SPEC 4.1), and at 1000 the hoisted form
  // differs by 1 ulp for 704 of these 2001 ticks -- so this is the loop that actually locks the
  // association down.
  FeedbackScales odd_steps;
  odd_steps.encoder_steps = 1000;
  ASSERT_DOUBLE_EQ(odd_steps.sign, 1.0);
  ASSERT_DOUBLE_EQ(odd_steps.offset, 0.0);
  for (int t = -1000; t <= 1000; t++) {
    FeedbackSample sample = reference_sample();
    sample.position_ticks = t;
    EXPECT_EQ(decode(sample, t, odd_steps).position, t * 2 * M_PI / 1000) << "tick " << t;
  }
}

TEST_F(Units, velocity_conversion_matches_the_legacy_expression)
{
  // The other half of the association rule (PHASE2_SPEC 3.1), on the velocity line. Again at an
  // encoder_steps that is not a power of two, where `t * (2*M_PI/steps)` is observably different.
  FeedbackScales k;
  k.encoder_steps = 1000;
  for (int t = -3000; t <= 3000; t++) {
    FeedbackSample sample = reference_sample();
    sample.speed_ticks = t;
    EXPECT_EQ(decode(sample, 0, k).velocity, t * 2 * M_PI / 1000) << "speed ticks " << t;
  }
}

TEST_F(Units, speed_ticks_convert_to_radians_per_second)
{
  const FeedbackScales k;
  FeedbackSample sample = reference_sample();
  sample.speed_ticks = 6000;
  EXPECT_EQ(decode(sample, 0, k).velocity, 6000 * 2 * M_PI / 4096);
  EXPECT_NEAR(decode(sample, 0, k).velocity, 9.2038847, 1e-6);
  sample.speed_ticks = -6000;
  EXPECT_NEAR(decode(sample, 0, k).velocity, -9.2038847, 1e-6);
  sample.speed_ticks = 0;
  EXPECT_DOUBLE_EQ(decode(sample, 0, k).velocity, 0.0);
}

TEST_F(Units, current_counts_convert_to_amps_with_the_configured_scale)
{
  EXPECT_DOUBLE_EQ(counts_to_amps(100, 0.006), 0.6);
  EXPECT_DOUBLE_EQ(counts_to_amps(100, 0.0065), 0.65);
  EXPECT_DOUBLE_EQ(counts_to_amps(0, 0.006), 0.0);

  FeedbackScales k;
  k.current_per_count_a = 0.0065;
  FeedbackSample sample = reference_sample();
  sample.current_counts = 100;
  EXPECT_DOUBLE_EQ(decode(sample, 0, k).current, 0.65);
}

TEST_F(Units, current_keeps_the_sign_it_is_given)
{
  // Whatever sign the library decoded is published unchanged. This is about the reading, not
  // about `inverted`: see inverted_leaves_the_current_family_alone for the other half.
  EXPECT_DOUBLE_EQ(counts_to_amps(-25, 0.006), -0.15);

  const FeedbackScales k;
  FeedbackSample sample = reference_sample();
  sample.current_counts = -50;
  const JointStates v = decode(sample, 0, k);
  EXPECT_LT(v.current, 0.0);
  EXPECT_DOUBLE_EQ(v.current, counts_to_amps(-50, k.current_per_count_a));
  EXPECT_LT(v.effort, 0.0);
  EXPECT_LT(v.torque, 0.0);
}

TEST_F(Units, effort_is_current_times_the_torque_constant)
{
  EXPECT_DOUBLE_EQ(amps_to_nm(2.0, 0.8825985), 2.0 * 0.8825985);
  EXPECT_DOUBLE_EQ(amps_to_nm(0.0, 0.8825985), 0.0);

  const FeedbackScales k;
  const JointStates v = decode(reference_sample(), 0, k);
  EXPECT_DOUBLE_EQ(v.effort, v.current * k.torque_constant_nm_per_a);
  EXPECT_DOUBLE_EQ(v.effort, amps_to_nm(v.current, k.torque_constant_nm_per_a));
}

TEST_F(Units, default_torque_constant_is_nine_kgf_cm_per_amp)
{
  EXPECT_DOUBLE_EQ(kNmPerKgfCm, 0.0980665);
  EXPECT_DOUBLE_EQ(kgfcm_per_amp(0.8825985), 9.0);

  const FeedbackScales k;
  EXPECT_DOUBLE_EQ(k.torque_constant_nm_per_a, 0.8825985);
  EXPECT_DOUBLE_EQ(k.torque_constant_kgfcm_per_a, 9.0);
  EXPECT_DOUBLE_EQ(kgfcm_per_amp(k.torque_constant_nm_per_a), k.torque_constant_kgfcm_per_a);
}

TEST_F(Units, torque_alias_reproduces_the_legacy_value)
{
  // cpp:737 was `ReadCurrent(-1) * 6.0 / 1000.0 * KT_` with KT_ == 9.0, in kg cm. The Phase 2
  // regrouping is not bit-identical (2 ulp), so the comparison is relative, never `==`.
  const FeedbackScales k;
  for (const int counts : {0, 1, 7, 50, 123, 1000, -50}) {
    FeedbackSample sample = reference_sample();
    sample.current_counts = counts;
    const double legacy = counts * 6.0 / 1000.0 * 9.0;
    const double got = decode(sample, 0, k).torque;
    EXPECT_NEAR(got, legacy, std::fabs(legacy) * 1e-12) << "counts " << counts;
  }
}

TEST_F(Units, voltage_raw_is_tenths_of_a_volt)
{
  EXPECT_DOUBLE_EQ(kVoltsPerCount, 0.1);
  EXPECT_DOUBLE_EQ(raw_to_volts(120), 12.0);
  EXPECT_DOUBLE_EQ(raw_to_volts(0), 0.0);
  EXPECT_DOUBLE_EQ(raw_to_volts(255), 25.5);

  const FeedbackScales k;
  EXPECT_DOUBLE_EQ(decode(reference_sample(), 0, k).voltage, 12.0);
}

TEST_F(Units, load_raw_is_thousandths_of_full_pwm)
{
  EXPECT_DOUBLE_EQ(kLoadFullScale, 1000.0);
  EXPECT_DOUBLE_EQ(raw_to_load_fraction(250), 0.25);
  EXPECT_DOUBLE_EQ(raw_to_load_fraction(-250), -0.25);
  EXPECT_DOUBLE_EQ(raw_to_load_fraction(0), 0.0);

  const FeedbackScales k;
  EXPECT_DOUBLE_EQ(decode(reference_sample(), 0, k).load, 0.25);
}

TEST_F(Units, load_raw_above_full_scale_is_not_clamped)
{
  // ReadLoad masks bit 10 only (src/SMS_STS.cpp:188-190), so the magnitude reaches 1023.
  EXPECT_DOUBLE_EQ(raw_to_load_fraction(1023), 1.023);
  EXPECT_DOUBLE_EQ(raw_to_load_fraction(-1023), -1.023);

  const FeedbackScales k;
  FeedbackSample sample = reference_sample();
  sample.load_raw = 1023;
  EXPECT_DOUBLE_EQ(decode(sample, 0, k).load, 1.023);
  EXPECT_GT(decode(sample, 0, k).load, 1.0);
}

TEST_F(Units, status_encodes_as_a_plain_bitmask)
{
  for (int v = 0; v <= 255; v++) {
    EXPECT_DOUBLE_EQ(status_to_double(static_cast<uint8_t>(v)), static_cast<double>(v));
  }
  const FeedbackScales k;
  FeedbackSample sample = reference_sample();
  sample.status = 0xFF;
  EXPECT_DOUBLE_EQ(decode(sample, 0, k).status, 255.0);
  sample.status = 0x00;
  EXPECT_DOUBLE_EQ(decode(sample, 0, k).status, 0.0);
  sample.status = 0x24;
  EXPECT_DOUBLE_EQ(decode(sample, 0, k).status, 36.0);
}

TEST_F(Units, status_text_of_zero_is_none)
{
  EXPECT_EQ(status_text(0x00), "none");
}

TEST_F(Units, status_text_names_every_cited_bit)
{
  // All five names come from FEETECH's own Python SDK (ERRBIT_VOLTAGE = 1, ERRBIT_ANGLE = 2,
  // ERRBIT_OVERHEAT = 4, ERRBIT_OVERELE = 8, ERRBIT_OVERLOAD = 32), retrieved by URL; see the
  // citation comment in src/units.cpp.
  EXPECT_EQ(status_text(0x01), "voltage");
  EXPECT_EQ(status_text(0x02), "angle");
  EXPECT_EQ(status_text(0x04), "overheat");
  EXPECT_EQ(status_text(0x08), "overcurrent");
  EXPECT_EQ(status_text(0x20), "overload");
}

TEST_F(Units, status_text_lists_several_bits_lowest_first)
{
  EXPECT_EQ(status_text(0x21), "voltage, overload");
  EXPECT_EQ(status_text(0x24), "overheat, overload");
  EXPECT_EQ(status_text(0x03), "voltage, angle");
}

TEST_F(Units, status_text_marks_uncited_bits_as_unverified)
{
  // The executable form of the honesty rule: no bit gets a name a primary source does not give
  // it. This case fails the day someone tidies an unverified bit into a guess.
  EXPECT_EQ(status_text(0x10), "bit4 (meaning unverified)");
  EXPECT_EQ(status_text(0x40), "bit6 (meaning unverified)");
  EXPECT_EQ(status_text(0x80), "bit7 (meaning unverified)");
  EXPECT_EQ(status_text(0x50), "bit4 (meaning unverified), bit6 (meaning unverified)");
}

TEST_F(Units, status_text_never_omits_a_set_bit)
{
  EXPECT_EQ(status_text(0x00), "none");
  for (int v = 1; v <= 255; v++) {
    const std::string text = status_text(static_cast<uint8_t>(v));
    EXPECT_FALSE(text.empty()) << "status " << v;
    EXPECT_NE(text, "none") << "status " << v;
    EXPECT_EQ(occurrences(text, ", "), set_bits(static_cast<unsigned int>(v)) - 1)
      << "status " << v << " rendered as '" << text << "'";
  }
}

TEST_F(Units, each_state_kind_reads_its_own_field)
{
  // The only guard against a transposition inside decode(): nine distinct sources, nine distinct
  // results, each checked against the value its own source implies.
  const FeedbackScales k;
  const JointStates v = decode(reference_sample(), kPositionTicks, k);

  const double expected[kStateKindCount] = {
    kPositionTicks * 2 * M_PI / 4096,                      // position, rad
    kSpeedTicks * 2 * M_PI / 4096,                         // velocity, rad/s
    kCurrentCounts * 0.006 * 0.8825985,                    // effort, N m
    kCurrentCounts * 0.006,                                // current, A
    kVoltageRaw * 0.1,                                     // voltage, V
    static_cast<double>(kTemperatureRaw),                  // temperature, deg C
    kLoadRaw / 1000.0,                                     // load, fraction of full PWM
    static_cast<double>(kStatusByte),                      // status, bitmask
    kCurrentCounts * 0.006 * 9.0};                         // torque, kg cm (deprecated)

  for (size_t i = 0; i < kStateKindCount; i++) {
    const double got = state_value(v, static_cast<StateKind>(i));
    EXPECT_DOUBLE_EQ(got, expected[i]) << "StateKind " << i;
    for (size_t j = i + 1; j < kStateKindCount; j++) {
      EXPECT_NE(got, state_value(v, static_cast<StateKind>(j)))
        << "StateKind " << i << " and " << j << " are indistinguishable in this sample";
    }
  }
}

TEST_F(Units, inverted_flips_position_velocity_and_load_only)
{
  EXPECT_DOUBLE_EQ(inv_.position, -ref_.position);   // exact negation, any offset
  EXPECT_DOUBLE_EQ(inv_.velocity, -ref_.velocity);
  EXPECT_DOUBLE_EQ(inv_.load, -ref_.load);
  EXPECT_GT(std::fabs(ref_.position), 0.0);          // the sample is not accidentally symmetric
  EXPECT_GT(std::fabs(ref_.velocity), 0.0);
  EXPECT_GT(std::fabs(ref_.load), 0.0);
}

TEST_F(Units, inverted_leaves_the_current_family_alone)
{
  EXPECT_DOUBLE_EQ(inv_.current, ref_.current);      // NOT negated
  EXPECT_DOUBLE_EQ(inv_.effort, ref_.effort);
  EXPECT_DOUBLE_EQ(inv_.torque, ref_.torque);
  EXPECT_DOUBLE_EQ(inv_.voltage, ref_.voltage);
  EXPECT_DOUBLE_EQ(inv_.temperature, ref_.temperature);
  EXPECT_DOUBLE_EQ(inv_.status, ref_.status);
  EXPECT_GT(ref_.current, 0.0);                      // and the test would notice a flip
}

TEST_F(Units, effort_equals_current_times_kt_for_both_signs)
{
  const FeedbackScales k = scales_with_sign(1.0);
  for (const JointStates & v : {ref_, inv_}) {
    EXPECT_DOUBLE_EQ(v.effort, v.current * k.torque_constant_nm_per_a);
    EXPECT_DOUBLE_EQ(v.torque, v.current * k.torque_constant_kgfcm_per_a);
  }
}

TEST_F(Units, offset_applies_to_position_only)
{
  FeedbackScales without;
  FeedbackScales with;
  with.offset = 0.5;
  const JointStates a = decode(sample_, sample_.position_ticks, without);
  const JointStates b = decode(sample_, sample_.position_ticks, with);

  EXPECT_DOUBLE_EQ(b.position, a.position - 0.5);
  EXPECT_DOUBLE_EQ(b.velocity, a.velocity);
  EXPECT_DOUBLE_EQ(b.effort, a.effort);
  EXPECT_DOUBLE_EQ(b.current, a.current);
  EXPECT_DOUBLE_EQ(b.voltage, a.voltage);
  EXPECT_DOUBLE_EQ(b.temperature, a.temperature);
  EXPECT_DOUBLE_EQ(b.load, a.load);
  EXPECT_DOUBLE_EQ(b.status, a.status);
  EXPECT_DOUBLE_EQ(b.torque, a.torque);
}

TEST_F(Units, speed_counts_match_the_phase1_default)
{
  // The Phase 1 goal-speed default is the register value 6000, not a rad/s number.
  EXPECT_NEAR(steps_from_rad(9.2038847, 4096), 6000.0, 0.5);
  EXPECT_NEAR(rad_from_steps(6000, 4096), 9.2038847, 1e-6);
  EXPECT_NEAR(rad_from_steps(32767, 4096), 50.264, 1e-3);
}

TEST_F(Units, accel_counts_match_the_phase1_default)
{
  // Likewise the ACC default: 150 counts at the (unverified) 100 steps/s^2 per count scale.
  EXPECT_NEAR(accel_counts_from_rad_s2(23.0097118, 4096), 150.0, 0.5);
  EXPECT_NEAR(accel_rad_s2_from_counts(150, 4096), 23.0097118, 1e-6);
  EXPECT_NEAR(accel_rad_s2_from_counts(255, 4096), 39.116, 1e-3);
}

TEST_F(Units, rad_and_step_conversions_round_trip)
{
  for (const int encoder_steps : {1000, 4096, 32768}) {
    for (const double rad : {-7.5, -1.0, 0.0, 0.25, 3.14159, 100.0}) {
      EXPECT_NEAR(rad_from_steps(steps_from_rad(rad, encoder_steps), encoder_steps), rad, 1e-12)
        << "encoder_steps " << encoder_steps << ", rad " << rad;
      EXPECT_NEAR(
        accel_rad_s2_from_counts(accel_counts_from_rad_s2(rad, encoder_steps), encoder_steps),
        rad, 1e-12) << "encoder_steps " << encoder_steps << ", rad/s^2 " << rad;
    }
  }
}

TEST_F(Units, plain_structs_default_every_member_to_zero)
{
  // Chunk 6 fills a FeedbackSample field by field from the ReadX(-1) accessors. A field left unset
  // on some path must read 0, not whatever was on the stack -- a stale `load_raw` would go straight
  // onto a state interface and no compiler warning catches it. Default-initialising over a poisoned
  // buffer makes the difference deterministic: with a default member initialiser on every member
  // these read 0, without one they read the poison.
  alignas(FeedbackSample) unsigned char sample_storage[sizeof(FeedbackSample)];
  std::memset(sample_storage, 0xAB, sizeof(sample_storage));
  FeedbackSample * sample = new (sample_storage) FeedbackSample;
  EXPECT_EQ(sample->position_ticks, 0);
  EXPECT_EQ(sample->speed_ticks, 0);
  EXPECT_EQ(sample->load_raw, 0);
  EXPECT_EQ(sample->current_counts, 0);
  EXPECT_EQ(sample->voltage_raw, 0);
  EXPECT_EQ(sample->temperature_raw, 0);
  EXPECT_EQ(static_cast<int>(sample->status), 0);
  sample->~FeedbackSample();

  alignas(JointStates) unsigned char states_storage[sizeof(JointStates)];
  std::memset(states_storage, 0xAB, sizeof(states_storage));
  JointStates * states = new (states_storage) JointStates;
  for (size_t i = 0; i < kStateKindCount; i++) {
    EXPECT_DOUBLE_EQ(state_value(*states, static_cast<StateKind>(i)), 0.0) << "StateKind " << i;
  }
  states->~JointStates();
}

TEST_F(Units, one_tick_is_two_pi_over_encoder_steps)
{
  static_assert(rad_from_steps(4096, 4096) == 2.0 * M_PI, "the helpers must be usable constexpr");
  EXPECT_DOUBLE_EQ(rad_from_steps(1, 4096), 2 * M_PI / 4096);
  for (const int encoder_steps : {1000, 4096, 32768}) {
    EXPECT_DOUBLE_EQ(
      rad_from_steps(1, encoder_steps) * encoder_steps, 2.0 * M_PI) << encoder_steps;
    EXPECT_DOUBLE_EQ(steps_from_rad(2.0 * M_PI, encoder_steps), encoder_steps) << encoder_steps;
  }
}
