// Unit conversions between servo registers and the joint frame, and decode() of one feedback
// block. Pure arithmetic, no ROS and no bus. See docs/configuration.md, "Joint frame".

#ifndef UNITS_HPP_
#define UNITS_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

namespace waveshare_servos
{

// The nine state values this driver can publish, in the order JointStates declares them.
enum class StateKind : uint8_t
{
  kPosition,      // rad, joint frame
  kVelocity,      // rad/s, joint frame
  kEffort,        // N m, magnitude
  kCurrent,       // A, magnitude
  kVoltage,       // V
  kTemperature,   // deg C
  kLoad,          // fraction of full PWM, joint frame, about -1.023 .. +1.023
  kStatus,        // bitmask 0..255, integer-valued
  kTorque         // kgf cm, deprecated alias of effort
};
constexpr size_t kStateKindCount = 9;

constexpr double kNmPerKgfCm = 0.0980665;   // exact: g0 = 9.80665 m/s^2, 1 cm = 0.01 m
constexpr double kVoltsPerCount = 0.1;      // register 62, third-party table, unverified
constexpr double kLoadFullScale = 1000.0;   // registers 60-61, include/SMS_STS.h:83
// Scale of SMS_STS_ACC (register 41) from the Feetech memory table, not verified on this firmware.
// Only a max_accel in rad/s^2 uses it. See docs/configuration.md, "Joint parameters".
constexpr double kAccelStepsPerS2PerCount = 100.0;

// Angle and rate conversions. Keep the operator order: a precomputed rad-per-tick factor rounds
// differently and would change the published values in the last bits.
constexpr double steps_from_rad(double rad, int encoder_steps)
{
  return rad * encoder_steps / (2.0 * M_PI);
}

constexpr double rad_from_steps(double steps, int encoder_steps)
{
  return steps * 2.0 * M_PI / encoder_steps;
}

constexpr double accel_counts_from_rad_s2(double a, int encoder_steps)
{
  return steps_from_rad(a, encoder_steps) / kAccelStepsPerS2PerCount;
}

constexpr double accel_rad_s2_from_counts(double counts, int encoder_steps)
{
  return rad_from_steps(counts * kAccelStepsPerS2PerCount, encoder_steps);
}

// The scale is the current_per_count_a parameter (0.006 A per count by default). The decoded sign
// passes through.
inline double counts_to_amps(int counts, double current_per_count_a)
{
  return counts * current_per_count_a;
}

inline double amps_to_nm(double amps, double torque_constant_nm_per_a)
{
  return amps * torque_constant_nm_per_a;
}

// The deprecated `torque` interface keeps kgf cm, so on_init converts the N m constant once and
// FeedbackScales carries the result.
inline double kgfcm_per_amp(double torque_constant_nm_per_a)
{
  return torque_constant_nm_per_a / kNmPerKgfCm;
}

inline double raw_to_volts(int raw)
{
  return raw * kVoltsPerCount;
}

// Not clamped: ReadLoad masks bit 10 only (src/SMS_STS.cpp:188-190), so the magnitude reaches
// 1023 and this legitimately returns up to +-1.023.
inline double raw_to_load_fraction(int raw)
{
  return raw / kLoadFullScale;
}

inline double status_to_double(uint8_t status)
{
  return static_cast<double>(status);
}

// One servo's feedback block (registers 56..70), raw. Only apply_feedback() fills it, from the
// same decoder on both transports; each field keeps the bit layout of the accessor it names.
// See docs/design.md, "Feedback block".
struct FeedbackSample
{
  // Every member is defaulted: a field that a fill path does not set reads 0, not garbage.
  int position_ticks = 0;     // reg 56, as ReadPos(-1)      (decode() takes the unwrapped count)
  int speed_ticks = 0;        // reg 58, as ReadSpeed(-1),   sign-magnitude on bit 15
  int load_raw = 0;           // reg 60, as ReadLoad(-1),    sign-magnitude on bit 10
  int current_counts = 0;     // reg 69, as ReadCurrent(-1), sign on bit 15; unset by this firmware
  int voltage_raw = 0;        // reg 62, as ReadVoltage(-1)
  int temperature_raw = 0;    // reg 63, as ReadTemper(-1)
  // Byte 4 of the frame that carried the bytes above, never a later re-read of the shared
  // SCS::Error, so one joint's read cannot change another joint's status.
  uint8_t status = 0;
};

// The per-joint constants decode() needs. Defaults match the hardware-parameter defaults.
struct FeedbackScales
{
  int encoder_steps = 4096;
  double offset = 0.0;                        // rad, servo frame
  double sign = 1.0;                          // +1.0, or -1.0 for an inverted joint
  double current_per_count_a = 0.006;
  double torque_constant_nm_per_a = 0.8825985;    // 9.0 kgf cm/A * kNmPerKgfCm
  double torque_constant_kgfcm_per_a = 9.0;
};

// The nine decoded values, in StateKind order. position, velocity and load: joint frame (inverted
// flips them); current, effort and torque: unsigned magnitudes; the others: no frame.
struct JointStates
{
  double position = 0.0;
  double velocity = 0.0;
  double effort = 0.0;
  double current = 0.0;
  double voltage = 0.0;
  double temperature = 0.0;
  double load = 0.0;
  double status = 0.0;
  double torque = 0.0;
};

// Pure. `unwrapped_ticks` is the servo-frame tick count after unwrapping (== s.position_ticks
// when the joint does not unwrap).
JointStates decode(
  const FeedbackSample & s, int64_t unwrapped_ticks, const FeedbackScales & k);

// "overload", "voltage, overheat", "bit4 (meaning unverified), bit6 (meaning unverified)", or
// "none" for 0. Allocates; fault edges only.
std::string status_text(uint8_t status);

}  // namespace waveshare_servos

#endif  // UNITS_HPP_
