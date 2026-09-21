// Unit conversions between the servo's registers and the joint frame, and the decode that turns
// one feedback block into the nine published state values.
//
// Driver-owned, and deliberately free of ros2_control, rclcpp and the vendored SCServo headers:
// it is pure arithmetic over plain structs, so it is testable with no bus and no motors.
//
// Two frames (PHASE2_SPEC 3.1):
//   servo frame  theta = tick * 2*pi / encoder_steps, what the encoder reports;
//   joint frame  q,     what ROS sees. `offset` is a servo-frame constant ("which servo tick is
//                joint zero") and `sign` is -1 for a joint declared inverted.
//
// `sign` flips exactly three quantities -- position, velocity and load -- and nothing else
// (PHASE2_SPEC 3.2): they are the ones that are signed in the joint frame. current is signed by
// protocol but is a magnitude on this firmware, so it, and the effort and torque derived from it,
// are left alone. The consequence is worth stating plainly: on an inverted joint `effort`,
// `current` and `torque` are unsigned magnitudes and do NOT agree in sign with `velocity`, and
// `load` is the only direction-bearing member of the effort family.

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
  kTorque         // kg cm, deprecated alias of effort
};
constexpr size_t kStateKindCount = 9;

constexpr double kNmPerKgfCm = 0.0980665;   // exact: g0 = 9.80665 m/s^2, 1 cm = 0.01 m
constexpr double kVoltsPerCount = 0.1;      // register 62, third-party table, unverified
constexpr double kLoadFullScale = 1000.0;   // registers 60-61, include/SMS_STS.h:83
// SMS_STS_ACC, from the Feetech memory table: NOT verified against this firmware. Nothing that
// has a Phase 1 default is routed through it -- those defaults are register values (PHASE2_SPEC
// 5.4) -- so an error here can only affect a user who asks for max_accel in rad/s^2.
constexpr double kAccelStepsPerS2PerCount = 100.0;

// Angle and rate conversions. Deliberately written as `x * encoder_steps / (2 * M_PI)` and
// `x * 2.0 * M_PI / encoder_steps`, keeping the Phase 1 operator association: a precomputed
// radians-per-tick factor would round differently and lose bit-identity with Phase 1.
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

// ReadCurrent is unitless; the scale is a hardware parameter (0.006 A per count by default,
// cpp:737's `6.0 / 1000.0`). The sign is whatever the library decoded, and is passed through.
inline double counts_to_amps(int counts, double current_per_count_a)
{
  return counts * current_per_count_a;
}

inline double amps_to_nm(double amps, double torque_constant_nm_per_a)
{
  return amps * torque_constant_nm_per_a;
}

// The deprecated `torque` interface keeps kg cm, so the N m constant is converted once, in
// on_init, and the result is carried in FeedbackScales rather than recomputed per sample.
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

// One servo's feedback block (registers 56..70) on its way into decode(), in raw servo units.
// WaveshareServos::apply_feedback() is the ONLY thing that fills it, on both transports: the
// sync-read burst and the per-servo unicast read decode the same fifteen bytes with the same
// ServoBus decoder, so the two publish the same nine doubles by construction (PHASE3 2.36, F9).
// No `ReadX(-1)` accessor and no `FeedBack()` call is left on any driver path; the field comments
// name the accessor whose exact bit layout each value still keeps, because that equality is what
// PHASE3 F10 pins, not the call that produced it.
struct FeedbackSample
{
  // Every member is defaulted: apply_feedback() fills this struct field by field, and a field left
  // unset on some path has to read 0 rather than whatever was on the stack -- no warning catches
  // that one.
  int position_ticks = 0;     // reg 56, as ReadPos(-1)      (decode() takes the unwrapped count)
  int speed_ticks = 0;        // reg 58, as ReadSpeed(-1),   sign-magnitude on bit 15
  int load_raw = 0;           // reg 60, as ReadLoad(-1),    sign-magnitude on bit 10
  int current_counts = 0;     // reg 69, as ReadCurrent(-1), sign on bit 15; unset by this firmware
  int voltage_raw = 0;        // reg 62, as ReadVoltage(-1)
  int temperature_raw = 0;    // reg 63, as ReadTemper(-1)
  // Byte 4 of the frame that carried the fifteen bytes above -- the sync path copies it out of the
  // burst and the per-servo path takes SCS::Error straight after its Read(), which is the same
  // byte. Never a later re-read of the shared SCS::Error, so joint i's fault edge cannot depend on
  // what joint i-1's branch did (PHASE3 2.49, F18).
  uint8_t status = 0;
};

// The per-joint constants decode() needs. Defaults are the Phase 1 values.
struct FeedbackScales
{
  int encoder_steps = 4096;
  double offset = 0.0;                        // rad, servo frame
  double sign = 1.0;                          // +1.0, or -1.0 for an inverted joint
  double current_per_count_a = 0.006;
  double torque_constant_nm_per_a = 0.8825985;    // 9.0 kgf cm/A * kNmPerKgfCm
  double torque_constant_kgfcm_per_a = 9.0;
};

// The nine decoded values, in StateKind order. position, velocity and load are in the joint frame
// (inverted flips them); current, effort and torque are unsigned magnitudes (PHASE2_SPEC 3.2);
// voltage, temperature and status have no frame.
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
