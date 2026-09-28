#include "units.hpp"

#include <cmath>
#include <string>

namespace waveshare_servos
{

namespace
{
// Bit names from the ERRBIT_* constants of FEETECH's FTServo_Python SDK, not verified on this
// firmware, so only log text uses them. See docs/configuration.md, "Status bits".
constexpr const char * kStatusBitNames[8] = {
  "voltage", "angle", "overheat", "overcurrent", "bit4 (meaning unverified)", "overload",
  "bit6 (meaning unverified)", "bit7 (meaning unverified)"};
}  // namespace

JointStates decode(
  const FeedbackSample & s, int64_t unwrapped_ticks, const FeedbackScales & k)
{
  JointStates out{};
  // `sign` (inverted) flips only position, velocity and load; current, effort and torque are
  // magnitudes on this firmware. See docs/configuration.md, "Joint frame".
  out.position = k.sign * (unwrapped_ticks * 2.0 * M_PI / k.encoder_steps - k.offset);
  out.velocity = k.sign * (s.speed_ticks * 2.0 * M_PI / k.encoder_steps);
  out.load = k.sign * raw_to_load_fraction(s.load_raw);

  out.current = counts_to_amps(s.current_counts, k.current_per_count_a);
  out.effort = amps_to_nm(out.current, k.torque_constant_nm_per_a);
  out.torque = out.current * k.torque_constant_kgfcm_per_a;   // deprecated alias, kgf cm

  out.voltage = raw_to_volts(s.voltage_raw);
  out.temperature = static_cast<double>(s.temperature_raw);
  out.status = status_to_double(s.status);
  return out;
}

std::string status_text(uint8_t status)
{
  if (status == 0) {
    return "none";
  }
  std::string text;
  for (int bit = 0; bit < 8; bit++) {
    if ((status & (1u << bit)) != 0u) {
      if (!text.empty()) {
        text += ", ";
      }
      text += kStatusBitNames[bit];
    }
  }
  return text;
}

}  // namespace waveshare_servos
