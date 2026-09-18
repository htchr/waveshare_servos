#include "units.hpp"

#include <cmath>
#include <string>

namespace waveshare_servos
{

namespace
{
// Names from FEETECH's own SDK: scservo_sdk/protocol_packet_handler.py in
// https://github.com/ftservo/FTServo_Python (retrieved 2026-09-16, by URL from
// https://raw.githubusercontent.com/ftservo/FTServo_Python/main/scservo_sdk/protocol_packet_handler.py)
// -- ERRBIT_VOLTAGE = 1, ERRBIT_ANGLE = 2, ERRBIT_OVERHEAT = 4, ERRBIT_OVERELE = 8,
// ERRBIT_OVERLOAD = 32 -- and its getRxPacketError(), which decodes the same byte this driver
// captures: that SDK reads rxpacket[PKT_ERROR] with PKT_ERROR == 4, and SCS::Read() assigns
// Error = bBuf[4] (src/SCS.cpp:201). Bits 4, 6 and 7 are named by no source we could retrieve.
// No source says which of these bits this firmware actually raises, which is why the raw byte is
// published on the `status` interface and only these names are ever printed.
//
// This is the ONLY place a bit name appears in the driver. If a later phase verifies a bit on
// this hardware, one entry here, one README row and this citation change together.
constexpr const char * kStatusBitNames[8] = {
  "voltage", "angle", "overheat", "overcurrent", "bit4 (meaning unverified)", "overload",
  "bit6 (meaning unverified)", "bit7 (meaning unverified)"};
}  // namespace

JointStates decode(
  const FeedbackSample & s, int64_t unwrapped_ticks, const FeedbackScales & k)
{
  JointStates out{};
  // The `sign` factor (from <param name="inverted">) flips exactly the quantities that are signed
  // in the joint frame: position, velocity and load. See PHASE2_SPEC 3.2.
  //  - ReadLoad's sign is bit 10 (src/SMS_STS.cpp:188-189) and carries a real direction: it flips.
  //  - ReadCurrent decodes a sign at bit 15 (src/SMS_STS.cpp:254-255), but this firmware never
  //    sets it (Phase 1 S6 recording: 0 negative samples in 3343 messages with the wheels driven
  //    both ways), so current is a magnitude in practice. Negating a magnitude would make every
  //    mirrored joint read uniformly negative, which is worse than unsigned. current, and the
  //    effort and torque derived from it, are therefore NOT flipped, and are documented as
  //    magnitudes that do not agree in sign with velocity on an inverted joint.
  // The three sites are one contiguous block on purpose: inverted is a property of three fields,
  // not of a "family", and the whole rule has to be readable without scanning the function.
  out.position = k.sign * (unwrapped_ticks * 2.0 * M_PI / k.encoder_steps - k.offset);
  out.velocity = k.sign * (s.speed_ticks * 2.0 * M_PI / k.encoder_steps);
  out.load = k.sign * raw_to_load_fraction(s.load_raw);

  out.current = counts_to_amps(s.current_counts, k.current_per_count_a);
  out.effort = amps_to_nm(out.current, k.torque_constant_nm_per_a);
  out.torque = out.current * k.torque_constant_kgfcm_per_a;   // deprecated alias, kg cm

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
