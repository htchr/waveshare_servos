// driver_defaults -- the hardware parameters' defaults, shared by the driver and the Phase 6 tools.
//
// ROS-free and header-only, so a target that links only the bus (the tools' library, the CLI test)
// can include it without hardware_interface, which src/param_parsing.hpp pulls in. Under src/ and
// not installed, for the reason param_parsing.hpp gives: an implementation detail, not interface.

#ifndef DRIVER_DEFAULTS_HPP_
#define DRIVER_DEFAULTS_HPP_

namespace waveshare_servos
{

// The compiled-in defaults of the eleven hardware parameters (PHASE2_SPEC 4.1, PHASE3 2.71), in
// one place so the driver, the README and Phase 6's tools cannot drift apart. Eight are
// byte-identical to the Phase 1 constants they replace; allow_missing_servos is new, and its false
// is the deliberate behaviour change of PHASE2_SPEC 6; feedback_mode is new in Phase 3, and
// io_timeout_ms is re-based on measurement there.
namespace defaults
{

constexpr const char * kPort = "/dev/ttyACM0";        // hpp:128
constexpr int kBaudrate = 1000000;                    // hpp:127
// Sized for the sync-read BURST, not for one transaction (PHASE3 5.19). Probe 3 Q2, with a wheel
// under load: a sync read of four ids fails 98.550 % at 1 ms and 0.000 % at 2, 3, 4, 5, 8, 10, 15
// and 20 ms, with the mean flat at 1.60-1.63 ms across that whole range -- so a larger timeout
// buys nothing when the bus is healthy. The upper bound is what a DEAD servo costs: the timeout is
// one overall budget per readSCS() call (probe 1 Q7: cost/timeout 1.00-1.03x from 2 to 50 ms), so
// one silent servo costs exactly one timeout per cycle. At 100 Hz, 5 ms leaves a 10 ms period
// running with half its budget gone; 10 ms consumes the period; the vendored 100 ms default stalls
// it for ten cycles. 5 ms is the largest value that keeps one dead servo survivable at 100 Hz and
// is 1.73x the worst clean sync read ever observed on this bench (2.892 ms, probe 3 Q5).
constexpr int kIoTimeoutMs = 5;                       // SCSerial::IOTimeOut
constexpr int kPingAttempts = 3;                      // hpp:138
constexpr int kMaxReadFails = 50;                     // hpp:139
constexpr bool kAllowMissingServos = false;           // new in Phase 2
constexpr const char * kProtocol = "sms_sts";         // new; scscl is reserved, not implemented
// 'auto' and not 'sync_read': these servos answer INST_SYNC_READ 5000/5000 [P1 Q6], but a
// different firmware on the same bus must degrade to the per-servo path rather than fail to
// activate, and only the activation probe can tell the two apart (PHASE3 2.71-2.72).
constexpr const char * kFeedbackMode = "auto";        // new in Phase 3
constexpr int kEncoderSteps = 4096;                   // hpp:131
constexpr double kCurrentPerCountA = 0.006;           // cpp:737's `6.0 / 1000.0`
constexpr double kTorqueConstantNmPerA = 0.8825985;   // hpp:130's 9.0 kgf cm/A, in N m/A

}  // namespace defaults

// The servo id ranges. kIdMin..kIdMax is what the hardware interface accepts for a joint's id;
// kIdMin0 is the bottom of the range a servo itself accepts, which the tools must be able to
// reach (a servo left at id 0 is invisible to the driver). 254 is the broadcast id, never a range
// end: every servo obeys it and none answers.
namespace limits
{

constexpr int kIdMin = 1, kIdMax = 253, kIdMin0 = 0;

}  // namespace limits
}  // namespace waveshare_servos

#endif  // DRIVER_DEFAULTS_HPP_
