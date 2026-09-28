// Defaults of the hardware parameters and the servo id ranges, for the driver and the tools.
// ROS-free and header-only, so the tools include it without hardware_interface. Not installed.

#ifndef DRIVER_DEFAULTS_HPP_
#define DRIVER_DEFAULTS_HPP_

namespace waveshare_servos
{

// Defaults of the eleven hardware parameters; test_readme.py checks the docs table against them.
// See docs/configuration.md, "Hardware parameters".
namespace defaults
{

constexpr const char * kPort = "/dev/ttyACM0";
constexpr int kBaudrate = 1000000;
// Sized for a whole sync-read burst (worst clean read on the reference bench: 2.892 ms), yet one
// silent servo is still survivable at 100 Hz.
// See docs/bus-timing.md, "Transaction timeout".
constexpr int kIoTimeoutMs = 5;                       // SCSerial::IOTimeOut
constexpr int kPingAttempts = 3;
constexpr int kMaxReadFails = 50;
constexpr bool kAllowMissingServos = false;
constexpr const char * kProtocol = "sms_sts";         // 'scscl' is refused: not implemented
// 'auto', not 'sync_read': other firmware on the same bus then falls back to per-servo reads
// instead of failing activation. See docs/bus-timing.md, "Feedback mode".
constexpr const char * kFeedbackMode = "auto";
constexpr int kEncoderSteps = 4096;
constexpr double kCurrentPerCountA = 0.006;           // A per current count
constexpr double kTorqueConstantNmPerA = 0.8825985;   // 9.0 kgf cm/A, in N m/A

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
