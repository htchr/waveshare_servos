// servo_tools: the ROS-free bus logic of scan, set_id, calibrate_midpoint and factory_reset.
// See docs/design.md, "Tool structure".

#ifndef SERVO_TOOLS_HPP_
#define SERVO_TOOLS_HPP_

#include <csignal>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include "servo_bus.hpp"

namespace waveshare_servos
{
namespace tools
{

// Exit codes shared by the four tools; 0-3 match test/hil/stop_wheels.cpp.
// See docs/tools.md, "Exit codes".
enum class Exit : int
{
  kOk = 0,              // done and verified by reading back
  kPortHeld = 1,        // another process holds the port; nothing sent
  kCannotOpen = 2,      // any other open failure, or the port vanished before any write
  kNoAnswer = 3,        // the addressed servo is silent (scan: no id answered)
  kRefused = 4,         // a precondition failed before any EEPROM write
  kNotApplied = 5,      // a write went out and the EEPROM reads back as it was
  kInconsistent = 6,    // a write went out and the state is changed or unknown
  kScanAnomaly = 7,     // scan only: a row it cannot vouch for
  kUsage = 64,          // a parameter or argument error; the port was never opened
  kInternal = 70,       // an exception, or a status that validated parameters cannot produce
  kInterrupted = 130    // a stop before the EEPROM unlock; no EEPROM byte written
};

// The snake_case name the detail lines use (verdict=...), e.g. "port_held".
const char * exit_name(Exit exit) noexcept;

// typed uint8_t copies of SMS_STS.h's SMS_STS_MODEL_L, SMS_STS_ID, SMS_STS_BAUD_RATE,
// SMS_STS_OFS_L and SMS_STS_LOCK, the registers the tools address by these names
constexpr uint8_t kRegModelL = 3, kRegId = 5, kRegBaud = 6, kRegOffsetL = 31, kRegLock = 55;
constexpr uint8_t kIdentityFirst = 3, kIdentityBytes = 37;      // registers 3..39, all EEPROM
constexpr int kScanFirstId = 0, kScanLastId = 253;              // 254 is the broadcast id
constexpr int kMidpointTicks = 2048, kMidpointTolTicks = 3;     // H16 records the real error
constexpr uint32_t kVerifyWindowMs = 500, kEepromAckMs = 100, kSettleMaxMs = 2000;

struct Session                        // everything a run function needs; no globals
{
  ServoBus & bus;
  int attempts;                       // defaults::kPingAttempts
  uint32_t io_timeout_ms;             // io_timeout_ms_for(baudrate): the ack window of SRAM writes
  const volatile std::sig_atomic_t * stop;   // the signal flag; nullptr = never stopped
  std::ostream & out;                 // results only
  std::ostream & err;                 // diagnostics; buffered during a sequence
};

// max(defaults::kIoTimeoutMs, ceil(51 * 10 * 1000 / baudrate) + 2): an 8-byte request and the
// 43-byte reply to the identity block, at 10 bits a byte, plus 2 ms. 5 ms at 1 Mbaud.
// See docs/tools.md, "Tool parameters".
uint32_t io_timeout_ms_for(int baudrate) noexcept;

// Register 31-32 as the servo stores it: sign-magnitude with the direction on bit 11.
int offset_from_raw(uint16_t raw) noexcept;

// " (pid 4321 ros2_control_no)", one "pid <n> <comm>" per process that has `port` open, with
// self_pid left out; " (no holder visible in /proc; it may belong to another user)" when none is
// left. comm is at most 15 characters, cut by the kernel, so nothing may rely on it.
std::string holders_text(const std::string & port, int self_pid);

// The exit and the message for a failed ServoBus::open(); kOk and "" for a good one. `holders`
// is holders_text() for the port. Apart from open_bus() so every status is testable without a tty.
Exit open_failure(
  const OpenResult & opened, const std::string & port, const std::string & holders,
  std::string * message);

// Opens the bus. Holders are looked up under the canonical path (a /dev/serial/by-id link works),
// and the vendored "serial speed N" line goes to stderr, never stdout. Returns kOk with the bus
// open, or the exit with the reason written to `err`.
Exit open_bus(ServoBus & bus, const std::string & port, int baudrate, std::ostream & err);

// A late ack: a bare status frame from one of `ids` where another frame was expected -- a
// WRONG_ID six-byte frame (a ping caught it) or a STATUS_ONLY one (a read caught it).
// See docs/design.md, "Late acks".
bool is_late_ack(const Reply & reply, std::initializer_list<int> ids) noexcept;

// After the run: if the port is gone, 2 when no write went out (and not 130), else `outcome`.
Exit final_exit(Exit outcome, std::size_t writes_sent, bool port_exists) noexcept;

// ---- scan: read-only, it sends no WRITE, SYNC_WRITE or broadcast ----

struct ScanRow
{
  int id = -1;
  int status = -1;                    // its status byte: the feedback reply's, else the ping's
  bool identity = false;              // registers 3..39 were read
  int model = -1;                     // b3 | b4 << 8, raw
  int id_register = -1;               // b5
  int baud_register = -1;             // b6
  int offset_raw = -1;                // b31 | b32 << 8
  int mode = -1;                      // b33
  bool feedback = false;              // registers 56..70 were read
  int position = 0;                   // ticks, sign on bit 15
  int voltage_raw = -1;
  int temperature = -1;               // degrees C
  std::vector<std::string> anomalies;  // each an `id <n>: ...` line on stderr; any gives exit 7
  std::vector<std::string> notes;      // stderr only, no effect on the exit
};

struct ScanResult
{
  bool not_open = false;
  bool interrupted = false;
  int first = kScanFirstId;           // the range asked for
  int last = kScanLastId;
  int last_pinged = -1;               // the last id whose pings all went out
  int attempts = 0;
  double seconds = 0.0;
  std::vector<ScanRow> rows;          // ascending id: every id that sent anything back
};

ScanResult scan(Session & session, uint8_t first, uint8_t last);  // executable passes kScan*Id
// The table on `out` (header, one 11-token row per servo, footer), then each row's anomalies
// and notes and the closing hint on `err`. See docs/tools.md, "scan".
void print_scan(
  const ScanResult & result, const std::string & port, int baudrate, std::ostream & out,
  std::ostream & err);
// 130 interrupted, then 7 any anomaly, then 3 no row, else 0; 2 for a closed bus.
Exit scan_exit(const ScanResult & result) noexcept;

// ---- set_id and calibrate_midpoint ----
// In every report, a field not measured stays -1 / nullopt and prints `none` in the detail line.

struct SetIdReport
{
  Exit exit = Exit::kInternal;
  std::size_t writes_sent = 0;
  int late_ack_from = -1;
  int late_ack_ms = -1;               // from the id write to the reply that carried the late ack
  int start_id = -1;
  int new_id = -1;
  int lock_before = -1;               // register 55 of S before anything was written
  int unlock_read = -1;               // register 55 of S read back after writing 0
  std::string id_write_ack = "not_sent";   // old_id, new_id, other, none or garbled
  int ack_ms = -1;                    // the id write's ack, when there was one
  int verify_ms = -1;                 // from the id write to the end of the poll of N
  int new_id_pings = -1;              // pings of N in that poll
  std::optional<bool> old_id_silent;  // every ping of S after the write went unanswered
  std::optional<bool> identity_same;  // registers 3..39 read back equal, 5 aside
  int lock_after = -1;                // register 55 read back after the last lock written
  bool signal_deferred = false;       // a stop signal arrived during the sequence
};

SetIdReport set_id(Session & session, uint8_t start_id, uint8_t new_id);

struct CalibrateReport
{
  Exit exit = Exit::kInternal;
  std::size_t writes_sent = 0;
  int late_ack_from = -1;
  int late_ack_ms = -1;               // from the 128 write to the reply that carried the late ack
  int id = -1;
  int mode = -1;
  int torque_before = -1;
  bool torque_written = false;        // step 4 switched the torque off
  int settle_ms = -1;
  std::optional<int> position_before;  // the settled, torque-off position the verdict uses
  int offset_raw_before = -1;
  int unlock_read = -1;
  std::string calibrate_ack = "not_sent";  // old_id, other, none or garbled
  int ack_ms = -1;
  std::optional<int> position_after;
  int offset_raw_after = -1;
  int offset_sign = 0;                // +1: the offset moved the way position_before - 2048 points
  int register40_after = -1;          // what the firmware left in register 40 after the 128
  int torque_final = -1;
  std::optional<bool> identity_same;  // registers 3..39 read back equal, 31-32 aside
  int lock_after = -1;                // register 55 read back after the last lock written
  bool signal_deferred = false;
};

CalibrateReport calibrate_midpoint(Session & session, uint8_t id);

// ---- factory_reset ----

// Register 6's factory value, 0, is this rate; an ST3025 reset at another rate came back at it.
// See docs/tools.md, "factory_reset".
constexpr int kFactoryBaudrate = 1000000;

struct FactoryResetReport
{
  Exit exit = Exit::kInternal;
  std::size_t writes_sent = 0;        // write transactions and the RESET put on the wire
  int late_ack_from = -1;
  int late_ack_ms = -1;               // from the RESET to the reply that carried the late ack
  int id = -1;
  int model = -1;                     // registers 3-4 before the reset
  int baud_register_before = -1;
  int offset_raw_before = -1;
  int mode_before = -1;
  int torque_before = -1;
  bool torque_written = false;        // the torque was switched off before the RESET
  int lock_before = -1;
  std::string reset_ack = "not_sent";  // old_id, other, none or garbled
  int ack_ms = -1;
  int baudrate_after = -1;            // the line rate the servo was looked for (and read) at last
  int verify_ms = -1;                 // from the RESET to the end of the read-back poll
  int baud_register_after = -1;
  int offset_raw_after = -1;
  int mode_after = -1;
  int changed_registers = -1;         // bytes of 3..39 that read differently afterwards
  int torque_final = -1;
  int lock_after = -1;
  bool signal_deferred = false;
};

// One servo back to its factory settings with RESET (0x06), verified by reading back at the
// factory rate. The ST3025 keeps its id, so the tool finds it again; nothing is broadcast.
// May leave the bus at kFactoryBaudrate.
FactoryResetReport factory_reset(Session & session, uint8_t id);

// "detail start_id=4 new_id=253 ... verdict=ok": one line of key=value tokens, no newline and no
// "<tool>: " prefix (the executable's stream adds it). The HIL gates parse it.
std::string detail_line(const SetIdReport & report);
std::string detail_line(const CalibrateReport & report);
std::string detail_line(const FactoryResetReport & report);

}  // namespace tools
}  // namespace waveshare_servos

#endif  // SERVO_TOOLS_HPP_
