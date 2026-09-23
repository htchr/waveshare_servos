// hil_eeprom core -- the bench's EEPROM oracle and repair tool, as a library (PHASE6_SPEC E.1).
//
// Everything hil_eeprom does to a bus lives here, so test/test_hil_eeprom.cpp can drive it on the
// fake bus in-process (D.6); test/hil/eeprom.cpp is only the signal handlers and a call to run().
//
// Independent of servo_tools ON PURPOSE (R12): it reaches the servos through the vendored Ping,
// Read, readByte, readWord, writeByte and writeWord only, never through ServoBus::checked_*, so a
// bug in the tools' transactions cannot hide itself behind the oracle that checks them. It still
// goes through ServoBus::open, so it takes the same exclusive lock and is refused like the tools.
//
// A snapshot is every EEPROM byte (0, 1, 3..39, read singly), SRAM 40 and 55, and a few volatile
// registers kept for evidence. It is written as a plain-text .snap that compare and restore parse
// back, and as the JSON on the RESULT line that the HIL gates read. An unreadable byte is `x` in
// both, never 0: a restore that read `x` as 0 would write 0 into an id, limit or mode register.

#ifndef HIL__EEPROM_CORE_HPP_
#define HIL__EEPROM_CORE_HPP_

#include <csignal>
#include <cstdint>
#include <map>
#include <ostream>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "servo_bus.hpp"

namespace waveshare_servos
{
namespace hil_eeprom
{

// Exit codes (E.1). 1 means three things, one per subcommand that can return it.
constexpr int kExitOk = 0;
constexpr int kExitPortHeld = 1;
constexpr int kExitNotEqual = 1;        // compare
constexpr int kExitBlockMismatch = 1;   // blockcheck
constexpr int kExitCannotOpen = 2;
constexpr int kExitCannotResolve = 3;   // cannot resolve, verify or read; a refused source
constexpr int kExitUsage = 64;
constexpr int kExitInternal = 70;       // an exception; never a verdict
constexpr int kExitInterrupted = 130;   // a signal before the first write; nothing was written

constexpr int kBaudrate = 1000000;
constexpr uint32_t kIoTimeoutMs = 20;
constexpr uint32_t kCensusTimeoutMs = 5;
constexpr int kCensusAttempts = 2;
constexpr int kCensusFirstId = 0;
constexpr int kCensusLastId = 253;
// Every read is tried twice before it is recorded as unreadable: one lost frame on the bench must
// not turn a snapshot into a refusal, and a byte that fails twice is still `x`.
constexpr int kReadAttempts = 2;
// How long a read-back may take to show a write. An EEPROM commit can hold the servo off the bus
// for longer than its ack window (the fake models this, D.1 #13), so a read-back polls.
constexpr uint32_t kVerifyWindowMs = 1000;
constexpr uint32_t kVerifyPollMs = 10;
// The id move: poll Ping(new) this long before calling it failed (E.1 restore step 1.3).
constexpr uint32_t kMoveWindowMs = 500;
constexpr uint32_t kDriftSampleMs = 50;
// The longest blockcheck block: the vendored Read's reply buffer is bBuf[255] (src/SCS.cpp:179)
// and the E.0 probe needs 37. A constant of its own rather than ServoBus::checked_max_bytes, so
// nothing here leans on the tools' transactions.
constexpr int kMaxBlockBytes = 64;

// Register addresses (context/sts3215_memory_table.xlsx; SMS_STS.h names most of them).
constexpr int kEepromLast = 39;
constexpr int kRegId = 5;
constexpr int kRegBaud = 6;
constexpr int kRegOffset = 31;
constexpr int kRegMode = 33;
constexpr int kRegTorque = 40;
constexpr int kRegGoalPosition = 42;
constexpr int kRegGoalSpeed = 46;
constexpr int kRegLock = 55;
constexpr int kRegPresentPosition = 56;
constexpr int kUnreadable = -1;         // `x`

// The registers a snapshot holds and compare compares: EEPROM 0, 1, 3..39 (address 2 is not
// defined), then SRAM 40 and 55.
const std::vector<int> & eeprom_registers();
const std::vector<int> & compared_registers(bool eeprom_only);
// The first register of each two-byte EEPROM field; restore rewrites these with one writeWord.
const std::vector<int> & word_fields();
// The volatile registers (evidence only, never compared): {register, is_word}.
const std::vector<std::pair<int, bool>> & volatile_registers();

struct ServoRecord
{
  std::map<int, int> regs;       // compared_registers(false) -> value or kUnreadable
  std::map<int, int> volatiles;  // 42, 56 (words), 62, 63, 65 -> value or kUnreadable
  int readable_eeprom() const;   // the JSON's `n`; 39 when every EEPROM byte was read
  bool complete() const;         // every register, volatile ones included, was read
  int reg(int address) const;    // kUnreadable when absent
};

struct Snapshot
{
  std::vector<int> ids;          // as listed on the command line, in that order
  bool ok = false;               // every listed id has a servo line and every byte was read
  bool has_census = false;
  std::vector<int> census;       // ascending
  std::map<int, ServoRecord> servos;
};

// ---- the .snap text and the RESULT JSON ----
std::string format_snap(const Snapshot & snap);
bool parse_snap(const std::string & text, Snapshot * snap, std::string * error);
std::string snapshot_json(const Snapshot & snap);

// ---- bus reads (all through the vendored calls) ----
// The ids that answer Ping over kCensusFirstId..kCensusLastId, kCensusAttempts each, under
// kCensusTimeoutMs; the bus's io timeout is restored afterwards. `stop` is polled between ids.
std::vector<int> census(
  ServoBus & bus, const volatile std::sig_atomic_t * stop, bool * interrupted);
ServoRecord read_servo(ServoBus & bus, int id);
Snapshot take_snapshot(
  ServoBus & bus, const std::vector<int> & ids, bool with_census,
  const volatile std::sig_atomic_t * stop, bool * interrupted);

// ---- compare (opens no port) ----
struct Diff
{
  int id = -1;
  int reg = -1;                  // -1 for a missing id or the census
  std::string a;                 // value in A (`x` when unreadable)
  std::string b;
  std::string missing_in;        // "A" or "B" for an id present in only one file
  bool census = false;
};
std::vector<Diff> compare(const Snapshot & a, const Snapshot & b, bool eeprom_only);
std::string diff_text(const Diff & diff);

// ---- restore ----
struct RestoreOptions
{
  bool limit_regs = false;       // --allow-regs was given
  std::set<int> allow_regs;
};

struct Written
{
  int id = -1;
  int reg = -1;
  int from = kUnreadable;
  int to = 0;
  int bytes = 1;                 // 2 for a writeWord
  bool eeprom = false;           // register 0..39; 40, 42, 46 and 55 are SRAM
};

struct RestoreReport
{
  int exit = kExitInternal;
  std::vector<std::string> problems;   // why it refused or failed, one line each
  std::vector<Diff> before;            // bench vs source before any write (a = source, b = bench)
  int moved_from = -1;
  int moved_to = -1;
  std::vector<Written> writes;         // every write put on the wire, in order
  std::vector<Diff> after;             // source vs the fresh snapshot; empty = equal
  bool signal_deferred = false;        // a signal arrived during the writes, which completed first
};

// Step 0 of restore: why a source cannot be restored from (empty = it can). Needs no bus.
std::vector<std::string> source_problems(const Snapshot & source);
RestoreReport restore(
  ServoBus & bus, const Snapshot & source, const RestoreOptions & options,
  const volatile std::sig_atomic_t * stop);

// ---- blockcheck (read-only) ----
struct BlockMismatch
{
  int id = -1;
  int reg = -1;
  int block = kUnreadable;
  int single = kUnreadable;
};

struct BlockcheckReport
{
  int exit = kExitInternal;
  bool ok = false;
  std::vector<std::string> unreadable;   // "id 2 block", "id 2 reg 13"
  std::vector<BlockMismatch> mismatches;
};
BlockcheckReport blockcheck(
  ServoBus & bus, const std::vector<int> & ids, int first, int count,
  const volatile std::sig_atomic_t * stop);

// ---- drift (SRAM only) ----
struct DriftReport
{
  int exit = kExitInternal;
  bool ok = false;
  std::string problem;
  int torque_before = kUnreadable;
  bool torque_written = false;
  int goal_before = kUnreadable;
  std::vector<int> samples;            // present position every kDriftSampleMs; kUnreadable = x
  int drift_total_ticks = -1;          // max - min over every sample
  int drift_last_1s_ticks = -1;        // max - min over the samples of the last second
  int goal_after = kUnreadable;
  int torque_after = kUnreadable;
  bool restored = false;
  std::vector<Written> writes;
  bool signal_deferred = false;
};
DriftReport drift(
  ServoBus & bus, int id, double seconds, const volatile std::sig_atomic_t * stop);

// ---- read (read-only) ----
struct ReadReport
{
  int exit = kExitInternal;
  bool ok = false;
  int value = kUnreadable;
};
ReadReport read_register(ServoBus & bus, int id, int address, bool word);

// ---- the command line ----
// hil_eeprom [--port P] <snapshot|compare|restore|blockcheck|drift|read> ...; human-readable lines,
// then `RESULT {json}`, all on `out`. Returns the exit code. `stop` is the signal flag.
int run(
  const std::vector<std::string> & args, std::ostream & out,
  const volatile std::sig_atomic_t * stop);

}  // namespace hil_eeprom
}  // namespace waveshare_servos

#endif  // HIL__EEPROM_CORE_HPP_
