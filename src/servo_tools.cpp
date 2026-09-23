#include "servo_tools.hpp"

#include <pthread.h>
#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <ostream>
#include <sstream>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include "driver_defaults.hpp"
#include "units.hpp"

namespace waveshare_servos
{
namespace tools
{
namespace
{

// servo_tools.hpp's typed copies of the vendored register names (review fix F29): kept equal.
static_assert(kRegModelL == SMS_STS_MODEL_L && kRegId == SMS_STS_ID, "register names drifted");
static_assert(kRegBaud == SMS_STS_BAUD_RATE && kRegOffsetL == SMS_STS_OFS_L, "register drift");
static_assert(kRegLock == SMS_STS_LOCK, "register names drifted");

// One process that has the port open, as /proc shows it.
struct Holder
{
  int pid = -1;
  std::string comm;                   // at most 15 characters, cut by the kernel; may be empty
};

// Every process with `canonical` open except `self_pid`. port_holder_pids matches the /proc link
// target literally, and those links always name the real device, hence the canonical path.
std::vector<Holder> other_holders(const std::string & canonical, int self_pid)
{
  std::vector<Holder> holders;
  for (const int pid : port_holder_pids(canonical)) {
    if (pid == self_pid) {
      continue;
    }
    Holder holder;
    holder.pid = pid;
    std::ifstream comm("/proc/" + std::to_string(pid) + "/comm");
    std::getline(comm, holder.comm);
    holders.push_back(holder);
  }
  return holders;
}

// "pid 4321 ros2_control_no, pid 77 screen"
std::string holder_list(const std::vector<Holder> & holders)
{
  std::string text;
  for (const Holder & holder : holders) {
    text += (text.empty() ? "" : ", ") + std::string("pid ") + std::to_string(holder.pid);
    if (!holder.comm.empty()) {
      text += " " + holder.comm;
    }
  }
  return text;
}

using Clock = std::chrono::steady_clock;

bool stop_requested(const Session & session)
{
  return session.stop != nullptr && *session.stop != 0;
}

// Register 6 as the memory table defines it: 0..7 are 1000000, 500000, 250000, 128000, 115200,
// 76800, 57600 and 38400 bits/s. -1 for anything else.
int baud_of_register(int value)
{
  constexpr int kRates[] = {1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400};
  return (value >= 0 && value < 8) ? kRates[value] : -1;
}

int word_at(const std::vector<uint8_t> & data, std::size_t offset)
{
  return static_cast<int>(data[offset]) | (static_cast<int>(data[offset + 1]) << 8);
}

std::string hex_byte(int value)
{
  char text[8];
  std::snprintf(text, sizeof(text), "0x%02x", value & 0xff);
  return text;
}

// The identity block (registers 3..39) as offsets into its payload.
constexpr std::size_t at(uint8_t reg) {return static_cast<std::size_t>(reg - kIdentityFirst);}

// The stderr line for a ping that got something other than silence or one clean reply.
std::string ping_anomaly(int id, const Reply & reply)
{
  switch (reply.kind) {
    case ReplyKind::EXTRA:
      return std::to_string(reply.extra_bytes) + " extra bytes followed its reply; two servos may "
             "share this id -- connect them one at a time and use set_id";
    case ReplyKind::GARBLED:
      return "garbled replies: two servos may share this id, or the line is noisy";
    case ReplyKind::WRONG_ID:
      return "a reply to id " + std::to_string(id) + " came from id " +
             std::to_string(reply.from_id);
    case ReplyKind::NOT_OPEN:
    case ReplyKind::INVALID_ID:
    case ReplyKind::INVALID_COUNT:
    case ReplyKind::SILENT:
    case ReplyKind::ONE:
    case ReplyKind::STATUS_ONLY:
      break;
  }
  return std::string("an unexpected reply (") + to_string(reply.kind) + ")";
}

// `anomaly` added to `anomalies` unless an earlier reply already put it there.
void add_once(std::vector<std::string> * anomalies, const std::string & anomaly)
{
  if (std::find(anomalies->begin(), anomalies->end(), anomaly) == anomalies->end()) {
    anomalies->push_back(anomaly);
  }
}

// C.1 step 3: "A read that is not ONE is retried once." A first reply that was doubled, garbled
// or from another id is still twin evidence when the retry comes back clean, so it is kept as an
// anomaly of the row, exactly as the ping path keeps one (R10; review fix F6).
Reply read_with_retry(
  ServoBus & bus, uint8_t id, uint8_t first, uint8_t count, std::vector<std::string> * anomalies)
{
  const Reply reply = bus.checked_read(id, first, count);
  if (reply) {
    return reply;
  }
  if (reply.kind == ReplyKind::EXTRA || reply.kind == ReplyKind::GARBLED ||
    reply.kind == ReplyKind::WRONG_ID)
  {
    add_once(anomalies, ping_anomaly(id, reply));
  }
  return bus.checked_read(id, first, count);
}

// One table row: 11 whitespace-separated tokens, `?` for whatever could not be read and for
// whatever derives from it, right-aligned under the header's column ends (C.1).
std::string scan_row_text(const ScanRow & row)
{
  const auto known = [](bool readable, const std::string & text) {
      return readable ? text : std::string("?");
    };
  std::string type = "?";
  if (row.identity) {
    type = row.mode == 0 ? "pos" : (row.mode == 1 ? "vel" : "-");
  }
  const int baud = row.identity ? baud_of_register(row.baud_register) : -1;
  char voltage[16] = "?";
  if (row.feedback) {
    std::snprintf(voltage, sizeof(voltage), "%.1f", raw_to_volts(row.voltage_raw));
  }
  char text[160];
  std::snprintf(
    text, sizeof(text), "%3d  %-4s%6s%7s%10s%9s%10s%11s%8s%8s%8s", row.id, type.c_str(),
    known(row.identity, std::to_string(row.mode)).c_str(),
    known(row.identity, std::to_string(row.model)).c_str(),
    known(row.identity, std::to_string(row.baud_register)).c_str(),
    known(baud > 0, std::to_string(baud)).c_str(),
    known(row.feedback, std::to_string(row.position)).c_str(), voltage,
    known(row.feedback, std::to_string(row.temperature)).c_str(),
    known(row.status >= 0, hex_byte(row.status)).c_str(),
    known(row.identity, std::to_string(offset_from_raw(static_cast<uint16_t>(row.offset_raw))))
    .c_str());
  return text;
}

int elapsed_ms(Clock::time_point since)
{
  return static_cast<int>(
    std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - since).count());
}

// "1", or "nothing (silent)" when the read that should have said got no clean reply.
std::string byte_read(const Reply & reply)
{
  return reply ? std::to_string(reply.data[0]) :
         std::string("nothing (") + to_string(reply.kind) + ")";
}

// One anomalous reply, named for a message: "wrong_id from id 99 at a ping of 253".
std::string reply_text(const Reply & reply, const char * transaction, int id)
{
  std::string text = to_string(reply.kind);
  if (reply.from_id >= 0) {
    text += " from id " + std::to_string(reply.from_id);
  }
  return text + " at a " + transaction + " of " + std::to_string(id);
}

// C.0 "Deferred signals" and "Quiet sequence", from the first write to the end of a sequence: the
// four stop signals are blocked on this thread (their handlers stay installed, so a signal
// delivered to another thread still only sets the flag), and every diagnostic goes to a buffer
// that is printed once the sequence is over. With SIGPIPE ignored as well, nothing a user or a
// closed pipe does can stop the tool between an EEPROM unlock and its lock.
class Sequence
{
public:
  explicit Sequence(Session & session)
  : session_(session) {}

  ~Sequence() {unblock();}

  Sequence(const Sequence &) = delete;
  Sequence & operator=(const Sequence &) = delete;
  Sequence(Sequence &&) = delete;
  Sequence & operator=(Sequence &&) = delete;

  // The pre-write notice goes out first, deliberately before the first write: it is the only
  // help a SIGKILL leaves the user.
  void begin(const std::string & notice)
  {
    session_.err << notice << "\n";
    session_.err.flush();
    sigset_t stops;
    sigemptyset(&stops);
    for (const int signal_number : {SIGINT, SIGTERM, SIGHUP, SIGQUIT}) {
      sigaddset(&stops, signal_number);
    }
    blocked_ = ::pthread_sigmask(SIG_BLOCK, &stops, &saved_) == 0;
    started_ = true;
  }

  std::ostream & err() {return started_ ? buffer_ : session_.err;}

  // The last stop check, just before the unlock (review fix F7): the flag, or a stop signal held
  // pending by this sequence's own mask -- its handler has not run, so the flag cannot show it.
  bool stop_pending() const
  {
    if (stop_requested(session_)) {
      return true;
    }
    sigset_t pending;
    sigemptyset(&pending);
    if (::sigpending(&pending) != 0) {
      return false;
    }
    for (const int signal_number : {SIGINT, SIGTERM, SIGHUP, SIGQUIT}) {
      if (sigismember(&pending, signal_number) == 1) {
        return true;
      }
    }
    return false;
  }

  // Unblocks (a pending signal runs its handler now), then prints what was held back. Returns
  // whether a stop signal arrived while the sequence ran.
  bool finish()
  {
    unblock();
    session_.err << buffer_.str();
    buffer_.str("");
    started_ = false;
    return stop_requested(session_);
  }

private:
  void unblock()
  {
    if (blocked_) {
      ::pthread_sigmask(SIG_SETMASK, &saved_, nullptr);
      blocked_ = false;
    }
  }

  Session & session_;
  std::ostringstream buffer_;
  sigset_t saved_{};
  bool blocked_ = false;
  bool started_ = false;
};

// The transactions of one set_id or calibrate run: every write is counted (the report's
// writes_sent is compared with the wire in the tests), and from the committing write on, the one
// tolerated late ack (C.0) is recognised, recorded and its transaction repeated.
class Wire
{
public:
  Wire(ServoBus & bus, std::size_t * writes_sent, int * late_from, int * late_ms)
  : bus_(bus), writes_sent_(writes_sent), late_from_(late_from), late_ms_(late_ms) {}

  Reply write(uint8_t id, uint8_t reg, uint8_t value, uint32_t ack_window_ms)
  {
    return counted(bus_.checked_write(id, reg, &value, 1, ack_window_ms));
  }

  // factory_reset's RESET, counted like a write: it changes the servo, and "2: nothing sent" after
  // it would be a lie (final_exit).
  Reply reset(uint8_t id, uint32_t ack_window_ms)
  {
    return counted(bus_.checked_reset(id, ack_window_ms));
  }

  // The committing write is about to go out: from here a bare status frame from `a` or `b` is
  // the one late ack a run may catch. `log` gets its line (the sequence's buffer).
  void arm(int a, int b, const std::string & write, std::ostream * log)
  {
    late_a_ = a;
    late_b_ = b;
    write_ = write;
    log_ = log;
    committed_ = Clock::now();
  }

  Clock::time_point committed() const {return committed_;}

  // True for the first reply that is_late_ack() accepts after arm(); it is recorded and the
  // caller repeats the transaction. A second one, like any other anomaly, is the caller's to judge.
  bool tolerated(const Reply & reply, const char * transaction, int id)
  {
    if (log_ == nullptr || *late_from_ != -1 || !is_late_ack(reply, {late_a_, late_b_})) {
      return false;
    }
    *late_from_ = reply.from_id;
    *late_ms_ = elapsed_ms(committed_);
    *log_ << "late ack from id " << reply.from_id << ", " << *late_ms_ << " ms after the " <<
      write_ << ", caught by a " << transaction << " of id " << id << "; it was repeated\n";
    return true;
  }

  Reply read(uint8_t id, uint8_t first, uint8_t count)
  {
    Reply reply = bus_.checked_read(id, first, count);
    while (tolerated(reply, "read", id)) {
      reply = bus_.checked_read(id, first, count);
    }
    return reply;
  }

  // Writes `value` to register 55 and reads it back: the verified unlock (0) and lock (1) of C.0.
  // Only a read-back of 55 itself can show that a lock did not open; a read-back of the payload
  // cannot. Returns the read-back.
  Reply set_lock(uint8_t id, uint8_t value)
  {
    write(id, kRegLock, value, kEepromAckMs);
    return read(id, kRegLock, 1);
  }

private:
  // A refusal the bus made before sending is no transaction on the wire.
  Reply counted(const Reply & reply)
  {
    if (reply.kind != ReplyKind::NOT_OPEN && reply.kind != ReplyKind::INVALID_ID &&
      reply.kind != ReplyKind::INVALID_COUNT)
    {
      (*writes_sent_)++;
    }
    return reply;
  }

  ServoBus & bus_;
  std::size_t * writes_sent_;
  int * late_from_;
  int * late_ms_;
  int late_a_ = -1;
  int late_b_ = -1;
  std::string write_;
  std::ostream * log_ = nullptr;
  Clock::time_point committed_{};
};

// The end of a sequence: signals unblocked, the held-back diagnostics printed, then the outcome
// -- a success on `out`, anything else on `err` -- and the deferred-signal note when one came. An
// exit 130 is the signal acted upon before the unlock, so it gets no "completed first".
template<typename Report>
Report conclude(
  Report & report, Session & session, Sequence & sequence, Exit exit, const std::string & message)
{
  report.exit = exit;
  report.signal_deferred = sequence.finish();
  (exit == Exit::kOk ? session.out : session.err) << message << "\n";
  if (report.signal_deferred && exit != Exit::kInterrupted) {
    session.err << "a signal arrived during the EEPROM sequence; it was completed first\n";
  }
  return report;
}

// Registers 56-57, the present position: sign-magnitude on bit 15, like ReadPos(-1).
int position_of(const Reply & reply)
{
  const int word = word_at(reply.data, 0);
  return (word & 0x8000) != 0 ? -(word & 0x7fff) : word;
}

std::string raw_word(int value)
{
  char text[16];
  std::snprintf(text, sizeof(text), "0x%04x", value & 0xffff);
  return text;
}

// The best-effort relock after an unlock that did not verify (C.0), read back and recorded
// (review fix F1): "" when 55 reads 1 again, else the clause exit 5 must carry -- A.3 allows SRAM
// 55 to differ only when the message names it.
std::string relock_clause(const Reply & relocked)
{
  if (relocked && relocked.data[0] == 1) {
    return "";
  }
  return "; its EEPROM lock may still be open until it is power-cycled (register 55 reads " +
         byte_read(relocked) + ")";
}

std::string port_held_message(const std::string & port, const std::string & holders)
{
  return "port '" + port + "' is held by another process" + holders + "; refusing to share the "
         "bus -- stop that process first (a running controller manager holds the port for as "
         "long as its hardware component is configured). Nothing was sent to the servos.";
}

}  // namespace

const char * exit_name(Exit exit) noexcept
{
  // No `default:` label, so a new Exit without a name here is a -Wswitch warning.
  switch (exit) {
    case Exit::kOk:
      return "ok";
    case Exit::kPortHeld:
      return "port_held";
    case Exit::kCannotOpen:
      return "cannot_open";
    case Exit::kNoAnswer:
      return "no_answer";
    case Exit::kRefused:
      return "refused";
    case Exit::kNotApplied:
      return "not_applied";
    case Exit::kInconsistent:
      return "inconsistent";
    case Exit::kScanAnomaly:
      return "scan_anomaly";
    case Exit::kUsage:
      return "usage";
    case Exit::kInternal:
      return "internal";
    case Exit::kInterrupted:
      return "interrupted";
  }
  return "unknown";
}

uint32_t io_timeout_ms_for(int baudrate) noexcept
{
  const uint32_t floor = static_cast<uint32_t>(defaults::kIoTimeoutMs);
  if (baudrate <= 0) {
    return floor;
  }
  // 51 bytes of 10 bits each, in milliseconds, rounded up: integer arithmetic, so the table in
  // C.0 is exact and no libm rounding mode can move it.
  constexpr uint64_t kBitMilliseconds = 51u * 10u * 1000u;
  const uint64_t rate = static_cast<uint64_t>(baudrate);
  const uint32_t wire = static_cast<uint32_t>((kBitMilliseconds + rate - 1) / rate) + 2u;
  return std::max(floor, wire);
}

int offset_from_raw(uint16_t raw) noexcept
{
  // -(raw & ~bit) like the driver's signed_on_bit_ten (src/servo_bus.cpp:62-65): bits above the
  // sign stay in the magnitude rather than being masked away, so garbage shows as garbage.
  constexpr uint16_t kSign = 1u << 11;
  return (raw & kSign) != 0 ? -static_cast<int>(raw & ~kSign) : static_cast<int>(raw);
}

std::string holders_text(const std::string & port, int self_pid)
{
  const std::vector<Holder> holders = other_holders(port, self_pid);
  if (holders.empty()) {
    return " (no holder visible in /proc; it may belong to another user)";
  }
  return " (" + holder_list(holders) + ")";
}

Exit open_failure(
  const OpenResult & opened, const std::string & port, const std::string & holders,
  std::string * message)
{
  message->clear();
  const int error = opened.error;
  switch (opened.status) {
    case BusStatus::OK:
      return Exit::kOk;
    case BusStatus::LOCK_FAILED:
      *message = port_held_message(port, holders);
      return Exit::kPortHeld;
    case BusStatus::LOCK_OPEN_FAILED:
      // EBUSY is TIOCEXCL: whoever holds the port marked it exclusive, as ServoBus does.
      if (error == EBUSY) {
        *message = port_held_message(port, holders);
        return Exit::kPortHeld;
      }
      if (error == ENOENT || error == ENXIO) {
        *message = "port '" + port + "' does not exist; 'ls /dev/ttyACM* /dev/ttyUSB*' lists "
          "what is plugged in";
        return Exit::kCannotOpen;
      }
      if (error == EACCES) {
        *message = "no permission to open '" + port + "'; add yourself to the dialout group "
          "(sudo usermod -aG dialout $USER) and log in again";
        return Exit::kCannotOpen;
      }
      break;
    case BusStatus::NOT_A_TTY:
      *message = "'" + port + "' is not a serial device";
      return Exit::kCannotOpen;
    case BusStatus::OPEN_FAILED:
    case BusStatus::TERMIOS_FAILED:
    case BusStatus::EXCLUSIVE_FAILED:
      break;
    case BusStatus::UNSUPPORTED_BAUDRATE:
    case BusStatus::INVALID_TIMEOUT:
    case BusStatus::ALREADY_OPEN:
      // parse_params() refused every value that could produce these, so reaching one is a bug
      *message = std::string("internal error: ") + to_string(opened.status) +
        " after the parameters were validated";
      return Exit::kInternal;
  }
  *message = "could not take '" + port + "': " + to_string(opened.status);
  if (error != 0) {
    *message += std::string(": ") + std::strerror(error);
  }
  return Exit::kCannotOpen;
}

Exit open_bus(ServoBus & bus, const std::string & port, int baudrate, std::ostream & err)
{
  // Step 1: the holders are looked up under the real device, so a /dev/serial/by-id path finds
  // them; the open itself and every message use the path as given.
  std::error_code ec;
  std::string canonical = std::filesystem::weakly_canonical(port, ec).string();
  if (ec || canonical.empty()) {
    canonical = port;
  }

  // Step 2: stdout is the tools' result channel, and SCSerial::begin() printf()s "serial speed N"
  // to it (ServoBus::open flushes right after). For the length of the open, descriptor 1 is
  // descriptor 2, so the line lands on stderr; a refusal before begin() prints it nowhere.
  std::fflush(stdout);
  const int saved = ::dup(STDOUT_FILENO);
  if (saved != -1) {
    ::dup2(STDERR_FILENO, STDOUT_FILENO);
  }
  const OpenResult opened = bus.open(port, baudrate, io_timeout_ms_for(baudrate));
  std::fflush(stdout);
  if (saved != -1) {
    ::dup2(saved, STDOUT_FILENO);
    ::close(saved);
  }

  // Step 3.
  if (!opened) {
    std::string message;
    const Exit exit = open_failure(opened, port, holders_text(canonical, ::getpid()), &message);
    err << message << "\n";
    return exit;
  }

  // Step 4: warnings only. Nothing can evict a process that had the port before the lock was
  // taken, and the CLI tests need to run while their fake keeps the slave end open.
  const std::vector<Holder> others = other_holders(canonical, ::getpid());
  if (!others.empty()) {
    err << "process(es) " << holder_list(others) << " had '" << port <<
      "' open before this tool took it; they hold no lock and can still write to the bus\n";
  }
  if (::geteuid() == 0) {
    err << "running as root: the kernel lets a root process open '" << port <<
      "' even though it is marked exclusive, so only the advisory lock protects this bus, and "
      "only against programs that take it\n";
  }
  return Exit::kOk;
}

bool is_late_ack(const Reply & reply, std::initializer_list<int> ids) noexcept
{
  // A ping that caught it sees a well-formed six-byte frame from the wrong id; a read that caught
  // it sees a bare status frame where its payload should be (B.4, appendix J #3).
  const bool shaped = (reply.kind == ReplyKind::WRONG_ID && reply.frame_bytes == 6) ||
    reply.kind == ReplyKind::STATUS_ONLY;
  return shaped && std::find(ids.begin(), ids.end(), reply.from_id) != ids.end();
}

Exit final_exit(Exit outcome, std::size_t writes_sent, bool port_exists) noexcept
{
  // A port that vanished during the run (a USB drop) says nothing about the servos unless a
  // write went out: then the run's own outcome is still the truth about their state, and "2:
  // nothing sent" would be a lie. 130 stays 130: the run stopped on a signal either way.
  if (port_exists || outcome == Exit::kInterrupted || writes_sent > 0) {
    return outcome;
  }
  return Exit::kCannotOpen;
}

ScanResult scan(Session & session, uint8_t first, uint8_t last)
{
  ScanResult result;
  // 254 is the broadcast and 255 a header byte: neither can answer as one servo.
  result.first = first;
  result.last = std::min<int>(last, kScanLastId);
  result.attempts = std::max(1, session.attempts);
  if (!session.bus.is_open()) {
    result.not_open = true;
    return result;
  }
  ServoBus & bus = session.bus;
  const Clock::time_point started = Clock::now();
  for (int id = result.first; id <= result.last; id++) {
    // Step 1, between ids only: a signal ends the scan with the rows found so far.
    if (stop_requested(session)) {
      result.interrupted = true;
      break;
    }
    // Step 2: up to `attempts` pings, and none after the first clean answer. Anything else that
    // came back is kept: a twin that answers cleanly once is still a twin.
    ScanRow row;
    row.id = id;
    const uint8_t address = static_cast<uint8_t>(id);
    bool answered = false;
    std::vector<Reply> odd;
    for (int attempt = 0; attempt < result.attempts && !answered; attempt++) {
      const Reply reply = bus.checked_ping(address);
      if (reply) {
        answered = true;
        row.status = reply.status;
      } else if (reply.kind != ReplyKind::SILENT) {
        odd.push_back(reply);
      }
    }
    result.last_pinged = id;
    if (!answered && odd.empty()) {
      continue;                       // silent on every attempt: nobody there
    }
    for (const Reply & reply : odd) {
      add_once(&row.anomalies, ping_anomaly(id, reply));
    }
    if (!answered) {
      result.rows.push_back(row);     // an anomaly row: no register is read from an unclean id
      continue;
    }

    // Step 3: the identity block and the feedback block, each retried once.
    const Reply identity = read_with_retry(
      bus, address, kIdentityFirst, kIdentityBytes, &row.anomalies);
    if (identity) {
      row.identity = true;
      row.model = word_at(identity.data, at(kRegModelL));
      row.id_register = identity.data[at(kRegId)];
      row.baud_register = identity.data[at(kRegBaud)];
      row.offset_raw = word_at(identity.data, at(kRegOffsetL));
      row.mode = identity.data[at(SMS_STS_MODE)];
      if (row.id_register != id) {
        row.anomalies.push_back("its id register reads " + std::to_string(row.id_register));
      }
      const int baud = baud_of_register(row.baud_register);
      if (baud != bus.baudrate()) {
        row.anomalies.push_back(
          "its baud register (" + std::to_string(row.baud_register) + " = " +
          (baud > 0 ? std::to_string(baud) : std::string("?")) +
          ") disagrees with the bus rate " + std::to_string(bus.baudrate()));
      }
    } else {
      row.anomalies.push_back(
        std::string("answered a ping but its identity block could not be read (") +
        to_string(identity.kind) + ")");
    }
    const Reply feedback = read_with_retry(
      bus, address, ServoBus::feedback_first_register,
      static_cast<uint8_t>(ServoBus::feedback_block_bytes), &row.anomalies);
    if (feedback) {
      const FeedbackBlock block = decode_feedback_block(feedback.data.data(), feedback.status);
      row.feedback = true;
      row.status = block.status;
      row.position = block.position_ticks;
      row.voltage_raw = block.voltage_raw;
      row.temperature = block.temperature_raw;
    } else {
      row.anomalies.push_back(
        std::string("answered a ping but its feedback block could not be read (") +
        to_string(feedback.kind) + ")");
    }

    // Step 5: notes, which change nothing about the exit.
    if (id == 0) {
      row.notes.push_back(
        "the hardware interface accepts ids 1..253; give this servo another id with set_id "
        "before putting it in a URDF");
    }
    if (row.identity && row.mode >= 2) {
      row.notes.push_back("mode " + std::to_string(row.mode) + " has no driver support");
    }
    // Raw, because the driver's bit names and memory-table row 57 disagree on bits 1 and 4.
    if (row.status > 0) {
      row.notes.push_back("status " + hex_byte(row.status));
    }
    result.rows.push_back(row);
  }
  result.seconds = std::chrono::duration<double>(Clock::now() - started).count();
  return result;
}

void print_scan(
  const ScanResult & result, const std::string & port, int baudrate, std::ostream & out,
  std::ostream & err)
{
  if (result.not_open) {
    return;
  }
  out << " id  type  mode  model  baud_reg     baud  position  voltage_V  temp_C  status  offset\n";
  for (const ScanRow & row : result.rows) {
    out << scan_row_text(row) << "\n";
  }

  std::string ids;
  for (const ScanRow & row : result.rows) {
    ids += " " + std::to_string(row.id);
  }
  char seconds[32];
  std::snprintf(seconds, sizeof(seconds), "%.1f", result.seconds);
  const int covered = result.interrupted ? result.last_pinged : result.last;
  const std::string pinged = covered >= result.first ?
    "pinged ids " + std::to_string(result.first) + ".." + std::to_string(covered) :
    std::string("pinged no id");
  const std::string tally = "(" + pinged + ", " + std::to_string(result.attempts) +
    " attempts each, " + seconds + " s)";
  const std::string where = " on " + port + " at " + std::to_string(baudrate) + " baud";
  if (result.rows.empty()) {
    out << "found no servo" << where << " " << tally;
  } else {
    out << "found " << result.rows.size() << " servo(s)" << where << ": ids" << ids << " " <<
      tally;
  }
  if (result.interrupted && result.last_pinged >= result.first) {
    out << ", interrupted after id " << result.last_pinged;
  } else if (result.interrupted) {
    out << ", interrupted before id " << result.first;
  }
  out << "\n";

  for (const ScanRow & row : result.rows) {
    for (const std::string & anomaly : row.anomalies) {
      err << "id " << row.id << ": " << anomaly << "\n";
    }
    for (const std::string & note : row.notes) {
      err << "id " << row.id << ": " << note << "\n";
    }
  }
  if (result.rows.empty() && !result.interrupted) {
    err << "check servo power (USB does not power the servos), the wiring, and the baud rate: a "
      "servo set to another rate answers only at that rate\n";
  }
  if (!result.rows.empty()) {
    err << "use the id column as <param name=\"id\"> and the type column as <param "
      "name=\"type\">; see description/ros2_control/example.ros2_control.xacro\n";
  }
}

Exit scan_exit(const ScanResult & result) noexcept
{
  if (result.not_open) {
    return Exit::kCannotOpen;
  }
  if (result.interrupted) {
    return Exit::kInterrupted;
  }
  for (const ScanRow & row : result.rows) {
    if (!row.anomalies.empty()) {
      return Exit::kScanAnomaly;
    }
  }
  return result.rows.empty() ? Exit::kNoAnswer : Exit::kOk;
}

SetIdReport set_id(Session & session, uint8_t start_id, uint8_t new_id)
{
  SetIdReport report;
  report.start_id = start_id;
  report.new_id = new_id;
  if (!session.bus.is_open()) {
    report.exit = Exit::kCannotOpen;
    return report;
  }
  ServoBus & bus = session.bus;
  const std::string s = std::to_string(start_id);
  const std::string n = std::to_string(new_id);
  const int attempts = std::max(1, session.attempts);
  const auto refuse = [&report, &session](Exit exit, const std::string & message) {
      session.err << message << "\n";
      report.exit = exit;
      return report;
    };
  if (stop_requested(session)) {
    return refuse(Exit::kInterrupted, "interrupted; nothing was written");
  }

  // Step 1, before anything is sent to S: N answers nothing, on EVERY attempt. This order is what
  // lets the bench's refusal cases address a silent start id safely (R11, E.4).
  for (int attempt = 0; attempt < attempts; attempt++) {
    if (bus.checked_ping(new_id).kind != ReplyKind::SILENT) {
      return refuse(
        Exit::kRefused, "id " + n + " already answers on '" + bus.port() + "'; refusing to give "
        "servo " + s + " an id that is taken -- two servos on one id answer on top of each other "
        "and cannot be told apart. Pick a free id; scan lists the taken ones. Nothing was "
        "written.");
    }
  }

  // Step 2: one clean answer from S, and every reply before it silent. A doubled, garbled or
  // misaddressed reply refuses even when a later ping comes back clean: two servos that collided
  // once and then happened to answer in step are still two servos (R10; review fix F5).
  const std::string twins = "more than one servo may answer at id " + s + " (factory-new servos "
    "all start at id 1); connect only the servo to renumber. Nothing was written.";
  bool answered = false;
  bool unclean = false;
  int pings = 0;
  for (int attempt = 0; attempt < attempts && !answered; attempt++) {
    const Reply reply = bus.checked_ping(start_id);
    pings++;
    if (reply.kind == ReplyKind::EXTRA) {
      return refuse(Exit::kRefused, twins);
    }
    answered = static_cast<bool>(reply);
    unclean = unclean || (!answered && reply.kind != ReplyKind::SILENT);
  }
  if (unclean) {
    return refuse(Exit::kRefused, twins);
  }
  if (!answered) {
    return refuse(
      Exit::kNoAnswer, "no servo answers at id " + s + " on '" + bus.port() + "' at " +
      std::to_string(bus.baudrate()) + " baud (" + std::to_string(pings) + " pings); nothing "
      "was written. scan lists the ids that do answer.");
  }

  // Step 3: its registers, read from the id that answered and checked against it.
  const Reply before = bus.checked_read(start_id, kIdentityFirst, kIdentityBytes);
  const Reply lock_before = before ? bus.checked_read(start_id, kRegLock, 1) : before;
  for (const Reply & reply : {before, lock_before}) {
    if (reply.kind == ReplyKind::GARBLED || reply.kind == ReplyKind::EXTRA) {
      return refuse(Exit::kRefused, twins);
    }
    if (!reply) {
      return refuse(
        Exit::kRefused, "id " + s + " answers pings but its registers cannot be read (" +
        to_string(reply.kind) + ")");
    }
  }
  report.lock_before = lock_before.data[0];
  if (before.data[at(kRegId)] != start_id) {
    return refuse(
      Exit::kRefused, "the servo answering at id " + s + " has " +
      std::to_string(before.data[at(kRegId)]) + " in its id register; refusing to write to a "
      "servo in an inconsistent state");
  }

  if (stop_requested(session)) {     // the last check: after this, a signal waits for the end
    return refuse(Exit::kInterrupted, "interrupted; nothing was written");
  }

  // Step 4: the sequence. From here every path ends in finish().
  Sequence sequence(session);
  sequence.begin(
    "about to give servo " + s + " the id " + n + ". If this run is interrupted or fails, run "
    "scan: the servo will answer at " + s + " or " + n + ".");
  Wire wire(bus, &report.writes_sent, &report.late_ack_from, &report.late_ack_ms);
  const auto finish = [&report, &session, &sequence](Exit exit, const std::string & message) {
      return conclude(report, session, sequence, exit, message);
    };
  // A stop that came after the check above -- while the notice was written -- is still acted on:
  // nothing has been written yet (review fix F7).
  if (sequence.stop_pending()) {
    return finish(Exit::kInterrupted, "interrupted; nothing was written");
  }

  const Reply unlocked = wire.set_lock(start_id, 0);
  report.unlock_read = unlocked ? unlocked.data[0] : -1;
  if (report.unlock_read != 0) {
    // best effort: put the lock back as it most likely was, and say what it reads (F1)
    const Reply relocked = wire.set_lock(start_id, 1);
    report.lock_after = relocked ? relocked.data[0] : -1;
    return finish(
      Exit::kNotApplied, "could not open the EEPROM write lock of id " + s + " (register 55 "
      "reads " + byte_read(unlocked) + " after writing 0); no EEPROM byte was changed" +
      relock_clause(relocked));
  }

  // Step 5: the id. Its ack is recorded, never believed.
  wire.arm(start_id, new_id, "id write", &sequence.err());
  const Reply ack = wire.write(start_id, kRegId, new_id, kEepromAckMs);
  if (ack.kind == ReplyKind::SILENT) {
    report.id_write_ack = "none";
  } else if (ack.kind == ReplyKind::GARBLED) {
    report.id_write_ack = "garbled";  // no ack_ms: its elapsed time may be the window's end (F28)
  } else {
    report.id_write_ack = ack.from_id == start_id ? "old_id" :
      (ack.from_id == new_id ? "new_id" : "other");
    report.ack_ms = static_cast<int>(ack.elapsed_us / 1000);
  }

  // Step 6: poll N until it answers or the window closes, then S on every attempt.
  std::vector<std::string> anomalies;
  const Clock::time_point poll_started = Clock::now();
  bool n_answers = false;
  report.new_id_pings = 0;
  while (true) {
    const Reply reply = bus.checked_ping(new_id);
    report.new_id_pings++;
    if (reply) {
      n_answers = true;
      break;
    }
    if (wire.tolerated(reply, "ping", new_id)) {
      continue;
    }
    if (reply.kind != ReplyKind::SILENT) {
      anomalies.push_back(reply_text(reply, "ping", new_id));
      break;
    }
    if (elapsed_ms(poll_started) >= static_cast<int>(kVerifyWindowMs)) {
      break;
    }
  }
  report.verify_ms = elapsed_ms(wire.committed());
  int s_answers = 0;
  if (anomalies.empty()) {
    int silent = 0;
    for (int attempt = 0; attempt < attempts; ) {
      const Reply reply = bus.checked_ping(start_id);
      if (wire.tolerated(reply, "ping", start_id)) {
        continue;                   // repeated, not an attempt
      }
      attempt++;
      if (reply.kind == ReplyKind::SILENT) {
        silent++;
      } else if (reply) {
        s_answers++;
      } else {
        anomalies.push_back(reply_text(reply, "ping", start_id));
      }
    }
    report.old_id_silent = silent == attempts;
  }

  // The outcome matrix of step 6. No row but the first writes anything further, apart from the
  // relock of the second.
  if (!anomalies.empty()) {
    std::string kinds;
    for (const std::string & anomaly : anomalies) {
      kinds += (kinds.empty() ? "" : "; ") + anomaly;
    }
    return finish(
      Exit::kInconsistent, "after writing id " + n + " to the servo at " + s + " the replies "
      "were not clean (" + kinds + "), so where it answers now is unknown. Run scan.");
  }
  if (!n_answers && s_answers > 0) {
    const Reply lock = wire.set_lock(start_id, 1);
    const Reply again = wire.read(start_id, kIdentityFirst, kIdentityBytes);
    report.lock_after = lock ? lock.data[0] : -1;
    report.identity_same = again && again.data == before.data;
    if (report.lock_after == 1 && *report.identity_same) {
      return finish(
        Exit::kNotApplied, "the id write did not take: the servo still answers at " + s +
        " and not at " + n + " (ack: " + report.id_write_ack + "); its EEPROM lock is closed "
        "again and nothing changed.");
    }
    return finish(
      Exit::kInconsistent, "the id write did not take: the servo still answers at " + s +
      " and not at " + n + ", but " + (report.lock_after != 1 ?
      "its EEPROM lock did not close again (register 55 reads " + byte_read(lock) + ")" :
      std::string("its registers no longer read as before")) + "; run scan");
  }
  if (!n_answers) {
    return finish(
      Exit::kInconsistent, "after writing id " + n + " to the servo at " + s + ", neither id "
      "answers. Run scan (it covers ids 0..253). If it answers nowhere, power-cycle it and scan "
      "again. Once scan finds it, factory_reset can put it back to its factory settings; the "
      "reset keeps whatever id it answers at, so it cannot find a lost servo by itself.");
  }
  if (s_answers > 0) {
    return finish(
      Exit::kInconsistent, "both " + s + " and " + n + " answer after the id write; a second "
      "servo was on id " + s + " or " + n + ". Run scan.");
  }

  // Step 7: the servo at N is the servo that was at S, with only its id changed. On a failure
  // exactly one servo answers at N and none at S, so the lock this run opened is closed there
  // before the exit, and the message says whether it closed (review fix F9).
  const auto relocked_at_n = [&wire, &report, new_id]() {
      const Reply locked = wire.set_lock(new_id, 1);
      report.lock_after = locked ? locked.data[0] : -1;
      return report.lock_after == 1 ? std::string("; its EEPROM lock is closed again") :
             "; its EEPROM lock is still open (register 55 reads " + byte_read(locked) +
             ") until it is power-cycled";
    };
  const Reply after = wire.read(new_id, kIdentityFirst, kIdentityBytes);
  if (!after) {
    const std::string lock = relocked_at_n();
    return finish(
      Exit::kInconsistent, "the servo now answers at id " + n + ", but its registers cannot be "
      "read (" + std::string(to_string(after.kind)) + ")" + lock + "; run scan");
  }
  std::string changed;
  for (std::size_t i = 0; i < kIdentityBytes; i++) {
    const int reg = static_cast<int>(kIdentityFirst + i);
    const int expected = reg == kRegId ? new_id : before.data[i];
    if (after.data[i] != expected) {
      changed += (changed.empty() ? "" : ", ") + std::string("register ") + std::to_string(reg) +
        " changed from " + std::to_string(before.data[i]) + " to " + std::to_string(after.data[i]);
    }
  }
  report.identity_same = changed.empty();
  if (!changed.empty()) {
    const std::string lock = relocked_at_n();
    return finish(
      Exit::kInconsistent, "the servo now answers at id " + n + ", but " + changed + lock +
      "; run scan");
  }

  // Step 8: the verified lock, left closed.
  const Reply locked = wire.set_lock(new_id, 1);
  report.lock_after = locked ? locked.data[0] : -1;
  if (report.lock_after != 1) {
    return finish(
      Exit::kInconsistent, "servo " + s + " is now id " + n + " and answers there, but its "
      "EEPROM lock did not close (register 55 reads " + byte_read(locked) + "); later EEPROM "
      "writes to it are not protected until it is power-cycled");
  }

  // Step 9.
  return finish(
    Exit::kOk, "servo " + s + " is now id " + n + ": it answers at " + n + " and no longer at " +
    s + ", its registers are otherwise unchanged, and its EEPROM lock is closed. The id is stored "
    "in EEPROM; power-cycle the servo and run scan to confirm it kept id " + n + ", then update "
    "<param name=\"id\"> in your URDF.");
}

CalibrateReport calibrate_midpoint(Session & session, uint8_t id)
{
  CalibrateReport report;
  report.id = id;
  if (!session.bus.is_open()) {
    report.exit = Exit::kCannotOpen;
    return report;
  }
  ServoBus & bus = session.bus;
  const std::string i = std::to_string(id);
  const int attempts = std::max(1, session.attempts);
  const auto refuse = [&report, &session](Exit exit, const std::string & message) {
      session.err << message << "\n";
      report.exit = exit;
      return report;
    };
  if (stop_requested(session)) {
    return refuse(Exit::kInterrupted, "interrupted; nothing was written");
  }

  // Step 1: one clean answer, and every reply before it silent -- as set_id's step 2 (R10; review
  // fix F5): a collision seen once is twin evidence, whatever a later ping says.
  const std::string twins = "more than one servo may answer at id " + i + "; connect only the "
    "servo to calibrate. Nothing was written.";
  bool answered = false;
  bool unclean = false;
  int pings = 0;
  for (int attempt = 0; attempt < attempts && !answered; attempt++) {
    const Reply reply = bus.checked_ping(id);
    pings++;
    if (reply.kind == ReplyKind::EXTRA) {
      return refuse(Exit::kRefused, twins);
    }
    answered = static_cast<bool>(reply);
    unclean = unclean || (!answered && reply.kind != ReplyKind::SILENT);
  }
  if (unclean) {
    return refuse(Exit::kRefused, twins);
  }
  if (!answered) {
    return refuse(
      Exit::kNoAnswer, "no servo answers at id " + i + " on '" + bus.port() + "' at " +
      std::to_string(bus.baudrate()) + " baud (" + std::to_string(pings) + " pings); nothing "
      "was written. scan lists the ids that do answer.");
  }

  // Step 2: identity, torque and lock, from the id that answered and checked against it.
  const Reply before = bus.checked_read(id, kIdentityFirst, kIdentityBytes);
  const Reply torque = before ? bus.checked_read(id, SMS_STS_TORQUE_ENABLE, 1) : before;
  const Reply lock = torque ? bus.checked_read(id, kRegLock, 1) : torque;
  for (const Reply & reply : {before, torque, lock}) {
    if (reply.kind == ReplyKind::GARBLED || reply.kind == ReplyKind::EXTRA) {
      return refuse(Exit::kRefused, twins);
    }
    if (!reply) {
      return refuse(
        Exit::kRefused, "id " + i + " answers pings but its registers cannot be read (" +
        to_string(reply.kind) + "); nothing was written");
    }
  }
  if (before.data[at(kRegId)] != id) {
    return refuse(
      Exit::kRefused, "the servo answering at id " + i + " has " +
      std::to_string(before.data[at(kRegId)]) + " in its id register; refusing to write to a "
      "servo in an inconsistent state");
  }
  report.mode = before.data[at(SMS_STS_MODE)];
  report.offset_raw_before = word_at(before.data, at(kRegOffsetL));
  report.torque_before = torque.data[0];

  // Step 3 [Q1]: a midpoint means something only to a position servo, and the mode is an EEPROM
  // write this tool does not make.
  if (report.mode != 0) {
    const char * kind = report.mode == 1 ? "wheel" : (report.mode == 2 ? "pwm" : "other");
    return refuse(
      Exit::kRefused, "servo " + i + " is in mode " + std::to_string(report.mode) + " (" + kind +
      "); a midpoint only means something to a position servo (mode 0). Changing the mode is an "
      "EEPROM write this tool does not make: declare the joint type 'pos' and let the hardware "
      "interface switch it at configure, then calibrate. Nothing was written.");
  }

  if (stop_requested(session)) {     // the last check: after this, a signal waits for the end
    return refuse(Exit::kInterrupted, "interrupted; nothing was written");
  }

  // Step 4 [Q2]: the sequence starts with the torque off. With torque on, the servo would drive to
  // its old goal in the new frame the moment the offset changed -- about 1022 ticks on the bench.
  Sequence sequence(session);
  sequence.begin(
    "about to calibrate the midpoint of servo " + i + " (an EEPROM write of its offset, registers "
    "31-32). If this run is interrupted or fails, run scan: the servo will answer at " + i + ".");
  Wire wire(bus, &report.writes_sent, &report.late_ack_from, &report.late_ack_ms);
  const auto finish = [&report, &session, &sequence](Exit exit, const std::string & message) {
      return conclude(report, session, sequence, exit, message);
    };
  std::string torque_clause;
  if (report.torque_before != 0) {
    wire.write(id, SMS_STS_TORQUE_ENABLE, 0, session.io_timeout_ms);
    report.torque_written = true;
    const Reply off = wire.read(id, SMS_STS_TORQUE_ENABLE, 1);
    if (!off || off.data[0] != 0) {
      // Read back as still on: nothing changed. Not read back at all: the write may have taken,
      // so the torque state is unknown, and the message may not claim otherwise (F8).
      return finish(
        Exit::kNotApplied, "could not switch the torque of servo " + i + " off (register 40 "
        "reads " + byte_read(off) + "); " + (off ? std::string("nothing was changed") :
        std::string("its torque state is unknown; no EEPROM byte was written")));
    }
    torque_clause = "; its torque is now OFF";
  }

  // Step 5 [Q2]: the position the verdict is measured against is the one it settles at with the
  // torque off: two reads 100 ms apart within 1 tick, sampled every 20 ms.
  constexpr int kSampleMs = 20;
  constexpr int kSettleSpanMs = 100;
  constexpr int kSettleTicks = 1;
  std::vector<std::pair<int, int>> samples;   // {ms since the first read, position}
  const Clock::time_point settle_started = Clock::now();
  while (true) {
    const Clock::time_point read_at = Clock::now();
    const Reply sample = wire.read(id, SMS_STS_PRESENT_POSITION_L, 2);
    if (!sample) {
      return finish(
        Exit::kRefused, "servo " + i + " answers but its position cannot be read (" +
        std::string(to_string(sample.kind)) + "). Its torque is now OFF; no EEPROM byte was "
        "written.");
    }
    const int now = elapsed_ms(settle_started);
    const int position = position_of(sample);
    samples.emplace_back(now, position);
    const std::pair<int, int> * earlier = nullptr;
    for (const std::pair<int, int> & older : samples) {
      if (older.first <= now - kSettleSpanMs) {
        earlier = &older;
      }
    }
    if (earlier != nullptr && std::abs(position - earlier->second) <= kSettleTicks) {
      report.position_before = position;
      report.settle_ms = now;
      break;
    }
    if (now >= static_cast<int>(kSettleMaxMs)) {
      const int from = earlier != nullptr ? earlier->second : samples.front().second;
      return finish(
        Exit::kRefused, "servo " + i + " is still moving (" + std::to_string(from) + " -> " +
        std::to_string(position) + "); hold it still at the intended midpoint and run again. Its "
        "torque is now OFF; no EEPROM byte was written.");
    }
    std::this_thread::sleep_until(read_at + std::chrono::milliseconds(kSampleMs));
  }

  // The last stop check (review fix F7). The torque write and the settle come before any EEPROM
  // write; R4's reason to hold a signal back -- never die between unlock and lock -- starts at
  // the unlock. So a Ctrl-C while the arm settles stops the run here, with the offset untouched.
  if (sequence.stop_pending()) {
    return finish(
      Exit::kInterrupted, torque_clause.empty() ? std::string("interrupted; nothing was written") :
      "interrupted" + torque_clause + "; no EEPROM byte was written");
  }

  // Step 6 [Q3]: the verified unlock, so the offset outlives a power cycle.
  const Reply unlocked = wire.set_lock(id, 0);
  report.unlock_read = unlocked ? unlocked.data[0] : -1;
  if (report.unlock_read != 0) {
    // best effort: put the lock back as it most likely was, and say what it reads (F1)
    const Reply relocked = wire.set_lock(id, 1);
    report.lock_after = relocked ? relocked.data[0] : -1;
    return finish(
      Exit::kNotApplied, "could not open the EEPROM write lock of id " + i + " (register 55 "
      "reads " + byte_read(unlocked) + " after writing 0); no EEPROM byte was changed" +
      relock_clause(relocked) + torque_clause);
  }

  // Step 7: the byte SMS_STS::CalibrationOfs writes, through the id-checked primitive. From here
  // one late ack from I is tolerated.
  wire.arm(id, id, "calibration write", &sequence.err());
  const Reply ack = wire.write(id, SMS_STS_TORQUE_ENABLE, 128, kEepromAckMs);
  if (ack.kind == ReplyKind::SILENT) {
    report.calibrate_ack = "none";
  } else if (ack.kind == ReplyKind::GARBLED) {
    report.calibrate_ack = "garbled";
  } else {
    report.calibrate_ack = ack.from_id == id ? "old_id" : "other";
    report.ack_ms = static_cast<int>(ack.elapsed_us / 1000);
  }

  // Step 8: poll the position until it reads the midpoint. A read the servo answered is paced at
  // 20 ms; a silent one has already waited its whole window, so the next goes out at once and a
  // late ack is always heard by somebody.
  std::vector<std::string> anomalies;
  const Clock::time_point poll_started = Clock::now();
  while (true) {
    const Clock::time_point read_at = Clock::now();
    const Reply reply = bus.checked_read(id, SMS_STS_PRESENT_POSITION_L, 2);
    if (wire.tolerated(reply, "read", id)) {
      continue;
    }
    if (reply) {
      report.position_after = position_of(reply);
      if (std::abs(*report.position_after - kMidpointTicks) <= kMidpointTolTicks) {
        break;
      }
    } else if (reply.kind != ReplyKind::SILENT) {
      anomalies.push_back(reply_text(reply, "read", id));
      break;
    }
    if (elapsed_ms(poll_started) >= static_cast<int>(kVerifyWindowMs)) {
      break;
    }
    if (reply) {
      std::this_thread::sleep_until(read_at + std::chrono::milliseconds(kSampleMs));
    }
  }
  const Reply offset = wire.read(id, kRegOffsetL, 2);
  report.offset_raw_after = offset ? word_at(offset.data, 0) : -1;
  const Reply register40 = wire.read(id, SMS_STS_TORQUE_ENABLE, 1);
  report.register40_after = register40 ? register40.data[0] : -1;

  // Step 9 [Q2]: torque stays off, whatever the firmware did to register 40 after the 128.
  Reply off;
  if (report.register40_after != 0) {
    wire.write(id, SMS_STS_TORQUE_ENABLE, 0, session.io_timeout_ms);
    off = wire.read(id, SMS_STS_TORQUE_ENABLE, 1);
    report.torque_final = off ? off.data[0] : -1;
  } else {
    report.torque_final = 0;
  }

  // Step 10 [Q3]: the verified lock, and every other identity register as it was.
  const Reply locked = wire.set_lock(id, 1);
  report.lock_after = locked ? locked.data[0] : -1;
  const Reply after = wire.read(id, kIdentityFirst, kIdentityBytes);
  std::string changed;
  if (after) {
    for (std::size_t k = 0; k < kIdentityBytes; k++) {
      const int reg = static_cast<int>(kIdentityFirst + k);
      if (reg != kRegOffsetL && reg != kRegOffsetL + 1 && after.data[k] != before.data[k]) {
        changed += (changed.empty() ? "" : ", ") + std::string("register ") +
          std::to_string(reg) + " changed from " + std::to_string(before.data[k]) + " to " +
          std::to_string(after.data[k]);
      }
    }
    report.identity_same = changed.empty();
  }

  // What the verdict does not gate (G.1.2): the sign of the offset change against
  // position_before - 2048, recorded only. H16 measured the ST3025's convention as +1 (the offset
  // moves by position_before - 2048; README NOTE offset_sign), and the fake models the same.
  const int before_ofs = offset_from_raw(static_cast<uint16_t>(report.offset_raw_before));
  const uint16_t raw_after = static_cast<uint16_t>(std::max(report.offset_raw_after, 0));
  const int after_ofs = offset_from_raw(raw_after);
  const int moved = after_ofs - before_ofs;
  const int wanted = *report.position_before - kMidpointTicks;
  if (report.offset_raw_after >= 0 && moved != 0 && wanted != 0) {
    report.offset_sign = (moved > 0) == (wanted > 0) ? 1 : -1;
  }

  // Step 11: the verdict. Every exit but 0 and 5 means the servo's state is changed or unknown.
  const std::string lock_note = "register 55 reads " + byte_read(locked);
  if (!anomalies.empty()) {
    return finish(
      Exit::kInconsistent, "after the calibration write to servo " + i + " the replies were not "
      "clean (" + anomalies.front() + "); its offset is unknown. Run scan.");
  }
  if (report.torque_final != 0) {
    // In this branch step 9 wrote and read 40 (torque_final is 0 without it), so `off` is its
    // read-back: a value, or what came instead of one -- never a -1 dressed as a value (F23).
    return finish(
      Exit::kInconsistent, "the torque of servo " + i + (off ? " will not go off" :
      " cannot be confirmed off") + " after the calibration write (register 40 reads " +
      byte_read(off) + "); its offset is unknown. Run scan.");
  }
  if (report.lock_after != 1) {
    return finish(
      Exit::kInconsistent, "the EEPROM lock of servo " + i + " did not close after the "
      "calibration write (" + lock_note + "); later EEPROM writes to it are not protected until "
      "it is power-cycled. Run scan.");
  }
  if (!after) {
    return finish(
      Exit::kInconsistent, "after the calibration write the registers of servo " + i + " cannot "
      "be read (" + std::string(to_string(after.kind)) + "); run scan");
  }
  if (!changed.empty()) {
    return finish(
      Exit::kInconsistent, "the calibration write changed more than the offset of servo " + i +
      ": " + changed + "; run scan");
  }
  // Step 10's read also holds 31-32, later than step 8's. The verdict below rests on step 8's, so
  // the two must agree: an offset that moved in between (a commit landing late) makes both the
  // "unchanged" of exit 5 and the "went from" of exit 0 claims about a stale read (review fix F2).
  const int offset_final = word_at(after.data, at(kRegOffsetL));
  if (report.offset_raw_after >= 0 && offset_final != report.offset_raw_after) {
    return finish(
      Exit::kInconsistent, "the offset registers 31-32 of servo " + i + " read " +
      raw_word(report.offset_raw_after) + " after the calibration write and " +
      raw_word(offset_final) + " at the final check; its offset is unknown. Run scan.");
  }
  if (report.offset_raw_after < 0 || !report.position_after.has_value()) {
    return finish(
      Exit::kInconsistent, "the result of the calibration of servo " + i + " cannot be read "
      "(position " + (report.position_after ? std::to_string(*report.position_after) :
      std::string("unread")) + ", offset " + (report.offset_raw_after >= 0 ?
      std::to_string(after_ofs) : std::string("unread")) + "); run scan");
  }
  const bool centred = std::abs(*report.position_after - kMidpointTicks) <= kMidpointTolTicks;
  const bool was_centred = std::abs(wanted) <= kMidpointTolTicks;
  const bool offset_changed = report.offset_raw_after != report.offset_raw_before;
  const std::string offsets = std::to_string(before_ofs) + " to " + std::to_string(after_ofs) +
    " (raw " + raw_word(report.offset_raw_before) + " -> " + raw_word(report.offset_raw_after) +
    ")";
  if (!centred && !offset_changed) {
    // A.3 row 5: an SRAM change this run made is named -- here the torque step 4 switched off.
    return finish(
      Exit::kNotApplied, "the calibration did not take: position still reads " +
      std::to_string(*report.position_after) + " and the offset register is unchanged; any lock "
      "this run opened is closed again" + torque_clause + ".");
  }
  if (!centred || !(was_centred || offset_changed)) {
    return finish(
      Exit::kInconsistent, "the calibration of servo " + i + " did not end where it should: its "
      "position went from " + std::to_string(*report.position_before) + " to " +
      std::to_string(*report.position_after) + " and its offset registers 31-32 from " + offsets +
      "; run scan");
  }

  // Step 12.
  return finish(
    Exit::kOk, "servo " + i + " now reads " + std::to_string(*report.position_after) + " at the "
    "position that read " + std::to_string(*report.position_before) + "; offset registers 31-32 "
    "went from " + offsets + ". Its torque is OFF; the hardware interface turns it on at "
    "activate. EEPROM lock closed. Power-cycle the servo and run scan to confirm the offset "
    "survived.");
}

FactoryResetReport factory_reset(Session & session, uint8_t id)
{
  FactoryResetReport report;
  report.id = id;
  if (!session.bus.is_open()) {
    report.exit = Exit::kCannotOpen;
    return report;
  }
  ServoBus & bus = session.bus;
  const std::string i = std::to_string(id);
  const int attempts = std::max(1, session.attempts);
  const auto refuse = [&report, &session](Exit exit, const std::string & message) {
      session.err << message << "\n";
      report.exit = exit;
      return report;
    };
  if (stop_requested(session)) {
    return refuse(Exit::kInterrupted, "interrupted; nothing was written");
  }

  // Step 1: one clean answer, and every reply before it silent, as set_id and calibrate (review
  // fix F5): a RESET to an id two servos share resets both.
  const std::string twins = "more than one servo may answer at id " + i + "; connect only the "
    "servo to reset. Nothing was written.";
  bool answered = false;
  bool unclean = false;
  int pings = 0;
  for (int attempt = 0; attempt < attempts && !answered; attempt++) {
    const Reply reply = bus.checked_ping(id);
    pings++;
    if (reply.kind == ReplyKind::EXTRA) {
      return refuse(Exit::kRefused, twins);
    }
    answered = static_cast<bool>(reply);
    unclean = unclean || (!answered && reply.kind != ReplyKind::SILENT);
  }
  if (unclean) {
    return refuse(Exit::kRefused, twins);
  }
  if (!answered) {
    return refuse(
      Exit::kNoAnswer, "no servo answers at id " + i + " on '" + bus.port() + "' at " +
      std::to_string(bus.baudrate()) + " baud (" + std::to_string(pings) + " pings); nothing "
      "was written. scan lists the ids that do answer.");
  }

  // Step 2: identity, torque and lock, from the id that answered and checked against it. The
  // identity block is what the verdict compares the servo with afterwards.
  const Reply before = bus.checked_read(id, kIdentityFirst, kIdentityBytes);
  const Reply torque = before ? bus.checked_read(id, SMS_STS_TORQUE_ENABLE, 1) : before;
  const Reply lock = torque ? bus.checked_read(id, kRegLock, 1) : torque;
  for (const Reply & reply : {before, torque, lock}) {
    if (reply.kind == ReplyKind::GARBLED || reply.kind == ReplyKind::EXTRA) {
      return refuse(Exit::kRefused, twins);
    }
    if (!reply) {
      return refuse(
        Exit::kRefused, "id " + i + " answers pings but its registers cannot be read (" +
        to_string(reply.kind) + "); nothing was written");
    }
  }
  if (before.data[at(kRegId)] != id) {
    return refuse(
      Exit::kRefused, "the servo answering at id " + i + " has " +
      std::to_string(before.data[at(kRegId)]) + " in its id register; refusing to write to a "
      "servo in an inconsistent state");
  }
  report.model = word_at(before.data, at(kRegModelL));
  report.baud_register_before = before.data[at(kRegBaud)];
  report.offset_raw_before = word_at(before.data, at(kRegOffsetL));
  report.mode_before = before.data[at(SMS_STS_MODE)];
  report.torque_before = torque.data[0];
  report.lock_before = lock.data[0];

  if (stop_requested(session)) {     // the last check: after this, a signal waits for the end
    return refuse(Exit::kInterrupted, "interrupted; nothing was written");
  }

  // Step 4: the sequence. From here every path ends in finish().
  const int rate_before = bus.baudrate();
  Sequence sequence(session);
  sequence.begin(
    "about to reset servo " + i + " to its factory settings: every EEPROM register but its id -- "
    "baud rate, offset (the midpoint calibration), mode, angle limits and gains. If this run is "
    "interrupted or fails, run scan: the servo will answer at id " + i +
    (rate_before == kFactoryBaudrate ? std::string(".") : ", at " + std::to_string(rate_before) +
    " baud before the reset and at " + std::to_string(kFactoryBaudrate) + " baud after it."));
  Wire wire(bus, &report.writes_sent, &report.late_ack_from, &report.late_ack_ms);
  const auto finish = [&report, &session, &sequence](Exit exit, const std::string & message) {
      return conclude(report, session, sequence, exit, message);
    };
  if (sequence.stop_pending()) {
    return finish(Exit::kInterrupted, "interrupted; nothing was written");
  }

  // Step 5: the torque off, as calibrate's step 4. The ST3025's reset switches it off by itself
  // (M4), but in which order it rewrites the mode, the offset and the torque is unmeasured, and a
  // servo still holding a position when its offset or mode changes under it would lurch.
  std::string torque_clause;
  if (report.torque_before != 0) {
    wire.write(id, SMS_STS_TORQUE_ENABLE, 0, session.io_timeout_ms);
    report.torque_written = true;
    const Reply off = wire.read(id, SMS_STS_TORQUE_ENABLE, 1);
    if (!off || off.data[0] != 0) {
      return finish(
        Exit::kNotApplied, "could not switch the torque of servo " + i + " off (register 40 "
        "reads " + byte_read(off) + "); " + (off ? std::string("nothing was changed") :
        std::string("its torque state is unknown; the reset was not sent")));
    }
    torque_clause = "; its torque is now OFF";
  }

  // Step 6: the last stop check (review fix F7's rule): nothing so far is more than an SRAM write.
  if (sequence.stop_pending()) {
    return finish(
      Exit::kInterrupted, torque_clause.empty() ? std::string("interrupted; nothing was written") :
      "interrupted" + torque_clause + "; the reset was not sent");
  }

  // Step 7: the RESET. Its ack is recorded, never believed; from here one late ack is tolerated.
  wire.arm(id, id, "reset", &sequence.err());
  const Reply ack = wire.reset(id, kEepromAckMs);
  if (ack.kind == ReplyKind::SILENT) {
    report.reset_ack = "none";
  } else if (ack.kind == ReplyKind::GARBLED) {
    report.reset_ack = "garbled";
  } else {
    report.reset_ack = ack.from_id == id ? "old_id" : "other";
    report.ack_ms = static_cast<int>(ack.elapsed_us / 1000);
  }

  // Step 8: a reset servo talks at the factory rate (M3). The line follows it on the descriptor
  // the bus already holds, so the port is never let go.
  const auto elsewhere_hint = []() {
      return "Run scan with -p baudrate:=" + std::to_string(kFactoryBaudrate) + ", the rate a "
             "reset servo talks at; power-cycle it if it answers nowhere.";
    };
  if (rate_before != kFactoryBaudrate && !bus.set_baudrate(kFactoryBaudrate)) {
    return finish(
      Exit::kInconsistent, "the reset was sent to servo " + i + " at " +
      std::to_string(rate_before) + " baud, but the port could not be switched to " +
      std::to_string(kFactoryBaudrate) + " baud, so the result cannot be checked. " +
      elsewhere_hint());
  }

  // Step 9: poll a READ of the identity block. A READ sees a late ack as a bare status frame,
  // which a ping could not tell from its own reply.
  std::vector<std::string> anomalies;
  const auto read_back = [&bus, &wire, &anomalies, id]() {
      Reply reply;
      const Clock::time_point started = Clock::now();
      while (true) {
        reply = bus.checked_read(id, kIdentityFirst, kIdentityBytes);
        if (wire.tolerated(reply, "read", id)) {
          continue;
        }
        if (reply) {
          break;
        }
        if (reply.kind != ReplyKind::SILENT) {
          anomalies.push_back(reply_text(reply, "read", id));
          break;
        }
        if (elapsed_ms(started) >= static_cast<int>(kVerifyWindowMs)) {
          break;
        }
      }
      return reply;
    };
  Reply after = read_back();
  report.baudrate_after = bus.baudrate();
  std::string looked_at = std::to_string(bus.baudrate());
  // Silent at the factory rate is no proof that the reset happened: a firmware without RESET
  // still talks at the old rate. Look there before saying anything.
  if (!after && anomalies.empty() && rate_before != kFactoryBaudrate &&
    bus.set_baudrate(rate_before))
  {
    after = read_back();
    report.baudrate_after = bus.baudrate();
    looked_at += " and at " + std::to_string(rate_before);
  }
  report.verify_ms = elapsed_ms(wire.committed());

  // Step 10: torque off and the lock closed, each verified -- the ST3025's reset leaves both so
  // (M4, M5), and a firmware that does not is put there.
  std::string lock_clause;
  if (after) {
    Reply off = wire.read(id, SMS_STS_TORQUE_ENABLE, 1);
    if (!off || off.data[0] != 0) {
      wire.write(id, SMS_STS_TORQUE_ENABLE, 0, session.io_timeout_ms);
      off = wire.read(id, SMS_STS_TORQUE_ENABLE, 1);
    }
    report.torque_final = off ? off.data[0] : -1;
    Reply locked = wire.read(id, kRegLock, 1);
    if (!locked || locked.data[0] != 1) {
      locked = wire.set_lock(id, 1);
      lock_clause = report.lock_before != 1 ? "; its EEPROM lock, open before, is now closed" : "";
    }
    report.lock_after = locked ? locked.data[0] : -1;
  }

  // Step 11: the verdict.
  if (!anomalies.empty()) {
    return finish(
      Exit::kInconsistent, "after the reset of servo " + i + " the replies were not clean (" +
      anomalies.front() + "); its settings are unknown. Run scan.");
  }
  if (!after) {
    return finish(
      Exit::kInconsistent, "servo " + i + " does not answer at id " + i + " after the reset "
      "(looked for at " + looked_at + " baud for " + std::to_string(kVerifyWindowMs) + " ms); its "
      "settings are unknown. " + elsewhere_hint());
  }
  report.baud_register_after = after.data[at(kRegBaud)];
  report.offset_raw_after = word_at(after.data, at(kRegOffsetL));
  report.mode_after = after.data[at(SMS_STS_MODE)];
  std::string changed;
  report.changed_registers = 0;
  for (std::size_t k = 0; k < kIdentityBytes; k++) {
    if (after.data[k] != before.data[k]) {
      report.changed_registers++;
      changed += (changed.empty() ? "" : ", ") + std::to_string(kIdentityFirst + k) + ": " +
        std::to_string(before.data[k]) + " -> " + std::to_string(after.data[k]);
    }
  }
  const int model_after = word_at(after.data, at(kRegModelL));
  if (after.data[at(kRegId)] != id || model_after != report.model) {
    return finish(
      Exit::kInconsistent, "after the reset the servo answering at id " + i + " reads id "
      "register " + std::to_string(after.data[at(kRegId)]) + " and model " +
      std::to_string(model_after) + " (before: " + i + " and " + std::to_string(report.model) +
      "); run scan");
  }
  if (report.torque_final != 0) {
    return finish(
      Exit::kInconsistent, "the torque of servo " + i + " will not go off after the reset "
      "(register 40 reads " + (report.torque_final >= 0 ? std::to_string(report.torque_final) :
      std::string("nothing")) + "); run scan.");
  }
  if (report.lock_after != 1) {
    return finish(
      Exit::kInconsistent, "the EEPROM lock of servo " + i + " did not close after the reset "
      "(register 55 reads " + (report.lock_after >= 0 ? std::to_string(report.lock_after) :
      std::string("nothing")) + "); later EEPROM writes to it are not protected until it is "
      "power-cycled. Run scan.");
  }
  const bool factory = report.baud_register_after == 0 && report.offset_raw_after == 0 &&
    report.mode_after == 0;
  if (!factory && report.changed_registers == 0) {
    const std::string where = bus.baudrate() == kFactoryBaudrate ? std::string() :
      " still answers at " + std::to_string(bus.baudrate()) + " baud and";
    return finish(
      Exit::kNotApplied, "the reset did not take: servo " + i + where + " reads exactly as before "
      "in registers 3..39 (reset ack: " + report.reset_ack + "); its firmware may not know the "
      "RESET instruction" + torque_clause + lock_clause + ".");
  }
  if (!factory) {
    return finish(
      Exit::kInconsistent, "the reset of servo " + i + " did not reach the factory values: baud "
      "register " + std::to_string(report.baud_register_after) + ", offset raw " +
      raw_word(report.offset_raw_after) + " and mode " + std::to_string(report.mode_after) +
      " (factory: 0, 0x0000 and 0); registers changed: " + changed + ". Run scan.");
  }

  // Step 12.
  const std::string what_changed = changed.empty() ?
    std::string("No register in 3..39 changed: it was at them already. ") :
    "Registers changed: " + changed + ". ";
  std::string message = "servo " + i + " is reset to its factory settings and keeps id " + i +
    ". " + what_changed;
  if (rate_before != kFactoryBaudrate) {
    message += "It now talks at " + std::to_string(kFactoryBaudrate) + " baud, not " +
      std::to_string(rate_before) + ": give scan and the hardware interface baudrate " +
      std::to_string(kFactoryBaudrate) + " for it. ";
  }
  if (report.offset_raw_before != 0) {
    message += "Its offset (the midpoint calibration) is 0 now; run calibrate_midpoint if it needs "
      "one. ";
  }
  if (report.mode_before != 0) {
    message += "It is in mode 0 (position) now; a joint declared 'vel' is switched back at "
      "configure. ";
  }
  return finish(
    Exit::kOk, message + "Its torque is OFF and its EEPROM lock closed. Power-cycle the servo and "
    "run scan to confirm the settings survived.");
}

std::string detail_line(const FactoryResetReport & report)
{
  const auto number = [](int value) {
      return value >= 0 ? std::to_string(value) : std::string("none");
    };
  const auto raw = [](int value) {
      return value >= 0 ? raw_word(value) : std::string("none");
    };
  std::ostringstream line;
  line << "detail id=" << number(report.id) << " model=" << number(report.model) <<
    " baud_reg_before=" << number(report.baud_register_before) << " offset_raw_before=" <<
    raw(report.offset_raw_before) << " mode_before=" << number(report.mode_before) <<
    " torque_before=" << number(report.torque_before) << " torque_written=" <<
    (report.torque_written ? "true" : "false") << " lock_before=" << number(report.lock_before) <<
    " reset_ack=" << report.reset_ack << " ack_ms=" << number(report.ack_ms) <<
    " late_ack_from=" << number(report.late_ack_from) << " late_ack_ms=" <<
    number(report.late_ack_ms) << " baudrate_after=" << number(report.baudrate_after) <<
    " verify_ms=" << number(report.verify_ms) << " baud_reg_after=" <<
    number(report.baud_register_after) << " offset_raw_after=" << raw(report.offset_raw_after) <<
    " mode_after=" << number(report.mode_after) << " changed_registers=" <<
    number(report.changed_registers) << " torque_final=" << number(report.torque_final) <<
    " lock_after=" << number(report.lock_after) << " writes_sent=" << report.writes_sent <<
    " verdict=" << exit_name(report.exit);
  return line.str();
}

std::string detail_line(const SetIdReport & report)
{
  const auto number = [](int value) {
      return value >= 0 ? std::to_string(value) : std::string("none");
    };
  const auto flag = [](const std::optional<bool> & value) {
      return value.has_value() ? std::string(*value ? "true" : "false") : std::string("none");
    };
  std::ostringstream line;
  line << "detail start_id=" << number(report.start_id) << " new_id=" << number(report.new_id) <<
    " lock_before=" << number(report.lock_before) << " unlock_read=" <<
    number(report.unlock_read) << " id_write_ack=" << report.id_write_ack << " ack_ms=" <<
    number(report.ack_ms) << " verify_ms=" << number(report.verify_ms) << " new_id_pings=" <<
    number(report.new_id_pings) << " late_ack_from=" << number(report.late_ack_from) <<
    " late_ack_ms=" << number(report.late_ack_ms) << " old_id_silent=" <<
    flag(report.old_id_silent) << " identity_same=" << flag(report.identity_same) <<
    " lock_after=" << number(report.lock_after) << " writes_sent=" << report.writes_sent <<
    " verdict=" << exit_name(report.exit);
  return line.str();
}

std::string detail_line(const CalibrateReport & report)
{
  const auto number = [](int value) {
      return value >= 0 ? std::to_string(value) : std::string("none");
    };
  const auto position = [](const std::optional<int> & value) {
      return value.has_value() ? std::to_string(*value) : std::string("none");
    };
  const auto raw = [](int value) {
      return value >= 0 ? raw_word(value) : std::string("none");
    };
  std::ostringstream line;
  line << "detail id=" << number(report.id) << " mode=" << number(report.mode) <<
    " torque_before=" << number(report.torque_before) << " torque_written=" <<
    (report.torque_written ? "true" : "false") << " settle_ms=" << number(report.settle_ms) <<
    " position_before=" << position(report.position_before) << " offset_raw_before=" <<
    raw(report.offset_raw_before) << " unlock_read=" << number(report.unlock_read) <<
    " calibrate_ack=" << report.calibrate_ack << " ack_ms=" << number(report.ack_ms) <<
    " late_ack_from=" << number(report.late_ack_from) << " late_ack_ms=" <<
    number(report.late_ack_ms) << " position_after=" << position(report.position_after) <<
    " offset_raw_after=" << raw(report.offset_raw_after) << " offset_sign=" <<
    (report.offset_sign > 0 ? "+1" : (report.offset_sign < 0 ? "-1" : "0")) <<
    " register40_after=" << number(report.register40_after) << " torque_final=" <<
    number(report.torque_final) << " identity_same=" <<
    (report.identity_same.has_value() ? (*report.identity_same ? "true" : "false") : "none") <<
    " lock_after=" << number(report.lock_after) << " writes_sent=" << report.writes_sent <<
    " verdict=" << exit_name(report.exit);
  return line.str();
}

}  // namespace tools
}  // namespace waveshare_servos
