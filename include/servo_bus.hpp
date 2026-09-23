// ServoBus -- the package's only door to the serial bus.
//
// It wraps the vendored SMS_STS packet layer with the three things that layer does not do:
// exclusive access to the tty, a trustworthy reason when the port cannot be taken, and a close
// that really releases the device.
//
// Why a wrapper rather than an SMS_STS member (PHASE2_SPEC 9):
//   - SCSerial::begin() calls perror() on both of its failure paths, and glibc's perror does not
//     restore errno, so the errno the caller sees afterwards is not the one that caused the
//     failure. open() therefore takes its own descriptor first (step 4 below) and reports that
//     errno, which is the real one.
//   - SCSerial::setBaudRate()'s `default: break` leaves a speed_t uninitialised before
//     cfsetispeed/cfsetospeed (src/SCSerial.cpp:104,127-131). It is hidden, along with begin() and
//     end(), by the private using-declarations at the bottom of the class.
//   - The vendored constructors leave Err and the six syncRead* members indeterminate even though
//     ReadPos/ReadSpeed/ReadLoad/ReadCurrent read Err. The out-of-line constructor zeroes them.
//
// Exclusivity is two mechanisms, because neither alone is enough (PHASE2_SPEC 9.7): TIOCEXCL stops
// any unprivileged open of the device, and an advisory flock stops a second cooperating process
// even when it is root. Nothing protects against a process that already had the port when this
// one started; on_configure warns about those.
//
// This header deliberately includes "SMS_STS.h" rather than "SCServo.h": the driver uses no part
// of the SMSBL / SCSCL / SMSCL classes that "SCServo.h" also pulls in.

#ifndef SERVO_BUS_HPP_
#define SERVO_BUS_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "SMS_STS.h"

namespace waveshare_servos
{

// Why an open() did not take the port. Every value but OK names exactly one step of
// ServoBus::open(), so a caller can say what happened without guessing from errno.
enum class BusStatus : uint8_t
{
  OK = 0,
  ALREADY_OPEN,           // this bus already holds a port; the session was left untouched
  UNSUPPORTED_BAUDRATE,   // not one of the seven rates SCSerial::begin maps
  INVALID_TIMEOUT,        // 0 ms makes every transaction fail before the servo can answer
  OPEN_FAILED,            // the library's own open() failed; its reason went to stderr
  NOT_A_TTY,              // the path exists but is not a serial device
  LOCK_OPEN_FAILED,       // the lock descriptor could not be opened (this is where EBUSY lands)
  LOCK_FAILED,            // another process holds the advisory lock
  TERMIOS_FAILED,         // the line settings could not be applied
  EXCLUSIVE_FAILED        // the tty refused TIOCEXCL
};

// A short name for a status, for a log line or a test failure message.
const char * to_string(BusStatus status);

struct OpenResult
{
  BusStatus status = BusStatus::OK;
  // the errno of the failing step, or 0 when that step reports none
  int error = 0;
  explicit operator bool() const noexcept {return status == BusStatus::OK;}
};

// The pids that currently have this port open, this process included, from /proc. Diagnostics
// only: a snapshot, and descriptors of other users' processes are invisible. Empty when nothing
// holds the port and when /proc cannot be read. Never throws.
std::vector<int> port_holder_pids(const std::string & port) noexcept;

namespace detail
{
// PHASE3 1.12 solved for n: a sync-write frame is `overhead + n * (record_bytes + 1)` bytes
// (src/SCS.cpp:130-149) and must fit SCSerial::txBuf whole, because writeSCS appends with no
// bounds check at all (src/SCSerial.cpp:179-191).
//
// At namespace scope, and taking the buffer and the overhead as arguments, only because a class
// cannot initialise its own static constexpr data member from one of its own member functions --
// the body is not complete until the closing brace, so the call is "before its definition is
// complete". ServoBus::max_records_per_packet() forwards here, which is what lets the two shipped
// chunk limits be DERIVED from the arithmetic rather than hand-written beside it (PHASE3 R18).
constexpr std::size_t max_sync_write_records(
  std::size_t buffer_bytes, std::size_t overhead_bytes, std::size_t record_bytes) noexcept
{
  return (buffer_bytes - overhead_bytes) / (record_bytes + 1);
}
}  // namespace detail

// The servos' signed-value encoding: bit 15 is the direction flag and bits 0..14 the magnitude
// (src/SMS_STS.cpp:58-61 for positions, :265-268 for speeds). NOT two's complement -- goal -100
// goes out as `64 80`, not `9c ff` (PHASE3 1.10).
uint16_t sign_magnitude_encode(int16_t value) noexcept;

// One position servo's goal, exactly as it travels in a 7-byte sync-write record based at
// register 41 (SMS_STS_ACC .. SMS_STS_GOAL_SPEED_H, include/SMS_STS.h:41-47).
//
// Plain data with no invariant of its own: every field is already clamped by the driver before a
// record is built, and the wrapper's only check is the id (PHASE3 1.18). One array of records
// rather than the library's four parallel arrays is deliberate -- the four-pointer signature
// (src/SMS_STS.cpp:53) is what makes a length disagreement and a null ACC pointer
// (src/SMS_STS.cpp:270) expressible at all (PHASE3 1.4).
struct GoalPosition
{
  uint8_t id = 0;         // 1..253; 0xfe is the broadcast the frame itself carries (SCS.cpp:132)
  int16_t position = 0;   // goal, encoder ticks, SERVO frame, in [-32767, 32767]
  uint16_t speed = 0;     // goal-speed MAGNITUDE, ticks/s; 0 means "no speed limit", not "stop"
  uint8_t acc = 0;        // register 41; 0 is the documented "no ramp" opt-out, not "unset"
};

// One wheel's goal speed, exactly as it travels in a 2-byte sync-write record at register 46
// (SMS_STS_GOAL_SPEED_L, include/SMS_STS.h:46). No ACC field: on this path the acceleration is a
// separate one-off write to register 41, which is the whole point of Phase 3 item 1 (PHASE3 1.5).
struct GoalSpeed
{
  uint8_t id = 0;
  // Signed ticks/s. 0 is STOP here -- the opposite of GoalPosition::speed, where 0 means "no
  // limit" (src/waveshare_servos.cpp:1499-1506 explains why the position path floors at 1).
  int16_t speed = 0;
};

// Why a sync write could not be sent. Both failures are refusals: nothing reached the tty, so the
// servos keep the goals of the previous cycle -- which on the velocity path means a turning wheel
// KEEPS TURNING. A refusal is therefore never "harmless"; it is logged loudly (PHASE3 1.6).
enum class WriteStatus : uint8_t
{
  OK = 0,
  NOT_OPEN,     // the bus holds no port; nothing was written
  INVALID_ID    // a goal named an id outside 1..253; NOTHING was written, not even the good ones
};

// A short name for a status, for a log line or a test failure message. An overload of
// to_string(BusStatus), not a rename.
const char * to_string(WriteStatus status);

struct WriteResult
{
  WriteStatus status = WriteStatus::OK;
  // sync-write frames handed to the tty. 0 for an empty goal list, which is success (PHASE3 1.16)
  std::size_t packets = 0;
  // goal records inside them; equals goals.size() whenever status == OK
  std::size_t records = 0;
  // index into the caller's vector of the first offending goal, meaningful only on INVALID_ID
  std::size_t first_bad = 0;
  explicit operator bool() const noexcept {return status == WriteStatus::OK;}
};

// One servo's raw feedback block (registers 56..70) as it came off the wire, in exactly the units
// the SMS_STS ReadX(-1) accessors return -- ticks, raw counts, the servo's own status byte.
// Nothing here is scaled, signed by `inverted` or converted: units.hpp::decode() owns all of that,
// so the sync-read path and the per-servo FeedBack() path hand the driver the same struct and the
// published numbers are bit-identical by construction (PHASE3 2.5, 2.63).
//
// It lives here and not in units.hpp because test_servo_bus links servo_bus and nothing else
// (CMakeLists.txt:223-225): keeping the block self-contained is what lets the decoder be tested
// with no ros2_control and no link-line change (PHASE3 2.11).
struct FeedbackBlock
{
  // Every member is defaulted for the reason FeedbackSample's are (include/units.hpp:121): a
  // field left unset on some path must read 0, and no warning catches that one.
  bool valid = false;         // this servo answered with a frame that passed the gate (PHASE3 2.33)
  uint8_t status = 0;         // frame byte 4 -- what SCS::Read would have put in SCS::Error
  int position_ticks = 0;     // ReadPos(-1)      sign-magnitude on bit 15
  int speed_ticks = 0;        // ReadSpeed(-1)    sign-magnitude on bit 15
  int load_raw = 0;           // ReadLoad(-1)     sign-magnitude on bit 10
  int voltage_raw = 0;        // ReadVoltage(-1)  unsigned byte
  int temperature_raw = 0;    // ReadTemper(-1)   unsigned byte, degrees C
  int moving_raw = 0;         // ReadMove(-1)     unsigned flag; carried, unpublished (PHASE3 2.12)
  int current_counts = 0;     // ReadCurrent(-1)  sign-magnitude on bit 15
};

// Decode the 15-byte block at `data` (registers 56..70, little endian) exactly as
// SMS_STS::ReadPos/ReadSpeed/ReadLoad/ReadVoltage/ReadTemper/ReadMove/ReadCurrent(-1) would
// (src/SMS_STS.cpp:136-148,157-170,178-190,198,213,228,243-256), and stamp `status` and
// valid = true. `data` must point at 15 readable bytes (PHASE3 2.7).
FeedbackBlock decode_feedback_block(const uint8_t * data, uint8_t status) noexcept;

// Sync-read health for the life of the session. jazzy.md Phase 3 item 3 asks for the failure
// count "per servo"; that half stays in the driver (read_fails_, src/waveshare_servos.cpp:1374),
// because only the driver knows which joint a bus id belongs to. These are the bus-level totals
// the README's "how often a transaction fails" paragraph quotes alongside it (PHASE3 2.6, R16).
struct SyncReadStats
{
  uint64_t transactions = 0;    // sync_read_feedback() chunks that reached the wire
  uint64_t short_bursts = 0;    // readSCS() returned fewer bytes than the id list demanded
  uint64_t bad_frames = 0;      // frames rejected by the header / id / length / checksum gate
  // Requested ids with no DECODED frame, summed over chunks (PHASE3 2.35's `count - filled`). A
  // frame the gate rejected leaves its id unanswered too, so that id is counted here as well as
  // in bad_frames: the two are overlapping views of one burst, not a partition of it.
  uint64_t missing_frames = 0;
  uint64_t drains = 0;          // drain_input() runs, i.e. error-path recoveries (PHASE3 2.53)
};

// Walk one reply burst of `length` bytes and fill `out`, which is resized to `ids.size()` and
// written slot for slot: out[k] is ids[k]'s reply, and valid == false for an id that did not
// answer or whose frame failed the gate. Returns the number of slots filled. `stats` may be null;
// when it is not, bad_frames and missing_frames are ADDED to, so a chunked read accumulates.
//
// Frames arrive in REQUEST order -- measured on this bench with ascending, descending and two
// shuffled id lists (probe 1 Q5) -- so the walk matches each frame against the id list FORWARD
// ONLY. Ids skipped on the way are the servos that did not answer; a frame whose id is not ahead
// of the cursor is stale, foreign or a duplicate and is dropped rather than believed. That
// positional check is what caught the contamination the vendored receive side did not: 71
// wrong-id-in-slot frames in 3000 reads at a 1 ms timeout (probe 3 Q6).
//
// A free function, not a member, so the frame gate, the slot match and the over-read case are
// unit-testable with no bus and no port (PHASE3 R17). NOT noexcept: it resizes `out`.
std::size_t parse_sync_read_burst(
  const uint8_t * buffer, std::size_t length, const std::vector<uint8_t> & ids,
  std::vector<FeedbackBlock> * out, SyncReadStats * stats);

// What one checked transaction (ServoBus::checked_ping/checked_read/checked_write) saw on the wire.
// The vendored calls collapse all of this into one int: SCS::Read checks neither the responder
// id nor the length byte (src/SCS.cpp:173-203), SCS::Ack rejects an ack from any id but the
// addressed one (src/SCS.cpp:279), and none of them notices a second reply behind the first --
// which is what two servos on one id look like (PHASE6_SPEC B.4, R5, R10).
enum class ReplyKind : uint8_t
{
  NOT_OPEN,       // closed bus; nothing sent
  INVALID_ID,     // id 0xfe or 0xff; nothing sent
  INVALID_COUNT,  // a count outside 1..checked_max_bytes, or no data to write; nothing sent
  SILENT,         // 0 bytes in the reply window
  ONE,            // exactly one well-formed frame from the addressed id, nothing after it
  EXTRA,          // one well-formed frame, then more bytes within the drain window (a doubled twin)
  WRONG_ID,       // a well-formed frame of the expected length from another id (ping/read only)
  STATUS_ONLY,    // read only: the first well-formed frame is a bare 6-byte status frame (length
                  //   byte 2, no payload) where a READ reply was expected -- typically a late ack
  GARBLED         // bytes arrived but no well-formed frame (collision, noise, bad length or
                  //   checksum)
};

// A short snake_case name for a kind, for a message or a key=value detail line: no spaces, so a
// detail line stays one token per field.
const char * to_string(ReplyKind kind);

struct Reply
{
  ReplyKind kind = ReplyKind::NOT_OPEN;
  int from_id = -1;             // id byte of the well-formed frame, if any
  uint8_t status = 0;           // its error byte
  std::vector<uint8_t> data;    // READ payload (count bytes) when kind == ONE or EXTRA
  std::size_t frame_bytes = 0;  // length of that frame (6 for a ping reply or a bare status frame)
  std::size_t bytes = 0;        // bytes received in the reply window
  std::size_t extra_bytes = 0;  // bytes drained after the frame
  uint32_t elapsed_us = 0;      // request written -> frame complete (or window end)
  explicit operator bool() const noexcept {return kind == ReplyKind::ONE;}
};

class ServoBus : public SMS_STS
{
public:
  // Defined out of line so no compiler-generated copy of either lands in a translation unit that
  // only sees the declaration.
  ServoBus();
  ~ServoBus();                                    // calls close(); non-virtual on purpose
  ServoBus(const ServoBus &) = delete;
  ServoBus & operator=(const ServoBus &) = delete;
  ServoBus(ServoBus &&) = delete;
  ServoBus & operator=(ServoBus &&) = delete;

  // The seven rates SCSerial::begin() maps (src/SCSerial.cpp:50-75). Anything else silently ends
  // up at 115200, which is why this is the single source of truth for both open() and the
  // on_init check of the `baudrate` hardware parameter.
  static bool is_supported_baudrate(int baudrate) noexcept;

  // Take the port exclusively. The argument order is the reverse of the library's
  // begin(baudRate, serialPort); this wrapper is the only place the flip happens. io_timeout_ms is
  // an argument rather than a setter because the timeout has to be in force before the first
  // transaction.
  //
  // Every failure path leaves the bus closed, both descriptors released and no TIOCEXCL set. On
  // the EXCLUSIVE_FAILED path begin() has already written the line settings and the vendored
  // library never restores the termios it saved (src/SCSerial.cpp:47,221-228), so the tty is left
  // configured for 8N1 at the requested rate rather than as it was found.
  OpenResult open(const std::string & port, int baudrate, uint32_t io_timeout_ms);
  // Clears TIOCEXCL, closes the library's descriptor and releases the lock, each only if it holds
  // it. Idempotent, and safe before any open() and after a failed one.
  void close() noexcept;

  bool is_open() const noexcept {return fd != -1;}
  const std::string & port() const noexcept {return port_;}
  int baudrate() const noexcept {return baudrate_;}
  uint32_t io_timeout_ms() const noexcept {return static_cast<uint32_t>(IOTimeOut);}
  // Refuses 0 and changes nothing: a zero timeout makes every transaction fail instantly.
  bool set_io_timeout_ms(uint32_t ms) noexcept;

  // The smallest io_timeout_ms that can carry a sync read of `servos` ids, from the measured cost
  // model t(n) = 0.476 + 0.290n ms (probe 1 Q3, least squares over n = 1..4) scaled by the
  // measured p99/mean tail factor of 1.19 (probe 3 Q1: 2.058/1.734) and rounded up with 1 ms of
  // slack. Whole milliseconds because /dev/ttyACM0 is USB CDC and the host polls it once per 1 ms
  // USB frame, so sub-millisecond tuning is meaningless (probe 3 bonus). n = 4 gives 3 ms, the
  // lowest value probe 3 Q2 found defensible; n = 12 gives 6 ms (PHASE3 R9, 5.20b).
  //
  // Advisory, not a bound the bus enforces: the argument is the CHUNK size, and the driver reads
  // fewer servos than it declares as soon as one is dropped, so this is an upper estimate of the
  // need. Integer arithmetic so it is usable in a constexpr context and cannot drift with the
  // libm rounding mode a ceil() of a double would answer to.
  static constexpr uint32_t min_io_timeout_ms(std::size_t servos) noexcept
  {
    return (119u * (476u + 290u * static_cast<uint32_t>(servos)) + 99999u) / 100000u + 1u;
  }

  // The vendored transmit buffer (SCSerial::txBuf); the constructor static_asserts that this is
  // still its size, as a tripwire against an upstream refresh.
  static constexpr std::size_t tx_buffer_bytes = 255;

  // The sync-write frame is 7 header bytes + one id and nLen record bytes per servo + 1 checksum
  // (src/SCS.cpp:130-149), and SCSerial::writeSCS appends to txBuf[255] with no bounds check at
  // all (src/SCSerial.cpp:179-191), so these constants are the only thing standing between a large
  // robot and an out-of-bounds write. src/servo_bus.cpp writes the arithmetic out in four
  // static_asserts (PHASE3 1.12-1.13).
  static constexpr std::size_t sync_write_overhead_bytes = 8;
  static constexpr std::size_t goal_position_record_bytes = 7;
  static constexpr std::size_t goal_speed_record_bytes = 2;

  // The two chunk limits, 30 and 82, derived rather than written down (PHASE3 R18): there is one
  // place the inequality lives, so a change to txBuf or to a record width moves both limits by
  // itself. The static_asserts in src/servo_bus.cpp still check the result from the other side --
  // that the limit fits AND that limit + 1 does not -- because a derivation can be wrong too.
  static constexpr std::size_t max_records_per_packet(std::size_t record_bytes) noexcept
  {
    return detail::max_sync_write_records(
      tx_buffer_bytes, sync_write_overhead_bytes, record_bytes);
  }
  static constexpr std::size_t max_goal_positions_per_packet =
    detail::max_sync_write_records(
    tx_buffer_bytes, sync_write_overhead_bytes, goal_position_record_bytes);
  static constexpr std::size_t max_goal_speeds_per_packet =
    detail::max_sync_write_records(
    tx_buffer_bytes, sync_write_overhead_bytes, goal_speed_record_bytes);

  // The record bytes themselves, as pure functions so a test can pin them with no port at all
  // (PHASE3 4 section 1.2, kept by R1 on top of the record-vector API below). They are what the
  // chunk loop writes into its scratch, so there is exactly one implementation of the layout.
  static std::array<uint8_t, goal_position_record_bytes> position_record(
    uint8_t acc, int16_t goal_ticks, uint16_t goal_speed) noexcept;
  static std::array<uint8_t, goal_speed_record_bytes> speed_record(int16_t speed_ticks) noexcept;

  // Send one INST_SYNC_WRITE of 7-byte records at register 41, chunked at
  // max_goal_positions_per_packet. The frame is built by the vendored SCS::syncWrite
  // (src/SCS.cpp:124-151); only the record bytes are ours, and they are byte-identical to
  // SMS_STS::SyncWritePosEx's for every input the driver can produce (PHASE3 1.3, 1.7). `goals` is
  // not modified -- unlike the library, which rewrites the caller's Position[] in place
  // (src/SMS_STS.cpp:58-61).
  WriteResult write_goal_positions(const std::vector<GoalPosition> & goals);

  // Send one INST_SYNC_WRITE of 2-byte records at register 46, chunked at
  // max_goal_speeds_per_packet. No per-servo ACC transaction: register 41 is written on its own
  // schedule (PHASE3 1.21-1.24). Byte-identical to the sync stage of SMS_STS::SyncWriteSpe.
  WriteResult write_goal_speeds(const std::vector<GoalSpeed> & goals);

  // Write one servo's acceleration register (41) on its own, and report whether it was acked.
  // This is the transaction that replaces the per-servo ACC write SyncWriteSpe used to do inside
  // every cycle (PHASE3 1.20-1.24). It is an addressed write, so it costs one round trip and a
  // silent servo costs exactly one io_timeout_ms.
  //
  // Beware of two things it does NOT promise: SCS::Ack returns 1 whatever status byte the servo
  // replied with, so a latched fault still reads as success; and Ack overwrites SCS::Error
  // (src/SCS.cpp:267), so a caller that wants this cycle's feedback status must already have
  // captured it.
  bool write_acc(uint8_t id, uint8_t acc);

  // Size the packet-building scratch so the first write() of an activation allocates nothing.
  // Called with the two group sizes; over-sizing is free, under-sizing only costs one allocation
  // on the first oversized cycle. The clamp at the per-packet maxima is what makes a 100-servo
  // group reserve one chunk rather than a hundred records (PHASE3 1.7).
  void reserve_goal_capacity(std::size_t position_servos, std::size_t speed_servos);

  // Diagnostic only: the two scratch vectors' capacities. Exists so a test can prove the write
  // path stops allocating and so a log line can report it (PHASE3 1.7).
  std::size_t goal_scratch_capacity_bytes() const noexcept;

  // SMS_STS_PRESENT_POSITION_L .. SMS_STS_PRESENT_CURRENT_H (include/SMS_STS.h:51,61), the block
  // FeedBack() reads in one transaction (src/SMS_STS.cpp:123) and the block one sync-read frame
  // carries (PHASE3 2.9).
  static constexpr uint8_t feedback_first_register = 56;
  static constexpr std::size_t feedback_block_bytes = 15;
  // One reply frame is `FF FF id (nLen+2) status data[nLen] ~cks` (recon/packets.md 3.1).
  static constexpr std::size_t sync_read_frame_bytes = feedback_block_bytes + 6;    // 21
  // Ids per request, so one burst fits one buffer and one io_timeout_ms (PHASE3 2.24).
  static constexpr std::size_t sync_read_max_ids = 30;
  // Slack past the last frame. The vendored syncReadPacketRx reads up to nLen + 3 bytes beyond
  // syncReadRxBuffLen when its resync loop runs off the end (src/SCS.cpp:355, ASAN-confirmed,
  // recon/packets.md landmine 8). This wrapper's own walker cannot do that -- the `pos + 21 <=
  // len` guard of PHASE3 2.32 is checked before any byte of a frame is touched -- but the buffer
  // is still sized with the slack, because the vendored code remains on the link line.
  static constexpr std::size_t sync_read_slack_bytes = feedback_block_bytes + 3;    // 18
  static constexpr std::size_t sync_read_buffer_bytes =
    sync_read_max_ids * sync_read_frame_bytes + sync_read_slack_bytes;              // 648
  // Two USB frames, and the WHOLE drain budget rather than a per-iteration one (PHASE3 2.53-2.54).
  static constexpr uint32_t sync_read_drain_ms = 2;

  // One INST_SYNC_READ of the feedback block for every id in `ids`, in request order. `blocks` is
  // resized to ids.size() and filled slot for slot: blocks[k] is ids[k]'s reply, and valid is
  // false when that servo did not answer or its frame failed the gate. Returns the number of
  // valid blocks. Lists longer than sync_read_max_ids are split into chunks, each its own
  // transaction with its own io_timeout_ms (PHASE3 2.8, 2.19-2.30).
  //
  // Preconditions on `ids`, documented rather than re-checked per call because on_init already
  // enforces both and an RT path should not pay for it twice: every id is in 1..253
  // (src/waveshare_servos.cpp:355 and its three refusals at :356-377) and they are unique
  // (:378-387). A duplicate would break the forward-only slot match -- the second copy could
  // never be filled -- and 0xfe or 0xff would collide with the broadcast and header bytes
  // (src/SCS.cpp:130-132) (PHASE3 2.22).
  //
  // Not reentrant, for the same reason the write path is not: it owns one receive buffer and one
  // id scratch, and the vendored request builder opens with rFlushSCS() (src/SCS.cpp:299), which
  // would discard anything already in flight.
  std::size_t sync_read_feedback(
    const std::vector<uint8_t> & ids, std::vector<FeedbackBlock> & blocks);

  // The per-servo fallback: one addressed Read(id, 56, ., 15), which is byte for byte the
  // transaction SMS_STS::FeedBack(id) makes (src/SMS_STS.cpp:123, sizeof(Mem) == 15), decoded by
  // the same decoder the sync path uses. One decoder instead of two is the point: it removes the
  // Err-gated sign trap from BOTH paths, and with no ReadX(-1) call left anywhere in the package
  // SMS_STS::Mem going stale cannot hurt (PHASE3 R15).
  //
  // `out` is value-initialised on every path, so a refusal cannot leave last cycle's sample
  // behind. The status byte is taken from SCS::Error the instant Read() returns 15 and never
  // later: Read writes it only on success (src/SCS.cpp:201) and every later Ack, Ping or write
  // clobbers it.
  bool read_feedback_one(uint8_t id, FeedbackBlock & out);

  // Read and discard whatever turns up within `max_ms`. The error path only (PHASE3 2.53): a
  // tcflush -- which is all rFlushSCS does (src/SCSerial.cpp:193-196) -- can discard only bytes
  // that have ALREADY arrived, and the frame that poisons the next cycle is by definition one
  // that has not. Returns the bytes discarded; a no-op returning 0 on a closed bus.
  std::size_t drain_input(uint32_t max_ms = sync_read_drain_ms) noexcept;

  // Per session in the process sense, and deliberately NOT reset by close(): that is what the
  // README's "how often a transaction fails" number means (PHASE3 2.18).
  const SyncReadStats & sync_read_stats() const noexcept {return sync_read_stats_;}

  // The Phase 6 tools' transactions (PHASE6_SPEC B.4), built from the protected vendored helpers
  // and never used by the driver. Each one: refuse a closed bus, an unaddressable id or a count
  // outside 1..checked_max_bytes without sending; rFlushSCS(), the request, wFlushSCS(); ONE
  // readSCS of exactly the expected reply under its window; gate the header, the length byte,
  // the checksum and the id; and, only if a byte arrived, drain what follows so a second reply is
  // counted instead of left for the next transaction. A silent id therefore costs exactly one
  // window, which is what keeps a 254-id scan near 3.8 s.
  static constexpr std::size_t checked_max_bytes = 64;
  static constexpr uint32_t checked_drain_ms = 2;
  Reply checked_ping(uint8_t id);
  Reply checked_read(uint8_t id, uint8_t first, uint8_t count);
  // A frame from ANY id is accepted as the ack and its id returned in from_id -- an id write may be
  // acked from the new id, which SCS::Ack would call a failure (src/SCS.cpp:279). Callers never
  // treat an ack as proof; they verify by reading back. The window is ack_timeout_ms rather than
  // io_timeout_ms (an EEPROM commit takes longer), and it must stay below 1000 ms: readSCS puts
  // the whole of it into a timeval's tv_usec (src/SCSerial.cpp:147).
  Reply checked_write(
    uint8_t id, uint8_t first, const uint8_t * data, uint8_t count, uint32_t ack_timeout_ms);

  // RESET, "reset control table to factory value" (context/motor_reset_command_email.png, the
  // protocol manual's 1.3.7), which include/INST.h does not define. What it does on the ST3025 is
  // measured, not documented (factory_reset_evidence/FACTORY_RESET_SPEC.md 1): EEPROM 6..39 back
  // to the factory table with the id KEPT, the baud register to 0 (1 Mbaud), torque off, goal 0
  // and the lock closed; the ack comes from the addressed id at the OLD rate, 25 ms later when the
  // flash was rewritten.
  static constexpr uint8_t inst_reset = 0x06;
  // The checked transaction of a RESET, for factory_reset only: no parameters, so the frame is
  // FF FF id 02 06 ~sum; an ack from any other id is WRONG_ID. The window is ack_timeout_ms for
  // checked_write's reason, and must stay below 1000 ms for it too.
  Reply checked_reset(uint8_t id, uint32_t ack_timeout_ms);

  // Re-time the line of an open bus to another supported rate on the descriptor it already holds,
  // so the lock and TIOCEXCL are never let go: a close() and open() would leave a moment in which
  // another process could take the port. For factory_reset, which moves a servo to 1 Mbaud. Input
  // already received is discarded (at the old rate it is noise). False with nothing changed on a
  // closed bus or an unsupported rate; false with the line in an unknown state if the kernel
  // refuses the settings. The vendored SCSerial::setBaudRate cannot do this: it maps no 1000000,
  // leaving its speed_t uninitialised, and never calls tcsetattr (src/SCSerial.cpp:94-125).
  bool set_baudrate(int baudrate) noexcept;

private:
  void drop_lock() noexcept;

  // The one body behind the four checked calls: the refusals, the request, one readSCS of
  // payload_bytes + 6 under window_ms, the drain, and the frame gate. `params` is what writeBuf
  // sends after the address (nullptr for a ping and a RESET, the length for a READ, the bytes for
  // a WRITE).
  Reply checked_transaction(
    uint8_t id, uint8_t instruction, uint8_t first, uint8_t * params, uint8_t param_bytes,
    uint8_t payload_bytes, uint32_t window_ms, bool count_ok);

  // drain_input()'s body, uncounted: the checked calls drain after every reply, and counting
  // those in SyncReadStats::drains would let one scan swamp the control loop's error count.
  std::size_t discard_input(uint32_t max_ms) noexcept;

  // The one body behind write_goal_positions() and write_goal_speeds(). The two differ only in
  // the record they build, the base register and the chunk limit; the refusal ORDER (1.16-1.18),
  // the all-or-nothing id scan and the resize/copy/chunk arithmetic (1.12, 1.14) are the same
  // decisions, and two copies of them is two places for the next fix to be applied to one of.
  // `make_record` returns the std::array the record builders return, so the wire bytes are still
  // position_record()'s and speed_record()'s, unchanged and still separately tested.
  // Defined in src/servo_bus.cpp: the two public entry points there are its only instantiations.
  template<typename Goal, std::size_t RecordBytes, typename MakeRecord>
  WriteResult write_goal_group(
    const std::vector<Goal> & goals, std::size_t max_records_per_packet, uint8_t base_register,
    MakeRecord make_record);

  // Hidden, not overridden: a deleted function may not override a non-deleted one. These three
  // stay reachable through an SMS_STS &, which is why nothing in this package ever holds a base
  // reference to the bus.
  using SCSerial::begin;        // open() calls SCSerial::begin(...) qualified
  using SCSerial::end;          // close() calls SCSerial::end() qualified
  using SCSerial::setBaudRate;  // its default: branch reads an uninitialised speed_t

  std::string port_;
  int baudrate_ = 0;
  // Packet-building scratch, owned by the bus so the real-time path allocates nothing after the
  // first cycle. One chunk at a time, so they are never larger than the per-packet maxima.
  // Consequence, deliberate: write_goal_positions/write_goal_speeds are not reentrant and must not
  // be called from two threads, nor interleaved between a syncReadPacketTx and its reply -- the
  // vendored syncWrite opens with rFlushSCS() (src/SCS.cpp:126), which would discard it.
  std::vector<uint8_t> goal_ids_;
  std::vector<uint8_t> goal_records_;
  // The receive buffer the vendored library would otherwise allocate for us. Sized once in the
  // constructor and never resized, so sync_read_feedback() allocates nothing on the RT path and
  // the syncReadRxBuff pointer can never dangle (PHASE3 2.10, 2.13-2.16).
  std::vector<uint8_t> rx_buf_;
  // A mutable copy of one chunk's id list: SCS::syncReadPacketTx takes `u8 ID[]` by non-const
  // pointer (include/SCS.h:28) and a vendored signature is not something to const_cast around.
  std::vector<uint8_t> tx_ids_;
  // One chunk's decoded blocks. The walker is a free function that resizes its own output
  // (PHASE3 R17), so a chunk cannot be parsed straight into a sub-range of the caller's vector;
  // this is that sub-range's staging area, reserved once beside tx_ids_.
  std::vector<FeedbackBlock> chunk_blocks_;
  SyncReadStats sync_read_stats_;
  // The lock descriptor, opened before the library's and held for the life of the session. It is
  // never read from, so rFlushSCS()'s tcflush of the shared input queue steals nothing from it.
  int lock_fd_ = -1;
};

}  // namespace waveshare_servos

#endif  // SERVO_BUS_HPP_
