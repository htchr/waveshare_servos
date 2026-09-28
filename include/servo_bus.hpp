// ServoBus: the package's only access to the serial bus. It adds a port lock, a real open()
// errno and a full close to the vendored SMS_STS layer. See docs/design.md, "Port lock".

#ifndef SERVO_BUS_HPP_
#define SERVO_BUS_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// SMS_STS.h, not SCServo.h: the driver uses no SMSBL, SCSCL or SMSCL class.
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
// Largest n with overhead + n * (record_bytes + 1) <= buffer_bytes: a sync-write frame must
// fit txBuf, which writeSCS fills with no bound check. At namespace scope because a class
// cannot initialise its own static constexpr members with its own member functions.
constexpr std::size_t max_sync_write_records(
  std::size_t buffer_bytes, std::size_t overhead_bytes, std::size_t record_bytes) noexcept
{
  return (buffer_bytes - overhead_bytes) / (record_bytes + 1);
}
}  // namespace detail

// Sign-magnitude, not two's complement: bit 15 is the direction and bits 0..14 the magnitude,
// so -100 goes out as `64 80`, not `9c ff`. See docs/design.md, "Signed values".
uint16_t sign_magnitude_encode(int16_t value) noexcept;

// One position servo's goal, as a 7-byte sync-write record at register 41: ACC, goal
// position, goal time (0), goal speed. The driver clamps every field; the bus checks the id.
struct GoalPosition
{
  uint8_t id = 0;         // 1..253; 0xfe is the broadcast the frame itself carries (SCS.cpp:132)
  int16_t position = 0;   // goal, encoder ticks, SERVO frame, in [-32767, 32767]
  uint16_t speed = 0;     // goal-speed MAGNITUDE, ticks/s; 0 means "no speed limit", not "stop"
  uint8_t acc = 0;        // register 41; 0 is the documented "no ramp" opt-out, not "unset"
};

// One wheel's goal speed, as a 2-byte sync-write record at register 46. No ACC field:
// write_acc() writes register 41 on its own.
struct GoalSpeed
{
  uint8_t id = 0;
  // Signed ticks/s. 0 is STOP here, but 0 in GoalPosition::speed means "no limit".
  // See docs/design.md, "Goal speed".
  int16_t speed = 0;
};

// Why a sync write was refused. Nothing reached the tty, so the servos keep their last goals:
// a turning wheel KEEPS TURNING. The driver logs refusals as an ERROR.
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
  // sync-write frames handed to the tty; 0 for an empty goal list, which is success
  std::size_t packets = 0;
  // goal records inside them; equals goals.size() whenever status == OK
  std::size_t records = 0;
  // index into the caller's vector of the first offending goal, meaningful only on INVALID_ID
  std::size_t first_bad = 0;
  explicit operator bool() const noexcept {return status == WriteStatus::OK;}
};

// One servo's raw feedback block (registers 56..70), in the units of the SMS_STS ReadX(-1)
// accessors. Nothing is scaled here: units.hpp decode() does that for both read paths.
// See docs/design.md, "Feedback block".
struct FeedbackBlock
{
  // Every member has a default, so a field that a path leaves unset reads 0.
  bool valid = false;         // this servo answered with a frame that passed the gate
  uint8_t status = 0;         // frame byte 4 -- what SCS::Read would have put in SCS::Error
  int position_ticks = 0;     // ReadPos(-1)      sign-magnitude on bit 15
  int speed_ticks = 0;        // ReadSpeed(-1)    sign-magnitude on bit 15
  int load_raw = 0;           // ReadLoad(-1)     sign-magnitude on bit 10
  int voltage_raw = 0;        // ReadVoltage(-1)  unsigned byte
  int temperature_raw = 0;    // ReadTemper(-1)   unsigned byte, degrees C
  int moving_raw = 0;         // ReadMove(-1)     unsigned flag; carried, not published
  int current_counts = 0;     // ReadCurrent(-1)  sign-magnitude on bit 15
};

// Decode the 15-byte block at `data` (registers 56..70, little endian) exactly as the SMS_STS
// ReadX(-1) accessors do, and set `status` and valid = true. `data` must hold 15 bytes.
FeedbackBlock decode_feedback_block(const uint8_t * data, uint8_t status) noexcept;

// Bus-level sync-read totals for the life of the process. The per-servo failure counts are
// in the driver, which knows the joint of each id.
struct SyncReadStats
{
  uint64_t transactions = 0;    // sync_read_feedback() chunks that reached the wire
  uint64_t short_bursts = 0;    // readSCS() returned fewer bytes than the id list demanded
  uint64_t bad_frames = 0;      // frames with a good header but a bad id, length or checksum
  // Requested ids with no decoded frame, summed over chunks. Overlaps bad_frames: a rejected
  // frame also leaves its id missing.
  uint64_t missing_frames = 0;
  uint64_t drains = 0;          // drain_input() runs, that is, error-path recoveries
};

// Walk one reply burst and fill `out` slot for slot: out[k] is ids[k]'s reply, and valid is
// false if that servo was silent or its frame failed the gate. Returns the slots filled, and
// adds to `stats` when it is not null. Replies come in request order, so the id match is
// forward only. See docs/design.md, "Sync read".
std::size_t parse_sync_read_burst(
  const uint8_t * buffer, std::size_t length, const std::vector<uint8_t> & ids,
  std::vector<FeedbackBlock> * out, SyncReadStats * stats);

// What one checked transaction saw on the wire. SCS::Read checks neither the id nor the
// length, and no vendored call sees a second reply (two servos on one id).
// See docs/design.md, "Checked transactions".
enum class ReplyKind : uint8_t
{
  NOT_OPEN,       // closed bus; nothing sent
  INVALID_ID,     // id 0xfe or 0xff; nothing sent
  INVALID_COUNT,  // a count outside 1..checked_max_bytes, or no data to write; nothing sent
  SILENT,         // 0 bytes in the reply window
  ONE,            // exactly one well-formed frame from the addressed id, nothing after it
  EXTRA,          // one well-formed frame, then more bytes in the drain window (a doubled twin)
  WRONG_ID,       // a well-formed frame of the expected length from another id (never for a WRITE)
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

  // The seven rates SCSerial::begin() maps (it runs any other rate at 115200). open() and
  // the on_init check of `baudrate` both use this.
  // See docs/configuration.md, "Hardware parameters".
  static bool is_supported_baudrate(int baudrate) noexcept;

  // Take the port exclusively. The arguments are in the reverse order of SCSerial::begin().
  // Every failure leaves the bus closed, with no lock and no TIOCEXCL. After EXCLUSIVE_FAILED,
  // the tty keeps the new 8N1 settings: the vendored library never restores the saved termios.
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

  // The smallest io_timeout_ms for a sync read of `servos` ids (one chunk): ceil(1.19 *
  // (0.476 + 0.290 * servos)) + 1 ms, from bench measurements. The bus does not enforce it;
  // on_init compares io_timeout_ms with it. See docs/bus-timing.md, "Timeout floor".
  static constexpr uint32_t min_io_timeout_ms(std::size_t servos) noexcept
  {
    return (119u * (476u + 290u * static_cast<uint32_t>(servos)) + 99999u) / 100000u + 1u;
  }

  // The vendored transmit buffer (SCSerial::txBuf); the constructor static_asserts that this is
  // still its size, as a tripwire against an upstream refresh.
  static constexpr std::size_t tx_buffer_bytes = 255;

  // A sync-write frame is 7 header bytes, (1 id + record) bytes per servo and 1 checksum.
  // writeSCS has no bound check, so these sizes keep a large robot from overflowing txBuf.
  // See docs/design.md, "Sync write".
  static constexpr std::size_t sync_write_overhead_bytes = 8;
  static constexpr std::size_t goal_position_record_bytes = 7;
  static constexpr std::size_t goal_speed_record_bytes = 2;

  // Records per frame (30 positions, 82 speeds), derived from txBuf and the record size.
  // servo_bus.cpp static_asserts that each limit fits and that limit + 1 does not.
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

  // The record bytes as pure functions: the chunk loop and the tests use the same layout code.
  static std::array<uint8_t, goal_position_record_bytes> position_record(
    uint8_t acc, int16_t goal_ticks, uint16_t goal_speed) noexcept;
  static std::array<uint8_t, goal_speed_record_bytes> speed_record(int16_t speed_ticks) noexcept;

  // Send 7-byte records at register 41, at most 30 per sync-write frame. The bytes match
  // SMS_STS::SyncWritePosEx for every goal the driver sends, but `goals` is not modified.
  WriteResult write_goal_positions(const std::vector<GoalPosition> & goals);

  // Send 2-byte records at register 46, at most 82 per frame: the sync stage of SyncWriteSpe,
  // without its per-servo ACC write (see write_acc()).
  WriteResult write_goal_speeds(const std::vector<GoalSpeed> & goals);

  // Write one servo's acceleration register (41); true if it acked. A silent servo costs one
  // io_timeout_ms. An ack with a fault status is still true, and the call overwrites SCS::Error.
  // See docs/design.md, "Wheel acceleration".
  bool write_acc(uint8_t id, uint8_t acc);

  // Size the write scratch so the first write() allocates nothing. Clamped to one chunk:
  // a 100-servo position group reserves 30 records, not 100.
  void reserve_goal_capacity(std::size_t position_servos, std::size_t speed_servos);

  // Diagnostic only: the scratch capacity, so a test can prove the write path stops allocating.
  std::size_t goal_scratch_capacity_bytes() const noexcept;

  // Registers 56..70 (present position .. present current): the block FeedBack() reads and
  // the block one sync-read frame carries.
  static constexpr uint8_t feedback_first_register = 56;
  static constexpr std::size_t feedback_block_bytes = 15;
  // One reply frame is `FF FF id (nLen+2) status data[nLen] ~cks`.
  static constexpr std::size_t sync_read_frame_bytes = feedback_block_bytes + 6;    // 21
  // Ids per request, so one burst fits one buffer and one io_timeout_ms.
  static constexpr std::size_t sync_read_max_ids = 30;
  // Slack: the vendored syncReadPacketRx can read up to nLen + 3 bytes past the received
  // burst. Our parser cannot, but the vendored code is still linked.
  // See docs/design.md, "Receive buffer".
  static constexpr std::size_t sync_read_slack_bytes = feedback_block_bytes + 3;    // 18
  static constexpr std::size_t sync_read_buffer_bytes =
    sync_read_max_ids * sync_read_frame_bytes + sync_read_slack_bytes;              // 648
  // Two USB frames: the whole drain budget (a deadline), not a per-read timeout.
  static constexpr uint32_t sync_read_drain_ms = 2;

  // One sync read of registers 56..70 for every id, in chunks of at most 30 ids (each waits at
  // most one io_timeout_ms). blocks[k] is ids[k]'s reply; returns how many blocks are valid.
  // Ids must be unique and in 1..253 (on_init checks this). Not reentrant: it owns one receive
  // buffer, and every request starts with rFlushSCS(). See docs/design.md, "Sync read".
  std::size_t sync_read_feedback(
    const std::vector<uint8_t> & ids, std::vector<FeedbackBlock> & blocks);

  // Per-servo fallback: one Read(id, 56, ., 15), the same transaction as SMS_STS::FeedBack(),
  // decoded like the sync path. `out` is reset on every path. The status byte comes from
  // SCS::Error at once, before another call overwrites it.
  bool read_feedback_one(uint8_t id, FeedbackBlock & out);

  // Read and discard input for `max_ms`, on the error path only: tcflush cannot remove a late
  // frame that has not arrived yet. Returns the bytes discarded; 0 on a closed bus.
  // See docs/design.md, "Late frames".
  std::size_t drain_input(uint32_t max_ms = sync_read_drain_ms) noexcept;

  // For the life of the process: close() does not reset it.
  const SyncReadStats & sync_read_stats() const noexcept {return sync_read_stats_;}

  // Checked transactions for the tools; the driver never uses them. Each refuses bad input
  // without sending, reads exactly one reply, gates header, length, checksum and id, and after
  // any reply drains 2 ms so a second reply is counted. A silent id costs one window.
  // See docs/design.md, "Checked transactions".
  static constexpr std::size_t checked_max_bytes = 64;
  static constexpr uint32_t checked_drain_ms = 2;
  Reply checked_ping(uint8_t id);
  Reply checked_read(uint8_t id, uint8_t first, uint8_t count);
  // An ack from ANY id is accepted and returned in from_id: an id write may be acked from the
  // new id, so callers verify by reading back. The window is ack_timeout_ms (EEPROM writes are
  // slow); keep it below 1000 ms, because readSCS puts all of it into tv_usec.
  Reply checked_write(
    uint8_t id, uint8_t first, const uint8_t * data, uint8_t count, uint32_t ack_timeout_ms);

  // RESET (0x06), not in INST.h. Measured on the ST3025: EEPROM 6..39 to factory values with
  // the id kept, baud to 1 Mbaud, torque off, lock closed. The ack comes at the old rate.
  // See docs/tools.md, "factory_reset".
  static constexpr uint8_t inst_reset = 0x06;
  // The checked transaction of a RESET, for factory_reset only: no parameters, so the frame is
  // FF FF id 02 06 ~sum; an ack from any other id is WRONG_ID. The window is ack_timeout_ms for
  // checked_write's reason, and must stay below 1000 ms for it too.
  Reply checked_reset(uint8_t id, uint32_t ack_timeout_ms);

  // Change the line rate of an open bus on the same descriptor, so the lock and TIOCEXCL stay
  // held. Discards received input. False with nothing changed if closed or unsupported; false
  // with the line state unknown if the kernel refuses. (SCSerial::setBaudRate cannot do this.)
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

  // Shared body of write_goal_positions() and write_goal_speeds(): refusals, id check and
  // chunking. Defined in servo_bus.cpp, which holds its only two instantiations.
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
  // Write scratch (one chunk), owned so the RT path does not allocate. Not reentrant: syncWrite
  // starts with rFlushSCS(), which would discard a pending sync-read reply.
  std::vector<uint8_t> goal_ids_;
  std::vector<uint8_t> goal_records_;
  // Our sync-read receive buffer, sized once and never resized: no RT allocation, and the
  // syncReadRxBuff pointer cannot dangle.
  std::vector<uint8_t> rx_buf_;
  // A mutable copy of one chunk's id list: SCS::syncReadPacketTx takes `u8 ID[]` by non-const
  // pointer (include/SCS.h:28) and a vendored signature is not something to const_cast around.
  std::vector<uint8_t> tx_ids_;
  // One chunk's decoded blocks: the parser resizes its output, so it cannot write into a
  // sub-range of the caller's vector.
  std::vector<FeedbackBlock> chunk_blocks_;
  SyncReadStats sync_read_stats_;
  // The lock descriptor, opened before the library's and held for the life of the session. It is
  // never read from, so rFlushSCS()'s tcflush of the shared input queue steals nothing from it.
  int lock_fd_ = -1;
};

}  // namespace waveshare_servos

#endif  // SERVO_BUS_HPP_
