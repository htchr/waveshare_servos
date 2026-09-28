#include "servo_bus.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <system_error>
#include <type_traits>
#include <vector>

namespace waveshare_servos
{
namespace
{

// The seven rates SCSerial::begin() maps. Not 230400: begin() would silently run it at 115200.
constexpr std::array<int, 7> kMappedBaudrates = {
  9600, 19200, 38400, 57600, 115200, 500000, 1000000};

// The termios speed of each of those rates, for set_baudrate(); B0 (hang up) for anything else,
// which set_baudrate() never passes on because it checks is_supported_baudrate() first.
speed_t speed_of(int baudrate) noexcept
{
  switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 500000:
      return B500000;
    case 1000000:
      return B1000000;
    default:
      return B0;
  }
}

// 254 is the broadcast id and 255 the header byte. on_init rejects both; this is a tripwire.
constexpr bool is_addressable_id(uint8_t id) noexcept
{
  return id >= 1 && id <= 253;
}

// Little endian, hard-coded like the ReadX(-1) accessors, so a change to SCS::End cannot
// change the decode.
constexpr int little_endian_word(const uint8_t * data, std::size_t offset) noexcept
{
  return static_cast<int>(data[offset]) | (static_cast<int>(data[offset + 1]) << 8);
}

// Position, speed, current: sign on bit 15, as the vendored accessors decode it. 0x8000 gives
// 0, the accessors' own answer, kept on purpose.
constexpr int signed_on_bit_fifteen(int word) noexcept
{
  return (word & (1 << 15)) != 0 ? -(word & ~(1 << 15)) : word;
}

// Load: sign on bit 10; bits 11..15 stay in the magnitude, as in ReadLoad (0xffff gives
// -64511). Do not narrow the mask to 0x3ff. See docs/design.md, "Signed values".
constexpr int signed_on_bit_ten(int word) noexcept
{
  return (word & (1 << 10)) != 0 ? -(word & ~(1 << 10)) : word;
}

// Reply frame: FF FF id len status data[15] ~cks. The checksum (~sum of bytes 2..19) is
// computed here, not by the library, so a pass checks the servo's bytes.
constexpr std::size_t kFrameIdOffset = 2;
constexpr std::size_t kFrameLengthOffset = 3;
constexpr std::size_t kFrameStatusOffset = 4;
constexpr std::size_t kFrameDataOffset = 5;

uint8_t sync_read_frame_checksum(const uint8_t * frame) noexcept
{
  uint8_t sum = 0;
  for (std::size_t i = kFrameIdOffset; i + 1 < ServoBus::sync_read_frame_bytes; i++) {
    sum = static_cast<uint8_t>(sum + frame[i]);
  }
  return static_cast<uint8_t>(~sum);
}

// A frame with no payload, `ff ff id 02 status ~cks`: a ping reply, a write ack, or a late ack
// that a READ catches instead of its reply.
constexpr std::size_t kStatusFrameBytes = 6;

// The same arithmetic as sync_read_frame_checksum, over a frame of any length: the complemented
// uint8_t sum of indices 2..length-2.
uint8_t frame_checksum(const uint8_t * frame, std::size_t length) noexcept
{
  uint8_t sum = 0;
  for (std::size_t i = kFrameIdOffset; i + 1 < length; i++) {
    sum = static_cast<uint8_t>(sum + frame[i]);
  }
  return static_cast<uint8_t>(~sum);
}

// total(n, nLen) = 7 header + n*(1 + nLen) records + 1 checksum (src/SCS.cpp:130-149).
constexpr std::size_t sync_write_bytes(std::size_t records, std::size_t record_bytes) noexcept
{
  return ServoBus::sync_write_overhead_bytes + records * (record_bytes + 1);
}

}  // namespace

// Per record size: a full chunk fits txBuf, and limit + 1 does not. A size change fails the
// build, not the bench.
static_assert(
  sync_write_bytes(
    ServoBus::max_goal_positions_per_packet,
    ServoBus::goal_position_record_bytes) <= ServoBus::tx_buffer_bytes,
  "a full position chunk no longer fits SCSerial::txBuf");
static_assert(
  sync_write_bytes(
    ServoBus::max_goal_positions_per_packet + 1,
    ServoBus::goal_position_record_bytes) > ServoBus::tx_buffer_bytes,
  "max_goal_positions_per_packet is not the largest position chunk that fits");
static_assert(
  sync_write_bytes(
    ServoBus::max_goal_speeds_per_packet,
    ServoBus::goal_speed_record_bytes) <= ServoBus::tx_buffer_bytes,
  "a full speed chunk no longer fits SCSerial::txBuf");
static_assert(
  sync_write_bytes(
    ServoBus::max_goal_speeds_per_packet + 1,
    ServoBus::goal_speed_record_bytes) > ServoBus::tx_buffer_bytes,
  "max_goal_speeds_per_packet is not the largest speed chunk that fits");

// This API mixes the fixed-width types with the vendored typedefs (include/INST.h:11-16), and the
// casts at the syncWrite call site are exact only while the two coincide.
static_assert(
  std::is_same_v<u8, uint8_t>&& std::is_same_v<u16, uint16_t>&& std::is_same_v<s16, int16_t>,
  "the vendored INST.h typedefs are no longer the fixed-width types this API uses");

const char * to_string(BusStatus status)
{
  // No `default:`, so an unnamed new status is a -Wswitch warning. The return after the switch
  // handles a value cast in from outside the enum.
  switch (status) {
    case BusStatus::OK:
      return "ok";
    case BusStatus::ALREADY_OPEN:
      return "already open";
    case BusStatus::UNSUPPORTED_BAUDRATE:
      return "unsupported baud rate";
    case BusStatus::INVALID_TIMEOUT:
      return "invalid io timeout";
    case BusStatus::OPEN_FAILED:
      return "open failed";
    case BusStatus::NOT_A_TTY:
      return "not a tty";
    case BusStatus::LOCK_OPEN_FAILED:
      return "lock descriptor could not be opened";
    case BusStatus::LOCK_FAILED:
      return "port is locked by another process";
    case BusStatus::TERMIOS_FAILED:
      return "line settings could not be applied";
    case BusStatus::EXCLUSIVE_FAILED:
      return "port refused exclusive access";
  }
  return "unknown";
}

const char * to_string(ReplyKind kind)
{
  // No `default:` label, for the reason to_string(BusStatus) has none. snake_case with no spaces,
  // because the tools put these into key=value detail lines.
  switch (kind) {
    case ReplyKind::NOT_OPEN:
      return "not_open";
    case ReplyKind::INVALID_ID:
      return "invalid_id";
    case ReplyKind::INVALID_COUNT:
      return "invalid_count";
    case ReplyKind::SILENT:
      return "silent";
    case ReplyKind::ONE:
      return "one";
    case ReplyKind::EXTRA:
      return "extra";
    case ReplyKind::WRONG_ID:
      return "wrong_id";
    case ReplyKind::STATUS_ONLY:
      return "status_only";
    case ReplyKind::GARBLED:
      return "garbled";
  }
  return "unknown";
}

const char * to_string(WriteStatus status)
{
  // No `default:` label, for the same reason to_string(BusStatus) has none: adding a status
  // without naming it here is a -Wswitch warning, and this package builds -Wall -Wextra.
  switch (status) {
    case WriteStatus::OK:
      return "ok";
    case WriteStatus::NOT_OPEN:
      return "bus is not open";
    case WriteStatus::INVALID_ID:
      return "goal names an id outside 1..253";
  }
  return "unknown";
}

uint16_t sign_magnitude_encode(int16_t value) noexcept
{
  // Saturates at magnitude 32767, so -32768 gives `ff ff` where the library gives `00 80`.
  // The driver clamps goals to +/-32767 first, so it never sends that value.
  const int32_t magnitude = std::min<int32_t>(std::abs(static_cast<int32_t>(value)), 0x7fff);
  return static_cast<uint16_t>(value < 0 ? (magnitude | 0x8000) : magnitude);
}

FeedbackBlock decode_feedback_block(const uint8_t * data, uint8_t status) noexcept
{
  FeedbackBlock block;
  block.valid = true;
  block.status = status;
  block.position_ticks = signed_on_bit_fifteen(little_endian_word(data, 0));
  block.speed_ticks = signed_on_bit_fifteen(little_endian_word(data, 2));
  block.load_raw = signed_on_bit_ten(little_endian_word(data, 4));
  block.voltage_raw = data[6];
  block.temperature_raw = data[7];
  block.moving_raw = data[10];
  block.current_counts = signed_on_bit_fifteen(little_endian_word(data, 13));
  return block;
}

std::size_t parse_sync_read_burst(
  const uint8_t * buffer, std::size_t length, const std::vector<uint8_t> & ids,
  std::vector<FeedbackBlock> * out, SyncReadStats * stats)
{
  const std::size_t count = ids.size();
  // assign(), not resize(): a stale block from the last cycle would be published as fresh.
  out->assign(count, FeedbackBlock{});

  std::size_t filled = 0;
  std::size_t pos = 0;      // read cursor in the burst
  std::size_t next = 0;     // the first request slot not yet filled
  uint64_t bad_frames = 0;

  // The bound is checked before any frame byte is read (the vendored parser over-reads by up
  // to 18 bytes). Every pass advances pos, so the loop ends.
  while (next < count && pos + ServoBus::sync_read_frame_bytes <= length) {
    // Gate 1, the header. A byte skipped here is not a rejected frame -- nothing was claimed --
    // so no counter moves. One byte at a time, as the library resyncs (src/SCS.cpp:344-351).
    if (!(buffer[pos] == 0xff && buffer[pos + 1] == 0xff &&
      buffer[pos + kFrameIdOffset] != 0xff))
    {
      pos++;
      continue;
    }

    // Gate 2, the id, forward only: skipped ids are silent servos, and an id behind the cursor
    // is stale, foreign or a duplicate.
    std::size_t slot = next;
    while (slot < count && ids[slot] != buffer[pos + kFrameIdOffset]) {
      slot++;
    }

    // On a rejection, resync by one byte and continue: one bad frame says nothing about the
    // next, and stopping would lose every servo after it.
    const bool addressed = slot < count;
    const bool sized = addressed &&
      buffer[pos + kFrameLengthOffset] == ServoBus::feedback_block_bytes + 2;
    const bool summed = sized &&
      buffer[pos + ServoBus::sync_read_frame_bytes - 1] == sync_read_frame_checksum(buffer + pos);
    if (!summed) {
      pos++;
      bad_frames++;
      continue;
    }

    // Data and status come from the same frame, which the shared SCS::Error cannot promise.
    // Slots next..slot-1 stay invalid: those servos did not answer.
    (*out)[slot] = decode_feedback_block(
      buffer + pos + kFrameDataOffset, buffer[pos + kFrameStatusOffset]);
    next = slot + 1;
    filled++;
    pos += ServoBus::sync_read_frame_bytes;
  }

  if (stats != nullptr) {
    // Added to, not assigned: the session totals span every chunk of a sync read.
    stats->bad_frames += bad_frames;
    stats->missing_frames += count - filled;
  }
  return filled;
}

std::array<uint8_t, ServoBus::goal_position_record_bytes> ServoBus::position_record(
  uint8_t acc, int16_t goal_ticks, uint16_t goal_speed) noexcept
{
  // Byte 0 is register 41 (ACC), so ACC rides in every position record. The byte split is
  // written here (Host2SCS is not static); tests compare it with Host2SCS and SyncWritePosEx.
  const uint16_t goal = sign_magnitude_encode(goal_ticks);
  return {
    acc,
    static_cast<uint8_t>(goal & 0xff), static_cast<uint8_t>(goal >> 8),
    // GOAL_TIME is hardcoded 0 by the library too (src/SMS_STS.cpp:75): the servo runs its own
    // profile and the goal speed is what paces it.
    0x00, 0x00,
    // The goal-speed field of the POSITION record is an unsigned magnitude with no direction bit
    // -- travel direction comes from the goal position -- so it is emitted unencoded.
    static_cast<uint8_t>(goal_speed & 0xff), static_cast<uint8_t>(goal_speed >> 8)};
}

std::array<uint8_t, ServoBus::goal_speed_record_bytes> ServoBus::speed_record(
  int16_t speed_ticks) noexcept
{
  // Sign-magnitude, bit 15 = direction. 0 is STOP, which every deactivation sends: never floor
  // it to 1 as the position path does.
  const uint16_t speed = sign_magnitude_encode(speed_ticks);
  return {static_cast<uint8_t>(speed & 0xff), static_cast<uint8_t>(speed >> 8)};
}

template<typename Goal, std::size_t RecordBytes, typename MakeRecord>
WriteResult ServoBus::write_goal_group(
  const std::vector<Goal> & goals, const std::size_t max_records_per_packet,
  const uint8_t base_register, MakeRecord make_record)
{
  WriteResult result;
  // Empty first: nothing to send is success even on a closed port, so an empty group (the
  // position group of a wheels-only robot) never logs a refusal.
  if (goals.empty()) {
    return result;
  }
  if (!is_open()) {
    result.status = WriteStatus::NOT_OPEN;
    return result;
  }
  // All or nothing, before any byte is built: a partial command set would hide a driver bug
  // behind plausible motion.
  for (std::size_t i = 0; i < goals.size(); i++) {
    if (!is_addressable_id(goals[i].id)) {
      result.status = WriteStatus::INVALID_ID;
      result.first_bad = i;
      return result;
    }
  }
  for (std::size_t first = 0; first < goals.size(); first += max_records_per_packet) {
    const std::size_t count = std::min(max_records_per_packet, goals.size() - first);
    // resize(), not assign(): with the capacity reserved by reserve_goal_capacity() neither call
    // allocates, and every byte below is overwritten before it is read.
    goal_ids_.resize(count);
    goal_records_.resize(count * RecordBytes);
    for (std::size_t k = 0; k < count; k++) {
      const Goal & goal = goals[first + k];
      goal_ids_[k] = goal.id;
      const std::array<uint8_t, RecordBytes> record = make_record(goal);
      std::copy(
        record.begin(), record.end(),
        goal_records_.begin() + static_cast<std::ptrdiff_t>(k * RecordBytes));
    }
    // The vendored syncWrite builds the frame. No gap between chunks: none is needed at 1 Mbaud
    // (measured).
    syncWrite(
      goal_ids_.data(), static_cast<u8>(count), base_register, goal_records_.data(),
      static_cast<u8>(RecordBytes));
    result.packets++;
    result.records += count;
  }
  return result;
}

WriteResult ServoBus::write_goal_positions(const std::vector<GoalPosition> & goals)
{
  return write_goal_group<GoalPosition, goal_position_record_bytes>(
    goals, max_goal_positions_per_packet, SMS_STS_ACC,
    [](const GoalPosition & goal) {
      return position_record(goal.acc, goal.position, goal.speed);
    });
}

WriteResult ServoBus::write_goal_speeds(const std::vector<GoalSpeed> & goals)
{
  // Register 46 only: SyncWriteSpe's per-wheel ACC write is gone, which saves about 0.57 ms per
  // wheel per cycle (measured).
  return write_goal_group<GoalSpeed, goal_speed_record_bytes>(
    goals, max_goal_speeds_per_packet, SMS_STS_GOAL_SPEED_L,
    [](const GoalSpeed & goal) {return speed_record(goal.speed);});
}

bool ServoBus::write_acc(uint8_t id, uint8_t acc)
{
  // A closed bus must not reach readSCS: FD_SET(-1) aborts under _FORTIFY_SOURCE.
  // See docs/design.md, "Vendored library traps".
  if (!is_open()) {
    return false;
  }
  // `!= 0`, not `!= -1`: Ack() returns 0 on failure and 1 on success, never -1. A -1 test would
  // always pass and hide the driver's unramped-wheel warning.
  return writeByte(id, SMS_STS_ACC, acc) != 0;
}

std::size_t ServoBus::sync_read_feedback(
  const std::vector<uint8_t> & ids, std::vector<FeedbackBlock> & blocks)
{
  // Every slot is written on every call, so a failed transaction can never leave last cycle's
  // sample behind to be published as this cycle's.
  blocks.assign(ids.size(), FeedbackBlock{});
  // An empty list would send a request and wait a full timeout for nothing. A closed bus would
  // reach FD_SET(-1) in readSCS and abort.
  if (ids.empty() || !is_open()) {
    return 0;
  }
  std::size_t valid = 0;
  // Chunks are independent transactions: a silent chunk does not stop the loop.
  for (std::size_t first = 0; first < ids.size(); first += sync_read_max_ids) {
    const std::size_t count = std::min(sync_read_max_ids, ids.size() - first);
    tx_ids_.assign(ids.begin() + first, ids.begin() + first + count);
    // Set on every call (one store) to keep the invariant next to the code that needs it.
    syncReadRxBuff = rx_buf_.data();
    // Ask for EXACTLY the bytes the repliers owe: readSCS returns early only when it has them
    // all, so more costs a full timeout every cycle and fewer cuts off the last frame.
    const std::size_t expected = count * sync_read_frame_bytes;
    syncReadRxBuffMax = static_cast<u16>(expected);
    sync_read_stats_.transactions++;
    // One readSCS per chunk, with one shrinking timeval: a chunk costs at most one io_timeout_ms.
    // See docs/bus-timing.md, "Transaction timeout".
    const int got = syncReadPacketTx(
      tx_ids_.data(), static_cast<u8>(count), feedback_first_register,
      static_cast<u8>(feedback_block_bytes));
    // Clamp to `expected`, not to the buffer size, so a bogus length cannot walk stale bytes.
    // A -1 from an unpatched readSCS arrives as 65535 through the u16 member.
    std::size_t len = (got < 0) ? 0u : static_cast<std::size_t>(got);
    len = std::min(len, expected);
    // A short burst is normal when a servo is silent, and the other frames are good (measured).
    // Count it and parse what arrived.
    const bool short_burst = len != expected;
    if (short_burst) {
      sync_read_stats_.short_bursts++;
    }
    valid += parse_sync_read_burst(
      rx_buf_.data(), len, tx_ids_, &chunk_blocks_, &sync_read_stats_);
    std::copy(
      chunk_blocks_.begin(), chunk_blocks_.end(),
      blocks.begin() + static_cast<std::ptrdiff_t>(first));
    // Drain only after a short burst: after a full one, the rFlushSCS() at the start of the next
    // request is enough (measured).
    if (short_burst) {
      drain_input();
    }
  }
  // SCS::Error and SCSerial::Err are not touched: each block takes its status from its frame.
  return valid;
}

bool ServoBus::read_feedback_one(uint8_t id, FeedbackBlock & out)
{
  out = FeedbackBlock{};
  // Same refusal as sync_read_feedback(): readSCS would FD_SET(-1) and abort.
  if (!is_open()) {
    return false;
  }
  std::array<uint8_t, feedback_block_bytes> block{};
  if (Read(id, feedback_first_register, block.data(), static_cast<u8>(feedback_block_bytes)) !=
    static_cast<int>(feedback_block_bytes))
  {
    return false;
  }
  // Error is read on the very next line, with nothing in between. Read() writes it only on
  // success (src/SCS.cpp:201) and Ack, Ping and every write rewrite it with a different meaning.
  out = decode_feedback_block(block.data(), Error);
  return true;
}

std::size_t ServoBus::drain_input(uint32_t max_ms) noexcept
{
  // Counted only when open; discard_input() does the work.
  if (!is_open()) {
    return 0;
  }
  sync_read_stats_.drains++;
  return discard_input(max_ms);
}

std::size_t ServoBus::discard_input(uint32_t max_ms) noexcept
{
  if (!is_open()) {
    return 0;
  }
  // A deadline that a quiet line does not end: the late frame may still come, so a failed
  // cycle pays the whole budget. See docs/bus-timing.md, "Cost of a silent servo".
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(max_ms);
  std::array<uint8_t, 64> scratch{};
  std::size_t discarded = 0;
  while (true) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return discarded;
    }
    const auto remaining =
      std::chrono::duration_cast<std::chrono::microseconds>(deadline - now).count();
    fd_set readable;
    FD_ZERO(&readable);
    FD_SET(fd, &readable);
    // A fresh timeval per call, because Linux's select() writes the unslept remainder back into
    // it and a reused one would shrink to zero and spin.
    struct timeval window;
    window.tv_sec = static_cast<time_t>(remaining / 1000000);
    window.tv_usec = static_cast<suseconds_t>(remaining % 1000000);
    if (::select(fd + 1, &readable, nullptr, nullptr, &window) > 0) {
      const ssize_t n = ::read(fd, scratch.data(), scratch.size());
      if (n > 0) {
        discarded += static_cast<std::size_t>(n);
      }
    }
  }
}

Reply ServoBus::checked_ping(uint8_t id)
{
  // No parameters; writeBuf still sums MemAddr into the checksum, so it is 0 as in SCS::Ping.
  return checked_transaction(
    id, INST_PING, 0, nullptr, 0, 0, static_cast<uint32_t>(IOTimeOut), true);
}

Reply ServoBus::checked_read(uint8_t id, uint8_t first, uint8_t count)
{
  const bool count_ok = count >= 1 && count <= checked_max_bytes;
  // The request SCS::Read builds (src/SCS.cpp:175-177): one parameter, the length.
  uint8_t length = count;
  return checked_transaction(
    id, INST_READ, first, &length, 1, count, static_cast<uint32_t>(IOTimeOut), count_ok);
}

Reply ServoBus::checked_write(
  uint8_t id, uint8_t first, const uint8_t * data, uint8_t count, uint32_t ack_timeout_ms)
{
  const bool count_ok = data != nullptr && count >= 1 && count <= checked_max_bytes;
  // writeBuf takes u8 * (include/SCS.h:51) and a vendored signature is not something to
  // const_cast around, so the bytes go through a copy -- bounded, because the count is.
  std::array<uint8_t, checked_max_bytes> bytes{};
  if (count_ok) {
    std::copy(data, data + count, bytes.begin());
  }
  return checked_transaction(
    id, INST_WRITE, first, bytes.data(), count, 0, ack_timeout_ms, count_ok);
}

Reply ServoBus::checked_reset(uint8_t id, uint32_t ack_timeout_ms)
{
  // No parameters, like a ping: writeBuf still sums MemAddr (0) into the checksum, which gives the
  // bench's FF FF 04 02 06 F3 for id 4.
  return checked_transaction(id, inst_reset, 0, nullptr, 0, 0, ack_timeout_ms, true);
}

bool ServoBus::set_baudrate(int baudrate) noexcept
{
  if (!is_open() || !is_supported_baudrate(baudrate)) {
    return false;
  }
  struct termios settings{};
  if (::tcgetattr(fd, &settings) != 0) {
    return false;
  }
  const speed_t speed = speed_of(baudrate);
  if (::cfsetispeed(&settings, speed) != 0 || ::cfsetospeed(&settings, speed) != 0) {
    return false;
  }
  // TCSADRAIN: a request still leaving at the old rate goes out whole before the switch.
  if (::tcsetattr(fd, TCSADRAIN, &settings) != 0) {
    return false;
  }
  ::tcflush(fd, TCIFLUSH);
  baudrate_ = baudrate;
  return true;
}

Reply ServoBus::checked_transaction(
  uint8_t id, uint8_t instruction, uint8_t first, uint8_t * params, uint8_t param_bytes,
  uint8_t payload_bytes, uint32_t window_ms, bool count_ok)
{
  Reply reply;
  // Refuse before any byte is built: port first (FD_SET(-1) aborts), then 0xfe (broadcast) and
  // 0xff (header byte), which no single servo answers for.
  if (!is_open()) {
    return reply;
  }
  if (id == 0xfe || id == 0xff) {
    reply.kind = ReplyKind::INVALID_ID;
    return reply;
  }
  if (!count_ok) {
    reply.kind = ReplyKind::INVALID_COUNT;
    return reply;
  }

  // Exactly the reply this request is owed, never more: readSCS returns early only once it has
  // them all, so asking for more would cost the whole window on every healthy transaction.
  const std::size_t expected = payload_bytes + kStatusFrameBytes;
  std::array<uint8_t, checked_max_bytes + kStatusFrameBytes> rx{};
  rFlushSCS();
  writeBuf(id, first, params, param_bytes, instruction);
  wFlushSCS();
  const auto started = std::chrono::steady_clock::now();
  // Borrowed for this one readSCS and given back on the only path out of it.
  const auto io_timeout = IOTimeOut;
  IOTimeOut = window_ms;
  const int got = readSCS(rx.data(), static_cast<int>(expected));
  IOTimeOut = io_timeout;
  reply.elapsed_us = static_cast<uint32_t>(
    std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - started).count());
  reply.bytes = (got > 0) ? std::min(static_cast<std::size_t>(got), expected) : 0u;
  if (reply.bytes == 0) {
    // No drain: nothing arrived, so a silent id costs one window and not a window and a drain.
    reply.kind = ReplyKind::SILENT;
    return reply;
  }
  // Whatever follows the reply -- a twin's copy, the rest of a frame longer than the one asked
  // for -- is read out and counted, not left to be taken for the next transaction's reply.
  reply.extra_bytes = discard_input(checked_drain_ms);

  // One frame at the head of the window, gated like a sync-read slot. No resync: a byte before
  // the header makes the reply GARBLED.
  const uint8_t * frame = rx.data();
  const bool headed = reply.bytes >= kStatusFrameBytes && frame[0] == 0xff && frame[1] == 0xff &&
    frame[kFrameIdOffset] != 0xff && frame[kFrameLengthOffset] >= 2;
  const std::size_t length = headed ? frame[kFrameLengthOffset] + 4u : 0u;
  const bool summed = headed && length <= reply.bytes &&
    frame[length - 1] == frame_checksum(frame, length);
  const bool asked_for = summed && length == expected;
  const bool late_status = summed && instruction == INST_READ && length == kStatusFrameBytes;
  if (!asked_for && !late_status) {
    reply.kind = ReplyKind::GARBLED;
    return reply;
  }
  reply.from_id = frame[kFrameIdOffset];
  reply.status = frame[kFrameStatusOffset];
  reply.frame_bytes = length;
  if (late_status) {
    reply.kind = ReplyKind::STATUS_ONLY;
    return reply;
  }
  // A write's ack is accepted from ANY id (an id write may be acked from the new one); a ping's
  // or a READ's reply only from the id that was asked.
  if (instruction != INST_WRITE && reply.from_id != id) {
    reply.kind = ReplyKind::WRONG_ID;
    return reply;
  }
  reply.kind = (reply.extra_bytes > 0) ? ReplyKind::EXTRA : ReplyKind::ONE;
  reply.data.assign(frame + kFrameDataOffset, frame + kFrameDataOffset + payload_bytes);
  return reply;
}

void ServoBus::reserve_goal_capacity(std::size_t position_servos, std::size_t speed_servos)
{
  // Clamped at the per-packet maxima because the scratch only ever holds ONE chunk: a group larger
  // than a chunk is written as several packets, not as one oversized buffer.
  const std::size_t positions = std::min(position_servos, max_goal_positions_per_packet);
  const std::size_t speeds = std::min(speed_servos, max_goal_speeds_per_packet);
  goal_ids_.reserve(std::max(positions, speeds));
  goal_records_.reserve(
    std::max(positions * goal_position_record_bytes, speeds * goal_speed_record_bytes));
}

std::size_t ServoBus::goal_scratch_capacity_bytes() const noexcept
{
  return goal_ids_.capacity() + goal_records_.capacity();
}

std::vector<int> port_holder_pids(const std::string & port) noexcept
{
  std::vector<int> pids;
  // Belt over the explicit error_codes below: this runs inside on_configure, and a lifecycle
  // callback that throws takes the whole node with it.
  try {
    std::error_code ec;
    // Both loops step with increment(ec) rather than a range-for: the range-for uses the
    // *throwing* operator++ even when the iterator was constructed with an error_code.
    const std::filesystem::directory_iterator end;
    std::filesystem::directory_iterator proc("/proc", ec);
    if (ec) {
      return {};
    }
    for (; proc != end; proc.increment(ec)) {
      if (ec) {
        return pids;
      }
      const std::string name = proc->path().filename().string();
      char * tail = nullptr;
      const int64_t pid = std::strtol(name.c_str(), &tail, 10);
      if (tail == name.c_str() || tail == nullptr || *tail != '\0' || pid <= 0) {
        continue;   // /proc holds far more than processes
      }
      std::error_code fd_ec;
      std::filesystem::directory_iterator fds(proc->path() / "fd", fd_ec);
      if (fd_ec) {
        continue;   // another user's process, or one that exited between the two calls
      }
      for (; fds != end; fds.increment(fd_ec)) {
        if (fd_ec) {
          break;
        }
        std::error_code link_ec;
        const std::filesystem::path target = std::filesystem::read_symlink(fds->path(), link_ec);
        if (!link_ec && target.string() == port) {
          pids.push_back(static_cast<int>(pid));
          break;    // a holder is listed once, however many descriptors it has on the port
        }
      }
    }
  } catch (...) {
    return {};
  }
  return pids;
}

ServoBus::ServoBus()
{
  // Tripwire: writeSCS() has no bound check, so txBuf's size is a safety property.
  static_assert(
    sizeof(txBuf) == tx_buffer_bytes,
    "the vendored SCSerial::txBuf is no longer 255 bytes");
  // The vendored constructors leave all of these indeterminate, and ReadPos/ReadSpeed/ReadLoad/
  // ReadCurrent read Err before anything has written it.
  Err = 0;
  syncReadRxPacket = nullptr;
  syncReadRxPacketIndex = 0;
  syncReadRxPacketLen = 0;
  syncReadRxBuffLen = 0;
  syncReadRxBuffMax = 0;
  // We own the receive buffer: never call syncReadBegin()/syncReadEnd() (new[] with a scalar
  // delete). Sized once, so the pointer cannot dangle and the RT path does not allocate.
  rx_buf_.assign(sync_read_buffer_bytes, 0);
  syncReadRxBuff = rx_buf_.data();
  tx_ids_.reserve(sync_read_max_ids);
  chunk_blocks_.reserve(sync_read_max_ids);
}

ServoBus::~ServoBus()
{
  close();
}

bool ServoBus::is_supported_baudrate(int baudrate) noexcept
{
  return std::find(kMappedBaudrates.begin(), kMappedBaudrates.end(), baudrate) !=
         kMappedBaudrates.end();
}

OpenResult ServoBus::open(const std::string & port, int baudrate, uint32_t io_timeout_ms)
{
  // These three checks touch nothing, so a refusal here is the same as no call at all.
  if (is_open()) {
    return OpenResult{BusStatus::ALREADY_OPEN, 0};
  }
  if (!is_supported_baudrate(baudrate)) {
    return OpenResult{BusStatus::UNSUPPORTED_BAUDRATE, 0};
  }
  if (io_timeout_ms == 0) {
    return OpenResult{BusStatus::INVALID_TIMEOUT, 0};
  }

  // Lock first, on our own descriptor: its errno is the real one (begin()'s perror() loses it),
  // and a process that loses the lock never touches TIOCEXCL. See docs/design.md, "Port lock".
  lock_fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (lock_fd_ == -1) {
    return OpenResult{BusStatus::LOCK_OPEN_FAILED, errno};
  }
  if (::isatty(lock_fd_) == 0) {
    const int failed_with = errno;
    drop_lock();
    return OpenResult{BusStatus::NOT_A_TTY, failed_with};
  }
  if (::flock(lock_fd_, LOCK_EX | LOCK_NB) == -1) {
    const int failed_with = errno;
    drop_lock();
    return OpenResult{BusStatus::LOCK_FAILED, failed_with};
  }

  const bool began = SCSerial::begin(baudrate, port.c_str());
  // begin() printf()s the baud rate and never flushes, so under a pipe that line would surface
  // much later or not at all. Flush on both paths.
  ::fflush(stdout);
  if (!began) {
    // begin() fails at open() (fd == -1) or at tcsetattr() (fd still open; end() closes it).
    // The error is 0: perror() printed the reason and may have changed errno.
    const BusStatus status = (fd != -1) ? BusStatus::TERMIOS_FAILED : BusStatus::OPEN_FAILED;
    if (fd != -1) {
      SCSerial::end();
    }
    drop_lock();
    return OpenResult{status, 0};
  }
  // Best effort, and unchecked on purpose: a descriptor that survives an exec is untidy, not
  // unsafe, and there is nothing to do about a failure here that is better than carrying on.
  ::fcntl(fd, F_SETFD, FD_CLOEXEC);
  if (::ioctl(fd, TIOCEXCL) == -1) {
    const int failed_with = errno;
    SCSerial::end();
    drop_lock();
    return OpenResult{BusStatus::EXCLUSIVE_FAILED, failed_with};
  }
  set_io_timeout_ms(io_timeout_ms);
  port_ = port;
  baudrate_ = baudrate;
  return OpenResult{BusStatus::OK, 0};
}

void ServoBus::close() noexcept
{
  // TIOCNXCL is required: TIOCEXCL belongs to the tty and outlives close() while the tty is held
  // elsewhere (a later open() got EBUSY). It needs a valid fd, so it comes before end().
  if (fd != -1) {
    ::ioctl(fd, TIOCNXCL);
    SCSerial::end();
  }
  drop_lock();
  port_.clear();
  baudrate_ = 0;
}

bool ServoBus::set_io_timeout_ms(uint32_t ms) noexcept
{
  if (ms == 0) {
    return false;   // every select() would return at once and every transaction would fail
  }
  IOTimeOut = ms;
  return true;
}

void ServoBus::drop_lock() noexcept
{
  if (lock_fd_ != -1) {
    ::flock(lock_fd_, LOCK_UN);
    ::close(lock_fd_);
    lock_fd_ = -1;
  }
}

}  // namespace waveshare_servos
