#include "servo_bus.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/select.h>
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

// The seven rates SCSerial::begin() maps (src/SCSerial.cpp:50-75). 230400 is deliberately absent:
// setBaudRate() maps it, begin() does not, and begin()'s `default:` falls back to 115200 without
// saying so.
constexpr std::array<int, 7> kMappedBaudrates = {
  9600, 19200, 38400, 57600, 115200, 500000, 1000000};

// 254 (0xfe) is the broadcast address the sync-write header itself carries (src/SCS.cpp:132) and
// 255 is the header byte (:130-131), so a record addressed to either is nonsense the servos would
// misparse. on_init rejects both at load time, which makes this a tripwire, not the primary gate.
constexpr bool is_addressable_id(uint8_t id) noexcept
{
  return id >= 1 && id <= 253;
}

// The feedback block is little endian and the byte order is hard-coded rather than routed through
// the vendored SCS2Host: the ID == -1 accessors hard-code it too
// (src/SMS_STS.cpp:136-138,157-159,178-180,243-245), so a future End = 1 must not silently change
// what this decodes (PHASE3 2.41).
constexpr int little_endian_word(const uint8_t * data, std::size_t offset) noexcept
{
  return static_cast<int>(data[offset]) | (static_cast<int>(data[offset + 1]) << 8);
}

// Position, speed and current: -(v & ~(1<<15)) on an int holding the 16-bit word
// (src/SMS_STS.cpp:146-148,168-170,254-256). 0x8000 therefore decodes to -0, i.e. 0; that is the
// accessors' own answer and PHASE3 2.43 says reproduce it, not fix it.
constexpr int signed_on_bit_fifteen(int word) noexcept
{
  return (word & (1 << 15)) != 0 ? -(word & ~(1 << 15)) : word;
}

// Load, and the mask is NOT 0x3ff (PHASE3 R4). ReadLoad is -(Load & ~(1<<10)) on an int holding
// the whole word (src/SMS_STS.cpp:188-190), so clearing bit 10 leaves bits 11..15 in the
// magnitude: 0xffff decodes to -64511, not -1023. A real servo reports 0..1000
// (include/SMS_STS.h:83) and never sets those bits, so tightening the mask would look harmless
// and would silently publish a different `load` state than Phase 2 does.
constexpr int signed_on_bit_ten(int word) noexcept
{
  return (word & (1 << 10)) != 0 ? -(word & ~(1 << 10)) : word;
}

// One reply frame is `FF FF <id> <len> <status> <15 data bytes> <~cks>`: index 2 is the id, 3 the
// length byte, 4 the status and 5..19 the block (recon/packets.md 3.1). The checksum is the
// uint8_t sum of indices 2..19 complemented, the same arithmetic as src/SCS.cpp:359-367 but
// computed independently of the library -- that independence is what makes a clean checksum a
// statement about the servos rather than a restatement of the library's own return value, and it
// is what probe 1 Q6's "0 bad checksums in 5000 transactions" actually measured.
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

// total(n, nLen) = 7 header + n*(1 + nLen) records + 1 checksum (src/SCS.cpp:130-149).
constexpr std::size_t sync_write_bytes(std::size_t records, std::size_t record_bytes) noexcept
{
  return ServoBus::sync_write_overhead_bytes + records * (record_bytes + 1);
}

}  // namespace

// Two asserts per record size, not one: the first says the chunk limit is SAFE, the second says it
// is the LARGEST safe one. Without the second, a limit of 1 would pass and the chunking would be
// silently pessimal; with it, a change to txBuf, to the record layout or to either constant fails
// the build rather than the bench (PHASE3 1.13).
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
  // No `default:` label, so adding a BusStatus without a name here is a -Wswitch warning, which
  // this package builds as part of -Wall. The fallthrough return is for a value cast in from
  // outside the enumeration.
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
  // Computed in int32_t and saturated, where the library computes `-Position[i]` in s16
  // (src/SMS_STS.cpp:59): at -32768 that negation overflows a signed short, which is undefined
  // behaviour, and the library ends up emitting magnitude 0 with the sign bit (`00 80`) on most
  // compilers. This version is defined for the whole int16_t domain and emits magnitude 32767
  // (`ff ff`) there instead. The difference is UNREACHABLE from the driver: send_commands clamps
  // goal_steps to [-32767, 32767] before the cast (src/waveshare_servos.cpp:1481) and the wheel
  // speed is clamped to +/- max_speed_counts, itself capped at 32767 (PHASE3 1.10).
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
  // assign(), not resize(): every slot must start value-initialised, because "this servo did not
  // answer" is expressed by a block that was never written and the caller reuses this vector
  // every cycle. A stale block left behind would be published as a fresh sample.
  out->assign(count, FeedbackBlock{});

  std::size_t filled = 0;
  std::size_t pos = 0;      // read cursor in the burst
  std::size_t next = 0;     // the first request slot not yet filled
  uint64_t bad_frames = 0;

  // The guard is evaluated BEFORE any byte of a frame is touched, which is the whole of this
  // walker's answer to the vendored over-read (src/SCS.cpp:355 reads up to 18 bytes past the
  // buffer, ASAN-confirmed, recon/packets.md landmine 8). Every path through the body does either
  // pos++ or pos += 21, so the loop always terminates.
  while (next < count && pos + ServoBus::sync_read_frame_bytes <= length) {
    // Gate 1, the header. A byte skipped here is not a rejected frame -- nothing was claimed --
    // so no counter moves. One byte at a time, as the library resyncs (src/SCS.cpp:344-351).
    if (!(buffer[pos] == 0xff && buffer[pos + 1] == 0xff &&
      buffer[pos + kFrameIdOffset] != 0xff))
    {
      pos++;
      continue;
    }

    // Gate 2, the responder id, and FORWARD ONLY: the search starts at the first unfilled slot,
    // so ids passed over are the servos that stayed silent and a frame whose id is behind the
    // cursor is stale, foreign or a duplicate. SCS::Read checks neither this nor the length byte
    // (src/SCS.cpp:190-199, recon/packets.md landmine 12); both cost one comparison here.
    std::size_t slot = next;
    while (slot < count && ids[slot] != buffer[pos + kFrameIdOffset]) {
      slot++;
    }

    // Each rejection resyncs by ONE byte rather than skipping the whole 21: the frame that failed
    // may not have been a frame at all, and a misaligned buffer must be able to find the real
    // header inside it. The parse then continues -- a frame is independently verifiable, so one
    // bad frame says nothing about the next, and stopping would turn one corrupted reply into
    // three lost joints (PHASE3 R3, 2.33).
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

    // Data and status out of the SAME 21 bytes, with no bus call in between: the atomicity the
    // shared SCS::Error cannot give (PHASE3 2.36, 2.49). Slots next..slot-1 are left invalid --
    // those servos did not answer.
    (*out)[slot] = decode_feedback_block(
      buffer + pos + kFrameDataOffset, buffer[pos + kFrameStatusOffset]);
    next = slot + 1;
    filled++;
    pos += ServoBus::sync_read_frame_bytes;
  }

  if (stats != nullptr) {
    // Added to, not assigned: one sync read of more than sync_read_max_ids servos is several
    // chunks and the session totals span all of them (PHASE3 2.35).
    stats->bad_frames += bad_frames;
    stats->missing_frames += count - filled;
  }
  return filled;
}

std::array<uint8_t, ServoBus::goal_position_record_bytes> ServoBus::position_record(
  uint8_t acc, int16_t goal_ticks, uint16_t goal_speed) noexcept
{
  // Byte 0 is register 41 because the record is based there: on this path the acceleration rides
  // along for free and needs no separate transaction, which is why the ACC-written-once policy of
  // PHASE3 1.21 concerns wheels only.
  //
  // The split is spelled out here rather than delegated to the vendored SCS::Host2SCS
  // (src/SCS.cpp:34-43), which is a non-static protected member and so unreachable from a builder
  // PHASE3 0.3 declares static. Delegating from the chunk loop instead would give the record
  // layout two implementations, so 1.8's decision is kept by test rather than by call:
  // the_record_builders_split_a_word_the_way_the_vendored_library_does compares these bytes
  // against Host2SCS directly, and the SyncWritePosEx A/B compares whole frames. Host2SCS is the
  // single place the `End` flag decides byte order and SMS_STS::SMS_STS() sets End = 0
  // (src/SMS_STS.cpp:12), so a flip upstream fails those cases instead of passing silently.
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
  // Sign-magnitude, bit 15 = direction (src/SMS_STS.cpp:265-268). 0 carries no sign bit and is the
  // STOP command every deactivation depends on (src/waveshare_servos.cpp:1278-1281), so it must
  // never be floored to 1 the way the position path's goal speed is (PHASE3 1.9).
  const uint16_t speed = sign_magnitude_encode(speed_ticks);
  return {static_cast<uint8_t>(speed & 0xff), static_cast<uint8_t>(speed >> 8)};
}

template<typename Goal, std::size_t RecordBytes, typename MakeRecord>
WriteResult ServoBus::write_goal_group(
  const std::vector<Goal> & goals, const std::size_t max_records_per_packet,
  const uint8_t base_register, MakeRecord make_record)
{
  WriteResult result;
  // Emptiness FIRST, before the port check: "nothing to send" is success whatever the port is
  // doing, and the park path must not log a refusal for a group that is legitimately empty.
  // Returning NOT_OPEN here instead would make a wheels-only robot report a write error on every
  // cycle after close_port() (PHASE3 1.16).
  if (goals.empty()) {
    return result;
  }
  if (!is_open()) {
    result.status = WriteStatus::NOT_OPEN;
    return result;
  }
  // All or nothing, and before a single byte is built: a partial command set would hide a driver
  // bug behind plausible motion, and each group is commanded as a unit (PHASE3 1.18).
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
    // The vendored builder, unchanged: the header, mesLen, the broadcast id and the checksum are
    // its code, which is what makes the position frame byte-identical to SyncWritePosEx's
    // (PHASE3 1.3). No pacing between chunks: measured, there is no minimum inter-packet gap at
    // 1 Mbaud -- 0 mismatches in 1500 pairs down to a 5 us gap (probe/out_p2_q5.txt).
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
  // Register 41, because the 7-byte record is based there and the acceleration is its byte 0 --
  // which is why the write-ACC-once policy of PHASE3 1.21 concerns wheels only. Chunked at 30 so
  // the unchecked 255-byte txBuf cannot overflow (1.12, 1.13).
  return write_goal_group<GoalPosition, goal_position_record_bytes>(
    goals, max_goal_positions_per_packet, SMS_STS_ACC,
    [](const GoalPosition & goal) {
      return position_record(goal.acc, goal.position, goal.speed);
    });
}

WriteResult ServoBus::write_goal_speeds(const std::vector<GoalSpeed> & goals)
{
  // Register 46, and nothing else: the per-servo genWrite of register 41 that SyncWriteSpe does
  // inside its own loop (src/SMS_STS.cpp:275) is what Phase 3 item 1 removes -- 0.567 ms per wheel
  // per cycle, measured (probe/out_p2_q1b.txt). The 2-byte record chunks at 82 (1.12).
  return write_goal_group<GoalSpeed, goal_speed_record_bytes>(
    goals, max_goal_speeds_per_packet, SMS_STS_GOAL_SPEED_L,
    [](const GoalSpeed & goal) {return speed_record(goal.speed);});
}

bool ServoBus::write_acc(uint8_t id, uint8_t acc)
{
  // The port check is a refusal like write_goal_positions's (PHASE3 1.17), but here it is also a
  // safety guard: writeByte ends in Ack(), which reaches SCSerial::readSCS and FD_SET(fd, ...)
  // (src/SCSerial.cpp:143-144). With fd == -1 that shifts by a negative count and, once glibc's
  // _FORTIFY_SOURCE fd_set check is compiled in, aborts the process outright -- reproduced. So a
  // closed bus must never reach the vendored call at all (PHASE3 1.31.15).
  if (!is_open()) {
    return false;
  }
  // NOT `!= -1`. SCS::writeByte returns SCS::Ack (src/SCS.cpp:158), and Ack returns 0 on every
  // failure path and 1 on success (src/SCS.cpp:265-295) -- it never returns -1. The -1 convention
  // belongs to the READ side (readByte, src/SCS.cpp:206-215), which is why set_mode compares
  // against it. Testing `!= -1` here would make this function always return true and the
  // unramped-wheel warning of PHASE3 1.25 unreachable (PHASE3 1.20).
  return writeByte(id, SMS_STS_ACC, acc) != 0;
}

std::size_t ServoBus::sync_read_feedback(
  const std::vector<uint8_t> & ids, std::vector<FeedbackBlock> & blocks)
{
  // Every slot is written on every call, so a failed transaction can never leave last cycle's
  // sample behind to be published as this cycle's.
  blocks.assign(ids.size(), FeedbackBlock{});
  // An empty list would put FF FF FE 04 82 38 0F ~cks on the wire and then wait a whole timeout
  // for zero bytes (PHASE3 2.20); a closed bus would reach FD_SET(-1, ...) inside readSCS and
  // abort the process (src/SCSerial.cpp:143-144, PHASE3 2.21).
  if (ids.empty() || !is_open()) {
    return 0;
  }
  std::size_t valid = 0;
  // Each chunk is an independent transaction, and a later chunk's servos are not implicated by an
  // earlier chunk's silence, so a bad chunk never stops the loop (PHASE3 2.30).
  for (std::size_t first = 0; first < ids.size(); first += sync_read_max_ids) {
    const std::size_t count = std::min(sync_read_max_ids, ids.size() - first);
    tx_ids_.assign(ids.begin() + first, ids.begin() + first + count);
    // Re-asserted every call rather than once in the constructor: rx_buf_ never resizes, so this
    // is one store, and it keeps the invariant local to the function that depends on it (2.16).
    syncReadRxBuff = rx_buf_.data();
    // EXACTLY the bytes this chunk's repliers owe, never rx_buf_.size(). readSCS returns early
    // only once it has collected syncReadRxBuffMax bytes (src/SCS.cpp:318,
    // src/SCSerial.cpp:165-169), so an oversized value burns a full timeout on every healthy
    // cycle -- +18.6 ms at a 20 ms timeout, measured (probe 1 Q5) -- and an undersized one
    // truncates the tail (PHASE3 2.15).
    const std::size_t expected = count * sync_read_frame_bytes;
    syncReadRxBuffMax = static_cast<u16>(expected);
    sync_read_stats_.transactions++;
    // syncReadPacketTx rFlushSCS()es, writes IDN + 8 request bytes and then does ONE readSCS()
    // for syncReadRxBuffMax bytes (src/SCS.cpp:299-319), and SCSerial::readSCS lets select()
    // decrement a single timeval across its whole loop -- so the chunk costs at most one
    // io_timeout_ms overall, not one per servo (measured 1.00-1.03x across 2..50 ms, probe 1 Q7).
    const int got = syncReadPacketTx(
      tx_ids_.data(), static_cast<u8>(count), feedback_first_register,
      static_cast<u8>(feedback_block_bytes));
    // Clamp to `expected` and NOT to rx_buf_.size(): readSCS was asked for exactly that many
    // bytes and can never deliver more, so anything above it is a bug, and clamping to 648 would
    // let a bogus value send the walker over stale buffer. The patched readSCS never returns
    // below 0 today (src/SCSerial.cpp:155-174), and an upstream refresh that restored `return -1`
    // would arrive here as 65535 through the u16 member, not as a negative (PHASE3 2.25).
    std::size_t len = (got < 0) ? 0u : static_cast<std::size_t>(got);
    len = std::min(len, expected);
    // A short burst is normal and benign when one servo is silent: the other frames are perfect,
    // 1200/1200 real decodes in every absent-id case on the bench (probe 1 Q5). Count it and walk
    // what arrived anyway (PHASE3 2.26).
    const bool short_burst = len != expected;
    if (short_burst) {
      sync_read_stats_.short_bursts++;
    }
    valid += parse_sync_read_burst(
      rx_buf_.data(), len, tx_ids_, &chunk_blocks_, &sync_read_stats_);
    std::copy(
      chunk_blocks_.begin(), chunk_blocks_.end(),
      blocks.begin() + static_cast<std::ptrdiff_t>(first));
    // After the walk and before the next chunk's request, and only after a short burst: on a
    // full-length burst there is nothing extra to do -- no tcflush, no sleep, no settling --
    // because the library's own rFlushSCS() at the head of every transaction is enough at a sane
    // timeout, measured at zero contamination in 500 absent-then-present pairs (PHASE3 2.27).
    if (short_burst) {
      drain_input();
    }
  }
  // Note what is NOT here: SCS::Error is neither read nor written (PHASE3 2.28) and SCSerial::Err
  // is left exactly as it was found (2.29), because the per-servo path's accessors still gate
  // their sign handling on it.
  return valid;
}

bool ServoBus::read_feedback_one(uint8_t id, FeedbackBlock & out)
{
  out = FeedbackBlock{};
  // The same refusal sync_read_feedback makes, and for the same reason: Read() ends in readSCS
  // and FD_SET(-1, ...) aborts the process (PHASE3 1.30).
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
  if (!is_open()) {
    return 0;
  }
  sync_read_stats_.drains++;
  // A DEADLINE, not a per-iteration timeout, and it deliberately does not stop at the first quiet
  // window: the frame this exists to swallow is by definition one that has not arrived yet, so
  // "the line went quiet for a millisecond" is no reason to believe it is not coming. It
  // therefore costs its whole budget on a cycle that has already failed -- worst case
  // io_timeout_ms + sync_read_drain_ms, ~7.2 ms of a 10 ms period at the 5 ms default -- which is
  // the cost the driver's drop policy exists to bound (PHASE3 2.53, 2.55).
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
  // A tripwire against an upstream refresh: SCSerial::writeSCS() writes into txBuf with no bound
  // check at all (src/SCSerial.cpp:178-190), so its size is a safety property of every packet the
  // driver builds.
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
  // We own the receive buffer for the life of the bus, and both of the vendored calls that would
  // otherwise manage it stay uncalled forever: syncReadBegin() would `new u8[]` it and
  // syncReadEnd() would free it with a scalar `delete` (src/SCS.cpp:325,331) -- mismatched, UB, a
  // leak if begin is called twice, and a double free if it were ever handed this vector's
  // storage. Sized once and never resized, so the pointer below cannot dangle and the RT path
  // allocates nothing (PHASE3 2.13-2.14).
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
  // Steps 1-3 change nothing about the tty at all, so a refusal here is indistinguishable from
  // never having been called.
  if (is_open()) {
    return OpenResult{BusStatus::ALREADY_OPEN, 0};
  }
  if (!is_supported_baudrate(baudrate)) {
    return OpenResult{BusStatus::UNSUPPORTED_BAUDRATE, 0};
  }
  if (io_timeout_ms == 0) {
    return OpenResult{BusStatus::INVALID_TIMEOUT, 0};
  }

  // Lock first, on a descriptor of this wrapper's own, for two reasons (PHASE2_SPEC 9.3).
  // begin() calls perror() on both its failure paths and glibc's perror does not restore errno,
  // so the errno a caller reads after begin() is not the one that caused the failure; this
  // descriptor's errno is. And between begin()'s open() and a TIOCEXCL set afterwards a second
  // process can open too -- both would then set the flag, and the one that loses the lock would
  // clear the winner's on its way out.
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
    // begin() returns false from two places: a failed open(), which leaves fd at -1, and a failed
    // tcsetattr(), which leaves the descriptor open and valid. Only end() ever closes it. errno is
    // reported as 0 either way: perror() has already destroyed it and printed the real reason.
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
  // The explicit TIOCNXCL is required, not cosmetic. flock is released by closing the descriptor,
  // because the lock belongs to the open file description, but TIOCEXCL belongs to the *tty* and
  // is cleared only when the tty itself is finally released. Measured on a pty whose master a test
  // still held: after a plain close(), a later open() of the same path failed with EBUSY in the
  // same process. It needs a valid fd, so it comes before end().
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
