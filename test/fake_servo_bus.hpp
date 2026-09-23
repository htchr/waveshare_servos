// A fake SMS/STS servo bus on a pseudo terminal (PHASE2_SPEC 10.1).
//
// This is a PORT of phase1_evidence/harness/fix_harness/bus.hpp, which already models a servo as a
// 256-byte register file with `absent` / `silent_feedback` flags (bus.hpp:39-48) and is already
// debugged against the vendored packet code. What is new here is the per-servo transaction
// counters, the status byte, the sync-write record capture and the RAII responder thread; what is
// deliberately dropped is the Phase 1 harness's motion model. Nothing moves on its own: a register
// changes only when a test changes it or when a packet writes it, so "the servo goes silent on
// cycle N" is the line `set_silent_feedback(3, true)` between two driver calls and the whole of
// PHASE2_SPEC 10.4 and 10.5 replays byte for byte on every machine.
//
// Packet facts the responder satisfies, all from the vendored sources:
//   - request  ff ff ID msgLen Inst MemAddr params... ~chk, total msgLen + 4, with
//     chk = (u8)~(ID + msgLen + Inst + MemAddr + sum(params))            (src/SCS.cpp:62-90)
//   - reply    ff ff ID n+2 Status params(n) ~chk, total n + 6; SCS::Read() reads exactly nLen+6
//     bytes, so a wrong total blocks until the timeout                   (src/SCS.cpp:182-185)
//   - Ping checks bBuf[2] == ID and bBuf[3] == 2                         (src/SCS.cpp:248,251)
//   - Ack runs for EVERY non-broadcast write, because Level defaults to 1 (src/SCS.cpp:14,268):
//     EnableTorque, Mode, unLockEprom, LockEprom and the driver's own register-41 write each burn
//     a full IOTimeOut when they are not acked. That last one used to be SyncWriteSpe's, made
//     inside every write() cycle; Phase 3 item 1 took it off the per-cycle path and left it at the
//     four edges of PHASE3 1.24, and SyncWriteSpe is no longer called from anywhere
//   - a syncWrite is a broadcast to 0xfe and is never acked             (src/SCS.cpp:132,268)
//   - byte order is little endian (End == 0, src/SMS_STS.cpp:12); the feedback block is registers
//     56..70 = 15 bytes, so a feedback reply is 21 bytes with len 0x11
//
// Threading: one responder thread owns the master side of the pty. Every register, flag and
// counter lives behind `mutex_`, and the setters take it, so a test may call them between two
// driver calls without a race. Never hold a `FakeServo &` from servo() across a driver call --
// use snapshot().
//
// What the Phase 6 tools need on top of that (PHASE6_SPEC D.1): a log of every request frame and
// a raw byte count (the proof of "sent nothing"), an EEPROM shadow with the three lock policies
// and a power cycle, an id write that moves the servo, the offset model behind a 128 write to
// register 40, and the failure knobs of a bus with two servos on one id, a slow EEPROM commit, a
// firmware that does not ack. Every knob defaults to off, and every side effect fires only on a
// write the driver never makes (register 5, 128 to register 40, 31-32 with the offset model on),
// so the Phase 2-5 suites see exactly the fake they were written against.
//
// What factory_reset needs on top of that (factory_reset_evidence/FACTORY_RESET_SPEC.md, section
// 1 is the bench measurement it models): the RESET instruction, a per-servo factory table, and a
// baud model in which a servo hears only the line rate its register 6 names. The driver never
// sends a RESET and the baud model is off by default, so every earlier suite is untouched again.

#ifndef FAKE_SERVO_BUS_HPP_
#define FAKE_SERVO_BUS_HPP_

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <map>
#include <mutex>
#include <ostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace waveshare_servos_test
{

// The instruction bytes of the protocol (include/INST.h), spelled here so the harness needs none
// of the vendored headers.
constexpr uint8_t kInstPing = 0x01;
constexpr uint8_t kInstRead = 0x02;
constexpr uint8_t kInstWrite = 0x03;
constexpr uint8_t kInstSyncWrite = 0x83;
constexpr uint8_t kInstSyncRead = 0x82;   // include/INST.h:23 (PHASE3 4.H1)
// Not in include/INST.h: context/motor_reset_command_email.png and the protocol manual's 1.3.7.
constexpr uint8_t kInstReset = 0x06;
constexpr uint8_t kBroadcastId = 0xfe;

// The registers the harness and its tests name (include/SMS_STS.h).
constexpr uint8_t kRegMode = 33;
constexpr uint8_t kRegTorqueEnable = 40;
constexpr uint8_t kRegAcc = 41;
constexpr uint8_t kRegGoalPosition = 42;
constexpr uint8_t kRegGoalSpeed = 46;
constexpr uint8_t kRegPresentPosition = 56;
constexpr uint8_t kRegPresentSpeed = 58;
constexpr uint8_t kRegPresentLoad = 60;
constexpr uint8_t kRegPresentVoltage = 62;
constexpr uint8_t kRegPresentTemperature = 63;
constexpr uint8_t kRegMoving = 66;
constexpr uint8_t kRegPresentCurrent = 69;
// SMS_STS::FeedBack() reads registers 56..70 in one transaction (src/SMS_STS.cpp:123).
constexpr uint8_t kFeedbackLength = 15;

// The registers the Phase 6 tools name (PHASE6_SPEC D.1; context/sts3215_memory_table.xlsx).
constexpr uint8_t kRegModelL = 3;          // "main and sub version": a little-endian word at 3-4
constexpr uint8_t kRegId = 5;
constexpr uint8_t kRegBaud = 6;
constexpr uint8_t kRegResponseLevel = 8;   // 0 = answer nothing but READ and PING
constexpr uint8_t kRegOffset = 31;         // a word at 31-32, sign-magnitude on bit 11
constexpr uint8_t kRegLock = 55;           // the EEPROM write lock; SRAM, 0 = writes persist
constexpr uint8_t kEepromLast = 39;        // EEPROM is 0..39, SRAM starts at 40
// Register 40 = 128: "current position correction is 2048" (SMS_STS::CalibrationOfs).
constexpr uint8_t kCalibrateMidpoint = 128;

// What a WRITE to EEPROM (0..39) does while register 55 reads 1. The memory table says the write
// is applied and lost at power-off (row 50), which is `volatile_when_locked` and the policy the
// tool tests run under; `drop_when_locked` is a firmware that refuses it outright. The default,
// `apply_always`, is the fake every earlier suite was written against: it ignores the lock.
enum class EepromPolicy
{
  apply_always,
  drop_when_locked,
  volatile_when_locked
};

// Which id acks a write that covers register 5. A servo may answer from the id it was addressed
// by, from the id it has just taken, or not at all; SCS::Ack calls the second a failure
// (src/SCS.cpp:279), which is why the tools never trust an ack.
enum class IdWriteAck
{
  old_id,
  new_id,
  none
};

// Two servos on one id, as far as the wire can show it. `doubled` is two replies in bit
// synchrony, back to back; `garbled` is the collision that corrupts every reply; `garbled_reads`
// is twins at different positions, whose pings agree and whose READ replies do not;
// `doubled_reads` is twins whose pings happen to collide into one clean reply while a READ reply
// comes back twice. set_twin() can limit any of them to the next n replies it acts on.
enum class TwinReply
{
  none,
  doubled,
  garbled,
  garbled_reads,
  doubled_reads
};

// One servo: a 256-byte register file, the two failure flags of bus.hpp:46-47, the status byte
// every reply carries, and the transaction counters PHASE2_SPEC 10.4 asserts against.
struct FakeServo
{
  std::array<uint8_t, 256> mem{};
  uint8_t status = 0;             // goes into the reply's error byte (bBuf[4], src/SCS.cpp:201)

  // The two flags model different failures and Phase 2 needs both: `absent` is "no servo on the
  // bus" (the configure-time gate, PHASE2_SPEC 6) and `silent_feedback` is "still on the bus, still
  // answers pings and still acks writes, but never answers the feedback read" (the runtime drop,
  // PHASE2_SPEC 10.4). One `answers` flag cannot express the second, which is the one the re-ping
  // on activate depends on.
  bool absent = false;
  bool silent_feedback = false;

  // every request this servo was addressed by, INCLUDING its slot in a broadcast sync read: a
  // sync read demands a feedback block of this servo exactly as an addressed read does, and about
  // twenty existing assertions are written in those terms (PHASE3 4.H2, 4.H11)
  int requests = 0;
  int pings = 0;                  // INST_PING requests seen
  int reads = 0;                  // INST_READ requests seen, feedback block or not
  int feedback_reads = 0;         // INST_READ of the feedback block (56..70) seen
  int writes = 0;                 // non-broadcast INST_WRITE requests seen
  int torque_enable_writes = 0;   // writes whose MemAddr is 40 (SMS_STS_TORQUE_ENABLE)
  int mode_writes = 0;            // writes whose MemAddr is 33 (SMS_STS_MODE)
  // one entry per broadcast sync-write record addressed to this servo: {MemAddr, record bytes}
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> sync_writes;

  // This id appeared in an INST_SYNC_READ id list, answered or not. It exists because every other
  // counter below is bumped AFTER the absent check, so none of them can tell "the driver did not
  // name it" from "it was named and stayed silent" -- and that distinction is the whole of the
  // driver case that proves an absent servo never enters the read group (PHASE3 3 section H).
  int sync_read_named = 0;
  int sync_reads = 0;             // INST_SYNC_READ requests whose id list names this servo
  // one entry per sync-read request naming this servo: {MemAddr, nLen} (PHASE3 4.H2)
  std::vector<std::pair<uint8_t, uint8_t>> sync_read_requests;

  // The four ways one servo's REPLY to a sync read can be wrong while the bus itself is healthy
  // (PHASE3 2.93). Each is a separate failure the wrapper's frame gate must separate: a frame the
  // checksum rejects, a frame carrying somebody else's id in this slot, a frame whose length byte
  // disagrees with its payload, and a frame that arrives after the transaction gave up. None of
  // them touches bad_checksums(), which counts inbound REQUEST frames (PHASE3 2.94).
  bool reply_checksum_corrupt = false;
  uint8_t reply_id_override = 0;  // 0 = none; otherwise the id this servo's frame claims
  int reply_length_delta = 0;     // added to the length byte, checksum repaired, same wire length
  int reply_delay_polls = 0;      // held back this many 1 ms poll windows

  // Phase 6 (PHASE6_SPEC D.1). What survives a power cycle: mem[0..39] as it was last committed
  // under the EEPROM policy. add_servo() and the test-side setters keep it in step with mem.
  std::array<uint8_t, kEepromLast + 1> eeprom{};
  int calibrations = 0;           // 128 writes to register 40 the offset model acted on

  // The Phase 6 knobs, per servo and kept HERE rather than beside the bus-level ones, so that an
  // id write carries them to the new key together with the registers and the counters.
  IdWriteAck id_write_ack = IdWriteAck::old_id;
  bool offset_model = false;
  int physical = 0;               // the offset model's shaft angle, ticks; present derives from it
  uint8_t register40_after = 0;   // what register 40 reads after a 128 the model acted on
  TwinReply twin = TwinReply::none;
  int twin_replies = -1;          // replies the twin still acts on; -1 = every one
  int silent_read_address = -1;   // a READ starting here gets no reply; -1 = none
  int silent_reads = -1;          // READs there still to go unanswered; -1 = every one
  bool garble_write_acks = false;  // a write's ack goes out with a broken checksum
  bool write_acks = true;         // false = register 8 is 0: writes apply and are never acked
  std::array<bool, 256> ignore_write{};   // acked, and that byte is not applied
  bool vanish_after_id_write = false;
  unsigned eeprom_commit_ms = 0;
  // Until then the servo is committing EEPROM: every request to it is ignored (D.1 #13).
  std::chrono::steady_clock::time_point busy_until{};
  // The outbox ticket of the ack that waits for that commit, while it may still be queued; 0 =
  // none. A request that finds the commit over and the ack still queued gets the ack (answer()).
  uint64_t commit_ack = 0;
  int pings_to_drop = 0;          // the next this-many pings are ignored

  // factory_reset (FACTORY_RESET_SPEC 1, 2). What a RESET puts back into registers 6..39: the id
  // (5) is kept, as measured, and 0..4 are the read-only version bytes, so the table's entries
  // for 0..5 are never used. All zeros unless a test sets them.
  std::array<uint8_t, kEepromLast + 1> factory{};
  bool reset_supported = true;    // false: a RESET is ignored and never acked
  std::array<bool, kEepromLast + 1> reset_skips{};   // registers a RESET leaves as they are
  bool reset_keeps_sram = false;  // true: a RESET leaves torque, goal and lock as they were
  int resets = 0;                 // RESET requests that reached this servo
};

// One request frame as the responder parsed it, whatever it was addressed to: a present servo,
// an absent or unknown id, or the broadcast id. `params` is everything between the instruction
// and the checksum, so a READ's is {address, length} and a WRITE's {address, bytes...}.
struct FrameRecord
{
  uint8_t id = 0;
  uint8_t instruction = 0;
  std::vector<uint8_t> params;
};

// One addressed (non-broadcast) INST_WRITE, in the order the frames arrived.
struct WriteRecord
{
  uint8_t id = 0;
  uint8_t address = 0;
  std::vector<uint8_t> bytes;
};

inline bool operator==(const WriteRecord & a, const WriteRecord & b)
{
  return a.id == b.id && a.address == b.address && a.bytes == b.bytes;
}

// So a failed comparison of two write lists prints "(4,5,{253})" and not a byte dump.
inline std::ostream & operator<<(std::ostream & out, const WriteRecord & write)
{
  out << "(" << static_cast<int>(write.id) << "," << static_cast<int>(write.address) << ",{";
  for (std::size_t i = 0; i < write.bytes.size(); i++) {
    out << (i == 0 ? "" : ",") << static_cast<int>(write.bytes[i]);
  }
  return out << "})";
}

// Sign-magnitude, the encoding SMS_STS uses for position, speed and current on bit 15 and for
// load on bit 10 (src/SMS_STS.cpp:146-147,168-169,188-189,254-255).
inline uint16_t sign_magnitude_encode(int value, int bit = 15)
{
  if (value < 0) {
    return static_cast<uint16_t>(static_cast<unsigned>(-value) | (1u << bit));
  }
  return static_cast<uint16_t>(value);
}

inline int sign_magnitude_decode(uint16_t raw, int bit = 15)
{
  if ((raw & (1u << bit)) != 0u) {
    return -static_cast<int>(raw & ~(1u << bit));
  }
  return static_cast<int>(raw);
}

// The order the replies to one INST_SYNC_READ come back in. The hardware answers in request
// order, deterministically -- probe 1 Q5 measured it over three different id orders, five
// transactions each -- and the wrapper's forward-only slot match depends on that; `reversed` is
// the negative control that proves a mis-ordered burst is refused rather than mis-attributed
// (PHASE3 4.H5).
enum class SyncReadReplyOrder
{
  request,
  reversed
};

class FakeBus
{
public:
  FakeBus()
  {
    if (::openpty(&master_, &slave_, nullptr, nullptr, nullptr) != 0) {
      throw std::runtime_error(std::string("openpty failed: ") + std::strerror(errno));
    }
    const char * name = ::ttyname(slave_);
    if (name == nullptr) {
      throw std::runtime_error(std::string("ttyname failed: ") + std::strerror(errno));
    }
    port_ = name;
    // Close-on-exec on both ends (PHASE6_SPEC D.1 #3): the CLI tests spawn the real tools on this
    // bus, and a child that inherited either descriptor would keep the pty open behind the test.
    if (::fcntl(master_, F_SETFD, FD_CLOEXEC) == -1 || ::fcntl(slave_, F_SETFD, FD_CLOEXEC) == -1) {
      throw std::runtime_error(std::string("fcntl(FD_CLOEXEC) failed: ") + std::strerror(errno));
    }
    // The slave descriptor stays open for the whole life of the bus. Closing it makes the master's
    // poll() report POLLHUP -- which is reported regardless of the events mask -- for every
    // interval in which no slave is open, so the poll timeout would never apply and the responder
    // would spin a core. Those intervals are real: construction until on_configure, and again
    // around every cleanup and shutdown case.
    thread_ = std::thread(&FakeBus::run, this);
  }

  ~FakeBus()
  {
    stop_.store(true);
    if (thread_.joinable()) {
      thread_.join();
    }
    if (slave_ != -1) {
      ::close(slave_);
    }
    if (master_ != -1) {
      ::close(master_);
    }
  }

  FakeBus(const FakeBus &) = delete;
  FakeBus & operator=(const FakeBus &) = delete;
  FakeBus(FakeBus &&) = delete;
  FakeBus & operator=(FakeBus &&) = delete;

  // The path a <param name="port"> must name for the driver to reach this bus.
  const std::string & port() const {return port_;}

  void add_servo(uint8_t id, uint8_t mode = 0)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    FakeServo servo;
    seed(&servo, kRegMode, mode);
    // A servo's register 5 IS its id; the tools read it back and refuse a servo whose register
    // disagrees with the id it answered at (PHASE6_SPEC D.1 #2).
    seed(&servo, kRegId, id);
    servos_[id] = servo;
  }

  // Only between driver calls, and never held across one: the responder thread writes through the
  // same reference under `mutex_`.
  FakeServo & servo(uint8_t id) {return servos_.at(id);}

  // A copy taken under the mutex: how a case reads the counters back while the harness runs.
  FakeServo snapshot(uint8_t id) const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return servos_.at(id);
  }

  // A test-side seed is the state the servo was delivered in, so an EEPROM register is committed
  // with it, and with the offset model on it is the PRESENT position the seed keeps: the model's
  // shaft angle follows (resync_physical), never the other way round.
  void set_byte(uint8_t id, uint8_t reg, uint8_t value)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    FakeServo & s = servos_.at(id);
    seed(&s, reg, value);
    resync_physical(&s);
  }

  // little endian, sign-magnitude on `bit`
  void set_word(uint8_t id, uint8_t reg, int value, int bit = 15)
  {
    const uint16_t raw = sign_magnitude_encode(value, bit);
    const std::lock_guard<std::mutex> lock(mutex_);
    FakeServo & s = servos_.at(id);
    seed(&s, reg, static_cast<uint8_t>(raw & 0xff));
    seed(&s, static_cast<uint8_t>(reg + 1), static_cast<uint8_t>(raw >> 8));
    resync_physical(&s);
  }

  // ReadLoad decodes its sign from bit 10, not bit 15 (src/SMS_STS.cpp:188-189).
  void set_load(uint8_t id, int value) {set_word(id, kRegPresentLoad, value, 10);}

  void set_position(uint8_t id, int ticks) {set_word(id, kRegPresentPosition, ticks);}

  void set_feedback(
    uint8_t id, int position, int speed, int load, uint8_t voltage, uint8_t temperature,
    uint8_t moving, int current)
  {
    set_word(id, kRegPresentPosition, position);
    set_word(id, kRegPresentSpeed, speed);
    set_load(id, load);
    set_byte(id, kRegPresentVoltage, voltage);
    set_byte(id, kRegPresentTemperature, temperature);
    set_byte(id, kRegMoving, moving);
    set_word(id, kRegPresentCurrent, current);
  }

  void set_absent(uint8_t id, bool absent)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    servos_.at(id).absent = absent;
  }

  void set_silent_feedback(uint8_t id, bool silent)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    servos_.at(id).silent_feedback = silent;
  }

  void set_status(uint8_t id, uint8_t status)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    servos_.at(id).status = status;
  }

  // a register pair read back the way the servo stores it
  int word(uint8_t id, uint8_t reg, int bit = 15) const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    const FakeServo & s = servos_.at(id);
    const uint16_t raw = static_cast<uint16_t>(
      s.mem[reg] | (s.mem[static_cast<uint8_t>(reg + 1)] << 8));
    return sign_magnitude_decode(raw, bit);
  }

  // INST_SYNC_READ frames seen, whatever id list they carried, and the newest list in request
  // order. Unlike bad_checksums(), which is a lock-free atomic, both read state the responder
  // thread writes, so both take mutex_. The id list is what proves "the dropped servo left the
  // list" and "the list is the present servos in URDF order"; a count alone cannot (PHASE3 4.H3).
  //
  // Deliberate near-collision with FakeServo::sync_read_requests: the bus-level one counts frames,
  // the per-servo one records {address, length} pairs. They never appear in the same expression.
  uint64_t sync_read_requests() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return sync_read_requests_;
  }

  std::vector<uint8_t> last_sync_read_ids() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return last_sync_read_ids_;
  }

  // Firmware that parses 0x82 and answers nothing: the request is still counted, bus-level and per
  // servo, no reply is emitted, and the caller pays one io_timeout_ms. This is the knob the
  // per-servo FeedBack() fallback is untestable without (PHASE3 4.H4).
  void set_sync_read_supported(bool supported)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    sync_read_supported_ = supported;
  }

  void set_sync_read_reply_order(SyncReadReplyOrder order)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    reply_order_ = order;
  }

  // Drops `bytes` off the tail of the burst AFTER it is assembled, modelling a reply cut mid
  // frame. Distinct from an absent servo, which loses whole frames, and from a bad checksum, where
  // the frame is present and its contents wrong (PHASE3 4.H6).
  void set_sync_read_truncate_bytes(size_t bytes)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    sync_read_truncate_bytes_ = bytes;
  }

  // One id at a time, 0 for none: the bus-level spelling of set_reply_checksum_corrupt, because
  // the burst-level cases name the servo they corrupt and nothing else (PHASE3 4.H7).
  void set_sync_read_bad_checksum(uint8_t id)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    for (auto & entry : servos_) {
      entry.second.reply_checksum_corrupt = (entry.first == id && id != 0);
    }
  }

  // Delays the WHOLE burst. Implemented with an outbox and never with a sleep: answer() runs with
  // mutex_ held, so sleeping there would block every setter and stall wait_quiet() (PHASE3 4.H8).
  // At 0 the burst is written inline exactly as it was, so every existing case is unchanged.
  void set_sync_read_delay_ms(unsigned ms)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    sync_read_delay_ms_ = ms;
  }

  void set_reply_checksum_corrupt(uint8_t id, bool corrupt)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    servos_.at(id).reply_checksum_corrupt = corrupt;
  }

  void set_reply_id_override(uint8_t id, uint8_t other)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    servos_.at(id).reply_id_override = other;
  }

  void set_reply_length_delta(uint8_t id, int delta)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    servos_.at(id).reply_length_delta = delta;
  }

  // Holds back ONE servo's frame while the rest of the burst goes out on time -- the late reply
  // that arrives during the next transaction's read window, which is the contamination probe 3
  // Q4/Q6 measured and the whole reason the error path drains (PHASE3 2.93, 2.115).
  void set_reply_delay_polls(uint8_t id, int polls)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    servos_.at(id).reply_delay_polls = polls;
  }

  // Frames whose checksum did not verify. A non-zero count means the wire itself misbehaved and
  // every other assertion in the case is worthless, so cases that decode packets assert it is 0.
  uint64_t bad_checksums() const {return bad_checksums_.load();}

  // Times wait_quiet() hit its safety deadline instead of observing silence. A non-zero count
  // means the harness lost the guarantee the sync-write assertions rest on, so it is asserted in
  // PtyFixture::TearDown rather than left to surface as a stale or missing record downstream.
  uint64_t quiet_timeouts() const {return quiet_timeouts_.load();}

  // Returns once the responder has seen the bus fall silent, so a packet that is still in flight
  // cannot be missed. Every transaction the driver makes is synchronous -- it waits for the reply
  // or for its timeout -- with one exception: a sync write is a broadcast and is never acked, so
  // when write() returns those bytes may still be in the kernel's queue. Decoding sync_writes
  // right then is a race, and it loses exactly the last record.
  //
  // The wait is on the responder thread, not on the clock: it counts poll() calls that TIMED OUT
  // with nothing queued and nothing half-parsed. A timeout proves the queue was empty for that
  // whole window, and waiting for TWO of them proves the second window began after this call did,
  // hence after the bytes were written -- had they still been queued, that poll would have
  // returned POLLIN instead of timing out. The deadline is a safety net for a machine that has
  // descheduled the responder for a second; it never shortens a successful wait. Giving up is
  // counted, never swallowed: quiet_timeouts() is what tells the next reader that a downstream
  // "last_speed(4) = INT_MIN" is a lost race and not a driver bug.
  void wait_quiet() const
  {
    const uint64_t target = idle_polls_.load() + 2;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (idle_polls_.load() < target) {
      if (std::chrono::steady_clock::now() > deadline) {
        quiet_timeouts_.fetch_add(1);
        return;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }

  // ---- Phase 6 (PHASE6_SPEC D.1) ----

  // For the close-on-exec self-test only; nothing else may touch the pty behind the responder.
  int master_fd() const {return master_;}
  int slave_fd() const {return slave_;}

  // Every request frame answer() was handed since construction or the last clear_frames(), in
  // arrival order, whatever it was addressed to. A copy under the mutex.
  std::vector<FrameRecord> frames() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return frames_;
  }

  // Starts a new observation window: the frame log AND the raw byte count, so "sent nothing
  // after this point" is one clear_frames() and one bytes_received() == 0.
  void clear_frames()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    frames_.clear();
    bytes_received_.store(0);
  }

  std::size_t frames_received() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return frames_.size();
  }

  // INST_PING frames per id, absent and unknown ids included: the coverage gate of a scan.
  std::map<uint8_t, int> ping_counts() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    std::map<uint8_t, int> counts;
    for (const FrameRecord & frame : frames_) {
      if (frame.instruction == kInstPing) {
        counts[frame.id]++;
      }
    }
    return counts;
  }

  // Every addressed INST_WRITE in order, answered or not: compared as a WHOLE list, this is what
  // makes "wrote nothing else" provable.
  std::vector<WriteRecord> writes() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    std::vector<WriteRecord> list;
    for (const FrameRecord & frame : frames_) {
      if (frame.instruction == kInstWrite && frame.id != kBroadcastId && !frame.params.empty()) {
        list.push_back(
          WriteRecord{frame.id, frame.params[0],
            std::vector<uint8_t>(frame.params.begin() + 1, frame.params.end())});
      }
    }
    return list;
  }

  // Every raw byte read off the master, counted before consume() can drop garbage or a
  // bad-checksum frame: "sent nothing" is this being 0, never an empty frame log (D.1 #4).
  uint64_t bytes_received() const {return bytes_received_.load();}

  // id writes that would have put two servos on one key and were refused
  uint64_t id_collisions() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return id_collisions_;
  }

  void set_id_write_ack(uint8_t id, IdWriteAck ack)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    servos_.at(id).id_write_ack = ack;
  }

  void set_eeprom_policy(EepromPolicy policy)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    eeprom_policy_ = policy;
  }

  // what register 55 reads after power_cycle()
  void set_power_up_lock(uint8_t value)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    power_up_lock_ = value;
  }

  // Every servo: EEPROM back from the shadow, torque off, the lock at its power-up value, any
  // commit abandoned, and the map re-keyed by register 5 -- so an id write that was never
  // committed moves the servo back.
  void power_cycle()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    std::vector<uint8_t> keys;
    for (auto & entry : servos_) {
      FakeServo & s = entry.second;
      for (std::size_t reg = 0; reg <= kEepromLast; reg++) {
        s.mem[reg] = s.eeprom[reg];
      }
      s.mem[kRegTorqueEnable] = 0;
      s.mem[kRegLock] = power_up_lock_;
      s.busy_until = {};
      s.commit_ack = 0;
      present_from_physical(&s);
      keys.push_back(entry.first);
    }
    for (const uint8_t key : keys) {
      rekey(key, servos_.at(key).mem[kRegId]);
    }
  }

  // Off: register 40 = 128 is stored like any byte ("calibration unsupported"). On: the servo
  // keeps a shaft angle, present = wrap4096(physical - offset), and 128 re-centres it (D.1 #7).
  void set_offset_model(uint8_t id, bool on)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    FakeServo & s = servos_.at(id);
    s.offset_model = on;
    resync_physical(&s);
  }

  void set_register40_after(uint8_t id, uint8_t value)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    servos_.at(id).register40_after = value;
  }

  // `replies` > 0: the twin acts on the next that-many replies it would change (a ping is not one
  // for the *_reads kinds) and is gone after them -- the twin heard once among clean replies.
  void set_twin(uint8_t id, TwinReply twin, int replies = -1)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    FakeServo & s = servos_.at(id);
    s.twin = twin;
    s.twin_replies = replies;
  }

  // `count` > 0: only the next that-many READs starting at `address` go unanswered.
  void set_silent_read(uint8_t id, int address, int count = -1)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    FakeServo & s = servos_.at(id);
    s.silent_read_address = address;
    s.silent_reads = count;
  }

  // A collision on the ack alone: the write applies, its ack arrives with a broken checksum.
  void set_garble_write_acks(uint8_t id, bool garble)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    servos_.at(id).garble_write_acks = garble;
  }

  void set_write_acks(uint8_t id, bool acks)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    servos_.at(id).write_acks = acks;
  }

  void set_ignore_write(uint8_t id, uint8_t address, bool ignore)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    servos_.at(id).ignore_write[address] = ignore;
  }

  void set_vanish_after_id_write(uint8_t id)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    servos_.at(id).vanish_after_id_write = true;
  }

  void set_eeprom_commit_ms(uint8_t id, unsigned ms)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    servos_.at(id).eeprom_commit_ms = ms;
  }

  // reply_id_override, reply_length_delta and reply_checksum_corrupt act on sync-read frames
  // always, and on addressed replies (ping, READ, write ack) only while this is on
  void set_faults_apply_to_addressed(bool apply)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    faults_apply_to_addressed_ = apply;
  }

  void drop_pings(uint8_t id, int count)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    servos_.at(id).pings_to_drop = count;
  }

  // ---- factory_reset (FACTORY_RESET_SPEC 2, "Fake bus") ----

  // The ids every RESET frame was addressed to, in arrival order, answered or not.
  std::vector<uint8_t> resets() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    std::vector<uint8_t> ids;
    for (const FrameRecord & frame : frames_) {
      if (frame.instruction == kInstReset) {
        ids.push_back(frame.id);
      }
    }
    return ids;
  }

  // One entry of the table a RESET restores (registers 6..39 only; see FakeServo::factory).
  void set_factory_byte(uint8_t id, uint8_t reg, uint8_t value)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    servos_.at(id).factory.at(reg) = value;
  }

  // little endian, sign-magnitude on `bit`, like set_word
  void set_factory_word(uint8_t id, uint8_t reg, int value, int bit = 15)
  {
    const uint16_t raw = sign_magnitude_encode(value, bit);
    const std::lock_guard<std::mutex> lock(mutex_);
    FakeServo & s = servos_.at(id);
    s.factory.at(reg) = static_cast<uint8_t>(raw & 0xff);
    s.factory.at(reg + 1u) = static_cast<uint8_t>(raw >> 8);
  }

  // The factory table made equal to the servo's EEPROM as it stands, apart from `changes`: the
  // bench servo whose registers are all factory but a few.
  void set_factory_from_eeprom(uint8_t id, const std::map<uint8_t, uint8_t> & changes = {})
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    FakeServo & s = servos_.at(id);
    for (std::size_t reg = 0; reg <= kEepromLast; reg++) {
      s.factory[reg] = s.mem[reg];
    }
    for (const auto & change : changes) {
      s.factory.at(change.first) = change.second;
    }
  }

  void set_reset_supported(uint8_t id, bool supported)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    servos_.at(id).reset_supported = supported;
  }

  void set_reset_skip(uint8_t id, uint8_t reg, bool skip)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    servos_.at(id).reset_skips.at(reg) = skip;
  }

  // A firmware whose RESET does not re-initialise SRAM (M4 and M5 measured that the ST3025's does).
  void set_reset_keeps_sram(uint8_t id, bool keeps)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    servos_.at(id).reset_keeps_sram = keeps;
  }

  // Off by default. On: a servo hears an addressed request only when the line runs at the rate its
  // register 6 names (0..7 = 1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400); any
  // other request is noise to it and goes unanswered. The line rate is the pty's own termios
  // speed, which ServoBus sets through the slave. Replies are never filtered: an ack goes out at
  // the rate its request came at, even when that request moved the servo to another rate
  // (FACTORY_RESET_SPEC M3). Sync reads and writes are the driver's, which never changes a rate,
  // and are not modelled.
  void set_baud_model(bool on)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    baud_model_ = on;
  }

private:
  // Short enough that wait_quiet costs a couple of milliseconds, and irrelevant to latency:
  // poll() returns the moment a byte arrives, whatever the timeout says.
  static constexpr int kPollTimeoutMs = 1;

  void run()
  {
    while (!stop_.load()) {
      // At the TOP of the iteration, so all three of the paths below reach it: the poll timeout,
      // the POLLHUP sleep and the byte-consuming path all continue back to here. Draining only
      // after consume() would mean a delayed reply reached the wire when the NEXT request arrived,
      // which is precisely the cycle PHASE3 2.115 must not see it in.
      const bool emitted = flush_outbox();
      struct pollfd waiting = {master_, POLLIN, 0};
      const int ready = ::poll(&waiting, 1, kPollTimeoutMs);
      if (ready == 0) {
        // Nothing queued for a whole poll window. A half-parsed frame is not silence, and neither
        // is a reply still sitting in the outbox or one written in this very iteration -- either
        // would let wait_quiet() declare quiet with a reply still to come (PHASE3 4.H8).
        if (pending_.empty() && outbox_.empty() && !emitted) {
          idle_polls_.fetch_add(1);
        }
        continue;
      }
      if (ready < 0) {
        continue;
      }
      // With no slave open POLLHUP is reported whatever the events mask asks for, so poll() would
      // return at once forever. Sleep instead of spinning a core (PHASE2_SPEC 10.1).
      if ((waiting.revents & POLLHUP) != 0 && (waiting.revents & POLLIN) == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      std::array<uint8_t, 256> chunk{};
      const ssize_t n = ::read(master_, chunk.data(), chunk.size());
      if (n > 0) {
        // Here and not in consume(), which throws garbage and mis-summed frames away unseen.
        bytes_received_.fetch_add(static_cast<uint64_t>(n));
        pending_.insert(pending_.end(), chunk.begin(), chunk.begin() + n);
        consume();
      }
    }
  }

  void consume()
  {
    while (pending_.size() >= 4) {
      if (pending_[0] != 0xff || pending_[1] != 0xff) {
        pending_.pop_front();
        continue;
      }
      // msgLen counts Inst through the checksum, so it is never below 2. A smaller byte is not a
      // frame header at all: resync instead of handing answer() a vector too short for frame[4].
      if (pending_[3] < 2) {
        pending_.pop_front();
        continue;
      }
      const size_t total = 4 + static_cast<size_t>(pending_[3]);
      if (pending_.size() < total) {
        return;
      }
      const std::vector<uint8_t> frame(pending_.begin(), pending_.begin() + total);
      pending_.erase(pending_.begin(), pending_.begin() + total);
      uint8_t sum = 0;
      for (size_t i = 2; i + 1 < frame.size(); i++) {
        sum = static_cast<uint8_t>(sum + frame[i]);
      }
      if (static_cast<uint8_t>(~sum) != frame.back()) {
        bad_checksums_.fetch_add(1);
        continue;
      }
      answer(frame);
    }
  }

  void answer(const std::vector<uint8_t> & frame)
  {
    const uint8_t id = frame[2];
    const uint8_t instruction = frame[4];
    const std::vector<uint8_t> params(frame.begin() + 5, frame.end() - 1);
    const std::lock_guard<std::mutex> lock(mutex_);
    // Before every return below: the log proves a request was SENT, answered or not, and a ping
    // to an empty id is exactly what a scan's coverage gate counts (PHASE6_SPEC D.1 #4).
    frames_.push_back(FrameRecord{id, instruction, params});
    if (id == kBroadcastId) {
      if (instruction == kInstSyncWrite) {
        record_sync_write(params);
      } else if (instruction == kInstSyncRead) {
        answer_sync_read(params);
      }
      return;                         // a broadcast is never acked (src/SCS.cpp:268)
    }
    const auto it = servos_.find(id);
    if (it == servos_.end() || it->second.absent) {
      return;                         // nothing on the bus: silence, and the caller pays a timeout
    }
    FakeServo & s = it->second;
    // A request at another rate than the servo's is noise to it (set_baud_model).
    if (baud_model_ && line_rate() != rate_of_register(s.mem[kRegBaud])) {
      return;
    }
    // Still committing EEPROM: the request is ignored as though the servo were absent. A
    // timestamp and never a sleep -- answer() runs with mutex_ held (D.1 #13).
    if (std::chrono::steady_clock::now() < s.busy_until) {
      return;
    }
    // The commit is over but its ack is still queued -- its due time passed while the responder
    // slept in poll(). A servo sends that ack the moment its commit ends, before it hears anything
    // else, so the ack goes out now and this request, which found it finishing, goes unheard.
    // Answering first would put a reply AHEAD of the ack in one read window, an order no servo
    // produces; the Phase 6 tool tests caught exactly that, at commit times one ping period apart.
    if (s.commit_ack != 0) {
      const uint64_t ticket = s.commit_ack;
      s.commit_ack = 0;
      if (emit_queued(ticket)) {
        return;
      }
    }
    if (instruction == kInstPing && s.pings_to_drop > 0) {
      s.pings_to_drop--;
      return;                         // a ping lost on the wire; the servo never saw it
    }
    s.requests++;
    if (instruction == kInstPing) {
      s.pings++;
      reply(s, id, {}, false);
      return;
    }
    if (instruction == kInstRead && params.size() >= 2) {
      s.reads++;
      const uint8_t address = params[0];
      const uint8_t length = params[1];
      if (address == kRegPresentPosition && length == kFeedbackLength) {
        s.feedback_reads++;
        if (s.silent_feedback) {
          return;                     // on the bus, answering pings, but never the feedback read
        }
      }
      if (static_cast<int>(address) == s.silent_read_address && s.silent_reads != 0) {
        if (s.silent_reads > 0) {
          s.silent_reads--;
        }
        return;
      }
      std::vector<uint8_t> payload;
      payload.reserve(length);
      for (size_t i = 0; i < length; i++) {
        payload.push_back(s.mem[(address + i) & 0xff]);
      }
      reply(s, id, payload, true);
      return;
    }
    if (instruction == kInstWrite && !params.empty()) {
      s.writes++;
      const uint8_t address = params[0];
      if (address == kRegTorqueEnable) {
        s.torque_enable_writes++;
      }
      if (address == kRegMode) {
        s.mode_writes++;
      }
      answer_write(id, s, params);
      return;
    }
    if (instruction == kInstReset) {
      s.resets++;
      answer_reset(id, s);
    }
  }

  // A RESET, FACTORY_RESET_SPEC 1 (measured on the ST3025): registers 6..39 back to the factory
  // table, committed to the shadow whatever the lock says -- a flash write of its own, not a WRITE
  // under the EEPROM policy -- with register 5 kept; then torque off, goal 0 and the lock closed;
  // then the ack from the addressed id, as late as the commit when a byte changed (25 ms on the
  // bench, 0.8 ms when nothing did).
  void answer_reset(uint8_t id, FakeServo & s)
  {
    if (!s.reset_supported) {
      return;                         // an instruction this firmware does not know: no reply
    }
    bool changed = false;
    for (std::size_t reg = kRegBaud; reg <= kEepromLast; reg++) {
      if (s.reset_skips[reg]) {
        continue;
      }
      changed = changed || s.mem[reg] != s.factory[reg] || s.eeprom[reg] != s.factory[reg];
      s.mem[reg] = s.factory[reg];
      s.eeprom[reg] = s.factory[reg];
    }
    if (!s.reset_keeps_sram) {
      s.mem[kRegTorqueEnable] = 0;
      s.mem[kRegGoalPosition] = 0;
      s.mem[kRegGoalPosition + 1] = 0;
      s.mem[kRegLock] = 1;
    }
    present_from_physical(&s);
    const unsigned commit_ms = changed ? s.eeprom_commit_ms : 0;
    if (commit_ms > 0) {
      s.busy_until = std::chrono::steady_clock::now() + std::chrono::milliseconds(commit_ms);
    }
    if (s.write_acks) {
      s.commit_ack = reply(
        s, id, {}, false,
        commit_ms > 0 ? s.busy_until : std::chrono::steady_clock::time_point{}, true);
    }
  }

  // The rate register 6 names, or -1 for a value outside 0..7.
  static int rate_of_register(uint8_t value)
  {
    constexpr std::array<int, 8> kRates = {
      1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400};
    return value < kRates.size() ? kRates[value] : -1;
  }

  // The rate the pty line runs at, as ServoBus set it through the slave; -1 for any other speed.
  int line_rate() const
  {
    struct termios settings{};
    if (::tcgetattr(slave_, &settings) != 0) {
      return -1;
    }
    switch (::cfgetospeed(&settings)) {
      case B1000000:
        return 1000000;
      case B500000:
        return 500000;
      case B115200:
        return 115200;
      case B57600:
        return 57600;
      case B38400:
        return 38400;
      case B19200:
        return 19200;
      case B9600:
        return 9600;
      default:
        return -1;
    }
  }

  // An addressed INST_WRITE, PHASE6_SPEC D.1 #5-#13. The bytes first, each under the EEPROM
  // policy; then the side effects of the registers the driver never writes (31-32 and 128 at 40
  // with the offset model on, 5 always); then the ack, from the id the knobs name and as late as
  // the commit takes. With every knob at its default this is exactly the old loop and send().
  void answer_write(uint8_t id, FakeServo & s, const std::vector<uint8_t> & params)
  {
    const uint8_t address = params[0];
    // the lock as the write found it: one frame is one decision, whatever bytes it carries
    const bool locked = s.mem[kRegLock] != 0;
    bool eeprom_applied = false;
    bool offset_written = false;
    bool id_written = false;          // the frame covered register 5, applied or not
    bool id_applied = false;
    bool calibrate = false;
    for (size_t i = 1; i < params.size(); i++) {
      const uint8_t reg = static_cast<uint8_t>((address + i - 1) & 0xff);
      id_written = id_written || reg == kRegId;
      if (s.ignore_write[reg]) {
        continue;
      }
      if (reg == kRegTorqueEnable && params[i] == kCalibrateMidpoint && s.offset_model) {
        calibrate = true;             // 128 is a command here, not a value to store
        continue;
      }
      if (store(&s, reg, params[i], locked)) {
        eeprom_applied = eeprom_applied || reg <= kEepromLast;
        offset_written = offset_written || reg == kRegOffset || reg == kRegOffset + 1;
        id_applied = id_applied || reg == kRegId;
      }
    }
    if (calibrate) {
      // "current position correction is 2048": the offset that puts present at 2048, written
      // to 31-32 as an EEPROM write under the policy like any other.
      const int wanted = s.physical - 2048;
      const int offset = wanted > 2047 ? 2047 : (wanted < -2047 ? -2047 : wanted);
      const uint16_t raw = sign_magnitude_encode(offset, 11);
      const std::array<uint8_t, 2> bytes = {
        static_cast<uint8_t>(raw & 0xff), static_cast<uint8_t>(raw >> 8)};
      for (size_t k = 0; k < bytes.size(); k++) {
        const uint8_t reg = static_cast<uint8_t>(kRegOffset + k);
        if (!s.ignore_write[reg] && store(&s, reg, bytes[k], locked)) {
          eeprom_applied = true;
          offset_written = true;
        }
      }
      s.mem[kRegTorqueEnable] = s.register40_after;
      s.calibrations++;
    }
    if (offset_written) {
      present_from_physical(&s);
    }
    const unsigned commit_ms = eeprom_applied ? s.eeprom_commit_ms : 0;
    if (commit_ms > 0) {
      s.busy_until = std::chrono::steady_clock::now() + std::chrono::milliseconds(commit_ms);
    }
    // The id the servo answers to from now on: register 5 if this frame changed it, else the key
    // it was addressed at. `s` stays valid across rekey(): a node extract moves the element
    // without copying it.
    const uint8_t now_id = id_applied ? s.mem[kRegId] : id;
    const bool vanish = id_written && s.vanish_after_id_write;
    bool ack = s.write_acks && !vanish;
    uint8_t ack_from = id;
    if (id_written) {
      ack = ack && s.id_write_ack != IdWriteAck::none;
      ack_from = (s.id_write_ack == IdWriteAck::new_id) ? now_id : id;
      rekey(id, now_id);
    }
    if (vanish) {
      s.absent = true;                // gone under both ids (context/motor_reset_command_email.png)
    }
    if (ack) {
      // Due at the very time point the commit ends, so a request that finds the servo free also
      // finds its ack due (answer()).
      s.commit_ack = reply(
        s, ack_from, {}, false,
        commit_ms > 0 ? s.busy_until : std::chrono::steady_clock::time_point{}, true);
    }
  }

  // One byte under the EEPROM policy. SRAM always takes it. Returns whether mem changed hands,
  // i.e. whether the servo acts on it now; the shadow says whether it outlives a power cycle.
  bool store(FakeServo * s, uint8_t reg, uint8_t value, bool locked) const
  {
    if (reg > kEepromLast) {
      s->mem[reg] = value;
      return true;
    }
    if (locked && eeprom_policy_ == EepromPolicy::drop_when_locked) {
      return false;
    }
    s->mem[reg] = value;
    if (!locked || eeprom_policy_ == EepromPolicy::apply_always) {
      s->eeprom[reg] = value;
    }
    return true;
  }

  // A test-side seed: mem, and the shadow with it for an EEPROM register.
  static void seed(FakeServo * s, uint8_t reg, uint8_t value)
  {
    s->mem[reg] = value;
    if (reg <= kEepromLast) {
      s->eeprom[reg] = value;
    }
  }

  static int offset_of(const FakeServo & s)
  {
    return sign_magnitude_decode(
      static_cast<uint16_t>(s.mem[kRegOffset] | (s.mem[kRegOffset + 1] << 8)), 11);
  }

  static int present_of(const FakeServo & s)
  {
    return sign_magnitude_decode(
      static_cast<uint16_t>(
        s.mem[kRegPresentPosition] | (s.mem[kRegPresentPosition + 1] << 8)));
  }

  // After a test-side seed: keep the present position the test wrote, move the shaft to match.
  static void resync_physical(FakeServo * s)
  {
    if (s->offset_model) {
      s->physical = present_of(*s) + offset_of(*s);
    }
  }

  // After the bus moved the offset: the shaft stays where it is and present follows it.
  static void present_from_physical(FakeServo * s)
  {
    if (!s->offset_model) {
      return;
    }
    const int present = ((s->physical - offset_of(*s)) % 4096 + 4096) % 4096;
    s->mem[kRegPresentPosition] = static_cast<uint8_t>(present & 0xff);
    s->mem[kRegPresentPosition + 1] = static_cast<uint8_t>(present >> 8);
  }

  // Move the servo keyed `from` to `to`, counters and knobs included. Two servos cannot share a
  // key, so a move onto a taken one is refused and counted, and the register keeps the write.
  void rekey(uint8_t from, uint8_t to)
  {
    if (from == to) {
      return;
    }
    if (servos_.count(to) != 0) {
      id_collisions_++;
      return;
    }
    auto node = servos_.extract(from);
    node.key() = to;
    servos_.insert(std::move(node));
  }

  // ff ff fe mesLen 83 MemAddr nLen {ID data(nLen)}... ~chk (src/SCS.cpp:124-151)
  void record_sync_write(const std::vector<uint8_t> & params)
  {
    if (params.size() < 2) {
      return;
    }
    const uint8_t address = params[0];
    const size_t length = params[1];
    for (size_t k = 2; k + length < params.size(); k += 1 + length) {
      const uint8_t id = params[k];
      const auto it = servos_.find(id);
      if (it == servos_.end() || it->second.absent) {
        continue;
      }
      FakeServo & s = it->second;
      std::vector<uint8_t> record(params.begin() + k + 1, params.begin() + k + 1 + length);
      for (size_t i = 0; i < length; i++) {
        s.mem[(address + i) & 0xff] = record[i];
      }
      s.sync_writes.emplace_back(address, std::move(record));
    }
  }

  // ff ff fe (IDN+4) 82 MemAddr nLen ID... ~chk (src/SCS.cpp:297-320), so params is
  // {MemAddr, nLen, ID...}. consume() needs no change at all: total = 4 + pending_[3] = IDN + 8
  // and its checksum over frame[2..n-2] is byte for byte the vendored one (PHASE3 4.H9).
  //
  // The whole burst is ASSEMBLED before a byte of it is written, because all three burst-level
  // knobs act on the assembled thing: the frame order is reversed, the tail is cut, one frame is
  // mis-summed. The replies are ordinary status packets emitted back to back in ID-LIST ORDER --
  // what the hardware does (probe 1 Q5), and what the wrapper's forward-only slot match depends
  // on (PHASE3 4.H10).
  void answer_sync_read(const std::vector<uint8_t> & params)
  {
    if (params.size() < 2) {
      return;
    }
    const uint8_t address = params[0];
    const uint8_t length = params[1];
    sync_read_requests_++;
    last_sync_read_ids_.assign(params.begin() + 2, params.end());

    std::vector<std::vector<uint8_t>> frames;
    for (size_t k = 2; k < params.size(); k++) {
      const uint8_t id = params[k];
      const auto it = servos_.find(id);
      if (it != servos_.end()) {
        // Before the absent check, and the only counter that is: it records that the driver PUT
        // this id in the list, which every counter below cannot (PHASE3 3 section H amendment).
        it->second.sync_read_named++;
      }
      // An unknown or absent servo contributes NO BYTES AT ALL -- not a short frame, not an error
      // frame. None of the three skips below comes for free: the broadcast branch returns before
      // answer()'s own servos_.find and requests++ ever run (PHASE3 2.91).
      if (it == servos_.end() || it->second.absent) {
        continue;
      }
      FakeServo & s = it->second;
      // A sync read bumps requests and feedback_reads as well as sync_reads, deliberately: about
      // twenty existing assertions read "a feedback block was demanded of this servo this cycle",
      // which is exactly as true of a sync read as of an addressed one, and sync_reads then says
      // which instruction carried it (PHASE3 4.H11).
      s.requests++;
      s.sync_reads++;
      s.sync_read_requests.emplace_back(address, length);
      if (address == kRegPresentPosition && length == kFeedbackLength) {
        s.feedback_reads++;
        if (s.silent_feedback) {
          continue;                   // on the bus, counted, and never answers the feedback block
        }
      }
      if (!sync_read_supported_) {
        continue;                     // parses 0x82, answers nothing (PHASE3 4.H4)
      }
      std::vector<uint8_t> payload;
      payload.reserve(length);
      for (size_t i = 0; i < length; i++) {
        payload.push_back(s.mem[(address + i) & 0xff]);
      }
      std::vector<uint8_t> frame = frame_bytes(
        s.reply_id_override != 0 ? s.reply_id_override : id, s.status, payload);
      if (s.reply_length_delta != 0) {
        // The length byte alone. The frame keeps its size on the wire and its checksum is
        // repaired, so the only thing under test is the walker's length gate (PHASE3 2.109).
        frame[3] = static_cast<uint8_t>(static_cast<int>(frame[3]) + s.reply_length_delta);
        repair_checksum(&frame);
      }
      if (s.reply_checksum_corrupt) {
        // One DATA byte flipped after the checksum was computed, so the frame is well formed and
        // mis-summed (PHASE3 4.H7). One bit and not 0xff: a payload byte turned into 0xff could
        // pair with its neighbour into a false header and cost the walker a second bad frame.
        frame[5] = static_cast<uint8_t>(frame[5] ^ 0x01);
      }
      if (s.reply_delay_polls > 0) {
        outbox_.push_back(
          Outgoing{std::chrono::steady_clock::now() +
            std::chrono::milliseconds(s.reply_delay_polls * kPollTimeoutMs), std::move(frame)});
        continue;
      }
      frames.push_back(std::move(frame));
    }

    std::vector<uint8_t> burst;
    for (size_t k = 0; k < frames.size(); k++) {
      // Indexed rather than reversed in place, so the harness needs no <algorithm> it did not
      // already have (PHASE3 4.H12).
      const std::vector<uint8_t> & frame =
        (reply_order_ == SyncReadReplyOrder::reversed) ? frames[frames.size() - 1 - k] : frames[k];
      burst.insert(burst.end(), frame.begin(), frame.end());
    }
    burst.resize(
      sync_read_truncate_bytes_ >= burst.size() ? 0 : burst.size() - sync_read_truncate_bytes_);
    if (burst.empty()) {
      return;
    }
    if (sync_read_delay_ms_ > 0) {
      outbox_.push_back(
        Outgoing{std::chrono::steady_clock::now() + std::chrono::milliseconds(sync_read_delay_ms_),
          std::move(burst)});
      return;
    }
    emit(burst);
  }

  // Recompute a frame's trailing checksum over bytes 2..n-2, the arithmetic src/SCS.cpp:359-367
  // verifies. Used only by the knobs that patch a frame after frame_bytes() summed it.
  static void repair_checksum(std::vector<uint8_t> * frame)
  {
    uint8_t sum = 0;
    for (size_t i = 2; i + 1 < frame->size(); i++) {
      sum = static_cast<uint8_t>(sum + (*frame)[i]);
    }
    frame->back() = static_cast<uint8_t>(~sum);
  }

  // Everything in the outbox that has come due, in insertion order. Touched only by the responder
  // thread -- run() -> consume() -> answer() -> answer_sync_read() is all one thread -- so it
  // needs no lock of its own (PHASE3 4.H8).
  bool flush_outbox()
  {
    const auto now = std::chrono::steady_clock::now();
    bool emitted = false;
    size_t kept = 0;
    for (size_t i = 0; i < outbox_.size(); i++) {
      if (outbox_[i].due <= now) {
        emit(outbox_[i].bytes);
        emitted = true;
      } else if (kept != i) {
        // Guarded, because kept == i is the ordinary case and self-move-assigning a std::vector
        // leaves it valid but unspecified -- in practice empty, which silently turned a held-back
        // 84-byte burst into nothing at all the first time this was written.
        outbox_[kept++] = std::move(outbox_[i]);
      } else {
        kept++;
      }
    }
    outbox_.resize(kept);
    return emitted;
  }

  // The one queued entry carrying `ticket`, sent now whatever its due time. Responder thread only,
  // like the rest of the outbox. Returns false when it already went out.
  bool emit_queued(uint64_t ticket)
  {
    for (size_t i = 0; i < outbox_.size(); i++) {
      if (outbox_[i].ticket == ticket) {
        emit(outbox_[i].bytes);
        outbox_.erase(outbox_.begin() + static_cast<std::ptrdiff_t>(i));
        return true;
      }
    }
    return false;
  }

  // The framing send() has always done: ff ff ID (nLen+2) Err data ~chk, nLen + 6 bytes
  // (src/SCS.cpp:182-185). Split out of send() so a sync-read burst can be assembled before it is
  // written, while the reply framing is still written down exactly once in the harness: send() is
  // byte for byte what it was, and every existing case is untouched (PHASE3 4.H10). Phase 6
  // renamed send() to reply(), which with every knob at its default still is.
  static std::vector<uint8_t> frame_bytes(
    uint8_t id, uint8_t error, const std::vector<uint8_t> & payload)
  {
    std::vector<uint8_t> reply = {
      0xff, 0xff, id, static_cast<uint8_t>(payload.size() + 2), error};
    reply.insert(reply.end(), payload.begin(), payload.end());
    uint8_t sum = 0;
    for (size_t i = 2; i < reply.size(); i++) {
      sum = static_cast<uint8_t>(sum + reply[i]);
    }
    reply.push_back(static_cast<uint8_t>(~sum));
    return reply;
  }

  void emit(const std::vector<uint8_t> & bytes)
  {
    size_t sent = 0;
    while (sent < bytes.size()) {
      const ssize_t n = ::write(master_, bytes.data() + sent, bytes.size() - sent);
      if (n <= 0) {
        return;
      }
      sent += static_cast<size_t>(n);
    }
  }

  // Every addressed reply -- a ping's, a READ's, a write's ack -- from `from`, carrying the
  // servo's status byte. The Phase 6 knobs act here (PHASE6_SPEC D.1 #8, #13, #14): the
  // sync-read faults when they are switched on for addressed replies, the twin, and the delay of
  // an ack that waits for its EEPROM commit, which goes through the outbox and never a sleep.
  // `due` is that commit's end ({} = send now); `write_ack` marks a write's ack. Returns the outbox
  // ticket of a queued reply, or 0. A twin limited to n replies counts down here.
  uint64_t reply(
    FakeServo & s, uint8_t from, const std::vector<uint8_t> & payload, bool read_reply,
    std::chrono::steady_clock::time_point due = {}, bool write_ack = false)
  {
    if (payload.size() > 253u) {
      return 0;   // the length byte counts Inst and the checksum; a longer reply would truncate
    }
    const bool faulty = faults_apply_to_addressed_;
    std::vector<uint8_t> frame = frame_bytes(
      (faulty && s.reply_id_override != 0) ? s.reply_id_override : from, s.status, payload);
    if (faulty && s.reply_length_delta != 0) {
      frame[3] = static_cast<uint8_t>(static_cast<int>(frame[3]) + s.reply_length_delta);
      repair_checksum(&frame);
    }
    if (faulty && s.reply_checksum_corrupt) {
      frame[5] = static_cast<uint8_t>(frame[5] ^ 0x01);   // as the sync-read knob does
    }
    const bool garbling = s.twin == TwinReply::garbled ||
      (s.twin == TwinReply::garbled_reads && read_reply);
    const bool doubling = s.twin == TwinReply::doubled ||
      (s.twin == TwinReply::doubled_reads && read_reply);
    const bool twin_acts = s.twin_replies != 0 && (garbling || doubling);
    if (twin_acts && s.twin_replies > 0) {
      s.twin_replies--;
    }
    if ((twin_acts && garbling) || (write_ack && s.garble_write_acks)) {
      frame.back() = static_cast<uint8_t>(~frame.back());
    }
    if (twin_acts && doubling) {
      const std::vector<uint8_t> copy = frame;
      frame.insert(frame.end(), copy.begin(), copy.end());
    }
    if (due != std::chrono::steady_clock::time_point{}) {
      const uint64_t ticket = ++next_ticket_;
      outbox_.push_back(Outgoing{due, std::move(frame), ticket});
      return ticket;
    }
    emit(frame);
    return 0;
  }

  int master_ = -1;
  int slave_ = -1;
  std::string port_;
  mutable std::mutex mutex_;
  std::map<uint8_t, FakeServo> servos_;
  // INST_SYNC_READ frames seen and the newest one's id list, both under mutex_ (PHASE3 4.H3)
  uint64_t sync_read_requests_ = 0;
  std::vector<uint8_t> last_sync_read_ids_;
  // the burst-level sync-read knobs, all under mutex_ (PHASE3 4.H4-4.H8)
  bool sync_read_supported_ = true;
  SyncReadReplyOrder reply_order_ = SyncReadReplyOrder::request;
  size_t sync_read_truncate_bytes_ = 0;
  unsigned sync_read_delay_ms_ = 0;
  // the Phase 6 bus-level state, all under mutex_ (PHASE6_SPEC D.1)
  std::vector<FrameRecord> frames_;
  uint64_t id_collisions_ = 0;
  EepromPolicy eeprom_policy_ = EepromPolicy::apply_always;
  uint8_t power_up_lock_ = 0;
  bool faults_apply_to_addressed_ = false;
  bool baud_model_ = false;           // factory_reset, under mutex_ like the rest
  // touched only by the responder thread
  std::deque<uint8_t> pending_;
  // Replies held back until their time comes. An outbox and not a sleep, because answer() runs
  // with mutex_ held (PHASE3 4.H8, 2.93). `ticket` names a commit's ack (0 for every other entry)
  // so answer() can send that one early; tickets count up from 1 and are never reused.
  struct Outgoing
  {
    std::chrono::steady_clock::time_point due;
    std::vector<uint8_t> bytes;
    uint64_t ticket = 0;
  };
  std::vector<Outgoing> outbox_;
  uint64_t next_ticket_ = 0;
  std::atomic<bool> stop_{false};
  // poll() calls that timed out with nothing queued and nothing half-parsed; wait_quiet's clock
  std::atomic<uint64_t> idle_polls_{0};
  std::atomic<uint64_t> bad_checksums_{0};
  // raw bytes read off the master; the responder adds, a test reads and clear_frames() zeroes
  std::atomic<uint64_t> bytes_received_{0};
  // wait_quiet() is const, so its give-up counter is mutable
  mutable std::atomic<uint64_t> quiet_timeouts_{0};
  // declared last, so every member it touches is alive before it starts
  std::thread thread_;
};

}  // namespace waveshare_servos_test

#endif  // FAKE_SERVO_BUS_HPP_
