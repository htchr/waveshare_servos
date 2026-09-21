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

#ifndef FAKE_SERVO_BUS_HPP_
#define FAKE_SERVO_BUS_HPP_

#include <errno.h>
#include <poll.h>
#include <pty.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <map>
#include <mutex>
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
};

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
    servo.mem[kRegMode] = mode;
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

  void set_byte(uint8_t id, uint8_t reg, uint8_t value)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    servos_.at(id).mem[reg] = value;
  }

  // little endian, sign-magnitude on `bit`
  void set_word(uint8_t id, uint8_t reg, int value, int bit = 15)
  {
    const uint16_t raw = sign_magnitude_encode(value, bit);
    const std::lock_guard<std::mutex> lock(mutex_);
    FakeServo & s = servos_.at(id);
    s.mem[reg] = static_cast<uint8_t>(raw & 0xff);
    s.mem[static_cast<uint8_t>(reg + 1)] = static_cast<uint8_t>(raw >> 8);
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
    s.requests++;
    if (instruction == kInstPing) {
      s.pings++;
      send(id, s.status, {});
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
      std::vector<uint8_t> payload;
      payload.reserve(length);
      for (size_t i = 0; i < length; i++) {
        payload.push_back(s.mem[(address + i) & 0xff]);
      }
      send(id, s.status, payload);
      return;
    }
    if (instruction == kInstWrite && !params.empty()) {
      s.writes++;
      const uint8_t address = params[0];
      for (size_t i = 1; i < params.size(); i++) {
        s.mem[(address + i - 1) & 0xff] = params[i];
      }
      if (address == kRegTorqueEnable) {
        s.torque_enable_writes++;
      }
      if (address == kRegMode) {
        s.mode_writes++;
      }
      send(id, s.status, {});
    }
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
        outbox_.emplace_back(
          std::chrono::steady_clock::now() +
          std::chrono::milliseconds(s.reply_delay_polls * kPollTimeoutMs), std::move(frame));
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
      outbox_.emplace_back(
        std::chrono::steady_clock::now() + std::chrono::milliseconds(sync_read_delay_ms_),
        std::move(burst));
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
      if (outbox_[i].first <= now) {
        emit(outbox_[i].second);
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

  // The framing send() has always done: ff ff ID (nLen+2) Err data ~chk, nLen + 6 bytes
  // (src/SCS.cpp:182-185). Split out of send() so a sync-read burst can be assembled before it is
  // written, while the reply framing is still written down exactly once in the harness: send() is
  // byte for byte what it was, and every existing case is untouched (PHASE3 4.H10).
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

  void send(uint8_t id, uint8_t error, const std::vector<uint8_t> & payload)
  {
    if (payload.size() > 253u) {
      return;   // the length byte counts Inst and the checksum too; a longer reply would truncate
    }
    emit(frame_bytes(id, error, payload));
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
  // touched only by the responder thread
  std::deque<uint8_t> pending_;
  // Replies held back until their time comes: {due, bytes}. An outbox and not a sleep, because
  // answer() runs with mutex_ held (PHASE3 4.H8, 2.93).
  std::vector<std::pair<std::chrono::steady_clock::time_point, std::vector<uint8_t>>> outbox_;
  std::atomic<bool> stop_{false};
  // poll() calls that timed out with nothing queued and nothing half-parsed; wait_quiet's clock
  std::atomic<uint64_t> idle_polls_{0};
  std::atomic<uint64_t> bad_checksums_{0};
  // wait_quiet() is const, so its give-up counter is mutable
  mutable std::atomic<uint64_t> quiet_timeouts_{0};
  // declared last, so every member it touches is alive before it starts
  std::thread thread_;
};

}  // namespace waveshare_servos_test

#endif  // FAKE_SERVO_BUS_HPP_
