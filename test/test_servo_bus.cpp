// Tests for include/servo_bus.hpp + src/servo_bus.cpp (PHASE2_SPEC 9).
//
// Every case runs over an openpty() pair, so the vendored packet code runs unchanged and nothing
// here needs a servo, a USB adapter or /dev/ttyACM0. The fixture opens the pair, closes the slave
// descriptor it was handed -- ServoBus opens the path itself, exactly as it would a real tty --
// and keeps the master, which is the fake servo's side of the wire.
//
// Two things in here are load-bearing rather than decorative:
//   - the responder is an RAII member, so an ASSERT_* that unwinds out of a test body still joins
//     the thread before the master descriptor is closed. Joining "at the end of the test body" is
//     a crash recipe: ~std::thread on a joinable thread calls std::terminate() and takes every
//     other case in the binary with it.
//   - close_releases_the_port_for_a_later_open is a regression test for the explicit TIOCNXCL in
//     ServoBus::close(). Without it, a port this fixture still references through the master stays
//     marked exclusive and the second open() fails with EBUSY in this very process.

#include <gmock/gmock.h>

#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <optional>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "fake_servo_bus.hpp"
#include "servo_bus.hpp"

namespace
{

// Per-name declarations, never a using-directive: cpplint's build/namespaces rule forbids
// using-directives outside a short std::*_literals whitelist, in sources as well as headers.
using waveshare_servos::BusStatus;
using waveshare_servos::FeedbackBlock;
using waveshare_servos::GoalPosition;
using waveshare_servos::GoalSpeed;
using waveshare_servos::OpenResult;
using waveshare_servos::ServoBus;
using waveshare_servos::SyncReadStats;
using waveshare_servos::WriteResult;
using waveshare_servos::WriteStatus;
using waveshare_servos::decode_feedback_block;
using waveshare_servos::parse_sync_read_burst;
using waveshare_servos::port_holder_pids;
using waveshare_servos::sign_magnitude_encode;
using waveshare_servos::to_string;

constexpr int kBaudrate = 1000000;
constexpr uint32_t kIoTimeoutMs = 20;
constexpr uint8_t kServoId = 1;

// The refusal a second ServoBus meets on a port this process already holds. It is always refused;
// only the reason differs. An unprivileged process cannot even open the device, because the first
// bus set TIOCEXCL, so it fails at the lock descriptor with EBUSY. Root is allowed past TIOCEXCL
// by the kernel, gets its descriptor, and then loses the advisory lock instead.
::testing::Matcher<BusStatus> refused_second_open()
{
  return ::geteuid() == 0 ? ::testing::Eq(BusStatus::LOCK_FAILED) :
         ::testing::Eq(BusStatus::LOCK_OPEN_FAILED);
}

int refused_second_open_errno()
{
  return ::geteuid() == 0 ? EWOULDBLOCK : EBUSY;
}

// How many of this process's descriptors point at `path`, straight out of /proc/self/fd. Immune to
// the descriptor the directory iterator itself holds, which a plain count of the directory is not.
size_t descriptors_on(const std::string & path)
{
  size_t count = 0;
  std::error_code ec;
  std::filesystem::directory_iterator it("/proc/self/fd", ec);
  if (ec) {
    return 0;
  }
  const std::filesystem::directory_iterator end;
  for (; it != end; it.increment(ec)) {
    if (ec) {
      break;
    }
    std::error_code link_ec;
    const std::filesystem::path target = std::filesystem::read_symlink(it->path(), link_ec);
    if (!link_ec && target.string() == path) {
      count++;
    }
  }
  return count;
}

// A regular file for the isatty() refusal to be exercised against: it opens, it is not a tty.
std::filesystem::path make_regular_file(const std::string & stem)
{
  const std::filesystem::path path =
    std::filesystem::temp_directory_path() / (stem + "_" + std::to_string(::getpid()));
  std::FILE * file = std::fopen(path.c_str(), "w");
  if (file != nullptr) {
    std::fclose(file);
  }
  return path;
}

// Closes a descriptor however the test body leaves it, so an ASSERT_* that unwinds out of the
// middle of a case cannot leak one onto the pty and keep it alive past TearDown.
class FdCloser
{
public:
  explicit FdCloser(int fd)
  : fd_(fd) {}

  ~FdCloser()
  {
    if (fd_ != -1) {
      ::close(fd_);
    }
  }

  FdCloser(const FdCloser &) = delete;
  FdCloser & operator=(const FdCloser &) = delete;
  FdCloser(FdCloser &&) = delete;
  FdCloser & operator=(FdCloser &&) = delete;

private:
  int fd_ = -1;
};

// Reaches the vendored buffer itself rather than the wrapper's constant that claims to describe
// it. txBuf is protected in SCSerial, so a derived type may name it through its own `this` -- a
// non-static member function, to stay clear of the [class.protected] access rule.
struct TxBufProbe : ServoBus
{
  size_t bytes() const {return sizeof(txBuf);}
};

// PHASE3 1.30, seam 1. Captures the frames a ServoBus builds without letting a byte reach a port,
// by overriding both writeSCS overloads, wFlushSCS, rFlushSCS and readSCS. This is the seam probe 2
// used to prove byte identity against SMS_STS::SyncWritePosEx (probe/out_p2_offline.txt),
// reproduced here so the proof lives in the suite instead of in a scratchpad.
//
// Two things are load-bearing rather than tidy:
//   - `fd` must be a valid descriptor, because ServoBus::is_open() is `fd != -1` and
//     write_goal_positions refuses a closed bus (PHASE3 1.17). /dev/null is the harmless one: the
//     writeSCS overrides mean nothing is ever written to it, and ~ServoBus closes it.
//   - readSCS must be overridden even though this seam never reads. SyncWriteSpe's per-servo
//     genWrite calls Ack(), which calls readSCS(), which does FD_SET(fd, ...)
//     (src/SCSerial.cpp:144) -- and with a -1 there glibc's _FORTIFY_SOURCE check aborts the whole
//     test binary. Returning 0 makes every Ack fail fast, which is what gives the five-frame
//     count of the SyncWriteSpe A/B.
//
// Because nothing here touches the genuine txBuf, this seam is also the only safe place to watch
// an UNCHUNKED implementation build an oversized frame: SCSerial::writeSCS has no bounds check
// (src/SCSerial.cpp:179-191), so the same experiment against the real buffer is UB, not a test.
struct PacketCapture : ServoBus
{
  PacketCapture() {fd = ::open("/dev/null", O_RDWR | O_CLOEXEC);}

  std::vector<std::vector<uint8_t>> frames;
  std::vector<uint8_t> building;

protected:
  int writeSCS(unsigned char * data, int length) override
  {
    building.insert(building.end(), data, data + length);
    return static_cast<int>(building.size());
  }
  int writeSCS(unsigned char byte) override
  {
    building.push_back(byte);
    return static_cast<int>(building.size());
  }
  void wFlushSCS() override
  {
    frames.push_back(building);
    building.clear();
  }
  void rFlushSCS() override {}                              // no tty to flush
  int readSCS(unsigned char *, int) override {return 0;}    // see above: never touch the fd
};

// PHASE3 1.30, seam 2. Reads the GENUINE txBufLen the vendored writeSCS produced, by overriding
// only wFlushSCS -- the one place txBufLen is zeroed (src/SCSerial.cpp:218). Runs over the pty
// fixture, so the base wFlushSCS still sends the bytes and the chunk boundaries are asserted
// against the real buffer rather than against the spec's own arithmetic.
struct TxBufLenProbe : ServoBus
{
  std::vector<int> lengths;

protected:
  void wFlushSCS() override
  {
    lengths.push_back(txBufLen);
    ServoBus::wFlushSCS();
  }
};

// The tie 1.8's "Decision: use the vendored SCS::Host2SCS" asks for, made where it can be made.
// ServoBus::position_record and ServoBus::speed_record are static (PHASE3 0.3, R1, so 4.T4-4.T7
// can call them with no object at all) and SCS::Host2SCS is a non-static protected member, so the
// builders cannot delegate to it; duplicating the split in the chunk loop just so the loop could
// would leave two implementations of the record layout. Instead the wrapper keeps one
// implementation and this seam pins it against the library's, byte for byte. Host2SCS is the
// single place the `End` flag decides byte order (src/SCS.cpp:34-43) and SMS_STS::SMS_STS() sets
// End = 0 (src/SMS_STS.cpp:12), so an upstream flip fails here instead of silently reversing
// every goal word on the wire.
struct EndiannessProbe : ServoBus
{
  std::array<uint8_t, 2> split(uint16_t value)
  {
    std::array<uint8_t, 2> halves{};
    Host2SCS(&halves[0], &halves[1], value);
    return halves;
  }
};

// One sync-write frame, taken apart the way a servo would: ff ff fe mesLen 83 addr nLen
// {id data(nLen)}... ~chk (src/SCS.cpp:124-151). `well_formed` is false unless the header, the
// length byte, the record stride and the checksum all agree, so a chunking case can verify each
// frame independently instead of trusting the builder that produced it.
struct ParsedSyncWrite
{
  bool well_formed = false;
  uint8_t address = 0;
  size_t record_bytes = 0;
  std::vector<uint8_t> ids;
  std::vector<std::vector<uint8_t>> records;
};

ParsedSyncWrite parse_sync_write(const std::vector<uint8_t> & frame)
{
  ParsedSyncWrite parsed;
  if (frame.size() < 8 || frame[0] != 0xff || frame[1] != 0xff || frame[2] != 0xfe ||
    frame[4] != 0x83)
  {
    return parsed;
  }
  parsed.address = frame[5];
  parsed.record_bytes = frame[6];
  const size_t stride = parsed.record_bytes + 1;
  if (parsed.record_bytes == 0 || (frame.size() - 8) % stride != 0) {
    return parsed;
  }
  const size_t records = (frame.size() - 8) / stride;
  // mesLen counts the instruction byte through the checksum (src/SCS.cpp:127), and it is a u8 that
  // wraps at 32 position / 84 speed records -- one step past the txBuf overflow, which is why the
  // chunker never has to reason about it.
  if (frame[3] != static_cast<uint8_t>(stride * records + 4)) {
    return parsed;
  }
  uint8_t sum = 0;
  for (size_t i = 2; i + 1 < frame.size(); i++) {
    sum = static_cast<uint8_t>(sum + frame[i]);
  }
  if (static_cast<uint8_t>(~sum) != frame.back()) {
    return parsed;
  }
  for (size_t r = 0; r < records; r++) {
    const size_t at = 7 + r * stride;
    parsed.ids.push_back(frame[at]);
    parsed.records.emplace_back(
      frame.begin() + at + 1, frame.begin() + at + 1 + parsed.record_bytes);
  }
  parsed.well_formed = true;
  return parsed;
}

std::chrono::milliseconds time_one_ping(ServoBus * bus, uint8_t id)
{
  const auto started = std::chrono::steady_clock::now();
  bus->Ping(id);
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - started);
}

// A fake servo living on the master side of the pty: a 256-byte register file answering PING,
// READ and WRITE the way the vendored packet layer expects. The thread is stopped and joined by
// the destructor, so unwinding out of a test body can never leave it running.
class FakeResponder
{
public:
  FakeResponder(int master, uint8_t id, const std::array<uint8_t, 256> & registers)
  : master_(master), id_(id), registers_(registers)
  {
    thread_ = std::thread(&FakeResponder::run, this);
  }

  ~FakeResponder()
  {
    stop_.store(true);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  FakeResponder(const FakeResponder &) = delete;
  FakeResponder & operator=(const FakeResponder &) = delete;
  FakeResponder(FakeResponder &&) = delete;
  FakeResponder & operator=(FakeResponder &&) = delete;

private:
  void run()
  {
    while (!stop_.load()) {
      struct pollfd waiting = {master_, POLLIN, 0};
      const int ready = ::poll(&waiting, 1, 10);
      if (ready <= 0) {
        continue;
      }
      // Between the fixture closing the slave and ServoBus opening the path there is no slave
      // open, and POLLHUP is then reported regardless of the events mask -- poll() would return
      // at once forever and spin a core. Sleep instead.
      if ((waiting.revents & POLLHUP) != 0 && (waiting.revents & POLLIN) == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      std::array<uint8_t, 64> chunk{};
      const ssize_t n = ::read(master_, chunk.data(), chunk.size());
      if (n <= 0) {
        continue;
      }
      pending_.insert(pending_.end(), chunk.begin(), chunk.begin() + n);
      consume();
    }
  }

  // ff ff ID LEN INST [params...] CHECKSUM, where LEN counts INST through CHECKSUM
  // (src/SCS.cpp:62-90).
  void consume()
  {
    while (pending_.size() >= 4) {
      if (pending_[0] != 0xff || pending_[1] != 0xff) {
        pending_.pop_front();
        continue;
      }
      // LEN counts INST through CHECKSUM, so it is never below 2. A smaller byte is not a frame
      // header at all: resync rather than hand answer() a vector too short for frame[4].
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
      answer(frame);
    }
  }

  void answer(const std::vector<uint8_t> & frame)
  {
    const uint8_t id = frame[2];
    const uint8_t instruction = frame[4];
    if (id != id_) {           // 0xfe is a broadcast and is never acked
      return;
    }
    if (instruction == 0x01) {                       // INST_PING
      send({});
    } else if (instruction == 0x02 && frame.size() >= 7) {   // INST_READ
      const size_t address = frame[5];
      const size_t length = frame[6];
      std::vector<uint8_t> payload;
      for (size_t i = 0; i < length; i++) {
        payload.push_back(registers_[(address + i) & 0xff]);
      }
      send(payload);
    } else if (instruction == 0x03) {                // INST_WRITE
      send({});
    }
  }

  void send(const std::vector<uint8_t> & payload)
  {
    if (payload.size() > 253u) {
      return;   // LEN is one byte and counts INST and CHECKSUM too; a longer reply would truncate
    }
    const uint8_t length = static_cast<uint8_t>(payload.size() + 2);
    std::vector<uint8_t> reply = {0xff, 0xff, id_, length, 0x00};
    reply.insert(reply.end(), payload.begin(), payload.end());
    uint8_t sum = 0;
    for (size_t i = 2; i < reply.size(); i++) {
      sum = static_cast<uint8_t>(sum + reply[i]);
    }
    reply.push_back(static_cast<uint8_t>(~sum));
    size_t sent = 0;
    while (sent < reply.size()) {
      const ssize_t n = ::write(master_, reply.data() + sent, reply.size() - sent);
      if (n <= 0) {
        return;
      }
      sent += static_cast<size_t>(n);
    }
  }

  int master_ = -1;
  uint8_t id_ = 0;
  std::array<uint8_t, 256> registers_{};
  std::deque<uint8_t> pending_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

class ServoBusPty : public ::testing::Test
{
protected:
  void SetUp() override
  {
    int slave = -1;
    ASSERT_EQ(::openpty(&master_, &slave, nullptr, nullptr, nullptr), 0) << std::strerror(errno);
    const char * name = ::ttyname(slave);
    ASSERT_NE(name, nullptr) << std::strerror(errno);
    port_ = name;
    // ServoBus opens the path itself, exactly as it would a real tty.
    ::close(slave);
  }

  void TearDown() override
  {
    responder_.reset();
    bus_.close();
    if (master_ != -1) {
      ::close(master_);
      master_ = -1;
    }
  }

  // registers 56..70 are the feedback block SMS_STS::FeedBack reads in one go
  std::array<uint8_t, 256> feedback_registers() const
  {
    std::array<uint8_t, 256> registers{};
    registers[33] = 0;      // SMS_STS_MODE
    registers[56] = 0x00;   // present position, low byte
    registers[57] = 0x08;   // present position, high byte -> 2048 ticks
    registers[62] = 120;    // present voltage
    registers[63] = 41;     // present temperature
    return registers;
  }

  // Everything the master side sees within `timeout_ms`. A refusal has to be proved by the
  // absence of bytes on the wire, and a poll of a pty nobody writes to is the only way to say so.
  std::vector<uint8_t> collect_master_bytes(int timeout_ms)
  {
    std::vector<uint8_t> seen;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
      struct pollfd waiting = {master_, POLLIN, 0};
      if (::poll(&waiting, 1, 5) <= 0 || (waiting.revents & POLLIN) == 0) {
        continue;
      }
      std::array<uint8_t, 512> chunk{};
      const ssize_t n = ::read(master_, chunk.data(), chunk.size());
      if (n <= 0) {
        break;
      }
      seen.insert(seen.end(), chunk.begin(), chunk.begin() + n);
    }
    return seen;
  }

  // Everything queued on the master right now, discarded, counted and never waited for. A pty
  // holds only a few kilobytes, and SCSerial::wFlushSCS answers a full one by spinning on EAGAIN
  // up to 1000 times and then giving up silently (src/SCSerial.cpp:205-218) -- so a case that
  // writes more than a pty-full has stopped testing whatever it meant to test and started testing
  // that retry loop (the drain note of PHASE3 1.30). Calling this between writes is what keeps the
  // queue empty. A pty write is synchronous into the master's queue, so a poll of 0 ms is enough:
  // by the time write_goal_*() has returned, the bytes are already readable here.
  size_t drain_master()
  {
    size_t drained = 0;
    while (true) {
      struct pollfd waiting = {master_, POLLIN, 0};
      if (::poll(&waiting, 1, 0) <= 0 || (waiting.revents & POLLIN) == 0) {
        return drained;
      }
      std::array<uint8_t, 4096> chunk{};
      const ssize_t n = ::read(master_, chunk.data(), chunk.size());
      if (n <= 0) {
        return drained;
      }
      drained += static_cast<size_t>(n);
    }
  }

  ServoBus bus_;
  int master_ = -1;
  std::string port_;
  // declared last, so it is destroyed (stopped and joined) before master_ is touched
  std::optional<FakeResponder> responder_;
};

// PHASE3 1.30, seam 3: the cases that need a register file rather than a byte stream. FakeBus owns
// its own openpty() pair and publishes the slave path, so it must NOT be mixed with ServoBusPty,
// which owns a different pair and a one-id FakeResponder that neither applies writes to its
// registers nor records broadcasts.
class ServoBusFakeBus : public ::testing::Test
{
protected:
  void SetUp() override
  {
    for (const uint8_t id : {1, 2, 3, 4}) {
      fake_.add_servo(id);
    }
    ASSERT_TRUE(static_cast<bool>(bus_.open(fake_.port(), kBaudrate, kIoTimeoutMs)));
  }

  void TearDown() override
  {
    bus_.close();
    EXPECT_EQ(fake_.bad_checksums(), 0u) << "the wire itself misbehaved";
    EXPECT_EQ(fake_.quiet_timeouts(), 0u) << "wait_quiet gave up; a record may have been missed";
  }

  waveshare_servos_test::FakeBus fake_;
  ServoBus bus_;
};

}  // namespace

TEST_F(ServoBusPty, open_succeeds_on_a_pseudo_terminal)
{
  const OpenResult opened = bus_.open(port_, kBaudrate, kIoTimeoutMs);
  ASSERT_TRUE(static_cast<bool>(opened)) << to_string(opened.status) << ": " <<
    std::strerror(opened.error);
  EXPECT_EQ(opened.status, BusStatus::OK);
  EXPECT_EQ(opened.error, 0);
  EXPECT_TRUE(bus_.is_open());
  EXPECT_EQ(bus_.port(), port_);
  EXPECT_EQ(bus_.baudrate(), kBaudrate);
  // the lock descriptor and the library's, which is the whole price of holding the port twice
  EXPECT_EQ(descriptors_on(port_), 2u);
}

TEST_F(ServoBusPty, open_sets_the_io_timeout_before_the_first_transaction)
{
  // The library's own constructor leaves IOTimeOut at 100 ms; the driver used to overwrite it
  // after begin(), which left the very first transaction on the stock value.
  ASSERT_TRUE(static_cast<bool>(bus_.open(port_, kBaudrate, 7)));
  EXPECT_EQ(bus_.io_timeout_ms(), 7u);
  EXPECT_NE(bus_.io_timeout_ms(), 100u);
}

TEST_F(ServoBusPty, open_refuses_an_unmapped_baud_rate_without_touching_the_port)
{
  const OpenResult opened = bus_.open(port_, 230400, kIoTimeoutMs);
  EXPECT_EQ(opened.status, BusStatus::UNSUPPORTED_BAUDRATE);
  EXPECT_EQ(opened.error, 0);
  EXPECT_FALSE(bus_.is_open());
  EXPECT_EQ(descriptors_on(port_), 0u);
  EXPECT_EQ(bus_.port(), "");
  // the port was never touched, so a plain open of it still succeeds
  const int probe = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  EXPECT_NE(probe, -1) << std::strerror(errno);
  if (probe != -1) {
    ::close(probe);
  }
}

TEST_F(ServoBusPty, open_refuses_a_zero_io_timeout)
{
  const OpenResult opened = bus_.open(port_, kBaudrate, 0);
  EXPECT_EQ(opened.status, BusStatus::INVALID_TIMEOUT);
  EXPECT_EQ(opened.error, 0);
  EXPECT_FALSE(bus_.is_open());
  EXPECT_EQ(descriptors_on(port_), 0u);
}

TEST_F(ServoBusPty, set_io_timeout_ms_refuses_zero_and_keeps_the_previous_value)
{
  ASSERT_TRUE(static_cast<bool>(bus_.open(port_, kBaudrate, kIoTimeoutMs)));
  EXPECT_FALSE(bus_.set_io_timeout_ms(0));
  EXPECT_EQ(bus_.io_timeout_ms(), kIoTimeoutMs);
  // 7 and not 5: 5 is the driver's default since PHASE3 5.19, and a probe value that can coincide
  // with a default would let a set_io_timeout_ms() that does nothing at all pass this case.
  EXPECT_TRUE(bus_.set_io_timeout_ms(7));
  EXPECT_EQ(bus_.io_timeout_ms(), 7u);
}

TEST_F(ServoBusPty, open_on_an_already_open_bus_is_refused_and_keeps_the_session)
{
  ASSERT_TRUE(static_cast<bool>(bus_.open(port_, kBaudrate, kIoTimeoutMs)));
  const OpenResult again = bus_.open(port_, 115200, 30);
  EXPECT_EQ(again.status, BusStatus::ALREADY_OPEN);
  EXPECT_EQ(again.error, 0);
  EXPECT_TRUE(bus_.is_open());
  EXPECT_EQ(bus_.baudrate(), kBaudrate);
  EXPECT_EQ(bus_.io_timeout_ms(), kIoTimeoutMs);
  EXPECT_EQ(descriptors_on(port_), 2u);
}

TEST_F(ServoBusPty, a_second_bus_on_the_same_port_is_refused)
{
  ASSERT_TRUE(static_cast<bool>(bus_.open(port_, kBaudrate, kIoTimeoutMs)));
  ServoBus other;
  const OpenResult second = other.open(port_, kBaudrate, kIoTimeoutMs);
  EXPECT_THAT(second.status, refused_second_open());
  EXPECT_EQ(second.error, refused_second_open_errno());
  EXPECT_FALSE(other.is_open());
  EXPECT_TRUE(bus_.is_open());
  // the refusal left nothing behind: still only the first bus's two descriptors
  EXPECT_EQ(descriptors_on(port_), 2u);
}

TEST_F(ServoBusPty, an_advisory_lock_held_by_another_descriptor_refuses_the_open)
{
  // The only case that puts the advisory lock on trial by itself. a_second_bus_is_refused never
  // reaches step 3 unless it runs as root, because the first bus's TIOCEXCL already refuses the
  // lock descriptor; here nobody has set TIOCEXCL, so steps 1-2 pass and flock is the one thing
  // that can say no. Without it, deleting the flock from open() breaks no test at all -- and the
  // lock is the half of the exclusivity that a root process cannot walk past.
  const int other = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  ASSERT_NE(other, -1) << std::strerror(errno);
  const FdCloser closer(other);
  ASSERT_EQ(::flock(other, LOCK_EX | LOCK_NB), 0) << std::strerror(errno);

  const OpenResult opened = bus_.open(port_, kBaudrate, kIoTimeoutMs);
  EXPECT_EQ(opened.status, BusStatus::LOCK_FAILED);
  EXPECT_EQ(opened.error, EWOULDBLOCK);
  EXPECT_FALSE(bus_.is_open());
  EXPECT_EQ(bus_.port(), "");
  // the refusal left nothing behind: only the descriptor this test itself holds
  EXPECT_EQ(descriptors_on(port_), 1u);
}

TEST_F(ServoBusPty, a_plain_open_by_this_process_is_refused_while_the_bus_holds_the_port)
{
  if (::geteuid() == 0) {
    GTEST_SKIP() << "root bypasses TIOCEXCL; the flock case is covered by a_second_bus_is_refused";
  }
  ASSERT_TRUE(static_cast<bool>(bus_.open(port_, kBaudrate, kIoTimeoutMs)));
  errno = 0;
  const int probe = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  const int refused_with = errno;
  if (probe != -1) {
    ::close(probe);
  }
  EXPECT_EQ(probe, -1) << "TIOCEXCL did not take: screen, minicom and set_id could still open it";
  EXPECT_EQ(refused_with, EBUSY);
}

TEST_F(ServoBusPty, a_refused_open_leaves_no_descriptor_behind)
{
  const std::string missing = "/dev/waveshare_servos_no_such_port";
  const std::filesystem::path regular = make_regular_file("servo_bus_not_a_tty");
  ASSERT_TRUE(std::filesystem::exists(regular));

  EXPECT_EQ(bus_.open(missing, kBaudrate, kIoTimeoutMs).status, BusStatus::LOCK_OPEN_FAILED);
  EXPECT_EQ(descriptors_on(missing), 0u);
  EXPECT_EQ(bus_.open(regular.string(), kBaudrate, kIoTimeoutMs).status, BusStatus::NOT_A_TTY);
  EXPECT_EQ(descriptors_on(regular.string()), 0u);
  EXPECT_EQ(bus_.open(port_, 230400, kIoTimeoutMs).status, BusStatus::UNSUPPORTED_BAUDRATE);
  EXPECT_EQ(bus_.open(port_, kBaudrate, 0).status, BusStatus::INVALID_TIMEOUT);
  EXPECT_EQ(descriptors_on(port_), 0u);
  EXPECT_FALSE(bus_.is_open());

  std::error_code ec;
  std::filesystem::remove(regular, ec);
}

TEST_F(ServoBusPty, close_releases_the_port_for_a_later_open)
{
  ASSERT_TRUE(static_cast<bool>(bus_.open(port_, kBaudrate, kIoTimeoutMs)));
  bus_.close();
  EXPECT_FALSE(bus_.is_open());
  EXPECT_EQ(bus_.port(), "");
  EXPECT_EQ(bus_.baudrate(), 0);
  EXPECT_EQ(descriptors_on(port_), 0u);
  // The fixture still holds the master, so the tty is not released by closing the slave: without
  // the explicit TIOCNXCL in close() this open() fails with EBUSY in this very process.
  const OpenResult again = bus_.open(port_, kBaudrate, kIoTimeoutMs);
  EXPECT_TRUE(static_cast<bool>(again)) << to_string(again.status) << ": " <<
    std::strerror(again.error);
}

TEST_F(ServoBusPty, close_is_safe_on_a_bus_that_was_never_opened)
{
  ServoBus fresh;
  fresh.close();
  fresh.close();
  EXPECT_FALSE(fresh.is_open());
  // and after a refused open, which is the state a failed on_configure leaves behind
  EXPECT_EQ(fresh.open(port_, 230400, kIoTimeoutMs).status, BusStatus::UNSUPPORTED_BAUDRATE);
  fresh.close();
  EXPECT_FALSE(fresh.is_open());
  EXPECT_EQ(descriptors_on(port_), 0u);
}

TEST_F(ServoBusPty, the_destructor_releases_the_port)
{
  {
    ServoBus scoped;
    ASSERT_TRUE(static_cast<bool>(scoped.open(port_, kBaudrate, kIoTimeoutMs)));
    EXPECT_EQ(descriptors_on(port_), 2u);
  }
  EXPECT_EQ(descriptors_on(port_), 0u);
  const OpenResult again = bus_.open(port_, kBaudrate, kIoTimeoutMs);
  EXPECT_TRUE(static_cast<bool>(again)) << to_string(again.status) << ": " <<
    std::strerror(again.error);
}

TEST_F(ServoBusPty, a_path_that_is_not_a_tty_is_refused)
{
  const std::filesystem::path regular = make_regular_file("servo_bus_regular_file");
  ASSERT_TRUE(std::filesystem::exists(regular));

  const OpenResult opened = bus_.open(regular.string(), kBaudrate, kIoTimeoutMs);
  EXPECT_EQ(opened.status, BusStatus::NOT_A_TTY);
  EXPECT_EQ(opened.error, ENOTTY);
  EXPECT_FALSE(bus_.is_open());

  std::error_code ec;
  std::filesystem::remove(regular, ec);
}

TEST_F(ServoBusPty, a_missing_port_is_refused)
{
  const OpenResult opened =
    bus_.open("/dev/waveshare_servos_no_such_port", kBaudrate, kIoTimeoutMs);
  EXPECT_EQ(opened.status, BusStatus::LOCK_OPEN_FAILED);
  EXPECT_EQ(opened.error, ENOENT);
  EXPECT_FALSE(bus_.is_open());
}

TEST_F(ServoBusPty, the_vendored_packet_layer_round_trips_over_the_pty)
{
  responder_.emplace(master_, kServoId, feedback_registers());
  ASSERT_TRUE(static_cast<bool>(bus_.open(port_, kBaudrate, 200)));

  EXPECT_EQ(bus_.Ping(kServoId), kServoId);
  EXPECT_EQ(bus_.readByte(kServoId, SMS_STS_MODE), 0);
  ASSERT_NE(bus_.FeedBack(kServoId), -1);
  EXPECT_EQ(bus_.ReadPos(-1), 2048);
  EXPECT_EQ(bus_.ReadVoltage(-1), 120);
  EXPECT_EQ(bus_.ReadTemper(-1), 41);
}

TEST_F(ServoBusPty, a_silent_bus_costs_one_io_timeout)
{
  // No responder: nothing on this bus ever answers, so every transaction costs exactly one
  // select() timeout. That is the whole reason io_timeout_ms exists -- the library's stock 100 ms
  // held the Phase 1 control loop down to about 1.1 Hz.
  ASSERT_TRUE(static_cast<bool>(bus_.open(port_, kBaudrate, 25)));
  const auto brief = time_one_ping(&bus_, 9);
  EXPECT_EQ(bus_.Ping(9), -1);
  ASSERT_TRUE(bus_.set_io_timeout_ms(200));
  const auto patient = time_one_ping(&bus_, 9);

  EXPECT_GE(brief.count(), 20);
  EXPECT_LT(brief.count(), 1000) << "one silent transaction must cost one timeout, not many";
  EXPECT_GE(patient.count(), 180);
  EXPECT_GT(patient.count(), brief.count() + 50) << "the timeout is what paces a silent bus";
}

TEST_F(ServoBusPty, begin_leaves_nothing_unflushed_on_stdout)
{
  // SCSerial::begin() printf()s "serial speed <rate>" and never flushes. Under ctest stdout is a
  // pipe, so it is fully buffered and that line would surface much later, interleaved into
  // whatever the process printed next -- or not at all, if the process is replaced by exec.
  // open() flushes unconditionally; this pins that.
  int pipe_fds[2] = {-1, -1};
  ASSERT_EQ(::pipe(pipe_fds), 0) << std::strerror(errno);
  std::fflush(stdout);
  const int saved_stdout = ::dup(STDOUT_FILENO);
  ASSERT_NE(saved_stdout, -1) << std::strerror(errno);
  static std::array<char, 4096> stdout_buffer{};

  ASSERT_NE(::dup2(pipe_fds[1], STDOUT_FILENO), -1) << std::strerror(errno);
  // force full buffering, so a missing flush really does hold the line back
  std::setvbuf(stdout, stdout_buffer.data(), _IOFBF, stdout_buffer.size());
  const OpenResult opened = bus_.open(port_, kBaudrate, kIoTimeoutMs);
  // deliberately no fflush here: open() is the thing under test
  ::dup2(saved_stdout, STDOUT_FILENO);
  std::setvbuf(stdout, nullptr, _IOLBF, 0);
  ::close(saved_stdout);
  ::close(pipe_fds[1]);

  std::string captured;
  std::array<char, 256> chunk{};
  for (ssize_t n = ::read(pipe_fds[0], chunk.data(), chunk.size()); n > 0;
    n = ::read(pipe_fds[0], chunk.data(), chunk.size()))
  {
    captured.append(chunk.data(), static_cast<size_t>(n));
  }
  ::close(pipe_fds[0]);

  EXPECT_TRUE(static_cast<bool>(opened)) << to_string(opened.status);
  EXPECT_THAT(captured, ::testing::HasSubstr("serial speed"));
}

TEST_F(ServoBusPty, port_holder_pids_finds_this_process)
{
  EXPECT_THAT(port_holder_pids(port_), ::testing::Not(::testing::Contains(::getpid())));
  ASSERT_TRUE(static_cast<bool>(bus_.open(port_, kBaudrate, kIoTimeoutMs)));

  const std::vector<int> holders = port_holder_pids(port_);
  EXPECT_THAT(holders, ::testing::Contains(::getpid()));
  // two descriptors, one pid: a holder is listed once however many times it opened the port
  EXPECT_EQ(std::count(holders.begin(), holders.end(), ::getpid()), 1);

  bus_.close();
  EXPECT_THAT(port_holder_pids(port_), ::testing::Not(::testing::Contains(::getpid())));
  EXPECT_THAT(port_holder_pids("/dev/waveshare_servos_no_such_port"), ::testing::IsEmpty());
}

TEST(ServoBus, the_vendored_transmit_buffer_is_255_bytes)
{
  // SCSerial::writeSCS() writes into txBuf with no bound check at all (src/SCSerial.cpp:178-190),
  // so its size is a safety property of every packet this driver builds. The real tripwire is the
  // static_assert in the out-of-line constructor, which an upstream refresh would break at compile
  // time; this reads the vendored array itself, so a resize upstream fails the case rather than
  // only a hand edit of the wrapper's constant.
  TxBufProbe probe;
  EXPECT_EQ(probe.bytes(), ServoBus::tx_buffer_bytes);
  EXPECT_EQ(ServoBus::tx_buffer_bytes, size_t{255});
}

TEST(ServoBus, only_the_seven_mapped_baud_rates_are_supported)
{
  for (const int rate : {9600, 19200, 38400, 57600, 115200, 500000, 1000000}) {
    EXPECT_TRUE(ServoBus::is_supported_baudrate(rate)) << rate;
  }
  // 230400 is the trap: SCSerial::setBaudRate maps it but SCSerial::begin does not, so begin()
  // would silently fall back to 115200.
  for (const int rate : {-1000000, -1, 0, 1, 4800, 230400, 250000, 921600, 2000000}) {
    EXPECT_FALSE(ServoBus::is_supported_baudrate(rate)) << rate;
  }
}

// ---------------------------------------------------------------------------------------------
// PHASE3 C1 stage A: the pure record builders and the chunk constants. No port, no frame, no
// vendored builder -- just the bytes a record is made of (PHASE3 1.8-1.10, 1.12-1.13).
// ---------------------------------------------------------------------------------------------

TEST(ServoBus, position_record_is_the_seven_bytes_sync_write_pos_ex_would_have_built)
{
  // PHASE3 4.T4. The record is based at register 41, so it covers ACC, the goal position word,
  // the always-zero GOAL_TIME word (src/SMS_STS.cpp:75) and the goal speed. Measured against the
  // library on the capture seam: probe/out_p2_offline.txt Q3a.
  EXPECT_EQ(
    ServoBus::position_record(30, 1000, 500),
    (std::array<uint8_t, 7>{0x1e, 0xe8, 0x03, 0x00, 0x00, 0xf4, 0x01}));
  // The one byte that changes is the high byte of the goal: bit 15 is the direction flag, so
  // -1000 is 0x83e8 and never the two's complement 0xfc18.
  EXPECT_EQ(
    ServoBus::position_record(30, -1000, 500),
    (std::array<uint8_t, 7>{0x1e, 0xe8, 0x83, 0x00, 0x00, 0xf4, 0x01}));
}

TEST(ServoBus, a_negative_goal_is_sign_magnitude_not_twos_complement)
{
  // PHASE3 4.T5. -100 is magnitude 100 (0x0064) with bit 15 set, little endian: `64 80`. Never
  // `9c ff` (two's complement) and never `9c 80` (two's complement with the sign bit bolted on).
  const std::array<uint8_t, 7> record = ServoBus::position_record(7, -100, 1);
  EXPECT_EQ(record, (std::array<uint8_t, 7>{0x07, 0x64, 0x80, 0x00, 0x00, 0x01, 0x00}));
  EXPECT_EQ(record[1], 0x64);
  EXPECT_EQ(record[2], 0x80);

  // -32767 is representable; -32768 is not reachable from the driver, because send_commands
  // clamps goal_steps to [-32767, 32767] before the cast (src/waveshare_servos.cpp:1481). That
  // clamp is load-bearing: the library negates in s16 (src/SMS_STS.cpp:59), which overflows there.
  EXPECT_EQ(
    ServoBus::position_record(0, -32767, 1),
    (std::array<uint8_t, 7>{0x00, 0xff, 0xff, 0x00, 0x00, 0x01, 0x00}));
}

TEST(ServoBus, an_acceleration_of_zero_is_written_as_zero)
{
  // PHASE3 4.T6 / 1.23. `max_accel="0"` is the documented no-ramp opt-out
  // (src/waveshare_servos.cpp:636-638) and SyncWritePosEx tests the ACC *pointer*, not the value
  // (src/SMS_STS.cpp:69), so 0 goes out as 0 today. A builder that reads 0 as "unset" and
  // substitutes a default would invert the parameter; this is the only guard on that.
  EXPECT_EQ(ServoBus::position_record(0, 2048, 100)[0], 0);
}

TEST(ServoBus, speed_record_is_sign_magnitude_and_zero_carries_no_sign_bit)
{
  // PHASE3 4.T7 / 1.9. Measured: probe/out_p2_offline.txt Q3b.
  EXPECT_EQ(ServoBus::speed_record(700), (std::array<uint8_t, 2>{0xbc, 0x02}));
  EXPECT_EQ(ServoBus::speed_record(-700), (std::array<uint8_t, 2>{0xbc, 0x82}));
  // Zero is the STOP command every deactivation depends on (src/waveshare_servos.cpp:1278-1281),
  // so it must never acquire the position path's `>= 1` goal-speed floor, nor a sign bit.
  EXPECT_EQ(ServoBus::speed_record(0), (std::array<uint8_t, 2>{0x00, 0x00}));
}

TEST(ServoBus, the_record_builders_split_a_word_the_way_the_vendored_library_does)
{
  // PHASE3 1.8-1.9. The wrapper's records must follow the library's endianness convention rather
  // than reproduce it by coincidence; since the static builders cannot call the protected
  // Host2SCS, this is where the two are held together. See the comment on EndiannessProbe.
  EndiannessProbe probe;
  for (const int16_t value : {0, 1, -1, 100, -100, 1000, -1000, 4095, -4095, 32767, -32767}) {
    const std::array<uint8_t, 2> library = probe.split(sign_magnitude_encode(value));
    const std::array<uint8_t, 7> record = ServoBus::position_record(30, value, 500);
    EXPECT_EQ(record[1], library[0]) << value;
    EXPECT_EQ(record[2], library[1]) << value;
    EXPECT_EQ(ServoBus::speed_record(value), library) << value;
  }
  // and the position record's goal-speed field, which carries a plain magnitude through the same
  // split rather than a sign-magnitude word
  const std::array<uint8_t, 2> speed_halves = probe.split(1234);
  EXPECT_EQ(ServoBus::position_record(30, 0, 1234)[5], speed_halves[0]);
  EXPECT_EQ(ServoBus::position_record(30, 0, 1234)[6], speed_halves[1]);
}

TEST(ServoBus, max_records_per_packet_matches_the_two_hundred_and_fifty_five_byte_buffer)
{
  // PHASE3 4.T8 / R18. The helper IS the arithmetic the two shipped constants are defined from,
  // so the literals here are the only independent entry: they are what catches the derivation
  // itself going wrong (drop the `+ 1` for the id byte and 30 becomes 35).
  EXPECT_EQ(ServoBus::max_records_per_packet(ServoBus::goal_position_record_bytes), size_t{30});
  EXPECT_EQ(ServoBus::max_records_per_packet(ServoBus::goal_speed_record_bytes), size_t{82});
  EXPECT_EQ(ServoBus::max_goal_positions_per_packet, size_t{30});
  EXPECT_EQ(ServoBus::max_goal_speeds_per_packet, size_t{82});
  // The overflow is asserted, never executed: SCSerial::writeSCS is a bare txBuf[txBufLen++]
  // with no bounds check (src/SCSerial.cpp:179-191), so building a 31-record packet against the
  // real buffer is undefined behaviour, not a measurement (probe/out_p2_offline.txt Q4).
  EXPECT_GT(size_t{8 + 31 * 8}, ServoBus::tx_buffer_bytes);
  EXPECT_GT(size_t{8 + 83 * 3}, ServoBus::tx_buffer_bytes);
}

TEST(ServoBus, the_chunk_limits_are_the_largest_packets_that_fit_the_transmit_buffer)
{
  // PHASE3 1.31.1, double entry: the measured literals on one side, the shipped constants on the
  // other, so a hand edit of either constant fails here as well as at the static_asserts of 1.13.
  EXPECT_EQ(size_t{248}, 8 + 30 * 8);
  EXPECT_EQ(size_t{256}, 8 + 31 * 8);
  EXPECT_EQ(size_t{254}, 8 + 82 * 3);
  EXPECT_EQ(size_t{257}, 8 + 83 * 3);

  const size_t position_overhead = ServoBus::goal_position_record_bytes + 1;
  const size_t speed_overhead = ServoBus::goal_speed_record_bytes + 1;
  EXPECT_LE(
    ServoBus::sync_write_overhead_bytes +
    ServoBus::max_goal_positions_per_packet * position_overhead, ServoBus::tx_buffer_bytes);
  EXPECT_GT(
    ServoBus::sync_write_overhead_bytes +
    (ServoBus::max_goal_positions_per_packet + 1) * position_overhead, ServoBus::tx_buffer_bytes);
  EXPECT_LE(
    ServoBus::sync_write_overhead_bytes +
    ServoBus::max_goal_speeds_per_packet * speed_overhead, ServoBus::tx_buffer_bytes);
  EXPECT_GT(
    ServoBus::sync_write_overhead_bytes +
    (ServoBus::max_goal_speeds_per_packet + 1) * speed_overhead, ServoBus::tx_buffer_bytes);

  // and the constant those four inequalities rest on is the vendored array itself
  TxBufProbe probe;
  EXPECT_EQ(ServoBus::tx_buffer_bytes, probe.bytes());
}

TEST(ServoBus, sign_magnitude_encode_puts_the_direction_in_bit_15)
{
  // PHASE3 1.10 / 1.31.2. Bit 15 is the direction flag and bits 0..14 the magnitude
  // (src/SMS_STS.cpp:58-61): goal -100 goes out as `64 80`, not `9c ff`.
  EXPECT_EQ(sign_magnitude_encode(0), 0x0000);
  EXPECT_EQ(sign_magnitude_encode(1), 0x0001);
  EXPECT_EQ(sign_magnitude_encode(-1), 0x8001);
  EXPECT_EQ(sign_magnitude_encode(100), 0x0064);
  EXPECT_EQ(sign_magnitude_encode(-100), 0x8064);
  EXPECT_EQ(sign_magnitude_encode(4095), 0x0fff);
  EXPECT_EQ(sign_magnitude_encode(-4095), 0x8fff);
  EXPECT_EQ(sign_magnitude_encode(32767), 0x7fff);
  EXPECT_EQ(sign_magnitude_encode(-32767), 0xffff);
  // -32768 saturates to magnitude 32767, so it shares -32767's encoding. The library instead
  // negates in s16 there (src/SMS_STS.cpp:59), which is signed overflow; this version is defined
  // over the whole int16_t domain, and the driver's clamp (src/waveshare_servos.cpp:1481) keeps
  // the input unreachable either way.
  EXPECT_EQ(sign_magnitude_encode(-32768), 0xffff);
}

TEST(ServoBus, every_write_status_has_a_name)
{
  // PHASE3 1.31.17, mirroring the discipline the BusStatus names already follow: a status with no
  // name reaches a log line as an empty string.
  const std::vector<WriteStatus> all = {
    WriteStatus::OK, WriteStatus::NOT_OPEN, WriteStatus::INVALID_ID};
  std::vector<std::string> names;
  for (const WriteStatus status : all) {
    const std::string name = to_string(status);
    EXPECT_FALSE(name.empty());
    names.push_back(name);
  }
  std::sort(names.begin(), names.end());
  EXPECT_EQ(std::unique(names.begin(), names.end()), names.end()) << "two statuses share a name";
}

// ---------------------------------------------------------------------------------------------
// PHASE3 C1 stage B: whole frames and chunking, over the capture seam of 1.30. The vendored
// SCS::syncWrite still assembles every frame -- header, mesLen, broadcast id and checksum are its
// code -- so these cases are about the record bytes, the chunk boundaries and the refusals.
// ---------------------------------------------------------------------------------------------

TEST(ServoBus, the_position_packet_is_byte_identical_to_sync_write_pos_ex)
{
  // PHASE3 4.T16 / F1. The measured 40 bytes of packets.md section 2.1, re-derived here: mesLen
  // 0x24 = (7+1)*4+4, register 41 = 0x29, record width 7, checksum 0x78.
  const std::vector<uint8_t> golden = {
    0xff, 0xff, 0xfe, 0x24, 0x83, 0x29, 0x07,
    0x01, 0x1e, 0xe8, 0x03, 0x00, 0x00, 0xf4, 0x01,
    0x02, 0x1e, 0xe8, 0x83, 0x00, 0x00, 0xf4, 0x01,
    0x03, 0x1e, 0x00, 0x00, 0x00, 0x00, 0xf4, 0x01,
    0x04, 0x1e, 0xff, 0x07, 0x00, 0x00, 0xf4, 0x01, 0x78};

  PacketCapture wrapper;
  ASSERT_TRUE(wrapper.is_open()) << "the seam needs a real descriptor: is_open() gates the write";
  const std::vector<GoalPosition> goals = {
    {1, 1000, 500, 30}, {2, -1000, 500, 30}, {3, 0, 500, 30}, {4, 2047, 500, 30}};
  const WriteResult result = wrapper.write_goal_positions(goals);
  EXPECT_TRUE(static_cast<bool>(result));
  EXPECT_EQ(result.packets, 1u);
  EXPECT_EQ(result.records, 4u);
  ASSERT_EQ(wrapper.frames.size(), 1u);
  EXPECT_EQ(wrapper.frames.front(), golden);

  // SyncWritePosEx rewrites its Position[] in place (src/SMS_STS.cpp:58-61), so the library
  // reference gets its own arrays.
  PacketCapture library;
  ASSERT_TRUE(library.is_open());
  std::array<uint8_t, 4> ids = {1, 2, 3, 4};
  std::array<int16_t, 4> positions = {1000, -1000, 0, 2047};
  std::array<uint16_t, 4> speeds = {500, 500, 500, 500};
  std::array<uint8_t, 4> accelerations = {30, 30, 30, 30};
  library.SyncWritePosEx(
    ids.data(), 4, positions.data(), speeds.data(), accelerations.data());
  EXPECT_EQ(library.frames, wrapper.frames);
}

TEST(ServoBus, write_goal_positions_emits_the_bytes_syncwriteposex_emits)
{
  // PHASE3 1.31.3 / 1.3. The eight input classes probe 2 compared, 23 of 23 matching
  // (probe/out_p2_offline.txt Q3a). A golden frame alone would pass if both paths drifted the same
  // way, and an A/B alone would pass if both were wrong; the suite carries both.
  struct Class
  {
    const char * name;
    std::vector<uint8_t> ids;
    std::vector<int16_t> positions;
    std::vector<uint16_t> speeds;
    std::vector<uint8_t> accelerations;
  };
  const std::vector<Class> classes = {
    {"positive goal", {1, 2, 3, 4}, {1000, 2047, 3000, 4095}, {500, 500, 500, 500},
      {30, 30, 30, 30}},
    {"negative goal", {1, 2, 3, 4}, {-1000, -2047, -3000, -4095}, {500, 500, 500, 500},
      {30, 30, 30, 30}},
    {"goal 0", {1, 2, 3, 4}, {0, 0, 0, 0}, {500, 500, 500, 500}, {30, 30, 30, 30}},
    {"speed 0", {1, 2, 3, 4}, {1000, -1000, 0, 4095}, {0, 0, 0, 0}, {30, 30, 30, 30}},
    {"ACC 0", {1, 2, 3, 4}, {1000, -1000, 0, 4095}, {500, 500, 500, 500}, {0, 0, 0, 0}},
    {"max 4095 pos/spd, ACC 255", {1, 2, 3, 4}, {4095, -4095, 4095, -4095},
      {4095, 4095, 4095, 4095}, {255, 255, 255, 255}},
    {"mixed", {1, 2, 3, 4}, {1000, -1000, 0, 4095}, {500, 0, 4095, 1}, {30, 0, 255, 1}},
    {"n = 1 negative", {1}, {-1234}, {777}, {7}},
  };

  for (const Class & one : classes) {
    PacketCapture wrapper;
    PacketCapture library;
    ASSERT_TRUE(wrapper.is_open()) << one.name;
    ASSERT_TRUE(library.is_open()) << one.name;

    std::vector<GoalPosition> goals;
    for (size_t i = 0; i < one.ids.size(); i++) {
      goals.push_back(
        GoalPosition{one.ids[i], one.positions[i], one.speeds[i], one.accelerations[i]});
    }
    EXPECT_TRUE(static_cast<bool>(wrapper.write_goal_positions(goals))) << one.name;

    std::vector<uint8_t> ids = one.ids;
    std::vector<int16_t> positions = one.positions;      // the library rewrites this one in place
    std::vector<uint16_t> speeds = one.speeds;
    std::vector<uint8_t> accelerations = one.accelerations;
    library.SyncWritePosEx(
      ids.data(), static_cast<uint8_t>(ids.size()), positions.data(), speeds.data(),
      accelerations.data());

    EXPECT_EQ(library.frames, wrapper.frames) << one.name;
  }
}

TEST(ServoBus, a_position_packet_matches_its_golden_bytes)
{
  // PHASE3 1.31.4. probe/out_p2_offline.txt Q3a "n=1 negative", checksum re-derived by hand:
  // 0xfe+0x0c+0x83+0x29+0x07+0x01+0x07+0xd2+0x84+0x00+0x00+0x09+0x03 = 0x327, low byte 0x27,
  // ~0x27 = 0xd8.
  PacketCapture capture;
  ASSERT_TRUE(capture.is_open());
  EXPECT_TRUE(static_cast<bool>(capture.write_goal_positions({GoalPosition{1, -1234, 777, 7}})));
  ASSERT_EQ(capture.frames.size(), 1u);
  EXPECT_EQ(
    capture.frames.front(),
    (std::vector<uint8_t>{
    0xff, 0xff, 0xfe, 0x0c, 0x83, 0x29, 0x07, 0x01, 0x07, 0xd2, 0x84, 0x00, 0x00, 0x09, 0x03,
    0xd8}));
}

TEST(ServoBus, the_speed_packet_is_byte_identical_to_the_sync_stage_of_sync_write_spe)
{
  // PHASE3 4.T17 / 1.31.5 / 1.31.7 / F2, and the whole of Phase 3 item 1 in one assertion: the
  // sync frame is unchanged and the four per-servo ACC round trips are gone. Measured sync stage:
  // probe/out_p2_offline.txt Q3b "ACC 0", checksum 0xd4.
  PacketCapture wrapper;
  ASSERT_TRUE(wrapper.is_open());
  EXPECT_TRUE(
    static_cast<bool>(
      wrapper.write_goal_speeds(
        {GoalSpeed{1, 700}, GoalSpeed{2, -700}, GoalSpeed{3, 0}, GoalSpeed{4, 100}})));
  ASSERT_EQ(wrapper.frames.size(), 1u) << "one broadcast, and no per-servo ACC write at all";
  EXPECT_EQ(
    wrapper.frames.front(),
    (std::vector<uint8_t>{
    0xff, 0xff, 0xfe, 0x10, 0x83, 0x2e, 0x02, 0x01, 0xbc, 0x02, 0x02, 0xbc, 0x82, 0x03, 0x00,
    0x00, 0x04, 0x64, 0x00, 0xd4}));

  // SyncWriteSpe dereferences ACC[i] unconditionally (src/SMS_STS.cpp:270), so the comparison call
  // must pass a real array and never nullptr.
  PacketCapture library;
  ASSERT_TRUE(library.is_open());
  std::array<uint8_t, 4> ids = {1, 2, 3, 4};
  std::array<int16_t, 4> speeds = {700, -700, 0, 100};
  std::array<uint8_t, 4> accelerations = {0, 0, 0, 0};
  library.SyncWriteSpe(ids.data(), 4, speeds.data(), accelerations.data());
  ASSERT_EQ(library.frames.size(), 5u) << "4 addressed writes of register 41 plus the sync frame";
  EXPECT_EQ(library.frames.back(), wrapper.frames.front());
}

TEST(ServoBus, write_goal_speeds_emits_the_sync_stage_syncwritespe_emits)
{
  // PHASE3 1.31.5 / 1.3, the speed path's counterpart to the eight position classes: the same
  // seven inputs probe 2 compared, 7 of 7 matching (probe/out_p2_offline.txt Q3b). Only the
  // INST_SYNC_WRITE frame is compared -- the library's other n frames are its per-servo ACC
  // writes, and that they are gone is what the frame count of 4.T17 asserts.
  struct Class
  {
    const char * name;
    std::vector<int16_t> speeds;
    std::vector<uint8_t> accelerations;
  };
  const std::vector<Class> classes = {
    {"positive speeds", {700, 1000, 1500, 100}, {30, 30, 30, 30}},
    {"negative speeds", {-700, -1000, -1500, -100}, {30, 30, 30, 30}},
    {"speed 0", {0, 0, 0, 0}, {30, 30, 30, 30}},
    {"ACC 0", {700, -700, 0, 100}, {0, 0, 0, 0}},
    {"max 4095 / ACC 255", {4095, -4095, 4095, -4095}, {255, 255, 255, 255}},
    {"mixed sign+zero", {700, -700, 0, 100}, {30, 0, 255, 1}},
    {"n = 1", {-1500}, {200}},
  };

  for (const Class & one : classes) {
    PacketCapture wrapper;
    PacketCapture library;
    ASSERT_TRUE(wrapper.is_open()) << one.name;
    ASSERT_TRUE(library.is_open()) << one.name;

    std::vector<GoalSpeed> goals;
    for (size_t i = 0; i < one.speeds.size(); i++) {
      goals.push_back(GoalSpeed{static_cast<uint8_t>(i + 1), one.speeds[i]});
    }
    EXPECT_TRUE(static_cast<bool>(wrapper.write_goal_speeds(goals))) << one.name;
    ASSERT_EQ(wrapper.frames.size(), 1u) << one.name;

    std::vector<uint8_t> ids;
    for (size_t i = 0; i < one.speeds.size(); i++) {
      ids.push_back(static_cast<uint8_t>(i + 1));
    }
    std::vector<int16_t> speeds = one.speeds;          // the library rewrites this one in place
    std::vector<uint8_t> accelerations = one.accelerations;
    library.SyncWriteSpe(
      ids.data(), static_cast<uint8_t>(ids.size()), speeds.data(), accelerations.data());
    ASSERT_EQ(library.frames.size(), ids.size() + 1) << one.name;
    EXPECT_EQ(library.frames.back(), wrapper.frames.front()) << one.name;
  }
}

TEST(ServoBus, a_speed_packet_matches_its_golden_bytes)
{
  // PHASE3 1.31.6, the speed path's golden frame -- the counterpart of 1.31.4, and there for the
  // same reason: an A/B alone passes if both paths drift the same way. probe/out_p2_offline.txt
  // Q3b "n=1", checksum re-derived by hand: 0xfe+0x07+0x83+0x2e+0x02+0x01+0xdc+0x85 = 0x31a, low
  // byte 0x1a, ~0x1a = 0xe5.
  PacketCapture capture;
  ASSERT_TRUE(capture.is_open());
  EXPECT_TRUE(static_cast<bool>(capture.write_goal_speeds({GoalSpeed{1, -1500}})));
  ASSERT_EQ(capture.frames.size(), 1u);
  EXPECT_EQ(
    capture.frames.front(),
    (std::vector<uint8_t>{0xff, 0xff, 0xfe, 0x07, 0x83, 0x2e, 0x02, 0x01, 0xdc, 0x85, 0xe5}));
}

TEST(ServoBus, the_wrapper_built_packet_is_identical_on_a_second_call_with_the_same_inputs)
{
  // PHASE3 4.T18 / 1.31.9 / 1.11. The library re-encodes its caller's array in place, so the
  // second call with an unchanged array emits a DIFFERENT frame; the wrapper reads its input and
  // writes only its own scratch, so it cannot.
  PacketCapture wrapper;
  ASSERT_TRUE(wrapper.is_open());
  const std::vector<GoalPosition> goals = {{1, -1234, 777, 7}, {2, 2048, 0, 0}};
  wrapper.write_goal_positions(goals);
  wrapper.write_goal_positions(goals);
  ASSERT_EQ(wrapper.frames.size(), 2u);
  EXPECT_EQ(wrapper.frames[0], wrapper.frames[1]);

  PacketCapture wheels;
  ASSERT_TRUE(wheels.is_open());
  const std::vector<GoalSpeed> wheel_goals = {{1, 700}, {2, -700}};
  wheels.write_goal_speeds(wheel_goals);
  wheels.write_goal_speeds(wheel_goals);
  ASSERT_EQ(wheels.frames.size(), 2u);
  EXPECT_EQ(
    wheels.frames[0],
    (std::vector<uint8_t>{
    0xff, 0xff, 0xfe, 0x0a, 0x83, 0x2e, 0x02, 0x01, 0xbc, 0x02, 0x02, 0xbc, 0x82, 0x45}));
  EXPECT_EQ(wheels.frames[0], wheels.frames[1]);

  // The contrast that explains why the clause exists: {700, -700} becomes {700, -32068} inside
  // SyncWriteSpe, so its second frame carries `44 fd` (measured, probe/out_p2_offline.txt Q3c).
  PacketCapture library;
  ASSERT_TRUE(library.is_open());
  std::array<uint8_t, 2> ids = {1, 2};
  std::array<int16_t, 2> speeds = {700, -700};
  std::array<uint8_t, 2> accelerations = {0, 0};
  library.SyncWriteSpe(ids.data(), 2, speeds.data(), accelerations.data());
  library.SyncWriteSpe(ids.data(), 2, speeds.data(), accelerations.data());
  ASSERT_EQ(library.frames.size(), 6u);   // (2 ACC writes + 1 sync) twice
  EXPECT_EQ(library.frames[2], wheels.frames[0]);
  EXPECT_EQ(
    library.frames[5], (std::vector<uint8_t>{
    0xff, 0xff, 0xfe, 0x0a, 0x83, 0x2e, 0x02, 0x01, 0xbc, 0x02, 0x02, 0x44, 0xfd, 0x42}));
  EXPECT_NE(library.frames[2], library.frames[5]);
}

TEST(ServoBus, write_goal_positions_does_not_modify_its_inputs)
{
  // PHASE3 4.T19 / 1.11 / 1.31.8. This is the property that makes the driver's per-cycle refill of
  // p_pos_ar_ (src/waveshare_servos.cpp:1481) optional rather than load-bearing. Compared field by
  // field because GoalPosition deliberately carries no operator== of its own.
  PacketCapture capture;
  ASSERT_TRUE(capture.is_open());
  const std::vector<GoalPosition> before = {{1, -1234, 777, 7}, {2, 2048, 0, 0}};
  std::vector<GoalPosition> goals = before;
  capture.write_goal_positions(goals);
  ASSERT_EQ(goals.size(), before.size());
  for (size_t i = 0; i < goals.size(); i++) {
    EXPECT_EQ(goals[i].id, before[i].id) << i;
    EXPECT_EQ(goals[i].position, before[i].position) << i;
    EXPECT_EQ(goals[i].speed, before[i].speed) << i;
    EXPECT_EQ(goals[i].acc, before[i].acc) << i;
  }

  const std::vector<GoalSpeed> wheels_before = {{3, -700}, {4, 0}};
  std::vector<GoalSpeed> wheels = wheels_before;
  capture.write_goal_speeds(wheels);
  ASSERT_EQ(wheels.size(), wheels_before.size());
  for (size_t i = 0; i < wheels.size(); i++) {
    EXPECT_EQ(wheels[i].id, wheels_before[i].id) << i;
    EXPECT_EQ(wheels[i].speed, wheels_before[i].speed) << i;
  }
}

TEST(ServoBus, thirty_one_position_records_go_out_as_two_packets_of_thirty_and_one)
{
  // PHASE3 4.T20 / 1.14. 31 records in one frame would be 256 bytes, one past the end of txBuf
  // (probe/out_p2_offline.txt Q4).
  PacketCapture capture;
  ASSERT_TRUE(capture.is_open());
  std::vector<GoalPosition> goals;
  for (uint8_t id = 1; id <= 31; id++) {
    goals.push_back(GoalPosition{id, static_cast<int16_t>(id * 10), 500, 30});
  }
  const WriteResult result = capture.write_goal_positions(goals);
  EXPECT_TRUE(static_cast<bool>(result));
  EXPECT_EQ(result.packets, 2u);
  EXPECT_EQ(result.records, 31u);
  ASSERT_EQ(capture.frames.size(), 2u);
  EXPECT_EQ(capture.frames[0].size(), 248u);
  EXPECT_EQ(capture.frames[0][3], 0xf4);   // mesLen = (7+1)*30 + 4
  EXPECT_EQ(capture.frames[1].size(), 16u);

  std::vector<uint8_t> seen;
  for (const std::vector<uint8_t> & frame : capture.frames) {
    const ParsedSyncWrite parsed = parse_sync_write(frame);
    ASSERT_TRUE(parsed.well_formed);
    EXPECT_EQ(parsed.address, 41);
    EXPECT_EQ(parsed.record_bytes, ServoBus::goal_position_record_bytes);
    seen.insert(seen.end(), parsed.ids.begin(), parsed.ids.end());
  }
  std::vector<uint8_t> expected;
  for (uint8_t id = 1; id <= 31; id++) {
    expected.push_back(id);
  }
  EXPECT_EQ(seen, expected) << "the caller's order is preserved across the chunk boundary";
}

TEST(ServoBus, write_goal_positions_chunks_at_thirty_records)
{
  // PHASE3 1.31.10. 61 goals is 30 + 30 + 1: the last chunk is short, never padded, and each
  // chunk is an independently valid frame with its own checksum, computed here rather than taken
  // from the library that built it.
  PacketCapture capture;
  ASSERT_TRUE(capture.is_open());
  std::vector<GoalPosition> goals;
  for (uint8_t id = 1; id <= 61; id++) {
    goals.push_back(GoalPosition{id, static_cast<int16_t>(id), 500, 30});
  }
  const WriteResult result = capture.write_goal_positions(goals);
  EXPECT_EQ(result.packets, 3u);
  EXPECT_EQ(result.records, 61u);
  ASSERT_EQ(capture.frames.size(), 3u);
  EXPECT_EQ(capture.frames[0].size(), 248u);
  EXPECT_EQ(capture.frames[1].size(), 248u);
  EXPECT_EQ(capture.frames[2].size(), 16u);

  const std::vector<size_t> expected_counts = {30, 30, 1};
  std::vector<uint8_t> seen;
  for (size_t f = 0; f < capture.frames.size(); f++) {
    const ParsedSyncWrite parsed = parse_sync_write(capture.frames[f]);
    ASSERT_TRUE(parsed.well_formed) << "frame " << f;
    EXPECT_EQ(parsed.ids.size(), expected_counts[f]) << "frame " << f;
    EXPECT_EQ(
      capture.frames[f][3],
      static_cast<uint8_t>((ServoBus::goal_position_record_bytes + 1) * parsed.ids.size() + 4));
    seen.insert(seen.end(), parsed.ids.begin(), parsed.ids.end());
  }
  std::vector<uint8_t> expected;
  for (uint8_t id = 1; id <= 61; id++) {
    expected.push_back(id);
  }
  EXPECT_EQ(seen, expected);
}

TEST(ServoBus, eighty_three_speed_records_go_out_as_two_packets_of_eighty_two_and_one)
{
  // PHASE3 4.T21. 83 records in one frame would be 257 bytes, two past the end of txBuf.
  PacketCapture capture;
  ASSERT_TRUE(capture.is_open());
  std::vector<GoalSpeed> goals;
  for (uint8_t id = 1; id <= 83; id++) {
    goals.push_back(GoalSpeed{id, static_cast<int16_t>(-id)});
  }
  const WriteResult result = capture.write_goal_speeds(goals);
  EXPECT_TRUE(static_cast<bool>(result));
  EXPECT_EQ(result.packets, 2u);
  EXPECT_EQ(result.records, 83u);
  ASSERT_EQ(capture.frames.size(), 2u);
  EXPECT_EQ(capture.frames[0].size(), 254u);
  EXPECT_EQ(capture.frames[0][3], 0xfa);   // mesLen = (2+1)*82 + 4
  EXPECT_EQ(capture.frames[1].size(), 11u);

  std::vector<uint8_t> seen;
  for (const std::vector<uint8_t> & frame : capture.frames) {
    const ParsedSyncWrite parsed = parse_sync_write(frame);
    ASSERT_TRUE(parsed.well_formed);
    EXPECT_EQ(parsed.address, 46);
    EXPECT_EQ(parsed.record_bytes, ServoBus::goal_speed_record_bytes);
    seen.insert(seen.end(), parsed.ids.begin(), parsed.ids.end());
  }
  std::vector<uint8_t> expected;
  for (uint8_t id = 1; id <= 83; id++) {
    expected.push_back(id);
  }
  EXPECT_EQ(seen, expected);
}

TEST(ServoBus, write_goal_speeds_chunks_at_eighty_two_records)
{
  // PHASE3 1.31.11. 165 goals is 82 + 82 + 1.
  PacketCapture capture;
  ASSERT_TRUE(capture.is_open());
  std::vector<GoalSpeed> goals;
  for (int id = 1; id <= 165; id++) {
    goals.push_back(GoalSpeed{static_cast<uint8_t>(id), static_cast<int16_t>(id)});
  }
  const WriteResult result = capture.write_goal_speeds(goals);
  EXPECT_EQ(result.packets, 3u);
  EXPECT_EQ(result.records, 165u);
  ASSERT_EQ(capture.frames.size(), 3u);
  EXPECT_EQ(capture.frames[0].size(), 254u);
  EXPECT_EQ(capture.frames[1].size(), 254u);
  EXPECT_EQ(capture.frames[2].size(), 11u);

  const std::vector<size_t> expected_counts = {82, 82, 1};
  std::vector<uint8_t> seen;
  for (size_t f = 0; f < capture.frames.size(); f++) {
    const ParsedSyncWrite parsed = parse_sync_write(capture.frames[f]);
    ASSERT_TRUE(parsed.well_formed) << "frame " << f;
    EXPECT_EQ(parsed.ids.size(), expected_counts[f]) << "frame " << f;
    EXPECT_EQ(
      capture.frames[f][3],
      static_cast<uint8_t>((ServoBus::goal_speed_record_bytes + 1) * parsed.ids.size() + 4));
    seen.insert(seen.end(), parsed.ids.begin(), parsed.ids.end());
  }
  std::vector<uint8_t> expected;
  for (int id = 1; id <= 165; id++) {
    expected.push_back(static_cast<uint8_t>(id));
  }
  EXPECT_EQ(seen, expected);
}

TEST(ServoBus, an_empty_group_puts_nothing_on_the_wire)
{
  // PHASE3 4.T23 / 1.16 / R13. `u8 offbuf[IDN][7]` is a zero-length VLA at IDN == 0
  // (src/SMS_STS.cpp:56, :263, undefined behaviour) and a bare syncWrite would broadcast
  // mesLen = 4 with no records. The driver's two guards move here, and this is what pins them.
  PacketCapture capture;
  ASSERT_TRUE(capture.is_open());
  const WriteResult positions = capture.write_goal_positions({});
  const WriteResult speeds = capture.write_goal_speeds({});
  EXPECT_TRUE(static_cast<bool>(positions));
  EXPECT_TRUE(static_cast<bool>(speeds));
  EXPECT_EQ(positions.packets, 0u);
  EXPECT_EQ(positions.records, 0u);
  EXPECT_EQ(speeds.packets, 0u);
  EXPECT_EQ(speeds.records, 0u);
  EXPECT_TRUE(capture.frames.empty());
}

TEST(ServoBus, the_position_record_carries_acc_first_and_a_zero_goal_time)
{
  // PHASE3 1.31.21 / 1.8, read straight out of a captured frame rather than out of the builder,
  // so the record's place inside the packet is pinned too.
  PacketCapture capture;
  ASSERT_TRUE(capture.is_open());
  capture.write_goal_positions({GoalPosition{1, -2000, 300, 77}, GoalPosition{2, 2000, 300, 0}});
  ASSERT_EQ(capture.frames.size(), 1u);
  const ParsedSyncWrite parsed = parse_sync_write(capture.frames.front());
  ASSERT_TRUE(parsed.well_formed);
  ASSERT_EQ(parsed.records.size(), 2u);
  EXPECT_EQ(parsed.address, 41) << "the record is based at SMS_STS_ACC, not at GOAL_POSITION";

  EXPECT_EQ(parsed.records[0][0], 77);
  EXPECT_EQ(parsed.records[1][0], 0);
  for (const std::vector<uint8_t> & record : parsed.records) {
    EXPECT_EQ(record[3], 0x00);   // GOAL_TIME_L, hardcoded 0 by the library too
    EXPECT_EQ(record[4], 0x00);   // GOAL_TIME_H
    // The goal speed is an unsigned magnitude on this path: 300 = 0x012c, and bit 15 stays clear
    // even for the servo whose goal position is negative.
    EXPECT_EQ(record[5], 0x2c);
    EXPECT_EQ(record[6], 0x01);
  }
  EXPECT_EQ(parsed.records[0][2] & 0x80, 0x80) << "the direction bit rides on the goal position";
  EXPECT_EQ(parsed.records[1][2] & 0x80, 0x00);
}

TEST(ServoBus, a_zero_wheel_speed_is_two_zero_bytes)
{
  // PHASE3 1.31.22 / 1.9 / F2: the stop command every deactivation depends on. `00 80` -- a signed
  // zero -- would be a direction flag with no magnitude, which is not what the servo is told today.
  PacketCapture capture;
  ASSERT_TRUE(capture.is_open());
  capture.write_goal_speeds({GoalSpeed{3, 0}});
  ASSERT_EQ(capture.frames.size(), 1u);
  const ParsedSyncWrite parsed = parse_sync_write(capture.frames.front());
  ASSERT_TRUE(parsed.well_formed);
  ASSERT_EQ(parsed.records.size(), 1u);
  EXPECT_EQ(parsed.records[0], (std::vector<uint8_t>{0x00, 0x00}));
}

TEST(ServoBus, write_goal_positions_on_a_closed_bus_writes_nothing)
{
  // PHASE3 1.17 / 1.31.15. Today the same call reaches wFlushSCS() -> write(-1, ...) -> EBADF and
  // silently does nothing; the explicit refusal exists so the caller can log it, because reaching
  // the write path with no port is a driver bug rather than an operating condition.
  ServoBus fresh;
  ASSERT_FALSE(fresh.is_open());
  const WriteResult refused = fresh.write_goal_positions({GoalPosition{1, 0, 0, 0}});
  EXPECT_EQ(refused.status, WriteStatus::NOT_OPEN);
  EXPECT_EQ(refused.packets, 0u);
  EXPECT_EQ(refused.records, 0u);
  EXPECT_FALSE(static_cast<bool>(refused));
  EXPECT_EQ(fresh.write_goal_speeds({GoalSpeed{1, 100}}).status, WriteStatus::NOT_OPEN);
  EXPECT_FALSE(fresh.write_acc(1, 40));

  // The second arm 1.31.15 names: a bus that HELD a descriptor and lost it. It is a separate
  // assertion because the two failures have different causes -- the first is a bus that was never
  // given a port, this one is close() putting fd back to -1 (src/SCSerial.cpp:224-227). The seam
  // supplies the descriptor because the refusal is decided by is_open() alone, and /dev/null is
  // opened and closed for real here.
  PacketCapture opened_then_closed;
  ASSERT_TRUE(opened_then_closed.is_open());
  opened_then_closed.close();
  ASSERT_FALSE(opened_then_closed.is_open());
  EXPECT_EQ(
    opened_then_closed.write_goal_positions({GoalPosition{1, 0, 0, 0}}).status,
    WriteStatus::NOT_OPEN);
  EXPECT_EQ(
    opened_then_closed.write_goal_speeds({GoalSpeed{1, 100}}).status, WriteStatus::NOT_OPEN);
  EXPECT_FALSE(opened_then_closed.write_acc(1, 40));
  EXPECT_TRUE(opened_then_closed.frames.empty()) << "a refusal builds no frame at all";
}

TEST_F(ServoBusFakeBus, write_acc_writes_register_41_and_reports_the_ack)
{
  // PHASE3 1.31.18 / 1.20 / F3. The ACC byte leaves the per-cycle sync write and travels in its
  // own addressed INST_WRITE instead; only the schedule changes, not the byte. This is the case
  // that CANNOT tell `!= 0` from the `!= -1` bug -- a successful write returns 1, which is neither
  // -- which is why 1.31.19 is the one that has to be written first.
  const waveshare_servos_test::FakeServo before = fake_.snapshot(1);
  EXPECT_TRUE(bus_.write_acc(1, 77));
  const waveshare_servos_test::FakeServo after = fake_.snapshot(1);
  EXPECT_EQ(after.mem[waveshare_servos_test::kRegAcc], 77);
  EXPECT_EQ(after.writes, before.writes + 1) << "one addressed write, and no EPROM unlock";
  // PHASE3 4.T37 / F6. A plain addressed INST_WRITE: not a sync write, and NOT the unLockEprom /
  // LockEprom pair set_mode has to bracket register 33 with (src/waveshare_servos.cpp:988-990).
  // Register 41 is SRAM (include/SMS_STS.h:39-48), so that pair would be two extra round trips
  // and two writes of an EPROM register (55) that Phase 3 promises never to touch.
  EXPECT_EQ(after.sync_writes.size(), before.sync_writes.size());
  EXPECT_EQ(after.mem[55], before.mem[55]) << "SMS_STS_LOCK must not be written";
  EXPECT_EQ(after.writes, 1) << "exactly one transaction, so no unlock/lock bracket";
}

TEST_F(ServoBusPty, write_acc_reports_a_servo_that_does_not_answer)
{
  // PHASE3 1.31.19 / 1.20. Nothing answers on this fixture's pty, so SCS::Ack reads six bytes that
  // never arrive and returns 0 (src/SCS.cpp:265-295). `!= 0` is the whole point: Ack returns 0 on
  // every failure path and 1 on success and NEVER -1, so the read side's `!= -1` convention
  // (readByte, src/SCS.cpp:206-215) would make this function always return true and 1.25's
  // unramped-wheel warning unreachable.
  ASSERT_TRUE(static_cast<bool>(bus_.open(port_, kBaudrate, 25)));
  const auto started = std::chrono::steady_clock::now();
  EXPECT_FALSE(bus_.write_acc(9, 40));
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - started);

  // Banded exactly like a_silent_bus_costs_one_io_timeout: one silent transaction costs one
  // select() timeout, not a retry loop of them.
  EXPECT_GE(elapsed.count(), 20);
  EXPECT_LT(elapsed.count(), 250) << "one silent ACC write must cost one timeout, not many";
}

TEST_F(ServoBusPty, a_full_chunk_fills_the_transmit_buffer_without_overrunning_it)
{
  // PHASE3 4.T22 / 1.31.12 / F7. The GENUINE txBufLen the vendored writeSCS produced, read before
  // wFlushSCS zeroes it (src/SCSerial.cpp:218) -- not the wrapper's own arithmetic. The four
  // numbers are the ones measured on a pty in probe/out_p2_offline.txt.
  //
  // 31 position records (256 bytes) and 83 speed records (257) are deliberately absent and must
  // stay absent: writeSCS has no bounds check, so building them against the real buffer is
  // undefined behaviour, not a measurement. The capture seam, which never touches txBuf, is where
  // the oversized cases live.
  TxBufLenProbe probe;
  ASSERT_TRUE(static_cast<bool>(probe.open(port_, kBaudrate, kIoTimeoutMs)));

  std::vector<GoalPosition> positions;
  for (uint8_t id = 1; id <= 30; id++) {
    positions.push_back(GoalPosition{id, 100, 500, 30});
  }
  std::vector<GoalSpeed> speeds;
  for (uint8_t id = 1; id <= 82; id++) {
    speeds.push_back(GoalSpeed{id, 100});
  }

  // Only these four frames, and never more: the fixture does not drain the master, so a case that
  // wrote many full-size frames would start exercising wFlushSCS's EAGAIN retry loop instead of
  // the chunker.
  probe.write_goal_positions(
    std::vector<GoalPosition>(positions.begin(), positions.begin() + 29));
  probe.write_goal_positions(positions);
  probe.write_goal_speeds(std::vector<GoalSpeed>(speeds.begin(), speeds.begin() + 81));
  probe.write_goal_speeds(speeds);

  EXPECT_EQ(probe.lengths, (std::vector<int>{240, 248, 251, 254}));
  for (const int length : probe.lengths) {
    EXPECT_LE(static_cast<size_t>(length), ServoBus::tx_buffer_bytes);
  }
  probe.close();
}

TEST_F(ServoBusPty, write_goal_positions_on_an_empty_list_sends_nothing)
{
  // PHASE3 1.16 / 1.31.13. Emptiness is checked BEFORE the port, because "nothing to send" is
  // success whatever the port is doing: stop_and_park prunes p_ids_ to the servos that answered
  // and legitimately leaves it empty, and a wheels-only robot has an empty position group from
  // the start.
  ServoBus never_opened;
  ASSERT_FALSE(never_opened.is_open());
  EXPECT_EQ(never_opened.write_goal_positions({}).status, WriteStatus::OK);
  EXPECT_EQ(never_opened.write_goal_speeds({}).status, WriteStatus::OK);

  ASSERT_TRUE(static_cast<bool>(bus_.open(port_, kBaudrate, kIoTimeoutMs)));
  const WriteResult positions = bus_.write_goal_positions({});
  const WriteResult speeds = bus_.write_goal_speeds({});
  EXPECT_TRUE(static_cast<bool>(positions));
  EXPECT_TRUE(static_cast<bool>(speeds));
  EXPECT_EQ(positions.packets, 0u);
  EXPECT_EQ(speeds.packets, 0u);
  EXPECT_THAT(collect_master_bytes(20), ::testing::IsEmpty());
}

TEST_F(ServoBusPty, reserve_goal_capacity_stops_the_write_path_allocating)
{
  // PHASE3 1.7 / 1.31.14. The real-time path must allocate nothing after build_groups() has sized
  // the scratch, and the reserve clamps at the per-packet maxima so a hundred-servo group reserves
  // one chunk rather than a hundred records.
  ASSERT_TRUE(static_cast<bool>(bus_.open(port_, kBaudrate, kIoTimeoutMs)));
  bus_.reserve_goal_capacity(30, 82);
  const size_t reserved = bus_.goal_scratch_capacity_bytes();
  EXPECT_GT(reserved, 0u);

  // The hundred cycles put about 23 kB on the wire, several times what a pty holds. Draining the
  // master every cycle is not tidiness: an undrained pty makes wFlushSCS spin on EAGAIN and then
  // drop the rest of the frame silently (src/SCSerial.cpp:205-218), so the back of the loop would
  // be exercising that retry budget instead of the chunker (PHASE3 1.30). `expected` and `drained`
  // are what prove the drain kept up -- every byte the chunker built arrived.
  size_t expected = 0;
  size_t drained = 0;
  for (int cycle = 0; cycle < 100; cycle++) {
    std::vector<GoalPosition> positions;
    for (int i = 0; i < cycle % 31; i++) {
      positions.push_back(GoalPosition{static_cast<uint8_t>(i + 1), 100, 500, 30});
    }
    std::vector<GoalSpeed> speeds;
    for (int i = 0; i < cycle % 83; i++) {
      speeds.push_back(GoalSpeed{static_cast<uint8_t>(i + 1), 100});
    }
    bus_.write_goal_positions(positions);
    bus_.write_goal_speeds(speeds);
    if (!positions.empty()) {
      expected += ServoBus::sync_write_overhead_bytes +
        positions.size() * (ServoBus::goal_position_record_bytes + 1);
    }
    if (!speeds.empty()) {
      expected += ServoBus::sync_write_overhead_bytes +
        speeds.size() * (ServoBus::goal_speed_record_bytes + 1);
    }
    drained += drain_master();
    ASSERT_EQ(bus_.goal_scratch_capacity_bytes(), reserved) << "cycle " << cycle;
  }
  EXPECT_EQ(drained, expected) << "the pty swallowed a frame: the case was testing wFlushSCS";

  bus_.reserve_goal_capacity(1000, 1000);
  EXPECT_EQ(bus_.goal_scratch_capacity_bytes(), reserved) << "the reserve clamps at one chunk";
}

TEST_F(ServoBusPty, an_id_outside_one_to_two_hundred_fifty_three_is_refused_whole)
{
  // PHASE3 1.18 / 1.31.16. All or nothing, and before a single byte is built: 254 is the broadcast
  // address the frame header itself carries and 255 is the header byte, so a record addressed to
  // either is nonsense the servos would misparse. on_init already rejects both, which makes this a
  // tripwire rather than the primary gate -- and a partial command set would hide the driver bug
  // behind plausible motion.
  ASSERT_TRUE(static_cast<bool>(bus_.open(port_, kBaudrate, kIoTimeoutMs)));
  for (const uint8_t bad : {uint8_t{254}, uint8_t{0}, uint8_t{255}}) {
    const std::vector<GoalPosition> goals = {
      {1, 100, 500, 30}, {2, 100, 500, 30}, {bad, 100, 500, 30}, {3, 100, 500, 30}};
    const WriteResult result = bus_.write_goal_positions(goals);
    EXPECT_EQ(result.status, WriteStatus::INVALID_ID) << static_cast<int>(bad);
    EXPECT_EQ(result.first_bad, 2u) << static_cast<int>(bad);
    EXPECT_EQ(result.packets, 0u) << static_cast<int>(bad);
    EXPECT_FALSE(static_cast<bool>(result)) << static_cast<int>(bad);

    const WriteResult wheels = bus_.write_goal_speeds({{1, 100}, {bad, 100}});
    EXPECT_EQ(wheels.status, WriteStatus::INVALID_ID) << static_cast<int>(bad);
    EXPECT_EQ(wheels.first_bad, 1u) << static_cast<int>(bad);
    EXPECT_EQ(wheels.packets, 0u) << static_cast<int>(bad);
  }
  EXPECT_THAT(collect_master_bytes(20), ::testing::IsEmpty()) <<
    "a refusal must not leave a three-record packet on the wire";
}

TEST_F(ServoBusFakeBus, the_records_reach_the_right_servos_in_request_order)
{
  // PHASE3 1.31.20, end to end against a register file: ids 1 and 2 are position servos, 3 and 4
  // are wheels. A sync write is a broadcast and is never acked (src/SCS.cpp:132, :268), so the
  // bytes may still be in the kernel's queue when write_goal_* returns -- wait_quiet() is what
  // makes reading the records back deterministic rather than a race that loses the last one.
  ASSERT_TRUE(static_cast<bool>(bus_.write_goal_positions({{1, 1000, 500, 30}, {2, -1000, 0, 0}})));
  ASSERT_TRUE(static_cast<bool>(bus_.write_goal_speeds({{3, 700}, {4, -700}})));
  fake_.wait_quiet();

  const waveshare_servos_test::FakeServo first = fake_.snapshot(1);
  ASSERT_EQ(first.sync_writes.size(), 1u);
  EXPECT_EQ(first.sync_writes[0].first, waveshare_servos_test::kRegAcc);
  EXPECT_EQ(
    first.sync_writes[0].second,
    (std::vector<uint8_t>{0x1e, 0xe8, 0x03, 0x00, 0x00, 0xf4, 0x01}));

  const waveshare_servos_test::FakeServo second = fake_.snapshot(2);
  ASSERT_EQ(second.sync_writes.size(), 1u);
  EXPECT_EQ(second.sync_writes[0].first, waveshare_servos_test::kRegAcc);
  EXPECT_EQ(
    second.sync_writes[0].second,
    (std::vector<uint8_t>{0x00, 0xe8, 0x83, 0x00, 0x00, 0x00, 0x00}));

  const waveshare_servos_test::FakeServo third = fake_.snapshot(3);
  ASSERT_EQ(third.sync_writes.size(), 1u);
  EXPECT_EQ(third.sync_writes[0].first, waveshare_servos_test::kRegGoalSpeed);
  EXPECT_EQ(third.sync_writes[0].second, (std::vector<uint8_t>{0xbc, 0x02}));

  const waveshare_servos_test::FakeServo fourth = fake_.snapshot(4);
  ASSERT_EQ(fourth.sync_writes.size(), 1u);
  EXPECT_EQ(fourth.sync_writes[0].first, waveshare_servos_test::kRegGoalSpeed);
  EXPECT_EQ(fourth.sync_writes[0].second, (std::vector<uint8_t>{0xbc, 0x82}));

  // and the records really landed in the registers they name
  EXPECT_EQ(fake_.word(1, waveshare_servos_test::kRegGoalPosition), 1000);
  EXPECT_EQ(fake_.word(2, waveshare_servos_test::kRegGoalPosition), -1000);
  EXPECT_EQ(fake_.word(3, waveshare_servos_test::kRegGoalSpeed), 700);
  EXPECT_EQ(fake_.word(4, waveshare_servos_test::kRegGoalSpeed), -700);
}

// ---------------------------------------------------------------------------------------------
// PHASE3 C2 stage A: the 15-byte decoder and the burst walker. Both are free functions over plain
// bytes, so nothing here opens a port, builds a frame or instantiates an SMS_STS -- which is the
// whole reason PHASE3 R17 made the walker public instead of an anonymous-namespace helper.
// ---------------------------------------------------------------------------------------------

TEST(ServoBus, decode_feedback_block_reproduces_the_sms_sts_accessors)
{
  // PHASE3 4.T1 / 2.95 / 2.46. The one vector measured field for field against FeedBack() plus
  // ReadPos/ReadSpeed/ReadLoad/ReadVoltage/ReadTemper/ReadMove/ReadCurrent(-1) themselves
  // (recon/packets.md 5.2): offsets 0-1 position, 2-3 speed, 4-5 load, 6 voltage, 7 temperature,
  // 10 moving, 13-14 current, all little endian. Offsets 8, 9, 11 and 12 are unnamed registers
  // and must reach no field.
  constexpr std::array<uint8_t, 15> kBlock = {
    0xe8, 0x03, 0x2c, 0x81, 0x10, 0x04, 0x7c, 0x21, 0x00, 0x00, 0x01, 0x00, 0x00, 0x20, 0x80};
  const FeedbackBlock block = decode_feedback_block(kBlock.data(), 0);
  EXPECT_TRUE(block.valid);
  EXPECT_EQ(block.status, 0);
  EXPECT_EQ(block.position_ticks, 1000);
  EXPECT_EQ(block.speed_ticks, -300);
  EXPECT_EQ(block.load_raw, -16);
  EXPECT_EQ(block.voltage_raw, 124);
  EXPECT_EQ(block.temperature_raw, 33);
  EXPECT_EQ(block.moving_raw, 1);
  EXPECT_EQ(block.current_counts, -32);

  // The status byte is the caller's -- the decoder is 15 bytes wide and cannot know what frame
  // carried them, which is why it is the burst walker that supplies it (PHASE3 2.36).
  EXPECT_EQ(decode_feedback_block(kBlock.data(), 0x2b).status, 0x2b);
}

TEST(ServoBus, decode_feedback_block_takes_load_from_bit_ten_and_the_rest_from_bit_fifteen)
{
  // PHASE3 4.T2 / 2.98 / R4. The two sign conventions differ twice over: in WHICH bit, and in
  // what survives above it. ReadPos computes -(Pos & ~(1<<15)) on an int holding a 16-bit word
  // (src/SMS_STS.cpp:146-148), so clearing bit 15 clears the only high bit and the result is
  // exact. ReadLoad computes -(Load & ~(1<<10)) on the same kind of int (:188-190), which clears
  // bit 10 and LEAVES BITS 11..15 IN PLACE -- 0xffff becomes 0xfbff = 64511.
  //
  // The 0xffff row is the point of this case. A real servo never sets bits 11..15 (0..1000,
  // include/SMS_STS.h:83), so a decoder that "cleans up" the mask to 0x3ff can only ever be caught
  // here -- and it would publish a different `load` state than Phase 2 did.
  std::array<uint8_t, 15> block = {};
  const std::array<std::pair<std::array<uint8_t, 2>, int>, 8> loads = {{
    {{{0x10, 0x04}}, -16},        // 0x0410, the measured resting value of 4.T1
    {{{0x10, 0x00}}, 16},         // 0x0010, bit 10 clear, so no sign at all
    {{{0xff, 0x03}}, 1023},       // 0x03ff, the largest positive a real servo reports
    {{{0x00, 0x04}}, 0},          // 0x0400, the load half of the `-0` of PHASE3 2.43
    {{{0x01, 0x04}}, -1},         // 0x0401, the smallest negative
    {{{0xff, 0x07}}, -1023},      // 0x07ff, the largest magnitude a real servo reports
    {{{0x00, 0x08}}, 2048},       // 0x0800, bit 11 is NOT a sign bit and must survive
    {{{0xff, 0xff}}, -64511}}};   // 0xffff, NOT -1023
  for (const auto & row : loads) {
    block[4] = row.first[0];
    block[5] = row.first[1];
    EXPECT_EQ(decode_feedback_block(block.data(), 0).load_raw, row.second) <<
      "load word 0x" << std::hex << (row.first[0] | (row.first[1] << 8));
  }

  // and, for contrast, the same bytes in the position field, where bit 15 is the sign and nothing
  // survives above it
  block = {};
  block[0] = 0xff;
  block[1] = 0xff;
  EXPECT_EQ(decode_feedback_block(block.data(), 0).position_ticks, -32767);
}

TEST(ServoBus, decode_feedback_block_signs_position_speed_and_current_on_bit_fifteen)
{
  // PHASE3 2.97. The same six-row table applied to all three bit-15 fields, so a decoder that
  // gets one of them right by copy-paste and another wrong fails here. 0x8000 decodes to -0,
  // i.e. 0: that is what -(v & ~(1<<15)) does with the magnitude bits clear, and PHASE3 2.43
  // says reproduce it rather than "fix" it.
  const std::array<std::pair<uint16_t, int>, 6> rows = {{
    {0x0000, 0}, {0x0001, 1}, {0x7fff, 32767}, {0x8000, 0}, {0x8001, -1}, {0xffff, -32767}}};
  for (const auto & row : rows) {
    std::array<uint8_t, 15> block = {};
    const uint8_t low = static_cast<uint8_t>(row.first & 0xff);
    const uint8_t high = static_cast<uint8_t>(row.first >> 8);
    block[0] = low;  block[1] = high;    // position
    block[2] = low;  block[3] = high;    // speed
    block[13] = low; block[14] = high;   // current
    const FeedbackBlock decoded = decode_feedback_block(block.data(), 0);
    EXPECT_EQ(decoded.position_ticks, row.second) << "word 0x" << std::hex << row.first;
    EXPECT_EQ(decoded.speed_ticks, row.second) << "word 0x" << std::hex << row.first;
    EXPECT_EQ(decoded.current_counts, row.second) << "word 0x" << std::hex << row.first;
  }
}

TEST(ServoBus, decode_feedback_block_reads_the_moving_flag_at_offset_ten)
{
  // PHASE3 2.96 / 2.12. Register 66, the one field no accessor call in the driver exercises
  // today: it is carried so the decoder's equivalence with the whole accessor set is complete,
  // and it is the field an off-by-one in the unnamed registers 64, 65, 67, 68 lands on first.
  std::array<uint8_t, 15> block = {};
  EXPECT_EQ(decode_feedback_block(block.data(), 0).moving_raw, 0);
  block[10] = 1;
  EXPECT_EQ(decode_feedback_block(block.data(), 0).moving_raw, 1);
  // the four unnamed registers either side of it must reach no field at all
  block = {};
  block[8] = 0xff; block[9] = 0xff; block[11] = 0xff; block[12] = 0xff;
  const FeedbackBlock decoded = decode_feedback_block(block.data(), 0);
  EXPECT_EQ(decoded.moving_raw, 0);
  EXPECT_EQ(decoded.current_counts, 0);
  EXPECT_EQ(decoded.temperature_raw, 0);
}

TEST(ServoBus, decode_feedback_block_leaves_voltage_and_temperature_unsigned)
{
  // PHASE3 2.100. ReadVoltage and ReadTemper return a plain byte (src/SMS_STS.cpp:198,213) with
  // no sign handling of any kind, so 0xff is 255 and never -1.
  std::array<uint8_t, 15> block = {};
  block[6] = 0xff;
  block[7] = 0xff;
  const FeedbackBlock decoded = decode_feedback_block(block.data(), 0);
  EXPECT_EQ(decoded.voltage_raw, 255);
  EXPECT_EQ(decoded.temperature_raw, 255);
}

namespace
{

// The 84-byte reply to `syncReadPacketTx({1,2,3,4}, 4, 56, 15)`, captured byte for byte off this
// bench at 1 Mbaud with the four servos at rest (probe/p1_syncread.md Q1, probe/out_q1.txt). Four
// independent 21-byte frames, each `FF FF <id> 0x11 <status> <15 data bytes> <~cks>`.
//
// It is the anchor for every parser case below, for two reasons a synthetic burst cannot match:
// servo 1 repeats its position word `89 06` at block offsets 11-12 (the unnamed registers 67, 68),
// so a decoder that reads current two bytes early returns 1673 instead of 0; and servos 3 and 4
// carry genuine bit-10 loads, 0x0408 and 0x040a, i.e. -8 and -10.
constexpr std::array<uint8_t, 84> kBenchBurst = {
  0xff, 0xff, 0x01, 0x11, 0x00, 0x89, 0x06, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x22, 0x00, 0x00, 0x00,
  0x89, 0x06, 0x00, 0x00, 0x31,
  0xff, 0xff, 0x02, 0x11, 0x00, 0xff, 0x07, 0x00, 0x00, 0x00, 0x00, 0x7b, 0x22, 0x00, 0x00, 0x00,
  0xff, 0x07, 0x00, 0x00, 0x43,
  0xff, 0xff, 0x03, 0x11, 0x00, 0xef, 0x0e, 0x00, 0x00, 0x08, 0x04, 0x7a, 0x21, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x47,
  0xff, 0xff, 0x04, 0x11, 0x00, 0xe9, 0x0e, 0x00, 0x00, 0x0a, 0x04, 0x7b, 0x21, 0x00, 0x00, 0x00,
  0x1b, 0x03, 0x00, 0x00, 0x2b};

constexpr std::size_t kFrameBytes = 21;

// The checksum the servo computed and SCS::Read verifies: the uint8_t sum of id, length, status
// and the 15 data bytes, complemented (src/SCS.cpp:359-367). Written out here rather than reused
// from the wrapper so the parser's arithmetic is checked against a second implementation, which
// is what made probe 1 Q6's "0 bad checksums in 5000 transactions" a result and not a tautology.
uint8_t frame_checksum(const uint8_t * frame)
{
  uint8_t sum = 0;
  for (std::size_t i = 2; i + 1 < kFrameBytes; i++) {
    sum = static_cast<uint8_t>(sum + frame[i]);
  }
  return static_cast<uint8_t>(~sum);
}

// Recompute one frame's trailing checksum in place. PHASE3 4.T11 insists the doctored frames are
// built here and not copied: probe 1 Q4 prints them with the data elided and with frame 1's
// checksum wrong (it prints frame 2's), so a copied literal would fail its own gate.
void repair(std::vector<uint8_t> & burst, std::size_t frame)  // NOLINT(runtime/references)
{
  burst[frame * kFrameBytes + kFrameBytes - 1] = frame_checksum(&burst[frame * kFrameBytes]);
}

// The capture rebuilt with its frames in `frames` order, each still carrying its own id, status
// and checksum. Reordering whole frames is the only way to build a burst that is individually
// well formed and collectively out of request order, which is the case a positional parser
// mis-attributes.
std::vector<uint8_t> burst_of(const std::vector<std::size_t> & frames)
{
  std::vector<uint8_t> burst;
  for (const std::size_t frame : frames) {
    const uint8_t * bytes = kBenchBurst.data() + frame * kFrameBytes;
    burst.insert(burst.end(), bytes, bytes + kFrameBytes);
  }
  return burst;
}

// What frame `frame` of the capture decodes to. Frame k is servo k+1's, so this is also "what
// slot k must hold" for every case parsed against the ids {1, 2, 3, 4}.
FeedbackBlock bench_block(std::size_t frame)
{
  const uint8_t * bytes = kBenchBurst.data() + frame * kFrameBytes;
  return decode_feedback_block(bytes + 5, bytes[4]);
}

}  // namespace

TEST(ServoBus, decode_feedback_block_reads_a_real_bench_frame)
{
  // PHASE3 4.T3 / 2.98a / 2.47. The three frames of the capture that carry something a synthetic
  // block cannot: frame 1's repeated position word in the unnamed registers, and frames 3 and 4's
  // real bit-10 loads at two different magnitudes -- two magnitudes because one alone does not
  // separate a `~(1<<10)` mask from a `0x3ff` one.
  const std::array<std::tuple<std::size_t, int, int, int, int>, 3> frames = {{
    // frame index, position, load, voltage, temperature
    {0, 1673, 0, 124, 34},
    {2, 3823, -8, 122, 33},
    {3, 3817, -10, 123, 33}}};
  for (const auto & row : frames) {
    const std::size_t frame = std::get<0>(row);
    const uint8_t * bytes = kBenchBurst.data() + frame * kFrameBytes;
    // the literal is only trustworthy if it still checksums, so check that before decoding it
    ASSERT_EQ(bytes[kFrameBytes - 1], frame_checksum(bytes)) << "frame " << frame;
    const FeedbackBlock block = decode_feedback_block(bytes + 5, bytes[4]);
    EXPECT_EQ(block.position_ticks, std::get<1>(row)) << "frame " << frame;
    EXPECT_EQ(block.speed_ticks, 0) << "frame " << frame;
    EXPECT_EQ(block.load_raw, std::get<2>(row)) << "frame " << frame;
    EXPECT_EQ(block.voltage_raw, std::get<3>(row)) << "frame " << frame;
    EXPECT_EQ(block.temperature_raw, std::get<4>(row)) << "frame " << frame;
    EXPECT_EQ(block.moving_raw, 0) << "frame " << frame;
    // Frame 1's registers 67-68 hold a second copy of its position. Nothing but a correct current
    // offset makes this zero, which is what this frame is in the suite for.
    EXPECT_EQ(block.current_counts, 0) << "frame " << frame;
  }
}

TEST(ServoBus, min_io_timeout_ms_tracks_the_measured_cost_model)
{
  // PHASE3 4.T9 / R9 / 5.20b. The floor is the measured sync-read cost model
  // t(n) = 0.476 + 0.290n ms (probe 1 Q3, least squares over n = 1..4) scaled by the measured
  // p99/mean tail factor of 1.19 (probe 3 Q1: 2.058/1.734) and rounded up with 1 ms of slack --
  // whole milliseconds because /dev/ttyACM0 is USB CDC and the host polls it once per 1 ms frame,
  // so sub-millisecond tuning is meaningless (probe 3 bonus).
  //
  // These are the model evaluated BY HAND, so a later edit to the formula has to be deliberate.
  EXPECT_EQ(ServoBus::min_io_timeout_ms(1), 2u);
  // 3 ms at four servos measured 0 failures in 2000 transactions (probe 3 Q2), which is what rules
  // out the flat +2 margin an earlier draft proposed: it would have warned about that run.
  EXPECT_EQ(ServoBus::min_io_timeout_ms(4), 3u);
  EXPECT_EQ(ServoBus::min_io_timeout_ms(8), 5u);
  // The claim the new 5 ms default rests on: it stays silent up to nine servos. Nothing else in
  // the suite would catch that going stale.
  EXPECT_EQ(ServoBus::min_io_timeout_ms(9), 5u);
  EXPECT_EQ(ServoBus::min_io_timeout_ms(12), 6u);
  EXPECT_EQ(ServoBus::min_io_timeout_ms(16), 8u);
  EXPECT_EQ(ServoBus::min_io_timeout_ms(30), 12u);

  // Monotonic over the whole chunk range, because the argument is the CHUNK size and a bus that
  // grows must never be told it needs less time (PHASE3 2.81).
  for (std::size_t servos = 2; servos <= ServoBus::sync_read_max_ids; servos++) {
    EXPECT_GE(ServoBus::min_io_timeout_ms(servos), ServoBus::min_io_timeout_ms(servos - 1)) <<
      servos;
  }

  // constexpr, so it can size a buffer or a static_assert and cannot drift with the libm rounding
  // mode the way a ceil() of a double would.
  static_assert(ServoBus::min_io_timeout_ms(4) == 3u, "the floor is not a constant expression");
}

TEST(ServoBus, parse_sync_read_burst_decodes_four_frames_in_request_order)
{
  // PHASE3 4.T10 / 2.32. The real four-servo burst, parsed against the list it was asked for.
  // The four checksums are verified here first, with the same independent helper the doctored
  // frames of 4.T11 use, so a transcription slip in the literal fails in this case rather than
  // somewhere downstream where it would look like a parser bug.
  for (std::size_t frame = 0; frame < 4; frame++) {
    const uint8_t * bytes = kBenchBurst.data() + frame * kFrameBytes;
    ASSERT_EQ(bytes[kFrameBytes - 1], frame_checksum(bytes)) << "frame " << frame;
  }

  const std::vector<uint8_t> ids = {1, 2, 3, 4};
  std::vector<FeedbackBlock> blocks;
  SyncReadStats stats;
  EXPECT_EQ(
    parse_sync_read_burst(kBenchBurst.data(), kBenchBurst.size(), ids, &blocks, &stats), 4u);

  ASSERT_EQ(blocks.size(), 4u);
  const std::array<int, 4> positions = {1673, 2047, 3823, 3817};
  const std::array<int, 4> loads = {0, 0, -8, -10};
  for (std::size_t slot = 0; slot < 4; slot++) {
    EXPECT_TRUE(blocks[slot].valid) << "slot " << slot;
    EXPECT_EQ(blocks[slot].position_ticks, positions[slot]) << "slot " << slot;
    EXPECT_EQ(blocks[slot].load_raw, loads[slot]) << "slot " << slot;
    EXPECT_EQ(blocks[slot].status, 0) << "slot " << slot;
  }
  EXPECT_EQ(stats.bad_frames, 0u);
  EXPECT_EQ(stats.missing_frames, 0u);
}

TEST(ServoBus, parse_sync_read_burst_gives_each_servo_the_status_byte_of_its_own_frame)
{
  // PHASE3 4.T11 / 2.36 / 2.49. Each servo's status is byte 4 of ITS OWN frame, taken from the
  // same 21 bytes the data came from with nothing in between. The vendored syncReadPacketRx
  // instead carries it in the shared SCS::Error, which it leaves untouched for an id that did not
  // answer (src/SCS.cpp:358, measured with a 0xee poison in probe 1 Q4) -- so an absent servo
  // there inherits the last answering one's byte.
  std::vector<uint8_t> burst = burst_of({0, 1, 2, 3});
  const std::array<uint8_t, 4> statuses = {0x11, 0x22, 0x44, 0x88};
  for (std::size_t frame = 0; frame < 4; frame++) {
    burst[frame * kFrameBytes + 4] = statuses[frame];
    repair(burst, frame);
  }
  // The checksums recomputed by hand from the capture, as a check on `repair` itself: probe 1 Q4
  // prints frame 1's as 0x21, which is frame 2's value, so a frame copied from the probe would
  // fail its own gate and take slot 0 with it.
  const std::array<uint8_t, 4> repaired = {0x20, 0x21, 0x03, 0xa3};
  for (std::size_t frame = 0; frame < 4; frame++) {
    EXPECT_EQ(burst[frame * kFrameBytes + kFrameBytes - 1], repaired[frame]) << "frame " << frame;
  }

  const std::vector<uint8_t> ids = {1, 2, 3, 4};
  std::vector<FeedbackBlock> blocks;
  ASSERT_EQ(parse_sync_read_burst(burst.data(), burst.size(), ids, &blocks, nullptr), 4u);
  for (std::size_t slot = 0; slot < 4; slot++) {
    EXPECT_EQ(blocks[slot].status, statuses[slot]) << "slot " << slot;
  }

  // and with servo 2 silent, slot 1 keeps a zero status -- never frame 1's 0x11, and never
  // frame 3's 0x44 either
  burst.erase(
    burst.begin() + static_cast<std::ptrdiff_t>(kFrameBytes),
    burst.begin() + static_cast<std::ptrdiff_t>(2 * kFrameBytes));
  ASSERT_EQ(parse_sync_read_burst(burst.data(), burst.size(), ids, &blocks, nullptr), 3u);
  EXPECT_FALSE(blocks[1].valid);
  EXPECT_EQ(blocks[1].status, 0);
  EXPECT_EQ(blocks[0].status, 0x11);
  EXPECT_EQ(blocks[2].status, 0x44);
}

TEST(ServoBus, parse_sync_read_burst_skips_a_missing_id_and_decodes_the_rest)
{
  // PHASE3 4.T12 / 2.34. Servo 2 says nothing, so its 21 bytes are simply absent from the wire.
  // Id 3 is AHEAD of the cursor in the request list, so slot 1 is abandoned and slot 2 takes the
  // frame. The bench's own evidence for going on rather than giving up is that a burst with an
  // absent id still decodes the rest perfectly, 1200/1200 in every case probe 1 Q5 tried.
  const std::vector<uint8_t> burst = burst_of({0, 2, 3});
  const std::vector<uint8_t> ids = {1, 2, 3, 4};
  std::vector<FeedbackBlock> blocks;
  SyncReadStats stats;
  EXPECT_EQ(parse_sync_read_burst(burst.data(), burst.size(), ids, &blocks, &stats), 3u);

  ASSERT_EQ(blocks.size(), 4u);
  EXPECT_TRUE(blocks[0].valid);
  EXPECT_EQ(blocks[0].position_ticks, 1673);
  // value-initialised, NOT servo 3's data: that is the whole difference from a positional parser
  EXPECT_FALSE(blocks[1].valid);
  EXPECT_EQ(blocks[1].position_ticks, 0);
  EXPECT_EQ(blocks[1].load_raw, 0);
  EXPECT_EQ(blocks[1].voltage_raw, 0);
  EXPECT_TRUE(blocks[2].valid);
  EXPECT_EQ(blocks[2].position_ticks, 3823);
  EXPECT_EQ(blocks[2].load_raw, -8);
  EXPECT_TRUE(blocks[3].valid);
  EXPECT_EQ(blocks[3].position_ticks, 3817);
  EXPECT_EQ(blocks[3].load_raw, -10);
  // one requested id with no frame in the burst, and no frame was rejected by the gate
  EXPECT_EQ(stats.missing_frames, 1u);
  EXPECT_EQ(stats.bad_frames, 0u);
}

TEST(ServoBus, parse_sync_read_burst_refuses_a_frame_that_is_not_the_next_expected_id)
{
  // PHASE3 4.T13. The same four well-formed frames, in reverse order. HOW MANY decode is an
  // implementation detail of the cursor and is deliberately not asserted; what is asserted is
  // that no servo's data ever lands in another servo's slot. A naive positional parser passes
  // 4.T10 and fails exactly here, which is why this case exists.
  const std::vector<uint8_t> burst = burst_of({3, 2, 1, 0});
  const std::vector<uint8_t> ids = {1, 2, 3, 4};
  std::vector<FeedbackBlock> blocks;
  parse_sync_read_burst(burst.data(), burst.size(), ids, &blocks, nullptr);

  ASSERT_EQ(blocks.size(), 4u);
  for (std::size_t slot = 0; slot < 4; slot++) {
    if (!blocks[slot].valid) {
      continue;
    }
    const FeedbackBlock expected = bench_block(slot);
    EXPECT_EQ(blocks[slot].position_ticks, expected.position_ticks) << "slot " << slot;
    EXPECT_EQ(blocks[slot].load_raw, expected.load_raw) << "slot " << slot;
    EXPECT_EQ(blocks[slot].voltage_raw, expected.voltage_raw) << "slot " << slot;
    EXPECT_EQ(blocks[slot].temperature_raw, expected.temperature_raw) << "slot " << slot;
  }
}

TEST(ServoBus, parse_sync_read_burst_never_refills_a_slot_the_cursor_has_passed)
{
  // PHASE3 2.34, the branch 4.T13 cannot reach. 4.T13's reversed burst fills slot 3 on its first
  // frame and the walk then ends, so a walker that searched the WHOLE id list from 0 every time
  // would pass it unchanged; this is the case that separates the two. The burst carries a second,
  // stale frame for id 1 -- well formed, correct checksum, but behind the cursor -- between the
  // frames of ids 2 and 3, which is what probe 3 Q6 measured coming off the wire: 71 frames whose
  // id did not belong in the slot, in 3000 reads at a 1 ms timeout.
  //
  // A global id search rewinds the cursor and overwrites slot 0 with the stale sample; the
  // forward-only search refuses the frame outright. The stale copy is doctored to position 0 so
  // the difference is a value, not just a count.
  std::vector<uint8_t> burst = burst_of({0, 1, 0, 2});
  burst[2 * kFrameBytes + 5] = 0x00;
  burst[2 * kFrameBytes + 6] = 0x00;
  repair(burst, 2);
  ASSERT_EQ(burst[3 * kFrameBytes - 1], frame_checksum(&burst[2 * kFrameBytes]));

  const std::vector<uint8_t> ids = {1, 2, 3, 4};
  std::vector<FeedbackBlock> blocks;
  SyncReadStats stats;
  EXPECT_EQ(parse_sync_read_burst(burst.data(), burst.size(), ids, &blocks, &stats), 3u);

  ASSERT_EQ(blocks.size(), 4u);
  // slot 0 still holds the FIRST frame's sample, never the stale duplicate's zero
  EXPECT_TRUE(blocks[0].valid);
  EXPECT_EQ(blocks[0].position_ticks, 1673);
  EXPECT_TRUE(blocks[1].valid);
  EXPECT_EQ(blocks[1].position_ticks, 2047);
  // the cursor did not rewind, so id 3's frame still lands in slot 2 and id 4 stays silent
  EXPECT_TRUE(blocks[2].valid);
  EXPECT_EQ(blocks[2].position_ticks, 3823);
  EXPECT_FALSE(blocks[3].valid);
  EXPECT_EQ(stats.bad_frames, 1u);
  EXPECT_EQ(stats.missing_frames, 1u);
}

TEST(ServoBus, parse_sync_read_burst_resyncs_one_byte_at_a_time_after_a_rejected_frame)
{
  // PHASE3 2.33: a rejection advances by ONE byte, never by a whole frame. The difference only
  // shows on a buffer whose real frames are not on 21-byte boundaries, which is precisely the
  // buffer the rule exists for -- three stray bytes ahead of the capture make a well-formed
  // header for a foreign id (0x63 is on no one's request list), and the real burst then starts at
  // offset 3. Skipping 21 bytes past that rejection lands mid-frame and loses id 1 entirely.
  std::vector<uint8_t> burst = {0xff, 0xff, 0x63};
  burst.insert(burst.end(), kBenchBurst.begin(), kBenchBurst.end());

  const std::vector<uint8_t> ids = {1, 2, 3, 4};
  std::vector<FeedbackBlock> blocks;
  SyncReadStats stats;
  EXPECT_EQ(parse_sync_read_burst(burst.data(), burst.size(), ids, &blocks, &stats), 4u);

  ASSERT_EQ(blocks.size(), 4u);
  EXPECT_TRUE(blocks[0].valid);
  EXPECT_EQ(blocks[0].position_ticks, 1673);
  EXPECT_TRUE(blocks[3].valid);
  EXPECT_EQ(blocks[3].position_ticks, 3817);
  // the foreign header is one rejected frame and no requested id went unanswered
  EXPECT_EQ(stats.bad_frames, 1u);
  EXPECT_EQ(stats.missing_frames, 0u);
}

TEST(ServoBus, parse_sync_read_burst_adds_to_the_counters_rather_than_assigning_them)
{
  // PHASE3 2.35 / 2.6: an id list longer than sync_read_max_ids is several chunks and several
  // walks, and the counters are session totals. Assigning instead of adding would make every
  // chunked read report only its last chunk's failures, and the README's per-million figure and
  // the bench's H11.fail_rate gate are both built on these two numbers.
  std::vector<uint8_t> burst = burst_of({0, 1, 2, 3});
  burst[kFrameBytes + 5 + 7] = 0x23;    // servo 2's temperature, checksum left stale
  const std::vector<uint8_t> ids = {1, 2, 3, 4};
  std::vector<FeedbackBlock> blocks;

  SyncReadStats stats;
  parse_sync_read_burst(burst.data(), burst.size(), ids, &blocks, &stats);
  parse_sync_read_burst(burst.data(), burst.size(), ids, &blocks, &stats);
  EXPECT_EQ(stats.bad_frames, 2u);
  EXPECT_EQ(stats.missing_frames, 2u);
  // and the three counters the transaction layer owns are none of the walker's business
  EXPECT_EQ(stats.transactions, 0u);
  EXPECT_EQ(stats.short_bursts, 0u);
  EXPECT_EQ(stats.drains, 0u);
}

TEST(ServoBus, parse_sync_read_burst_rejects_only_the_frame_that_failed_the_gate)
{
  // PHASE3 4.T14 as rewritten by R3. A rejected frame does NOT stop the parse: each 21-byte frame
  // is independently verifiable -- header, id in its slot, length byte and its own checksum -- so
  // one bad frame says nothing about the next, and stopping would turn one corrupted reply into
  // three lost joints.
  //
  // Servo 2's temperature byte is flipped and its checksum left alone, so the frame fails gate
  // step 4 and nothing else. The precondition R3 states holds here: no `FF FF` pair appears
  // anywhere inside frame 2's body for the one-byte resync to latch on to, so the walk runs on to
  // frame 3's real header.
  std::vector<uint8_t> burst = burst_of({0, 1, 2, 3});
  const std::size_t temperature = kFrameBytes + 5 + 7;
  ASSERT_EQ(burst[temperature], 0x22);
  burst[temperature] = 0x23;
  ASSERT_NE(burst[2 * kFrameBytes - 1], frame_checksum(&burst[kFrameBytes]));

  const std::vector<uint8_t> ids = {1, 2, 3, 4};
  std::vector<FeedbackBlock> blocks;
  SyncReadStats stats;
  EXPECT_EQ(parse_sync_read_burst(burst.data(), burst.size(), ids, &blocks, &stats), 3u);

  ASSERT_EQ(blocks.size(), 4u);
  EXPECT_TRUE(blocks[0].valid);
  EXPECT_EQ(blocks[0].position_ticks, 1673);
  EXPECT_FALSE(blocks[1].valid);
  EXPECT_TRUE(blocks[2].valid);
  EXPECT_EQ(blocks[2].position_ticks, 3823);
  EXPECT_TRUE(blocks[3].valid);
  EXPECT_EQ(blocks[3].position_ticks, 3817);
  EXPECT_EQ(stats.bad_frames, 1u);
  EXPECT_EQ(stats.missing_frames, 1u);
}

TEST(ServoBus, parse_sync_read_burst_never_reads_past_the_buffer)
{
  // PHASE3 4.T15 / 2.32. A 21-byte buffer of all 0x01 against the id 1 is the exact shape that
  // makes the vendored syncReadPacketRx read 18 bytes past the end: its resync loop runs to the
  // end without finding `FF FF`, the last byte happens to equal the requested id, and
  // src/SCS.cpp:355,358,361,365 then read 1 + 1 + 15 + 1 bytes beyond syncReadRxBuffLen
  // (ASAN-confirmed, recon/packets.md 3.4). The `pos + 21 <= len` guard, evaluated before any
  // byte of a frame is touched, is the whole fix.
  const std::vector<uint8_t> ids = {1};
  std::vector<FeedbackBlock> blocks;
  const std::vector<uint8_t> all_ones(ServoBus::sync_read_frame_bytes, 0x01);
  EXPECT_EQ(
    parse_sync_read_burst(all_ones.data(), all_ones.size(), ids, &blocks, nullptr), 0u);
  ASSERT_EQ(blocks.size(), 1u);
  EXPECT_FALSE(blocks[0].valid);

  // a buffer with nothing in it at all, one byte, and a header with no frame behind it
  const std::vector<uint8_t> header = {0xff, 0xff};
  EXPECT_EQ(parse_sync_read_burst(all_ones.data(), 0, ids, &blocks, nullptr), 0u);
  EXPECT_EQ(parse_sync_read_burst(all_ones.data(), 1, ids, &blocks, nullptr), 0u);
  EXPECT_EQ(parse_sync_read_burst(header.data(), header.size(), ids, &blocks, nullptr), 0u);

  // and a frame that is well formed in every way except its length byte, which claims 0x12 where
  // the 15-byte block makes it 0x11. SCS::Read never checks that byte at all (src/SCS.cpp:190-199,
  // recon/packets.md landmine 12); the wrapper does, because it costs one comparison.
  std::vector<uint8_t> wrong_length(kBenchBurst.begin(), kBenchBurst.begin() + kFrameBytes);
  wrong_length[3] = 0x12;
  repair(wrong_length, 0);
  EXPECT_EQ(
    parse_sync_read_burst(wrong_length.data(), wrong_length.size(), ids, &blocks, nullptr), 0u);
  EXPECT_FALSE(blocks[0].valid);

  // `length` is what readSCS returned, NOT the buffer's size, and that is the shape the shipped
  // transaction layer calls with: the 648-byte rx_buf_ is reused every cycle (PHASE3 2.13-2.14),
  // so every byte past `length` is still inside the allocation and holds last cycle's burst. No
  // sanitizer can see a walker that trusts the allocation instead of the count -- only the return
  // value can. A reply one byte short of its last frame must leave that slot empty rather than
  // publish the previous cycle's frame as this cycle's sample, which is the contamination R8
  // banned io_timeout_ms = 1 for (probe 3 Q4/Q6).
  std::vector<uint8_t> rx(ServoBus::sync_read_buffer_bytes, 0);
  std::copy(kBenchBurst.begin(), kBenchBurst.end(), rx.begin());
  const std::vector<uint8_t> four = {1, 2, 3, 4};
  EXPECT_EQ(
    parse_sync_read_burst(rx.data(), kBenchBurst.size() - 1, four, &blocks, nullptr), 3u);
  ASSERT_EQ(blocks.size(), 4u);
  EXPECT_TRUE(blocks[2].valid);
  EXPECT_FALSE(blocks[3].valid);
}

// ---------------------------------------------------------------------------------------------
// PHASE3 C3 stage C: the fake bus on trial before anything depends on it (4.T24-4.T26).
//
// These drive a plain ServoBus straight at the vendored syncReadPacketTx, so what they pin is the
// harness's reply burst and none of the wrapper's own transaction layer. The suite name is
// FakeBusSyncRead and deliberately not ServoBusSyncRead: stage D declares a fixture class of that
// name and GoogleTest refuses to run a suite whose cases do not all share one fixture
// (PHASE3 4 section 3.3).
// ---------------------------------------------------------------------------------------------

namespace
{

// The four servos every sync-read case below starts from. Every field differs per servo, so a
// block attributed to the wrong slot is visible rather than plausible, and no two bytes of any
// reply are `ff ff`: a false header inside a payload would let the walker resync onto it after a
// rejected frame and claim a second bad frame, which is what PHASE3 2.107's `bad_frames == 1`
// would then be measuring instead of the gate.
void seed_four_servos(waveshare_servos_test::FakeBus * fake)
{
  for (uint8_t id = 1; id <= 4; id++) {
    fake->add_servo(id);
    fake->set_feedback(
      id, 1000 * id, 10 * id, -id, static_cast<uint8_t>(120 + id), static_cast<uint8_t>(40 + id),
      static_cast<uint8_t>(id % 2), 100 * id);
  }
}

// One raw sync read through the vendored request builder, with the receive buffer supplied by the
// caller. Stage C owns that buffer itself rather than leaning on the one PHASE3 2.13 gives the
// constructor, so these cases stay a statement about the harness alone.
int raw_sync_read(ServoBus * bus, std::vector<uint8_t> * rx, const std::vector<uint8_t> & ids)
{
  rx->assign(ids.size() * kFrameBytes + ServoBus::sync_read_slack_bytes, 0);
  bus->syncReadRxBuff = rx->data();
  bus->syncReadRxBuffMax = static_cast<uint16_t>(ids.size() * kFrameBytes);
  std::vector<uint8_t> mutable_ids = ids;   // syncReadPacketTx takes u8 ID[], not a const pointer
  return bus->syncReadPacketTx(
    mutable_ids.data(), static_cast<uint8_t>(ids.size()), ServoBus::feedback_first_register,
    static_cast<uint8_t>(ServoBus::feedback_block_bytes));
}

}  // namespace

TEST(FakeBusSyncRead, the_fake_bus_is_silent_for_an_absent_id_and_answers_for_the_rest)
{
  // PHASE3 4.T25. An absent servo contributes no bytes at all -- not a short frame, not an error
  // frame -- so the burst is three frames and the caller still waits the whole io_timeout_ms for
  // the 84 bytes it asked for. That is the shape probe 1 Q5 measured on the bench and the reason
  // a missing servo costs one timeout and not four (PHASE3 2.106).
  waveshare_servos_test::FakeBus fake;
  seed_four_servos(&fake);
  fake.set_absent(2, true);
  ServoBus bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));
  std::vector<uint8_t> rx;

  EXPECT_EQ(raw_sync_read(&bus, &rx, {1, 2, 3, 4}), 3 * static_cast<int>(kFrameBytes));

  // The three present servos' frames, intact and in request order.
  const std::array<uint8_t, 3> answered = {1, 3, 4};
  for (std::size_t frame = 0; frame < answered.size(); frame++) {
    const uint8_t * bytes = rx.data() + frame * kFrameBytes;
    EXPECT_EQ(bytes[0], 0xff) << "frame " << frame;
    EXPECT_EQ(bytes[1], 0xff) << "frame " << frame;
    EXPECT_EQ(bytes[2], answered[frame]) << "frame " << frame;
    EXPECT_EQ(bytes[3], ServoBus::feedback_block_bytes + 2) << "frame " << frame;
    EXPECT_EQ(bytes[kFrameBytes - 1], frame_checksum(bytes)) << "frame " << frame;
    EXPECT_EQ(
      decode_feedback_block(bytes + 5, bytes[4]).position_ticks, 1000 * answered[frame]) <<
      "frame " << frame;
  }

  // The absent servo answered nothing, but it WAS named in the request. The two counters have to
  // be separable: sync_reads is bumped after the absent check, so it cannot tell "not asked" from
  // "asked and silent", and the driver case that proves an absent servo is never put in the id
  // list at all (PHASE3 3.38, F14) would then assert nothing (3 section H amendment).
  EXPECT_EQ(fake.snapshot(2).sync_reads, 0);
  EXPECT_EQ(fake.snapshot(2).sync_read_named, 1);
  EXPECT_EQ(fake.snapshot(1).sync_read_named, 1);
  bus.close();
}

TEST(FakeBusSyncRead, the_fake_bus_answers_a_sync_read_with_one_frame_per_listed_servo)
{
  // PHASE3 4.T24 / F4. The request is the vendored builder's -- FF FF FE (IDN+4) 82 38 0F <ids>
  // ~cks -- and what this pins is that the harness parses it and answers it the way the hardware
  // does: one ordinary status frame per listed servo, back to back, in ID-LIST ORDER.
  waveshare_servos_test::FakeBus fake;
  seed_four_servos(&fake);
  ServoBus bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));
  std::vector<uint8_t> rx;

  EXPECT_EQ(raw_sync_read(&bus, &rx, {1, 2, 3, 4}), 4 * static_cast<int>(kFrameBytes));

  for (std::size_t frame = 0; frame < 4; frame++) {
    const uint8_t * bytes = rx.data() + frame * kFrameBytes;
    EXPECT_EQ(bytes[0], 0xff) << "frame " << frame;
    EXPECT_EQ(bytes[1], 0xff) << "frame " << frame;
    EXPECT_EQ(bytes[2], frame + 1) << "frame " << frame;
    EXPECT_EQ(bytes[3], 0x11) << "frame " << frame;
    // Recomputed here rather than taken from the harness, so a clean checksum is a statement
    // about the bytes and not a restatement of the code that wrote them.
    EXPECT_EQ(bytes[kFrameBytes - 1], frame_checksum(bytes)) << "frame " << frame;
  }

  // Every servo saw the request, and it asked for the feedback block: registers 56..70.
  for (uint8_t id = 1; id <= 4; id++) {
    const waveshare_servos_test::FakeServo servo = fake.snapshot(id);
    EXPECT_EQ(servo.sync_reads, 1) << "id " << static_cast<int>(id);
    // A sync read counts as a feedback read, deliberately (PHASE3 4.H11).
    EXPECT_EQ(servo.feedback_reads, 1) << "id " << static_cast<int>(id);
    ASSERT_FALSE(servo.sync_read_requests.empty()) << "id " << static_cast<int>(id);
    EXPECT_EQ(servo.sync_read_requests.back().first, 56) << "id " << static_cast<int>(id);
    EXPECT_EQ(servo.sync_read_requests.back().second, 15) << "id " << static_cast<int>(id);
  }
  EXPECT_EQ(fake.sync_read_requests(), 1u);
  EXPECT_EQ(fake.last_sync_read_ids(), (std::vector<uint8_t>{1, 2, 3, 4}));
  bus.close();
}

TEST(FakeBusSyncRead, the_fake_bus_can_delay_truncate_and_mis_sum_a_burst)
{
  // PHASE3 4.T26 / 4.H6-4.H8. The three failure knobs the wrapper's error path is tested with, on
  // trial themselves. Each phase clears the previous one, so the burst is only ever wrong in the
  // one way the phase is about.
  waveshare_servos_test::FakeBus fake;
  seed_four_servos(&fake);
  ServoBus bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));
  std::vector<uint8_t> rx;
  const int whole = 4 * static_cast<int>(kFrameBytes);

  // Phase 1: the delay, compared only against an undelayed read of the same burst. Never an
  // absolute bound -- the house rule at a_silent_bus_costs_one_io_timeout.
  const auto prompt_started = std::chrono::steady_clock::now();
  ASSERT_EQ(raw_sync_read(&bus, &rx, {1, 2, 3, 4}), whole);
  const auto prompt = std::chrono::steady_clock::now() - prompt_started;
  fake.set_sync_read_delay_ms(3);
  const auto delayed_started = std::chrono::steady_clock::now();
  EXPECT_EQ(raw_sync_read(&bus, &rx, {1, 2, 3, 4}), whole) << "the delay must not lose bytes";
  const auto delayed = std::chrono::steady_clock::now() - delayed_started;
  EXPECT_GT(delayed, prompt) << "a delayed burst must take measurably longer than a prompt one";
  fake.set_sync_read_delay_ms(0);

  // Phase 2: the tail cut off mid frame. 10 bytes is half of the last frame, which is the shape
  // an over-reading parser decodes from short data (PHASE3 4.T33).
  fake.set_sync_read_truncate_bytes(10);
  EXPECT_EQ(raw_sync_read(&bus, &rx, {1, 2, 3, 4}), whole - 10);
  fake.set_sync_read_truncate_bytes(0);

  // Phase 3: one frame well formed and mis-summed. The burst keeps its length; only servo 3's
  // checksum stops verifying, and the other three still do.
  fake.set_sync_read_bad_checksum(3);
  EXPECT_EQ(raw_sync_read(&bus, &rx, {1, 2, 3, 4}), whole);
  for (std::size_t frame = 0; frame < 4; frame++) {
    const uint8_t * bytes = rx.data() + frame * kFrameBytes;
    ASSERT_EQ(bytes[2], frame + 1) << "frame " << frame;
    if (frame == 2) {
      EXPECT_NE(bytes[kFrameBytes - 1], frame_checksum(bytes)) << "servo 3's frame must mis-sum";
    } else {
      EXPECT_EQ(bytes[kFrameBytes - 1], frame_checksum(bytes)) << "frame " << frame;
    }
  }
  fake.set_sync_read_bad_checksum(0);
  bus.close();
}

// ---------------------------------------------------------------------------------------------
// PHASE3 C3 stage D: the sync-read transaction through ServoBus (4.T27-4.T36, 2.99, 2.101-2.116).
// ---------------------------------------------------------------------------------------------

namespace
{

// What seed_four_servos() put in servo `id`'s registers, written out from the seed rather than
// read back through the bus: a decode that only agrees with itself proves nothing.
FeedbackBlock seeded_block(uint8_t id, uint8_t status = 0)
{
  FeedbackBlock block;
  block.valid = true;
  block.status = status;
  block.position_ticks = 1000 * id;
  block.speed_ticks = 10 * id;
  block.load_raw = -static_cast<int>(id);
  block.voltage_raw = 120 + id;
  block.temperature_raw = 40 + id;
  block.moving_raw = id % 2;
  block.current_counts = 100 * id;
  return block;
}

// All nine fields, so a slot that carries the right position and somebody else's load still
// fails. `where` names the slot, because every one of these runs inside a loop.
void expect_block_eq(const FeedbackBlock & got, const FeedbackBlock & want, const std::string & w)
{
  EXPECT_EQ(got.valid, want.valid) << w;
  EXPECT_EQ(got.status, want.status) << w;
  EXPECT_EQ(got.position_ticks, want.position_ticks) << w;
  EXPECT_EQ(got.speed_ticks, want.speed_ticks) << w;
  EXPECT_EQ(got.load_raw, want.load_raw) << w;
  EXPECT_EQ(got.voltage_raw, want.voltage_raw) << w;
  EXPECT_EQ(got.temperature_raw, want.temperature_raw) << w;
  EXPECT_EQ(got.moving_raw, want.moving_raw) << w;
  EXPECT_EQ(got.current_counts, want.current_counts) << w;
}

// A ServoBus wired to the full fake bus of test/fake_servo_bus.hpp rather than to this file's
// cut-down FakeResponder: the sync-read cases need several servos, per-servo counters and the
// failure knobs of PHASE3 4.H4-4.H8, none of which the local responder has.
class ServoBusSyncRead : public ::testing::Test
{
protected:
  void SetUp() override
  {
    seed_four_servos(&fake_);
    // 20 ms is generous on purpose: an absent or silent servo then costs one known unit, and
    // nothing in this stage times anything except the two cases that are about timing.
    ASSERT_TRUE(static_cast<bool>(bus_.open(fake_.port(), kBaudrate, kIoTimeoutMs)));
  }

  void TearDown() override
  {
    bus_.close();
    // The reply-corruption knobs corrupt the REPLY, and the fake checksum-verifies only what it
    // RECEIVES (fake_servo_bus.hpp:337-344), so neither counter may move (PHASE3 2.94).
    EXPECT_EQ(fake_.bad_checksums(), 0u) << "the wire itself misbehaved";
    EXPECT_EQ(fake_.quiet_timeouts(), 0u) << "wait_quiet gave up";
  }

  waveshare_servos_test::FakeBus fake_;
  ServoBus bus_;                        // declared after fake_, so it is destroyed first
  std::vector<FeedbackBlock> blocks_;
};

}  // namespace

TEST_F(ServoBusSyncRead, sync_read_feedback_fills_one_block_per_id_in_request_order)
{
  // PHASE3 4.T27 / 2.101. One INST_SYNC_READ carries all four feedback blocks, and every slot is
  // its own servo's: `blocks[k] is ids[k]'s reply` is the whole contract the driver indexes by.
  const std::vector<uint8_t> ids = {1, 2, 3, 4};
  EXPECT_EQ(bus_.sync_read_feedback(ids, blocks_), 4u);
  ASSERT_EQ(blocks_.size(), 4u);
  for (std::size_t slot = 0; slot < 4; slot++) {
    expect_block_eq(
      blocks_[slot], seeded_block(ids[slot]), "slot " + std::to_string(slot));
  }
  // One sync read each and NOT one addressed INST_READ each: `reads` is what would grow if the
  // wrapper had quietly fallen back to four FeedBack() round trips.
  for (uint8_t id = 1; id <= 4; id++) {
    const waveshare_servos_test::FakeServo servo = fake_.snapshot(id);
    EXPECT_EQ(servo.sync_reads, 1) << "id " << static_cast<int>(id);
    EXPECT_EQ(servo.reads, 0) << "id " << static_cast<int>(id);
  }
  EXPECT_EQ(bus_.sync_read_stats().transactions, 1u);
}

TEST_F(ServoBusSyncRead, sync_read_feedback_writes_every_slot_even_when_nobody_answers)
{
  // PHASE3 4.T28. Every slot is written on every call, so a failed transaction can never publish
  // the previous cycle's sample as this cycle's. The first call fills all four on purpose: a
  // wrapper that only writes the slots that decoded passes this case without it.
  const std::vector<uint8_t> ids = {1, 2, 3, 4};
  ASSERT_EQ(bus_.sync_read_feedback(ids, blocks_), 4u);
  fake_.set_sync_read_supported(false);

  EXPECT_EQ(bus_.sync_read_feedback(ids, blocks_), 0u);
  ASSERT_EQ(blocks_.size(), 4u);
  for (std::size_t slot = 0; slot < 4; slot++) {
    expect_block_eq(blocks_[slot], FeedbackBlock{}, "slot " + std::to_string(slot));
  }
  // The firmware parsed 0x82 and answered nothing, so the request still reached every servo.
  for (uint8_t id = 1; id <= 4; id++) {
    EXPECT_EQ(fake_.snapshot(id).sync_reads, 2) << "id " << static_cast<int>(id);
  }
}

TEST_F(ServoBusSyncRead, sync_read_feedback_agrees_field_for_field_with_read_feedback_one)
{
  // PHASE3 4.T29 / F9. The two transports are proved equal once, here at the seam, instead of
  // forever: read_feedback_one issues Read(id, 56, ., 15), which is byte for byte the transaction
  // FeedBack(id) makes (src/SMS_STS.cpp:123), and both paths run the SAME decoder. The fake is a
  // register file with no ADC, so equality is exact -- the bench twin of this check has to allow
  // +/-1 LSB on voltage and temperature, because two reads of the same servo by the same path
  // differ that often (probe 1 Q2).
  const std::vector<uint8_t> ids = {1, 2, 3, 4};
  ASSERT_EQ(bus_.sync_read_feedback(ids, blocks_), 4u);
  for (std::size_t slot = 0; slot < 4; slot++) {
    FeedbackBlock one;
    ASSERT_TRUE(bus_.read_feedback_one(ids[slot], one)) << "id " << static_cast<int>(ids[slot]);
    expect_block_eq(one, blocks_[slot], "id " + std::to_string(ids[slot]));
  }
}

TEST_F(ServoBusSyncRead, sync_read_feedback_matches_feedback_and_the_accessors_for_the_same_servo)
{
  // PHASE3 2.104 / F10. The other half of the equality: against the vendored FeedBack() plus the
  // seven ReadX(-1) accessors, which is what Phase 2 published and what may not change. Nothing
  // in the package calls an accessor after this chunk (R15), so this case is the record of what
  // they used to answer.
  const std::vector<uint8_t> ids = {1, 2, 3, 4};
  ASSERT_EQ(bus_.sync_read_feedback(ids, blocks_), 4u);
  for (std::size_t slot = 0; slot < 4; slot++) {
    const int id = ids[slot];
    ASSERT_NE(bus_.FeedBack(id), -1) << "id " << id;
    EXPECT_EQ(blocks_[slot].position_ticks, bus_.ReadPos(-1)) << "id " << id;
    EXPECT_EQ(blocks_[slot].speed_ticks, bus_.ReadSpeed(-1)) << "id " << id;
    EXPECT_EQ(blocks_[slot].load_raw, bus_.ReadLoad(-1)) << "id " << id;
    EXPECT_EQ(blocks_[slot].voltage_raw, bus_.ReadVoltage(-1)) << "id " << id;
    EXPECT_EQ(blocks_[slot].temperature_raw, bus_.ReadTemper(-1)) << "id " << id;
    EXPECT_EQ(blocks_[slot].moving_raw, bus_.ReadMove(-1)) << "id " << id;
    EXPECT_EQ(blocks_[slot].current_counts, bus_.ReadCurrent(-1)) << "id " << id;
  }
}

TEST_F(ServoBusSyncRead, the_receive_buffer_is_sized_to_the_id_list_and_owned_by_the_wrapper)
{
  // PHASE3 4.T30 / 2.15-2.17 / 2.102. syncReadRxBuffMax is the number of bytes readSCS waits for
  // before it returns early (src/SCS.cpp:318, src/SCSerial.cpp:165-169), so it must be EXACTLY
  // what this chunk's repliers owe. Oversized burns a whole io_timeout_ms on every healthy cycle
  // -- probe 1 Q5 measured +18.6 ms at a 20 ms timeout, a 12.6x blow-up -- and undersized
  // truncates the tail.
  const uint8_t * const buffer = bus_.syncReadRxBuff;
  ASSERT_NE(buffer, nullptr) << "the constructor owns the buffer (PHASE3 2.13)";
  const std::vector<uint8_t> all = {1, 2, 3, 4};
  for (std::size_t n = 1; n <= all.size(); n++) {
    const std::vector<uint8_t> ids(all.begin(), all.begin() + static_cast<std::ptrdiff_t>(n));
    ASSERT_EQ(bus_.sync_read_feedback(ids, blocks_), n) << "n = " << n;
    EXPECT_EQ(bus_.syncReadRxBuffMax, n * ServoBus::sync_read_frame_bytes) << "n = " << n;
    EXPECT_EQ(bus_.syncReadRxBuff, buffer) << "n = " << n;
  }
  // The slack past the last frame is sized in, and it is not decoration: it neutralises the
  // src/SCS.cpp:355 over-read for anything that ever points the vendored decoder at this buffer.
  static_assert(
    ServoBus::sync_read_buffer_bytes >=
    ServoBus::sync_read_max_ids * ServoBus::sync_read_frame_bytes +
    ServoBus::sync_read_slack_bytes,
    "the receive buffer no longer carries the nLen + 3 bytes of slack");

  // No allocation after the first call, on the two proxies a test can actually observe: rx_buf_
  // and tx_ids_ are private, so what is checked is the caller's vector and the public pointer
  // that IS rx_buf_.data() (PHASE3 2.17).
  const std::size_t capacity = blocks_.capacity();
  for (int cycle = 0; cycle < 100; cycle++) {
    ASSERT_EQ(bus_.sync_read_feedback(all, blocks_), 4u) << "cycle " << cycle;
  }
  EXPECT_EQ(blocks_.capacity(), capacity);
  EXPECT_EQ(bus_.syncReadRxBuff, buffer);
}

TEST_F(ServoBusSyncRead, sync_read_feedback_never_replaces_the_receive_buffer)
{
  // PHASE3 4.T31 / 2.103 / 2.13. syncReadBegin() allocates with `new u8[]` and syncReadEnd() frees
  // with a scalar `delete` (src/SCS.cpp:325,331) -- mismatched, UB, a leak if begin is called
  // twice, and a double free if it were ever handed the vector's storage. Both stay uncalled
  // forever, and the pointer the constructor installed is the proof.
  const uint8_t * const from_the_constructor = bus_.syncReadRxBuff;
  ASSERT_NE(from_the_constructor, nullptr);
  const std::vector<uint8_t> ids = {1, 2, 3, 4};
  for (int cycle = 0; cycle < 10; cycle++) {
    ASSERT_EQ(bus_.sync_read_feedback(ids, blocks_), 4u) << "cycle " << cycle;
    ASSERT_EQ(bus_.syncReadRxBuff, from_the_constructor) << "cycle " << cycle;
  }
  // close() leaves the buffer alone too: it belongs to the object, not to the session, and
  // clearing it would only open a window where the pointer is stale (PHASE3 2.18).
  bus_.close();
  EXPECT_EQ(bus_.syncReadRxBuff, from_the_constructor);
}

TEST_F(ServoBusSyncRead, an_absent_servo_fails_alone_and_the_others_still_decode)
{
  // PHASE3 4.T32 / R3. Nothing on the bus at id 2: no frame at all, so the burst is 63 of the 84
  // bytes asked for. The other three frames are perfect -- 1200/1200 real decodes in every
  // absent-id case on the bench (probe 1 Q5) -- and exactly one slot is lost, not four.
  fake_.set_absent(2, true);
  const std::vector<uint8_t> ids = {1, 2, 3, 4};

  EXPECT_EQ(bus_.sync_read_feedback(ids, blocks_), 3u);
  ASSERT_EQ(blocks_.size(), 4u);
  expect_block_eq(blocks_[0], seeded_block(1), "slot 0");
  EXPECT_FALSE(blocks_[1].valid);
  expect_block_eq(blocks_[2], seeded_block(3), "slot 2");
  expect_block_eq(blocks_[3], seeded_block(4), "slot 3");
  const SyncReadStats stats = bus_.sync_read_stats();
  EXPECT_EQ(stats.missing_frames, 1u) << "one slot lost, not four";
  EXPECT_EQ(stats.bad_frames, 0u) << "a frame that never arrived is not a bad frame";
  EXPECT_EQ(stats.short_bursts, 1u);
}

TEST_F(ServoBusSyncRead, a_servo_missing_from_the_burst_leaves_only_its_own_slot_invalid)
{
  // PHASE3 2.105. The other absence: servo 3 is on the bus and answers pings, but never answers
  // the feedback block. The wrapper cannot tell the two apart and must not try -- and note that a
  // silent servo keeps status == 0, which is also what every HEALTHY servo on this bench reports
  // (probe 1 Q1), so the driver must key on `valid` and never on `status == 0` (PHASE3 2.50).
  fake_.set_silent_feedback(3, true);
  const std::vector<uint8_t> ids = {1, 2, 3, 4};

  EXPECT_EQ(bus_.sync_read_feedback(ids, blocks_), 3u);
  ASSERT_EQ(blocks_.size(), 4u);
  expect_block_eq(blocks_[0], seeded_block(1), "slot 0");
  expect_block_eq(blocks_[1], seeded_block(2), "slot 1");
  EXPECT_FALSE(blocks_[2].valid);
  EXPECT_EQ(blocks_[2].status, 0);
  expect_block_eq(blocks_[3], seeded_block(4), "slot 3");
  EXPECT_EQ(bus_.sync_read_stats().missing_frames, 1u);
  // It was still asked, which is what distinguishes this from the absent case.
  EXPECT_EQ(fake_.snapshot(3).sync_reads, 1);
}

TEST_F(ServoBusSyncRead, a_missing_servo_costs_one_io_timeout_for_the_whole_burst)
{
  // PHASE3 2.106 / 2.23. One INST_SYNC_READ is ONE readSCS() for the whole chunk
  // (src/SCS.cpp:318), so a silent servo costs one timeout however many servos are listed -- not
  // one per servo, which is what the per-servo FeedBack() path would have cost. Banded the way
  // a_silent_bus_costs_one_io_timeout is: never an absolute figure.
  ASSERT_TRUE(bus_.set_io_timeout_ms(50));
  fake_.set_silent_feedback(3, true);
  const std::vector<uint8_t> ids = {1, 2, 3, 4};

  const auto started = std::chrono::steady_clock::now();
  EXPECT_EQ(bus_.sync_read_feedback(ids, blocks_), 3u);
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - started);

  EXPECT_GE(elapsed.count(), 40);
  EXPECT_LT(elapsed.count(), 150) << "one silent servo must cost one timeout, not four";
}

TEST_F(ServoBusSyncRead, a_truncated_burst_fails_only_the_servos_it_cut)
{
  // PHASE3 4.T33. A reply cut mid frame is the shape that makes an over-reading parser decode a
  // block from bytes that are not there. The `pos + 21 <= len` guard is checked before any byte
  // of a frame is touched, so 11 of 21 bytes is refused rather than decoded short.
  const std::vector<uint8_t> ids = {1, 2, 3, 4};
  for (const std::size_t cut : {ServoBus::sync_read_frame_bytes, std::size_t{10}}) {
    fake_.set_sync_read_truncate_bytes(cut);
    EXPECT_EQ(bus_.sync_read_feedback(ids, blocks_), 3u) << "cut " << cut;
    ASSERT_EQ(blocks_.size(), 4u);
    expect_block_eq(blocks_[0], seeded_block(1), "cut " + std::to_string(cut) + " slot 0");
    expect_block_eq(blocks_[1], seeded_block(2), "cut " + std::to_string(cut) + " slot 1");
    expect_block_eq(blocks_[2], seeded_block(3), "cut " + std::to_string(cut) + " slot 2");
    EXPECT_FALSE(blocks_[3].valid) << "cut " << cut;
  }
  fake_.set_sync_read_truncate_bytes(0);
}

TEST_F(ServoBusSyncRead, a_frame_with_a_bad_checksum_invalidates_only_its_own_slot)
{
  // PHASE3 4.T34 as R3 rewrites it, and 2.107. Each 21-byte frame is independently verifiable --
  // header, id in its slot, length byte and its own checksum -- so one bad frame says nothing
  // about the next: the walker rejects it, resyncs by ONE byte and carries on. Stopping the parse
  // would turn one corrupted reply into three lost joints.
  //
  // `bad_frames == 1` exactly is only sound because the seeded registers contain no `ff ff` pair
  // (see seed_four_servos): the one-byte resync after the rejected frame would otherwise find a
  // false header inside it and claim a second bad frame.
  fake_.set_sync_read_bad_checksum(2);
  const std::vector<uint8_t> ids = {1, 2, 3, 4};

  EXPECT_EQ(bus_.sync_read_feedback(ids, blocks_), 3u);
  ASSERT_EQ(blocks_.size(), 4u);
  expect_block_eq(blocks_[0], seeded_block(1), "slot 0");
  EXPECT_FALSE(blocks_[1].valid);
  expect_block_eq(blocks_[2], seeded_block(3), "slot 2");
  expect_block_eq(blocks_[3], seeded_block(4), "slot 3");
  const SyncReadStats stats = bus_.sync_read_stats();
  EXPECT_EQ(stats.bad_frames, 1u);
  // The burst was full length, so this is NOT a short burst and nothing is drained: the three
  // failure kinds have to stay distinguishable in the counters (PHASE3 2.116).
  EXPECT_EQ(stats.short_bursts, 0u);
  EXPECT_EQ(stats.drains, 0u);
  fake_.set_sync_read_bad_checksum(0);
}

TEST_F(ServoBusSyncRead, a_frame_carrying_an_unrequested_id_is_discarded)
{
  // PHASE3 2.108. The check syncReadPacketRx's return value never gave us: it accepts any frame
  // anywhere in the buffer that carries the id it was asked about and never looks at the id's
  // SLOT (src/SCS.cpp:352-355). On the bench at a 1 ms timeout that let 71 frames in 3000 reads
  // sit in the wrong slot (probe 3 Q6), each one a foreign servo's data published as this one's.
  fake_.set_reply_id_override(2, 7);
  const std::vector<uint8_t> ids = {1, 2, 3, 4};

  EXPECT_EQ(bus_.sync_read_feedback(ids, blocks_), 3u);
  ASSERT_EQ(blocks_.size(), 4u);
  expect_block_eq(blocks_[0], seeded_block(1), "slot 0");
  EXPECT_FALSE(blocks_[1].valid);
  expect_block_eq(blocks_[2], seeded_block(3), "slot 2");
  expect_block_eq(blocks_[3], seeded_block(4), "slot 3");
  EXPECT_EQ(bus_.sync_read_stats().bad_frames, 1u);
  fake_.set_reply_id_override(2, 0);
}

TEST_F(ServoBusSyncRead, a_frame_with_a_wrong_length_byte_is_discarded)
{
  // PHASE3 2.109. The knob keeps the frame the same length on the wire and repairs its checksum,
  // so the only thing under test is gate step 3. SCS::Read never checks that byte at all
  // (src/SCS.cpp:190-199, recon/packets.md landmine 12); the wrapper does, for one comparison.
  fake_.set_reply_length_delta(2, +1);
  const std::vector<uint8_t> ids = {1, 2, 3, 4};

  EXPECT_EQ(bus_.sync_read_feedback(ids, blocks_), 3u);
  ASSERT_EQ(blocks_.size(), 4u);
  expect_block_eq(blocks_[0], seeded_block(1), "slot 0");
  EXPECT_FALSE(blocks_[1].valid);
  expect_block_eq(blocks_[2], seeded_block(3), "slot 2");
  expect_block_eq(blocks_[3], seeded_block(4), "slot 3");
  EXPECT_EQ(bus_.sync_read_stats().bad_frames, 1u);
  fake_.set_reply_length_delta(2, 0);
}

TEST_F(ServoBusSyncRead, replies_that_arrive_out_of_request_order_are_never_misattributed)
{
  // PHASE3 4.T35. The negative control for the forward-only slot match: every frame is
  // individually perfect and the burst as a whole is backwards. A positional parser reports four
  // successful reads with the data rotated, which is the one failure mode a control loop cannot
  // see. How many decode is deliberately not asserted -- only that nothing decodes as somebody
  // else.
  fake_.set_sync_read_reply_order(waveshare_servos_test::SyncReadReplyOrder::reversed);
  const std::vector<uint8_t> ids = {1, 2, 3, 4};

  bus_.sync_read_feedback(ids, blocks_);
  ASSERT_EQ(blocks_.size(), 4u);
  for (std::size_t slot = 0; slot < 4; slot++) {
    if (blocks_[slot].valid) {
      expect_block_eq(blocks_[slot], seeded_block(ids[slot]), "slot " + std::to_string(slot));
    }
  }
  fake_.set_sync_read_reply_order(waveshare_servos_test::SyncReadReplyOrder::request);
}

TEST_F(ServoBusSyncRead, each_frames_status_byte_lands_on_its_own_servo)
{
  // PHASE3 2.110 / 4.T51 / F18. The status byte is frame byte 4 of the servo's OWN frame, taken
  // out of the same 21 bytes as its data with no bus call in between -- not SCS::Error, which
  // Ping, Ack and Read all rewrite with different meanings, and which syncReadPacketRx leaves
  // holding the PREVIOUS transaction's value for an absent id (measured with a 0xee poison,
  // probe 1 Q4). The four values are the ones that probe doctored a real capture with.
  const std::array<uint8_t, 4> statuses = {0x11, 0x22, 0x44, 0x88};
  for (uint8_t id = 1; id <= 4; id++) {
    fake_.set_status(id, statuses[id - 1]);
  }

  // Once in ascending order, once descending, which is how probe 1 Q4 ruled out positional luck.
  for (const std::vector<uint8_t> & ids :
    {std::vector<uint8_t>{1, 2, 3, 4}, std::vector<uint8_t>{4, 3, 2, 1}})
  {
    ASSERT_EQ(bus_.sync_read_feedback(ids, blocks_), 4u);
    for (std::size_t slot = 0; slot < 4; slot++) {
      expect_block_eq(
        blocks_[slot], seeded_block(ids[slot], statuses[ids[slot] - 1]),
        "id " + std::to_string(ids[slot]));
    }
  }
}

TEST_F(ServoBusSyncRead, sync_read_feedback_leaves_scs_error_untouched)
{
  // PHASE3 2.111 / 2.28. Nothing on this path may read or write SCS::Error. It is one shared byte
  // that Ping, Ack and Read each give a different meaning (src/SCS.cpp:201,261,267,292), so a
  // burst that routed its status bytes through it could not keep them apart for four servos.
  fake_.set_status(2, 0x22);
  bus_.Error = 0xee;
  const std::vector<uint8_t> ids = {1, 2, 3, 4};

  ASSERT_EQ(bus_.sync_read_feedback(ids, blocks_), 4u);
  EXPECT_EQ(bus_.Error, 0xee) << "the sync path must not touch the shared status member";
  EXPECT_EQ(blocks_[1].status, 0x22) << "and must still carry each frame's own status byte";
  EXPECT_EQ(blocks_[0].status, 0x00);
}

TEST_F(ServoBusSyncRead, a_sync_read_signs_its_values_even_when_err_is_set)
{
  // PHASE3 2.99 / F11 / R15. The one intentional difference from the vendored accessors: the sign
  // is applied unconditionally. ReadSpeed/ReadLoad/ReadPos/ReadCurrent(-1) all return the RAW word
  // when SMS_STS::Err is non-zero (src/SMS_STS.cpp:146,168,188,254), so a stale failed FeedBack()
  // anywhere in the process would publish 33768 instead of -1000. Poisoning Err here is what
  // fails the moment anyone routes the sync path back through those accessors.
  //
  // It has to be a BUS case and not a decoder case: decode_feedback_block is a free function over
  // 15 bytes, so setting Err on some unrelated object could not change its answer and the
  // assertion would be vacuous.
  fake_.set_feedback(1, -1000, -250, -300, 121, 41, 1, -700);
  bus_.Err = 1;
  const std::vector<uint8_t> ids = {1};

  ASSERT_EQ(bus_.sync_read_feedback(ids, blocks_), 1u);
  ASSERT_EQ(blocks_.size(), 1u);
  EXPECT_EQ(blocks_[0].position_ticks, -1000);
  EXPECT_EQ(blocks_[0].speed_ticks, -250);
  EXPECT_EQ(blocks_[0].load_raw, -300);
  EXPECT_EQ(blocks_[0].current_counts, -700);
  EXPECT_EQ(bus_.Err, 1) << "and the sync path leaves the accessors' own gate alone (PHASE3 2.29)";
}

TEST_F(ServoBusSyncRead, an_empty_id_list_is_refused_without_touching_the_bus)
{
  // PHASE3 2.112 / 2.20. IDN == 0 would put FF FF FE 04 82 38 0F ~cks on the wire and then wait a
  // whole timeout for zero bytes. The guard lives in the wrapper rather than at each call site so
  // that no caller can forget it (R13's reasoning, applied to the read path).
  const std::vector<uint8_t> none;
  blocks_.assign(4, seeded_block(1));

  EXPECT_EQ(bus_.sync_read_feedback(none, blocks_), 0u);
  EXPECT_TRUE(blocks_.empty());
  EXPECT_EQ(bus_.sync_read_stats().transactions, 0u);
  fake_.wait_quiet();
  EXPECT_EQ(fake_.sync_read_requests(), 0u);
  for (uint8_t id = 1; id <= 4; id++) {
    EXPECT_EQ(fake_.snapshot(id).requests, 0) << "id " << static_cast<int>(id);
  }
}

TEST_F(ServoBusSyncRead, a_closed_bus_refuses_a_sync_read)
{
  // PHASE3 2.113 / 2.21. Not merely pointless but dangerous: syncReadPacketTx ends in readSCS,
  // which does FD_SET(fd, ...) (src/SCSerial.cpp:143-144), and with fd == -1 that shifts by a
  // negative count and, with glibc's _FORTIFY_SOURCE fd_set check compiled in, aborts the whole
  // test binary -- the same abort write_acc's port check exists for (PHASE3 1.30).
  const std::vector<uint8_t> ids = {1, 2, 3, 4};
  ServoBus never_opened;
  EXPECT_EQ(never_opened.sync_read_feedback(ids, blocks_), 0u);
  ASSERT_EQ(blocks_.size(), 4u);
  for (const FeedbackBlock & block : blocks_) {
    EXPECT_FALSE(block.valid);
  }

  bus_.close();
  EXPECT_EQ(bus_.sync_read_feedback(ids, blocks_), 0u);
  EXPECT_EQ(bus_.sync_read_stats().transactions, 0u);
  // and the drain is a no-op on a closed bus too, rather than a select() on -1
  EXPECT_EQ(bus_.drain_input(), 0u);
}

TEST_F(ServoBusSyncRead, an_id_list_longer_than_thirty_is_split_into_chunks)
{
  // PHASE3 2.114 / 2.24. 30 ids per request, the same number the position sync write uses. The
  // vendored limit is far higher -- the request is IDN + 8 bytes into the unchecked 255-byte
  // txBuf -- but the burst must fit ONE io_timeout_ms, and at the measured 0.476 + 0.290n ms
  // (probe 1 Q3) thirty ids is already ~9.2 ms, a whole 100 Hz period.
  std::vector<uint8_t> ids;
  for (uint8_t id = 1; id <= 40; id++) {
    if (id > 4) {
      fake_.add_servo(id);
    }
    fake_.set_position(id, 10 * id);
    ids.push_back(id);
  }

  EXPECT_EQ(bus_.sync_read_feedback(ids, blocks_), 40u);
  ASSERT_EQ(blocks_.size(), 40u);
  for (std::size_t slot = 0; slot < 40; slot++) {
    EXPECT_TRUE(blocks_[slot].valid) << "slot " << slot;
    EXPECT_EQ(blocks_[slot].position_ticks, 10 * static_cast<int>(ids[slot])) << "slot " << slot;
    EXPECT_EQ(fake_.snapshot(ids[slot]).sync_reads, 1) << "slot " << slot;
  }
  // Two transactions, and the member holds the LAST chunk's size -- which is also what makes the
  // split visible from outside at all.
  EXPECT_EQ(bus_.sync_read_stats().transactions, 2u);
  EXPECT_EQ(bus_.syncReadRxBuffMax, 10 * ServoBus::sync_read_frame_bytes);
}

TEST_F(ServoBusSyncRead, a_short_burst_drains_the_line_before_the_next_transaction)
{
  // PHASE3 2.115 / 2.53. The contamination regression, and the case the whole error path exists
  // for. A tcflush -- which is all rFlushSCS does -- can only discard bytes that have ALREADY
  // arrived; the frame that poisons the next cycle is by definition one that has not. On the
  // bench at a 1 ms timeout that produced 8 checksum-valid, correct-id, correct-length STALE
  // frames in 3000 reads, the fastest completing in 0.371 ms against a 0.96 ms physical airtime
  // (probe 3 Q4/Q6) -- data a control loop cannot tell from fresh. Reading the line out to a
  // deadline is the only thing that fixes it: with a drain, all 2000 reads at 1 ms failed
  // honestly instead (probe 3 Q4 row E).
  //
  // Servo 3's reply is held back two 1 ms poll windows, strictly inside io_timeout + drain, so
  // the delayed frame lands during the drain and not during cycle 2's read window.
  ASSERT_TRUE(bus_.set_io_timeout_ms(2));
  fake_.set_reply_delay_polls(3, 2);
  const std::vector<uint8_t> ids = {1, 2, 3, 4};

  fake_.set_position(3, 1234);
  bus_.sync_read_feedback(ids, blocks_);
  ASSERT_EQ(blocks_.size(), 4u);
  EXPECT_FALSE(blocks_[2].valid) << "cycle 1's late frame cannot have arrived in time";

  fake_.set_position(3, 4321);
  bus_.sync_read_feedback(ids, blocks_);
  ASSERT_EQ(blocks_.size(), 4u);
  // The whole point: slot 2 may be empty or may carry cycle 2's value, but NEVER cycle 1's.
  if (blocks_[2].valid) {
    EXPECT_EQ(blocks_[2].position_ticks, 4321) << "cycle 1's frame was published as cycle 2's";
  }
  // And the servos that answered on time are still themselves, in their own slots: a stale frame
  // accepted at the head of the burst would push the cursor past them and lose all three.
  EXPECT_TRUE(blocks_[0].valid);
  EXPECT_EQ(blocks_[0].position_ticks, 1000);
  EXPECT_TRUE(blocks_[1].valid);
  EXPECT_EQ(blocks_[1].position_ticks, 2000);
  EXPECT_TRUE(blocks_[3].valid);
  EXPECT_EQ(blocks_[3].position_ticks, 4000);
  EXPECT_GE(bus_.sync_read_stats().drains, 1u);
  fake_.set_reply_delay_polls(3, 0);
}

TEST_F(ServoBusSyncRead, sync_read_stats_count_transactions_short_bursts_bad_frames_and_missing)
{
  // PHASE3 2.116. The five counters, moved by the three failure kinds one at a time, because the
  // README number Phase 3 item 3 asks for is read straight out of them and a counter that lumps
  // two failures together cannot be reported honestly.
  const std::vector<uint8_t> ids = {1, 2, 3, 4};

  // A clean burst moves the transaction count and nothing else. `drains == 0` is the assertion
  // that the drain really is error-path-only: it costs its whole 2 ms budget when it runs.
  ASSERT_EQ(bus_.sync_read_feedback(ids, blocks_), 4u);
  SyncReadStats stats = bus_.sync_read_stats();
  EXPECT_EQ(stats.transactions, 1u);
  EXPECT_EQ(stats.short_bursts, 0u);
  EXPECT_EQ(stats.bad_frames, 0u);
  EXPECT_EQ(stats.missing_frames, 0u);
  EXPECT_EQ(stats.drains, 0u);

  // A silent servo: fewer bytes than the id list demanded, one id with no frame, and a drain.
  fake_.set_silent_feedback(3, true);
  ASSERT_EQ(bus_.sync_read_feedback(ids, blocks_), 3u);
  stats = bus_.sync_read_stats();
  EXPECT_EQ(stats.transactions, 2u);
  EXPECT_EQ(stats.short_bursts, 1u);
  EXPECT_EQ(stats.bad_frames, 0u);
  EXPECT_EQ(stats.missing_frames, 1u);
  EXPECT_EQ(stats.drains, 1u);
  fake_.set_silent_feedback(3, false);

  // A corrupt frame: every byte arrived, so this is NOT a short burst and nothing is drained --
  // but the id behind the rejected frame went unanswered, so missing_frames moves as well as
  // bad_frames. The two overlap by design; they are not a partition of the burst.
  fake_.set_sync_read_bad_checksum(2);
  ASSERT_EQ(bus_.sync_read_feedback(ids, blocks_), 3u);
  stats = bus_.sync_read_stats();
  EXPECT_EQ(stats.transactions, 3u);
  EXPECT_EQ(stats.short_bursts, 1u);
  EXPECT_EQ(stats.bad_frames, 1u);
  EXPECT_EQ(stats.missing_frames, 2u);
  EXPECT_EQ(stats.drains, 1u);
  fake_.set_sync_read_bad_checksum(0);
}

TEST_F(ServoBusSyncRead, sync_read_feedback_counts_what_it_lost)
{
  // PHASE3 4.T36. The scripted sequence the README's failure number is read out of: four clean
  // bursts, one with an absent servo, one nobody answers. On this bench the real rate is 0 in
  // 193,992 read transactions -- an upper bound of 29 per million at 95% confidence, by the rule
  // of three (probe 3 Q1) -- so what the counter is for is making a DETERMINISTIC failure
  // visible, not sampling a stochastic one.
  const std::vector<uint8_t> ids = {1, 2, 3, 4};
  for (int cycle = 0; cycle < 4; cycle++) {
    ASSERT_EQ(bus_.sync_read_feedback(ids, blocks_), 4u) << "cycle " << cycle;
  }
  fake_.set_absent(2, true);
  EXPECT_EQ(bus_.sync_read_feedback(ids, blocks_), 3u);
  fake_.set_absent(2, false);
  fake_.set_sync_read_supported(false);
  EXPECT_EQ(bus_.sync_read_feedback(ids, blocks_), 0u);

  const SyncReadStats stats = bus_.sync_read_stats();
  EXPECT_EQ(stats.transactions, 6u);
  EXPECT_EQ(stats.missing_frames, 1u + 4u);
  EXPECT_EQ(stats.short_bursts, 2u);
  fake_.set_sync_read_supported(true);
}
