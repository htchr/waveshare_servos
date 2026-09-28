// ServoBus tests over an openpty() pair: no servo or adapter needed.
// See docs/development.md, "Fake servo bus".

#include <gmock/gmock.h>

#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
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
#include <map>
#include <optional>
#include <stdexcept>
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

// A second open is always refused: EBUSY (TIOCEXCL), or EWOULDBLOCK (flock) when run as root.
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

// Reads sizeof(txBuf) of the vendored array itself, not the wrapper's constant.
struct TxBufProbe : ServoBus
{
  size_t bytes() const {return sizeof(txBuf);}
};

// Capture seam: records each frame a ServoBus builds; no byte reaches a port. fd is /dev/null
// (is_open() must pass), and readSCS returns 0, so each Ack() fails at once.
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

// Records the real txBufLen at each flush (the one place it is zeroed), then sends as usual.
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

// Exposes the protected SCS::Host2SCS, so a test pins the static record builders to its byte
// order (End = 0: low byte first).
struct EndiannessProbe : ServoBus
{
  std::array<uint8_t, 2> split(uint16_t value)
  {
    std::array<uint8_t, 2> halves{};
    Host2SCS(&halves[0], &halves[1], value);
    return halves;
  }
};

// One sync-write frame, parsed on its own: ff ff fe mesLen 83 addr nLen {id data}... ~chk.
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
  // mesLen (a u8) wraps only at 32 position / 84 speed records, past the txBuf limit.
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

// Fake servo on the pty master: a 256-byte register file that answers PING, READ and WRITE.
// The destructor stops and joins the thread.
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
      // With no slave open, poll() reports POLLHUP at once every time: sleep, do not spin.
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

  // Discards what is queued on the master now. Call it between large writes: on a full pty,
  // wFlushSCS gives up after 1000 EAGAIN retries and drops the frame silently.
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

// Cases that need a register file. FakeBus has its own pty pair: do not mix it with ServoBusPty.
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
  // 7, not 5: 5 ms is the driver default, and a setter that does nothing must not pass.
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
  // Tests the flock alone (no TIOCEXCL here): it is the lock that root cannot bypass.
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
  EXPECT_EQ(probe, -1) << "TIOCEXCL did not take: screen and minicom could still open it";
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
  // Nothing answers, so each transaction costs one io timeout.
  // See docs/bus-timing.md, "Transaction timeout".
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
  // SCSerial::begin() prints "serial speed <rate>" with no flush; open() must flush stdout.
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
  // writeSCS has no bounds check, so txBuf's size bounds every packet. This reads the vendored
  // array itself; the constructor's static_assert checks the same at compile time.
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

// ---- Record builders and chunk constants: bytes only, no port and no frame ----

TEST(ServoBus, position_record_is_the_seven_bytes_sync_write_pos_ex_would_have_built)
{
  // 7-byte record at register 41: ACC, goal position, goal time (always 0), goal speed.
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
  // -100 is 64 80 (sign-magnitude): never 9c ff (two's complement) or 9c 80.
  const std::array<uint8_t, 7> record = ServoBus::position_record(7, -100, 1);
  EXPECT_EQ(record, (std::array<uint8_t, 7>{0x07, 0x64, 0x80, 0x00, 0x00, 0x01, 0x00}));
  EXPECT_EQ(record[1], 0x64);
  EXPECT_EQ(record[2], 0x80);

  // -32767 is the lowest goal: send_commands clamps to +/-32767 (the library overflows at -32768).
  EXPECT_EQ(
    ServoBus::position_record(0, -32767, 1),
    (std::array<uint8_t, 7>{0x00, 0xff, 0xff, 0x00, 0x00, 0x01, 0x00}));
}

TEST(ServoBus, an_acceleration_of_zero_is_written_as_zero)
{
  // max_accel="0" means no ramp: ACC 0 must go out as 0, never be replaced by a default.
  EXPECT_EQ(ServoBus::position_record(0, 2048, 100)[0], 0);
}

TEST(ServoBus, speed_record_is_sign_magnitude_and_zero_carries_no_sign_bit)
{
  EXPECT_EQ(ServoBus::speed_record(700), (std::array<uint8_t, 2>{0xbc, 0x02}));
  EXPECT_EQ(ServoBus::speed_record(-700), (std::array<uint8_t, 2>{0xbc, 0x82}));
  // Zero is the stop command every deactivation uses: no `>= 1` speed floor and no sign bit.
  EXPECT_EQ(ServoBus::speed_record(0), (std::array<uint8_t, 2>{0x00, 0x00}));
}

TEST(ServoBus, the_record_builders_split_a_word_the_way_the_vendored_library_does)
{
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
  // The literals check the derivation itself: without the id byte's + 1, 30 becomes 35.
  EXPECT_EQ(ServoBus::max_records_per_packet(ServoBus::goal_position_record_bytes), size_t{30});
  EXPECT_EQ(ServoBus::max_records_per_packet(ServoBus::goal_speed_record_bytes), size_t{82});
  EXPECT_EQ(ServoBus::max_goal_positions_per_packet, size_t{30});
  EXPECT_EQ(ServoBus::max_goal_speeds_per_packet, size_t{82});
  // Asserted, never built: 31 or 83 records overrun txBuf, and writeSCS has no bounds check.
  EXPECT_GT(size_t{8 + 31 * 8}, ServoBus::tx_buffer_bytes);
  EXPECT_GT(size_t{8 + 83 * 3}, ServoBus::tx_buffer_bytes);
}

TEST(ServoBus, the_chunk_limits_are_the_largest_packets_that_fit_the_transmit_buffer)
{
  // Double entry: literal frame sizes and the shipped constants, so an edit of either fails.
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
  // Bit 15 is the direction, bits 0..14 the magnitude: -100 is 0x8064 (64 80 on the wire).
  EXPECT_EQ(sign_magnitude_encode(0), 0x0000);
  EXPECT_EQ(sign_magnitude_encode(1), 0x0001);
  EXPECT_EQ(sign_magnitude_encode(-1), 0x8001);
  EXPECT_EQ(sign_magnitude_encode(100), 0x0064);
  EXPECT_EQ(sign_magnitude_encode(-100), 0x8064);
  EXPECT_EQ(sign_magnitude_encode(4095), 0x0fff);
  EXPECT_EQ(sign_magnitude_encode(-4095), 0x8fff);
  EXPECT_EQ(sign_magnitude_encode(32767), 0x7fff);
  EXPECT_EQ(sign_magnitude_encode(-32767), 0xffff);
  // -32768 saturates to 0xffff, like -32767. The library's s16 negation overflows there.
  EXPECT_EQ(sign_magnitude_encode(-32768), 0xffff);
}

TEST(ServoBus, every_write_status_has_a_name)
{
  // A status with no name would reach a log line as an empty string.
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

// ---- Whole frames and chunking over PacketCapture; SCS::syncWrite builds the frames ----

TEST(ServoBus, the_position_packet_is_byte_identical_to_sync_write_pos_ex)
{
  // Golden frame: mesLen 0x24 = (7 + 1) * 4 + 4, address 0x29 (41), checksum 0x78.
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
  // A/B against SyncWritePosEx over eight input classes; the golden frames catch a shared drift.
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
  // Golden n = 1 frame (goal -1234, speed 777, ACC 7): sum 0x327, so checksum ~0x27 = 0xd8.
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
  // Same sync frame as SyncWriteSpe, without its per-servo ACC writes: one frame, not five.
  // See docs/design.md, "Wheel acceleration".
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
  // A/B against SyncWriteSpe over seven input classes; only its last (sync) frame is compared.
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
  // Golden n = 1 speed frame (speed -1500): sum 0x31a, so checksum ~0x1a = 0xe5.
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
  // The library re-encodes its caller's array in place; the wrapper does not, so repeats match.
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

  // Contrast: SyncWriteSpe turns -700 into -32068 in place, so its second frame has 44 fd.
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
  // Inputs stay unchanged; compared field by field, because GoalPosition has no operator==.
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
  // 31 records in one frame would be 256 bytes, one past the end of txBuf.
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
  // 61 goals = 30 + 30 + 1: the last chunk is short, and each chunk has its own checksum.
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
  // 83 records in one frame would be 257 bytes, two past the end of txBuf.
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
  // 165 goals = 82 + 82 + 1.
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
  // An empty group sends nothing. In the library, IDN == 0 is a zero-length VLA (undefined
  // behaviour) and an empty broadcast.
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
  // Read from a captured frame, not the builder, so the record's place in the packet is checked.
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
  // Speed 0 is the stop command: 00 00, never 00 80 (a sign bit with no magnitude).
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
  // Refused as NOT_OPEN so the caller can log it; the library would write(-1) and fail silently.
  ServoBus fresh;
  ASSERT_FALSE(fresh.is_open());
  const WriteResult refused = fresh.write_goal_positions({GoalPosition{1, 0, 0, 0}});
  EXPECT_EQ(refused.status, WriteStatus::NOT_OPEN);
  EXPECT_EQ(refused.packets, 0u);
  EXPECT_EQ(refused.records, 0u);
  EXPECT_FALSE(static_cast<bool>(refused));
  EXPECT_EQ(fresh.write_goal_speeds({GoalSpeed{1, 100}}).status, WriteStatus::NOT_OPEN);
  EXPECT_FALSE(fresh.write_acc(1, 40));

  // Also a bus that was open and then closed: close() sets fd back to -1.
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
  // ACC travels in its own addressed write of register 41. A success returns 1, so only the
  // no-answer case below tells `!= 0` from `!= -1`.
  const waveshare_servos_test::FakeServo before = fake_.snapshot(1);
  EXPECT_TRUE(bus_.write_acc(1, 77));
  const waveshare_servos_test::FakeServo after = fake_.snapshot(1);
  EXPECT_EQ(after.mem[waveshare_servos_test::kRegAcc], 77);
  EXPECT_EQ(after.writes, before.writes + 1) << "one addressed write, and no EPROM unlock";
  // One plain INST_WRITE: register 41 is SRAM, so no unlock/lock of register 55 around it.
  EXPECT_EQ(after.sync_writes.size(), before.sync_writes.size());
  EXPECT_EQ(after.mem[55], before.mem[55]) << "SMS_STS_LOCK must not be written";
  EXPECT_EQ(after.writes, 1) << "exactly one transaction, so no unlock/lock bracket";
}

TEST_F(ServoBusPty, write_acc_reports_a_servo_that_does_not_answer)
{
  // SCS::Ack returns 1 or 0, never -1: write_acc must test != 0, unlike readByte's != -1.
  // See docs/design.md, "Vendored library traps".
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
  // The real txBufLen for 29/30 position and 81/82 speed records. Never build 31 or 83 here:
  // writeSCS has no bounds check (PacketCapture holds the oversized cases).
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

  // Four frames only: this case does not drain the master (see drain_master).
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
  // Empty is checked before the port: an empty group is normal (wheels-only robot, all pruned).
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
  // No allocation after reserve_goal_capacity(); the reserve clamps at one chunk (30/82 records).
  ASSERT_TRUE(static_cast<bool>(bus_.open(port_, kBaudrate, kIoTimeoutMs)));
  bus_.reserve_goal_capacity(30, 82);
  const size_t reserved = bus_.goal_scratch_capacity_bytes();
  EXPECT_GT(reserved, 0u);

  // ~23 kB in 100 cycles: drain each cycle (see drain_master) and check every built byte arrived.
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
  // Ids 0, 254 (broadcast) and 255 (header byte) refuse the whole group before a byte is built.
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
  // A sync write is never acked: wait_quiet() before reading the fake's registers back.
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

// ---- Feedback decoder and sync-read burst walker: free functions over bytes, no port ----

TEST(ServoBus, decode_feedback_block_reproduces_the_sms_sts_accessors)
{
  // Offsets 0-1 position, 2-3 speed, 4-5 load, 6 voltage, 7 temperature, 10 moving, 13-14 current.
  // See docs/design.md, "Feedback block".
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

  // The decoder sees only the 15 data bytes; the walker passes the frame's own status byte.
  EXPECT_EQ(decode_feedback_block(kBlock.data(), 0x2b).status, 0x2b);
}

TEST(ServoBus, decode_feedback_block_takes_load_from_bit_ten_and_the_rest_from_bit_fifteen)
{
  // Load signs on bit 10 and keeps bits 11..15 (0xffff -> -64511), as ReadLoad does. Only the
  // 0xffff row catches a decoder that masks with 0x3ff.
  std::array<uint8_t, 15> block = {};
  const std::array<std::pair<std::array<uint8_t, 2>, int>, 8> loads = {{
    {{{0x10, 0x04}}, -16},        // 0x0410, the measured resting value
    {{{0x10, 0x00}}, 16},         // 0x0010, bit 10 clear, so no sign at all
    {{{0xff, 0x03}}, 1023},       // 0x03ff, the largest positive a real servo reports
    {{{0x00, 0x04}}, 0},          // 0x0400, a negative zero: decodes to 0
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
  // One table for all three bit-15 fields. 0x8000 decodes to 0 (-0), as in the library.
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
  // Moving flag = register 66 (offset 10), between the unnamed registers 64-65 and 67-68.
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
  // Voltage and temperature are plain bytes: 0xff is 255, never -1.
  std::array<uint8_t, 15> block = {};
  block[6] = 0xff;
  block[7] = 0xff;
  const FeedbackBlock decoded = decode_feedback_block(block.data(), 0);
  EXPECT_EQ(decoded.voltage_raw, 255);
  EXPECT_EQ(decoded.temperature_raw, 255);
}

namespace
{

// Real 84-byte sync-read reply (bench, ids 1-4, 1 Mbaud, at rest): four 21-byte frames.
// Registers 67-68 are not zero on real servos; servos 3 and 4 report bit-10 (negative) loads.
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

// Independent checksum: ~(id + length + status + 15 data bytes), low 8 bits; not the wrapper's.
uint8_t frame_checksum(const uint8_t * frame)
{
  uint8_t sum = 0;
  for (std::size_t i = 2; i + 1 < kFrameBytes; i++) {
    sum = static_cast<uint8_t>(sum + frame[i]);
  }
  return static_cast<uint8_t>(~sum);
}

// Recomputes one frame's checksum in place: doctored frames are built here, never copied.
void repair(std::vector<uint8_t> & burst, std::size_t frame)  // NOLINT(runtime/references)
{
  burst[frame * kFrameBytes + kFrameBytes - 1] = frame_checksum(&burst[frame * kFrameBytes]);
}

// The capture with its frames reordered; each frame stays valid, only the order changes.
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
  // Real frames: frame 1 repeats its position in registers 67-68; frames 3, 4 have bit-10 loads.
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
  // Floor = ceil(1.19 * (0.476 + 0.290 n)) + 1 ms, from bench data; values computed by hand.
  // See docs/bus-timing.md, "Timeout floor".
  EXPECT_EQ(ServoBus::min_io_timeout_ms(1), 2u);
  // 3 ms at four servos: 0 failures in 2000 bench transactions.
  EXPECT_EQ(ServoBus::min_io_timeout_ms(4), 3u);
  EXPECT_EQ(ServoBus::min_io_timeout_ms(8), 5u);
  // The claim the new 5 ms default rests on: it stays silent up to nine servos. Nothing else in
  // the suite would catch that going stale.
  EXPECT_EQ(ServoBus::min_io_timeout_ms(9), 5u);
  EXPECT_EQ(ServoBus::min_io_timeout_ms(12), 6u);
  EXPECT_EQ(ServoBus::min_io_timeout_ms(16), 8u);
  EXPECT_EQ(ServoBus::min_io_timeout_ms(30), 12u);

  // Monotonic over the chunk range: a larger bus never gets a smaller floor.
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
  // The real burst against its request list. Checksums first, so a typo in the literal fails here.
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
  // Status = byte 4 of the servo's own frame. SCS::Error would keep a stale value for a silent id.
  std::vector<uint8_t> burst = burst_of({0, 1, 2, 3});
  const std::array<uint8_t, 4> statuses = {0x11, 0x22, 0x44, 0x88};
  for (std::size_t frame = 0; frame < 4; frame++) {
    burst[frame * kFrameBytes + 4] = statuses[frame];
    repair(burst, frame);
  }
  // Hand-computed checksums of the doctored frames: a check on repair() itself.
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
  // Servo 2 is silent, so its frame is missing: slot 1 stays empty and the rest decode.
  // See docs/design.md, "Sync read".
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
  // Reversed frames: the count is not asserted, only that no slot holds another servo's data.
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
  // A stale duplicate of id 1 behind the cursor must be refused: the search only moves forward.
  // A search from slot 0 would overwrite slot 0 with the stale (position 0) sample.
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
  // A rejected frame advances the walk one byte, not 21: three stray bytes shift every frame here.
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
  // The counters are session totals across chunks: add to them, never assign.
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
  // One bad checksum costs one slot and the walk goes on. This holds because frame 2 has no ff ff
  // pair for the one-byte resync to latch on to.
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
  // All-0x01 input makes the library's syncReadPacketRx read 18 bytes past the end; the walker's
  // `pos + 21 <= len` check prevents that.
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

  // Only the length byte is wrong (0x12, not 0x11). SCS::Read never checks it; the walker does.
  std::vector<uint8_t> wrong_length(kBenchBurst.begin(), kBenchBurst.begin() + kFrameBytes);
  wrong_length[3] = 0x12;
  repair(wrong_length, 0);
  EXPECT_EQ(
    parse_sync_read_burst(wrong_length.data(), wrong_length.size(), ids, &blocks, nullptr), 0u);
  EXPECT_FALSE(blocks[0].valid);

  // Parse only `length` bytes: past it, the reused 648-byte buffer still holds the last burst.
  // See docs/design.md, "Receive buffer".
  std::vector<uint8_t> rx(ServoBus::sync_read_buffer_bytes, 0);
  std::copy(kBenchBurst.begin(), kBenchBurst.end(), rx.begin());
  const std::vector<uint8_t> four = {1, 2, 3, 4};
  EXPECT_EQ(
    parse_sync_read_burst(rx.data(), kBenchBurst.size() - 1, four, &blocks, nullptr), 3u);
  ASSERT_EQ(blocks.size(), 4u);
  EXPECT_TRUE(blocks[2].valid);
  EXPECT_FALSE(blocks[3].valid);
}

// ---- FakeBus sync-read replies on their own. Not named ServoBusSyncRead: gtest needs one
// fixture per suite name. ----

namespace
{

// Every field differs per servo, and no reply holds an ff ff pair (a false header).
void seed_four_servos(waveshare_servos_test::FakeBus * fake)
{
  for (uint8_t id = 1; id <= 4; id++) {
    fake->add_servo(id);
    fake->set_feedback(
      id, 1000 * id, 10 * id, -id, static_cast<uint8_t>(120 + id), static_cast<uint8_t>(40 + id),
      static_cast<uint8_t>(id % 2), 100 * id);
  }
}

// One raw sync read via the vendored request builder, into a receive buffer the caller owns.
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
  // A silent servo sends nothing: 63 of 84 bytes arrive and the read waits one full timeout.
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

  // sync_read_named counts requests that name the servo, absent or not; sync_reads does not.
  EXPECT_EQ(fake.snapshot(2).sync_reads, 0);
  EXPECT_EQ(fake.snapshot(2).sync_read_named, 1);
  EXPECT_EQ(fake.snapshot(1).sync_read_named, 1);
  bus.close();
}

TEST(FakeBusSyncRead, the_fake_bus_answers_a_sync_read_with_one_frame_per_listed_servo)
{
  // Request FF FF FE (IDN+4) 82 38 0F ids ~chk. Reply: one status frame per id, in list order.
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
    // A sync read also counts as a feedback read, on purpose.
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
  // The fake's three reply faults, one at a time; each step clears the one before.
  waveshare_servos_test::FakeBus fake;
  seed_four_servos(&fake);
  ServoBus bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));
  std::vector<uint8_t> rx;
  const int whole = 4 * static_cast<int>(kFrameBytes);

  // Step 1: delay, compared with an undelayed read of the same burst.
  const auto prompt_started = std::chrono::steady_clock::now();
  ASSERT_EQ(raw_sync_read(&bus, &rx, {1, 2, 3, 4}), whole);
  const auto prompt = std::chrono::steady_clock::now() - prompt_started;
  fake.set_sync_read_delay_ms(3);
  const auto delayed_started = std::chrono::steady_clock::now();
  EXPECT_EQ(raw_sync_read(&bus, &rx, {1, 2, 3, 4}), whole) << "the delay must not lose bytes";
  const auto delayed = std::chrono::steady_clock::now() - delayed_started;
  EXPECT_GT(delayed, prompt) << "a delayed burst must take measurably longer than a prompt one";
  fake.set_sync_read_delay_ms(0);

  // Step 2: cut 10 bytes, half of the last frame.
  fake.set_sync_read_truncate_bytes(10);
  EXPECT_EQ(raw_sync_read(&bus, &rx, {1, 2, 3, 4}), whole - 10);
  fake.set_sync_read_truncate_bytes(0);

  // Step 3: servo 3's frame keeps its length but fails its checksum.
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

// ---- The sync-read transaction through ServoBus ----

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

// ServoBus on the full FakeBus: several servos, per-servo counters and reply faults.
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
    // Reply faults do not move these: the fake checks only the requests it receives.
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
  // One sync read fills all four slots; blocks[k] is always ids[k]'s reply.
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
  // Every slot is written on every call, so a failed read never republishes an old sample.
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
  // Same decoder on both paths, so equality is exact here (the fake has no ADC noise).
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
  // Equal to FeedBack() plus the seven ReadX(-1) accessors, which the driver no longer calls.
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
  // syncReadRxBuffMax must be n * 21: readSCS returns early only when that many bytes arrive.
  const uint8_t * const buffer = bus_.syncReadRxBuff;
  ASSERT_NE(buffer, nullptr) << "the constructor owns the buffer";
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

  // No allocation after the first call, seen through the caller's vector and syncReadRxBuff.
  const std::size_t capacity = blocks_.capacity();
  for (int cycle = 0; cycle < 100; cycle++) {
    ASSERT_EQ(bus_.sync_read_feedback(all, blocks_), 4u) << "cycle " << cycle;
  }
  EXPECT_EQ(blocks_.capacity(), capacity);
  EXPECT_EQ(bus_.syncReadRxBuff, buffer);
}

TEST_F(ServoBusSyncRead, sync_read_feedback_never_replaces_the_receive_buffer)
{
  // syncReadBegin/End are never called (new[] freed by scalar delete); the constructor's buffer
  // stays for the life of the object.
  const uint8_t * const from_the_constructor = bus_.syncReadRxBuff;
  ASSERT_NE(from_the_constructor, nullptr);
  const std::vector<uint8_t> ids = {1, 2, 3, 4};
  for (int cycle = 0; cycle < 10; cycle++) {
    ASSERT_EQ(bus_.sync_read_feedback(ids, blocks_), 4u) << "cycle " << cycle;
    ASSERT_EQ(bus_.syncReadRxBuff, from_the_constructor) << "cycle " << cycle;
  }
  // close() keeps the buffer: it belongs to the object, not to the session.
  bus_.close();
  EXPECT_EQ(bus_.syncReadRxBuff, from_the_constructor);
}

TEST_F(ServoBusSyncRead, an_absent_servo_fails_alone_and_the_others_still_decode)
{
  // Id 2 is absent: 63 of 84 bytes arrive and only slot 1 is lost.
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
  // A silent servo's slot has status 0, as a healthy one has: callers must test valid, not status.
  // See docs/design.md, "Feedback block".
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
  // One sync read is one readSCS(): a silent servo costs one timeout per chunk, not per servo.
  // See docs/bus-timing.md, "Cost of a silent servo".
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
  // Cut mid-frame: the `pos + 21 <= len` check refuses a short frame before reading it.
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
  // One bad checksum invalidates one slot (seed_four_servos says why bad_frames is exactly 1).
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
  // Full length: not a short burst, so no drain. The failure kinds stay separate in the counters.
  EXPECT_EQ(stats.short_bursts, 0u);
  EXPECT_EQ(stats.drains, 0u);
  fake_.set_sync_read_bad_checksum(0);
}

TEST_F(ServoBusSyncRead, a_frame_carrying_an_unrequested_id_is_discarded)
{
  // A frame with an unrequested id is discarded; syncReadPacketRx never checks the id's slot.
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
  // Length byte off by one, checksum repaired: only the length check (gate step 3) catches it.
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
  // Replies in reverse order: nothing may decode as another servo; the count is not asserted.
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
  // Each status byte comes from its servo's own frame, never from SCS::Error.
  const std::array<uint8_t, 4> statuses = {0x11, 0x22, 0x44, 0x88};
  for (uint8_t id = 1; id <= 4; id++) {
    fake_.set_status(id, statuses[id - 1]);
  }

  // Ascending, then descending: rules out positional luck.
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
  // The sync path must not touch SCS::Error: Ping, Ack and Read each give it a different meaning.
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
  // Signs apply even with Err set; ReadX(-1) would return the raw word (33768, not -1000).
  // A bus case, because the free decoder cannot see Err.
  fake_.set_feedback(1, -1000, -250, -300, 121, 41, 1, -700);
  bus_.Err = 1;
  const std::vector<uint8_t> ids = {1};

  ASSERT_EQ(bus_.sync_read_feedback(ids, blocks_), 1u);
  ASSERT_EQ(blocks_.size(), 1u);
  EXPECT_EQ(blocks_[0].position_ticks, -1000);
  EXPECT_EQ(blocks_[0].speed_ticks, -250);
  EXPECT_EQ(blocks_[0].load_raw, -300);
  EXPECT_EQ(blocks_[0].current_counts, -700);
  EXPECT_EQ(bus_.Err, 1) << "and the sync path leaves the accessors' own gate alone";
}

TEST_F(ServoBusSyncRead, an_empty_id_list_is_refused_without_touching_the_bus)
{
  // IDN 0 would still send a request and wait a full timeout; the wrapper refuses it.
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
  // Refused before readSCS: FD_SET(-1) aborts the process under _FORTIFY_SOURCE.
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
  // 30 ids per sync read: at 0.476 + 0.290 n ms, 30 ids already take ~9.2 ms.
  // See docs/bus-timing.md, "Cost per servo".
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
  // A late frame must be drained, not read as the next cycle's reply: tcflush cannot drop bytes
  // that have not arrived yet. See docs/design.md, "Late frames".
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
  // Five counters, moved one failure kind at a time, so each can be reported on its own.
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

  // A bad frame moves bad_frames and missing_frames both (they overlap by design); no drain.
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
  // Scripted losses: 4 clean bursts, 1 with an absent servo, 1 that nobody answers.
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

// ---- FakeBus features the tools use, tested first. Plain TESTs, because two of them send
// bad-checksum frames on purpose. ----

namespace
{

using waveshare_servos_test::EepromPolicy;
using waveshare_servos_test::FakeBus;
using waveshare_servos_test::FakeServo;
using waveshare_servos_test::FrameRecord;
using waveshare_servos_test::IdWriteAck;
using waveshare_servos_test::TwinReply;
using waveshare_servos_test::WriteRecord;
using waveshare_servos_test::kBroadcastId;
using waveshare_servos_test::kInstPing;
using waveshare_servos_test::kInstRead;
using waveshare_servos_test::kInstReset;
using waveshare_servos_test::kInstSyncWrite;
using waveshare_servos_test::kInstWrite;
using waveshare_servos_test::kRegAcc;
using waveshare_servos_test::kRegId;
using waveshare_servos_test::kRegLock;
using waveshare_servos_test::kRegMode;
using waveshare_servos_test::kRegOffset;
using waveshare_servos_test::kRegPresentPosition;
using waveshare_servos_test::kRegTorqueEnable;

// ServoBus plus raw access: send any bytes or a request with no Ack(), and read what arrives.
struct RawWire : ServoBus
{
  void send_raw(std::vector<uint8_t> bytes)
  {
    rFlushSCS();
    writeSCS(bytes.data(), static_cast<int>(bytes.size()));
    wFlushSCS();
  }

  void send_ping(uint8_t id)
  {
    rFlushSCS();
    writeBuf(id, 0, nullptr, 0, kInstPing);
    wFlushSCS();
  }

  void send_read(uint8_t id, uint8_t address, uint8_t length)
  {
    rFlushSCS();
    writeBuf(id, address, &length, 1, kInstRead);
    wFlushSCS();
  }

  void send_write(uint8_t id, uint8_t address, std::vector<uint8_t> bytes)
  {
    rFlushSCS();
    writeBuf(id, address, bytes.data(), static_cast<uint8_t>(bytes.size()), kInstWrite);
    wFlushSCS();
  }

  // RESET (0x06): FF FF id 02 06 ~checksum, no parameters.
  void send_reset(uint8_t id)
  {
    rFlushSCS();
    writeBuf(id, 0, nullptr, 0, kInstReset);
    wFlushSCS();
  }

  // Up to `bytes` bytes within one io timeout, unparsed.
  std::vector<uint8_t> receive(std::size_t bytes)
  {
    std::vector<uint8_t> got(bytes);
    const int n = readSCS(got.data(), static_cast<int>(bytes));
    got.resize(n > 0 ? static_cast<std::size_t>(n) : 0u);
    return got;
  }
};

int offset_word(const FakeBus & fake, uint8_t id)
{
  const FakeServo servo = fake.snapshot(id);
  return servo.mem[kRegOffset] | (servo.mem[kRegOffset + 1] << 8);
}

}  // namespace

TEST(FakeBusTools, add_servo_seeds_its_id_register)
{
  // Register 5 is the id; the tools refuse a servo whose register 5 differs from its answering id.
  FakeBus fake;
  fake.add_servo(7, 1);
  const FakeServo servo = fake.snapshot(7);
  EXPECT_EQ(servo.mem[kRegId], 7);
  EXPECT_EQ(servo.mem[kRegMode], 1);
  // and the EEPROM shadow starts as the register file's EEPROM half, so a power cycle of a servo
  // nobody wrote to changes nothing
  EXPECT_EQ(servo.eeprom[kRegId], 7);
  EXPECT_EQ(servo.eeprom[kRegMode], 1);

  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));
  EXPECT_EQ(bus.readByte(7, kRegId), 7);
}

TEST(FakeBusTools, descriptors_are_close_on_exec)
{
  // The CLI tests spawn tools on this fake: an inherited master or slave would keep the pty open.
  FakeBus fake;
  const int master_flags = ::fcntl(fake.master_fd(), F_GETFD);
  const int slave_flags = ::fcntl(fake.slave_fd(), F_GETFD);
  ASSERT_NE(master_flags, -1) << std::strerror(errno);
  ASSERT_NE(slave_flags, -1) << std::strerror(errno);
  EXPECT_NE(master_flags & FD_CLOEXEC, 0) << "a spawned tool would inherit the master";
  EXPECT_NE(slave_flags & FD_CLOEXEC, 0) << "a spawned tool would inherit the slave";
}

TEST(FakeBusTools, frame_log_records_requests_to_absent_unknown_and_broadcast_ids)
{
  // The frame log records every request, answered or not: a scan's coverage check reads it.
  FakeBus fake;
  fake.add_servo(1);
  fake.add_servo(2);
  fake.set_absent(2, true);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  EXPECT_EQ(bus.Ping(1), 1);
  EXPECT_EQ(bus.readByte(1, kRegMode), 0);
  EXPECT_EQ(bus.writeByte(1, kRegAcc, 9), 1);
  EXPECT_EQ(bus.Ping(2), -1);       // absent
  EXPECT_EQ(bus.Ping(9), -1);       // unknown
  EXPECT_TRUE(static_cast<bool>(bus.write_goal_speeds({GoalSpeed{1, 100}})));
  fake.wait_quiet();                // a broadcast is never acked, so nothing else waits for it

  const std::vector<FrameRecord> frames = fake.frames();
  ASSERT_EQ(frames.size(), 6u);
  const std::vector<std::vector<uint8_t>> params = {
    {}, {kRegMode, 1}, {kRegAcc, 9}, {}, {}, {46, 2, 1, 100, 0}};
  const std::vector<uint8_t> ids = {1, 1, 1, 2, 9, kBroadcastId};
  const std::vector<uint8_t> instructions = {
    kInstPing, kInstRead, kInstWrite, kInstPing, kInstPing, kInstSyncWrite};
  for (std::size_t i = 0; i < frames.size(); i++) {
    EXPECT_EQ(frames[i].id, ids[i]) << "frame " << i;
    EXPECT_EQ(frames[i].instruction, instructions[i]) << "frame " << i;
    EXPECT_EQ(frames[i].params, params[i]) << "frame " << i;
  }
  EXPECT_EQ(fake.frames_received(), 6u);
  EXPECT_EQ(fake.ping_counts(), (std::map<uint8_t, int>{{1, 1}, {2, 1}, {9, 1}}));
  EXPECT_EQ(fake.writes(), (std::vector<WriteRecord>{WriteRecord{1, kRegAcc, {9}}})) <<
    "a broadcast sync write is not an addressed write";

  fake.clear_frames();
  EXPECT_TRUE(fake.frames().empty());
  EXPECT_EQ(fake.frames_received(), 0u);
  EXPECT_EQ(fake.bytes_received(), 0u) << "clear_frames() starts a new observation window";
}

TEST(FakeBusTools, bytes_received_counts_garbage_and_bad_checksum_frames)
{
  // consume() drops garbage unlogged; bytes_received() counts every byte read, before parsing.
  FakeBus fake;
  fake.add_servo(1);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));
  EXPECT_EQ(fake.bytes_received(), 0u) << "opening the port sends nothing";

  const std::vector<uint8_t> garbage = {0x12, 0x34, 0x56};
  // a ping of id 1 with its checksum (0xfb) inverted
  const std::vector<uint8_t> mis_summed = {0xff, 0xff, 0x01, 0x02, kInstPing, 0x04};
  std::vector<uint8_t> bytes = garbage;
  bytes.insert(bytes.end(), mis_summed.begin(), mis_summed.end());
  bus.send_raw(bytes);
  fake.wait_quiet();

  EXPECT_EQ(fake.bytes_received(), garbage.size() + mis_summed.size());
  EXPECT_EQ(fake.frames_received(), 0u) << "neither reached answer()";
  EXPECT_EQ(fake.bad_checksums(), 1u);
  EXPECT_EQ(fake.snapshot(1).pings, 0);
}

TEST(FakeBusTools, writing_the_id_register_moves_the_servo)
{
  // set_id's whole effect. The map entry moves, counters and knobs with it, so a case can follow
  // one servo across the move.
  FakeBus fake;
  fake.add_servo(4);
  fake.set_status(4, 0x20);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));
  ASSERT_EQ(bus.Ping(4), 4);

  EXPECT_EQ(bus.writeByte(4, kRegId, 253), 1) << "acked from the old id, the default";
  ASSERT_EQ(bus.Ping(253), 253);
  EXPECT_EQ(bus.Ping(4), -1);
  EXPECT_THROW(fake.snapshot(4), std::out_of_range);
  FakeServo moved;
  ASSERT_NO_THROW(moved = fake.snapshot(253));
  EXPECT_EQ(moved.mem[kRegId], 253);
  EXPECT_EQ(moved.eeprom[kRegId], 253) << "the default policy commits it";
  EXPECT_EQ(moved.pings, 2) << "the counters moved with it";
  EXPECT_EQ(moved.writes, 1);
  EXPECT_EQ(moved.status, 0x20) << "and so did the knobs";
  EXPECT_EQ(fake.id_collisions(), 0u);
}

TEST(FakeBusTools, id_write_ack_can_come_from_the_new_id_or_not_at_all)
{
  // The ST3025 acks an id write from the old id (measured). The tools must also accept an ack
  // from the new id or no ack, and the fake can do all three.
  FakeBus fake;
  fake.add_servo(4);
  fake.add_servo(5);
  fake.add_servo(6);
  fake.set_id_write_ack(4, IdWriteAck::new_id);
  fake.set_id_write_ack(5, IdWriteAck::none);
  fake.set_id_write_ack(6, IdWriteAck::new_id);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  bus.send_write(4, kRegId, {253});
  const std::vector<uint8_t> ack = bus.receive(6);
  ASSERT_EQ(ack.size(), 6u);
  EXPECT_EQ(ack[2], 253) << "the ack carries the id the servo has now";
  ASSERT_EQ(bus.Ping(253), 253);

  bus.send_write(5, kRegId, {252});
  EXPECT_TRUE(bus.receive(6).empty()) << "no ack at all";
  ASSERT_EQ(bus.Ping(252), 252);

  // And the vendored writeByte calls an ack from the new id a failure (src/SCS.cpp:279) although
  // the servo moved -- the reason no tool may read an ack as proof either way.
  EXPECT_EQ(bus.writeByte(6, kRegId, 251), 0);
  EXPECT_EQ(bus.Ping(251), 251);
}

TEST(FakeBusTools, moving_onto_a_taken_id_counts_a_collision)
{
  // Two servos on one key cannot be represented, so the move is refused and counted. The tool
  // tests assert id_collisions() == 0 on the way out: a tool that let this happen fails there.
  FakeBus fake;
  fake.add_servo(3);
  fake.add_servo(4);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  EXPECT_EQ(bus.writeByte(4, kRegId, 3), 1);
  EXPECT_EQ(fake.id_collisions(), 1u);
  EXPECT_EQ(fake.snapshot(4).mem[kRegId], 3) << "the register took the write";
  EXPECT_EQ(fake.snapshot(3).mem[kRegId], 3);
  EXPECT_EQ(bus.Ping(4), 4) << "but the servo did not move";
}

TEST(FakeBusTools, drop_when_locked_ignores_eeprom_writes_until_unlocked)
{
  FakeBus fake;
  fake.add_servo(1);
  fake.set_eeprom_policy(EepromPolicy::drop_when_locked);
  fake.set_byte(1, kRegLock, 1);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  EXPECT_EQ(bus.writeByte(1, kRegOffset, 7), 1) << "acked";
  EXPECT_EQ(fake.snapshot(1).mem[kRegOffset], 0) << "and ignored";
  EXPECT_EQ(fake.snapshot(1).eeprom[kRegOffset], 0);
  // SRAM is not EEPROM: torque applies whatever the lock says
  EXPECT_EQ(bus.writeByte(1, kRegTorqueEnable, 1), 1);
  EXPECT_EQ(fake.snapshot(1).mem[kRegTorqueEnable], 1);

  EXPECT_EQ(bus.writeByte(1, kRegLock, 0), 1);
  EXPECT_EQ(bus.writeByte(1, kRegOffset, 7), 1);
  EXPECT_EQ(fake.snapshot(1).mem[kRegOffset], 7);
  EXPECT_EQ(fake.snapshot(1).eeprom[kRegOffset], 7);
}

TEST(FakeBusTools, volatile_when_locked_reverts_at_power_cycle)
{
  // Written while locked (55 = 1): applied at once, lost at power-off.
  // See docs/design.md, "Servo registers".
  FakeBus fake;
  fake.add_servo(1);
  fake.set_eeprom_policy(EepromPolicy::volatile_when_locked);
  fake.set_byte(1, kRegLock, 1);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  EXPECT_EQ(bus.writeByte(1, kRegOffset, 7), 1);
  EXPECT_EQ(fake.snapshot(1).mem[kRegOffset], 7) << "applied";
  EXPECT_EQ(fake.snapshot(1).eeprom[kRegOffset], 0) << "and not committed";
  fake.power_cycle();
  EXPECT_EQ(fake.snapshot(1).mem[kRegOffset], 0) << "so the power cycle takes it back";

  fake.set_byte(1, kRegLock, 1);
  EXPECT_EQ(bus.writeByte(1, kRegId, 9), 1);
  ASSERT_EQ(bus.Ping(9), 9) << "the servo moves at once";
  fake.power_cycle();
  EXPECT_EQ(bus.Ping(1), 1) << "and moves back at power-up";
  EXPECT_EQ(bus.Ping(9), -1);
}

TEST(FakeBusTools, unlocked_write_survives_power_cycle)
{
  // The negative control of the case above: without it, a fake whose power cycle reverted
  // everything would pass that case and fail no tool test that forgets the unlock.
  FakeBus fake;
  fake.add_servo(1);
  fake.set_eeprom_policy(EepromPolicy::volatile_when_locked);
  fake.set_power_up_lock(1);
  fake.set_byte(1, kRegLock, 1);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  EXPECT_EQ(bus.writeByte(1, kRegLock, 0), 1);
  EXPECT_EQ(bus.writeByte(1, kRegOffset, 7), 1);
  EXPECT_EQ(bus.writeByte(1, kRegLock, 1), 1);
  EXPECT_EQ(bus.writeByte(1, kRegTorqueEnable, 1), 1);
  EXPECT_EQ(fake.snapshot(1).eeprom[kRegOffset], 7) << "committed while unlocked";
  fake.power_cycle();
  const FakeServo after = fake.snapshot(1);
  EXPECT_EQ(after.mem[kRegOffset], 7);
  EXPECT_EQ(after.mem[kRegTorqueEnable], 0) << "torque is off at power-up";
  EXPECT_EQ(after.mem[kRegLock], 1) << "and the lock is the power-up value";

  EXPECT_EQ(bus.writeByte(1, kRegLock, 0), 1);
  EXPECT_EQ(bus.writeByte(1, kRegId, 9), 1);
  ASSERT_EQ(bus.Ping(9), 9);
  EXPECT_EQ(bus.writeByte(9, kRegLock, 1), 1);
  fake.power_cycle();
  EXPECT_EQ(bus.Ping(9), 9) << "an unlocked id write survives";
  EXPECT_EQ(bus.Ping(1), -1);
}

TEST(FakeBusTools, default_policy_is_apply_always)
{
  // The fake every earlier suite was written against ignores the lock. The driver's set_mode
  // brackets register 33 with an unlock and a lock, and those suites must see it land.
  FakeBus fake;
  fake.add_servo(1);
  fake.set_byte(1, kRegLock, 1);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  EXPECT_EQ(bus.writeByte(1, kRegOffset, 7), 1);
  EXPECT_EQ(fake.snapshot(1).mem[kRegOffset], 7);
  EXPECT_EQ(fake.snapshot(1).eeprom[kRegOffset], 7) << "committed although 55 reads 1";
  fake.power_cycle();
  EXPECT_EQ(fake.snapshot(1).mem[kRegOffset], 7);
}

TEST(FakeBusTools, calibration_centres_present_and_moves_the_offset_on_bit_11)
{
  // 128 to register 40 sets present to 2048: offset += position - 2048, sign on bit 11
  // (measured on the ST3025). See docs/tools.md, "calibrate_midpoint".
  FakeBus fake;
  fake.add_servo(2);
  fake.add_servo(3);
  fake.set_offset_model(2, true);
  fake.set_offset_model(3, true);
  fake.set_position(2, 1026);
  fake.set_position(3, 3000);
  fake.set_register40_after(3, 1);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  EXPECT_EQ(bus.writeByte(2, kRegTorqueEnable, 128), 1);
  EXPECT_EQ(fake.word(2, kRegPresentPosition), 2048);
  EXPECT_EQ(offset_word(fake, 2), 0x0800 | 1022) << "1026 - 2048 = -1022, sign on bit 11";
  EXPECT_EQ(fake.snapshot(2).eeprom[kRegOffset], 0xfe) << "an EEPROM write";
  EXPECT_EQ(fake.snapshot(2).mem[kRegTorqueEnable], 0) << "128 is not stored";
  EXPECT_EQ(fake.snapshot(2).calibrations, 1);

  EXPECT_EQ(bus.writeByte(3, kRegTorqueEnable, 128), 1);
  EXPECT_EQ(fake.word(3, kRegPresentPosition), 2048);
  EXPECT_EQ(offset_word(fake, 3), 952) << "3000 - 2048, bit 11 clear";
  EXPECT_EQ(fake.snapshot(3).mem[kRegTorqueEnable], 1) << "set_register40_after";

  // a write to 31-32 moves present too: offset 0 puts servo 2 back at its physical 1026
  EXPECT_EQ(bus.writeWord(2, kRegOffset, 0), 1);
  EXPECT_EQ(fake.word(2, kRegPresentPosition), 1026);
}

TEST(FakeBusTools, offset_model_off_stores_128_and_nothing_else)
{
  // "Calibration unsupported": today's fake, which the driver suites rely on, and the firmware
  // calibrate_midpoint must report as not applied.
  FakeBus fake;
  fake.add_servo(2);
  fake.set_position(2, 1026);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  EXPECT_EQ(bus.writeByte(2, kRegTorqueEnable, 128), 1);
  EXPECT_EQ(fake.writes(), (std::vector<WriteRecord>{WriteRecord{2, kRegTorqueEnable, {128}}}));
  const FakeServo after = fake.snapshot(2);
  EXPECT_EQ(after.mem[kRegTorqueEnable], 128);
  EXPECT_EQ(fake.word(2, kRegPresentPosition), 1026);
  EXPECT_EQ(offset_word(fake, 2), 0);
  EXPECT_EQ(after.calibrations, 0);
}

TEST(FakeBusTools, doubled_twin_leaves_six_extra_bytes_after_a_ping)
{
  FakeBus fake;
  fake.add_servo(1);
  fake.set_twin(1, TwinReply::doubled);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  EXPECT_EQ(bus.Ping(1), 1) << "the first copy is a perfect reply";
  EXPECT_EQ(bus.drain_input(), 6u) << "and the second is still on the line";
  // writes apply once, however many servos ack them
  EXPECT_EQ(bus.writeByte(1, kRegAcc, 9), 1);
  EXPECT_EQ(bus.drain_input(), 6u);
  EXPECT_EQ(fake.snapshot(1).writes, 1);
  EXPECT_EQ(fake.snapshot(1).mem[kRegAcc], 9);
}

TEST(FakeBusTools, garbled_twin_fails_the_ping)
{
  FakeBus fake;
  fake.add_servo(1);
  fake.set_twin(1, TwinReply::garbled);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  EXPECT_EQ(bus.Ping(1), -1);
  EXPECT_EQ(bus.readByte(1, kRegMode), -1);
  EXPECT_EQ(bus.writeByte(1, kRegAcc, 9), 0) << "the ack is garbled too";
  EXPECT_EQ(fake.snapshot(1).mem[kRegAcc], 9) << "but the write applied, once";
  EXPECT_EQ(fake.snapshot(1).writes, 1);
  EXPECT_EQ(fake.snapshot(1).pings, 1) << "it answered; the answer was unreadable";
}

TEST(FakeBusTools, garbled_reads_twin_pings_clean_and_garbles_reads)
{
  // Twins at different positions: identical pings, disagreeing READ replies.
  FakeBus fake;
  fake.add_servo(1);
  fake.set_twin(1, TwinReply::garbled_reads);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  EXPECT_EQ(bus.Ping(1), 1);
  EXPECT_EQ(bus.readByte(1, kRegMode), -1);
  EXPECT_EQ(bus.readWord(1, kRegPresentPosition), -1);
  EXPECT_EQ(bus.writeByte(1, kRegAcc, 9), 1) << "write acks are not READ replies";
}

TEST(FakeBusTools, silent_read_only_silences_that_address)
{
  FakeBus fake;
  fake.add_servo(1);
  fake.set_byte(1, 3, 9);
  fake.set_byte(1, 4, 3);
  fake.set_silent_read(1, 3);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  EXPECT_EQ(bus.readByte(1, 3), -1);
  EXPECT_EQ(bus.readWord(1, 3), -1) << "any READ that starts there";
  EXPECT_EQ(bus.readByte(1, 4), 3) << "and no other";
  EXPECT_EQ(bus.readWord(1, 2), 9 << 8) << "a READ that starts before it still answers";
  EXPECT_EQ(bus.Ping(1), 1);
  fake.set_silent_read(1, -1);
  EXPECT_EQ(bus.readByte(1, 3), 9);
}

TEST(FakeBusTools, write_acks_off_applies_without_acking)
{
  // Register 8 = 0: a servo that answers READ and PING and nothing else.
  FakeBus fake;
  fake.add_servo(1);
  fake.set_write_acks(1, false);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  EXPECT_EQ(bus.writeByte(1, kRegAcc, 9), 0);
  EXPECT_EQ(fake.snapshot(1).mem[kRegAcc], 9);
  EXPECT_EQ(bus.Ping(1), 1);
  EXPECT_EQ(bus.readByte(1, kRegAcc), 9);
}

TEST(FakeBusTools, ignore_write_acks_but_does_not_apply)
{
  // A firmware that refuses one register and says nothing: the case a read-back exists for.
  FakeBus fake;
  fake.add_servo(1);
  fake.set_byte(1, kRegLock, 1);
  fake.set_ignore_write(1, kRegLock, true);
  fake.set_ignore_write(1, kRegOffset + 1, true);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  EXPECT_EQ(bus.writeByte(1, kRegLock, 0), 1) << "acked";
  EXPECT_EQ(fake.snapshot(1).mem[kRegLock], 1) << "and not applied";
  EXPECT_EQ(bus.writeWord(1, kRegOffset, 0x0102), 1);
  EXPECT_EQ(fake.snapshot(1).mem[kRegOffset], 0x02) << "the other byte of the write applies";
  EXPECT_EQ(fake.snapshot(1).mem[kRegOffset + 1], 0x00);

  fake.set_ignore_write(1, kRegLock, false);
  EXPECT_EQ(bus.writeByte(1, kRegLock, 0), 1);
  EXPECT_EQ(fake.snapshot(1).mem[kRegLock], 0);
}

TEST(FakeBusTools, vanish_after_id_write)
{
  // A servo that answers at no id after an id change.
  FakeBus fake;
  fake.add_servo(4);
  fake.set_vanish_after_id_write(4);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  EXPECT_EQ(bus.writeByte(4, kRegId, 253), 0) << "no ack: it is gone";
  EXPECT_EQ(bus.Ping(4), -1);
  EXPECT_EQ(bus.Ping(253), -1);
  FakeServo moved;
  ASSERT_NO_THROW(moved = fake.snapshot(253)) << "keyed at the id it was given";
  EXPECT_TRUE(moved.absent);
  EXPECT_EQ(moved.mem[kRegId], 253);
}

TEST(FakeBusTools, eeprom_commit_delays_the_ack_and_ignores_requests_meanwhile)
{
  // A commit that outlasts the ack window: the late ack lands in a later transaction's window.
  // See docs/design.md, "Late acks".
  FakeBus fake;
  fake.add_servo(1);
  fake.set_eeprom_commit_ms(1, 100);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  const auto started = std::chrono::steady_clock::now();
  bus.send_write(1, kRegOffset, {7});
  EXPECT_TRUE(bus.receive(6).empty()) << "a 20 ms window cannot see a 100 ms commit's ack";
  EXPECT_EQ(bus.Ping(1), -1) << "a request inside the commit window is ignored";
  ASSERT_TRUE(bus.set_io_timeout_ms(500));
  const std::vector<uint8_t> ack = bus.receive(6);
  const auto acked_after = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - started);
  ASSERT_EQ(ack.size(), 6u) << "the late ack still arrives, on its own";
  EXPECT_EQ(ack[2], 1);
  EXPECT_EQ(ack[3], 2) << "a bare status frame";
  EXPECT_GE(acked_after.count(), 100);

  EXPECT_EQ(fake.snapshot(1).mem[kRegOffset], 7) << "the write itself applied at once";
  EXPECT_EQ(fake.snapshot(1).pings, 0) << "the ignored ping never reached the servo";
  EXPECT_EQ(fake.ping_counts(), (std::map<uint8_t, int>{{1, 1}})) << "but it is in the log";
  ASSERT_TRUE(bus.set_io_timeout_ms(kIoTimeoutMs));
  EXPECT_EQ(bus.Ping(1), 1) << "after the window the servo answers again";
  EXPECT_EQ(bus.writeByte(1, kRegAcc, 5), 1) << "an SRAM write is no commit and acks at once";
}

TEST(FakeBusTools, a_calibration_is_an_eeprom_write_for_commit_latency)
{
  // With the offset model on, 128 to register 40 writes 31-32, so it commits like one.
  FakeBus fake;
  fake.add_servo(2);
  fake.add_servo(3);
  fake.set_offset_model(2, true);
  fake.set_position(2, 1026);
  fake.set_eeprom_commit_ms(2, 100);
  fake.set_eeprom_commit_ms(3, 100);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  const auto started = std::chrono::steady_clock::now();
  bus.send_write(2, kRegTorqueEnable, {128});
  EXPECT_TRUE(bus.receive(6).empty()) << "the calibration's ack waits for its commit";
  ASSERT_TRUE(bus.set_io_timeout_ms(500));
  const std::vector<uint8_t> ack = bus.receive(6);
  const auto acked_after = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - started);
  ASSERT_EQ(ack.size(), 6u);
  EXPECT_EQ(ack[2], 2);
  EXPECT_GE(acked_after.count(), 100);
  ASSERT_TRUE(bus.set_io_timeout_ms(kIoTimeoutMs));
  EXPECT_EQ(fake.word(2, kRegPresentPosition), 2048);
  EXPECT_EQ(fake.snapshot(2).calibrations, 1);

  // A torque write is SRAM, and with the model off so is 128: neither waits.
  EXPECT_EQ(bus.writeByte(2, kRegTorqueEnable, 1), 1);
  EXPECT_EQ(bus.writeByte(3, kRegTorqueEnable, 128), 1);
}

TEST(FakeBusTools, addressed_faults_are_off_by_default_and_act_when_on)
{
  // The sync-read fault knobs, reused for addressed replies only when asked: the sync-read suites
  // set them on servos they also ping, and a ping that suddenly failed would break them.
  FakeBus fake;
  fake.add_servo(1);
  fake.set_reply_id_override(1, 9);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));
  EXPECT_EQ(bus.Ping(1), 1) << "off by default";

  fake.set_faults_apply_to_addressed(true);
  bus.send_ping(1);
  std::vector<uint8_t> reply = bus.receive(6);
  ASSERT_EQ(reply.size(), 6u);
  EXPECT_EQ(reply[2], 9) << "the reply claims id 9";
  EXPECT_EQ(bus.Ping(1), -1);
  fake.set_reply_id_override(1, 0);

  fake.set_reply_length_delta(1, 1);
  bus.send_read(1, kRegMode, 1);
  reply = bus.receive(7);
  ASSERT_EQ(reply.size(), 7u) << "the same wire length";
  EXPECT_EQ(reply[3], 4) << "one more than a 1-byte READ reply's 3";
  EXPECT_EQ(bus.Ping(1), -1) << "SCS::Ping checks the length byte (src/SCS.cpp:251)";
  fake.set_reply_length_delta(1, 0);

  fake.set_reply_checksum_corrupt(1, true);
  EXPECT_EQ(bus.readByte(1, kRegMode), -1);
  fake.set_reply_checksum_corrupt(1, false);
  EXPECT_EQ(bus.Ping(1), 1);
  EXPECT_EQ(bus.readByte(1, kRegMode), 0);
}

TEST(FakeBusTools, drop_pings_ignores_exactly_n)
{
  // The retry a scan and a set_id make: a servo found only on the last attempt.
  FakeBus fake;
  fake.add_servo(1);
  fake.drop_pings(1, 2);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  EXPECT_EQ(bus.Ping(1), -1);
  EXPECT_EQ(bus.Ping(1), -1);
  EXPECT_EQ(bus.Ping(1), 1);
  EXPECT_EQ(bus.Ping(1), 1);
  EXPECT_EQ(fake.ping_counts(), (std::map<uint8_t, int>{{1, 4}}));

  fake.drop_pings(1, 1);
  EXPECT_EQ(bus.readByte(1, kRegMode), 0) << "only pings are dropped";
  EXPECT_EQ(bus.Ping(1), -1);
  EXPECT_EQ(bus.Ping(1), 1);
}

TEST(FakeBusTools, a_twin_or_a_silent_read_can_be_limited_to_the_next_n)
{
  // A tool must not forget one odd reply (twins out of step, a noisy line) when a retry is clean.
  FakeBus fake;
  fake.add_servo(1);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  fake.set_twin(1, TwinReply::garbled, 1);
  EXPECT_EQ(bus.Ping(1), -1) << "the first reply is garbled";
  EXPECT_EQ(bus.Ping(1), 1) << "and only the first";

  fake.set_twin(1, TwinReply::doubled_reads, 1);
  EXPECT_EQ(bus.Ping(1), 1) << "a ping is not a READ reply";
  EXPECT_EQ(bus.drain_input(), 0u);
  EXPECT_EQ(bus.readByte(1, kRegMode), 0) << "the first copy of a doubled read is perfect";
  EXPECT_EQ(bus.drain_input(), 7u) << "and the second is still on the line";
  EXPECT_EQ(bus.readByte(1, kRegMode), 0);
  EXPECT_EQ(bus.drain_input(), 0u) << "the next read comes back once";

  fake.set_twin(1, TwinReply::garbled_reads, 2);
  EXPECT_EQ(bus.readByte(1, kRegMode), -1);
  EXPECT_EQ(bus.Ping(1), 1) << "a ping does not use up the count";
  EXPECT_EQ(bus.readByte(1, kRegMode), -1);
  EXPECT_EQ(bus.readByte(1, kRegMode), 0);

  fake.set_silent_read(1, kRegMode, 1);
  EXPECT_EQ(bus.readByte(1, kRegMode), -1);
  EXPECT_EQ(bus.readByte(1, kRegMode), 0) << "one silent read, then it answers again";
}

TEST(FakeBusTools, garbled_write_acks_break_the_ack_and_nothing_else)
{
  // A collision on the ack alone: the write applies, and reads and pings stay clean.
  FakeBus fake;
  fake.add_servo(1);
  fake.set_garble_write_acks(1, true);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  EXPECT_EQ(bus.writeByte(1, kRegAcc, 9), 0) << "the ack fails its checksum";
  EXPECT_EQ(fake.snapshot(1).mem[kRegAcc], 9) << "but the write applied, once";
  EXPECT_EQ(fake.snapshot(1).writes, 1);
  EXPECT_EQ(bus.Ping(1), 1);
  EXPECT_EQ(bus.readByte(1, kRegAcc), 9);
}

// ---- factory_reset: the fake's RESET model ----

TEST(FakeBusTools, reset_restores_the_factory_table_keeps_the_id_and_reinitialises_sram)
{
  // Measured on the bench: RESET restores EEPROM from register 6 on, keeps the id, and resets
  // torque, goal and lock. Bytes 0..4 (version) are read-only.
  FakeBus fake;
  fake.add_servo(4, 1);
  fake.set_byte(4, 3, 10);
  fake.set_byte(4, 4, 25);
  fake.set_byte(4, 6, 1);
  fake.set_word(4, kRegOffset, 100, 11);
  fake.set_byte(4, 37, 26);
  fake.set_factory_byte(4, 3, 99);          // never used: 3 is read-only
  fake.set_factory_byte(4, kRegId, 1);      // never used: the id is kept
  fake.set_factory_byte(4, 37, 25);
  fake.set_byte(4, kRegTorqueEnable, 1);
  fake.set_word(4, waveshare_servos_test::kRegGoalPosition, 2838);
  fake.set_byte(4, kRegLock, 0);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  bus.send_reset(4);
  const std::vector<uint8_t> ack = bus.receive(6);
  EXPECT_EQ(ack, (std::vector<uint8_t>{0xff, 0xff, 4, 2, 0, 0xf9})) << "the bench's own ack";
  const FakeServo after = fake.snapshot(4);
  EXPECT_EQ(after.resets, 1);
  EXPECT_EQ(after.mem[3], 10);
  EXPECT_EQ(after.mem[4], 25);
  EXPECT_EQ(after.mem[kRegId], 4);
  EXPECT_EQ(after.mem[6], 0);
  EXPECT_EQ(after.mem[kRegOffset], 0);
  EXPECT_EQ(after.mem[kRegOffset + 1], 0);
  EXPECT_EQ(after.mem[kRegMode], 0);
  EXPECT_EQ(after.mem[37], 25);
  EXPECT_EQ(after.mem[kRegTorqueEnable], 0);
  EXPECT_EQ(fake.word(4, waveshare_servos_test::kRegGoalPosition), 0);
  EXPECT_EQ(after.mem[kRegLock], 1) << "the reset closes the lock";
  EXPECT_EQ(fake.resets(), (std::vector<uint8_t>{4}));
  EXPECT_EQ(bus.Ping(4), 4);
}

TEST(FakeBusTools, reset_is_a_flash_write_whatever_the_lock_says)
{
  // RESET is not a WRITE: its values survive a power cycle even with the lock closed (measured).
  // See docs/tools.md, "factory_reset".
  FakeBus fake;
  fake.add_servo(4, 1);
  fake.set_eeprom_policy(EepromPolicy::volatile_when_locked);
  fake.set_power_up_lock(1);
  fake.set_byte(4, kRegLock, 1);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  bus.send_reset(4);
  EXPECT_EQ(bus.receive(6).size(), 6u);
  fake.power_cycle();
  EXPECT_EQ(fake.snapshot(4).mem[kRegMode], 0);
  EXPECT_EQ(fake.snapshot(4).eeprom[kRegMode], 0);
}

TEST(FakeBusTools, reset_ack_waits_for_the_commit_only_when_a_byte_changed)
{
  // Measured: the RESET ack takes ~25 ms when flash changes, under 1 ms when nothing differs.
  FakeBus fake;
  fake.add_servo(4, 1);
  fake.set_eeprom_commit_ms(4, 60);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  bus.send_reset(4);
  EXPECT_TRUE(bus.receive(6).empty()) << "a 20 ms window cannot see a 60 ms commit's ack";
  EXPECT_EQ(bus.Ping(4), -1) << "a request inside the commit is ignored";
  ASSERT_TRUE(bus.set_io_timeout_ms(200));
  EXPECT_EQ(bus.receive(6).size(), 6u) << "the late ack still comes";
  ASSERT_TRUE(bus.set_io_timeout_ms(kIoTimeoutMs));

  bus.send_reset(4);
  EXPECT_EQ(bus.receive(6).size(), 6u) << "already factory: no commit, an ack at once";
  EXPECT_EQ(fake.snapshot(4).resets, 2);
}

TEST(FakeBusTools, reset_knobs_ignore_it_or_skip_a_register)
{
  FakeBus fake;
  fake.add_servo(4, 1);
  fake.add_servo(5, 1);
  fake.set_reset_supported(4, false);
  fake.set_reset_skip(5, kRegMode, true);
  fake.set_byte(5, 6, 1);
  RawWire bus;
  ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));

  bus.send_reset(4);
  EXPECT_TRUE(bus.receive(6).empty()) << "an unknown instruction gets no reply";
  EXPECT_EQ(fake.snapshot(4).mem[kRegMode], 1) << "and changes nothing";
  EXPECT_EQ(fake.snapshot(4).resets, 1) << "but it reached the servo";

  bus.send_reset(5);
  EXPECT_EQ(bus.receive(6).size(), 6u);
  EXPECT_EQ(fake.snapshot(5).mem[kRegMode], 1) << "skipped";
  EXPECT_EQ(fake.snapshot(5).mem[6], 0) << "the rest is reset";
  EXPECT_EQ(fake.resets(), (std::vector<uint8_t>{4, 5}));

  fake.set_byte(5, kRegTorqueEnable, 1);
  fake.set_byte(5, kRegLock, 0);
  fake.set_reset_keeps_sram(5, true);
  bus.send_reset(5);
  EXPECT_EQ(bus.receive(6).size(), 6u);
  EXPECT_EQ(fake.snapshot(5).mem[kRegTorqueEnable], 1) << "SRAM kept";
  EXPECT_EQ(fake.snapshot(5).mem[kRegLock], 0);
}

TEST(FakeBusTools, baud_model_hears_only_the_rate_register_6_names)
{
  // Off by default. On: register 6 = 1 answers at 500000 only; after RESET the ack comes at
  // 500000, and then only 1000000 works.
  FakeBus fake;
  fake.add_servo(4);
  fake.set_byte(4, 6, 1);
  {
    RawWire bus;
    ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));
    EXPECT_EQ(bus.Ping(4), 4) << "no baud model by default";
  }
  fake.set_baud_model(true);
  {
    RawWire bus;
    ASSERT_TRUE(static_cast<bool>(bus.open(fake.port(), kBaudrate, kIoTimeoutMs)));
    EXPECT_EQ(bus.Ping(4), -1) << "1000000 is noise to a servo at register 6 = 1";
  }
  RawWire slow;
  ASSERT_TRUE(static_cast<bool>(slow.open(fake.port(), 500000, kIoTimeoutMs)));
  EXPECT_EQ(slow.Ping(4), 4);
  slow.send_reset(4);
  EXPECT_EQ(slow.receive(6).size(), 6u) << "the ack at the old rate";
  EXPECT_EQ(slow.Ping(4), -1) << "and nothing more at it";
  slow.close();
  RawWire fast;
  ASSERT_TRUE(static_cast<bool>(fast.open(fake.port(), kBaudrate, kIoTimeoutMs)));
  EXPECT_EQ(fast.Ping(4), 4) << "the factory rate";
}

// ---- Checked transactions. Each reply kind comes from the wire: twin, foreign id, late ack ----

namespace
{

using waveshare_servos::Reply;
using waveshare_servos::ReplyKind;

}  // namespace

namespace waveshare_servos
{

// gtest prints an enum it has no printer for as "1-byte object <00>"; with this a failed kind
// reads "not_open" instead. Found by argument-dependent lookup, hence this namespace.
void PrintTo(ReplyKind kind, std::ostream * out)
{
  *out << to_string(kind);
}

}  // namespace waveshare_servos

namespace
{

class ServoBusChecked : public ::testing::Test
{
protected:
  void SetUp() override
  {
    fake_.add_servo(1);
    ASSERT_TRUE(static_cast<bool>(bus_.open(fake_.port(), kBaudrate, kIoTimeoutMs)));
  }

  void TearDown() override
  {
    bus_.close();
    // Every fault below is in a REPLY; a request the fake could not verify means the wire itself
    // misbehaved and the case proved nothing.
    EXPECT_EQ(fake_.bad_checksums(), 0u) << "the wire itself misbehaved";
    EXPECT_EQ(fake_.quiet_timeouts(), 0u) << "wait_quiet gave up";
    EXPECT_EQ(fake_.id_collisions(), 0u);
  }

  FakeBus fake_;
  ServoBus bus_;                        // declared after fake_, so it is destroyed first
};

}  // namespace

TEST_F(ServoBusChecked, checked_ping_silent_costs_one_timeout_and_zero_bytes)
{
  // A scan pings 254 ids 3 times, mostly silent: a silent id must cost one window, no drain.
  ASSERT_TRUE(bus_.set_io_timeout_ms(5));
  const Reply reply = bus_.checked_ping(9);
  EXPECT_EQ(reply.kind, ReplyKind::SILENT);
  EXPECT_FALSE(static_cast<bool>(reply));
  EXPECT_EQ(reply.bytes, 0u);
  EXPECT_EQ(reply.extra_bytes, 0u);
  EXPECT_EQ(reply.frame_bytes, 0u);
  EXPECT_EQ(reply.from_id, -1);
  EXPECT_GE(reply.elapsed_us, 5000u) << "the whole window";

  // Compare the fastest call of each kind: a drain adds 2 ms to every call, a stall only to some.
  constexpr int kPings = 10;
  std::chrono::steady_clock::duration vendored = std::chrono::hours(1);
  std::chrono::steady_clock::duration checked = std::chrono::hours(1);
  for (int i = 0; i < kPings; i++) {
    const auto started = std::chrono::steady_clock::now();
    EXPECT_EQ(bus_.Ping(9), -1);
    vendored = std::min(vendored, std::chrono::steady_clock::now() - started);
  }
  for (int i = 0; i < kPings; i++) {
    const auto started = std::chrono::steady_clock::now();
    EXPECT_EQ(bus_.checked_ping(9).kind, ReplyKind::SILENT);
    checked = std::min(checked, std::chrono::steady_clock::now() - started);
  }
  // in microseconds, so a failure prints numbers; the margin is half of a 2 ms drain
  const int64_t checked_us =
    std::chrono::duration_cast<std::chrono::microseconds>(checked).count();
  const int64_t vendored_us =
    std::chrono::duration_cast<std::chrono::microseconds>(vendored).count();
  EXPECT_LT(checked_us, vendored_us + 1000) << "silence paid for a drain";
  fake_.wait_quiet();
  EXPECT_EQ(fake_.ping_counts(), (std::map<uint8_t, int>{{9, 2 * kPings + 1}})) <<
    "one request per call: no retry hides inside";
}

TEST_F(ServoBusChecked, checked_ping_one_servo_is_ONE_with_its_status)
{
  fake_.set_status(1, 0x20);
  const Reply reply = bus_.checked_ping(1);
  EXPECT_EQ(reply.kind, ReplyKind::ONE);
  EXPECT_TRUE(static_cast<bool>(reply));
  EXPECT_EQ(reply.from_id, 1);
  // SCS::Ping overwrites SCS::Error with the id byte (src/SCS.cpp:261); this is the real one
  EXPECT_EQ(reply.status, 0x20);
  EXPECT_EQ(reply.frame_bytes, 6u);
  EXPECT_EQ(reply.bytes, 6u);
  EXPECT_EQ(reply.extra_bytes, 0u);
  EXPECT_TRUE(reply.data.empty());
  EXPECT_GT(reply.elapsed_us, 0u);
  EXPECT_LT(reply.elapsed_us, kIoTimeoutMs * 1000u) << "a complete frame ends the window early";
}

TEST_F(ServoBusChecked, checked_ping_doubled_twin_is_EXTRA_with_six_extra_bytes)
{
  // Two servos on one id answering in bit synchrony: the first reply is perfect, the second is
  // the only evidence, and it is drained and counted instead of left for the next transaction.
  fake_.set_twin(1, TwinReply::doubled);
  const Reply reply = bus_.checked_ping(1);
  EXPECT_EQ(reply.kind, ReplyKind::EXTRA);
  EXPECT_FALSE(static_cast<bool>(reply)) << "only ONE is a clean answer";
  EXPECT_EQ(reply.from_id, 1);
  EXPECT_EQ(reply.frame_bytes, 6u);
  EXPECT_EQ(reply.bytes, 6u);
  EXPECT_EQ(reply.extra_bytes, 6u);
  fake_.set_twin(1, TwinReply::none);
  EXPECT_EQ(bus_.checked_ping(1).kind, ReplyKind::ONE) << "nothing of the copy was left behind";
}

TEST_F(ServoBusChecked, checked_ping_garbled_is_GARBLED)
{
  fake_.set_twin(1, TwinReply::garbled);
  const Reply reply = bus_.checked_ping(1);
  EXPECT_EQ(reply.kind, ReplyKind::GARBLED);
  EXPECT_FALSE(static_cast<bool>(reply));
  EXPECT_EQ(reply.bytes, 6u) << "bytes arrived: this is not an absence";
  EXPECT_EQ(reply.from_id, -1) << "no well-formed frame, so no id to believe";
  EXPECT_EQ(reply.frame_bytes, 0u);
}

TEST_F(ServoBusChecked, checked_ping_reply_from_another_id_is_WRONG_ID_with_frame_bytes_6)
{
  // A late ack caught by a ping looks like this: a well-formed 6-byte frame from another id.
  fake_.set_faults_apply_to_addressed(true);
  fake_.set_reply_id_override(1, 7);
  const Reply reply = bus_.checked_ping(1);
  EXPECT_EQ(reply.kind, ReplyKind::WRONG_ID);
  EXPECT_FALSE(static_cast<bool>(reply));
  EXPECT_EQ(reply.from_id, 7);
  EXPECT_EQ(reply.frame_bytes, 6u);
  EXPECT_EQ(reply.extra_bytes, 0u);
}

TEST_F(ServoBusChecked, checked_calls_refuse_0xfe_and_0xff_without_sending)
{
  // 0xfe is the broadcast every servo obeys and none answers, 0xff a header byte. A checked call
  // exists to hear ONE servo, so neither ever reaches the wire.
  const uint8_t value = 1;
  for (const uint8_t id : {uint8_t{0xfe}, uint8_t{0xff}}) {
    EXPECT_EQ(bus_.checked_ping(id).kind, ReplyKind::INVALID_ID) << static_cast<int>(id);
    EXPECT_EQ(bus_.checked_read(id, kRegMode, 1).kind, ReplyKind::INVALID_ID) <<
      static_cast<int>(id);
    EXPECT_EQ(
      bus_.checked_write(id, kRegTorqueEnable, &value, 1, kIoTimeoutMs).kind,
      ReplyKind::INVALID_ID) << static_cast<int>(id);
  }
  fake_.wait_quiet();
  EXPECT_EQ(fake_.bytes_received(), 0u);
  EXPECT_EQ(fake_.frames_received(), 0u);
}

TEST_F(ServoBusChecked, checked_calls_on_a_closed_bus_return_NOT_OPEN_without_aborting)
{
  // Refused before readSCS (see a_closed_bus_refuses_a_sync_read). The bus answers before and
  // after, so NOT_OPEN means closed.
  ASSERT_EQ(bus_.checked_ping(1).kind, ReplyKind::ONE);
  bus_.close();
  fake_.clear_frames();
  const uint8_t value = 1;
  EXPECT_EQ(bus_.checked_ping(1).kind, ReplyKind::NOT_OPEN);
  EXPECT_EQ(bus_.checked_read(1, kRegMode, 1).kind, ReplyKind::NOT_OPEN);
  EXPECT_EQ(
    bus_.checked_write(1, kRegTorqueEnable, &value, 1, kIoTimeoutMs).kind, ReplyKind::NOT_OPEN);
  ServoBus never_opened;
  EXPECT_EQ(never_opened.checked_ping(1).kind, ReplyKind::NOT_OPEN);
  EXPECT_EQ(fake_.bytes_received(), 0u);

  ASSERT_TRUE(static_cast<bool>(bus_.open(fake_.port(), kBaudrate, kIoTimeoutMs)));
  EXPECT_EQ(bus_.checked_ping(1).kind, ReplyKind::ONE);
}

TEST_F(ServoBusChecked, checked_read_returns_the_payload)
{
  // The tools' identity block: registers 3..39 in one READ, every byte distinct.
  std::vector<uint8_t> identity;
  for (uint8_t reg = 3; reg <= 39; reg++) {
    identity.push_back(static_cast<uint8_t>(0x40 + reg));
    fake_.set_byte(1, reg, identity.back());
  }
  fake_.set_status(1, 0x08);
  const Reply reply = bus_.checked_read(1, 3, 37);
  ASSERT_EQ(reply.kind, ReplyKind::ONE);
  EXPECT_TRUE(static_cast<bool>(reply));
  EXPECT_EQ(reply.from_id, 1);
  EXPECT_EQ(reply.status, 0x08);
  EXPECT_EQ(reply.frame_bytes, 43u);
  EXPECT_EQ(reply.bytes, 43u);
  EXPECT_EQ(reply.extra_bytes, 0u);
  EXPECT_EQ(reply.data, identity);

  fake_.set_position(1, 1234);
  const Reply position = bus_.checked_read(1, kRegPresentPosition, 2);
  ASSERT_EQ(position.kind, ReplyKind::ONE);
  EXPECT_EQ(position.data, (std::vector<uint8_t>{1234 & 0xff, 1234 >> 8}));
}

TEST_F(ServoBusChecked, checked_read_rejects_a_reply_from_another_id)
{
  // SCS::Read checks neither the reply's id nor its length byte; checked_read checks both.
  // See docs/design.md, "Checked transactions".
  fake_.set_faults_apply_to_addressed(true);
  fake_.set_reply_id_override(1, 7);
  uint8_t mode = 0xaa;
  EXPECT_EQ(bus_.Read(1, kRegMode, &mode, 1), 1) << "the vendored read believes it";

  const Reply reply = bus_.checked_read(1, kRegMode, 1);
  EXPECT_EQ(reply.kind, ReplyKind::WRONG_ID);
  EXPECT_FALSE(static_cast<bool>(reply));
  EXPECT_EQ(reply.from_id, 7);
  EXPECT_EQ(reply.frame_bytes, 7u);
  EXPECT_TRUE(reply.data.empty()) << "a payload from the wrong servo is never handed over";
}

TEST_F(ServoBusChecked, checked_read_of_a_bare_status_frame_is_STATUS_ONLY)
{
  // A late write ack caught by a read: right id, but a bare 6-byte status frame (STATUS_ONLY).
  fake_.set_eeprom_commit_ms(1, 30);
  ASSERT_TRUE(bus_.set_io_timeout_ms(60));
  const uint8_t offset = 7;
  EXPECT_EQ(bus_.checked_write(1, kRegOffset, &offset, 1, 5).kind, ReplyKind::SILENT);
  const Reply reply = bus_.checked_read(1, kRegPresentPosition, 2);
  EXPECT_EQ(reply.kind, ReplyKind::STATUS_ONLY);
  EXPECT_FALSE(static_cast<bool>(reply));
  EXPECT_EQ(reply.from_id, 1);
  EXPECT_EQ(reply.frame_bytes, 6u);
  EXPECT_EQ(reply.bytes, 6u);
  EXPECT_TRUE(reply.data.empty());
  EXPECT_EQ(bus_.checked_read(1, kRegPresentPosition, 2).kind, ReplyKind::ONE) <<
    "repeated after the commit, the read is answered";
}

TEST_F(ServoBusChecked, checked_read_rejects_a_bad_length_byte)
{
  // One more than the payload says, checksum repaired and the wire length unchanged: only the
  // length gate can catch it, and SCS::Read has none.
  fake_.set_faults_apply_to_addressed(true);
  fake_.set_reply_length_delta(1, 1);
  const Reply reply = bus_.checked_read(1, kRegMode, 1);
  EXPECT_EQ(reply.kind, ReplyKind::GARBLED);
  EXPECT_EQ(reply.bytes, 7u) << "every byte arrived; the length byte disagrees with them";
  EXPECT_EQ(reply.from_id, -1);
  EXPECT_TRUE(reply.data.empty());

  fake_.set_reply_length_delta(1, 0);
  const Reply clean = bus_.checked_read(1, kRegMode, 1);
  EXPECT_EQ(clean.kind, ReplyKind::ONE) << "nothing of the bad frame was left on the line";
  EXPECT_EQ(clean.data, (std::vector<uint8_t>{0}));
}

TEST_F(ServoBusChecked, checked_read_rejects_a_bad_checksum)
{
  fake_.set_faults_apply_to_addressed(true);
  fake_.set_reply_checksum_corrupt(1, true);
  const Reply reply = bus_.checked_read(1, kRegMode, 1);
  EXPECT_EQ(reply.kind, ReplyKind::GARBLED);
  EXPECT_EQ(reply.bytes, 7u);
  EXPECT_EQ(reply.from_id, -1);
  EXPECT_TRUE(reply.data.empty());

  fake_.set_reply_checksum_corrupt(1, false);
  const Reply clean = bus_.checked_read(1, kRegMode, 1);
  EXPECT_EQ(clean.kind, ReplyKind::ONE) << "nothing of the bad frame was left on the line";
  EXPECT_EQ(clean.data, (std::vector<uint8_t>{0}));
}

TEST_F(ServoBusChecked, checked_write_reports_the_acking_id_old_new_or_none)
{
  // Acks are advisory: checked_write takes a frame from any id as the ack and reports the id.
  fake_.add_servo(4);
  fake_.add_servo(5);
  fake_.add_servo(6);
  fake_.set_id_write_ack(5, IdWriteAck::new_id);
  fake_.set_id_write_ack(6, IdWriteAck::none);
  const uint8_t to_253 = 253;
  const uint8_t to_252 = 252;
  const uint8_t to_251 = 251;

  const Reply old_id = bus_.checked_write(4, kRegId, &to_253, 1, kIoTimeoutMs);
  EXPECT_EQ(old_id.kind, ReplyKind::ONE);
  EXPECT_EQ(old_id.from_id, 4);
  EXPECT_EQ(old_id.frame_bytes, 6u);
  const Reply new_id = bus_.checked_write(5, kRegId, &to_252, 1, kIoTimeoutMs);
  EXPECT_EQ(new_id.kind, ReplyKind::ONE) << "a frame from any id is the ack";
  EXPECT_EQ(new_id.from_id, 252);
  const Reply none = bus_.checked_write(6, kRegId, &to_251, 1, kIoTimeoutMs);
  EXPECT_EQ(none.kind, ReplyKind::SILENT);
  EXPECT_EQ(none.from_id, -1);
  // all three moved, which only a read-back can tell
  EXPECT_EQ(bus_.checked_ping(253).kind, ReplyKind::ONE);
  EXPECT_EQ(bus_.checked_ping(252).kind, ReplyKind::ONE);
  EXPECT_EQ(bus_.checked_ping(251).kind, ReplyKind::ONE);

  const uint8_t acc = 9;
  const Reply plain = bus_.checked_write(1, kRegAcc, &acc, 1, kIoTimeoutMs);
  EXPECT_EQ(plain.kind, ReplyKind::ONE);
  EXPECT_EQ(plain.from_id, 1);
  EXPECT_EQ(fake_.snapshot(1).mem[kRegAcc], 9);
}

TEST_F(ServoBusChecked, checked_write_restores_io_timeout_after_its_ack_window)
{
  // The ack window replaces the io timeout for one transaction only, longer or shorter.
  ASSERT_TRUE(bus_.set_io_timeout_ms(5));
  const uint8_t value = 1;
  const Reply write = bus_.checked_write(9, kRegTorqueEnable, &value, 1, 40);
  EXPECT_EQ(write.kind, ReplyKind::SILENT);
  EXPECT_GE(write.elapsed_us, 40000u) << "the ack window, not the io timeout";
  EXPECT_EQ(bus_.io_timeout_ms(), 5u);
  const Reply ping = bus_.checked_ping(9);
  EXPECT_EQ(ping.kind, ReplyKind::SILENT);
  EXPECT_LT(ping.elapsed_us, write.elapsed_us / 2) << "the next transaction is back on 5 ms";

  ASSERT_TRUE(bus_.set_io_timeout_ms(40));
  const Reply quick = bus_.checked_write(9, kRegTorqueEnable, &value, 1, 5);
  EXPECT_EQ(quick.kind, ReplyKind::SILENT);
  EXPECT_LT(quick.elapsed_us, write.elapsed_us / 2) << "a shorter window is honoured too";
  EXPECT_EQ(bus_.io_timeout_ms(), 40u);
}

TEST_F(ServoBusChecked, a_late_ack_is_drained_and_not_read_as_the_next_reply)
{
  // A late ack on the line must be drained, not taken as the next transaction's reply.
  fake_.set_eeprom_commit_ms(1, 30);
  const uint8_t offset = 7;
  EXPECT_EQ(bus_.checked_write(1, kRegOffset, &offset, 1, 5).kind, ReplyKind::SILENT);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));   // the ack is on the line now

  const Reply silent = bus_.checked_ping(9);
  EXPECT_EQ(silent.kind, ReplyKind::SILENT) << "the stale ack was read as id 9's reply";
  EXPECT_EQ(silent.bytes, 0u);
  const Reply read = bus_.checked_read(1, kRegOffset, 1);
  EXPECT_EQ(read.kind, ReplyKind::ONE);
  EXPECT_EQ(read.data, (std::vector<uint8_t>{7}));
  EXPECT_EQ(read.extra_bytes, 0u);
}

TEST_F(ServoBusChecked, checked_calls_leave_the_sync_read_drain_count_alone)
{
  // drains is the control loop's error-path count. The checked calls drain after every reply, so
  // counting those would let one scan swamp it.
  fake_.set_twin(1, TwinReply::doubled);
  const uint64_t before = bus_.sync_read_stats().drains;
  const Reply reply = bus_.checked_ping(1);
  EXPECT_EQ(reply.kind, ReplyKind::EXTRA);
  EXPECT_EQ(reply.extra_bytes, 6u) << "the drain ran";
  EXPECT_EQ(bus_.sync_read_stats().drains, before);
  EXPECT_EQ(bus_.drain_input(), 0u) << "and left nothing behind";
  EXPECT_EQ(bus_.sync_read_stats().drains, before + 1) << "drain_input still counts";
}

TEST_F(ServoBusChecked, checked_calls_refuse_a_count_outside_1_to_64_without_sending)
{
  // Counts outside 1..64 are refused: a large count overruns txBuf, and a READ of 0 is answered
  // by a bare status frame.
  const std::array<uint8_t, 255> bytes{};
  const uint8_t too_many = static_cast<uint8_t>(ServoBus::checked_max_bytes + 1);
  EXPECT_EQ(bus_.checked_read(1, 3, 0).kind, ReplyKind::INVALID_COUNT);
  EXPECT_EQ(bus_.checked_read(1, 3, too_many).kind, ReplyKind::INVALID_COUNT);
  EXPECT_EQ(
    bus_.checked_write(1, 3, bytes.data(), 0, kIoTimeoutMs).kind, ReplyKind::INVALID_COUNT);
  EXPECT_EQ(
    bus_.checked_write(1, 3, bytes.data(), 255, kIoTimeoutMs).kind, ReplyKind::INVALID_COUNT);
  fake_.wait_quiet();
  EXPECT_EQ(fake_.bytes_received(), 0u);

  const Reply most = bus_.checked_read(
    1, 3, static_cast<uint8_t>(ServoBus::checked_max_bytes));
  EXPECT_EQ(most.kind, ReplyKind::ONE) << "the largest legal count";
  EXPECT_EQ(most.data.size(), ServoBus::checked_max_bytes);
}

// ---- factory_reset: checked_reset and set_baudrate ----

TEST_F(ServoBusChecked, checked_reset_sends_the_bench_frame_and_hears_the_ack)
{
  // FF FF 04 02 06 F3: the RESET frame used on the bench (six bytes, no parameter).
  fake_.add_servo(4, 1);
  fake_.clear_frames();
  const Reply reply = bus_.checked_reset(4, 100);
  EXPECT_EQ(reply.kind, ReplyKind::ONE);
  EXPECT_EQ(reply.from_id, 4);
  EXPECT_EQ(reply.frame_bytes, 6u);
  EXPECT_TRUE(reply.data.empty());
  fake_.wait_quiet();
  EXPECT_EQ(fake_.bytes_received(), 6u);
  const std::vector<FrameRecord> frames = fake_.frames();
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].id, 4);
  EXPECT_EQ(frames[0].instruction, ServoBus::inst_reset);
  EXPECT_TRUE(frames[0].params.empty());
  EXPECT_EQ(fake_.snapshot(4).mem[kRegMode], 0) << "the servo was reset";
  EXPECT_EQ(fake_.snapshot(4).mem[kRegId], 4);
}

TEST_F(ServoBusChecked, checked_reset_refuses_0xfe_0xff_and_a_closed_bus_without_sending)
{
  // A broadcast RESET would reset every servo on the bus; the checked call hears one servo only.
  EXPECT_EQ(bus_.checked_reset(0xfe, 100).kind, ReplyKind::INVALID_ID);
  EXPECT_EQ(bus_.checked_reset(0xff, 100).kind, ReplyKind::INVALID_ID);
  bus_.close();
  EXPECT_EQ(bus_.checked_reset(1, 100).kind, ReplyKind::NOT_OPEN);
  fake_.wait_quiet();
  EXPECT_EQ(fake_.bytes_received(), 0u);
  EXPECT_EQ(fake_.snapshot(1).resets, 0);
}

TEST_F(ServoBusChecked, checked_reset_names_a_silent_id_a_foreign_ack_and_a_doubled_one)
{
  fake_.add_servo(4);
  fake_.add_servo(5);
  const Reply silent = bus_.checked_reset(9, 30);
  EXPECT_EQ(silent.kind, ReplyKind::SILENT);
  EXPECT_GE(silent.elapsed_us, 30000u) << "the ack window, not the io timeout";
  EXPECT_EQ(bus_.io_timeout_ms(), kIoTimeoutMs) << "and the io timeout is given back";

  fake_.set_faults_apply_to_addressed(true);
  fake_.set_reply_id_override(4, 9);
  const Reply foreign = bus_.checked_reset(4, 100);
  EXPECT_EQ(foreign.kind, ReplyKind::WRONG_ID) << "only the addressed id acks a reset";
  EXPECT_EQ(foreign.from_id, 9);
  fake_.set_faults_apply_to_addressed(false);

  fake_.set_twin(5, TwinReply::doubled);
  const Reply doubled = bus_.checked_reset(5, 100);
  EXPECT_EQ(doubled.kind, ReplyKind::EXTRA);
  EXPECT_EQ(doubled.extra_bytes, 6u);
}

TEST_F(ServoBusChecked, set_baudrate_retimes_the_line_and_keeps_the_port)
{
  // Under the baud model a servo at register 6 = 1 answers at 500000 only. The bus moves to it and
  // back without ever letting go of the port.
  fake_.add_servo(4);
  fake_.set_byte(4, 6, 1);
  fake_.set_baud_model(true);
  ASSERT_EQ(bus_.checked_ping(4).kind, ReplyKind::SILENT);

  ASSERT_TRUE(bus_.set_baudrate(500000));
  EXPECT_EQ(bus_.baudrate(), 500000);
  struct termios settings{};
  ASSERT_EQ(::tcgetattr(fake_.slave_fd(), &settings), 0);
  EXPECT_EQ(::cfgetospeed(&settings), static_cast<speed_t>(B500000));
  EXPECT_EQ(::cfgetispeed(&settings), static_cast<speed_t>(B500000));
  EXPECT_EQ(bus_.checked_ping(4).kind, ReplyKind::ONE);
  EXPECT_EQ(bus_.port(), fake_.port());

  // TIOCEXCL is still set, so a second open is refused before it even reaches the flock.
  ServoBus other;
  const OpenResult held = other.open(fake_.port(), kBaudrate, kIoTimeoutMs);
  EXPECT_EQ(held.status, BusStatus::LOCK_OPEN_FAILED) << "the port is still held";
  EXPECT_EQ(held.error, EBUSY);

  ASSERT_TRUE(bus_.set_baudrate(kBaudrate));
  EXPECT_EQ(bus_.baudrate(), kBaudrate);
  EXPECT_EQ(bus_.checked_ping(4).kind, ReplyKind::SILENT);
  EXPECT_EQ(bus_.checked_ping(1).kind, ReplyKind::ONE) << "servo 1 is at register 6 = 0";
}

TEST_F(ServoBusChecked, set_baudrate_refuses_a_closed_bus_and_an_unmapped_rate)
{
  for (const int rate : {0, 250000, 128000, 76800, 230400, -1}) {
    EXPECT_FALSE(bus_.set_baudrate(rate)) << rate;
    EXPECT_EQ(bus_.baudrate(), kBaudrate) << rate;
  }
  struct termios settings{};
  ASSERT_EQ(::tcgetattr(fake_.slave_fd(), &settings), 0);
  EXPECT_EQ(::cfgetospeed(&settings), static_cast<speed_t>(B1000000));
  bus_.close();
  EXPECT_FALSE(bus_.set_baudrate(500000));
  ServoBus never_opened;
  EXPECT_FALSE(never_opened.set_baudrate(kBaudrate));
}

TEST_F(ServoBusChecked, every_reply_kind_has_a_name)
{
  // The BusStatus and WriteStatus discipline, plus one rule of its own: a name goes into the
  // tools' key=value detail lines, so it may not contain a space.
  const std::vector<ReplyKind> all = {
    ReplyKind::NOT_OPEN, ReplyKind::INVALID_ID, ReplyKind::INVALID_COUNT, ReplyKind::SILENT,
    ReplyKind::ONE, ReplyKind::EXTRA, ReplyKind::WRONG_ID, ReplyKind::STATUS_ONLY,
    ReplyKind::GARBLED};
  std::vector<std::string> names;
  for (const ReplyKind kind : all) {
    const std::string name = to_string(kind);
    EXPECT_FALSE(name.empty());
    EXPECT_EQ(name.find(' '), std::string::npos) << name;
    names.push_back(name);
  }
  std::sort(names.begin(), names.end());
  EXPECT_EQ(std::unique(names.begin(), names.end()), names.end()) << "two kinds share a name";
}
