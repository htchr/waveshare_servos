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

#include "servo_bus.hpp"

namespace
{

// Per-name declarations, never a using-directive: cpplint's build/namespaces rule forbids
// using-directives outside a short std::*_literals whitelist, in sources as well as headers.
using waveshare_servos::BusStatus;
using waveshare_servos::OpenResult;
using waveshare_servos::ServoBus;
using waveshare_servos::port_holder_pids;
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

  ServoBus bus_;
  int master_ = -1;
  std::string port_;
  // declared last, so it is destroyed (stopped and joined) before master_ is touched
  std::optional<FakeResponder> responder_;
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
  EXPECT_TRUE(bus_.set_io_timeout_ms(5));
  EXPECT_EQ(bus_.io_timeout_ms(), 5u);
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
