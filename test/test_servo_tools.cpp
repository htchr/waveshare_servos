// Tests for src/servo_tools.{hpp,cpp} -- the Phase 6 tools' bus logic (PHASE6_SPEC B.3, C, D.4).
//
// Every case drives the library in-process against the fake bus of test/fake_servo_bus.hpp: the
// vendored packet code runs unchanged over an openpty() pair, and the fake's frame log is the
// witness. That log, not anything the tools report about themselves, is what the gates read:
//   - a scan's coverage is the exact ping-count map, compared as a whole;
//   - "wrote nothing else" is the whole ordered list of WRITE frames;
//   - "sent nothing" is the raw byte count, which sees even a frame the fake could not parse.
// The fake's EEPROM model runs under volatile_when_locked, the memory table's reading and the
// strictest policy: a write made while register 55 reads 1 applies and is lost at power-off, so
// only a verified unlock makes an id or an offset survive power_cycle().

#include <gmock/gmock.h>

#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <sys/file.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <streambuf>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "driver_defaults.hpp"
#include "fake_servo_bus.hpp"
#include "servo_bus.hpp"
#include "servo_tools.hpp"

namespace
{

// Per-name declarations, never a using-directive (cpplint build/namespaces).
using ::testing::ContainsRegex;
using ::testing::ElementsAre;
using ::testing::HasSubstr;
using ::testing::MatchesRegex;
using ::testing::Not;
using ::testing::StartsWith;
using waveshare_servos::BusStatus;
using waveshare_servos::OpenResult;
using waveshare_servos::ServoBus;
using waveshare_servos::tools::CalibrateReport;
using waveshare_servos::tools::Exit;
using waveshare_servos::tools::FactoryResetReport;
using waveshare_servos::tools::ScanResult;
using waveshare_servos::tools::Session;
using waveshare_servos::tools::SetIdReport;
using waveshare_servos::tools::calibrate_midpoint;
using waveshare_servos::tools::detail_line;
using waveshare_servos::tools::factory_reset;
using waveshare_servos::tools::final_exit;
using waveshare_servos::tools::kFactoryBaudrate;
using waveshare_servos::tools::kScanFirstId;
using waveshare_servos::tools::kScanLastId;
using waveshare_servos::tools::holders_text;
using waveshare_servos::tools::io_timeout_ms_for;
using waveshare_servos::tools::open_bus;
using waveshare_servos::tools::open_failure;
using waveshare_servos::tools::print_scan;
using waveshare_servos::tools::scan;
using waveshare_servos::tools::scan_exit;
using waveshare_servos::tools::set_id;
using waveshare_servos_test::EepromPolicy;
using waveshare_servos_test::FakeBus;
using waveshare_servos_test::FakeServo;
using waveshare_servos_test::IdWriteAck;
using waveshare_servos_test::FrameRecord;
using waveshare_servos_test::WriteRecord;
using waveshare_servos_test::kBroadcastId;
using waveshare_servos_test::kInstPing;
using waveshare_servos_test::kInstRead;
using waveshare_servos_test::TwinReply;

constexpr int kBaudrate = 1000000;
constexpr uint32_t kIoTimeoutMs = 5;    // io_timeout_ms_for(1000000), the tools' own (C.0)

}  // namespace

namespace waveshare_servos
{
namespace tools
{

// gtest prints an enum it has no printer for as "4-byte object <46-00 00-00>"; with this a failed
// exit reads "internal (70)". Found by argument-dependent lookup, hence this namespace.
void PrintTo(Exit exit, std::ostream * out)
{
  *out << exit_name(exit) << " (" << static_cast<int>(exit) << ")";
}

}  // namespace tools
}  // namespace waveshare_servos

namespace
{

std::string read_file(const std::string & path)
{
  std::ifstream in(path);
  std::stringstream text;
  text << in.rdbuf();
  return text.str();
}

std::size_t count_of(const std::string & text, const std::string & needle)
{
  std::size_t count = 0;
  for (std::size_t at = text.find(needle); at != std::string::npos;
    at = text.find(needle, at + 1))
  {
    count++;
  }
  return count;
}

// A path in the temp directory, unique to this process, removed however the case ends.
class TempPath
{
public:
  explicit TempPath(const std::string & stem)
  : path_((std::filesystem::temp_directory_path() /
      (stem + "_" + std::to_string(::getpid()))).string())
  {
    std::error_code ignored;
    std::filesystem::remove(path_, ignored);
  }

  ~TempPath()
  {
    std::error_code ignored;
    std::filesystem::remove(path_, ignored);
  }

  TempPath(const TempPath &) = delete;
  TempPath & operator=(const TempPath &) = delete;
  TempPath(TempPath &&) = delete;
  TempPath & operator=(TempPath &&) = delete;

  const std::string & path() const {return path_;}

private:
  std::string path_;
};

// Descriptors 1 and 2 of the whole process pointed at two files, for the length of one call: the
// vendored begin() printf()s "serial speed N" through C stdio, which no std::ostream argument can
// catch. finish() puts both back and reads what arrived; call it before any EXPECT, whose output
// would otherwise land in the files.
class CapturedStdio
{
public:
  CapturedStdio()
  : out_path_("test_servo_tools_stdout"), err_path_("test_servo_tools_stderr")
  {
    std::fflush(stdout);
    std::fflush(stderr);
    saved_out_ = ::dup(STDOUT_FILENO);
    saved_err_ = ::dup(STDERR_FILENO);
    redirect(out_path_.path(), STDOUT_FILENO);
    redirect(err_path_.path(), STDERR_FILENO);
  }

  ~CapturedStdio() {finish();}

  CapturedStdio(const CapturedStdio &) = delete;
  CapturedStdio & operator=(const CapturedStdio &) = delete;
  CapturedStdio(CapturedStdio &&) = delete;
  CapturedStdio & operator=(CapturedStdio &&) = delete;

  void finish()
  {
    if (saved_out_ == -1 && saved_err_ == -1) {
      return;
    }
    std::fflush(stdout);
    std::fflush(stderr);
    restore(&saved_out_, STDOUT_FILENO);
    restore(&saved_err_, STDERR_FILENO);
    out_ = read_file(out_path_.path());
    err_ = read_file(err_path_.path());
  }

  const std::string & out() const {return out_;}
  const std::string & err() const {return err_;}

private:
  static void redirect(const std::string & path, int target)
  {
    const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
    if (fd != -1) {
      ::dup2(fd, target);
      ::close(fd);
    }
  }

  static void restore(int * saved, int target)
  {
    if (*saved != -1) {
      ::dup2(*saved, target);
      ::close(*saved);
      *saved = -1;
    }
  }

  TempPath out_path_;
  TempPath err_path_;
  int saved_out_ = -1;
  int saved_err_ = -1;
  std::string out_;
  std::string err_;
};

// Another PROCESS holding the port by its advisory lock alone, with no TIOCEXCL: the stimulus of
// the bench's flock-only holder (port_probe --no-exclusive, E.4), and the only holder whose pid a
// refusal can name, since open_bus leaves this process's own pid out. The child is forked from a
// threaded process, so it calls nothing but async-signal-safe functions until it _exit()s.
class ChildHolder
{
public:
  explicit ChildHolder(const std::string & port)
  {
    int ready[2] = {-1, -1};
    int release[2] = {-1, -1};
    if (::pipe(ready) != 0 || ::pipe(release) != 0) {
      return;
    }
    pid_ = ::fork();
    if (pid_ == 0) {
      ::close(ready[0]);
      ::close(release[1]);
      const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
      const char held = (fd != -1 && ::flock(fd, LOCK_EX | LOCK_NB) == 0) ? '1' : '0';
      if (::write(ready[1], &held, 1) != 1) {
        ::_exit(1);
      }
      char ignored = 0;
      while (::read(release[0], &ignored, 1) > 0) {
      }
      ::_exit(0);
    }
    ::close(ready[1]);
    ::close(release[0]);
    release_ = release[1];
    char held = '0';
    if (pid_ > 0 && ::read(ready[0], &held, 1) == 1) {
      held_ = held == '1';
    }
    ::close(ready[0]);
  }

  ~ChildHolder()
  {
    if (release_ != -1) {
      ::close(release_);            // EOF: the child exits and its lock goes with it
    }
    if (pid_ > 0) {
      ::waitpid(pid_, nullptr, 0);
    }
  }

  ChildHolder(const ChildHolder &) = delete;
  ChildHolder & operator=(const ChildHolder &) = delete;
  ChildHolder(ChildHolder &&) = delete;
  ChildHolder & operator=(ChildHolder &&) = delete;

  bool held() const {return held_;}
  pid_t pid() const {return pid_;}

private:
  pid_t pid_ = -1;
  int release_ = -1;
  bool held_ = false;
};

// The shared fixture: a fake bus under the strictest EEPROM policy, a ServoBus opened on it at the
// tools' own 5 ms, and a Session over them with a local stop flag and string streams.
class ToolFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    fake_.set_eeprom_policy(EepromPolicy::volatile_when_locked);
    if (opens_bus()) {
      ASSERT_TRUE(static_cast<bool>(bus_.open(fake_.port(), kBaudrate, kIoTimeoutMs)));
    }
  }

  void TearDown() override
  {
    bus_.close();
    fake_.wait_quiet();
    // A request the fake could not verify means the wire itself misbehaved and the case proved
    // nothing; a collision means a tool put two servos on one id.
    EXPECT_EQ(fake_.bad_checksums(), 0u) << "the wire itself misbehaved";
    EXPECT_EQ(fake_.quiet_timeouts(), 0u) << "wait_quiet gave up";
    EXPECT_EQ(fake_.id_collisions(), 0u) << "two servos were put on one id";
    for (const FrameRecord & frame : fake_.frames()) {
      EXPECT_NE(frame.id, kBroadcastId) << "no tool ever broadcasts";
    }
    // The report's count of writes against the wire's: a write the tool did not count, or one it
    // counted and never sent, both show here. A RESET is counted as a write (factory_reset).
    if (writes_sent_.has_value()) {
      EXPECT_EQ(*writes_sent_, fake_.writes().size() + fake_.resets().size()) <<
        "writes_sent disagrees with the wire";
    }
  }

  virtual bool opens_bus() const {return true;}

  // The servo keyed at `id`, or nullopt. snapshot() throws for an id nobody is keyed at, and a
  // case that expects a move (or no move) must fail on an assertion when that did not happen.
  std::optional<FakeServo> servo_at(uint8_t id) const
  {
    try {
      return fake_.snapshot(id);
    } catch (const std::out_of_range &) {
      return std::nullopt;
    }
  }

  // A servo as the bench delivers it (E.0): firmware 3.6, model word 9 3, baud register 0 (1 M),
  // response level 1, angle limits 0 and 4095, 12.2 V, 30 C, torque on and the EEPROM lock closed
  // (register 55 reads 1 on all four bench servos) -- plus an offset and a position of its own,
  // so a value read from the wrong servo cannot pass for the right one.
  void seed_bench_like(uint8_t id, uint8_t mode)
  {
    fake_.add_servo(id, mode);
    fake_.set_byte(id, 0, 3);
    fake_.set_byte(id, 1, 6);
    fake_.set_byte(id, 3, 9);
    fake_.set_byte(id, 4, 3);
    fake_.set_byte(id, 6, 0);
    fake_.set_byte(id, 8, 1);
    fake_.set_word(id, 9, 0);
    fake_.set_word(id, 11, 4095);
    fake_.set_byte(id, 62, 122);
    fake_.set_byte(id, 63, 30);
    fake_.set_word(id, 31, static_cast<int>(id % 7) + 1, 11);
    fake_.set_position(id, 1000 + 13 * id);
    fake_.set_byte(id, 40, 1);
    fake_.set_byte(id, 55, 1);
  }

  FakeBus fake_;
  ServoBus bus_;                        // declared after fake_, so it is destroyed first
  volatile std::sig_atomic_t stop_ = 0;
  std::ostringstream out_;
  std::ostringstream err_;
  Session session_{bus_, waveshare_servos::defaults::kPingAttempts, kIoTimeoutMs, &stop_, out_,
    err_};
  std::optional<std::size_t> writes_sent_;   // set by the set_id, calibrate and reset cases
};

// scan's stdout contract (C.1), the header the HIL parser matches exactly.
constexpr const char * kHeader =
  " id  type  mode  model  baud_reg     baud  position  voltage_V  temp_C  status  offset";

std::vector<std::string> lines_of(const std::string & text)
{
  std::vector<std::string> lines;
  std::istringstream in(text);
  for (std::string line; std::getline(in, line); ) {
    lines.push_back(line);
  }
  return lines;
}

std::vector<std::string> tokens_of(const std::string & line)
{
  std::vector<std::string> tokens;
  std::istringstream in(line);
  for (std::string token; in >> token; ) {
    tokens.push_back(token);
  }
  return tokens;
}

// What an empty bus sees from a full scan: every id 0..253 pinged `attempts` times [Q5].
std::map<uint8_t, int> empty_bus_pings()
{
  std::map<uint8_t, int> pings;
  for (int id = kScanFirstId; id <= kScanLastId; id++) {
    pings[static_cast<uint8_t>(id)] = waveshare_servos::defaults::kPingAttempts;
  }
  return pings;
}

// Every case scans the whole range the executable scans [Q5]: a narrowed range in a test would
// hide a narrowed range in the tool.
class ToolScan : public ToolFixture
{
protected:
  ScanResult run_scan()
  {
    const ScanResult result = scan(session_, kScanFirstId, kScanLastId);
    print_scan(result, fake_.port(), kBaudrate, out_, err_);
    fake_.wait_quiet();
    return result;
  }

  // The stdout row of `id`, as tokens; empty when there is none.
  std::vector<std::string> row(int id) const
  {
    for (const std::string & line : lines_of(out_.str())) {
      const std::vector<std::string> tokens = tokens_of(line);
      if (!tokens.empty() && tokens.front() == std::to_string(id)) {
        return tokens;
      }
    }
    return {};
  }

  // READ frames to `id` starting at `address`, from the fake's log.
  int reads_of(uint8_t id, uint8_t address) const
  {
    int count = 0;
    for (const FrameRecord & frame : fake_.frames()) {
      count += frame.instruction == kInstRead && frame.id == id && !frame.params.empty() &&
        frame.params[0] == address;
    }
    return count;
  }

  std::string footer_pattern(const std::string & found) const
  {
    return "found " + found + " on " + fake_.port() + " at 1000000 baud.*\\(pinged ids 0\\.\\.253, "
           "3 attempts each, [0-9]+\\.[0-9] s\\)";
  }
};

// A stream buffer another thread may read while the tool writes: how a case watches `err` DURING a
// sequence rather than after it (nothing_is_printed_during_the_sequence).
class LockedBuffer : public std::streambuf
{
public:
  std::string text() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return text_;
  }

  std::size_t size() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return text_.size();
  }

protected:
  int_type overflow(int_type c) override
  {
    if (!traits_type::eq_int_type(c, traits_type::eof())) {
      const std::lock_guard<std::mutex> lock(mutex_);
      text_.push_back(traits_type::to_char_type(c));
    }
    return c;
  }

  std::streamsize xsputn(const char * data, std::streamsize count) override
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    text_.append(data, static_cast<std::size_t>(count));
    return count;
  }

private:
  mutable std::mutex mutex_;
  std::string text_;
};

// Runs `action` on another thread once the fake has logged `writes` WRITE frames: how a case
// reaches into the middle of a sequence (a signal, a knob) without sleeping on a guess.
class AfterWrites
{
public:
  template<typename Action>
  AfterWrites(const FakeBus & fake, std::size_t writes, Action action)
  : thread_([&fake, writes, action] {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (fake.writes().size() < writes && std::chrono::steady_clock::now() < deadline) {
          std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
        action();
      })
  {
  }

  ~AfterWrites() {join();}

  AfterWrites(const AfterWrites &) = delete;
  AfterWrites & operator=(const AfterWrites &) = delete;
  AfterWrites(AfterWrites &&) = delete;
  AfterWrites & operator=(AfterWrites &&) = delete;

  void join()
  {
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  std::thread thread_;
};

// Servo 4, a wheel like the bench's, is the one renumbered unless a case says otherwise.
class ToolSetId : public ToolFixture
{
protected:
  void SetUp() override
  {
    ToolFixture::SetUp();
    seed_bench_like(4, 1);
  }

  SetIdReport run(uint8_t start_id, uint8_t new_id)
  {
    const SetIdReport report = set_id(session_, start_id, new_id);
    writes_sent_ = report.writes_sent;
    fake_.wait_quiet();
    return report;
  }

  // The whole write list of a clean move: the verified unlock of S, the id, the verified lock of N.
  static std::vector<WriteRecord> clean_move(uint8_t start_id, uint8_t new_id)
  {
    return {WriteRecord{start_id, 55, {0}}, WriteRecord{start_id, 5, {new_id}},
      WriteRecord{new_id, 55, {1}}};
  }

  // Every frame to `id` in the fake's log.
  std::size_t frames_to(uint8_t id) const
  {
    std::size_t count = 0;
    for (const FrameRecord & frame : fake_.frames()) {
      count += frame.id == id;
    }
    return count;
  }
};

class ToolSetIdPolicy : public ToolSetId, public ::testing::WithParamInterface<EepromPolicy> {};
class ToolSetIdAck : public ToolSetId, public ::testing::WithParamInterface<IdWriteAck> {};
class ToolSetIdSlowCommit : public ToolSetId, public ::testing::WithParamInterface<IdWriteAck> {};

std::string policy_name(const ::testing::TestParamInfo<EepromPolicy> & info)
{
  switch (info.param) {
    case EepromPolicy::apply_always:
      return "apply_always";
    case EepromPolicy::drop_when_locked:
      return "drop_when_locked";
    case EepromPolicy::volatile_when_locked:
      return "volatile_when_locked";
  }
  return "unknown";
}

std::string ack_name(const ::testing::TestParamInfo<IdWriteAck> & info)
{
  switch (info.param) {
    case IdWriteAck::old_id:
      return "old_id";
    case IdWriteAck::new_id:
      return "new_id";
    case IdWriteAck::none:
      return "none";
  }
  return "unknown";
}

// Servo 2 in mode 0 at 1026 with the offset model on, torque on and the lock closed (D.4; the
// bench's id 2 rests at 1026), under volatile_when_locked.
class ToolCalibrate : public ToolFixture
{
protected:
  void SetUp() override
  {
    ToolFixture::SetUp();
    seed_bench_like(2, 0);
    fake_.set_offset_model(2, true);
    fake_.set_position(2, 1026);
  }

  CalibrateReport run(uint8_t id)
  {
    const CalibrateReport report = calibrate_midpoint(session_, id);
    writes_sent_ = report.writes_sent;
    fake_.wait_quiet();
    return report;
  }

  // The whole write list of a calibration that switched the torque off: torque, verified unlock,
  // 128, verified lock [Q1, Q2, Q3].
  static std::vector<WriteRecord> clean_calibration()
  {
    return {WriteRecord{2, 40, {0}}, WriteRecord{2, 55, {0}}, WriteRecord{2, 40, {128}},
      WriteRecord{2, 55, {1}}};
  }
};

// Servo 4 as the bench had it for the reset (FACTORY_RESET_SPEC M1, M2): a wheel with an offset and
// a return delay that are not factory, torque on and the lock closed. Its factory table is its
// EEPROM with those three put back, and the offset model is on, so a reset that clears the offset
// moves the present position the way the servo's would (1052 + 5).
class ToolFactoryReset : public ToolFixture
{
protected:
  void SetUp() override
  {
    ToolFixture::SetUp();
    seed_bench_like(4, 1);
    fake_.set_factory_from_eeprom(4, {{33, 0}, {7, 0}, {31, 0}, {32, 0}});
    fake_.set_byte(4, 7, 5);
    fake_.set_offset_model(4, true);
  }

  FactoryResetReport run(uint8_t id)
  {
    const FactoryResetReport report = factory_reset(session_, id);
    writes_sent_ = report.writes_sent;
    fake_.wait_quiet();
    return report;
  }

  // Servo 4 at register 6 = 1 under the baud model, on a bus opened at 500000: the servo a reset
  // moves to the factory rate (M3).
  void at_500000()
  {
    fake_.set_byte(4, 6, 1);
    fake_.set_baud_model(true);
    bus_.close();
    ASSERT_TRUE(static_cast<bool>(bus_.open(fake_.port(), 500000, kIoTimeoutMs)));
  }

  static constexpr const char * kNotice =
    "about to reset servo 4 to its factory settings: every EEPROM register but its id -- baud "
    "rate, offset (the midpoint calibration), mode, angle limits and gains. If this run is "
    "interrupted or fails, run scan: the servo will answer at id 4";
};

// open_bus is about taking the port, so the fixture leaves it closed.
class ToolOpen : public ToolFixture
{
protected:
  bool opens_bus() const override {return false;}
};

}  // namespace

// ---- ToolOpen ----

TEST_F(ToolOpen, a_port_held_by_another_bus_exits_1_and_sends_nothing)
{
  ServoBus other;
  ASSERT_TRUE(static_cast<bool>(other.open(fake_.port(), kBaudrate, kIoTimeoutMs)));
  fake_.clear_frames();
  EXPECT_EQ(open_bus(bus_, fake_.port(), kBaudrate, err_), Exit::kPortHeld);
  EXPECT_FALSE(bus_.is_open());
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "port '" + fake_.port() + "' is held by another process"));
  EXPECT_THAT(err_.str(), HasSubstr("Nothing was sent to the servos."));
  fake_.wait_quiet();
  EXPECT_EQ(fake_.bytes_received(), 0u);
}

TEST_F(ToolOpen, holders_text_lists_this_pid_when_self_pid_is_not_this_process)
{
  // open_bus always passes getpid(), which leaves this process out; so the pid itself is tested
  // here, with the self pid set to one that matches nobody.
  ServoBus other;
  ASSERT_TRUE(static_cast<bool>(other.open(fake_.port(), kBaudrate, kIoTimeoutMs)));
  const std::string mine = "pid " + std::to_string(::getpid());
  EXPECT_THAT(holders_text(fake_.port(), -1), HasSubstr(mine));
  EXPECT_THAT(holders_text(fake_.port(), -1), StartsWith(" ("));
  EXPECT_THAT(holders_text(fake_.port(), ::getpid()), Not(HasSubstr(mine)));
  EXPECT_EQ(
    holders_text(fake_.port(), ::getpid()),
    " (no holder visible in /proc; it may belong to another user)");
}

TEST_F(ToolOpen, a_flock_only_holder_exits_1_before_begin)
{
  // The discriminating case of item 4 (G.3): a tool that opened the port with a raw
  // SMS_STS::begin would get past a lock-only holder and print "serial speed". open_bus is refused
  // at the lock, before begin(), so the line appears on neither stream.
  const ChildHolder holder(fake_.port());
  ASSERT_TRUE(holder.held());
  fake_.clear_frames();
  CapturedStdio captured;
  const Exit exit = open_bus(bus_, fake_.port(), kBaudrate, err_);
  captured.finish();
  EXPECT_EQ(exit, Exit::kPortHeld);
  EXPECT_FALSE(bus_.is_open());
  EXPECT_THAT(err_.str(), HasSubstr("is held by another process"));
  EXPECT_THAT(err_.str(), HasSubstr("pid " + std::to_string(holder.pid())));
  EXPECT_EQ(count_of(captured.out(), "serial speed"), 0u) << captured.out();
  EXPECT_EQ(count_of(captured.err(), "serial speed"), 0u) << captured.err();
  EXPECT_EQ(count_of(err_.str(), "serial speed"), 0u);
  fake_.wait_quiet();
  EXPECT_EQ(fake_.bytes_received(), 0u);
}

TEST_F(ToolOpen, a_missing_port_exits_2_with_the_ls_hint)
{
  const TempPath missing("test_servo_tools_no_such_port");
  EXPECT_EQ(open_bus(bus_, missing.path(), kBaudrate, err_), Exit::kCannotOpen);
  EXPECT_FALSE(bus_.is_open());
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "port '" + missing.path() + "' does not exist; 'ls /dev/ttyACM* /dev/ttyUSB*' lists what "
      "is plugged in"));
}

TEST_F(ToolOpen, a_regular_file_is_not_a_tty)
{
  const TempPath file("test_servo_tools_regular_file");
  std::ofstream(file.path()) << "not a tty\n";
  EXPECT_EQ(open_bus(bus_, file.path(), kBaudrate, err_), Exit::kCannotOpen);
  EXPECT_FALSE(bus_.is_open());
  EXPECT_THAT(err_.str(), HasSubstr("'" + file.path() + "' is not a serial device"));
}

TEST_F(ToolOpen, holders_are_found_through_a_symlink)
{
  // /proc/<pid>/fd links name the real device, so a /dev/serial/by-id path finds its holders only
  // once it is canonicalised. Without that the message would say no holder is visible.
  const TempPath link("test_servo_tools_by_id_link");
  std::filesystem::create_symlink(fake_.port(), link.path());
  const ChildHolder holder(fake_.port());
  ASSERT_TRUE(holder.held());
  EXPECT_EQ(open_bus(bus_, link.path(), kBaudrate, err_), Exit::kPortHeld);
  EXPECT_THAT(err_.str(), HasSubstr("port '" + link.path() + "' is held by another process"));
  EXPECT_THAT(err_.str(), HasSubstr("pid " + std::to_string(holder.pid())));
  EXPECT_THAT(err_.str(), Not(HasSubstr("no holder visible")));
}

TEST_F(ToolOpen, every_bus_status_maps_to_one_exit_and_a_non_empty_message)
{
  // C.0 step 3's table, for every status the bus can report and the errnos that split them.
  const std::string port = "/dev/ttyUSB7";
  const std::string holders = " (pid 4321 ros2_control_no)";
  struct Row
  {
    BusStatus status;
    int error;
    Exit exit;
    const char * says;
  };
  const std::vector<Row> rows = {
    {BusStatus::LOCK_OPEN_FAILED, EBUSY, Exit::kPortHeld, "is held by another process (pid 4321"},
    {BusStatus::LOCK_FAILED, EWOULDBLOCK, Exit::kPortHeld, "is held by another process (pid 4321"},
    {BusStatus::LOCK_FAILED, 0, Exit::kPortHeld, "refusing to share the bus"},
    {BusStatus::LOCK_OPEN_FAILED, ENOENT, Exit::kCannotOpen, "does not exist"},
    {BusStatus::LOCK_OPEN_FAILED, ENXIO, Exit::kCannotOpen, "does not exist"},
    {BusStatus::LOCK_OPEN_FAILED, EACCES, Exit::kCannotOpen, "dialout"},
    {BusStatus::LOCK_OPEN_FAILED, EIO, Exit::kCannotOpen, "could not take '/dev/ttyUSB7'"},
    {BusStatus::NOT_A_TTY, ENOTTY, Exit::kCannotOpen, "is not a serial device"},
    {BusStatus::OPEN_FAILED, 0, Exit::kCannotOpen, "could not take '/dev/ttyUSB7': open failed"},
    {BusStatus::TERMIOS_FAILED, 0, Exit::kCannotOpen, "line settings could not be applied"},
    {BusStatus::EXCLUSIVE_FAILED, EPERM, Exit::kCannotOpen, "port refused exclusive access: "},
    {BusStatus::UNSUPPORTED_BAUDRATE, 0, Exit::kInternal, "after the parameters were validated"},
    {BusStatus::INVALID_TIMEOUT, 0, Exit::kInternal, "after the parameters were validated"},
    {BusStatus::ALREADY_OPEN, 0, Exit::kInternal, "after the parameters were validated"},
  };
  for (const Row & row : rows) {
    SCOPED_TRACE(std::string(waveshare_servos::to_string(row.status)) + " errno " +
      std::to_string(row.error));
    std::string message;
    EXPECT_EQ(open_failure(OpenResult{row.status, row.error}, port, holders, &message), row.exit);
    EXPECT_FALSE(message.empty());
    EXPECT_THAT(message, HasSubstr(row.says));
    if (row.exit != Exit::kInternal) {
      EXPECT_THAT(message, HasSubstr(port));
    }
  }
  std::string message = "stale";
  EXPECT_EQ(open_failure(OpenResult{BusStatus::OK, 0}, port, holders, &message), Exit::kOk);
  EXPECT_TRUE(message.empty());
}

TEST_F(ToolOpen, serial_speed_goes_to_stderr_and_stdout_stays_clean)
{
  // stdout is the tools' result channel (the scan table the HIL parses); the vendored begin()
  // printf()s its line to stdout, so open_bus points stdout at stderr for the length of the open.
  CapturedStdio captured;
  const Exit exit = open_bus(bus_, fake_.port(), kBaudrate, err_);
  captured.finish();
  EXPECT_EQ(exit, Exit::kOk) << err_.str();
  EXPECT_TRUE(bus_.is_open());
  EXPECT_EQ(count_of(captured.out(), "serial speed"), 0u) << captured.out();
  EXPECT_EQ(count_of(captured.err(), "serial speed 1000000\n"), 1u) << captured.err();
  EXPECT_EQ(bus_.io_timeout_ms(), 5u) << "the timeout of io_timeout_ms_for(1000000)";
}

TEST_F(ToolOpen, io_timeout_values)
{
  // C.0's table: an 8-byte request and a 43-byte reply at 10 bits a byte, plus 2 ms, never below
  // the driver's 5 ms.
  const std::vector<std::pair<int, uint32_t>> table = {
    {9600, 56}, {19200, 29}, {38400, 16}, {57600, 11}, {115200, 7}, {500000, 5}, {1000000, 5}};
  uint32_t previous = UINT32_MAX;
  for (const auto & row : table) {
    SCOPED_TRACE(row.first);
    const uint32_t timeout = io_timeout_ms_for(row.first);
    EXPECT_EQ(timeout, row.second);
    EXPECT_GE(timeout, static_cast<uint32_t>(waveshare_servos::defaults::kIoTimeoutMs));
    EXPECT_LE(timeout, previous) << "a faster rate never needs a longer window";
    previous = timeout;
  }
}

TEST_F(ToolOpen, every_run_function_returns_2_on_a_closed_bus_without_aborting)
{
  // The vendored readSCS would FD_SET(-1) on a closed bus and abort under _FORTIFY_SOURCE.
  fake_.add_servo(4);
  ASSERT_FALSE(bus_.is_open());
  EXPECT_EQ(scan_exit(scan(session_, 0, 253)), Exit::kCannotOpen);
  const SetIdReport moved = set_id(session_, 4, 253);
  EXPECT_EQ(moved.exit, Exit::kCannotOpen);
  EXPECT_EQ(moved.writes_sent, 0u);
  const CalibrateReport centred = calibrate_midpoint(session_, 4);
  EXPECT_EQ(centred.exit, Exit::kCannotOpen);
  EXPECT_EQ(centred.writes_sent, 0u);
  const FactoryResetReport reset = factory_reset(session_, 4);
  EXPECT_EQ(reset.exit, Exit::kCannotOpen);
  EXPECT_EQ(reset.writes_sent, 0u);
  writes_sent_ = moved.writes_sent + centred.writes_sent + reset.writes_sent;
  fake_.wait_quiet();
  EXPECT_EQ(fake_.bytes_received(), 0u);
}

// ---- ToolExit (final_exit, tool_main step 5) ----

TEST(ToolExit, the_port_gone_with_no_write_is_2)
{
  // A USB drop before anything was written: whatever the run concluded, the honest exit is
  // "cannot open", nothing written.
  for (const Exit outcome : {Exit::kOk, Exit::kNoAnswer, Exit::kRefused, Exit::kScanAnomaly}) {
    SCOPED_TRACE(static_cast<int>(outcome));
    EXPECT_EQ(final_exit(outcome, 0, false), Exit::kCannotOpen);
  }
}

TEST(ToolExit, the_port_gone_after_a_write_keeps_the_outcome)
{
  // After a write the outcome is what the servo's state is known to be: a USB drop after the id
  // write is 6, never "2: nothing sent".
  for (const Exit outcome : {Exit::kOk, Exit::kRefused, Exit::kNotApplied, Exit::kInconsistent}) {
    SCOPED_TRACE(static_cast<int>(outcome));
    EXPECT_EQ(final_exit(outcome, 1, false), outcome);
    EXPECT_EQ(final_exit(outcome, 4, false), outcome);
  }
}

TEST(ToolExit, interrupted_stays_130)
{
  EXPECT_EQ(final_exit(Exit::kInterrupted, 0, false), Exit::kInterrupted);
  EXPECT_EQ(final_exit(Exit::kInterrupted, 0, true), Exit::kInterrupted);
  EXPECT_EQ(final_exit(Exit::kInterrupted, 2, false), Exit::kInterrupted);
}

TEST(ToolExit, a_present_port_changes_nothing)
{
  for (const Exit outcome : {Exit::kOk, Exit::kPortHeld, Exit::kCannotOpen, Exit::kNoAnswer,
      Exit::kRefused, Exit::kNotApplied, Exit::kInconsistent, Exit::kScanAnomaly, Exit::kUsage,
      Exit::kInternal, Exit::kInterrupted})
  {
    SCOPED_TRACE(static_cast<int>(outcome));
    EXPECT_EQ(final_exit(outcome, 0, true), outcome);
    EXPECT_EQ(final_exit(outcome, 3, true), outcome);
  }
}

// ---- ToolScan [Q5 sets the range in every case] ----

TEST_F(ToolScan, pings_every_id_0_to_253_exactly_attempts_times_and_nothing_else)
{
  // The coverage gate, read off the fake's log and never off the tool's report: red for a 1..253
  // loop, a lost retry or an early stop. Nothing but PING goes out, and nothing to 254 or 255.
  const ScanResult result = run_scan();
  EXPECT_EQ(fake_.ping_counts(), empty_bus_pings());
  for (const FrameRecord & frame : fake_.frames()) {
    EXPECT_EQ(frame.instruction, kInstPing) << "to id " << static_cast<int>(frame.id);
    EXPECT_LE(frame.id, 253);
  }
  EXPECT_TRUE(result.rows.empty());
  EXPECT_EQ(scan_exit(result), Exit::kNoAnswer);
  const std::vector<std::string> lines = lines_of(out_.str());
  ASSERT_EQ(lines.size(), 2u) << out_.str();
  EXPECT_EQ(lines[0], kHeader);
  EXPECT_THAT(
    lines[1], MatchesRegex(
      "found no servo on " + fake_.port() + " at 1000000 baud \\(pinged ids 0\\.\\.253, 3 "
      "attempts each, [0-9]+\\.[0-9] s\\)"));
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "check servo power (USB does not power the servos), the wiring, and the baud rate: a "
      "servo set to another rate answers only at that rate"));
}

TEST_F(ToolScan, a_found_servo_is_pinged_until_it_answers_and_no_more)
{
  seed_bench_like(1, 0);
  fake_.drop_pings(1, 2);
  const ScanResult result = run_scan();
  std::map<uint8_t, int> expected = empty_bus_pings();
  expected[1] = 3;          // two lost pings, then the answer, and not one ping more
  EXPECT_EQ(fake_.ping_counts(), expected);
  ASSERT_EQ(result.rows.size(), 1u);
  EXPECT_EQ(result.rows[0].id, 1);
  EXPECT_EQ(scan_exit(result), Exit::kOk);
}

TEST_F(ToolScan, finds_servos_at_0_1_2_3_4_and_253_and_decodes_every_column)
{
  // Six servos with a value of their own in every column, so a column shift, a byte-order error or
  // one servo's data under another's id cannot pass. Both ends of the range are occupied.
  seed_bench_like(0, 0);
  seed_bench_like(1, 0);
  seed_bench_like(2, 0);
  seed_bench_like(3, 1);
  seed_bench_like(4, 1);
  seed_bench_like(253, 3);
  for (const uint8_t id : {0, 1, 2, 3, 4}) {
    fake_.set_byte(id, 63, static_cast<uint8_t>(30 + id));
  }
  fake_.set_byte(253, 63, 45);
  fake_.set_byte(1, 62, 121);                 // 12.1 V
  fake_.set_byte(2, 31, 0x05);                // offset raw 0x0805: -5, the sign on bit 11
  fake_.set_byte(2, 32, 0x08);
  fake_.set_byte(3, 3, 0x0b);                 // model word 0x0a0b = 2571, little endian
  fake_.set_byte(3, 4, 0x0a);
  fake_.set_position(4, -7);                  // the sign on bit 15
  fake_.set_position(253, 3000);
  fake_.set_status(253, 0x20);

  const ScanResult result = run_scan();
  std::map<uint8_t, int> expected = empty_bus_pings();
  for (const uint8_t id : {0, 1, 2, 3, 4, 253}) {
    expected[id] = 1;       // each answered its first ping and was pinged no more
  }
  EXPECT_EQ(fake_.ping_counts(), expected);

  EXPECT_THAT(row(0), ElementsAre("0", "pos", "0", "777", "0", "1000000", "1000", "12.2", "30",
    "0x00", "1"));
  EXPECT_THAT(row(1), ElementsAre("1", "pos", "0", "777", "0", "1000000", "1013", "12.1", "31",
    "0x00", "2"));
  EXPECT_THAT(row(2), ElementsAre("2", "pos", "0", "777", "0", "1000000", "1026", "12.2", "32",
    "0x00", "-5"));
  EXPECT_THAT(row(3), ElementsAre("3", "vel", "1", "2571", "0", "1000000", "1039", "12.2", "33",
    "0x00", "4"));
  EXPECT_THAT(row(4), ElementsAre("4", "vel", "1", "777", "0", "1000000", "-7", "12.2", "34",
    "0x00", "5"));
  EXPECT_THAT(row(253), ElementsAre("253", "-", "3", "777", "0", "1000000", "3000", "12.2", "45",
    "0x20", "2"));
  const std::vector<std::string> lines = lines_of(out_.str());
  ASSERT_EQ(lines.size(), 8u) << out_.str();
  EXPECT_THAT(lines[7],
    HasSubstr("found 6 servo(s) on " + fake_.port() + " at 1000000 baud: ids 0 1 2 3 4 253 ("));
  EXPECT_THAT(err_.str(), HasSubstr("id 253: mode 3 has no driver support"));
  EXPECT_THAT(err_.str(), HasSubstr("id 253: status 0x20"));
  EXPECT_EQ(scan_exit(result), Exit::kOk) << "notes are not anomalies\n" << err_.str();
}

TEST_F(ToolScan, rows_are_eleven_tokens_and_the_header_and_footer_are_fixed)
{
  // The HIL parser's contract (E.2 scan_table): the exact header, rows of exactly 11 tokens that
  // start with the id, the footer, and nothing else on stdout. The column widths are pinned too.
  seed_bench_like(1, 0);
  seed_bench_like(2, 1);
  const ScanResult result = run_scan();
  EXPECT_EQ(scan_exit(result), Exit::kOk);
  const std::vector<std::string> lines = lines_of(out_.str());
  ASSERT_EQ(lines.size(), 4u) << "header, two rows, footer and nothing else:\n" << out_.str();
  EXPECT_EQ(lines[0], kHeader);
  EXPECT_EQ(
    lines[1],
    "  1  pos      0    777         0  1000000      1013       12.2      30    0x00       2");
  EXPECT_EQ(
    lines[2],
    "  2  vel      1    777         0  1000000      1026       12.2      30    0x00       3");
  for (const std::size_t i : {1u, 2u}) {
    EXPECT_EQ(tokens_of(lines[i]).size(), 11u) << lines[i];
    EXPECT_EQ(lines[i].size(), lines[0].size()) << "a row is as wide as the header";
  }
  EXPECT_THAT(lines[3], MatchesRegex(footer_pattern("2 servo\\(s\\)")));
  EXPECT_THAT(lines[3], HasSubstr(": ids 1 2 ("));
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "use the id column as <param name=\"id\"> and the type column as <param name=\"type\">; "
      "see description/ros2_control/example.ros2_control.xacro"));
}

TEST_F(ToolScan, id_0_gets_the_driver_note)
{
  seed_bench_like(0, 0);
  const ScanResult result = run_scan();
  EXPECT_EQ(row(0).size(), 11u);
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "id 0: the hardware interface accepts ids 1..253; give this servo another id with set_id "
      "before putting it in a URDF"));
  EXPECT_EQ(scan_exit(result), Exit::kOk) << "a note, not an anomaly";
}

TEST_F(ToolScan, a_servo_that_pings_but_does_not_read_prints_question_marks_and_exits_7)
{
  seed_bench_like(1, 0);
  fake_.set_silent_read(1, 3);
  const ScanResult result = run_scan();
  EXPECT_THAT(row(1), ElementsAre("1", "?", "?", "?", "?", "?", "1013", "12.2", "30", "0x00", "?"))
    << "the identity columns, and the type and offset derived from them, print ?";
  EXPECT_EQ(reads_of(1, 3), 2) << "a failed read is retried once";
  EXPECT_THAT(
    err_.str(),
    HasSubstr("id 1: answered a ping but its identity block could not be read (silent)"));
  EXPECT_EQ(scan_exit(result), Exit::kScanAnomaly);
}

TEST_F(ToolScan, a_doubled_twin_is_an_anomaly)
{
  // Two servos on one id in bit synchrony: every reply is perfect and followed by a copy.
  seed_bench_like(1, 0);
  fake_.set_twin(1, TwinReply::doubled);
  const ScanResult result = run_scan();
  EXPECT_THAT(row(1), ElementsAre("1", "?", "?", "?", "?", "?", "?", "?", "?", "?", "?"));
  EXPECT_EQ(reads_of(1, 3) + reads_of(1, 56), 0) << "no register is read from an unclean id";
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "id 1: 6 extra bytes followed its reply; two servos may share this id -- connect them one "
      "at a time and use set_id"));
  EXPECT_EQ(scan_exit(result), Exit::kScanAnomaly);
}

TEST_F(ToolScan, a_garbled_twin_is_an_anomaly_not_an_absence)
{
  seed_bench_like(1, 0);
  fake_.set_twin(1, TwinReply::garbled);
  const ScanResult result = run_scan();
  EXPECT_EQ(row(1).size(), 11u) << "bytes came back: something is at id 1";
  EXPECT_THAT(
    err_.str(),
    HasSubstr("id 1: garbled replies: two servos may share this id, or the line is noisy"));
  EXPECT_THAT(out_.str(), HasSubstr("found 1 servo(s)"));
  EXPECT_EQ(scan_exit(result), Exit::kScanAnomaly) << "not 3: the bus is not empty";
}

TEST_F(ToolScan, a_garbled_reads_twin_is_an_anomaly)
{
  // Twins at different positions: the pings agree, the READ replies collide.
  seed_bench_like(1, 0);
  fake_.set_twin(1, TwinReply::garbled_reads);
  const ScanResult result = run_scan();
  EXPECT_THAT(row(1), ElementsAre("1", "?", "?", "?", "?", "?", "?", "?", "?", "0x00", "?"));
  EXPECT_THAT(err_.str(),
    HasSubstr("id 1: answered a ping but its identity block could not be read (garbled)"));
  EXPECT_THAT(err_.str(),
    HasSubstr("id 1: answered a ping but its feedback block could not be read (garbled)"));
  EXPECT_EQ(scan_exit(result), Exit::kScanAnomaly);
}

TEST_F(ToolScan, a_twin_seen_once_on_a_read_is_still_an_anomaly)
{
  // Review fix F6. Twins that ping in step and collide on ONE register read: the retry comes back
  // clean and every column reads, but the odd reply was a twin all the same -- as on the ping
  // path, it is kept. One doubled read at id 1, one garbled read at id 2.
  seed_bench_like(1, 0);
  seed_bench_like(2, 0);
  fake_.set_twin(1, TwinReply::doubled_reads, 1);
  fake_.set_twin(2, TwinReply::garbled_reads, 1);
  const ScanResult result = run_scan();
  EXPECT_THAT(row(1), ElementsAre("1", "pos", "0", "777", "0", "1000000", "1013", "12.2", "30",
    "0x00", "2")) << "the retry read everything";
  EXPECT_THAT(row(2), ElementsAre("2", "pos", "0", "777", "0", "1000000", "1026", "12.2", "30",
    "0x00", "3"));
  EXPECT_EQ(reads_of(1, 3), 2) << "the odd identity read was retried once";
  EXPECT_EQ(reads_of(2, 3), 2);
  EXPECT_THAT(
    err_.str(), ContainsRegex(
      "id 1: [0-9]+ extra bytes followed its reply; two servos may share this id -- connect them "
      "one at a time and use set_id"));
  EXPECT_THAT(
    err_.str(),
    HasSubstr("id 2: garbled replies: two servos may share this id, or the line is noisy"));
  EXPECT_EQ(scan_exit(result), Exit::kScanAnomaly) << err_.str();
}

TEST_F(ToolScan, an_id_register_mismatch_is_an_anomaly)
{
  seed_bench_like(1, 0);
  fake_.set_byte(1, 5, 9);                    // answers at 1, register 5 says 9
  const ScanResult result = run_scan();
  EXPECT_EQ(row(1).size(), 11u);
  EXPECT_THAT(err_.str(), HasSubstr("id 1: its id register reads 9"));
  EXPECT_EQ(scan_exit(result), Exit::kScanAnomaly);
}

TEST_F(ToolScan, a_baud_register_mismatch_is_an_anomaly)
{
  seed_bench_like(1, 0);
  seed_bench_like(2, 0);
  fake_.set_byte(1, 6, 4);                    // 115200 on a 1 Mbaud bus
  fake_.set_byte(2, 6, 9);                    // not a rate the table defines
  const ScanResult result = run_scan();
  EXPECT_THAT(row(1), ElementsAre("1", "pos", "0", "777", "4", "115200", "1013", "12.2", "30",
    "0x00", "2"));
  const std::vector<std::string> unknown_rate = row(2);
  ASSERT_EQ(unknown_rate.size(), 11u);
  EXPECT_EQ(unknown_rate[5], "?") << "a register value the table does not define prints ?";
  EXPECT_THAT(
    err_.str(),
    HasSubstr("id 1: its baud register (4 = 115200) disagrees with the bus rate 1000000"));
  EXPECT_THAT(
    err_.str(), HasSubstr("id 2: its baud register (9 = ?) disagrees with the bus rate 1000000"));
  EXPECT_EQ(scan_exit(result), Exit::kScanAnomaly);
}

TEST_F(ToolScan, scan_sends_no_write_of_any_kind)
{
  seed_bench_like(1, 0);
  seed_bench_like(2, 0);
  seed_bench_like(3, 1);
  seed_bench_like(4, 1);
  std::map<uint8_t, FakeServo> before;
  for (const uint8_t id : {1, 2, 3, 4}) {
    before.emplace(id, fake_.snapshot(id));   // seeded just above: every one is keyed
  }
  const ScanResult result = run_scan();
  EXPECT_EQ(scan_exit(result), Exit::kOk);
  EXPECT_TRUE(fake_.writes().empty());
  for (const FrameRecord & frame : fake_.frames()) {
    EXPECT_TRUE(frame.instruction == kInstPing || frame.instruction == kInstRead) <<
      "instruction " << static_cast<int>(frame.instruction);
  }
  for (const uint8_t id : {1, 2, 3, 4}) {
    const std::optional<FakeServo> after = servo_at(id);
    ASSERT_TRUE(after.has_value()) << "id " << static_cast<int>(id) << " moved";
    EXPECT_EQ(after->mem, before[id].mem) << "id " << static_cast<int>(id);
    EXPECT_EQ(after->eeprom, before[id].eeprom) << "id " << static_cast<int>(id);
  }
}

TEST_F(ToolScan, stop_flag_between_ids_exits_130_with_the_partial_table)
{
  seed_bench_like(1, 0);
  seed_bench_like(2, 0);
  seed_bench_like(200, 0);
  // Raised from another thread once id 20 is being pinged, as a signal would be.
  std::thread raiser([this] {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while (fake_.ping_counts().count(20) == 0 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      stop_ = 1;
    });
  const ScanResult result = run_scan();
  raiser.join();
  EXPECT_TRUE(result.interrupted);
  EXPECT_EQ(scan_exit(result), Exit::kInterrupted);
  ASSERT_EQ(result.rows.size(), 2u);
  EXPECT_EQ(result.rows[0].id, 1);
  EXPECT_EQ(result.rows[1].id, 2);
  const std::map<uint8_t, int> pings = fake_.ping_counts();
  ASSERT_FALSE(pings.empty());
  const int last = pings.rbegin()->first;
  EXPECT_GE(last, 20);
  EXPECT_LT(last, 200) << "it stopped between ids, long before 200";
  EXPECT_EQ(result.last_pinged, last);
  const std::vector<std::string> lines = lines_of(out_.str());
  ASSERT_EQ(lines.size(), 4u) << "header, the two rows found so far, footer:\n" << out_.str();
  EXPECT_EQ(lines[0], kHeader);
  EXPECT_THAT(lines[3], HasSubstr("ids 1 2 ("));
  EXPECT_THAT(lines[3], ContainsRegex(", interrupted after id " + std::to_string(last) + "$"));
}

TEST_F(ToolScan, scan_on_a_closed_bus_returns_2)
{
  seed_bench_like(1, 0);
  bus_.close();
  fake_.clear_frames();
  const ScanResult result = scan(session_, kScanFirstId, kScanLastId);
  EXPECT_EQ(scan_exit(result), Exit::kCannotOpen);
  fake_.wait_quiet();
  EXPECT_EQ(fake_.bytes_received(), 0u);
}

// ---- ToolSetId (C.2): writes() is compared as a WHOLE ORDERED LIST ----

TEST_F(ToolSetId, moves_4_to_253_and_verifies)
{
  const SetIdReport report = run(4, 253);
  EXPECT_EQ(report.exit, Exit::kOk) << err_.str();
  EXPECT_EQ(fake_.writes(), clean_move(4, 253));
  EXPECT_EQ(report.writes_sent, 3u);
  EXPECT_FALSE(servo_at(4).has_value()) << "nothing answers at 4 any more";
  const std::optional<FakeServo> moved = servo_at(253);
  ASSERT_TRUE(moved.has_value()) << "nothing is keyed at 253";
  EXPECT_EQ(moved->mem[5], 253);
  EXPECT_EQ(moved->mem[55], 1) << "the lock is left closed, as the driver's set_mode leaves it";
  EXPECT_EQ(moved->mem[33], 1) << "the mode is preserved";
  EXPECT_EQ(moved->mode_writes, 0);
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "about to give servo 4 the id 253. If this run is interrupted or fails, run scan: the "
      "servo will answer at 4 or 253."));
  EXPECT_EQ(
    out_.str(),
    "servo 4 is now id 253: it answers at 253 and no longer at 4, its registers are otherwise "
    "unchanged, and its EEPROM lock is closed. The id is stored in EEPROM; power-cycle the servo "
    "and run scan to confirm it kept id 253, then update <param name=\"id\"> in your URDF.\n");
  EXPECT_THAT(
    detail_line(report), MatchesRegex(
      "detail start_id=4 new_id=253 lock_before=1 unlock_read=0 id_write_ack=old_id "
      "ack_ms=[0-9]+ verify_ms=[0-9]+ new_id_pings=[0-9]+ late_ack_from=none late_ack_ms=none "
      "old_id_silent=true identity_same=true lock_after=1 writes_sent=3 verdict=ok"));
}

TEST_F(ToolSetId, the_id_survives_a_power_cycle_under_volatile_when_locked)
{
  // seed_bench_like leaves register 55 at 1, as on the bench: an id written without the verified
  // unlock, or before it, is applied and then lost at power-off (memory-table row 50).
  const SetIdReport report = run(4, 253);
  ASSERT_EQ(report.exit, Exit::kOk) << err_.str();
  fake_.power_cycle();
  EXPECT_EQ(bus_.checked_ping(253).kind, waveshare_servos::ReplyKind::ONE);
  EXPECT_EQ(bus_.checked_ping(4).kind, waveshare_servos::ReplyKind::SILENT);
  const std::optional<FakeServo> kept = servo_at(253);
  ASSERT_TRUE(kept.has_value()) << "the power cycle took the id back";
  EXPECT_EQ(kept->eeprom[5], 253) << "committed while unlocked";
}

TEST_P(ToolSetIdPolicy, succeeds_under_every_eeprom_policy)
{
  fake_.set_eeprom_policy(GetParam());
  const SetIdReport report = run(4, 253);
  EXPECT_EQ(report.exit, Exit::kOk) << err_.str();
  EXPECT_EQ(fake_.writes(), clean_move(4, 253));
  const std::optional<FakeServo> moved = servo_at(253);
  ASSERT_TRUE(moved.has_value());
  EXPECT_EQ(moved->eeprom[5], 253);
  EXPECT_EQ(moved->mem[55], 1);
}

INSTANTIATE_TEST_SUITE_P(
  ToolSetId, ToolSetIdPolicy,
  ::testing::Values(
    EepromPolicy::apply_always, EepromPolicy::drop_when_locked,
    EepromPolicy::volatile_when_locked), policy_name);

TEST_P(ToolSetIdAck, succeeds_whichever_id_acks_the_id_write)
{
  // SCS::Ack would call an ack from the new id a failure; an implementation that trusted the ack
  // either way fails two of the three. Only the recorded ack differs between them.
  fake_.set_id_write_ack(4, GetParam());
  const SetIdReport report = run(4, 253);
  EXPECT_EQ(report.exit, Exit::kOk) << err_.str();
  EXPECT_EQ(fake_.writes(), clean_move(4, 253));
  EXPECT_EQ(report.id_write_ack, ack_name(::testing::TestParamInfo<IdWriteAck>(GetParam(), 0)));
  const std::optional<FakeServo> moved = servo_at(253);
  ASSERT_TRUE(moved.has_value());
  EXPECT_EQ(moved->mem[5], 253);
}

INSTANTIATE_TEST_SUITE_P(
  ToolSetId, ToolSetIdAck,
  ::testing::Values(IdWriteAck::old_id, IdWriteAck::new_id, IdWriteAck::none), ack_name);

TEST_F(ToolSetId, succeeds_for_a_servo_that_acks_no_write)
{
  // Register 8 = 0: writes apply and nothing is ever acked. The read-backs are the verdict.
  fake_.set_write_acks(4, false);
  const SetIdReport report = run(4, 253);
  EXPECT_EQ(report.exit, Exit::kOk) << err_.str();
  EXPECT_EQ(fake_.writes(), clean_move(4, 253));
  EXPECT_EQ(report.id_write_ack, "none");
  const std::optional<FakeServo> moved = servo_at(253);
  ASSERT_TRUE(moved.has_value());
  EXPECT_EQ(moved->mem[55], 1);
}

TEST_F(ToolSetId, refuses_a_taken_new_id_before_addressing_the_start_id)
{
  // The bench's H14 `taken` case sends start id 200 (silent) and new id 3 (a servo): it can write
  // nothing even to a regressed tool only because the taken check comes first and sends nothing
  // at all to S.
  seed_bench_like(3, 1);
  const SetIdReport report = run(4, 3);
  EXPECT_EQ(report.exit, Exit::kRefused);
  EXPECT_TRUE(fake_.writes().empty());
  EXPECT_EQ(frames_to(4), 0u) << "no frame at all to the start id";
  EXPECT_GE(fake_.ping_counts()[3], 1);
  for (const FrameRecord & frame : fake_.frames()) {
    EXPECT_EQ(frame.id, 3) << "every frame is a ping of the new id";
    EXPECT_EQ(frame.instruction, kInstPing);
  }
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "id 3 already answers on '" + fake_.port() + "'; refusing to give servo 4 an id that is "
      "taken -- two servos on one id answer on top of each other and cannot be told apart. Pick "
      "a free id; scan lists the taken ones. Nothing was written."));
  const std::optional<FakeServo> untouched = servo_at(4);
  ASSERT_TRUE(untouched.has_value());
  EXPECT_EQ(untouched->mem[5], 4);
}

TEST_F(ToolSetId, refuses_when_the_new_id_answers_only_on_the_last_attempt)
{
  // Every attempt is used even after silence: a servo that missed two pings is still there.
  seed_bench_like(253, 0);
  fake_.drop_pings(253, 2);
  const SetIdReport report = run(4, 253);
  EXPECT_EQ(report.exit, Exit::kRefused);
  EXPECT_TRUE(fake_.writes().empty());
  EXPECT_EQ(fake_.ping_counts()[253], 3);
  EXPECT_EQ(frames_to(4), 0u);
}

TEST_F(ToolSetId, refuses_a_silent_start_id_exit_3_no_writes)
{
  const SetIdReport report = run(9, 253);
  EXPECT_EQ(report.exit, Exit::kNoAnswer);
  EXPECT_TRUE(fake_.writes().empty());
  EXPECT_EQ(fake_.ping_counts(), (std::map<uint8_t, int>{{9, 3}, {253, 3}}));
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "no servo answers at id 9 on '" + fake_.port() + "' at 1000000 baud (3 pings); nothing was "
      "written. scan lists the ids that do answer."));
}

TEST_F(ToolSetId, refuses_a_doubled_twin_at_the_start_id_exit_4_no_writes)
{
  fake_.set_twin(4, TwinReply::doubled);
  const SetIdReport report = run(4, 253);
  EXPECT_EQ(report.exit, Exit::kRefused);
  EXPECT_TRUE(fake_.writes().empty());
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "more than one servo may answer at id 4 (factory-new servos all start at id 1); connect "
      "only the servo to renumber. Nothing was written."));
}

TEST_F(ToolSetId, refuses_a_garbled_reads_twin_at_the_start_id_exit_4_no_writes)
{
  // Twins at different positions: the pings agree and the register reads collide.
  fake_.set_twin(4, TwinReply::garbled_reads);
  const SetIdReport report = run(4, 253);
  EXPECT_EQ(report.exit, Exit::kRefused);
  EXPECT_TRUE(fake_.writes().empty());
  EXPECT_THAT(err_.str(), HasSubstr("more than one servo may answer at id 4"));
}

TEST_F(ToolSetId, a_start_id_garbled_once_then_clean_is_refused_exit_4_no_writes)
{
  // Review fix F5 (R10). Two factory-new servos on id 4 collide on the first ping and happen to
  // be in step for the second: the collision was the only twin evidence there will be, and it is
  // not thrown away because a later ping came back clean.
  fake_.set_twin(4, TwinReply::garbled, 1);
  const SetIdReport report = run(4, 253);
  EXPECT_EQ(report.exit, Exit::kRefused) << err_.str();
  EXPECT_TRUE(fake_.writes().empty());
  EXPECT_EQ(fake_.ping_counts()[4], 2) << "garbled, then clean, then no more";
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "more than one servo may answer at id 4 (factory-new servos all start at id 1); connect "
      "only the servo to renumber. Nothing was written."));
}

TEST_F(ToolSetId, refuses_an_id_register_mismatch_exit_4)
{
  fake_.set_byte(4, 5, 7);
  const SetIdReport report = run(4, 253);
  EXPECT_EQ(report.exit, Exit::kRefused);
  EXPECT_TRUE(fake_.writes().empty());
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "the servo answering at id 4 has 7 in its id register; refusing to write to a servo in an "
      "inconsistent state"));
}

TEST_F(ToolSetId, stops_before_the_id_write_when_the_lock_does_not_open)
{
  // Register 55 reads 1 and a write to it is ignored: the unlock is not verified, so the id is
  // never written, and the best-effort relock is the only other write.
  fake_.set_ignore_write(4, 55, true);
  const SetIdReport report = run(4, 253);
  EXPECT_EQ(report.exit, Exit::kNotApplied);
  EXPECT_EQ(
    fake_.writes(), (std::vector<WriteRecord>{WriteRecord{4, 55, {0}}, WriteRecord{4, 55, {1}}}));
  EXPECT_EQ(report.unlock_read, 1);
  const std::optional<FakeServo> untouched = servo_at(4);
  ASSERT_TRUE(untouched.has_value());
  EXPECT_EQ(untouched->mem[5], 4);
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "could not open the EEPROM write lock of id 4 (register 55 reads 1 after writing 0); no "
      "EEPROM byte was changed"));
  EXPECT_EQ(report.lock_after, 1) << "the best-effort relock is read back and recorded";
  EXPECT_THAT(err_.str(), Not(HasSubstr("may still be open")));
}

TEST_F(ToolSetId, an_unlock_whose_read_back_is_lost_is_relocked_and_verified_exit_5)
{
  // Review fix F1. The unlock applies, but its read-back is lost: the lock may be open, so the
  // relock matters, and its read-back is what says whether it closed. The servo acks no write,
  // so each write waits out its 100 ms window and the knob lands between write and read-back.
  fake_.set_write_acks(4, false);
  AfterWrites lose(fake_, 1, [this] {fake_.set_silent_read(4, 55, 1);});
  const SetIdReport report = run(4, 253);
  lose.join();
  EXPECT_EQ(report.exit, Exit::kNotApplied) << err_.str();
  EXPECT_EQ(
    fake_.writes(), (std::vector<WriteRecord>{WriteRecord{4, 55, {0}}, WriteRecord{4, 55, {1}}}));
  EXPECT_EQ(report.unlock_read, -1);
  EXPECT_EQ(report.lock_after, 1) << "the relock was read back as 1";
  EXPECT_THAT(detail_line(report), HasSubstr(" unlock_read=none "));
  EXPECT_THAT(detail_line(report), HasSubstr(" lock_after=1 "));
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "could not open the EEPROM write lock of id 4 (register 55 reads nothing (silent) after "
      "writing 0); no EEPROM byte was changed\n"));
  const std::optional<FakeServo> relocked = servo_at(4);
  ASSERT_TRUE(relocked.has_value());
  EXPECT_EQ(relocked->mem[55], 1);
}

TEST_F(ToolSetId, a_lock_left_unverified_after_a_failed_unlock_is_named_exit_5)
{
  // Review fix F1: neither read-back of 55 comes back. Nothing is known about the lock, and exit
  // 5 names register 55 (A.3: SRAM 55 may differ, and is named).
  fake_.set_write_acks(4, false);
  AfterWrites lose(fake_, 1, [this] {fake_.set_silent_read(4, 55);});
  const SetIdReport report = run(4, 253);
  lose.join();
  EXPECT_EQ(report.exit, Exit::kNotApplied) << err_.str();
  EXPECT_EQ(
    fake_.writes(), (std::vector<WriteRecord>{WriteRecord{4, 55, {0}}, WriteRecord{4, 55, {1}}}));
  EXPECT_EQ(report.lock_after, -1);
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "no EEPROM byte was changed; its EEPROM lock may still be open until it is power-cycled "
      "(register 55 reads nothing (silent))"));
}

TEST_F(ToolSetId, an_id_write_that_does_not_take_is_relocked_exit_5)
{
  fake_.set_ignore_write(4, 5, true);
  const SetIdReport report = run(4, 253);
  EXPECT_EQ(report.exit, Exit::kNotApplied);
  EXPECT_EQ(
    fake_.writes(), (std::vector<WriteRecord>{WriteRecord{4, 55, {0}}, WriteRecord{4, 5, {253}},
      WriteRecord{4, 55, {1}}}));
  const std::optional<FakeServo> relocked = servo_at(4);
  ASSERT_TRUE(relocked.has_value());
  EXPECT_EQ(relocked->mem[55], 1);
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "the id write did not take: the servo still answers at 4 and not at 253 (ack: old_id); its "
      "EEPROM lock is closed again and nothing changed."));
}

TEST_F(ToolSetId, a_servo_lost_after_the_write_exits_6_with_no_further_writes)
{
  // context/motor_reset_command_email.png: after an id write the servo answered at neither id.
  fake_.set_vanish_after_id_write(4);
  const SetIdReport report = run(4, 253);
  EXPECT_EQ(report.exit, Exit::kInconsistent);
  EXPECT_EQ(
    fake_.writes(), (std::vector<WriteRecord>{WriteRecord{4, 55, {0}}, WriteRecord{4, 5, {253}}}));
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "after writing id 253 to the servo at 4, neither id answers. Run scan (it covers ids "
      "0..253). If it answers nowhere, power-cycle it and scan again. Once scan finds it, "
      "factory_reset can put it back to its factory settings; the reset keeps whatever id it "
      "answers at, so it cannot find a lost servo by itself."));
}

TEST_F(ToolSetId, a_lock_that_does_not_close_exits_6)
{
  // The lock starts open here, so a servo that ignores writes to 55 still passes the unlock
  // (it reads 0) and moves; the knob travels with it to 253, where the closing lock fails.
  fake_.set_byte(4, 55, 0);
  fake_.set_ignore_write(4, 55, true);
  const SetIdReport report = run(4, 253);
  EXPECT_EQ(report.exit, Exit::kInconsistent);
  EXPECT_EQ(fake_.writes(), clean_move(4, 253));
  EXPECT_EQ(report.lock_after, 0);
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "servo 4 is now id 253 and answers there, but its EEPROM lock did not close (register 55 "
      "reads 0); later EEPROM writes to it are not protected until it is power-cycled"));
}

TEST_F(ToolSetId, an_unreadable_servo_at_the_new_id_is_relocked_exit_6)
{
  // Review fix F9. The move happened (N answers, S is silent), but the identity read at N fails:
  // exit 6, and the lock the run opened is closed again -- exactly one servo answers at N, so a
  // verified lock there is safe -- and the message says so rather than leaving it open unnamed.
  AfterWrites unreadable(fake_, 2, [this] {fake_.set_silent_read(253, 3);});
  const SetIdReport report = run(4, 253);
  unreadable.join();
  EXPECT_EQ(report.exit, Exit::kInconsistent) << err_.str();
  EXPECT_EQ(fake_.writes(), clean_move(4, 253));
  EXPECT_EQ(report.lock_after, 1);
  const std::optional<FakeServo> moved = servo_at(253);
  ASSERT_TRUE(moved.has_value());
  EXPECT_EQ(moved->mem[55], 1);
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "the servo now answers at id 253, but its registers cannot be read (silent); its EEPROM "
      "lock is closed again; run scan"));
}

TEST_P(ToolSetIdSlowCommit, a_slow_eeprom_commit_is_waited_out)
{
  // The commit outlasts the 100 ms ack window. An ack from the old id then lands in a ping of
  // the new one, where it looks like a reply from the wrong servo: it is recorded as the one
  // tolerated late ack, and the ping is repeated.
  fake_.set_id_write_ack(4, GetParam());
  fake_.set_eeprom_commit_ms(4, 200);
  const SetIdReport report = run(4, 253);
  EXPECT_EQ(report.exit, Exit::kOk) << err_.str();
  EXPECT_EQ(fake_.writes(), clean_move(4, 253));
  const std::optional<FakeServo> moved = servo_at(253);
  ASSERT_TRUE(moved.has_value());
  EXPECT_EQ(moved->eeprom[5], 253);
  if (GetParam() == IdWriteAck::old_id) {
    EXPECT_THAT(detail_line(report), HasSubstr(" late_ack_from=4 "));
    EXPECT_EQ(report.late_ack_from, 4);
    EXPECT_GE(report.late_ack_ms, 150);
  } else {
    EXPECT_THAT(detail_line(report), HasSubstr(" late_ack_from=none "));
  }
}

INSTANTIATE_TEST_SUITE_P(
  ToolSetId, ToolSetIdSlowCommit,
  ::testing::Values(IdWriteAck::old_id, IdWriteAck::new_id, IdWriteAck::none), ack_name);

TEST_F(ToolSetId, a_commit_longer_than_the_verify_window_exits_6)
{
  fake_.set_eeprom_commit_ms(4, 800);
  const SetIdReport report = run(4, 253);
  EXPECT_EQ(report.exit, Exit::kInconsistent);
  EXPECT_EQ(
    fake_.writes(), (std::vector<WriteRecord>{WriteRecord{4, 55, {0}}, WriteRecord{4, 5, {253}}}))
    << "no write after the id write";
  EXPECT_THAT(err_.str(), HasSubstr("neither id answers"));
}

TEST_F(ToolSetId, a_reply_from_a_third_id_during_verify_exits_6)
{
  // One late ack is tolerated; a stranger is not. The override is switched on once the id write
  // is out (the ack was built before it, so the late ack itself still comes from 4).
  fake_.set_eeprom_commit_ms(4, 200);
  AfterWrites stranger(
    fake_, 2, [this] {
      // Nothing moved to 253 (a tool that never wrote the id): no servo to override, and an
      // exception must not escape this thread.
      try {
        fake_.set_reply_id_override(253, 99);
        fake_.set_faults_apply_to_addressed(true);
      } catch (const std::out_of_range &) {
      }
    });
  const SetIdReport report = run(4, 253);
  stranger.join();
  EXPECT_EQ(report.exit, Exit::kInconsistent);
  EXPECT_EQ(
    fake_.writes(), (std::vector<WriteRecord>{WriteRecord{4, 55, {0}}, WriteRecord{4, 5, {253}}}));
  EXPECT_THAT(err_.str(), HasSubstr("wrong_id from id 99"));
}

TEST_F(ToolSetId, stop_flag_before_the_first_write_exits_130_with_no_writes)
{
  stop_ = 1;
  const SetIdReport report = run(4, 253);
  EXPECT_EQ(report.exit, Exit::kInterrupted);
  EXPECT_TRUE(fake_.writes().empty());
  EXPECT_THAT(err_.str(), HasSubstr("interrupted; nothing was written"));
}

TEST_F(ToolSetId, stop_flag_during_the_pre_write_checks_exits_130_with_no_writes)
{
  // Review fix F12. Raised once the first ping of N is out, i.e. after the entry check: two more
  // silent pings of N, the pings of S and its register reads come before the last check, which
  // is the one that has to catch it -- before the pre-write notice and before any write.
  std::thread raiser([this] {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while (fake_.ping_counts()[253] < 1 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::microseconds(200));
      }
      stop_ = 1;
    });
  const SetIdReport report = run(4, 253);
  raiser.join();
  EXPECT_EQ(report.exit, Exit::kInterrupted) << err_.str();
  EXPECT_TRUE(fake_.writes().empty());
  EXPECT_EQ(fake_.ping_counts()[253], 3) << "step 1 ran to its end: the flag came after the entry";
  EXPECT_THAT(err_.str(), HasSubstr("interrupted; nothing was written"));
  EXPECT_THAT(err_.str(), Not(HasSubstr("about to give servo")));
  EXPECT_THAT(err_.str(), Not(HasSubstr("completed first")));
}

TEST_F(ToolSetId, a_garbled_id_write_ack_is_recorded_without_an_ack_time)
{
  // Review fix F28. Acks are advisory, so the move succeeds; a garbled ack's elapsed time may be
  // its window's end, so no ack_ms is recorded for it -- the rule calibrate_midpoint follows.
  fake_.set_garble_write_acks(4, true);
  const SetIdReport report = run(4, 253);
  EXPECT_EQ(report.exit, Exit::kOk) << err_.str();
  EXPECT_EQ(fake_.writes(), clean_move(4, 253));
  EXPECT_EQ(report.id_write_ack, "garbled");
  EXPECT_EQ(report.ack_ms, -1);
  EXPECT_THAT(detail_line(report), HasSubstr(" id_write_ack=garbled ack_ms=none "));
}

TEST_F(ToolSetId, stop_flag_after_the_first_write_is_deferred)
{
  // Raised while the servo is committing its new id: the sequence runs to its verified lock.
  fake_.set_eeprom_commit_ms(4, 200);
  AfterWrites raiser(fake_, 2, [this] {stop_ = 1;});
  const SetIdReport report = run(4, 253);
  raiser.join();
  EXPECT_EQ(report.exit, Exit::kOk) << err_.str();
  EXPECT_EQ(fake_.writes(), clean_move(4, 253));
  const std::optional<FakeServo> moved = servo_at(253);
  ASSERT_TRUE(moved.has_value());
  EXPECT_EQ(moved->mem[55], 1);
  EXPECT_TRUE(report.signal_deferred);
  EXPECT_THAT(
    err_.str(), HasSubstr("a signal arrived during the EEPROM sequence; it was completed first"));
}

TEST_F(ToolSetId, nothing_is_printed_during_the_sequence)
{
  // Observed from the commit-latency window: from the first write (the unlock) until the closing
  // lock goes out, `err` must not grow -- not even by the late-ack line this run has to report,
  // which arrives in the middle of that window and is only printed once the sequence is over.
  LockedBuffer buffer;
  std::ostream err(&buffer);
  Session session{bus_, waveshare_servos::defaults::kPingAttempts, kIoTimeoutMs, &stop_, out_,
    err};
  fake_.set_eeprom_commit_ms(4, 200);
  std::vector<std::size_t> sizes;             // err's size whenever 1 or 2 writes were out
  std::atomic<bool> done{false};
  std::thread watcher([&] {
      while (!done.load()) {
        const std::size_t writes = fake_.writes().size();
        if (writes >= 1 && writes < 3) {
          sizes.push_back(buffer.size());
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
      }
    });
  const SetIdReport report = set_id(session, 4, 253);
  done.store(true);
  watcher.join();
  writes_sent_ = report.writes_sent;
  EXPECT_EQ(report.exit, Exit::kOk) << buffer.text();
  EXPECT_EQ(report.late_ack_from, 4) << "the window held something to print";
  ASSERT_GT(sizes.size(), 100u) << "the watcher saw the window";
  EXPECT_EQ(*std::min_element(sizes.begin(), sizes.end()),
    *std::max_element(sizes.begin(), sizes.end())) << "err grew during the sequence";
  const std::string text = buffer.text();
  EXPECT_THAT(text.substr(0, sizes.front()), HasSubstr("about to give servo 4 the id 253"))
    << "the notice was out before the first write";
  EXPECT_THAT(text.substr(sizes.front()), HasSubstr("late ack from id 4"))
    << "and the late-ack line only after the sequence";
}

TEST_F(ToolSetId, edges_start_0_and_new_253)
{
  seed_bench_like(0, 0);
  const SetIdReport report = run(0, 253);
  EXPECT_EQ(report.exit, Exit::kOk) << err_.str();
  EXPECT_EQ(fake_.writes(), clean_move(0, 253));
  const std::optional<FakeServo> moved = servo_at(253);
  ASSERT_TRUE(moved.has_value());
  EXPECT_EQ(moved->mem[5], 253);
}

// ---- ToolCalibrate (C.3) [Q1, Q2, Q3] ----

TEST_F(ToolCalibrate, centres_a_position_servo)
{
  const CalibrateReport report = run(2);
  EXPECT_EQ(report.exit, Exit::kOk) << err_.str();
  EXPECT_EQ(fake_.writes(), clean_calibration());
  EXPECT_EQ(report.writes_sent, 4u);
  const std::optional<FakeServo> servo = servo_at(2);
  ASSERT_TRUE(servo.has_value());
  EXPECT_EQ(servo->mode_writes, 0) << "no Mode(id, 0) on a servo already in mode 0";
  EXPECT_EQ(fake_.word(2, 56), 2048);
  EXPECT_EQ(servo->calibrations, 1);
  EXPECT_EQ(servo->mem[40], 0) << "the torque is left off";
  EXPECT_EQ(servo->mem[55], 1) << "the lock is left closed";
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "about to calibrate the midpoint of servo 2 (an EEPROM write of its offset, registers "
      "31-32). If this run is interrupted or fails, run scan: the servo will answer at 2."));
  // seeded offset 3 at a present 1026: the shaft is at 1029, so the new offset is 1029 - 2048
  EXPECT_EQ(
    out_.str(),
    "servo 2 now reads 2048 at the position that read 1026; offset registers 31-32 went from 3 "
    "to -1019 (raw 0x0003 -> 0x0bfb). Its torque is OFF; the hardware interface turns it on at "
    "activate. EEPROM lock closed. Power-cycle the servo and run scan to confirm the offset "
    "survived.\n");
  EXPECT_THAT(
    detail_line(report), MatchesRegex(
      "detail id=2 mode=0 torque_before=1 torque_written=true settle_ms=[0-9]+ "
      "position_before=1026 offset_raw_before=0x0003 unlock_read=0 calibrate_ack=old_id "
      "ack_ms=[0-9]+ late_ack_from=none late_ack_ms=none position_after=2048 "
      "offset_raw_after=0x0bfb offset_sign=\\+1 register40_after=0 torque_final=0 "
      "identity_same=true lock_after=1 writes_sent=4 verdict=ok"));
}

TEST_F(ToolCalibrate, with_torque_already_off_the_torque_write_is_skipped)
{
  fake_.set_byte(2, 40, 0);
  const CalibrateReport report = run(2);
  EXPECT_EQ(report.exit, Exit::kOk) << err_.str();
  EXPECT_EQ(
    fake_.writes(), (std::vector<WriteRecord>{WriteRecord{2, 55, {0}}, WriteRecord{2, 40, {128}},
      WriteRecord{2, 55, {1}}}));
  EXPECT_FALSE(report.torque_written);
  EXPECT_THAT(detail_line(report), HasSubstr(" torque_before=0 torque_written=false "));
  EXPECT_EQ(fake_.word(2, 56), 2048);
}

TEST_F(ToolCalibrate, a_firmware_that_turns_torque_on_after_128_is_switched_off_again)
{
  fake_.set_register40_after(2, 1);
  const CalibrateReport report = run(2);
  EXPECT_EQ(report.exit, Exit::kOk) << err_.str();
  EXPECT_EQ(
    fake_.writes(), (std::vector<WriteRecord>{WriteRecord{2, 40, {0}}, WriteRecord{2, 55, {0}},
      WriteRecord{2, 40, {128}}, WriteRecord{2, 40, {0}}, WriteRecord{2, 55, {1}}}));
  EXPECT_EQ(report.register40_after, 1);
  EXPECT_EQ(report.torque_final, 0);
  const std::optional<FakeServo> servo = servo_at(2);
  ASSERT_TRUE(servo.has_value());
  EXPECT_EQ(servo->mem[40], 0);
}

TEST_F(ToolCalibrate, the_offset_survives_a_power_cycle_under_volatile_when_locked)
{
  // Register 55 reads 1 when the tool starts, as on the bench. Without the verified unlock the
  // offset is applied, the position reads 2048, and a power cycle takes both back.
  const CalibrateReport report = run(2);
  ASSERT_EQ(report.exit, Exit::kOk) << err_.str();
  fake_.power_cycle();
  const std::optional<FakeServo> servo = servo_at(2);
  ASSERT_TRUE(servo.has_value());
  EXPECT_EQ(servo->eeprom[31], 0xfb);
  EXPECT_EQ(servo->eeprom[32], 0x0b);
  EXPECT_EQ(fake_.word(2, 56), 2048) << "the new frame outlived the power cycle";
}

TEST_F(ToolCalibrate, refuses_a_wheel_exit_4_zero_writes)
{
  // [Q1] A midpoint means nothing to a wheel, and the old tool's Mode(id, 0) would have silently
  // made it a position servo.
  seed_bench_like(3, 1);
  const CalibrateReport report = run(3);
  EXPECT_EQ(report.exit, Exit::kRefused);
  EXPECT_TRUE(fake_.writes().empty());
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "servo 3 is in mode 1 (wheel); a midpoint only means something to a position servo (mode "
      "0). Changing the mode is an EEPROM write this tool does not make: declare the joint type "
      "'pos' and let the hardware interface switch it at configure, then calibrate. Nothing was "
      "written."));
}

TEST_F(ToolCalibrate, refuses_a_silent_id_exit_3)
{
  const CalibrateReport report = run(9);
  EXPECT_EQ(report.exit, Exit::kNoAnswer);
  EXPECT_TRUE(fake_.writes().empty());
  EXPECT_EQ(fake_.ping_counts(), (std::map<uint8_t, int>{{9, 3}}));
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "no servo answers at id 9 on '" + fake_.port() + "' at 1000000 baud (3 pings); nothing was "
      "written. scan lists the ids that do answer."));
}

TEST_F(ToolCalibrate, refuses_twins_exit_4)
{
  fake_.set_twin(2, TwinReply::doubled);
  const CalibrateReport report = run(2);
  EXPECT_EQ(report.exit, Exit::kRefused);
  EXPECT_TRUE(fake_.writes().empty());
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "more than one servo may answer at id 2; connect only the servo to calibrate. Nothing was "
      "written."));
}

TEST_F(ToolCalibrate, a_firmware_that_ignores_128_is_not_applied_exit_5_and_relocked)
{
  // The offset model off is the fake's "calibration unsupported": 128 is stored in register 40
  // and nothing else happens. Torque goes off again, the lock closes, and the exit says so.
  fake_.set_offset_model(2, false);
  const CalibrateReport report = run(2);
  EXPECT_EQ(report.exit, Exit::kNotApplied) << err_.str();
  EXPECT_EQ(
    fake_.writes(), (std::vector<WriteRecord>{WriteRecord{2, 40, {0}}, WriteRecord{2, 55, {0}},
      WriteRecord{2, 40, {128}}, WriteRecord{2, 40, {0}}, WriteRecord{2, 55, {1}}}));
  const std::optional<FakeServo> servo = servo_at(2);
  ASSERT_TRUE(servo.has_value());
  EXPECT_EQ(servo->mem[55], 1);
  EXPECT_EQ(servo->mem[40], 0);
  // review fix F21/F8: the torque the run switched off is named, as A.3 row 5 requires
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "the calibration did not take: position still reads 1026 and the offset register is "
      "unchanged; any lock this run opened is closed again; its torque is now OFF.\n"));
}

TEST_F(ToolCalibrate, a_moving_servo_is_refused_exit_4_with_only_the_torque_write)
{
  // [Q2] A servo that is still moving once its torque is off has no midpoint to take. A helper
  // thread moves the shaft between reads for longer than the settle window.
  std::atomic<bool> done{false};
  std::thread mover([this, &done] {
      int position = 1026;
      while (!done.load()) {
        position += 5;
        fake_.set_position(2, position);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });
  const CalibrateReport report = run(2);
  done.store(true);
  mover.join();
  EXPECT_EQ(report.exit, Exit::kRefused);
  EXPECT_EQ(fake_.writes(), (std::vector<WriteRecord>{WriteRecord{2, 40, {0}}}));
  EXPECT_THAT(err_.str(), HasSubstr("servo 2 is still moving ("));
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "); hold it still at the intended midpoint and run again. Its torque is now OFF; no EEPROM "
      "byte was written."));
  const std::optional<FakeServo> servo = servo_at(2);
  ASSERT_TRUE(servo.has_value());
  EXPECT_EQ(servo->calibrations, 0);
}

TEST_F(ToolCalibrate, an_already_centred_servo_succeeds_without_an_offset_change)
{
  fake_.set_position(2, 2048);
  const CalibrateReport report = run(2);
  EXPECT_EQ(report.exit, Exit::kOk) << err_.str();
  EXPECT_EQ(fake_.writes(), clean_calibration());
  EXPECT_EQ(report.offset_raw_after, report.offset_raw_before);
  EXPECT_THAT(detail_line(report), HasSubstr(" offset_sign=0 "));
  EXPECT_EQ(fake_.word(2, 56), 2048);
}

TEST_F(ToolCalibrate, stops_before_128_when_the_lock_does_not_open)
{
  // [Q3] 55 reads 1 and ignores writes: no 128 without a verified unlock; the torque it already
  // switched off is named.
  fake_.set_ignore_write(2, 55, true);
  const CalibrateReport report = run(2);
  EXPECT_EQ(report.exit, Exit::kNotApplied);
  EXPECT_EQ(
    fake_.writes(), (std::vector<WriteRecord>{WriteRecord{2, 40, {0}}, WriteRecord{2, 55, {0}},
      WriteRecord{2, 55, {1}}}));
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "could not open the EEPROM write lock of id 2 (register 55 reads 1 after writing 0); no "
      "EEPROM byte was changed; its torque is now OFF"));
  const std::optional<FakeServo> servo = servo_at(2);
  ASSERT_TRUE(servo.has_value());
  EXPECT_EQ(servo->calibrations, 0);
}

TEST_F(ToolCalibrate, a_slow_calibration_commit_is_waited_out)
{
  // The 128 writes 31-32, so it commits like an EEPROM write: its ack outlasts the 100 ms window
  // and lands in a read of the position, as a bare status frame from id 2.
  fake_.set_eeprom_commit_ms(2, 200);
  const CalibrateReport report = run(2);
  EXPECT_EQ(report.exit, Exit::kOk) << err_.str();
  EXPECT_EQ(fake_.writes(), clean_calibration());
  EXPECT_THAT(detail_line(report), HasSubstr(" late_ack_from=2 "));
  EXPECT_EQ(report.late_ack_from, 2);
  EXPECT_EQ(fake_.word(2, 56), 2048);
}

// ---- review fixes (phase6_evidence/DEVIATIONS.md, "review fix F<n>") ----

namespace
{

// The SIGHUP handler of a_signal_pending_before_the_unlock_exits_130: like tool_main's, it only
// sets a flag, and that flag is the session's stop.
volatile std::sig_atomic_t g_hup = 0;

void note_hup(int signal_number)
{
  static_cast<void>(signal_number);
  g_hup = 1;
}

}  // namespace

TEST_F(ToolCalibrate, an_id_garbled_once_then_clean_is_refused_exit_4_no_writes)
{
  // Review fix F5 (R10), as for set_id: a collision on the first ping is twin evidence, whatever
  // the second ping says.
  fake_.set_twin(2, TwinReply::garbled, 1);
  const CalibrateReport report = run(2);
  EXPECT_EQ(report.exit, Exit::kRefused) << err_.str();
  EXPECT_TRUE(fake_.writes().empty());
  EXPECT_EQ(fake_.ping_counts()[2], 2) << "garbled, then clean, then no more";
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "more than one servo may answer at id 2; connect only the servo to calibrate. Nothing was "
      "written."));
}

TEST_F(ToolCalibrate, a_lock_left_unverified_after_a_failed_unlock_is_named_exit_5)
{
  // Review fix F1, calibrate's side: both read-backs of 55 are lost, so whether the lock closed
  // again is unknown, and the exit-5 message names register 55 next to the torque. Acks are off,
  // so the unlock waits out its 100 ms window and the knob lands before its read-back.
  fake_.set_write_acks(2, false);
  AfterWrites lose(fake_, 2, [this] {fake_.set_silent_read(2, 55);});
  const CalibrateReport report = run(2);
  lose.join();
  EXPECT_EQ(report.exit, Exit::kNotApplied) << err_.str();
  EXPECT_EQ(
    fake_.writes(), (std::vector<WriteRecord>{WriteRecord{2, 40, {0}}, WriteRecord{2, 55, {0}},
      WriteRecord{2, 55, {1}}}));
  EXPECT_EQ(report.lock_after, -1);
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "no EEPROM byte was changed; its EEPROM lock may still be open until it is power-cycled "
      "(register 55 reads nothing (silent)); its torque is now OFF"));
  const std::optional<FakeServo> servo = servo_at(2);
  ASSERT_TRUE(servo.has_value());
  EXPECT_EQ(servo->calibrations, 0);
}

TEST_F(ToolCalibrate, an_offset_that_changes_after_it_was_read_is_inconsistent_exit_6)
{
  // Review fix F2. The firmware ignores 128 (the offset model off), and the offset registers move
  // between step 8's read and step 10's identity read -- a commit that lands late. The later read
  // is the truth: no "unchanged, nothing changed" exit 5 on the stale one. Acks are off, so the
  // closing lock's 100 ms window holds the knob's landing before the identity read.
  fake_.set_offset_model(2, false);
  fake_.set_write_acks(2, false);
  AfterWrites late(fake_, 4, [this] {fake_.set_byte(2, 31, 0x07);});
  const CalibrateReport report = run(2);
  late.join();
  EXPECT_EQ(report.exit, Exit::kInconsistent) << err_.str();
  EXPECT_EQ(report.offset_raw_after, 0x0003) << "step 8 read the offset as it was";
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "the offset registers 31-32 of servo 2 read 0x0003 after the calibration write and 0x0007 "
      "at the final check; its offset is unknown. Run scan."));
}

TEST_F(ToolCalibrate, a_stop_during_the_settle_exits_130_before_the_unlock)
{
  // Review fix F7. The torque is off and an arm may be sagging when the user presses Ctrl-C:
  // nothing has been written to EEPROM yet, and nothing is. R4's reason for deferring a signal --
  // never die between unlock and lock -- starts at the unlock, so the last check is just before it.
  AfterWrites raiser(fake_, 1, [this] {stop_ = 1;});
  const CalibrateReport report = run(2);
  raiser.join();
  EXPECT_EQ(report.exit, Exit::kInterrupted) << err_.str();
  EXPECT_EQ(fake_.writes(), (std::vector<WriteRecord>{WriteRecord{2, 40, {0}}}));
  const std::optional<FakeServo> servo = servo_at(2);
  ASSERT_TRUE(servo.has_value());
  EXPECT_EQ(servo->calibrations, 0);
  EXPECT_EQ(servo->mem[55], 1) << "the lock was never touched";
  EXPECT_THAT(
    err_.str(), HasSubstr("interrupted; its torque is now OFF; no EEPROM byte was written\n"));
  EXPECT_THAT(err_.str(), Not(HasSubstr("completed first")));
}

TEST_F(ToolCalibrate, a_signal_pending_before_the_unlock_exits_130)
{
  // Review fix F7, through a signal: the sequence blocks the four stop signals on this thread from
  // its first write, so a SIGHUP sent to it stays pending -- its handler has not run, the flag is
  // still 0 -- when the last check comes. Only sigpending() can see it there.
  struct sigaction mine{};
  mine.sa_handler = note_hup;
  sigemptyset(&mine.sa_mask);
  struct sigaction saved{};
  ASSERT_EQ(::sigaction(SIGHUP, &mine, &saved), 0);
  g_hup = 0;
  Session session{bus_, waveshare_servos::defaults::kPingAttempts, kIoTimeoutMs, &g_hup, out_,
    err_};
  const pthread_t main_thread = ::pthread_self();
  AfterWrites sender(fake_, 1, [main_thread] {::pthread_kill(main_thread, SIGHUP);});
  const CalibrateReport report = calibrate_midpoint(session, 2);
  sender.join();
  writes_sent_ = report.writes_sent;
  fake_.wait_quiet();
  ::sigaction(SIGHUP, &saved, nullptr);
  EXPECT_EQ(report.exit, Exit::kInterrupted) << err_.str();
  EXPECT_EQ(g_hup, 1) << "the handler ran once the mask was restored";
  EXPECT_TRUE(report.signal_deferred);
  EXPECT_EQ(fake_.writes(), (std::vector<WriteRecord>{WriteRecord{2, 40, {0}}}));
  EXPECT_THAT(
    err_.str(), HasSubstr("interrupted; its torque is now OFF; no EEPROM byte was written\n"));
  EXPECT_THAT(err_.str(), Not(HasSubstr("completed first")));
}

TEST_F(ToolCalibrate, stop_flag_during_the_pre_write_checks_exits_130_with_no_writes)
{
  // Review fix F12. Raised once the first (dropped) ping is out, after the entry check: the last
  // check before the notice is what has to catch it.
  fake_.drop_pings(2, 2);
  std::thread raiser([this] {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while (fake_.ping_counts()[2] < 1 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::microseconds(200));
      }
      stop_ = 1;
    });
  const CalibrateReport report = run(2);
  raiser.join();
  EXPECT_EQ(report.exit, Exit::kInterrupted) << err_.str();
  EXPECT_TRUE(fake_.writes().empty());
  EXPECT_EQ(fake_.ping_counts()[2], 3) << "the flag came after the entry check";
  EXPECT_THAT(err_.str(), HasSubstr("interrupted; nothing was written"));
  EXPECT_THAT(err_.str(), Not(HasSubstr("about to calibrate")));
}

TEST_F(ToolCalibrate, stop_flag_after_the_calibration_write_is_deferred)
{
  // Review fix F12: from the unlock on, a stop waits for the verified lock (R4).
  fake_.set_eeprom_commit_ms(2, 200);
  AfterWrites raiser(fake_, 3, [this] {stop_ = 1;});
  const CalibrateReport report = run(2);
  raiser.join();
  EXPECT_EQ(report.exit, Exit::kOk) << err_.str();
  EXPECT_EQ(fake_.writes(), clean_calibration());
  EXPECT_TRUE(report.signal_deferred);
  EXPECT_THAT(
    err_.str(), HasSubstr("a signal arrived during the EEPROM sequence; it was completed first"));
  EXPECT_EQ(fake_.word(2, 56), 2048);
}

TEST_F(ToolCalibrate, a_torque_that_will_not_go_off_is_not_applied_exit_5)
{
  // Review fix F17 (a), the Q2 lurch guard: no unlock and no 128 while the torque is still on.
  fake_.set_ignore_write(2, 40, true);
  const CalibrateReport report = run(2);
  EXPECT_EQ(report.exit, Exit::kNotApplied) << err_.str();
  EXPECT_EQ(fake_.writes(), (std::vector<WriteRecord>{WriteRecord{2, 40, {0}}}));
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "could not switch the torque of servo 2 off (register 40 reads 1); nothing was changed"));
  const std::optional<FakeServo> servo = servo_at(2);
  ASSERT_TRUE(servo.has_value());
  EXPECT_EQ(servo->calibrations, 0);
}

TEST_F(ToolCalibrate, a_torque_read_back_lost_before_the_unlock_is_named_exit_5)
{
  // Review fix F8 (c): a torque write whose read-back is lost leaves the torque state unknown,
  // which "nothing was changed" would misstate. A 50 ms ack window for SRAM writes and no acks,
  // so the knob lands between the torque write and its read-back.
  fake_.set_write_acks(2, false);
  Session session{bus_, waveshare_servos::defaults::kPingAttempts, 50, &stop_, out_, err_};
  AfterWrites lost(fake_, 1, [this] {fake_.set_silent_read(2, 40, 1);});
  const CalibrateReport report = calibrate_midpoint(session, 2);
  lost.join();
  writes_sent_ = report.writes_sent;
  fake_.wait_quiet();
  EXPECT_EQ(report.exit, Exit::kNotApplied) << err_.str();
  EXPECT_EQ(fake_.writes(), (std::vector<WriteRecord>{WriteRecord{2, 40, {0}}}));
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "could not switch the torque of servo 2 off (register 40 reads nothing (silent)); its "
      "torque state is unknown; no EEPROM byte was written"));
}

TEST_F(ToolCalibrate, a_lock_that_does_not_close_exits_6)
{
  // Review fix F17 (b), the Q3 verified lock: the lock starts open, so a servo that ignores
  // writes to 55 passes the unlock (it reads 0), takes the 128, and never locks again.
  fake_.set_byte(2, 55, 0);
  fake_.set_ignore_write(2, 55, true);
  const CalibrateReport report = run(2);
  EXPECT_EQ(report.exit, Exit::kInconsistent) << err_.str();
  EXPECT_EQ(fake_.writes(), clean_calibration());
  EXPECT_EQ(report.lock_after, 0);
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "the EEPROM lock of servo 2 did not close after the calibration write (register 55 reads "
      "0); later EEPROM writes to it are not protected until it is power-cycled. Run scan."));
}

TEST_F(ToolCalibrate, a_torque_that_comes_back_on_and_stays_on_exits_6)
{
  // Review fix F17 (c), Q2 after the 128: the firmware turns the torque on, and the write that
  // should switch it off again is ignored. The commit's 200 ms hold the knob's landing.
  fake_.set_register40_after(2, 1);
  fake_.set_eeprom_commit_ms(2, 200);
  AfterWrites stuck(fake_, 3, [this] {fake_.set_ignore_write(2, 40, true);});
  const CalibrateReport report = run(2);
  stuck.join();
  EXPECT_EQ(report.exit, Exit::kInconsistent) << err_.str();
  EXPECT_EQ(
    fake_.writes(), (std::vector<WriteRecord>{WriteRecord{2, 40, {0}}, WriteRecord{2, 55, {0}},
      WriteRecord{2, 40, {128}}, WriteRecord{2, 40, {0}}, WriteRecord{2, 55, {1}}}));
  EXPECT_EQ(report.torque_final, 1);
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "the torque of servo 2 will not go off after the calibration write (register 40 reads 1); "
      "its offset is unknown. Run scan."));
}

TEST_F(ToolCalibrate, a_torque_read_back_lost_after_the_calibration_write_is_named)
{
  // Review fix F23: an unread register 40 prints as what happened, never as the value -1.
  fake_.set_eeprom_commit_ms(2, 200);
  AfterWrites lost(fake_, 3, [this] {fake_.set_silent_read(2, 40);});
  const CalibrateReport report = run(2);
  lost.join();
  EXPECT_EQ(report.exit, Exit::kInconsistent) << err_.str();
  EXPECT_EQ(report.torque_final, -1);
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "the torque of servo 2 cannot be confirmed off after the calibration write (register 40 "
      "reads nothing (silent)); its offset is unknown. Run scan."));
  EXPECT_THAT(err_.str(), Not(HasSubstr("reads -1")));
}

TEST_F(ToolCalibrate, a_calibration_that_changes_another_register_exits_6)
{
  // Review fix F17 (d): registers 3..39 other than 31-32 must read back as before.
  fake_.set_eeprom_commit_ms(2, 200);
  AfterWrites stray(fake_, 3, [this] {fake_.set_byte(2, 13, 77);});
  const CalibrateReport report = run(2);
  stray.join();
  EXPECT_EQ(report.exit, Exit::kInconsistent) << err_.str();
  EXPECT_EQ(fake_.writes(), clean_calibration());
  EXPECT_EQ(report.identity_same, std::optional<bool>(false));
  EXPECT_THAT(
    err_.str(), ContainsRegex(
      "the calibration write changed more than the offset of servo 2: register 13 changed from "
      "[0-9]+ to 77; run scan"));
}

// ---- ToolFactoryReset (factory_reset_evidence/FACTORY_RESET_SPEC.md 2) ----

namespace
{

// Runs `action` once the fake has seen a RESET, or after 10 s: the AfterWrites of the reset.
class AfterReset
{
public:
  template<typename Action>
  AfterReset(const FakeBus & fake, Action action)
  : thread_([&fake, action] {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (fake.resets().empty() && std::chrono::steady_clock::now() < deadline) {
          std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
        action();
      })
  {
  }

  ~AfterReset() {join();}

  AfterReset(const AfterReset &) = delete;
  AfterReset & operator=(const AfterReset &) = delete;
  AfterReset(AfterReset &&) = delete;
  AfterReset & operator=(AfterReset &&) = delete;

  void join()
  {
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  std::thread thread_;
};

}  // namespace

TEST_F(ToolFactoryReset, resets_a_wheel_to_factory_and_keeps_its_id)
{
  const FactoryResetReport report = run(4);
  EXPECT_EQ(report.exit, Exit::kOk) << err_.str();
  EXPECT_EQ(fake_.writes(), (std::vector<WriteRecord>{WriteRecord{4, 40, {0}}})) <<
    "the torque goes off first, and nothing else is written";
  EXPECT_EQ(fake_.resets(), (std::vector<uint8_t>{4}));
  EXPECT_EQ(report.writes_sent, 2u);
  const std::optional<FakeServo> servo = servo_at(4);
  ASSERT_TRUE(servo.has_value());
  EXPECT_EQ(servo->mem[5], 4);
  EXPECT_EQ(servo->mem[33], 0);
  EXPECT_EQ(servo->mem[7], 0);
  EXPECT_EQ(fake_.word(4, 31, 11), 0);
  EXPECT_EQ(servo->mem[40], 0) << "the torque is left off";
  EXPECT_EQ(servo->mem[55], 1) << "the lock is left closed";
  EXPECT_EQ(fake_.word(4, 56), 1057) << "the shaft did not move; the offset under it did";
  EXPECT_EQ(bus_.baudrate(), kBaudrate) << "a servo at the factory rate needs no switch";
  EXPECT_THAT(err_.str(), HasSubstr(std::string(kNotice) + ".\n"));
  EXPECT_EQ(
    out_.str(),
    "servo 4 is reset to its factory settings and keeps id 4. Registers changed: 7: 5 -> 0, 31: 5 "
    "-> 0, 33: 1 -> 0. Its offset (the midpoint calibration) is 0 now; run calibrate_midpoint if "
    "it needs one. It is in mode 0 (position) now; a joint declared 'vel' is switched back at "
    "configure. Its torque is OFF and its EEPROM lock closed. Power-cycle the servo and run scan "
    "to confirm the settings survived.\n");
  EXPECT_THAT(
    detail_line(report), MatchesRegex(
      "detail id=4 model=777 baud_reg_before=0 offset_raw_before=0x0005 mode_before=1 "
      "torque_before=1 torque_written=true lock_before=1 reset_ack=old_id ack_ms=[0-9]+ "
      "late_ack_from=none late_ack_ms=none baudrate_after=1000000 verify_ms=[0-9]+ "
      "baud_reg_after=0 offset_raw_after=0x0000 mode_after=0 changed_registers=3 torque_final=0 "
      "lock_after=1 writes_sent=2 verdict=ok"));
}

TEST_F(ToolFactoryReset, with_torque_already_off_only_the_reset_is_sent)
{
  fake_.set_byte(4, 40, 0);
  const FactoryResetReport report = run(4);
  EXPECT_EQ(report.exit, Exit::kOk) << err_.str();
  EXPECT_TRUE(fake_.writes().empty());
  EXPECT_EQ(fake_.resets(), (std::vector<uint8_t>{4}));
  EXPECT_EQ(report.writes_sent, 1u);
  EXPECT_FALSE(report.torque_written);
  EXPECT_THAT(detail_line(report), HasSubstr(" torque_before=0 torque_written=false "));
}

TEST_F(ToolFactoryReset, moves_a_servo_at_500000_to_the_factory_rate_and_verifies_it_there)
{
  // M3: the RESET goes out at 500000 and is acked at that rate; afterwards the servo answers at
  // 1000000 only. The bus follows it without letting go of the port.
  at_500000();
  const FactoryResetReport report = run(4);
  EXPECT_EQ(report.exit, Exit::kOk) << err_.str();
  EXPECT_EQ(bus_.baudrate(), kFactoryBaudrate);
  EXPECT_EQ(fake_.resets(), (std::vector<uint8_t>{4}));
  const std::optional<FakeServo> servo = servo_at(4);
  ASSERT_TRUE(servo.has_value());
  EXPECT_EQ(servo->mem[6], 0);
  EXPECT_THAT(
    err_.str(), HasSubstr(
      std::string(kNotice) + ", at 500000 baud before the reset and at 1000000 baud after it.\n"));
  EXPECT_THAT(out_.str(), HasSubstr("Registers changed: 6: 1 -> 0, 7: 5 -> 0, 31: 5 -> 0, 33: "));
  EXPECT_THAT(
    out_.str(), HasSubstr(
      "It now talks at 1000000 baud, not 500000: give scan and the hardware interface baudrate "
      "1000000 for it. "));
  EXPECT_THAT(
    detail_line(report), ContainsRegex(
      " baud_reg_before=1 .* reset_ack=old_id .* baudrate_after=1000000 verify_ms=[0-9]+ "
      "baud_reg_after=0 "));
}

TEST_F(ToolFactoryReset, a_servo_already_at_factory_changes_nothing_and_says_so)
{
  // M4: a servo already at its factory values is reset all the same, and nothing reads
  // differently afterwards. That is a success, not a "did not take".
  fake_.set_byte(4, 33, 0);
  fake_.set_byte(4, 7, 0);
  fake_.set_word(4, 31, 0, 11);
  const FactoryResetReport report = run(4);
  EXPECT_EQ(report.exit, Exit::kOk) << err_.str();
  EXPECT_EQ(report.changed_registers, 0);
  EXPECT_EQ(
    out_.str(),
    "servo 4 is reset to its factory settings and keeps id 4. No register in 3..39 changed: it "
    "was at them already. Its torque is OFF and its EEPROM lock closed. Power-cycle the servo and "
    "run scan to confirm the settings survived.\n");
}

TEST_F(ToolFactoryReset, refuses_a_silent_id_exit_3)
{
  const FactoryResetReport report = run(9);
  EXPECT_EQ(report.exit, Exit::kNoAnswer);
  EXPECT_TRUE(fake_.writes().empty());
  EXPECT_TRUE(fake_.resets().empty());
  EXPECT_EQ(fake_.ping_counts(), (std::map<uint8_t, int>{{9, 3}}));
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "no servo answers at id 9 on '" + fake_.port() + "' at 1000000 baud (3 pings); nothing was "
      "written. scan lists the ids that do answer."));
  EXPECT_EQ(
    detail_line(report),
    "detail id=9 model=none baud_reg_before=none offset_raw_before=none mode_before=none "
    "torque_before=none torque_written=false lock_before=none reset_ack=not_sent ack_ms=none "
    "late_ack_from=none late_ack_ms=none baudrate_after=none verify_ms=none baud_reg_after=none "
    "offset_raw_after=none mode_after=none changed_registers=none torque_final=none "
    "lock_after=none writes_sent=0 verdict=no_answer");
}

TEST_F(ToolFactoryReset, refuses_twins_exit_4_and_sends_no_reset)
{
  fake_.set_twin(4, TwinReply::doubled);
  const FactoryResetReport report = run(4);
  EXPECT_EQ(report.exit, Exit::kRefused);
  EXPECT_TRUE(fake_.writes().empty());
  EXPECT_TRUE(fake_.resets().empty());
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "more than one servo may answer at id 4; connect only the servo to reset. Nothing was "
      "written."));
}

TEST_F(ToolFactoryReset, an_id_garbled_once_then_clean_is_refused_exit_4)
{
  // PHASE6 review fix F5, as for set_id and calibrate: one collision is twin evidence.
  fake_.set_twin(4, TwinReply::garbled, 1);
  const FactoryResetReport report = run(4);
  EXPECT_EQ(report.exit, Exit::kRefused) << err_.str();
  EXPECT_TRUE(fake_.writes().empty());
  EXPECT_TRUE(fake_.resets().empty());
  EXPECT_EQ(fake_.ping_counts()[4], 2) << "garbled, then clean, then no more";
}

TEST_F(ToolFactoryReset, refuses_an_id_register_mismatch_exit_4)
{
  fake_.set_byte(4, 5, 7);
  const FactoryResetReport report = run(4);
  EXPECT_EQ(report.exit, Exit::kRefused);
  EXPECT_TRUE(fake_.writes().empty());
  EXPECT_TRUE(fake_.resets().empty());
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "the servo answering at id 4 has 7 in its id register; refusing to write to a servo in an "
      "inconsistent state"));
}

TEST_F(ToolFactoryReset, a_firmware_without_reset_is_not_applied_exit_5)
{
  // An instruction the firmware does not know goes unanswered and changes nothing. The torque the
  // run switched off is named, as A.3 row 5 requires of an SRAM change.
  fake_.set_reset_supported(4, false);
  const FactoryResetReport report = run(4);
  EXPECT_EQ(report.exit, Exit::kNotApplied) << err_.str();
  EXPECT_EQ(fake_.writes(), (std::vector<WriteRecord>{WriteRecord{4, 40, {0}}}));
  EXPECT_EQ(fake_.resets(), (std::vector<uint8_t>{4}));
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "the reset did not take: servo 4 reads exactly as before in registers 3..39 (reset ack: "
      "none); its firmware may not know the RESET instruction; its torque is now OFF.\n"));
  EXPECT_THAT(detail_line(report), HasSubstr(" reset_ack=none "));
  EXPECT_THAT(detail_line(report), HasSubstr(" verdict=not_applied"));
}

TEST_F(ToolFactoryReset, a_firmware_without_reset_at_500000_is_found_at_its_old_rate_exit_5)
{
  // Silent at the factory rate is not proof the reset happened and lost the servo: it may never
  // have happened. The tool looks at the old rate too before it says anything.
  at_500000();
  fake_.set_reset_supported(4, false);
  const FactoryResetReport report = run(4);
  EXPECT_EQ(report.exit, Exit::kNotApplied) << err_.str();
  EXPECT_EQ(bus_.baudrate(), 500000);
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "the reset did not take: servo 4 still answers at 500000 baud and reads exactly as before "
      "in registers 3..39 (reset ack: none); its firmware may not know the RESET instruction; its "
      "torque is now OFF.\n"));
  EXPECT_THAT(detail_line(report), HasSubstr(" baudrate_after=500000 "));
}

TEST_F(ToolFactoryReset, a_partial_reset_exits_6)
{
  fake_.set_reset_skip(4, 33, true);
  const FactoryResetReport report = run(4);
  EXPECT_EQ(report.exit, Exit::kInconsistent) << err_.str();
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "the reset of servo 4 did not reach the factory values: baud register 0, offset raw 0x0000 "
      "and mode 1 (factory: 0, 0x0000 and 0); registers changed: 7: 5 -> 0, 31: 5 -> 0. Run "
      "scan.\n"));
  EXPECT_THAT(detail_line(report), HasSubstr(" mode_after=1 changed_registers=2 "));
}

TEST_F(ToolFactoryReset, a_servo_that_stays_silent_after_the_reset_exits_6)
{
  // A commit longer than the ack window and the verify window together: the servo is lost for
  // the whole of the read-back, and its state is unknown.
  fake_.set_eeprom_commit_ms(4, 800);
  const FactoryResetReport report = run(4);
  EXPECT_EQ(report.exit, Exit::kInconsistent) << err_.str();
  EXPECT_EQ(fake_.resets(), (std::vector<uint8_t>{4}));
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "servo 4 does not answer at id 4 after the reset (looked for at 1000000 baud for 500 ms); "
      "its settings are unknown. Run scan with -p baudrate:=1000000, the rate a reset servo talks "
      "at; power-cycle it if it answers nowhere.\n"));
}

TEST_F(ToolFactoryReset, a_slow_reset_commit_is_waited_out_and_its_late_ack_tolerated)
{
  // The flash rewrite outlasts the 100 ms ack window, so the ack lands in a read of the identity
  // block as a bare status frame from id 4: the one late ack a run tolerates.
  fake_.set_eeprom_commit_ms(4, 150);
  const FactoryResetReport report = run(4);
  EXPECT_EQ(report.exit, Exit::kOk) << err_.str();
  EXPECT_EQ(report.reset_ack, "none");
  EXPECT_EQ(report.late_ack_from, 4);
  EXPECT_GE(report.late_ack_ms, 100);
  EXPECT_THAT(detail_line(report), HasSubstr(" reset_ack=none ack_ms=none late_ack_from=4 "));
  EXPECT_THAT(err_.str(), HasSubstr("late ack from id 4, "));
  EXPECT_THAT(
    err_.str(), HasSubstr(" ms after the reset, caught by a read of id 4; it was repeated"));
}

TEST_F(ToolFactoryReset, a_torque_that_will_not_go_off_is_not_applied_exit_5_and_no_reset_is_sent)
{
  fake_.set_ignore_write(4, 40, true);
  const FactoryResetReport report = run(4);
  EXPECT_EQ(report.exit, Exit::kNotApplied) << err_.str();
  EXPECT_EQ(fake_.writes(), (std::vector<WriteRecord>{WriteRecord{4, 40, {0}}}));
  EXPECT_TRUE(fake_.resets().empty());
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "could not switch the torque of servo 4 off (register 40 reads 1); nothing was changed"));
}

TEST_F(ToolFactoryReset, stop_flag_during_the_pre_write_checks_exits_130_with_nothing_sent)
{
  // Raised once the first (dropped) ping is out, after the entry check: the last check before the
  // notice is what has to catch it.
  fake_.drop_pings(4, 2);
  std::thread raiser([this] {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while (fake_.ping_counts()[4] < 1 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::microseconds(200));
      }
      stop_ = 1;
    });
  const FactoryResetReport report = run(4);
  raiser.join();
  EXPECT_EQ(report.exit, Exit::kInterrupted) << err_.str();
  EXPECT_TRUE(fake_.writes().empty());
  EXPECT_TRUE(fake_.resets().empty());
  EXPECT_THAT(err_.str(), HasSubstr("interrupted; nothing was written"));
  EXPECT_THAT(err_.str(), Not(HasSubstr("about to reset")));
}

TEST_F(ToolFactoryReset, a_stop_after_the_torque_write_exits_130_before_the_reset)
{
  // Review fix F7's rule for the reset: the torque write comes before anything that cannot be
  // undone, so a stop there is still acted on. The servo acks no write and the SRAM ack window is
  // 50 ms, which is what gives the raiser time to land between the torque write and the check.
  fake_.set_write_acks(4, false);
  Session session{bus_, waveshare_servos::defaults::kPingAttempts, 50, &stop_, out_, err_};
  AfterWrites raiser(fake_, 1, [this] {stop_ = 1;});
  const FactoryResetReport report = factory_reset(session, 4);
  raiser.join();
  writes_sent_ = report.writes_sent;
  fake_.wait_quiet();
  EXPECT_EQ(report.exit, Exit::kInterrupted) << err_.str();
  EXPECT_EQ(fake_.writes(), (std::vector<WriteRecord>{WriteRecord{4, 40, {0}}}));
  EXPECT_TRUE(fake_.resets().empty());
  EXPECT_THAT(
    err_.str(), HasSubstr("interrupted; its torque is now OFF; the reset was not sent\n"));
  EXPECT_THAT(err_.str(), Not(HasSubstr("completed first")));
}

TEST_F(ToolFactoryReset, a_signal_pending_before_the_reset_exits_130)
{
  // The same, through a signal: blocked by the sequence, it is still pending -- its handler has
  // not run, the flag is 0 -- when the last check comes, and only sigpending() can see it.
  struct sigaction mine{};
  mine.sa_handler = note_hup;
  sigemptyset(&mine.sa_mask);
  struct sigaction saved{};
  ASSERT_EQ(::sigaction(SIGHUP, &mine, &saved), 0);
  g_hup = 0;
  fake_.set_write_acks(4, false);
  Session session{bus_, waveshare_servos::defaults::kPingAttempts, 50, &g_hup, out_, err_};
  const pthread_t main_thread = ::pthread_self();
  AfterWrites sender(fake_, 1, [main_thread] {::pthread_kill(main_thread, SIGHUP);});
  const FactoryResetReport report = factory_reset(session, 4);
  sender.join();
  writes_sent_ = report.writes_sent;
  fake_.wait_quiet();
  ::sigaction(SIGHUP, &saved, nullptr);
  EXPECT_EQ(report.exit, Exit::kInterrupted) << err_.str();
  EXPECT_EQ(g_hup, 1) << "the handler ran once the mask was restored";
  EXPECT_TRUE(report.signal_deferred);
  EXPECT_TRUE(fake_.resets().empty());
  EXPECT_THAT(
    err_.str(), HasSubstr("interrupted; its torque is now OFF; the reset was not sent\n"));
}

TEST_F(ToolFactoryReset, a_stop_after_the_reset_is_deferred_to_the_end_of_the_read_back)
{
  // From the RESET on, a stop waits: the run ends with the servo found and read back.
  fake_.set_eeprom_commit_ms(4, 50);
  AfterReset raiser(fake_, [this] {stop_ = 1;});
  const FactoryResetReport report = run(4);
  raiser.join();
  EXPECT_EQ(report.exit, Exit::kOk) << err_.str();
  EXPECT_TRUE(report.signal_deferred);
  EXPECT_THAT(
    err_.str(), HasSubstr("a signal arrived during the EEPROM sequence; it was completed first"));
}

TEST_F(ToolFactoryReset, the_reset_survives_a_power_cycle_under_volatile_when_locked)
{
  const FactoryResetReport report = run(4);
  ASSERT_EQ(report.exit, Exit::kOk) << err_.str();
  fake_.power_cycle();
  const std::optional<FakeServo> servo = servo_at(4);
  ASSERT_TRUE(servo.has_value());
  EXPECT_EQ(servo->mem[33], 0);
  EXPECT_EQ(servo->mem[7], 0);
  EXPECT_EQ(fake_.word(4, 31, 11), 0);
}

TEST_F(ToolFactoryReset, a_reset_that_leaves_the_lock_open_is_closed_and_verified)
{
  // The ST3025's reset closes the lock (M5); a firmware that leaves SRAM alone would leave a lock
  // this run found open still open, so the tool closes it.
  fake_.set_byte(4, 55, 0);
  fake_.set_reset_keeps_sram(4, true);
  const FactoryResetReport report = run(4);
  EXPECT_EQ(report.exit, Exit::kOk) << err_.str();
  EXPECT_EQ(
    fake_.writes(), (std::vector<WriteRecord>{WriteRecord{4, 40, {0}}, WriteRecord{4, 55, {1}}}));
  EXPECT_THAT(detail_line(report), HasSubstr(" lock_before=0 "));
  EXPECT_THAT(detail_line(report), HasSubstr(" lock_after=1 "));
  const std::optional<FakeServo> servo = servo_at(4);
  ASSERT_TRUE(servo.has_value());
  EXPECT_EQ(servo->mem[55], 1);
}

TEST_F(ToolFactoryReset, a_lock_that_will_not_close_after_the_reset_exits_6)
{
  fake_.set_byte(4, 55, 0);
  fake_.set_reset_keeps_sram(4, true);
  fake_.set_ignore_write(4, 55, true);
  const FactoryResetReport report = run(4);
  EXPECT_EQ(report.exit, Exit::kInconsistent) << err_.str();
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "the EEPROM lock of servo 4 did not close after the reset (register 55 reads 0); later "
      "EEPROM writes to it are not protected until it is power-cycled. Run scan.\n"));
}

TEST_F(ToolFactoryReset, a_reply_from_another_id_during_the_read_back_exits_6)
{
  // The 50 ms commit holds the ack back long enough for the knob to land before the read-back.
  fake_.set_eeprom_commit_ms(4, 50);
  AfterReset stranger(fake_, [this] {
      fake_.set_faults_apply_to_addressed(true);
      fake_.set_reply_id_override(4, 9);
    });
  const FactoryResetReport report = run(4);
  stranger.join();
  EXPECT_EQ(report.exit, Exit::kInconsistent) << err_.str();
  EXPECT_THAT(
    err_.str(), HasSubstr(
      "after the reset of servo 4 the replies were not clean (wrong_id from id 9 at a read of 4); "
      "its settings are unknown. Run scan.\n"));
}
