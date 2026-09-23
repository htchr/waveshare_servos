// Tests for the executables scan, set_id and calibrate_midpoint (PHASE6_SPEC D.5), and
// factory_reset (factory_reset_evidence/FACTORY_RESET_SPEC.md).
//
// test_servo_tools drives the tools' logic in-process. What it cannot reach belongs to the PROCESS:
// how the parameters arrive through rclcpp, the exit code a shell sees, which stream gets what,
// the signal handlers, and that the port is let go on every exit. So every case here spawns the
// BUILT binary ($WAVESHARE_TOOL_*, from the ENV of this test's ctest entry) against the fake bus
// of test/fake_servo_bus.hpp on an openpty() pair, and the fake's frame log and raw byte count --
// never anything a tool says about itself -- are the witness of what reached the wire.
//
// Two things keep a child off the bench, and neither is optional (R17):
//   - DefaultPortGuard holds defaults::kPort for the whole suite: opened, flocked and made
//     exclusive, and never a byte sent. A child that ignores `port` and falls back to the default
//     is refused with EBUSY. A port another process holds is waited for (30 s), never trusted:
//     that holder may let go mid-suite. SetUpTestSuite fails the suite outright when it cannot
//     prove the refusal, and then no tool is spawned at all;
//   - the fake's servos sit at ids 11-14, never the bench's 1-4, so even a child that reached a
//     real bus would address ids nobody answers at.
//
// Every exit is asserted as WIFEXITED && WEXITSTATUS == code: a shell reports 130 for a death by
// SIGINT too, and only the wait status tells a clean exit from a lost handler (A.3).

#include <gmock/gmock.h>

#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "driver_defaults.hpp"
#include "fake_servo_bus.hpp"
#include "servo_bus.hpp"
#include "test_support.hpp"

namespace
{

// Per-name declarations, never a using-directive (cpplint build/namespaces).
using ::testing::Contains;
using ::testing::ContainsRegex;
using ::testing::HasSubstr;
using ::testing::IsEmpty;
using ::testing::MatchesRegex;
using ::testing::Not;
using ::testing::StartsWith;
using waveshare_servos::OpenResult;
using waveshare_servos::ServoBus;
using waveshare_servos_test::EepromPolicy;
using waveshare_servos_test::FakeBus;
using waveshare_servos_test::FakeServo;
using waveshare_servos_test::FrameRecord;
using waveshare_servos_test::WriteRecord;
using waveshare_servos_test::descriptors_on;
using waveshare_servos_test::kBroadcastId;
using waveshare_servos_test::kRegId;
using waveshare_servos_test::kRegLock;
using waveshare_servos_test::kRegMode;
using waveshare_servos_test::kRegPresentPosition;
using waveshare_servos_test::kRegTorqueEnable;

using Clock = std::chrono::steady_clock;

constexpr const char * kDefaultPort = waveshare_servos::defaults::kPort;
constexpr int kBaudrate = 1000000;
constexpr uint32_t kIoTimeoutMs = 5;              // io_timeout_ms_for(1000000), the tools' own
constexpr auto kDeadline = std::chrono::seconds(30);

// One of the four executables: its name (the "<tool>: " prefix of its stderr lines and the word
// in its usage line) and the ENV variable that carries its built path.
struct ToolBinary
{
  const char * name;
  const char * variable;
};

constexpr ToolBinary kScan{"scan", "WAVESHARE_TOOL_SCAN"};
constexpr ToolBinary kSetId{"set_id", "WAVESHARE_TOOL_SET_ID"};
constexpr ToolBinary kCalibrate{"calibrate_midpoint", "WAVESHARE_TOOL_CALIBRATE_MIDPOINT"};
constexpr ToolBinary kFactoryReset{"factory_reset", "WAVESHARE_TOOL_FACTORY_RESET"};
constexpr ToolBinary kAllTools[] = {kScan, kSetId, kCalibrate, kFactoryReset};

// scan's stdout contract (C.1), the header the HIL parser matches exactly.
constexpr const char * kHeader =
  " id  type  mode  model  baud_reg     baud  position  voltage_V  temp_C  status  offset";

std::string read_file(const std::string & path)
{
  std::ifstream in(path);
  std::stringstream text;
  text << in.rdbuf();
  return text.str();
}

std::vector<std::string> lines_of(const std::string & text)
{
  std::vector<std::string> lines;
  std::istringstream in(text);
  for (std::string line; std::getline(in, line); ) {
    lines.push_back(line);
  }
  return lines;
}

// The stderr lines a tool wrote without its "<tool>: " prefix. The vendored begin() printf()s
// "serial speed N" itself, past every stream the tool owns, so that line is the one exception.
std::vector<std::string> unprefixed(const std::string & err, const ToolBinary & tool)
{
  std::vector<std::string> lines;
  for (const std::string & line : lines_of(err)) {
    if (line.rfind(std::string(tool.name) + ": ", 0) != 0 && line != "serial speed 1000000") {
      lines.push_back(line);
    }
  }
  return lines;
}

// The path of a built tool. A missing variable or file is a FAILURE, never a skip: a case must not
// pass because its tool never ran. The spawn then fails too, and the exit assertion says so.
std::string binary_of(const ToolBinary & tool)
{
  const char * path = std::getenv(tool.variable);
  EXPECT_NE(path, nullptr) << tool.variable << " is not set: run this test through ctest";
  if (path == nullptr) {
    return "";
  }
  EXPECT_EQ(::access(path, X_OK), 0) << tool.variable << "=" << path << " is not an executable";
  return path;
}

// Holds the default port for the whole suite (D.5): opened, flocked and made exclusive, never a
// byte sent. Every outcome it can end in is named, and only the ones that leave a child refused
// for the WHOLE suite count as safe: absent, held, and refused (EACCES) -- a permission this user
// lacks stays lacking. A port another process holds is NOT safe: this guard would hold nothing,
// and the other holder may let go mid-suite (review fixes F4, F16, F19). So an open refused with
// EBUSY, or a flock another holder has, is retried every 100 ms for up to `busy_wait`, and a port
// still held after that is fatal. `port` and `busy_wait` are parameters only so the guard's own
// cases can drive it on a pty; the suite uses the default port and 30 s.
class DefaultPortGuard
{
public:
  explicit DefaultPortGuard(
    const std::string & port = kDefaultPort,
    std::chrono::milliseconds busy_wait = std::chrono::seconds(30))
  {
    if (!std::filesystem::exists(port)) {
      state_ = "absent";              // nothing to protect, and nothing a child could reach
      return;
    }
    port_exists_ = true;
    if (::geteuid() == 0) {
      state_ = "root";                // root ignores TIOCEXCL: a child could open it regardless
      return;
    }
    const Clock::time_point started = Clock::now();
    while (true) {
      const std::string busy = take(port);
      if (busy.empty()) {
        break;                        // taken, refused for good, or failed: state_ says which
      }
      waited_ms_ = ms(Clock::now() - started);
      if (Clock::now() - started >= busy_wait) {
        state_ = "held by another process (" + busy + " for " + std::to_string(waited_ms_) +
          " ms); stop it before running this suite";
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    waited_ms_ = ms(Clock::now() - started);
  }

  ~DefaultPortGuard() {drop();}

  DefaultPortGuard(const DefaultPortGuard &) = delete;
  DefaultPortGuard & operator=(const DefaultPortGuard &) = delete;
  DefaultPortGuard(DefaultPortGuard &&) = delete;
  DefaultPortGuard & operator=(DefaultPortGuard &&) = delete;

  bool safe() const
  {
    return state_ == "absent" || state_ == "held" || state_ == "refused (EACCES)";
  }
  bool port_exists() const {return port_exists_;}
  int open_errno() const {return open_errno_;}
  const std::string & state() const {return state_;}
  int64_t waited_ms() const {return waited_ms_;}

private:
  static int64_t ms(Clock::duration duration)
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
  }

  // One attempt: open, flock, TIOCEXCL. Returns what another holder did ("EBUSY", "flocked") when
  // the attempt should be retried, else "" with state_ set.
  std::string take(const std::string & port)
  {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ == -1) {
      open_errno_ = errno;
      if (open_errno_ == EBUSY) {
        return "EBUSY";               // someone holds it with TIOCEXCL, for now
      }
      state_ = open_errno_ == EACCES ? std::string("refused (EACCES)") :
        std::string("open failed: ") + std::strerror(open_errno_);
      return "";
    }
    open_errno_ = 0;
    if (::flock(fd_, LOCK_EX | LOCK_NB) == -1) {
      const int error = errno;
      drop();
      if (error == EWOULDBLOCK) {
        return "flocked";             // a flock-only holder: the port stays openable meanwhile
      }
      state_ = std::string("flock failed: ") + std::strerror(error);
      return "";
    }
    if (::ioctl(fd_, TIOCEXCL) == -1) {
      state_ = std::string("TIOCEXCL failed: ") + std::strerror(errno);
      return "";
    }
    state_ = "held";
    return "";
  }

  void drop()
  {
    if (fd_ != -1) {
      ::ioctl(fd_, TIOCNXCL);
      ::flock(fd_, LOCK_UN);
      ::close(fd_);
      fd_ = -1;
    }
  }

  int fd_ = -1;
  bool port_exists_ = false;
  int open_errno_ = 0;
  int64_t waited_ms_ = 0;
  std::string state_;
};

// What a spawned tool did.
struct ToolRun
{
  bool spawned = false;
  bool finished = false;              // it exited (or died) before the deadline
  int status = 0;                     // from waitpid
  std::string out;
  std::string err;
  Clock::time_point ended{};
};

// "exit 64", "signal 6 (Aborted)", "not spawned": one string, so that a case asserts
// WIFEXITED(status) && WEXITSTATUS(status) == code in one comparison whose failure prints what
// happened instead -- a death by SIGINT is never mistaken for the 130 a shell would show for it.
std::string how(const ToolRun & run)
{
  if (!run.spawned) {
    return "not spawned";
  }
  if (!run.finished) {
    return "still running after 30 s (killed)";
  }
  if (WIFEXITED(run.status)) {
    return "exit " + std::to_string(WEXITSTATUS(run.status));
  }
  if (WIFSIGNALED(run.status)) {
    return "signal " + std::to_string(WTERMSIG(run.status)) + " (" +
           ::strsignal(WTERMSIG(run.status)) + ")";
  }
  return "wait status " + std::to_string(run.status);
}

std::string exited(int code) {return "exit " + std::to_string(code);}

// Milliseconds, so a failed timing assertion prints a number rather than a byte dump.
int64_t ms(Clock::duration duration)
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

// A scratch directory per case for the tools' output files, removed however the case ends.
class TempDir
{
public:
  TempDir()
  {
    std::string pattern =
      (std::filesystem::temp_directory_path() / "test_tools_cli_XXXXXX").string();
    if (::mkdtemp(pattern.data()) != nullptr) {
      path_ = pattern;
    }
  }

  ~TempDir()
  {
    std::error_code ignored;
    if (!path_.empty()) {
      std::filesystem::remove_all(path_, ignored);
    }
  }

  TempDir(const TempDir &) = delete;
  TempDir & operator=(const TempDir &) = delete;
  TempDir(TempDir &&) = delete;
  TempDir & operator=(TempDir &&) = delete;

  bool ok() const {return !path_.empty();}
  std::string file(const std::string & name) const {return path_ + "/" + name;}

private:
  std::string path_;
};

class ToolsCli : public ::testing::Test
{
protected:
  // No tool is spawned without the guard: a fatal failure here makes gtest skip every case of the
  // suite and fail the binary (0.2.9).
  static void SetUpTestSuite()
  {
    guard_ = std::make_unique<DefaultPortGuard>();
    ASSERT_TRUE(guard_->safe()) <<
      "DefaultPortGuard on " << kDefaultPort << ": " << guard_->state() <<
      " -- cannot prove a child cannot reach the bench, so no tool is spawned";
  }

  static void TearDownTestSuite() {guard_.reset();}

  static const DefaultPortGuard & guard() {return *guard_;}

  void SetUp() override
  {
    ASSERT_TRUE(guard_ != nullptr && guard_->safe()) << "no tool is spawned without the guard";
    ASSERT_TRUE(scratch_.ok());
    fake_.set_eeprom_policy(EepromPolicy::volatile_when_locked);
    for (const uint8_t id : {11, 12, 13, 14}) {
      seed(id);
    }
  }

  void TearDown() override
  {
    fake_.wait_quiet();
    // A request the fake could not verify means the wire itself misbehaved and the case proved
    // nothing; no tool ever broadcasts.
    EXPECT_EQ(fake_.bad_checksums(), 0u) << "the wire itself misbehaved";
    EXPECT_EQ(fake_.quiet_timeouts(), 0u) << "wait_quiet gave up";
    for (const FrameRecord & frame : fake_.frames()) {
      EXPECT_NE(frame.id, kBroadcastId) << "no tool ever broadcasts";
    }
  }

  // A servo as the bench delivers it (E.0) -- firmware 3.6, model word 9 3 (777), baud register 0,
  // response level 1, limits 0 and 4095, torque on, the EEPROM lock closed -- at an id of the
  // fake's 11-14, positions 11-12 and wheels 13-14, each with its own offset (bit 11), position,
  // voltage and temperature, so a column read from the wrong servo cannot pass for the right one.
  void seed(uint8_t id)
  {
    const int k = id - 10;
    const int offsets[] = {0, 5, -6, 7, -1};
    fake_.add_servo(id, id <= 12 ? 0 : 1);
    fake_.set_byte(id, 0, 3);
    fake_.set_byte(id, 1, 6);
    fake_.set_byte(id, 3, 9);
    fake_.set_byte(id, 4, 3);
    fake_.set_byte(id, 6, 0);
    fake_.set_byte(id, 8, 1);
    fake_.set_word(id, 9, 0);
    fake_.set_word(id, 11, 4095);
    fake_.set_word(id, 31, offsets[k], 11);
    fake_.set_position(id, 1000 + 13 * id);
    fake_.set_byte(id, 62, static_cast<uint8_t>(120 + k));
    fake_.set_byte(id, 63, static_cast<uint8_t>(30 + k));
    fake_.set_byte(id, 40, 1);
    fake_.set_byte(id, kRegLock, 1);
  }

  // The servo keyed at `id`, or nullopt: snapshot() throws for an id nobody is keyed at, and a
  // case that expects a move must fail on an assertion when it did not happen.
  std::optional<FakeServo> servo_at(uint8_t id) const
  {
    try {
      return fake_.snapshot(id);
    } catch (const std::out_of_range &) {
      return std::nullopt;
    }
  }

  // Spawns `args` (args[0] the binary) with stdout, and stderr unless `stderr_fd` is given, into
  // files -- never pipes, which a chatty child could fill and block on. The child gets an empty
  // signal mask and default dispositions, so it cannot inherit an immunity to the very signal a
  // case sends it. waitpid(WNOHANG) every 10 ms, `while_running` in between; after 30 s, SIGKILL
  // and a failure.
  ToolRun spawn(
    const std::vector<std::string> & args,
    const std::function<void(pid_t)> & while_running = nullptr, int stderr_fd = -1)
  {
    const std::string stem = scratch_.file("run" + std::to_string(runs_++));
    const std::string out_path = stem + ".out";
    const std::string err_path = stem + ".err";
    std::vector<char *> argv;
    for (const std::string & arg : args) {
      argv.push_back(const_cast<char *>(arg.c_str()));
    }
    argv.push_back(nullptr);

    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_addopen(
      &actions, STDOUT_FILENO, out_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (stderr_fd == -1) {
      posix_spawn_file_actions_addopen(
        &actions, STDERR_FILENO, err_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    } else {
      posix_spawn_file_actions_adddup2(&actions, stderr_fd, STDERR_FILENO);
    }
    posix_spawnattr_t attributes;
    posix_spawnattr_init(&attributes);
    sigset_t none;
    sigemptyset(&none);
    sigset_t defaults;
    sigemptyset(&defaults);
    for (const int signal_number : {SIGINT, SIGTERM, SIGHUP, SIGQUIT, SIGPIPE}) {
      sigaddset(&defaults, signal_number);
    }
    posix_spawnattr_setsigmask(&attributes, &none);
    posix_spawnattr_setsigdefault(&attributes, &defaults);
    posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETSIGMASK | POSIX_SPAWN_SETSIGDEF);

    ToolRun run;
    pid_t pid = -1;
    run.spawned = !args.empty() && !args[0].empty() &&
      ::posix_spawn(&pid, argv[0], &actions, &attributes, argv.data(), environ) == 0;
    posix_spawnattr_destroy(&attributes);
    posix_spawn_file_actions_destroy(&actions);
    if (!run.spawned) {
      return run;
    }

    const Clock::time_point deadline = Clock::now() + kDeadline;
    while (true) {
      if (::waitpid(pid, &run.status, WNOHANG) == pid) {
        run.finished = true;
        break;
      }
      if (Clock::now() >= deadline) {
        ::kill(pid, SIGKILL);
        ::waitpid(pid, &run.status, 0);
        ADD_FAILURE() << args[0] << " did not exit within 30 s; killed";
        break;
      }
      if (while_running) {
        while_running(pid);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    run.ended = Clock::now();
    run.out = read_file(out_path);
    run.err = stderr_fd == -1 ? read_file(err_path) : std::string();
    return run;
  }

  // `tool --ros-args -p <param> -p <param> ...`
  ToolRun run_tool(
    const ToolBinary & tool, const std::vector<std::string> & params,
    const std::function<void(pid_t)> & while_running = nullptr, int stderr_fd = -1)
  {
    std::vector<std::string> args = {binary_of(tool), "--ros-args"};
    for (const std::string & param : params) {
      args.push_back("-p");
      args.push_back(param);
    }
    return spawn(args, while_running, stderr_fd);
  }

  // `port:=<the fake's pty>`, which every case passes unless it is about a stale name.
  std::string port_param() const {return "port:=" + fake_.port();}

  // The ids a tool is given when a case must not depend on them: 200 and 201, which answer on no
  // bus this test builds, so a regression that got past the refusal under test still writes to
  // nobody (R16).
  static std::vector<std::string> absent_ids(const ToolBinary & tool)
  {
    if (std::string(tool.name) == kSetId.name) {
      return {"start_id:=200", "new_id:=201"};
    }
    if (std::string(tool.name) == kCalibrate.name || std::string(tool.name) == kFactoryReset.name) {
      return {"id:=200"};
    }
    return {};
  }

  static std::vector<std::string> joined(
    std::vector<std::string> first, const std::vector<std::string> & second)
  {
    first.insert(first.end(), second.begin(), second.end());
    return first;
  }

  // Sends `signal_number` to the child once `ready()` holds, and records when: the stimulus of the
  // signal cases, taken from the fake's log rather than from a guess at the child's timing.
  struct SignalOnce
  {
    int signal_number;
    std::function<bool()> ready;
    std::optional<Clock::time_point> sent;

    void operator()(pid_t pid)
    {
      if (!sent.has_value() && ready()) {
        ::kill(pid, signal_number);
        sent = Clock::now();
      }
    }
  };

  // An in-process bus takes the pty and lets it go again: whatever the child did to the tty's
  // exclusive flag and its advisory lock was undone when it exited. Around it, this process holds
  // exactly one descriptor on the pty -- the fake's own slave.
  void expect_port_retakeable(const std::string & after)
  {
    SCOPED_TRACE("after " + after);
    EXPECT_EQ(descriptors_on(fake_.port()), 1u) << "only the fake's own slave";
    ServoBus probe;
    const OpenResult opened = probe.open(fake_.port(), kBaudrate, kIoTimeoutMs);
    EXPECT_TRUE(static_cast<bool>(opened)) << to_string(opened.status) << " errno " <<
      opened.error;
    probe.close();
    EXPECT_EQ(descriptors_on(fake_.port()), 1u) << "only the fake's own slave";
  }

  static std::map<uint8_t, int> scan_pings_with_found(const std::vector<uint8_t> & found)
  {
    std::map<uint8_t, int> pings;
    for (int id = 0; id <= 253; id++) {
      pings[static_cast<uint8_t>(id)] = waveshare_servos::defaults::kPingAttempts;
    }
    for (const uint8_t id : found) {
      pings[id] = 1;
    }
    return pings;
  }

  // The whole write list of set_id 14 -> 253: verified unlock, the id write, verified lock at the
  // new id.
  static std::vector<WriteRecord> move_14_to_253()
  {
    return {WriteRecord{14, 55, {0}}, WriteRecord{14, 5, {253}}, WriteRecord{253, 55, {1}}};
  }

  static inline std::unique_ptr<DefaultPortGuard> guard_;

  TempDir scratch_;
  FakeBus fake_;
  int runs_ = 0;
};

}  // namespace

// ---- the guard itself ----

TEST_F(ToolsCli, default_port_guard_blocks_a_second_open)
{
  // H step 8.2 runs this case alone before the red run, whose old binaries ignore `port`.
  RecordProperty("default_port_guard", guard().state());
  RecordProperty("default_port_guard_waited_ms", std::to_string(guard().waited_ms()));
  std::cout << "DefaultPortGuard on " << kDefaultPort << ": " << guard().state() << " (waited " <<
    guard().waited_ms() << " ms for another holder)" << std::endl;
  if (!guard().port_exists()) {
    GTEST_SKIP() << kDefaultPort << " does not exist: there is nothing to protect";
  }
  const int fd = ::open(kDefaultPort, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  const int error = errno;
  if (fd != -1) {
    ::close(fd);                      // opened and closed, never a byte sent
  }
  ASSERT_EQ(fd, -1) << "a second open of " << kDefaultPort << " succeeded: a child could reach it";
  if (guard().open_errno() == EACCES) {
    EXPECT_EQ(error, EACCES) << std::strerror(error);
  } else {
    EXPECT_EQ(error, EBUSY) << std::strerror(error);
  }
}

// ---- the three tools, end to end ----

TEST_F(ToolsCli, scan_prints_the_fake_bus_and_exits_0)
{
  fake_.clear_frames();
  const ToolRun run = run_tool(kScan, {port_param()});
  EXPECT_EQ(how(run), exited(0)) << run.err;
  const std::vector<std::string> lines = lines_of(run.out);
  ASSERT_EQ(lines.size(), 6u) << run.out;
  EXPECT_EQ(lines[0], kHeader);
  EXPECT_EQ(
    lines[1],
    " 11  pos      0    777         0  1000000      1143       12.1      31    0x00       5");
  EXPECT_EQ(
    lines[2],
    " 12  pos      0    777         0  1000000      1156       12.2      32    0x00      -6");
  EXPECT_EQ(
    lines[3],
    " 13  vel      1    777         0  1000000      1169       12.3      33    0x00       7");
  EXPECT_EQ(
    lines[4],
    " 14  vel      1    777         0  1000000      1182       12.4      34    0x00      -1");
  EXPECT_THAT(
    lines[5], MatchesRegex(
      "found 4 servo\\(s\\) on " + fake_.port() + " at 1000000 baud: ids 11 12 13 14 \\(pinged "
      "ids 0\\.\\.253, 3 attempts each, [0-9]+\\.[0-9] s\\)"));
  // The vendored line goes to stderr, never to the table's stream (R19).
  EXPECT_THAT(lines_of(run.err), Contains("serial speed 1000000"));
  EXPECT_THAT(run.err, HasSubstr("scan: use the id column as <param name=\"id\">"));
  EXPECT_THAT(unprefixed(run.err, kScan), IsEmpty()) << run.err;
  fake_.wait_quiet();
  // The coverage gate, from the fake's log: every id 0..253 [Q5], a found one no more than once.
  EXPECT_EQ(fake_.ping_counts(), scan_pings_with_found({11, 12, 13, 14}));
  EXPECT_TRUE(fake_.writes().empty());
}

TEST_F(ToolsCli, scan_of_an_empty_bus_exits_3)
{
  for (const uint8_t id : {11, 12, 13, 14}) {
    fake_.set_absent(id, true);
  }
  fake_.clear_frames();
  const ToolRun run = run_tool(kScan, {port_param()});
  EXPECT_EQ(how(run), exited(3)) << run.err;
  const std::vector<std::string> lines = lines_of(run.out);
  ASSERT_EQ(lines.size(), 2u) << run.out;
  EXPECT_EQ(lines[0], kHeader);
  EXPECT_THAT(
    lines[1], MatchesRegex(
      "found no servo on " + fake_.port() + " at 1000000 baud \\(pinged ids 0\\.\\.253, 3 "
      "attempts each, [0-9]+\\.[0-9] s\\)"));
  EXPECT_THAT(run.err, HasSubstr("scan: check servo power (USB does not power the servos)"));
  fake_.wait_quiet();
  EXPECT_EQ(fake_.ping_counts(), scan_pings_with_found({}));
}

TEST_F(ToolsCli, set_id_moves_14_to_253_exit_0)
{
  fake_.clear_frames();
  const ToolRun run = run_tool(kSetId, {port_param(), "start_id:=14", "new_id:=253"});
  EXPECT_EQ(how(run), exited(0)) << run.err;
  fake_.wait_quiet();
  EXPECT_EQ(fake_.writes(), move_14_to_253());
  EXPECT_FALSE(servo_at(14).has_value()) << "nothing answers at 14 any more";
  const std::optional<FakeServo> moved = servo_at(253);
  ASSERT_TRUE(moved.has_value());
  EXPECT_EQ(moved->mem[kRegId], 253);
  EXPECT_EQ(moved->mem[kRegLock], 1);
  EXPECT_EQ(moved->mem[kRegMode], 1) << "the wheel is still a wheel";
  EXPECT_EQ(
    run.out,
    "servo 14 is now id 253: it answers at 253 and no longer at 14, its registers are otherwise "
    "unchanged, and its EEPROM lock is closed. The id is stored in EEPROM; power-cycle the servo "
    "and run scan to confirm it kept id 253, then update <param name=\"id\"> in your URDF.\n");
  EXPECT_THAT(run.err, HasSubstr("set_id: about to give servo 14 the id 253."));
  EXPECT_THAT(
    run.err, ContainsRegex(
      "set_id: detail start_id=14 new_id=253 lock_before=1 unlock_read=0 .* writes_sent=3 "
      "verdict=ok"));
  EXPECT_THAT(lines_of(run.err), Contains("serial speed 1000000"));
  EXPECT_THAT(unprefixed(run.err, kSetId), IsEmpty()) << run.err;
}

TEST_F(ToolsCli, set_id_refuses_a_taken_id_exit_4_and_writes_nothing)
{
  fake_.clear_frames();
  const ToolRun run = run_tool(kSetId, {port_param(), "start_id:=14", "new_id:=11"});
  EXPECT_EQ(how(run), exited(4)) << run.err;
  fake_.wait_quiet();
  EXPECT_TRUE(fake_.writes().empty());
  for (const FrameRecord & frame : fake_.frames()) {
    EXPECT_NE(frame.id, 14) << "nothing is sent to the start id once the new id is taken";
  }
  EXPECT_THAT(
    run.err, HasSubstr(
      "set_id: id 11 already answers on '" + fake_.port() + "'; refusing to give servo 14 an id "
      "that is taken"));
  EXPECT_THAT(run.err, ContainsRegex("set_id: detail start_id=14 new_id=11 .* verdict=refused"));
  EXPECT_EQ(run.out, "");
  EXPECT_TRUE(servo_at(14).has_value());
}

TEST_F(ToolsCli, calibrate_centres_12_exit_0)
{
  fake_.set_offset_model(12, true);
  fake_.set_position(12, 1026);
  fake_.clear_frames();
  const ToolRun run = run_tool(kCalibrate, {port_param(), "id:=12"});
  EXPECT_EQ(how(run), exited(0)) << run.err;
  fake_.wait_quiet();
  // torque off, verified unlock, 128, verified lock [Q1, Q2, Q3]
  EXPECT_EQ(
    fake_.writes(), (std::vector<WriteRecord>{WriteRecord{12, 40, {0}}, WriteRecord{12, 55, {0}},
      WriteRecord{12, 40, {128}}, WriteRecord{12, 55, {1}}}));
  EXPECT_EQ(fake_.word(12, kRegPresentPosition), 2048);
  EXPECT_THAT(
    run.out, StartsWith(
      "servo 12 now reads 2048 at the position that read 1026; offset registers 31-32 went from "
      "-6 to "));
  EXPECT_THAT(
    run.err, ContainsRegex(
      "calibrate_midpoint: detail id=12 mode=0 torque_before=1 torque_written=true .* "
      "position_after=2048 .* lock_after=1 writes_sent=4 verdict=ok"));
  EXPECT_THAT(unprefixed(run.err, kCalibrate), IsEmpty()) << run.err;
}

TEST_F(ToolsCli, calibrate_refuses_the_wheel_13_exit_4)
{
  fake_.clear_frames();
  const ToolRun run = run_tool(kCalibrate, {port_param(), "id:=13"});
  EXPECT_EQ(how(run), exited(4)) << run.err;
  fake_.wait_quiet();
  EXPECT_TRUE(fake_.writes().empty());
  EXPECT_THAT(
    run.err, HasSubstr(
      "calibrate_midpoint: servo 13 is in mode 1 (wheel); a midpoint only means something to a "
      "position servo (mode 0)."));
  const std::optional<FakeServo> wheel = servo_at(13);
  ASSERT_TRUE(wheel.has_value());
  EXPECT_EQ(wheel->mem[kRegMode], 1);
  EXPECT_EQ(run.out, "");
}

TEST_F(ToolsCli, factory_reset_resets_the_wheel_13_exit_0)
{
  // Its factory table is its EEPROM with the mode and the offset put back: torque off, the RESET,
  // and nothing else on the wire; the id stays 13.
  fake_.set_factory_from_eeprom(13, {{kRegMode, 0}, {31, 0}, {32, 0}});
  fake_.clear_frames();
  const ToolRun run = run_tool(kFactoryReset, {port_param(), "id:=13"});
  EXPECT_EQ(how(run), exited(0)) << run.err;
  fake_.wait_quiet();
  EXPECT_EQ(fake_.writes(), (std::vector<WriteRecord>{WriteRecord{13, 40, {0}}}));
  EXPECT_EQ(fake_.resets(), (std::vector<uint8_t>{13}));
  const std::optional<FakeServo> reset = servo_at(13);
  ASSERT_TRUE(reset.has_value());
  EXPECT_EQ(reset->mem[kRegMode], 0);
  EXPECT_EQ(reset->mem[kRegTorqueEnable], 0);
  EXPECT_THAT(
    run.out, StartsWith(
      "servo 13 is reset to its factory settings and keeps id 13. Registers changed: 31: 7 -> 0, "
      "33: 1 -> 0. "));
  EXPECT_THAT(
    run.err, HasSubstr(
      "factory_reset: about to reset servo 13 to its factory settings: every EEPROM register but "
      "its id"));
  EXPECT_THAT(
    run.err, ContainsRegex(
      "factory_reset: detail id=13 model=777 baud_reg_before=0 offset_raw_before=0x0007 "
      "mode_before=1 torque_before=1 torque_written=true lock_before=1 reset_ack=old_id .* "
      "changed_registers=2 torque_final=0 lock_after=1 writes_sent=2 verdict=ok"));
  EXPECT_THAT(unprefixed(run.err, kFactoryReset), IsEmpty()) << run.err;
}

TEST_F(ToolsCli, factory_reset_at_500000_follows_the_servo_to_1000000_exit_0)
{
  // M3 through the process: the child sends the RESET at the rate it was given and finds the servo
  // at the factory rate afterwards, without letting go of the port in between.
  fake_.set_factory_from_eeprom(14, {{kRegMode, 0}, {31, 0}, {32, 0}});
  fake_.set_byte(14, 6, 1);
  fake_.set_baud_model(true);
  fake_.clear_frames();
  const ToolRun run = run_tool(kFactoryReset, {port_param(), "baudrate:=500000", "id:=14"});
  EXPECT_EQ(how(run), exited(0)) << run.err;
  fake_.wait_quiet();
  EXPECT_EQ(fake_.resets(), (std::vector<uint8_t>{14}));
  EXPECT_THAT(
    run.out, HasSubstr(
      "It now talks at 1000000 baud, not 500000: give scan and the hardware interface baudrate "
      "1000000 for it. "));
  EXPECT_THAT(run.err, ContainsRegex("baud_reg_before=1 .* baudrate_after=1000000 .* verdict=ok"));
  ServoBus probe;
  ASSERT_TRUE(static_cast<bool>(probe.open(fake_.port(), kBaudrate, kIoTimeoutMs)));
  EXPECT_EQ(probe.checked_ping(14).kind, waveshare_servos::ReplyKind::ONE) << "at the factory rate";
}

TEST_F(ToolsCli, factory_reset_of_a_silent_id_exits_3_and_sends_no_reset)
{
  fake_.clear_frames();
  const ToolRun run = run_tool(kFactoryReset, {port_param(), "id:=200"});
  EXPECT_EQ(how(run), exited(3)) << run.err;
  fake_.wait_quiet();
  EXPECT_TRUE(fake_.writes().empty());
  EXPECT_TRUE(fake_.resets().empty());
  EXPECT_THAT(run.err, ContainsRegex("factory_reset: detail id=200 .* verdict=no_answer"));
  EXPECT_EQ(run.out, "");
}

// ---- parameters: every refusal is exit 64, before the port is opened ----

TEST_F(ToolsCli, each_tool_exits_64_for_device_port_and_sends_nothing)
{
  // The old name, pointing at the fake: a tool that honoured it would talk to the fake, and one
  // that ignored it would fall back to the guarded default port and exit 1.
  for (const ToolBinary & tool : kAllTools) {
    SCOPED_TRACE(tool.name);
    fake_.clear_frames();
    const ToolRun run = run_tool(tool, joined({"device_port:=" + fake_.port()}, absent_ids(tool)));
    EXPECT_EQ(how(run), exited(64)) << run.err;
    EXPECT_THAT(
      run.err, HasSubstr(
        std::string(tool.name) + ": parameter 'device_port' was renamed to 'port'"));
    EXPECT_THAT(run.err, HasSubstr(std::string("usage: ros2 run waveshare_servos ") + tool.name));
    EXPECT_THAT(run.err + run.out, Not(HasSubstr("serial speed")));
    fake_.wait_quiet();
    EXPECT_EQ(fake_.bytes_received(), 0u);
  }
}

TEST_F(ToolsCli, each_tool_exits_64_for_baud_rate)
{
  for (const ToolBinary & tool : kAllTools) {
    SCOPED_TRACE(tool.name);
    fake_.clear_frames();
    const ToolRun run =
      run_tool(tool, joined({port_param(), "baud_rate:=115200"}, absent_ids(tool)));
    EXPECT_EQ(how(run), exited(64)) << run.err;
    EXPECT_THAT(
      run.err, HasSubstr(
        std::string(tool.name) + ": parameter 'baud_rate' was renamed to 'baudrate'"));
    fake_.wait_quiet();
    EXPECT_EQ(fake_.bytes_received(), 0u);
  }
}

TEST_F(ToolsCli, each_tool_exits_64_for_an_override_addressed_to_another_node)
{
  // rclcpp drops both of these for a node of another name without a word, and the tool would
  // then run at the default rate; rcl still has them, which is where the refusal comes from.
  const std::string params_file = scratch_.file("other_node.yaml");
  {
    std::ofstream yaml(params_file);
    yaml << "/other_node:\n  ros__parameters:\n    baudrate: 9600\n";
  }
  for (const ToolBinary & tool : kAllTools) {
    SCOPED_TRACE(tool.name);
    fake_.clear_frames();
    const ToolRun prefixed =
      run_tool(tool, joined({port_param(), "other_node:baudrate:=9600"}, absent_ids(tool)));
    EXPECT_EQ(how(prefixed), exited(64)) << prefixed.err;
    EXPECT_THAT(prefixed.err, HasSubstr(std::string(tool.name) + ": parameter override(s)"));
    EXPECT_THAT(prefixed.err, HasSubstr("are addressed to node 'other_node'"));

    std::vector<std::string> args = {binary_of(tool), "--ros-args", "-p", port_param()};
    for (const std::string & id : absent_ids(tool)) {
      args.push_back("-p");
      args.push_back(id);
    }
    args.push_back("--params-file");
    args.push_back(params_file);
    const ToolRun from_file = spawn(args);
    EXPECT_EQ(how(from_file), exited(64)) << from_file.err;
    EXPECT_THAT(from_file.err, HasSubstr("are addressed to node '/other_node'"));
    fake_.wait_quiet();
    EXPECT_EQ(fake_.bytes_received(), 0u);
  }
}

TEST_F(ToolsCli, set_id_and_calibrate_without_ids_exit_64_and_send_nothing)
{
  // [Q4] The bare invocation, the only place it is run (R16): it names no port either, so the
  // guard is what a regression would meet; the second run names the fake so the wire can witness
  // that nothing was sent. factory_reset takes calibrate's `id`, required the same way.
  for (const ToolBinary & tool : {kSetId, kCalibrate, kFactoryReset}) {
    SCOPED_TRACE(tool.name);
    const ToolRun bare = spawn({binary_of(tool)});
    EXPECT_EQ(how(bare), exited(64)) << bare.err;
    if (std::string(tool.name) == kSetId.name) {
      EXPECT_THAT(bare.err, HasSubstr("set_id: parameter 'new_id' is missing"));
      EXPECT_THAT(bare.err, HasSubstr("set_id: parameter 'start_id' is missing"));
    } else {
      EXPECT_THAT(
        bare.err, HasSubstr(
          std::string(tool.name) + ": parameter 'id' is missing; expected an integer between 0 "
          "and 253"));
    }
    EXPECT_THAT(bare.err, HasSubstr(std::string("usage: ros2 run waveshare_servos ") + tool.name));
    EXPECT_THAT(bare.err + bare.out, Not(HasSubstr("serial speed")));

    fake_.clear_frames();
    const ToolRun port_only = run_tool(tool, {port_param()});
    EXPECT_EQ(how(port_only), exited(64)) << port_only.err;
    fake_.wait_quiet();
    EXPECT_EQ(fake_.bytes_received(), 0u);
  }
}

TEST_F(ToolsCli, a_wrongly_typed_parameter_is_64_not_134)
{
  // declare_parameter<int> would throw on a double, and an uncaught throw is SIGABRT (134).
  fake_.clear_frames();
  const ToolRun set_id = run_tool(kSetId, {port_param(), "start_id:=200", "new_id:=201.0"});
  EXPECT_EQ(how(set_id), exited(64)) << set_id.err;
  EXPECT_THAT(
    set_id.err,
    HasSubstr("set_id: parameter 'new_id' is 201.0 (a double), which is not an integer"));
  const ToolRun calibrate = run_tool(kCalibrate, {port_param(), "id:=200.0"});
  EXPECT_EQ(how(calibrate), exited(64)) << calibrate.err;
  EXPECT_THAT(
    calibrate.err,
    HasSubstr("calibrate_midpoint: parameter 'id' is 200.0 (a double), which is not an integer"));
  fake_.wait_quiet();
  EXPECT_EQ(fake_.bytes_received(), 0u);
}

TEST_F(ToolsCli, malformed_ros_args_is_64)
{
  // `-p port` with no `:=` makes rclcpp::init throw; uncaught, that is SIGABRT. rcl logs its own
  // "Failed to parse global arguments" line first, so the tool's line is not the only one.
  for (const ToolBinary & tool : kAllTools) {
    SCOPED_TRACE(tool.name);
    const ToolRun run = spawn({binary_of(tool), "--ros-args", "-p", "port"});
    EXPECT_EQ(how(run), exited(64)) << run.err;
    EXPECT_THAT(lines_of(run.err), Contains(StartsWith(std::string(tool.name) + ": ")));
    EXPECT_THAT(run.err, HasSubstr(std::string("usage: ros2 run waveshare_servos ") + tool.name));
    EXPECT_THAT(run.err + run.out, Not(HasSubstr("serial speed")));
  }
}

TEST_F(ToolsCli, a_positional_argument_is_64)
{
  // A port given without --ros-args is not a parameter at all; ignored, the tool would open the
  // default port instead.
  for (const ToolBinary & tool : kAllTools) {
    SCOPED_TRACE(tool.name);
    fake_.clear_frames();
    const ToolRun run = spawn({binary_of(tool), fake_.port()});
    EXPECT_EQ(how(run), exited(64)) << run.err;
    EXPECT_THAT(
      run.err, HasSubstr(
        std::string(tool.name) + ": unexpected argument '" + fake_.port() + "'; parameters go "
        "after --ros-args, for example: ros2 run waveshare_servos " + tool.name +
        " --ros-args -p port:=" + fake_.port()));
    fake_.wait_quiet();
    EXPECT_EQ(fake_.bytes_received(), 0u);
  }
}

// ---- the port: taken through ServoBus, refused while held, let go on every exit ----

TEST_F(ToolsCli, each_tool_is_refused_by_a_bus_this_process_holds_exit_1)
{
  // This process holds the pty the way the driver does (lock and TIOCEXCL). The child leaves its
  // own pid out of the holder list, so the pid it names is this one. Close-on-exec is NOT what
  // this shows -- an inherited descriptor would carry the child's pid, which the list drops;
  // FakeBusTools.descriptors_are_close_on_exec is that proof.
  ServoBus holder;
  const OpenResult opened = holder.open(fake_.port(), kBaudrate, kIoTimeoutMs);
  ASSERT_TRUE(static_cast<bool>(opened)) << to_string(opened.status);
  const std::string pid = std::to_string(::getpid());
  for (const ToolBinary & tool : kAllTools) {
    SCOPED_TRACE(tool.name);
    fake_.clear_frames();
    const ToolRun run = run_tool(tool, joined({port_param()}, absent_ids(tool)));
    EXPECT_EQ(how(run), exited(1)) << run.err;
    EXPECT_THAT(
      run.err, HasSubstr(
        std::string(tool.name) + ": port '" + fake_.port() + "' is held by another process ("));
    EXPECT_THAT(run.err, ContainsRegex("pid " + pid + "[ ,)]"));
    EXPECT_THAT(run.err + run.out, Not(HasSubstr("serial speed")));
    fake_.wait_quiet();
    EXPECT_EQ(fake_.bytes_received(), 0u);
  }
}

TEST_F(ToolsCli, each_tool_is_refused_by_a_flock_only_holder_before_begin)
{
  // The advisory lock alone, no TIOCEXCL: the open succeeds, so only a tool that takes the lock
  // is refused -- and it must be refused BEFORE begin(), which is what prints "serial speed". A
  // tool that called SMS_STS::begin itself would open the port and print the line (G.3, item 4).
  const int fd = ::open(fake_.port().c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  ASSERT_NE(fd, -1) << std::strerror(errno);
  ASSERT_EQ(::flock(fd, LOCK_EX | LOCK_NB), 0) << std::strerror(errno);
  const std::string pid = std::to_string(::getpid());
  for (const ToolBinary & tool : kAllTools) {
    SCOPED_TRACE(tool.name);
    fake_.clear_frames();
    const ToolRun run = run_tool(tool, joined({port_param()}, absent_ids(tool)));
    EXPECT_EQ(how(run), exited(1)) << run.err;
    EXPECT_THAT(run.out, Not(HasSubstr("serial speed")));
    EXPECT_THAT(run.err, Not(HasSubstr("serial speed")));
    EXPECT_THAT(run.err, HasSubstr("is held by another process"));
    EXPECT_THAT(run.err, ContainsRegex("pid " + pid + "[ ,)]"));
    fake_.wait_quiet();
    EXPECT_EQ(fake_.bytes_received(), 0u);
  }
  ::flock(fd, LOCK_UN);
  ::close(fd);
}

TEST_F(ToolsCli, a_missing_port_exits_2)
{
  const std::string missing = "/dev/waveshare_servos_test_missing_" + std::to_string(::getpid());
  for (const ToolBinary & tool : kAllTools) {
    SCOPED_TRACE(tool.name);
    const ToolRun run = run_tool(tool, joined({"port:=" + missing}, absent_ids(tool)));
    EXPECT_EQ(how(run), exited(2)) << run.err;
    EXPECT_THAT(
      run.err, HasSubstr(
        std::string(tool.name) + ": port '" + missing + "' does not exist; 'ls /dev/ttyACM* "
        "/dev/ttyUSB*' lists what is plugged in"));
  }
}

// ---- signals ----

TEST_F(ToolsCli, sigint_during_scan_exits_130_and_releases_the_port)
{
  // Sent once the fake has seen a frame: the port is open and the scan is pinging, not still
  // starting up.
  SignalOnce sigint{SIGINT, [this] {return fake_.frames_received() > 0;}, std::nullopt};
  const ToolRun run = run_tool(kScan, {port_param()}, std::ref(sigint));
  ASSERT_TRUE(sigint.sent.has_value()) << "the scan never reached the wire: " << how(run);
  EXPECT_EQ(how(run), exited(130)) << run.err;
  EXPECT_LE(ms(run.ended - *sigint.sent), 1000) << "exit 130 within 1 s of the signal";
  const std::vector<std::string> lines = lines_of(run.out);
  ASSERT_FALSE(lines.empty());
  EXPECT_EQ(lines.front(), kHeader);
  EXPECT_THAT(lines.back(), HasSubstr(", interrupted after id "));
  expect_port_retakeable("SIGINT");
}

TEST_F(ToolsCli, sighup_during_scan_exits_130)
{
  // Without a handler of its own, SIGHUP -- an ssh session dropping -- would kill the tool.
  SignalOnce sighup{SIGHUP, [this] {return fake_.frames_received() > 0;}, std::nullopt};
  const ToolRun run = run_tool(kScan, {port_param()}, std::ref(sighup));
  ASSERT_TRUE(sighup.sent.has_value()) << "the scan never reached the wire: " << how(run);
  EXPECT_EQ(how(run), exited(130)) << run.err;
  EXPECT_LE(ms(run.ended - *sighup.sent), 1000) << "exit 130 within 1 s of the signal";
  EXPECT_THAT(run.out, HasSubstr(", interrupted after id "));
}

TEST_F(ToolsCli, a_signal_during_the_eeprom_sequence_is_deferred)
{
  // A 300 ms commit keeps the sequence open well after the id write; the signal lands inside it.
  fake_.set_eeprom_commit_ms(14, 300);
  fake_.clear_frames();
  SignalOnce sighup{SIGHUP, [this] {return fake_.writes().size() >= 2;}, std::nullopt};
  const ToolRun run =
    run_tool(kSetId, {port_param(), "start_id:=14", "new_id:=253"}, std::ref(sighup));
  ASSERT_TRUE(sighup.sent.has_value()) << "the id write never went out: " << how(run);
  EXPECT_EQ(how(run), exited(0)) << run.err;
  fake_.wait_quiet();
  EXPECT_EQ(fake_.writes(), move_14_to_253()) << "the sequence ran to its verified lock";
  EXPECT_THAT(
    run.err, HasSubstr(
      "set_id: a signal arrived during the EEPROM sequence; it was completed first"));
  EXPECT_THAT(run.out, StartsWith("servo 14 is now id 253:"));
}

TEST_F(ToolsCli, a_signal_during_calibrates_settle_exits_130_before_the_unlock)
{
  // Review fix F7, end to end: SIGHUP once the torque-off write is out, while the tool waits for
  // the servo to settle. Nothing has been written to EEPROM, so the signal is acted on at the last
  // check before the unlock -- whichever thread took it, the flag or the pending set shows it.
  fake_.set_offset_model(12, true);
  fake_.set_position(12, 1026);
  fake_.clear_frames();
  SignalOnce sighup{SIGHUP, [this] {return fake_.writes().size() >= 1;}, std::nullopt};
  const ToolRun run = run_tool(kCalibrate, {port_param(), "id:=12"}, std::ref(sighup));
  ASSERT_TRUE(sighup.sent.has_value()) << "the torque write never went out: " << how(run);
  EXPECT_EQ(how(run), exited(130)) << run.err;
  fake_.wait_quiet();
  EXPECT_EQ(fake_.writes(), (std::vector<WriteRecord>{WriteRecord{12, 40, {0}}}));
  const std::optional<FakeServo> servo = servo_at(12);
  ASSERT_TRUE(servo.has_value());
  EXPECT_EQ(servo->calibrations, 0);
  EXPECT_THAT(
    run.err, HasSubstr(
      "calibrate_midpoint: interrupted; its torque is now OFF; no EEPROM byte was written"));
  EXPECT_THAT(run.err, Not(HasSubstr("completed first")));
  EXPECT_THAT(run.err, ContainsRegex("calibrate_midpoint: detail id=12 .* verdict=interrupted"));
}

TEST_F(ToolsCli, a_closed_stderr_pipe_does_not_stop_set_id)
{
  // `set_id ... 2>&1 | head` after head has exited: every write to stderr is EPIPE. A tool that
  // did not ignore SIGPIPE would die on the pre-write notice, just before the first EEPROM write,
  // or anywhere after it.
  int pipe_fds[2] = {-1, -1};
  ASSERT_EQ(::pipe2(pipe_fds, O_CLOEXEC), 0) << std::strerror(errno);
  ::close(pipe_fds[0]);
  fake_.clear_frames();
  const ToolRun run =
    run_tool(kSetId, {port_param(), "start_id:=14", "new_id:=253"}, nullptr, pipe_fds[1]);
  ::close(pipe_fds[1]);
  EXPECT_EQ(how(run), exited(0));
  fake_.wait_quiet();
  EXPECT_EQ(fake_.writes(), move_14_to_253());
  EXPECT_THAT(run.out, StartsWith("servo 14 is now id 253:"));
}

TEST_F(ToolsCli, the_port_is_retakeable_after_every_exit)
{
  // exit 0: an EEPROM sequence from its unlock to its lock
  const ToolRun moved = run_tool(kSetId, {port_param(), "start_id:=14", "new_id:=253"});
  EXPECT_EQ(how(moved), exited(0)) << moved.err;
  expect_port_retakeable("exit 0");

  // exit 3: the addressed servo is silent
  const ToolRun silent = run_tool(kCalibrate, {port_param(), "id:=200"});
  EXPECT_EQ(how(silent), exited(3)) << silent.err;
  expect_port_retakeable("exit 3");

  // exit 4: the new id is taken
  const ToolRun taken = run_tool(kSetId, {port_param(), "start_id:=13", "new_id:=11"});
  EXPECT_EQ(how(taken), exited(4)) << taken.err;
  expect_port_retakeable("exit 4");

  // exit 7: a servo whose id register disagrees with the id it answers at
  fake_.set_byte(11, kRegId, 99);
  const ToolRun anomaly = run_tool(kScan, {port_param()});
  EXPECT_EQ(how(anomaly), exited(7)) << anomaly.err;
  expect_port_retakeable("exit 7");

  // exit 130: a signal in the middle of a scan
  SignalOnce sigint{SIGINT, [this] {return fake_.frames_received() > 0;}, std::nullopt};
  fake_.clear_frames();
  const ToolRun interrupted = run_tool(kScan, {port_param()}, std::ref(sigint));
  EXPECT_EQ(how(interrupted), exited(130)) << interrupted.err;
  expect_port_retakeable("exit 130");
}

// ---- DefaultPortGuard's own mechanics, on a pty (review fixes F4, F16, F19) ----
// A port somebody else holds is waited for and then held; one that stays held is fatal. It is
// never "safe" while this suite holds nothing, because the other holder may let go mid-suite.

namespace
{

// Another holder of `port`, the way the controller manager holds it (TIOCEXCL) or the way a
// flock-only process does; release() lets it go. In this process, on its own open file
// description, so it conflicts with the guard's exactly as another process's would.
class OtherHolder
{
public:
  OtherHolder(const std::string & port, bool exclusive)
  : exclusive_(exclusive)
  {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    held_ = fd_ != -1 && (exclusive ? ::ioctl(fd_, TIOCEXCL) == 0 : ::flock(fd_, LOCK_EX) == 0);
  }

  ~OtherHolder() {release();}

  OtherHolder(const OtherHolder &) = delete;
  OtherHolder & operator=(const OtherHolder &) = delete;
  OtherHolder(OtherHolder &&) = delete;
  OtherHolder & operator=(OtherHolder &&) = delete;

  bool held() const {return held_;}

  void release()
  {
    if (fd_ != -1) {
      if (exclusive_) {
        ::ioctl(fd_, TIOCNXCL);
      }
      ::close(fd_);
      fd_ = -1;
    }
  }

private:
  bool exclusive_;
  bool held_ = false;
  int fd_ = -1;
};

// A raw open of `port` fails, and with what errno (0 when it succeeded).
int second_open_errno(const std::string & port)
{
  const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  const int error = errno;
  if (fd != -1) {
    ::close(fd);                      // opened and closed, never a byte sent
    return 0;
  }
  return error;
}

}  // namespace

TEST(ToolsCliGuard, a_busy_port_is_waited_for_and_then_held)
{
  FakeBus fake;
  OtherHolder other(fake.port(), true);
  ASSERT_TRUE(other.held());
  ASSERT_EQ(second_open_errno(fake.port()), EBUSY) << "the pty honours TIOCEXCL";
  std::thread release([&other] {
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      other.release();
    });
  const DefaultPortGuard guard(fake.port(), std::chrono::seconds(10));
  release.join();
  EXPECT_EQ(guard.state(), "held");
  EXPECT_TRUE(guard.safe());
  EXPECT_GE(guard.waited_ms(), 200) << "it waited for the other holder";
  EXPECT_EQ(second_open_errno(fake.port()), EBUSY) << "and a child is now refused by the guard";
}

TEST(ToolsCliGuard, a_port_that_stays_busy_is_fatal_not_safe)
{
  FakeBus fake;
  OtherHolder other(fake.port(), true);
  ASSERT_TRUE(other.held());
  const DefaultPortGuard guard(fake.port(), std::chrono::milliseconds(300));
  EXPECT_FALSE(guard.safe()) << guard.state();
  EXPECT_THAT(guard.state(), HasSubstr("held by another process"));
}

TEST(ToolsCliGuard, a_flock_only_holder_is_waited_for_too)
{
  // A flock-only holder leaves the port openable: a child that took no lock would reach it.
  FakeBus fake;
  OtherHolder other(fake.port(), false);
  ASSERT_TRUE(other.held());
  std::thread release([&other] {
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      other.release();
    });
  const DefaultPortGuard guard(fake.port(), std::chrono::seconds(10));
  release.join();
  EXPECT_EQ(guard.state(), "held");
  EXPECT_TRUE(guard.safe());
  EXPECT_EQ(second_open_errno(fake.port()), EBUSY);
}
