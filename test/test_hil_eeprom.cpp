// Tests for test/hil/eeprom_core.{hpp,cpp} and the hil_eeprom binary (PHASE6_SPEC D.6, E.1).
//
// hil_eeprom is the oracle every "wrote nothing" and "restored" gate of the Phase 6 bench rests
// on, and the tool that repairs the bench when a scenario did write. A bug in it would either hide
// a tool's write or make one, so its logic runs here on the fake bus first: the vendored packet
// code unchanged over an openpty() pair, the fake's frame log for what went on the wire, and its
// EEPROM model (lock policy, power cycle, commit latency, id re-key) for what a servo keeps.
//
// Two cases spawn the built binary ($WAVESHARE_HIL_EEPROM_BIN) instead of calling the library,
// because what they pin belongs to the process: that compare opens no port at all, and that a
// SIGTERM in the middle of a restore waits until the EEPROM lock is closed again.

#include <gmock/gmock.h>

#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include "eeprom_core.hpp"
#include "fake_servo_bus.hpp"
#include "servo_bus.hpp"

namespace
{

// Per-name declarations, never a using-directive (cpplint build/namespaces).
using ::testing::HasSubstr;
using waveshare_servos::ServoBus;
using waveshare_servos::hil_eeprom::BlockcheckReport;
using waveshare_servos::hil_eeprom::Diff;
using waveshare_servos::hil_eeprom::DriftReport;
using waveshare_servos::hil_eeprom::ReadReport;
using waveshare_servos::hil_eeprom::RestoreOptions;
using waveshare_servos::hil_eeprom::RestoreReport;
using waveshare_servos::hil_eeprom::ServoRecord;
using waveshare_servos::hil_eeprom::Snapshot;
using waveshare_servos::hil_eeprom::blockcheck;
using waveshare_servos::hil_eeprom::census;
using waveshare_servos::hil_eeprom::compare;
using waveshare_servos::hil_eeprom::diff_text;
using waveshare_servos::hil_eeprom::drift;
using waveshare_servos::hil_eeprom::format_snap;
using waveshare_servos::hil_eeprom::kUnreadable;
using waveshare_servos::hil_eeprom::parse_snap;
using waveshare_servos::hil_eeprom::read_register;
using waveshare_servos::hil_eeprom::restore;
using waveshare_servos::hil_eeprom::run;
using waveshare_servos::hil_eeprom::snapshot_json;
using waveshare_servos::hil_eeprom::source_problems;
using waveshare_servos::hil_eeprom::take_snapshot;
using waveshare_servos_test::EepromPolicy;
using waveshare_servos_test::FakeBus;
using waveshare_servos_test::FrameRecord;
using waveshare_servos_test::WriteRecord;
using waveshare_servos_test::kInstPing;
using waveshare_servos_test::kInstRead;
using waveshare_servos_test::kRegGoalPosition;
using waveshare_servos_test::kRegGoalSpeed;
using waveshare_servos_test::kRegId;
using waveshare_servos_test::kRegLock;
using waveshare_servos_test::kRegMode;
using waveshare_servos_test::kRegOffset;
using waveshare_servos_test::kRegPresentPosition;
using waveshare_servos_test::kRegTorqueEnable;

constexpr int kBaudrate = 1000000;
constexpr uint32_t kIoTimeoutMs = 20;   // hil_eeprom's own (E.1)
// The port a compare that opened one by default would reach: defaults::kPort, spelled out
// because this target has only test/ and test/hil/ on its include path (B.5).
constexpr const char * kDefaultPort = "/dev/ttyACM0";

// The registers a snapshot holds, written out here rather than taken from eeprom_core, so no case
// asks the code under test which registers it should have read: EEPROM 0, 1, 3..39 (address 2 is
// not defined), then SRAM 40 and 55.
std::vector<int> snapshot_registers()
{
  std::vector<int> regs = {0, 1};
  for (int reg = 3; reg <= 39; reg++) {
    regs.push_back(reg);
  }
  regs.push_back(40);
  regs.push_back(55);
  return regs;
}

// A delivered servo's EEPROM with every byte different from its neighbours and from the other
// servos', so a byte read from the wrong address or from the wrong servo cannot pass for the right
// one. Bytes 0, 1, 3 and 4 (firmware and model) are the same on all four, as on the bench.
uint8_t seeded(int id, int reg)
{
  switch (reg) {
    case 0:
      return 3;
    case 1:
      return 6;
    case 3:
      return 9;
    case 4:
      return 3;
    default:
      return static_cast<uint8_t>(10 + 3 * reg + id);
  }
}

uint8_t low(int word) {return static_cast<uint8_t>(word & 0xff);}
uint8_t high(int word) {return static_cast<uint8_t>((word >> 8) & 0xff);}

std::string read_file(const std::string & path)
{
  std::ifstream in(path);
  std::stringstream text;
  text << in.rdbuf();
  return text.str();
}

void write_file(const std::string & path, const std::string & text)
{
  std::ofstream out(path, std::ios::trunc);
  out << text;
}

// A scratch directory per case, removed however the case ends.
class TempDir
{
public:
  TempDir()
  {
    std::string pattern =
      (std::filesystem::temp_directory_path() / "test_hil_eeprom_XXXXXX").string();
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

// The built binary, from the ENV of its ctest entry (CMakeLists.txt). A missing one is a failure,
// never a skip: the two process-level cases would otherwise pass by not running.
std::string hil_eeprom_bin()
{
  const char * bin = std::getenv("WAVESHARE_HIL_EEPROM_BIN");
  return bin == nullptr ? std::string() : std::string(bin);
}

// posix_spawn with stdout and stderr into one file (a file, not a pipe: nothing can block on a
// full pipe), with an empty signal mask and default dispositions, so the child cannot inherit
// immunity to the very signal a case sends it. -1 when the spawn failed.
pid_t spawn_to_file(const std::vector<std::string> & args, const std::string & out_path)
{
  std::vector<char *> argv;
  for (const std::string & arg : args) {
    argv.push_back(const_cast<char *>(arg.c_str()));
  }
  argv.push_back(nullptr);

  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_addopen(
    &actions, STDOUT_FILENO, out_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  posix_spawn_file_actions_adddup2(&actions, STDOUT_FILENO, STDERR_FILENO);
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

  pid_t pid = -1;
  const int rc = ::posix_spawn(&pid, argv[0], &actions, &attributes, argv.data(), environ);
  posix_spawnattr_destroy(&attributes);
  posix_spawn_file_actions_destroy(&actions);
  return rc == 0 ? pid : -1;
}

// waitpid(WNOHANG) every 10 ms until the child exits or `limit` passes; then SIGKILL, reap, and
// return false, so a hung child can never outlive the case.
bool wait_exit(pid_t pid, int * status, std::chrono::seconds limit)
{
  const auto deadline = std::chrono::steady_clock::now() + limit;
  while (std::chrono::steady_clock::now() < deadline) {
    if (::waitpid(pid, status, WNOHANG) == pid) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ::kill(pid, SIGKILL);
  ::waitpid(pid, status, 0);
  return false;
}

// Holds the default port the way test_tools_cli's DefaultPortGuard does (D.5): opened, flocked
// and made exclusive, and never a byte sent. A compare that opened any port by default would then
// be refused, which is what makes "compare opens no port" a claim a case can fail.
class HeldDefaultPort
{
public:
  HeldDefaultPort()
  {
    if (!std::filesystem::exists(kDefaultPort)) {
      state_ = "absent";      // nothing to protect, and nothing a child could reach
      return;
    }
    if (::geteuid() == 0) {
      state_ = "root";        // root ignores TIOCEXCL: a child could still open it
      return;
    }
    fd_ = ::open(kDefaultPort, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ == -1) {
      // somebody else holds it, or nobody here may open it: a child is refused as well
      state_ = (errno == EBUSY || errno == EACCES) ? "refused" : "open failed";
      return;
    }
    if (::flock(fd_, LOCK_EX | LOCK_NB) == -1) {
      state_ = (errno == EWOULDBLOCK) ? "flocked elsewhere" : "flock failed";
      return;
    }
    state_ = (::ioctl(fd_, TIOCEXCL) == 0) ? "held" : "TIOCEXCL failed";
  }

  ~HeldDefaultPort()
  {
    if (fd_ != -1) {
      ::ioctl(fd_, TIOCNXCL);
      ::flock(fd_, LOCK_UN);
      ::close(fd_);
    }
  }

  HeldDefaultPort(const HeldDefaultPort &) = delete;
  HeldDefaultPort & operator=(const HeldDefaultPort &) = delete;
  HeldDefaultPort(HeldDefaultPort &&) = delete;
  HeldDefaultPort & operator=(HeldDefaultPort &&) = delete;

  bool safe() const
  {
    return state_ == "absent" || state_ == "held" || state_ == "refused" ||
           state_ == "flocked elsewhere";
  }
  const std::string & state() const {return state_;}

private:
  int fd_ = -1;
  std::string state_;
};

// The block read of a firmware whose 37-byte READ disagrees with its single-byte reads, the risk
// E.0's blockcheck exists for (G.2 R4). Every 43-byte reply -- a 37-byte payload, and nothing else
// on this bus is that long -- has its register-13 byte changed and its checksum repaired, so the
// vendored Read accepts it. readSCS is virtual and protected, so a derived bus is the only way in.
struct DisagreeingBlockBus : ServoBus
{
protected:
  int readSCS(unsigned char * data, int length) override
  {
    const int got = ServoBus::readSCS(data, length);
    if (length == 43 && got == 43) {
      data[5 + (13 - 3)] ^= 0x40;
      uint8_t sum = 0;
      for (int i = 2; i < 42; i++) {
        sum = static_cast<uint8_t>(sum + data[i]);
      }
      data[42] = static_cast<uint8_t>(~sum);
    }
    return got;
  }
};

bool contains(const std::vector<WriteRecord> & writes, const WriteRecord & wanted)
{
  return std::find(writes.begin(), writes.end(), wanted) != writes.end();
}

std::string joined(const std::vector<std::string> & lines)
{
  std::string text;
  for (const std::string & line : lines) {
    text += line + "\n";
  }
  return text;
}

// The bench in miniature: ids 1-2 position servos, 3-4 wheels, every EEPROM byte seeded, each at
// its own position, and hil_eeprom's bus open on it at hil_eeprom's own io timeout.
class HilEeprom : public ::testing::Test
{
protected:
  void SetUp() override
  {
    for (int id = 1; id <= 4; id++) {
      const auto key = static_cast<uint8_t>(id);
      fake_.add_servo(key, id <= 2 ? 0 : 1);
      for (int reg = 0; reg <= 39; reg++) {
        if (reg != 2 && reg != kRegId && reg != kRegMode) {
          fake_.set_byte(key, static_cast<uint8_t>(reg), seeded(id, reg));
        }
      }
      fake_.set_position(key, 1000 + id);
    }
    ASSERT_TRUE(static_cast<bool>(bus_.open(fake_.port(), kBaudrate, kIoTimeoutMs)));
  }

  // A restore source: ids 1-4 as they are now, with the census the fake has. The census is filled
  // in rather than pinged for -- a census costs 2.5 s of absent-id timeouts, and the ping pass has
  // a case of its own (census_lists_exactly_the_answering_ids).
  Snapshot source_of_now()
  {
    bool interrupted = true;
    Snapshot snap = take_snapshot(bus_, {1, 2, 3, 4}, false, &no_stop_, &interrupted);
    snap.has_census = true;
    snap.census = {1, 2, 3, 4};
    return snap;
  }

  int offset_raw(int id) const
  {
    const auto servo = fake_.snapshot(static_cast<uint8_t>(id));
    return servo.mem[kRegOffset] | (servo.mem[kRegOffset + 1] << 8);
  }

  volatile std::sig_atomic_t no_stop_ = 0;
  FakeBus fake_;
  ServoBus bus_;   // after fake_, so it closes before the pty goes away
};

}  // namespace

TEST_F(HilEeprom, snapshot_reads_every_eeprom_byte_and_fails_if_one_is_unreadable)
{
  fake_.set_byte(2, kRegTorqueEnable, 1);
  fake_.set_byte(2, kRegLock, 1);
  fake_.clear_frames();
  bool interrupted = true;
  const Snapshot snap = take_snapshot(bus_, {1, 2}, false, &no_stop_, &interrupted);
  EXPECT_FALSE(interrupted);
  ASSERT_TRUE(snap.ok);
  ASSERT_EQ(snap.servos.size(), 2u);
  for (const int id : {1, 2}) {
    const ServoRecord & servo = snap.servos.at(id);
    EXPECT_EQ(servo.readable_eeprom(), 39) << "id " << id;
    const auto truth = fake_.snapshot(static_cast<uint8_t>(id));
    for (const int reg : snapshot_registers()) {
      EXPECT_EQ(servo.reg(reg), truth.mem[reg]) << "id " << id << " reg " << reg;
    }
  }
  // singly: one READ of one byte at every address, none at the undefined address 2, no write
  std::map<int, int> single_reads;
  for (const FrameRecord & frame : fake_.frames()) {
    if (frame.id == 2 && frame.instruction == kInstRead && frame.params.size() == 2 &&
      frame.params[1] == 1)
    {
      single_reads[frame.params[0]]++;
    }
  }
  for (const int reg : snapshot_registers()) {
    EXPECT_EQ(single_reads[reg], 1) << "reg " << reg;
  }
  EXPECT_EQ(single_reads.count(2), 0u);
  EXPECT_TRUE(fake_.writes().empty());

  // one unreadable byte: the snapshot is not ok, the byte is `x` -- never 0 -- in the .snap and
  // in the JSON, and `n` counts only the bytes that were read
  fake_.set_silent_read(2, 13);
  const Snapshot holed = take_snapshot(bus_, {1, 2}, false, &no_stop_, &interrupted);
  EXPECT_FALSE(holed.ok);
  ASSERT_EQ(holed.servos.count(2), 1u);
  EXPECT_EQ(holed.servos.at(2).reg(13), kUnreadable);
  EXPECT_EQ(holed.servos.at(2).readable_eeprom(), 38);
  EXPECT_EQ(holed.servos.at(1).readable_eeprom(), 39);

  std::vector<std::string> tokens;
  std::istringstream lines(format_snap(holed));
  for (std::string line; std::getline(lines, line); ) {
    if (line.rfind("servo 2 eeprom ", 0) == 0) {
      std::istringstream words(line);
      for (std::string word; words >> word; ) {
        tokens.push_back(word);
      }
    }
  }
  ASSERT_EQ(tokens.size(), 3u + 40u) << "servo 2's eeprom line: 40 values after 'servo 2 eeprom'";
  EXPECT_EQ(tokens[3 + 13], "x");
  EXPECT_EQ(tokens[3 + 2], "-");
  EXPECT_EQ(tokens[3 + 12], std::to_string(seeded(2, 12)));
  const std::string json = snapshot_json(holed);
  EXPECT_THAT(json, HasSubstr("\"ok\": false"));
  EXPECT_THAT(json, HasSubstr("\"n\": 38"));
  EXPECT_THAT(json, HasSubstr("\"13\": \"x\""));
}

TEST_F(HilEeprom, compare_equal_is_0_and_one_byte_difference_in_any_register_is_1)
{
  const Snapshot a = source_of_now();
  ASSERT_TRUE(a.ok);
  // through the file format, as the binary sees it
  Snapshot parsed;
  std::string error;
  ASSERT_TRUE(parse_snap(format_snap(a), &parsed, &error)) << error;
  EXPECT_TRUE(compare(a, parsed, false).empty());

  const TempDir dir;
  ASSERT_TRUE(dir.ok());
  write_file(dir.file("a.snap"), format_snap(a));
  write_file(dir.file("b.snap"), format_snap(parsed));
  std::ostringstream equal_out;
  EXPECT_EQ(run({"compare", dir.file("a.snap"), dir.file("b.snap")}, equal_out, &no_stop_), 0)
    << equal_out.str();
  EXPECT_THAT(equal_out.str(), HasSubstr("RESULT {\"equal\": true"));

  for (const int reg : snapshot_registers()) {
    Snapshot b = parsed;
    const int was = a.servos.at(3).reg(reg);
    const int now = (was + 1) % 256;
    b.servos.at(3).regs[reg] = now;
    const std::vector<Diff> diffs = compare(a, b, false);
    ASSERT_EQ(diffs.size(), 1u) << "reg " << reg;
    EXPECT_EQ(diffs[0].id, 3);
    EXPECT_EQ(diffs[0].reg, reg);
    EXPECT_EQ(
      diff_text(diffs[0]),
      "id 3 reg " + std::to_string(reg) + ": " + std::to_string(was) + " -> " +
      std::to_string(now));
    // --eeprom-only drops 40 and 55, and nothing else
    EXPECT_EQ(compare(a, b, true).size(), reg >= 40 ? 0u : 1u) << "reg " << reg;

    write_file(dir.file("b.snap"), format_snap(b));
    std::ostringstream out;
    EXPECT_EQ(run({"compare", dir.file("a.snap"), dir.file("b.snap")}, out, &no_stop_), 1)
      << "reg " << reg << "\n" << out.str();
    EXPECT_THAT(out.str(), HasSubstr("id 3 reg " + std::to_string(reg) + ": "));
    EXPECT_THAT(out.str(), HasSubstr("RESULT {\"equal\": false"));
  }
}

TEST_F(HilEeprom, compare_treats_unreadable_as_unequal)
{
  const Snapshot a = source_of_now();
  ASSERT_TRUE(a.ok);
  // an `x` against a value, either way round
  Snapshot b = a;
  b.servos.at(2).regs[13] = kUnreadable;
  ASSERT_EQ(compare(a, b, false).size(), 1u);
  EXPECT_EQ(diff_text(compare(a, b, false)[0]), "id 2 reg 13: " + std::to_string(seeded(2, 13)) +
    " -> x");
  EXPECT_EQ(compare(b, a, false).size(), 1u);
  // and an `x` against an `x`: two unknowns are not known to be equal
  const std::vector<Diff> both = compare(b, b, false);
  ASSERT_EQ(both.size(), 1u);
  EXPECT_EQ(diff_text(both[0]), "id 2 reg 13: x -> x");
  // SRAM too, and --eeprom-only does not excuse an unreadable EEPROM byte
  Snapshot c = a;
  c.servos.at(1).regs[55] = kUnreadable;
  EXPECT_EQ(compare(c, c, false).size(), 1u);
  EXPECT_EQ(compare(b, b, true).size(), 1u);
}

TEST_F(HilEeprom, compare_counts_a_missing_id_as_a_difference)
{
  const Snapshot a = source_of_now();
  ASSERT_TRUE(a.ok);
  Snapshot b = a;
  b.servos.erase(4);
  const std::vector<Diff> missing_in_b = compare(a, b, false);
  ASSERT_EQ(missing_in_b.size(), 1u);
  EXPECT_EQ(diff_text(missing_in_b[0]), "id 4: missing in B");
  const std::vector<Diff> missing_in_a = compare(b, a, true);
  ASSERT_EQ(missing_in_a.size(), 1u);
  EXPECT_EQ(diff_text(missing_in_a[0]), "id 4: missing in A");
  // a census that differs is a difference of its own
  Snapshot c = a;
  c.census = {1, 2, 3, 4, 253};
  const std::vector<Diff> census_diff = compare(a, c, true);
  ASSERT_EQ(census_diff.size(), 1u);
  EXPECT_TRUE(census_diff[0].census);
  EXPECT_EQ(diff_text(census_diff[0]), "census: 1 2 3 4 -> 1 2 3 4 253");
}

TEST_F(HilEeprom, compare_opens_no_port)
{
  const std::string bin = hil_eeprom_bin();
  ASSERT_FALSE(bin.empty()) << "WAVESHARE_HIL_EEPROM_BIN is not set (CMakeLists.txt ENV)";
  ASSERT_EQ(::access(bin.c_str(), X_OK), 0) << bin;
  const Snapshot a = source_of_now();
  ASSERT_TRUE(a.ok);
  Snapshot b = a;
  b.servos.at(1).regs[31] = (a.servos.at(1).reg(31) + 1) % 256;
  const TempDir dir;
  ASSERT_TRUE(dir.ok());
  write_file(dir.file("a.snap"), format_snap(a));
  write_file(dir.file("b.snap"), format_snap(b));

  const HeldDefaultPort held;
  ASSERT_TRUE(held.safe()) << "cannot hold " << kDefaultPort << ": " << held.state();
  RecordProperty("default_port", held.state());

  int status = 0;
  const pid_t equal = spawn_to_file(
    {bin, "compare", dir.file("a.snap"), dir.file("a.snap")}, dir.file("equal.txt"));
  ASSERT_GT(equal, 0);
  ASSERT_TRUE(wait_exit(equal, &status, std::chrono::seconds(30)));
  const std::string equal_out = read_file(dir.file("equal.txt"));
  ASSERT_TRUE(WIFEXITED(status)) << equal_out;
  EXPECT_EQ(WEXITSTATUS(status), 0) << equal_out;
  EXPECT_THAT(equal_out, HasSubstr("RESULT {\"equal\": true"));
  EXPECT_THAT(equal_out, ::testing::Not(HasSubstr("serial speed")));

  const pid_t differ = spawn_to_file(
    {bin, "compare", dir.file("a.snap"), dir.file("b.snap")}, dir.file("differ.txt"));
  ASSERT_GT(differ, 0);
  ASSERT_TRUE(wait_exit(differ, &status, std::chrono::seconds(30)));
  const std::string differ_out = read_file(dir.file("differ.txt"));
  ASSERT_TRUE(WIFEXITED(status)) << differ_out;
  EXPECT_EQ(WEXITSTATUS(status), 1) << differ_out;
  EXPECT_THAT(differ_out, HasSubstr("id 1 reg 31: "));
}

TEST_F(HilEeprom, restore_refuses_a_source_with_unreadable_bytes)
{
  const Snapshot good = source_of_now();
  ASSERT_TRUE(good.ok);
  ASSERT_TRUE(source_problems(good).empty()) << joined(source_problems(good));

  Snapshot not_ok = good;
  not_ok.ok = false;
  Snapshot with_x = good;             // ok left true, so only the `x` itself can refuse it
  with_x.servos.at(2).regs[13] = kUnreadable;
  Snapshot unlisted = good;           // id 4 listed and in the census, but no servo line
  unlisted.servos.erase(4);
  Snapshot census_off = good;         // a census that is not the servo lines
  census_off.census = {1, 2, 3, 4, 200};
  const std::map<std::string, Snapshot> sources = {
    {"ok false", not_ok}, {"an x", with_x}, {"a listed id with no servo line", unlisted},
    {"a census that differs from its servo lines", census_off}};

  for (const auto & entry : sources) {
    fake_.clear_frames();
    const RestoreReport report = restore(bus_, entry.second, RestoreOptions{}, &no_stop_);
    EXPECT_EQ(report.exit, 3) << entry.first;
    EXPECT_FALSE(report.problems.empty()) << entry.first;
    EXPECT_FALSE(source_problems(entry.second).empty()) << entry.first;
    EXPECT_TRUE(report.writes.empty()) << entry.first;
    // refused before a single byte went out: not even the census
    EXPECT_EQ(fake_.bytes_received(), 0u) << entry.first;
  }
}

TEST_F(HilEeprom, restore_of_an_unchanged_bench_sends_zero_eeprom_writes)
{
  fake_.set_byte(1, kRegTorqueEnable, 1);
  fake_.set_byte(2, kRegLock, 1);
  const Snapshot source = source_of_now();
  ASSERT_TRUE(source.ok);
  fake_.clear_frames();
  const RestoreReport report = restore(bus_, source, RestoreOptions{}, &no_stop_);
  EXPECT_EQ(report.exit, 0) << joined(report.problems);
  EXPECT_TRUE(report.before.empty());
  EXPECT_TRUE(report.after.empty());
  EXPECT_TRUE(report.writes.empty());
  EXPECT_TRUE(fake_.writes().empty());
  EXPECT_EQ(report.moved_from, -1);
  // and it did look: a census over every id 0..253 before, and again in the closing snapshot
  const std::map<uint8_t, int> pings = fake_.ping_counts();
  EXPECT_EQ(pings.size(), 254u);
  EXPECT_EQ(pings.count(254), 0u);
}

TEST_F(HilEeprom, restore_moves_a_stray_id_back_when_exactly_one_is_missing_and_one_unexpected)
{
  const Snapshot source = source_of_now();
  ASSERT_TRUE(source.ok);
  // the stray: servo 4 left at 253, as an interrupted set_id round trip would leave it
  ASSERT_EQ(bus_.writeByte(4, kRegId, 253), 1);
  ASSERT_EQ(bus_.Ping(253), 253);
  fake_.clear_frames();

  const RestoreReport report = restore(bus_, source, RestoreOptions{}, &no_stop_);
  EXPECT_EQ(report.exit, 0) << joined(report.problems);
  EXPECT_EQ(report.moved_from, 253);
  EXPECT_EQ(report.moved_to, 4);
  // verified unlock at 253, the id write, the verified lock where the servo now answers, and the
  // lock back to the source's 0 last
  EXPECT_EQ(
    fake_.writes(), (std::vector<WriteRecord>{
    WriteRecord{253, kRegLock, {0}}, WriteRecord{253, kRegId, {4}}, WriteRecord{4, kRegLock, {1}},
    WriteRecord{4, kRegLock, {0}}}));
  EXPECT_EQ(bus_.Ping(4), 4);
  EXPECT_EQ(bus_.Ping(253), -1);
  EXPECT_EQ(fake_.snapshot(4).eeprom[kRegId], 4);
  EXPECT_TRUE(report.after.empty());
}

TEST_F(HilEeprom, restore_refuses_to_guess_with_two_missing_or_two_unexpected)
{
  const Snapshot source = source_of_now();
  ASSERT_TRUE(source.ok);

  // two missing (3 gone, 4 moved) and one unexpected (253): which of 3 and 4 is 253?
  fake_.set_absent(3, true);
  ASSERT_EQ(bus_.writeByte(4, kRegId, 253), 1);
  fake_.clear_frames();
  const RestoreReport two_missing = restore(bus_, source, RestoreOptions{}, &no_stop_);
  EXPECT_EQ(two_missing.exit, 3);
  EXPECT_THAT(joined(two_missing.problems), HasSubstr("ambiguous; resolve by hand with scan"));
  EXPECT_TRUE(two_missing.writes.empty());
  EXPECT_TRUE(fake_.writes().empty());

  // one missing (4) and two unexpected (200, 253): which of them is 4?
  fake_.set_absent(3, false);
  fake_.add_servo(200, 1);
  fake_.clear_frames();
  const RestoreReport two_unexpected = restore(bus_, source, RestoreOptions{}, &no_stop_);
  EXPECT_EQ(two_unexpected.exit, 3);
  EXPECT_THAT(joined(two_unexpected.problems), HasSubstr("ambiguous; resolve by hand with scan"));
  EXPECT_TRUE(two_unexpected.writes.empty());
  EXPECT_TRUE(fake_.writes().empty());
}

TEST_F(HilEeprom, restore_refuses_when_read_only_bytes_0_1_3_4_differ)
{
  const Snapshot source = source_of_now();
  ASSERT_TRUE(source.ok);
  // one read-only byte different on each servo: every one of them is a different model
  const std::map<int, int> changed = {{1, 0}, {2, 1}, {3, 3}, {4, 4}};
  for (const auto & entry : changed) {
    fake_.set_byte(
      static_cast<uint8_t>(entry.first), static_cast<uint8_t>(entry.second),
      static_cast<uint8_t>(seeded(entry.first, entry.second) + 1));
  }
  // and a writable difference, so a refusal cannot be mistaken for "nothing to do"
  fake_.set_byte(2, kRegOffset, static_cast<uint8_t>(seeded(2, kRegOffset) + 1));
  fake_.clear_frames();

  const RestoreReport report = restore(bus_, source, RestoreOptions{}, &no_stop_);
  EXPECT_EQ(report.exit, 3);
  const std::string problems = joined(report.problems);
  for (const auto & entry : changed) {
    EXPECT_THAT(
      problems,
      HasSubstr("id " + std::to_string(entry.first) + " reg " + std::to_string(entry.second)));
  }
  EXPECT_THAT(problems, HasSubstr("different servo model"));
  EXPECT_TRUE(report.writes.empty());
  EXPECT_TRUE(fake_.writes().empty());
}

TEST_F(HilEeprom, restore_with_allow_regs_refuses_a_difference_elsewhere_with_zero_writes)
{
  const Snapshot source = source_of_now();
  ASSERT_TRUE(source.ok);
  RestoreOptions phase6_regs;
  phase6_regs.limit_regs = true;
  phase6_regs.allow_regs = {5, 31, 32, 33, 40, 55};

  // 31 is in the list and 13 is not: nothing is written, and both differences are reported
  fake_.set_byte(2, kRegOffset, static_cast<uint8_t>(seeded(2, kRegOffset) + 1));
  fake_.set_byte(2, 13, static_cast<uint8_t>(seeded(2, 13) + 1));
  fake_.clear_frames();
  const RestoreReport refused = restore(bus_, source, phase6_regs, &no_stop_);
  EXPECT_EQ(refused.exit, 3);
  EXPECT_THAT(joined(refused.problems), HasSubstr("id 2 reg 13"));
  std::set<int> reported;
  for (const Diff & diff : refused.before) {
    reported.insert(diff.reg);
  }
  EXPECT_EQ(reported, (std::set<int>{13, kRegOffset}));
  EXPECT_TRUE(refused.writes.empty());
  EXPECT_TRUE(fake_.writes().empty());

  // the control: with 13 as it was, the same list lets the offset through
  fake_.set_byte(2, 13, seeded(2, 13));
  fake_.clear_frames();
  const RestoreReport allowed = restore(bus_, source, phase6_regs, &no_stop_);
  EXPECT_EQ(allowed.exit, 0) << joined(allowed.problems);
  EXPECT_TRUE(contains(fake_.writes(), WriteRecord{2, kRegOffset, {seeded(2, 31), seeded(2, 32)}}));
}

TEST_F(
  HilEeprom,
  restore_rewrites_offset_and_mode_inside_a_verified_unlock_lock_with_torque_off_then_55_and_40)
{
  fake_.set_byte(2, kRegTorqueEnable, 1);
  const Snapshot source = source_of_now();
  ASSERT_TRUE(source.ok);
  // what a calibration and a mode switch leave behind: another offset, mode 1, torque still on --
  // with the offset model on, so rewriting 31-32 moves the frame the present position is read in,
  // as on the bench (README NOTE: the calibration moved it by 1022 ticks). The goal must be read
  // AFTER that rewrite; one read before it would drive the arm a quarter turn at torque-on (E.1
  // step 5; review fix F13). The seed keeps present and moves the model's shaft instead.
  fake_.set_offset_model(2, true);
  fake_.set_byte(2, kRegOffset, 0x12);
  fake_.set_byte(2, kRegOffset + 1, 0x04);
  fake_.set_byte(2, kRegMode, 1);
  fake_.clear_frames();
  const int before = fake_.word(2, kRegPresentPosition);

  const RestoreReport report = restore(bus_, source, RestoreOptions{}, &no_stop_);
  EXPECT_EQ(report.exit, 0) << joined(report.problems);
  const int present = fake_.word(2, kRegPresentPosition);
  EXPECT_NE(present, before) << "the offset rewrite did not move the frame: this proves nothing";
  EXPECT_EQ(
    fake_.writes(), (std::vector<WriteRecord>{
    WriteRecord{2, kRegTorqueEnable, {0}},
    WriteRecord{2, kRegLock, {0}},
    WriteRecord{2, kRegOffset, {seeded(2, 31), seeded(2, 32)}},
    WriteRecord{2, kRegMode, {0}},
    WriteRecord{2, kRegLock, {1}},
    WriteRecord{2, kRegLock, {0}},
    WriteRecord{2, kRegGoalPosition, {low(present), high(present)}},
    WriteRecord{2, kRegTorqueEnable, {1}}}));
  EXPECT_EQ(offset_raw(2), seeded(2, 31) | (seeded(2, 32) << 8));
  EXPECT_EQ(fake_.snapshot(2).eeprom[kRegMode], 0);
  EXPECT_TRUE(report.after.empty());
  // RESULT kinds: 31 and 33 are EEPROM, 40, 42 and 55 SRAM
  int eeprom_writes = 0;
  for (const auto & write : report.writes) {
    eeprom_writes += write.eeprom ? 1 : 0;
    EXPECT_EQ(write.eeprom, write.reg <= 39) << "reg " << write.reg;
  }
  EXPECT_EQ(eeprom_writes, 2);
}

TEST_F(HilEeprom, restore_sets_the_goal_to_the_present_position_before_enabling_torque)
{
  fake_.set_byte(1, kRegTorqueEnable, 1);
  fake_.set_byte(3, kRegTorqueEnable, 1);
  const Snapshot source = source_of_now();
  ASSERT_TRUE(source.ok);
  // torque off since, a stale goal behind it, and the arm moved by hand meanwhile
  fake_.set_byte(1, kRegTorqueEnable, 0);
  fake_.set_word(1, kRegGoalPosition, 700);
  fake_.set_position(1, 1500);
  fake_.set_byte(3, kRegTorqueEnable, 0);
  fake_.set_word(3, kRegGoalSpeed, 300);
  fake_.clear_frames();

  const RestoreReport report = restore(bus_, source, RestoreOptions{}, &no_stop_);
  EXPECT_EQ(report.exit, 0) << joined(report.problems);
  EXPECT_EQ(
    fake_.writes(), (std::vector<WriteRecord>{
    WriteRecord{1, kRegGoalPosition, {low(1500), high(1500)}},
    WriteRecord{1, kRegTorqueEnable, {1}},
    WriteRecord{3, kRegGoalSpeed, {0, 0}},
    WriteRecord{3, kRegTorqueEnable, {1}}}));
  EXPECT_EQ(fake_.word(1, kRegGoalPosition), 1500);
  EXPECT_EQ(fake_.word(3, kRegGoalSpeed), 0);
}

TEST_F(HilEeprom, restore_writes_word_registers_with_one_write)
{
  const Snapshot source = source_of_now();
  ASSERT_TRUE(source.ok);
  // half of a word changed: the low byte of the offset, the high byte of the minimum angle
  fake_.set_byte(2, kRegOffset, static_cast<uint8_t>(seeded(2, 31) ^ 0x01));
  fake_.set_byte(1, 10, static_cast<uint8_t>(seeded(1, 10) ^ 0x01));
  fake_.clear_frames();

  const RestoreReport report = restore(bus_, source, RestoreOptions{}, &no_stop_);
  EXPECT_EQ(report.exit, 0) << joined(report.problems);
  EXPECT_EQ(
    fake_.writes(), (std::vector<WriteRecord>{
    WriteRecord{1, kRegLock, {0}},
    WriteRecord{1, 9, {seeded(1, 9), seeded(1, 10)}},
    WriteRecord{1, kRegLock, {1}},
    WriteRecord{2, kRegLock, {0}},
    WriteRecord{2, kRegOffset, {seeded(2, 31), seeded(2, 32)}},
    WriteRecord{2, kRegLock, {1}},
    WriteRecord{1, kRegLock, {0}},
    WriteRecord{2, kRegLock, {0}}}));
}

TEST_F(HilEeprom, restore_persists_under_volatile_when_locked_power_cycle)
{
  // the memory table's lock: a write made while 55 reads 1 is applied and lost at power-off
  fake_.set_eeprom_policy(EepromPolicy::volatile_when_locked);
  fake_.set_power_up_lock(1);
  fake_.set_byte(2, kRegLock, 1);
  const Snapshot source = source_of_now();
  ASSERT_TRUE(source.ok);
  fake_.set_byte(2, kRegOffset, static_cast<uint8_t>(seeded(2, 31) + 1));
  fake_.set_byte(2, kRegMode, 1);

  const RestoreReport report = restore(bus_, source, RestoreOptions{}, &no_stop_);
  EXPECT_EQ(report.exit, 0) << joined(report.problems);
  fake_.power_cycle();
  EXPECT_EQ(fake_.snapshot(2).mem[kRegOffset], seeded(2, 31)) << "the offset did not persist";
  EXPECT_EQ(fake_.snapshot(2).mem[kRegMode], 0) << "the mode did not persist";
}

TEST_F(HilEeprom, census_lists_exactly_the_answering_ids)
{
  fake_.add_servo(0, 0);
  fake_.add_servo(253, 1);
  fake_.set_absent(3, true);
  fake_.drop_pings(2, 1);   // a ping lost on the wire: the second attempt still finds it
  fake_.clear_frames();
  bool interrupted = true;
  EXPECT_EQ(census(bus_, &no_stop_, &interrupted), (std::vector<int>{0, 1, 2, 4, 253}));
  EXPECT_FALSE(interrupted);

  // every id 0..253, two pings for a silent one and one for a servo that answered the first
  std::map<uint8_t, int> expected;
  for (int id = 0; id <= 253; id++) {
    expected[static_cast<uint8_t>(id)] = 2;
  }
  for (const int id : {0, 1, 4, 253}) {
    expected[static_cast<uint8_t>(id)] = 1;
  }
  EXPECT_EQ(fake_.ping_counts(), expected);
  for (const FrameRecord & frame : fake_.frames()) {
    EXPECT_EQ(frame.instruction, kInstPing);
  }
  EXPECT_EQ(bus_.io_timeout_ms(), kIoTimeoutMs) << "the census must put the io timeout back";
}

TEST_F(HilEeprom, blockcheck_compares_one_block_read_with_single_reads)
{
  fake_.clear_frames();
  const BlockcheckReport healthy = blockcheck(bus_, {1, 2, 3, 4}, 3, 37, &no_stop_);
  EXPECT_EQ(healthy.exit, 0);
  EXPECT_TRUE(healthy.ok);
  EXPECT_TRUE(healthy.mismatches.empty());
  EXPECT_TRUE(healthy.unreadable.empty());
  // one 37-byte block and 37 single reads per servo, nothing else
  std::map<std::pair<int, int>, int> reads;
  for (const FrameRecord & frame : fake_.frames()) {
    ASSERT_EQ(frame.instruction, kInstRead);
    if (frame.id == 2) {
      reads[{frame.params[0], frame.params[1]}]++;
    }
  }
  std::map<std::pair<int, int>, int> expected = {{{3, 37}, 1}};
  for (int reg = 3; reg <= 39; reg++) {
    expected[{reg, 1}] = 1;
  }
  EXPECT_EQ(reads, expected);

  // unreadable: the block and the single read at 3 both go silent
  fake_.set_silent_read(2, 3);
  const BlockcheckReport silent = blockcheck(bus_, {1, 2, 3, 4}, 3, 37, &no_stop_);
  EXPECT_EQ(silent.exit, 3);
  EXPECT_FALSE(silent.ok);
  EXPECT_FALSE(silent.unreadable.empty());
  fake_.set_silent_read(2, -1);

  // a block that disagrees with the single reads
  bus_.close();
  DisagreeingBlockBus disagreeing;
  ASSERT_TRUE(static_cast<bool>(disagreeing.open(fake_.port(), kBaudrate, kIoTimeoutMs)));
  const BlockcheckReport differs = blockcheck(disagreeing, {1, 2}, 3, 37, &no_stop_);
  EXPECT_EQ(differs.exit, 1);
  EXPECT_FALSE(differs.ok);
  ASSERT_EQ(differs.mismatches.size(), 2u);
  for (const auto & mismatch : differs.mismatches) {
    EXPECT_EQ(mismatch.reg, 13);
    EXPECT_EQ(mismatch.single, seeded(mismatch.id, 13));
    EXPECT_EQ(mismatch.block, seeded(mismatch.id, 13) ^ 0x40);
  }
}

TEST_F(HilEeprom, drift_samples_with_torque_off_and_restores_torque_with_the_goal_at_present)
{
  fake_.set_byte(2, kRegTorqueEnable, 1);
  fake_.set_word(2, kRegGoalPosition, 700);
  fake_.clear_frames();
  const DriftReport still = drift(bus_, 2, 0.5, &no_stop_);
  EXPECT_EQ(still.exit, 0) << still.problem;
  EXPECT_TRUE(still.ok);
  EXPECT_EQ(still.torque_before, 1);
  EXPECT_TRUE(still.torque_written);
  EXPECT_EQ(still.goal_before, 700);
  EXPECT_GE(still.samples.size(), 10u);
  for (const int sample : still.samples) {
    EXPECT_EQ(sample, 1002);
  }
  EXPECT_EQ(still.drift_total_ticks, 0);
  EXPECT_EQ(still.drift_last_1s_ticks, 0);
  EXPECT_TRUE(still.restored);
  // SRAM only, torque off before the samples and back on after them, the goal at present first
  const std::vector<WriteRecord> expected = {
    WriteRecord{2, kRegTorqueEnable, {0}},
    WriteRecord{2, kRegGoalPosition, {low(1002), high(1002)}},
    WriteRecord{2, kRegTorqueEnable, {1}}};
  EXPECT_EQ(fake_.writes(), expected);
  std::size_t written = 0;
  std::size_t samples_in_between = 0;
  for (const FrameRecord & frame : fake_.frames()) {
    const bool position_read =
      frame.instruction == kInstRead && frame.params[0] == kRegPresentPosition;
    written += frame.instruction == waveshare_servos_test::kInstWrite ? 1 : 0;
    samples_in_between += (written == 1 && position_read) ? 1 : 0;
  }
  EXPECT_GE(samples_in_between, still.samples.size());
  EXPECT_EQ(fake_.snapshot(2).mem[kRegTorqueEnable], 1);
  EXPECT_EQ(fake_.word(2, kRegGoalPosition), 1002);

  // a sag early on and nothing after it: the total sees it, the last second does not
  std::thread sag([this]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      fake_.set_position(2, 1010);
    });
  const DriftReport sagged = drift(bus_, 2, 1.5, &no_stop_);
  sag.join();
  EXPECT_EQ(sagged.exit, 0) << sagged.problem;
  EXPECT_EQ(sagged.drift_total_ticks, 8);
  EXPECT_EQ(sagged.drift_last_1s_ticks, 0);

  // a wheel is refused before anything is written
  fake_.clear_frames();
  const DriftReport wheel = drift(bus_, 3, 0.5, &no_stop_);
  EXPECT_EQ(wheel.exit, 3);
  EXPECT_THAT(wheel.problem, HasSubstr("mode 1"));
  EXPECT_TRUE(fake_.writes().empty());
}

TEST_F(HilEeprom, read_returns_one_register_or_fails)
{
  fake_.clear_frames();
  const ReadReport mode = read_register(bus_, 2, kRegMode, false);
  EXPECT_EQ(mode.exit, 0);
  EXPECT_TRUE(mode.ok);
  EXPECT_EQ(mode.value, 0);
  const ReadReport position = read_register(bus_, 2, kRegPresentPosition, true);
  EXPECT_TRUE(position.ok);
  EXPECT_EQ(position.value, 1002);
  const ReadReport byte13 = read_register(bus_, 3, 13, false);
  EXPECT_EQ(byte13.value, seeded(3, 13));

  fake_.set_silent_read(2, 13);
  const ReadReport silent = read_register(bus_, 2, 13, false);
  EXPECT_EQ(silent.exit, 3);
  EXPECT_FALSE(silent.ok);
  EXPECT_EQ(silent.value, kUnreadable);
  const ReadReport absent = read_register(bus_, 9, kRegMode, false);
  EXPECT_EQ(absent.exit, 3);
  EXPECT_FALSE(absent.ok);
  EXPECT_TRUE(fake_.writes().empty());

  // and on the RESULT line a failed read is null, never 0
  bus_.close();
  std::ostringstream out;
  EXPECT_EQ(
    run({"--port", fake_.port(), "read", "--id", "2", "--addr", "13"}, out, &no_stop_), 3)
    << out.str();
  EXPECT_THAT(out.str(), HasSubstr("\"ok\": false"));
  EXPECT_THAT(out.str(), HasSubstr("\"value\": null"));
}

TEST_F(HilEeprom, restore_defers_sigterm_until_the_lock_is_closed)
{
  const std::string bin = hil_eeprom_bin();
  ASSERT_FALSE(bin.empty()) << "WAVESHARE_HIL_EEPROM_BIN is not set (CMakeLists.txt ENV)";
  ASSERT_EQ(::access(bin.c_str(), X_OK), 0) << bin;
  fake_.set_byte(2, kRegLock, 1);   // the source's lock is closed, so a finished restore is too
  const Snapshot source = source_of_now();
  ASSERT_TRUE(source.ok);
  fake_.set_byte(2, kRegOffset, static_cast<uint8_t>(seeded(2, 31) ^ 0x10));
  fake_.set_eeprom_commit_ms(2, 300);   // the offset write holds the servo off the bus for 300 ms
  const TempDir dir;
  ASSERT_TRUE(dir.ok());
  write_file(dir.file("source.snap"), format_snap(source));
  bus_.close();   // the child takes the port
  fake_.clear_frames();

  const pid_t pid = spawn_to_file(
    {bin, "--port", fake_.port(), "restore", "--from", dir.file("source.snap")},
    dir.file("restore.txt"));
  ASSERT_GT(pid, 0);
  const WriteRecord unlock{2, kRegLock, {0}};
  bool saw_unlock = false;
  bool exited = false;
  int status = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (!saw_unlock && !exited && std::chrono::steady_clock::now() < deadline) {
    saw_unlock = contains(fake_.writes(), unlock);
    exited = !saw_unlock && ::waitpid(pid, &status, WNOHANG) == pid;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!saw_unlock) {
    if (!exited) {
      ::kill(pid, SIGKILL);
      ::waitpid(pid, &status, 0);
    }
    FAIL() << "the unlock never reached the bus; the restore said:\n" <<
      read_file(dir.file("restore.txt"));
  }
  ASSERT_EQ(::kill(pid, SIGTERM), 0);
  ASSERT_TRUE(wait_exit(pid, &status, std::chrono::seconds(30)));
  const std::string output = read_file(dir.file("restore.txt"));
  ASSERT_TRUE(WIFEXITED(status)) << "killed by signal " << WTERMSIG(status) << "\n" << output;
  EXPECT_EQ(WEXITSTATUS(status), 0) << output;
  // the sequence ran to its verified lock after the SIGTERM, and nothing was left open
  EXPECT_EQ(
    fake_.writes(), (std::vector<WriteRecord>{
    unlock, WriteRecord{2, kRegOffset, {seeded(2, 31), seeded(2, 32)}},
    WriteRecord{2, kRegLock, {1}}}));
  EXPECT_EQ(fake_.snapshot(2).mem[kRegLock], 1);
  EXPECT_EQ(fake_.snapshot(2).eeprom[kRegOffset], seeded(2, 31));
  EXPECT_THAT(output, HasSubstr("a signal arrived during the restore; it was completed first"));
}
