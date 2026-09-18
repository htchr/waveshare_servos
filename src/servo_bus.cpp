#include "servo_bus.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <system_error>
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

}  // namespace

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
  syncReadRxBuff = nullptr;
  syncReadRxBuffLen = 0;
  syncReadRxBuffMax = 0;
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
