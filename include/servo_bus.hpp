// ServoBus -- the package's only door to the serial bus.
//
// It wraps the vendored SMS_STS packet layer with the three things that layer does not do:
// exclusive access to the tty, a trustworthy reason when the port cannot be taken, and a close
// that really releases the device.
//
// Why a wrapper rather than an SMS_STS member (PHASE2_SPEC 9):
//   - SCSerial::begin() calls perror() on both of its failure paths, and glibc's perror does not
//     restore errno, so the errno the caller sees afterwards is not the one that caused the
//     failure. open() therefore takes its own descriptor first (step 4 below) and reports that
//     errno, which is the real one.
//   - SCSerial::setBaudRate()'s `default: break` leaves a speed_t uninitialised before
//     cfsetispeed/cfsetospeed (src/SCSerial.cpp:104,127-131). It is hidden, along with begin() and
//     end(), by the private using-declarations at the bottom of the class.
//   - The vendored constructors leave Err and the six syncRead* members indeterminate even though
//     ReadPos/ReadSpeed/ReadLoad/ReadCurrent read Err. The out-of-line constructor zeroes them.
//
// Exclusivity is two mechanisms, because neither alone is enough (PHASE2_SPEC 9.7): TIOCEXCL stops
// any unprivileged open of the device, and an advisory flock stops a second cooperating process
// even when it is root. Nothing protects against a process that already had the port when this
// one started; on_configure warns about those.
//
// This header deliberately includes "SMS_STS.h" rather than "SCServo.h": the driver uses no part
// of the SMSBL / SCSCL / SMSCL classes that "SCServo.h" also pulls in.

#ifndef SERVO_BUS_HPP_
#define SERVO_BUS_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "SMS_STS.h"

namespace waveshare_servos
{

// Why an open() did not take the port. Every value but OK names exactly one step of
// ServoBus::open(), so a caller can say what happened without guessing from errno.
enum class BusStatus : uint8_t
{
  OK = 0,
  ALREADY_OPEN,           // this bus already holds a port; the session was left untouched
  UNSUPPORTED_BAUDRATE,   // not one of the seven rates SCSerial::begin maps
  INVALID_TIMEOUT,        // 0 ms makes every transaction fail before the servo can answer
  OPEN_FAILED,            // the library's own open() failed; its reason went to stderr
  NOT_A_TTY,              // the path exists but is not a serial device
  LOCK_OPEN_FAILED,       // the lock descriptor could not be opened (this is where EBUSY lands)
  LOCK_FAILED,            // another process holds the advisory lock
  TERMIOS_FAILED,         // the line settings could not be applied
  EXCLUSIVE_FAILED        // the tty refused TIOCEXCL
};

// A short name for a status, for a log line or a test failure message.
const char * to_string(BusStatus status);

struct OpenResult
{
  BusStatus status = BusStatus::OK;
  // the errno of the failing step, or 0 when that step reports none
  int error = 0;
  explicit operator bool() const noexcept {return status == BusStatus::OK;}
};

// The pids that currently have this port open, this process included, from /proc. Diagnostics
// only: a snapshot, and descriptors of other users' processes are invisible. Empty when nothing
// holds the port and when /proc cannot be read. Never throws.
std::vector<int> port_holder_pids(const std::string & port) noexcept;

class ServoBus : public SMS_STS
{
public:
  // Defined out of line so no compiler-generated copy of either lands in a translation unit that
  // only sees the declaration.
  ServoBus();
  ~ServoBus();                                    // calls close(); non-virtual on purpose
  ServoBus(const ServoBus &) = delete;
  ServoBus & operator=(const ServoBus &) = delete;
  ServoBus(ServoBus &&) = delete;
  ServoBus & operator=(ServoBus &&) = delete;

  // The seven rates SCSerial::begin() maps (src/SCSerial.cpp:50-75). Anything else silently ends
  // up at 115200, which is why this is the single source of truth for both open() and the
  // on_init check of the `baudrate` hardware parameter.
  static bool is_supported_baudrate(int baudrate) noexcept;

  // Take the port exclusively. The argument order is the reverse of the library's
  // begin(baudRate, serialPort); this wrapper is the only place the flip happens. io_timeout_ms is
  // an argument rather than a setter because the timeout has to be in force before the first
  // transaction.
  //
  // Every failure path leaves the bus closed, both descriptors released and no TIOCEXCL set. On
  // the EXCLUSIVE_FAILED path begin() has already written the line settings and the vendored
  // library never restores the termios it saved (src/SCSerial.cpp:47,221-228), so the tty is left
  // configured for 8N1 at the requested rate rather than as it was found.
  OpenResult open(const std::string & port, int baudrate, uint32_t io_timeout_ms);
  // Clears TIOCEXCL, closes the library's descriptor and releases the lock, each only if it holds
  // it. Idempotent, and safe before any open() and after a failed one.
  void close() noexcept;

  bool is_open() const noexcept {return fd != -1;}
  const std::string & port() const noexcept {return port_;}
  int baudrate() const noexcept {return baudrate_;}
  uint32_t io_timeout_ms() const noexcept {return static_cast<uint32_t>(IOTimeOut);}
  // Refuses 0 and changes nothing: a zero timeout makes every transaction fail instantly.
  bool set_io_timeout_ms(uint32_t ms) noexcept;

  // The vendored transmit buffer (SCSerial::txBuf); the constructor static_asserts that this is
  // still its size, as a tripwire against an upstream refresh.
  static constexpr std::size_t tx_buffer_bytes = 255;

private:
  void drop_lock() noexcept;

  // Hidden, not overridden: a deleted function may not override a non-deleted one. These three
  // stay reachable through an SMS_STS &, which is why nothing in this package ever holds a base
  // reference to the bus.
  using SCSerial::begin;        // open() calls SCSerial::begin(...) qualified
  using SCSerial::end;          // close() calls SCSerial::end() qualified
  using SCSerial::setBaudRate;  // its default: branch reads an uninitialised speed_t

  std::string port_;
  int baudrate_ = 0;
  // The lock descriptor, opened before the library's and held for the life of the session. It is
  // never read from, so rFlushSCS()'s tcflush of the shared input queue steals nothing from it.
  int lock_fd_ = -1;
};

}  // namespace waveshare_servos

#endif  // SERVO_BUS_HPP_
