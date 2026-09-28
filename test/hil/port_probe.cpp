// port_probe: tries to take the tty the driver holds and reports which lock refused it. Never
// reads or writes the tty. See docs/bench-check.md, "port_probe".

#include <fcntl.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

namespace
{

constexpr int kMaxHoldSeconds = 60;

void usage()
{
  std::fprintf(stderr,
    "usage: port_probe [--port /dev/ttyACM0] [--hold SECONDS] [--no-exclusive]\n");
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string port = "/dev/ttyACM0";
  int hold = 0;
  bool no_exclusive = false;

  for (int a = 1; a < argc; a++) {
    const std::string arg = argv[a];
    auto next = [&]() -> std::string {
        if (a + 1 >= argc) {usage(); std::exit(64);}
        return argv[++a];
      };
    if (arg == "--port") {
      port = next();
    } else if (arg == "--hold") {
      hold = std::atoi(next().c_str());
    } else if (arg == "--no-exclusive") {
      no_exclusive = true;
    } else {
      usage();
      return 64;
    }
  }
  if (hold < 0) {
    hold = 0;
  }
  if (hold > kMaxHoldSeconds) {
    hold = kMaxHoldSeconds;
  }

  const char * verdict = "acquired";
  int rc = 0;
  int open_errno = 0;
  int flock_errno = 0;
  int ioctl_errno = 0;

  const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd == -1) {
    open_errno = errno;
    verdict = (open_errno == EBUSY) ? "refused_by_tiocexcl" : "open_failed";
    rc = (open_errno == EBUSY) ? 10 : 20;
  } else if (::flock(fd, LOCK_EX | LOCK_NB) == -1) {
    flock_errno = errno;
    verdict = (flock_errno == EWOULDBLOCK) ? "refused_by_flock" : "flock_failed";
    rc = (flock_errno == EWOULDBLOCK) ? 11 : 21;
  } else if (!no_exclusive && ::ioctl(fd, TIOCEXCL) == -1) {
    ioctl_errno = errno;
    verdict = "tiocexcl_failed";
    rc = 22;
  } else {
    if (hold > 0) {
      std::printf(
        "HOLDING {\"pid\": %d, \"port\": \"%s\", \"hold_s\": %d, \"no_exclusive\": %s}\n",
        ::getpid(), port.c_str(), hold, no_exclusive ? "true" : "false");
      std::fflush(stdout);
      std::this_thread::sleep_for(std::chrono::seconds(hold));
    }
    ::ioctl(fd, TIOCNXCL);
    ::flock(fd, LOCK_UN);
  }
  if (fd != -1) {
    ::close(fd);
  }

  const int reason = open_errno ? open_errno : (flock_errno ? flock_errno : ioctl_errno);
  std::printf(
    "RESULT {\"verdict\": \"%s\", \"port\": \"%s\", \"euid\": %u, \"pid\": %d, "
    "\"open_errno\": %d, \"flock_errno\": %d, \"ioctl_errno\": %d, \"hold_s\": %d, "
    "\"no_exclusive\": %s, \"strerror\": \"%s\", \"rc\": %d}\n",
    verdict, port.c_str(), ::geteuid(), ::getpid(), open_errno, flock_errno, ioctl_errno, hold,
    no_exclusive ? "true" : "false", reason ? ::strerror(reason) : "", rc);
  return rc;
}
