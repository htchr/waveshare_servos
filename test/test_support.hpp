// Test support with no ros2_control dependency, so a target that links only the bus can use it.

#ifndef TEST_SUPPORT_HPP_
#define TEST_SUPPORT_HPP_

#include <cstddef>
#include <filesystem>
#include <string>
#include <system_error>

namespace waveshare_servos_test
{

// Bus guard: SCSerial::begin() keeps the tty open, so a configure would leave a descriptor behind.
inline bool process_has_serial_port_open()
{
  for (const auto & entry : std::filesystem::directory_iterator("/proc/self/fd")) {
    std::error_code error;
    const std::string target = std::filesystem::read_symlink(entry.path(), error).string();
    if (error) {
      continue;
    }
    for (const char * prefix : {"/dev/ttyACM", "/dev/ttyUSB", "/dev/ttyTHS", "/dev/serial"}) {
      if (target.rfind(prefix, 0) == 0) {
        return true;
      }
    }
  }
  return false;
}

// How many of this process's descriptors point at `path`, from /proc/self/fd. Immune to the
// descriptor the directory iterator itself holds, which a plain count of the directory is not.
// The body of test_lifecycle_over_pty.cpp's file-local copy; that copy and test_servo_bus.cpp's
// stay where they are, and per-name using-declarations keep the three from clashing.
inline size_t descriptors_on(const std::string & path)
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

}  // namespace waveshare_servos_test

#endif  // TEST_SUPPORT_HPP_
