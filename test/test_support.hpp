// Test support with no ros2_control dependency, so a target that links only the bus can use it.

#ifndef TEST_SUPPORT_HPP_
#define TEST_SUPPORT_HPP_

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

}  // namespace waveshare_servos_test

#endif  // TEST_SUPPORT_HPP_
