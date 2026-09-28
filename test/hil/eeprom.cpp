// hil_eeprom: the bench's EEPROM oracle and repair tool (a test fixture); this file owns only
// the signal handlers. See docs/bench-check.md, "hil_eeprom".

#include <signal.h>

#include <exception>
#include <iostream>
#include <string>
#include <vector>

#include "eeprom_core.hpp"

namespace
{

volatile std::sig_atomic_t g_stop = 0;

void on_signal(int signal_number)
{
  static_cast<void>(signal_number);
  g_stop = 1;
}

}  // namespace

int main(int argc, char ** argv)
{
  struct sigaction action {};
  action.sa_handler = on_signal;
  action.sa_flags = SA_RESTART;
  sigemptyset(&action.sa_mask);
  for (const int signal_number : {SIGINT, SIGTERM, SIGHUP, SIGQUIT}) {
    sigaction(signal_number, &action, nullptr);
  }
  signal(SIGPIPE, SIG_IGN);

  const std::vector<std::string> args(argv + 1, argv + argc);
  try {
    return waveshare_servos::hil_eeprom::run(args, std::cout, &g_stop);
  } catch (const std::exception & e) {
    std::cout << "internal error: " << e.what() << "\n"
              << "RESULT {\"ok\": false, \"error\": \"internal\"}" << std::endl;
  } catch (...) {
    std::cout << "internal error\nRESULT {\"ok\": false, \"error\": \"internal\"}" << std::endl;
  }
  return waveshare_servos::hil_eeprom::kExitInternal;
}
