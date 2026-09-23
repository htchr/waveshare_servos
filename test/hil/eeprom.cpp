// hil_eeprom: the bench's EEPROM oracle and repair tool (PHASE6_SPEC E.1). A test fixture for the
// HIL check, never installed.
//
// Everything that touches a bus is in eeprom_core.cpp, where test_hil_eeprom drives it on the
// fake. This file owns only what a process owns: the signal handlers. SIGINT, SIGTERM, SIGHUP and
// SIGQUIT set a flag and nothing else. The core checks it before its first write (exit 130,
// nothing written) and blocks the four signals from the first write to the end of the sequence,
// so a Ctrl-C, a `timeout -s TERM` or an ssh drop cannot stop a restore between unlock and lock.
// SIGPIPE is ignored so a closed output pipe cannot either. SIGKILL cannot be caught; the journal
// (E.2) is the answer to that.
//
// Usage and exit codes: see eeprom_core.hpp and `hil_eeprom` with no arguments.

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
