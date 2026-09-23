// scan -- lists every servo that answers on a bus, ids 0..253 (PHASE6_SPEC C.1). Read-only.
// All of it is in tool_main (parameters, port) and servo_tools (the bus logic).

#include "tool_main.hpp"

int main(int argc, char ** argv)
{
  return waveshare_servos::tools::run_tool(waveshare_servos::tools::Tool::kScan, argc, argv);
}
