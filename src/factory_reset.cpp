// factory_reset -- puts one servo back to its factory settings with the protocol's RESET (0x06),
// keeping its id, and verifies it by reading back at the factory rate
// (factory_reset_evidence/FACTORY_RESET_SPEC.md). All of it is in tool_main (parameters, port) and
// servo_tools (the bus logic).

#include "tool_main.hpp"

int main(int argc, char ** argv)
{
  return waveshare_servos::tools::run_tool(
    waveshare_servos::tools::Tool::kFactoryReset, argc, argv);
}
