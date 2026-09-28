// factory_reset -- puts one servo back to its factory settings with RESET (0x06), keeping its
// id, and verifies it at the factory rate. All of it is in tool_main and servo_tools.

#include "tool_main.hpp"

int main(int argc, char ** argv)
{
  return waveshare_servos::tools::run_tool(
    waveshare_servos::tools::Tool::kFactoryReset, argc, argv);
}
