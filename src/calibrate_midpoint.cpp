// calibrate_midpoint -- makes a position servo's present position its midpoint, 2048, verified
// by reading back. All of it is in tool_main (parameters, port) and servo_tools (bus logic).

#include "tool_main.hpp"

int main(int argc, char ** argv)
{
  return waveshare_servos::tools::run_tool(
    waveshare_servos::tools::Tool::kCalibrateMidpoint, argc, argv);
}
