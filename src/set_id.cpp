// set_id -- gives one servo a new id, verified by reading back (PHASE6_SPEC C.2).
// All of it is in tool_main (parameters, port) and servo_tools (the bus logic).

#include "tool_main.hpp"

int main(int argc, char ** argv)
{
  return waveshare_servos::tools::run_tool(waveshare_servos::tools::Tool::kSetId, argc, argv);
}
