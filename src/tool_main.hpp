// tool_main -- the whole of the Phase 6 executables' main(): parameters, then the bus (B.3).
//
// scan, set_id, calibrate_midpoint and factory_reset are each one line that calls run_tool(). What
// it does, in order (PHASE6_SPEC A.1, B.3):
//   1. installs its own handlers for SIGINT, SIGTERM, SIGHUP and SIGQUIT, which only set a flag,
//      and ignores SIGPIPE -- before rclcpp exists, so DDS start-up is covered too;
//   2. reads the overrides through rclcpp and rcl, refuses every one it cannot apply (A.1: stale,
//      unknown or wrongly typed names, overrides addressed to another node, leftover arguments)
//      and shuts rclcpp down again, so the tool is no DDS participant while it holds the port;
//   3. opens the bus through ServoBus (the driver's exclusive lock), runs the tool, prints the
//      detail line, and maps the outcome through final_exit().
// Exit codes are A.3's: 64 before the port is touched, 70 for an exception, 130 for a signal.
//
// Under src/ and not installed, like the rest of the tools' code.

#ifndef TOOL_MAIN_HPP_
#define TOOL_MAIN_HPP_

#include "tool_params.hpp"

namespace waveshare_servos
{
namespace tools
{

// The executable's exit code (A.3). Never throws: an exception is caught and becomes 70.
int run_tool(Tool tool, int argc, char ** argv);

}  // namespace tools
}  // namespace waveshare_servos

#endif  // TOOL_MAIN_HPP_
