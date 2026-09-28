// tool_main: the shared main() of the four tools (a private header, not installed).
// See docs/design.md, "Tool structure".

#ifndef TOOL_MAIN_HPP_
#define TOOL_MAIN_HPP_

#include "tool_params.hpp"

namespace waveshare_servos
{
namespace tools
{

// The executable's exit code. Never throws: an exception is caught and becomes 70.
int run_tool(Tool tool, int argc, char ** argv);

}  // namespace tools
}  // namespace waveshare_servos

#endif  // TOOL_MAIN_HPP_
