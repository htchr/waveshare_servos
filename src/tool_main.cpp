#include "tool_main.hpp"

#include <signal.h>

#include <csignal>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <ostream>
#include <stdexcept>
#include <streambuf>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "driver_defaults.hpp"
#include "rcl/arguments.h"
#include "rcl/error_handling.h"
#include "rcl_yaml_param_parser/parser.h"
#include "rclcpp/rclcpp.hpp"
#include "servo_bus.hpp"
#include "servo_tools.hpp"

namespace waveshare_servos
{
namespace tools
{
namespace
{

using Overrides = std::map<std::string, rclcpp::ParameterValue>;

constexpr int kUsage = static_cast<int>(Exit::kUsage);

// A.1 step 1. The handlers are process-wide, so a signal delivered to any thread -- a DDS thread
// during start-up included -- only sets this flag; the run functions read it through
// Session::stop and decide where it is safe to stop (never between an EEPROM unlock and its lock).
volatile std::sig_atomic_t g_stop = 0;

void on_stop_signal(int signal_number)
{
  static_cast<void>(signal_number);
  g_stop = 1;
}

void install_stop_handlers()
{
  struct sigaction action{};
  action.sa_handler = on_stop_signal;
  sigemptyset(&action.sa_mask);
  // Restarted, so a write to a slow terminal is not cut short by the signal; the bus reads are
  // select()s, which a signal interrupts anyway and readSCS retries (0.2.2).
  action.sa_flags = SA_RESTART;
  for (const int signal_number : {SIGINT, SIGTERM, SIGHUP, SIGQUIT}) {
    ::sigaction(signal_number, &action, nullptr);
  }
  // A closed stdout or stderr pipe (`2>&1 | head`) is then an EPIPE the stream swallows, never a
  // death in the middle of an EEPROM sequence.
  ::signal(SIGPIPE, SIG_IGN);
}

// The stream servo_tools writes its diagnostics to (D-22): every line reaches `target` with
// "<tool>: " in front, whole, in one write. The library writes bare lines and stays tool-agnostic;
// the executable is what knows its own name.
class PrefixedLines : public std::streambuf
{
public:
  PrefixedLines(std::streambuf * target, std::string prefix)
  : target_(target), prefix_(std::move(prefix)) {}

  ~PrefixedLines() override {sync();}

  PrefixedLines(const PrefixedLines &) = delete;
  PrefixedLines & operator=(const PrefixedLines &) = delete;
  PrefixedLines(PrefixedLines &&) = delete;
  PrefixedLines & operator=(PrefixedLines &&) = delete;

protected:
  int_type overflow(int_type c) override
  {
    if (traits_type::eq_int_type(c, traits_type::eof())) {
      return traits_type::not_eof(c);
    }
    pending_.push_back(traits_type::to_char_type(c));
    if (pending_.back() == '\n' && !emit()) {
      return traits_type::eof();
    }
    return c;
  }

  std::streamsize xsputn(const char * data, std::streamsize count) override
  {
    for (std::streamsize i = 0; i < count; i++) {
      const int_type written = overflow(traits_type::to_int_type(data[i]));
      if (traits_type::eq_int_type(written, traits_type::eof())) {
        return i;
      }
    }
    return count;
  }

  // A flush in the middle of a line writes what there is; the rest of that line then follows
  // without a second prefix.
  int sync() override
  {
    if (!pending_.empty() && !emit()) {
      return -1;
    }
    return target_->pubsync();
  }

private:
  bool emit()
  {
    const std::string text = (at_line_start_ ? prefix_ : std::string()) + pending_;
    at_line_start_ = pending_.back() == '\n';
    pending_.clear();
    return target_->sputn(text.data(), static_cast<std::streamsize>(text.size())) ==
           static_cast<std::streamsize>(text.size());
  }

  std::streambuf * target_;
  std::string prefix_;
  std::string pending_;
  bool at_line_start_ = true;
};

// Frees what rcl_arguments_get_param_overrides() allocated, however the reading of it ends.
struct ParamsDeleter
{
  void operator()(rcl_params_t * params) const {rcl_yaml_node_struct_fini(params);}
};

// A.1 step 4. The overrides rclcpp resolved for this tool's node, minus the parameters rclcpp
// declares itself (use_sim_time and the like, so no allow-list of rclcpp's own names is kept
// here); and in `refusals`, one message per other node name that rcl holds overrides for, which
// rclcpp would drop without a word.
Overrides node_overrides(Tool tool, std::vector<std::string> * refusals)
{
  auto node = std::make_shared<rclcpp::Node>(
    name_of(tool), rclcpp::NodeOptions().start_parameter_services(false).enable_rosout(false));
  Overrides overrides;
  for (const auto & entry : node->get_node_parameters_interface()->get_parameter_overrides()) {
    if (!node->has_parameter(entry.first)) {
      overrides.insert(entry);
    }
  }

  // Every override rcl parsed -- `-p` rules and --params-file contents alike -- keyed by the node
  // name as written; NULL when there were none.
  rcl_params_t * raw = nullptr;
  const rcl_ret_t ret = rcl_arguments_get_param_overrides(
    &rclcpp::contexts::get_global_default_context()->get_rcl_context()->global_arguments, &raw);
  const std::unique_ptr<rcl_params_t, ParamsDeleter> params(raw);
  if (ret != RCL_RET_OK) {
    throw std::runtime_error(
      std::string("could not read the parameter overrides: ") + rcl_get_error_string().str);
  }
  std::map<std::string, std::vector<std::string>> keys_by_node;
  for (std::size_t n = 0; params != nullptr && n < params->num_nodes; n++) {
    std::vector<std::string> & keys = keys_by_node[params->node_names[n]];
    for (std::size_t k = 0; k < params->params[n].num_params; k++) {
      keys.push_back(params->params[n].parameter_names[k]);
    }
  }
  for (const std::string & refusal :
    foreign_override_nodes(node->get_fully_qualified_name(), keys_by_node))
  {
    refusals->push_back(refusal);
  }
  return overrides;
}

// Each error as one "<tool>: <message>" line, then the usage line as A.2 shows it.
int usage_exit(Tool tool, const std::vector<std::string> & errors, std::ostream & err)
{
  for (const std::string & error : errors) {
    err << error << "\n";
  }
  err.flush();
  std::cerr << usage(tool) << "\n";
  return kUsage;
}

// A.1 steps 2-7: everything before the port. 0 with `config` filled when the tool may run,
// otherwise the exit code, with its reason already printed. Nothing here touches the port.
int start_up(Tool tool, int argc, char ** argv, std::ostream & err, ToolConfig * config)
{
  // Step 2. Logging is not initialised (no ROS log files; the tools print their own messages),
  // and rclcpp installs no signal handler: step 1's are the only ones.
  try {
    rclcpp::init(
      argc, argv, rclcpp::InitOptions().auto_initialize_logging(false),
      rclcpp::SignalHandlerOptions::None);
  } catch (const std::exception & error) {
    return usage_exit(tool, {error.what()}, err);
  }

  std::vector<std::string> errors;
  Overrides overrides;
  try {
    // Step 3. Whatever rclcpp does not consume is refused: a positional port would otherwise be
    // ignored and the default one opened.
    const std::vector<std::string> left = rclcpp::remove_ros_arguments(argc, argv);
    if (left.size() > 1) {
      rclcpp::shutdown();
      return usage_exit(
        tool, {"unexpected argument '" + left[1] + "'; parameters go after --ros-args, for "
          "example: ros2 run waveshare_servos " + name_of(tool) + " --ros-args -p port:=" +
          left[1]}, err);
    }
    // Step 4.
    overrides = node_overrides(tool, &errors);
  } catch (const std::exception & error) {
    rclcpp::shutdown();
    return usage_exit(tool, {error.what()}, err);
  }

  // Step 5. From here on the tool is no DDS participant, and a signal only sets g_stop.
  rclcpp::shutdown();

  // Step 6: all errors of steps 4 and 6 together, so a command line is fixed in one round.
  ParseResult parsed = parse_params(tool, overrides);
  errors.insert(errors.end(), parsed.errors.begin(), parsed.errors.end());
  if (!errors.empty() || !parsed.config.has_value()) {
    return usage_exit(tool, errors, err);
  }

  // Step 7.
  if (g_stop != 0) {
    err << "interrupted before the port was opened; nothing was sent\n";
    return static_cast<int>(Exit::kInterrupted);
  }
  *config = *parsed.config;
  return 0;
}

// B.3 steps 1-6: the port, the tool, the detail line and the final code.
Exit run_on_bus(Tool tool, const ToolConfig & config, std::ostream & err)
{
  ServoBus bus;                       // its destructor closes the port on every path, a throw too
  const Exit opened = open_bus(bus, config.port, config.baudrate, err);
  if (opened != Exit::kOk) {
    return opened;
  }
  Session session{bus, defaults::kPingAttempts, io_timeout_ms_for(config.baudrate), &g_stop,
    std::cout, err};

  Exit outcome = Exit::kInternal;
  std::size_t writes_sent = 0;        // scan sends none
  switch (tool) {
    case Tool::kScan: {
        const ScanResult result = scan(
          session, static_cast<uint8_t>(kScanFirstId), static_cast<uint8_t>(kScanLastId));
        print_scan(result, config.port, config.baudrate, std::cout, err);
        outcome = scan_exit(result);
        break;
      }
    case Tool::kSetId: {
        // parse_params range-checked both ids as int64: neither can narrow to 0xfe or 0xff
        const SetIdReport report = set_id(
          session, static_cast<uint8_t>(config.start_id), static_cast<uint8_t>(config.new_id));
        err << detail_line(report) << "\n";
        outcome = report.exit;
        writes_sent = report.writes_sent;
        break;
      }
    case Tool::kCalibrateMidpoint: {
        const CalibrateReport report = calibrate_midpoint(session, static_cast<uint8_t>(config.id));
        err << detail_line(report) << "\n";
        outcome = report.exit;
        writes_sent = report.writes_sent;
        break;
      }
    case Tool::kFactoryReset: {
        // It may leave the bus at kFactoryBaudrate; nothing after this reads the bus's rate.
        const FactoryResetReport report = factory_reset(session, static_cast<uint8_t>(config.id));
        err << detail_line(report) << "\n";
        outcome = report.exit;
        writes_sent = report.writes_sent;
        break;
      }
  }

  // Step 5: a port that vanished during the run (a USB drop) says "nothing sent" only if nothing
  // was written; otherwise the run's own outcome still describes the servo.
  std::error_code ignored;
  const bool port_exists = std::filesystem::exists(config.port, ignored);
  if (!port_exists) {
    err << "the port disappeared during the run\n";
    if (writes_sent > 0) {
      err << "after " << writes_sent << " write(s); the servo's state is what the message above "
        "says, not \"nothing sent\"\n";
    }
  }
  const Exit code = final_exit(outcome, writes_sent, port_exists);
  bus.close();
  return code;
}

}  // namespace

int run_tool(Tool tool, int argc, char ** argv)
{
  install_stop_handlers();            // A.1 step 1: before anything else
  PrefixedLines prefixed(std::cerr.rdbuf(), std::string(name_of(tool)) + ": ");
  std::ostream err(&prefixed);
  int code = static_cast<int>(Exit::kInternal);
  try {
    ToolConfig config;
    code = start_up(tool, argc, argv, err, &config);
    if (code == 0) {
      code = static_cast<int>(run_on_bus(tool, config, err));
    }
  } catch (const std::exception & error) {
    err << "internal error: " << error.what() << "\n";
    code = static_cast<int>(Exit::kInternal);
  } catch (...) {
    err << "internal error: an exception that is not a std::exception\n";
    code = static_cast<int>(Exit::kInternal);
  }
  err.flush();
  std::cout.flush();
  return code;
}

}  // namespace tools
}  // namespace waveshare_servos
