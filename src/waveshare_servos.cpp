#include "waveshare_servos.hpp"

#include <unistd.h>

#include <vector>
#include <algorithm>
#include <array>
#include <cerrno>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "param_parsing.hpp"
#include "units.hpp"

namespace waveshare_servos
{
namespace
{

// The pids other than this one that already have the port open, as "1234, 5678", or "" when the
// port is ours alone. Diagnostics: /proc shows nothing about another user's processes.
std::string other_holders_of(const std::string & port)
{
  std::string others;
  for (const int pid : port_holder_pids(port)) {
    if (pid == static_cast<int>(::getpid())) {
      continue;
    }
    others += others.empty() ? std::to_string(pid) : ", " + std::to_string(pid);
  }
  return others;
}

// `parts` as one sentence fragment: "a", "a, b", "a, b, c".
std::string join(const std::vector<std::string> & parts, const std::string & separator)
{
  std::string joined;
  for (const std::string & part : parts) {
    joined += joined.empty() ? part : separator + part;
  }
  return joined;
}

// The parenthetical a "the port is taken" message ends with. Named separately from the sentence
// so the two EBUSY branches cannot drift apart.
std::string holder_suffix(const std::string & port)
{
  const std::string others = other_holders_of(port);
  if (others.empty()) {
    return " (no holder visible in /proc; it may belong to another user)";
  }
  return " (pid " + others + ")";
}

// Known <hardware> params. Others get a WARN and are ignored: rejecting them would break
// descriptions with extra params, and silence would hide a typo.
constexpr std::array<const char *, 11> kKnownHardwareParams = {
  "port", "baudrate", "io_timeout_ms", "ping_attempts", "max_read_fails",
  "allow_missing_servos", "protocol", "feedback_mode", "encoder_steps", "current_per_count_a",
  "torque_constant_nm_per_a"};

// Known <joint> params; others get a WARN and are ignored, so a typo like 'invert' shows.
constexpr std::array<const char *, 7> kKnownJointParams = {
  "id", "type", "offset", "inverted", "max_speed", "max_accel", "unwrap"};

// std::lround outside long's range is unspecified (LONG_MIN here), so saturate at +/-2^62
// (exact in a double) first. Callers pass finite values only.
int64_t saturating_lround(double value)
{
  constexpr double kSaturation = 4611686018427387904.0;  // 2^62
  if (!(value > -kSaturation)) {
    return -4611686018427387904;
  }
  if (!(value < kSaturation)) {
    return 4611686018427387904;
  }
  return std::lround(value);
}

// An optional parameter keeps its default when it is absent; every other status is fatal.
bool rejected(params::Status status)
{
  return status != params::Status::kOk && status != params::Status::kDefaulted;
}

std::string subject_of(const std::string & name)
{
  return "hardware parameter '" + name + "'";
}

// The nine state interface names, indexed by StateKind; on_init refuses any other name.
// See docs/configuration.md, "State interfaces".
constexpr std::array<const char *, kStateKindCount> kStateKindNames = {
  hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
  hardware_interface::HW_IF_EFFORT, hardware_interface::HW_IF_CURRENT,
  HW_IF_VOLTAGE, hardware_interface::HW_IF_TEMPERATURE, HW_IF_LOAD, HW_IF_STATUS,
  hardware_interface::HW_IF_TORQUE};

// Exact match, no aliasing: the framework already strips leading and trailing whitespace
// from the name attribute (component_parser.cpp).
std::optional<StateKind> state_kind(const std::string & name)
{
  for (size_t k = 0; k < kStateKindNames.size(); k++) {
    if (name == kStateKindNames[k]) {
      return static_cast<StateKind>(k);
    }
  }
  return std::nullopt;
}

}  // namespace

hardware_interface::CallbackReturn WaveshareServos::read_hardware_parameters()
{
  const params::ParameterMap & p = info_.hardware_parameters;
  // Start from the compiled-in defaults rather than from whatever the members happen to hold, so
  // the documented value survives an absent parameter even on a second on_init.
  port_ = defaults::kPort;
  baudrate_ = defaults::kBaudrate;
  protocol_ = defaults::kProtocol;
  feedback_mode_ = defaults::kFeedbackMode;
  io_timeout_ms_ = static_cast<uint32_t>(defaults::kIoTimeoutMs);
  ping_attempts_ = defaults::kPingAttempts;
  max_read_fails_ = defaults::kMaxReadFails;
  allow_missing_servos_ = defaults::kAllowMissingServos;
  encoder_steps_ = defaults::kEncoderSteps;
  current_per_count_a_ = defaults::kCurrentPerCountA;
  torque_constant_nm_per_a_ = defaults::kTorqueConstantNmPerA;

  // One FATAL, then ERROR: the first bad value wins and nothing after it is parsed.
  const auto fail = [this](const std::string & message) {
      RCLCPP_FATAL(get_logger(), "%s", message.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    };

  // The unknown-parameter scan runs first, so a typo is visible even when a later value is fatal.
  // hardware_parameters is unordered, so only sorting makes the log reproducible.
  std::vector<std::string> unknown;
  for (const auto & entry : p) {
    const std::string & name = entry.first;
    const bool known = std::any_of(
      kKnownHardwareParams.begin(), kKnownHardwareParams.end(),
      [&name](const char * candidate) {return name == candidate;});
    if (!known) {
      unknown.emplace_back(name);
    }
  }
  std::sort(unknown.begin(), unknown.end());
  for (const std::string & name : unknown) {
    RCLCPP_WARN(get_logger(),
      "hardware parameter '%s' is not used by this driver; ignoring it", name.c_str());
  }

  params::Status st = params::get_string(p, "port", port_);
  if (rejected(st)) {
    return fail(params::message(
      st, subject_of("port"), params::raw(p, "port"),
      "a device path such as '/dev/ttyACM0'"));
  }

  // baudrate is a set, not a range: get_int checks only the syntax, and one message covers
  // every unsupported value.
  int64_t baudrate = baudrate_;
  st = params::get_int(p, "baudrate", INT64_MIN, INT64_MAX, baudrate);
  if (rejected(st)) {
    return fail(params::message(
      st, subject_of("baudrate"), params::raw(p, "baudrate"), "an integer"));
  }
  if (st == params::Status::kOk) {
    // Check the int range first: narrowing 2^32 + 9600 would wrap onto a supported rate.
    // is_supported_baudrate() is shared with ServoBus::open().
    if (baudrate < std::numeric_limits<int>::min() || baudrate > std::numeric_limits<int>::max() ||
      !ServoBus::is_supported_baudrate(static_cast<int>(baudrate)))
    {
      return fail(
        subject_of("baudrate") + " is '" + params::raw(p, "baudrate") +
        "'; the servo library maps only 9600, 19200, 38400, 57600, 115200, 500000 and 1000000, "
        "and silently falls back to 115200 for anything else");
    }
    baudrate_ = static_cast<int>(baudrate);
  }

  std::string protocol = protocol_;
  st = params::get_string(p, "protocol", protocol);
  if (rejected(st)) {
    return fail(params::message(
      st, subject_of("protocol"), params::raw(p, "protocol"), "'sms_sts'"));
  }
  protocol = hardware_interface::to_lower_case(protocol);
  if (protocol == "scscl") {
    return fail(
      subject_of("protocol") +
      " is 'scscl'; only 'sms_sts' is implemented, the SCS/SCSCL series is not supported yet");
  }
  if (protocol != "sms_sts") {
    return fail(params::message(
      params::Status::kMalformed, subject_of("protocol"), params::raw(p, "protocol"),
      "a known protocol; expected 'sms_sts'"));
  }
  protocol_ = protocol;

  // Lower-cased like protocol, so the configuration line prints one spelling.
  std::string feedback_mode = feedback_mode_;
  st = params::get_string(p, "feedback_mode", feedback_mode);
  if (rejected(st)) {
    return fail(params::message(
      st, subject_of("feedback_mode"), params::raw(p, "feedback_mode"),
      "'auto', 'sync_read' or 'per_servo'"));
  }
  feedback_mode = hardware_interface::to_lower_case(feedback_mode);
  if (feedback_mode != "auto" && feedback_mode != "sync_read" && feedback_mode != "per_servo") {
    return fail(params::message(
      params::Status::kMalformed, subject_of("feedback_mode"), params::raw(p, "feedback_mode"),
      "a known feedback mode; expected 'auto', 'sync_read' or 'per_servo'"));
  }
  feedback_mode_ = feedback_mode;

  // 2..1000 ms. 1 ms is refused: a sync read can then return the previous cycle's frames,
  // which pass every check. See docs/bus-timing.md, "Transaction timeout".
  int64_t io_timeout_ms = io_timeout_ms_;
  st = params::get_int(p, "io_timeout_ms", 2, 1000, io_timeout_ms);
  if (rejected(st)) {
    return fail(params::message(
      st, subject_of("io_timeout_ms"), params::raw(p, "io_timeout_ms"),
      "an integer between 2 and 1000 (milliseconds); 1 ms is not enough for a batched feedback "
      "read, which needs about 0.48 ms plus 0.29 ms per servo"));
  }
  const bool io_timeout_user_set = st == params::Status::kOk;
  io_timeout_ms_ = static_cast<uint32_t>(io_timeout_ms);

  int64_t ping_attempts = ping_attempts_;
  st = params::get_int(p, "ping_attempts", 1, 10, ping_attempts);
  if (rejected(st)) {
    return fail(params::message(
      st, subject_of("ping_attempts"), params::raw(p, "ping_attempts"),
      "an integer between 1 and 10"));
  }
  ping_attempts_ = static_cast<int>(ping_attempts);

  int64_t max_read_fails = max_read_fails_;
  st = params::get_int(p, "max_read_fails", 1, 1000000, max_read_fails);
  if (rejected(st)) {
    return fail(params::message(
      st, subject_of("max_read_fails"), params::raw(p, "max_read_fails"),
      "an integer between 1 and 1000000"));
  }
  max_read_fails_ = static_cast<int>(max_read_fails);

  st = params::get_bool(p, "allow_missing_servos", allow_missing_servos_);
  if (rejected(st)) {
    return fail(params::message(
      st, subject_of("allow_missing_servos"), params::raw(p, "allow_missing_servos"),
      "'true' or 'false'"));
  }

  // At most 32768, so the largest goal tick (32767) fits the 15-bit position field; even,
  // so half a revolution (encoder_steps / 2) is exact.
  const std::string encoder_steps_expected =
    "an even integer between 2 and 32768 (encoder steps per revolution)";
  int64_t encoder_steps = encoder_steps_;
  st = params::get_int(p, "encoder_steps", 2, 32768, encoder_steps);
  if (rejected(st)) {
    return fail(params::message(
      st, subject_of("encoder_steps"), params::raw(p, "encoder_steps"), encoder_steps_expected));
  }
  if ((encoder_steps % 2) != 0) {
    return fail(params::message(
      params::Status::kOutOfRange, subject_of("encoder_steps"), params::raw(p, "encoder_steps"),
      encoder_steps_expected));
  }
  encoder_steps_ = static_cast<int>(encoder_steps);

  st = params::get_double(p, "current_per_count_a", 0.0, 1.0, true, current_per_count_a_);
  if (rejected(st)) {
    return fail(params::message(
      st, subject_of("current_per_count_a"), params::raw(p, "current_per_count_a"),
      "a number greater than 0 and at most 1 (amperes per current count)"));
  }

  st = params::get_double(p, "torque_constant_nm_per_a", 0.0, 100.0, true,
      torque_constant_nm_per_a_);
  if (rejected(st)) {
    return fail(params::message(
      st, subject_of("torque_constant_nm_per_a"), params::raw(p, "torque_constant_nm_per_a"),
      "a number greater than 0 and at most 100 (newton metres per ampere)"));
  }

  // io_timeout_ms must carry a whole sync-read burst (about 0.48 ms + 0.29 ms per servo, per
  // chunk of up to 30 ids). See docs/bus-timing.md, "Timeout floor".
  const size_t chunk_servos = std::min(info_.joints.size(), ServoBus::sync_read_max_ids);
  const uint32_t floor_ms = ServoBus::min_io_timeout_ms(chunk_servos);
  if (feedback_mode_ != "per_servo" && io_timeout_ms_ < floor_ms) {
    if (!io_timeout_user_set) {
      // Defaulted: raise it to the floor (INFO) and keep the sync path. The 5 ms default is
      // enough for up to 9 servos.
      RCLCPP_INFO(get_logger(),
        "io_timeout_ms raised from %u to %u ms for a sync read of %zu servos",
        io_timeout_ms_, floor_ms, chunk_servos);
      io_timeout_ms_ = floor_ms;
    } else if (feedback_mode_ == "sync_read") {
      // 'sync_read' means "never demote", so no degraded path is left: the only alternative to
      // refusing is to run the transport the user explicitly ruled out.
      return fail(
        subject_of("io_timeout_ms") + " is '" + params::raw(p, "io_timeout_ms") + "', below the " +
        std::to_string(floor_ms) + " ms a sync read of " + std::to_string(chunk_servos) +
        " servos needs; feedback_mode is 'sync_read', which rules out the per-servo path that "
        "would survive it, so raise io_timeout_ms to at least " + std::to_string(floor_ms) +
        " or use 'auto'");
    } else {
      // User-set with 'auto': keep the value and use per-servo reads, which work down to 1 ms
      // (one 21-byte reply). Tests match this WARN text.
      RCLCPP_WARN(get_logger(),
        "io_timeout_ms %u is below the %u ms a sync read of %zu servos needs here; using one "
        "feedback read per servo", io_timeout_ms_, floor_ms, chunk_servos);
      feedback_mode_ = "per_servo";
    }
  }

  // WARN above max(8 ms, floor): 8 ms is a 10 ms period minus the 2 ms drain of a failed
  // read. Expensive is not wrong: a slower loop can need it.
  const uint32_t stall_warn_ms = std::max(8u, floor_ms);
  if (feedback_mode_ != "per_servo" && io_timeout_ms_ > stall_warn_ms) {
    RCLCPP_WARN(get_logger(),
      "io_timeout_ms is %u ms and a failed read pays the %u ms drain on top of it; that is what "
      "one non-answering servo costs in every cycle that polls it (measured at 1.00-1.03x the "
      "setting across 2..50 ms), and a 100 Hz loop has 10 ms in all. A slower loop or a "
      "deliberately patient bus makes that legitimate; it is a problem only if a servo goes "
      "silent and 100 Hz still has to be met",
      io_timeout_ms_, static_cast<uint32_t>(ServoBus::sync_read_drain_ms));
  }

  // The effective configuration, after the floor logic above. %.7g keeps 0.8825985 exact; a
  // repeated <param> silently keeps its last value, so this line is the user's check.
  RCLCPP_INFO(get_logger(),
    "bus configuration: port '%s', %d baud, protocol '%s', io timeout %u ms, %d ping attempt(s), "
    "drop a servo after %d consecutive read failures, allow_missing_servos %s, feedback_mode "
    "'%s', %d encoder steps per revolution, %.7g A per current count, %.7g N m/A",
    port_.c_str(), baudrate_, protocol_.c_str(), io_timeout_ms_, ping_attempts_, max_read_fails_,
    allow_missing_servos_ ? "true" : "false", feedback_mode_.c_str(), encoder_steps_,
    current_per_count_a_, torque_constant_nm_per_a_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WaveshareServos::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // the hardware parameters come before any joint, so a description with both a bad hardware
  // parameter and a bad joint reports the hardware one
  if (read_hardware_parameters() != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // check urdf definitions
  joints_.assign(info_.joints.size(), JointConfig{});
  // Ids have to be unique inside this <ros2_control> block; two blocks that share a port are kept
  // apart by the port itself, not here.
  std::unordered_map<int64_t, std::string> ids_seen;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    const hardware_interface::ComponentInfo & joint = info_.joints[i];
    const params::ParameterMap & jp = joint.parameters;
    JointConfig & c = joints_[i];
    // 0. Unknown params first, so a typo shows even if a later value is fatal; sorted for a
    // reproducible log.
    std::vector<std::string> unknown_joint_params;
    for (const auto & entry : jp) {
      const std::string & key = entry.first;
      const bool known = std::any_of(
        kKnownJointParams.begin(), kKnownJointParams.end(),
        [&key](const char * candidate) {return key == candidate;});
      if (!known) {
        unknown_joint_params.emplace_back(key);
      }
    }
    std::sort(unknown_joint_params.begin(), unknown_joint_params.end());
    for (const std::string & key : unknown_joint_params) {
      RCLCPP_WARN(get_logger(),
        "joint '%s' parameter '%s' is not used by this driver; ignoring it",
        joint.name.c_str(), key.c_str());
    }
    // 1. id: required, 1..253.
    int64_t id = 0;
    params::Status st = params::get_int(jp, "id", limits::kIdMin, limits::kIdMax, id);
    if (st == params::Status::kDefaulted || st == params::Status::kEmpty) {
      RCLCPP_FATAL(get_logger(),
        "joint '%s' has no <param name=\"id\">; every joint needs the bus id of its servo (1..253)",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (st == params::Status::kMalformed) {
      RCLCPP_FATAL(get_logger(),
        "joint '%s' has an id that is not a whole number: '%s'",
        joint.name.c_str(), params::raw(jp, "id").c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (st != params::Status::kOk) {
      // Quote the declared text, not the parsed number. 254 is the broadcast id of the sync
      // writes and 255 (0xFF) the packet header byte.
      RCLCPP_FATAL(get_logger(),
        "joint '%s' has id %s, outside the range 1..253; 254 is the broadcast id the sync writes "
        "use and 255 is the packet header byte",
        joint.name.c_str(), params::raw(jp, "id").c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto seen = ids_seen.find(id);
    if (seen != ids_seen.end()) {
      // the parsed number, not the declared text: stoi_generic parses through std::stol, which
      // accepts a leading '+' and leading zeros, so '+001' and '1' are the same id written twice
      RCLCPP_FATAL(get_logger(),
        "joint '%s' has id %ld, which joint '%s' already uses; ids must be unique within a "
        "<ros2_control> block",
        joint.name.c_str(), id, seen->second.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    ids_seen.emplace(id, joint.name);
    // the range check above makes this cast exact
    c.id = static_cast<u8>(id);
    // 2. State interfaces: any subset of the nine, in any order, or none. Each must be known,
    // declared once and of data_type 'double' (else set_state throws later, in read()).
    std::vector<StateKind> seen_kinds;
    seen_kinds.reserve(joint.state_interfaces.size());
    bool declares_torque_alias = false;
    for (const hardware_interface::InterfaceInfo & si : joint.state_interfaces) {
      const auto kind = state_kind(si.name);
      if (!kind.has_value()) {
        RCLCPP_FATAL(get_logger(),
          "joint '%s' declares the unsupported state interface '%s'; supported names are position, "
          "velocity, effort, current, voltage, temperature, load, status and the deprecated torque",
          joint.name.c_str(), si.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (si.data_type != "double") {
        RCLCPP_FATAL(get_logger(),
          "joint '%s' declares the state interface '%s' with data_type '%s'; only 'double' is "
          "supported",
          joint.name.c_str(), si.name.c_str(), si.data_type.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (std::find(seen_kinds.begin(), seen_kinds.end(), *kind) != seen_kinds.end()) {
        RCLCPP_FATAL(get_logger(),
          "joint '%s' declares the state interface '%s' more than once",
          joint.name.c_str(), si.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      seen_kinds.emplace_back(*kind);
      declares_torque_alias = declares_torque_alias || (*kind == StateKind::kTorque);
    }
    if (declares_torque_alias) {
      // The deprecated alias keeps its original unit, kgf cm, so existing users see no change.
      RCLCPP_WARN(get_logger(),
        "joint '%s' declares the deprecated state interface 'torque' (kg cm); it keeps working and "
        "keeps reporting kg cm, but declare 'effort' instead, which reports N m",
        joint.name.c_str());
    }
    // 3. check presence and types of command interfaces
    if (joint.command_interfaces.size() < 1) {
      RCLCPP_FATAL(get_logger(),
        "a joint does not have a command interfaces");
      return hardware_interface::CallbackReturn::ERROR;
    }
    bool has_position_command = false;
    bool has_velocity_command = false;
    for (size_t ci = 0; ci < joint.command_interfaces.size(); ci++) {
      if (joint.command_interfaces[ci].name != hardware_interface::HW_IF_POSITION &&
        joint.command_interfaces[ci].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(get_logger(),
          "a joint is using a command interface that isn't position or velocity");
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (joint.command_interfaces[ci].name == hardware_interface::HW_IF_POSITION) {
        has_position_command = true;
      } else {
        has_velocity_command = true;
      }
    }
    // 4. Type: 'pos' (mode 0, goal position) or 'vel' (mode 1, wheel), declared or inferred
    // from the command interfaces; it must agree with them.
    if (jp.find("type") != jp.end()) {
      // case-sensitive: 'pos' and 'vel' are enum tokens, not English words
      const std::string declared_type = params::raw(jp, "type");
      if (declared_type == "pos") {
        c.type = JointType::position;
      } else if (declared_type == "vel") {
        c.type = JointType::velocity;
      } else {
        RCLCPP_FATAL(get_logger(),
          "joint '%s' has type '%s'; it must be 'pos' or 'vel', or left out so the driver infers "
          "it from the command interfaces",
          joint.name.c_str(), declared_type.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    } else {
      // a joint that takes a velocity command and no position command is a wheel; anything else
      // is driven to a goal position
      if (has_velocity_command && !has_position_command) {
        c.type = JointType::velocity;
      } else {
        c.type = JointType::position;
      }
      RCLCPP_DEBUG(get_logger(),
        "joint '%s' has no type param; inferred '%s' from its command interfaces",
        joint.name.c_str(), c.type == JointType::velocity ? "vel" : "pos");
    }
    if (c.type == JointType::position && !has_position_command) {
      RCLCPP_FATAL(get_logger(),
        "joint '%s' has type 'pos' but declares no position command interface; a position joint "
        "needs <command_interface name=\"position\"> (a velocity command interface only paces the "
        "move)",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (c.type == JointType::velocity && has_position_command) {
      RCLCPP_FATAL(get_logger(),
        "joint '%s' has type 'vel' but declares a position command interface; a velocity joint "
        "runs its servo in wheel mode and takes only <command_interface name=\"velocity\">",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    // 5. Position command limits; write() clamps to them (the controller manager does so only
    // with enforce_command_limits). stod rejects 'nan', so a bad limit cannot disable the clamp.
    double pos_min = -std::numeric_limits<double>::infinity();
    double pos_max = std::numeric_limits<double>::infinity();
    for (const hardware_interface::InterfaceInfo & ci : joint.command_interfaces) {
      if (ci.name != hardware_interface::HW_IF_POSITION) {
        continue;
      }
      try {
        if (!ci.min.empty()) {
          pos_min = hardware_interface::stod(ci.min);
        }
        if (!ci.max.empty()) {
          pos_max = hardware_interface::stod(ci.max);
        }
      } catch (const std::exception &) {
        RCLCPP_FATAL(get_logger(),
          "joint '%s' has a position min or max that is not a number", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    if (!(pos_min <= pos_max)) {
      RCLCPP_FATAL(get_logger(),
        "joint '%s' has a position min greater than its max", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (std::isfinite(pos_min) || std::isfinite(pos_max)) {
      RCLCPP_INFO(get_logger(),
        "joint '%s' position commands clamped to [%g, %g] rad", joint.name.c_str(), pos_min,
          pos_max);
    }
    c.pos_min = pos_min;
    c.pos_max = pos_max;
    // 6. offset (rad, servo frame): the servo angle of joint zero. It applies to wheels too.
    double offset = c.offset;
    st = params::get_double(
      jp, "offset", -std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(), false, offset);
    // belt and braces: hardware_interface::stod already rejects "nan" and "inf", so the isfinite
    // test can only fire if that ever changes
    if ((st != params::Status::kOk && st != params::Status::kDefaulted) || !std::isfinite(offset)) {
      RCLCPP_FATAL(get_logger(),
        "joint '%s' has an offset that is not a finite number: '%s'",
        joint.name.c_str(), params::raw(jp, "offset").c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    c.offset = offset;
    // 7. inverted: 'true' or 'false' only, stored as a +/-1.0 sign. It flips position,
    // velocity, load and both commands.
    bool inverted = false;
    st = params::get_bool(jp, "inverted", inverted);
    if (st != params::Status::kOk && st != params::Status::kDefaulted) {
      RCLCPP_FATAL(get_logger(),
        "joint '%s' has inverted='%s'; it must be 'true' or 'false'",
        joint.name.c_str(), params::raw(jp, "inverted").c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (inverted) {
      c.sign = -1.0;
    }
    // 8. max_speed, max_accel: magnitudes; when absent, the register values 6000 and 150 stay
    // (the acceleration scale is unverified). See docs/configuration.md, "Joint parameters".
    if (jp.find("max_speed") != jp.end()) {
      double max_speed = 0.0;
      st = params::get_double(
        jp, "max_speed", -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(), false, max_speed);
      if (st != params::Status::kOk || !std::isfinite(max_speed)) {
        RCLCPP_FATAL(get_logger(),
          "joint '%s' has a max_speed that is not a finite number: '%s'",
          joint.name.c_str(), params::raw(jp, "max_speed").c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (max_speed <= 0.0) {
        RCLCPP_FATAL(get_logger(),
          "joint '%s' has max_speed %g rad/s; it must be greater than 0",
          joint.name.c_str(), max_speed);
        return hardware_interface::CallbackReturn::ERROR;
      }
      const int64_t counts = saturating_lround(steps_from_rad(max_speed, encoder_steps_));
      if (counts < 1) {
        // the write path clamps the goal speed into [1, max_speed_counts], and std::clamp with
        // lo > hi is undefined behaviour
        RCLCPP_FATAL(get_logger(),
          "joint '%s' has max_speed %g rad/s, which is less than one encoder step per second "
          "(%g rad/s); raise it",
          joint.name.c_str(), max_speed, rad_from_steps(1.0, encoder_steps_));
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (counts > 32767) {
        // Bit 15 of the goal speed is the sign bit; above 32767 the position path would send a
        // negative speed.
        RCLCPP_WARN(get_logger(),
          "joint '%s' has max_speed %g rad/s, above the largest the goal speed register can hold "
          "(%g rad/s); using that instead",
          joint.name.c_str(), max_speed, rad_from_steps(32767.0, encoder_steps_));
        c.max_speed_counts = 32767;
      } else {
        c.max_speed_counts = static_cast<u16>(counts);
      }
    }
    if (jp.find("max_accel") != jp.end()) {
      double max_accel = 0.0;
      st = params::get_double(
        jp, "max_accel", -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(), false, max_accel);
      if (st != params::Status::kOk || !std::isfinite(max_accel)) {
        RCLCPP_FATAL(get_logger(),
          "joint '%s' has a max_accel that is not a finite number: '%s'",
          joint.name.c_str(), params::raw(jp, "max_accel").c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (max_accel < 0.0) {
        RCLCPP_FATAL(get_logger(),
          "joint '%s' has max_accel %g rad/s^2; it must be 0 (no acceleration limit) or greater",
          joint.name.c_str(), max_accel);
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (max_accel == 0.0) {
        // the explicit opt-out: 0 in the register means no ramp
        c.acc_counts = 0;
      } else {
        const int64_t counts =
          saturating_lround(accel_counts_from_rad_s2(max_accel, encoder_steps_));
        if (counts < 1) {
          // A small positive limit that rounds to 0 counts would silently mean 'no limit'.
          RCLCPP_FATAL(get_logger(),
            "joint '%s' has max_accel %g rad/s^2, which rounds to 0 acceleration-register counts; "
            "the smallest step is %g rad/s^2, and 0 means 'no acceleration limit'",
            joint.name.c_str(), max_accel, accel_rad_s2_from_counts(1.0, encoder_steps_));
          return hardware_interface::CallbackReturn::ERROR;
        }
        if (counts > 255) {
          RCLCPP_WARN(get_logger(),
            "joint '%s' has max_accel %g rad/s^2, above the largest the acceleration register can "
            "hold (%g rad/s^2); using that instead",
            joint.name.c_str(), max_accel, accel_rad_s2_from_counts(255.0, encoder_steps_));
          c.acc_counts = 255;
        } else {
          c.acc_counts = static_cast<u8>(counts);
        }
      }
    }
    // 9. unwrap: default true for 'vel'. FATAL for 'pos': the goal-speed pacing, the limit
    // check and the activation seed need the absolute single-turn position.
    bool unwrap = (c.type == JointType::velocity);
    st = params::get_bool(jp, "unwrap", unwrap);
    if (st != params::Status::kOk && st != params::Status::kDefaulted) {
      RCLCPP_FATAL(get_logger(),
        "joint '%s' has unwrap='%s'; it must be 'true' or 'false'",
        joint.name.c_str(), params::raw(jp, "unwrap").c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (unwrap && c.type == JointType::position) {
      RCLCPP_FATAL(get_logger(),
        "joint '%s' has unwrap=true with type pos; only a vel joint has a multi-turn position",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    c.unwrap = unwrap;
    if (c.unwrap) {
      RCLCPP_INFO(get_logger(),
        "joint '%s' reports an unwrapped, multi-turn position", joint.name.c_str());
    }
    // 10. Single-turn check (pos joints; needs offset, sign and limits): refuse limits that
    // map outside ticks [0, encoder_steps - 1], goals the servo would ignore.
    if (c.type == JointType::position) {
      const auto tick_of = [&c, this](double q) {
          return saturating_lround((c.sign * q + c.offset) * encoder_steps_ / (2 * M_PI));
        };
      const bool min_finite = std::isfinite(c.pos_min);
      const bool max_finite = std::isfinite(c.pos_max);
      if (min_finite && max_finite) {
        // an inverted joint maps pos_min to the upper tick, so the two swap
        const int64_t min_tick = tick_of(c.pos_min);
        const int64_t max_tick = tick_of(c.pos_max);
        const int64_t lo = std::min(min_tick, max_tick);
        const int64_t hi = std::max(min_tick, max_tick);
        if (lo < 0 || hi > encoder_steps_ - 1) {
          RCLCPP_FATAL(get_logger(),
            "joint '%s': position limits [%.4f, %.4f] rad with offset %.4f rad and inverted=%s "
            "map to servo ticks [%ld, %ld], outside the servo's single-turn range [0, %d]; change "
            "the offset, the limits, or 'inverted'",
            joint.name.c_str(), c.pos_min, c.pos_max, c.offset, c.sign < 0 ? "true" : "false",
            lo, hi, encoder_steps_ - 1);
          return hardware_interface::CallbackReturn::ERROR;
        }
      } else {
        // One WARN per joint. Finite limits are not required: stock descriptions have none.
        RCLCPP_WARN(get_logger(),
          "joint '%s' has type 'pos' but no finite position command limits, so its offset cannot "
          "be checked against the servo's single-turn range [0, %d] ticks; add "
          "<param name=\"min\"> and <param name=\"max\"> to its position command interface",
          joint.name.c_str(), encoder_steps_ - 1);
        if (min_finite || max_finite) {
          const char * side = min_finite ? "min" : "max";
          const double limit = min_finite ? c.pos_min : c.pos_max;
          const int64_t tick = tick_of(limit);
          if (tick < 0 || tick > encoder_steps_ - 1) {
            RCLCPP_FATAL(get_logger(),
              "joint '%s': position command %s %.4f rad with offset %.4f rad and inverted=%s maps "
              "to servo tick %ld, outside the servo's single-turn range [0, %d]; change the "
              "offset, that limit, or 'inverted'",
              joint.name.c_str(), side, limit, c.offset, c.sign < 0 ? "true" : "false", tick,
              encoder_steps_ - 1);
            return hardware_interface::CallbackReturn::ERROR;
          }
        } else {
          // with no limit at all it is joint zero itself that has to be reachable
          const int64_t zero_tick =
            saturating_lround(c.offset * encoder_steps_ / (2 * M_PI));
          if (zero_tick < 0 || zero_tick > encoder_steps_ - 1) {
            RCLCPP_FATAL(get_logger(),
              "joint '%s' has offset %.4f rad, which maps its zero position to servo tick %ld, "
              "outside the servo's single-turn range [0, %d]",
              joint.name.c_str(), c.offset, zero_tick, encoder_steps_ - 1);
            return hardware_interface::CallbackReturn::ERROR;
          }
        }
      }
    }
  }
  // The deprecated `torque` interface uses kgf cm: convert the constant once, here.
  torque_constant_kgfcm_per_a_ = kgfcm_per_amp(torque_constant_nm_per_a_);
  // All nine start at NaN, status too: NaN = no reply yet; status 0.0 = replied, no fault.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  pos_states_.resize(joints_.size(), nan);
  vel_states_.resize(joints_.size(), nan);
  eff_states_.resize(joints_.size(), nan);
  cur_states_.resize(joints_.size(), nan);
  volt_states_.resize(joints_.size(), nan);
  temp_states_.resize(joints_.size(), nan);
  load_states_.resize(joints_.size(), nan);
  status_states_.resize(joints_.size(), nan);
  torq_states_.resize(joints_.size(), nan);
  // the raw byte behind status_states_ and the fault edges; 0 is "no fault seen yet"
  status_bytes_.resize(joints_.size(), 0);
  // One unwrapper per joint (used or not), so all per-joint vectors share one index.
  unwrap_.assign(joints_.size(), PositionUnwrapper(encoder_steps_));
  unwrap_gap_warns_.assign(joints_.size(), 0);
  // Failed-read counters; on_activate zeroes them, so one activation is one measurement window.
  read_attempts_.assign(joints_.size(), 0);
  read_failures_.assign(joints_.size(), 0);
  // Reserve for every joint once: build_read_group() runs in read() and must not allocate.
  r_ids_.reserve(joints_.size());
  r_js_.reserve(joints_.size());
  r_blocks_.reserve(joints_.size());
  r_slot_.assign(joints_.size(), kNoSlot);
  // create vectors for command interfaces
  pos_cmds_.resize(joints_.size(), std::numeric_limits<double>::quiet_NaN());
  vel_cmds_.resize(joints_.size(), std::numeric_limits<double>::quiet_NaN());
  // no joint is held outside its limits until on_activate (or a park) finds one there
  hold_pos_.resize(joints_.size(), std::numeric_limits<double>::quiet_NaN());
  // One line per successful load saying how the joints resolved, so an inferred type is visible
  // without turning debug logging on.
  size_t position_joints = 0;
  size_t velocity_joints = 0;
  for (const JointConfig & c : joints_) {
    if (c.type == JointType::position) {
      position_joints++;
    } else {
      velocity_joints++;
    }
  }
  RCLCPP_INFO(get_logger(),
    "parsed %zu joints: %zu position (mode 0), %zu velocity (mode 1)",
    joints_.size(), position_joints, velocity_joints);
  return hardware_interface::CallbackReturn::SUCCESS;
}

namespace
{
// The handles for `names`, in that order. A name listed twice gets the same handle twice;
// a handle that no name asks for is left out.
template<typename Handle>
std::vector<Handle> handles_named(
  const std::vector<Handle> & interfaces, const std::vector<std::string> & names)
{
  std::unordered_map<std::string, Handle> by_name;
  for (const Handle & handle : interfaces) {
    by_name.emplace(handle->get_name(), handle);
  }
  std::vector<Handle> named;
  named.reserve(names.size());
  for (const std::string & name : names) {
    const auto it = by_name.find(name);
    if (it != by_name.end()) {
      named.emplace_back(it->second);
    }
  }
  return named;
}
}  // namespace

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
WaveshareServos::on_export_state_interfaces()
{
  // Export only the joints' state interfaces, joint by joint in description order (the
  // framework's list is in hash order and has interfaces this driver does not serve).
  const auto interfaces = hardware_interface::SystemInterface::on_export_state_interfaces();
  std::vector<std::string> names;
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    for (const hardware_interface::InterfaceInfo & state : joint.state_interfaces) {
      names.emplace_back(joint.name + "/" + state.name);
    }
  }
  return handles_named(interfaces, names);
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
WaveshareServos::on_export_command_interfaces()
{
  // same as on_export_state_interfaces
  const auto interfaces = hardware_interface::SystemInterface::on_export_command_interfaces();
  std::vector<std::string> names;
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    for (const hardware_interface::InterfaceInfo & command : joint.command_interfaces) {
      names.emplace_back(joint.name + "/" + command.name);
    }
  }
  return handles_named(interfaces, names);
}

hardware_interface::CallbackReturn WaveshareServos::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // the framework created the interface handles when the component was loaded; look them up once
  cache_handles();
  // Take the port exclusively; open() applies io_timeout_ms before the first transaction (the
  // library's 100 ms default would cost 100 ms per silent servo per cycle).
  const OpenResult opened = bus_.open(port_, baudrate_, io_timeout_ms_);
  if (!opened) {
    log_open_failure(opened);
    close_port();
    return hardware_interface::CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(get_logger(),
    "bus on '%s' at %d baud, exclusive (TIOCEXCL and flock), io timeout %u ms",
    port_.c_str(), baudrate_, io_timeout_ms_);
  const std::string others = other_holders_of(port_);
  if (!others.empty()) {
    RCLCPP_WARN(get_logger(),
      "process(es) %s already had '%s' open when this component took it. They hold no lock, so "
      "they can still write to the bus: two programs on one servo bus produce garbage reads and a "
      "joint that drifts. Stop them before running the controllers.", others.c_str(),
      port_.c_str());
  }
  if (::geteuid() == 0) {
    RCLCPP_WARN(get_logger(),
      "running as root: the kernel lets a root process open '%s' even though it is marked "
      "exclusive, so only the advisory lock protects this bus, and only against programs that "
      "take it. This package's scan, set_id and calibrate_midpoint take it; screen and minicom do "
      "not.",
      port_.c_str());
  }
  // ping motors and remember which ones are actually on the bus
  present_.assign(joints_.size(), false);
  read_fails_.assign(joints_.size(), 0);
  last_error_.assign(joints_.size(), 0);
  // nothing is measured on this port yet: a sample from before a cleanup may be stale
  measured_.assign(joints_.size(), false);
  // the continuous count is the same kind of session state: it belongs to one open port
  reset_unwrap();
  for (size_t i = 0; i < joints_.size(); i++) {
    for (int attempt = 0; attempt < ping_attempts_ && !present_[i]; attempt++) {
      present_[i] = (bus_.Ping(joints_[i].id) != -1);
    }
    if (!present_[i]) {
      RCLCPP_WARN(get_logger(),
        "unable to ping motor id '%d'; joint '%s' will be skipped on the bus",
        joints_[i].id, info_.joints[i].name.c_str());
    }
  }
  // Check before build_groups(): set_mode() may rewrite EPROM register 33, and a refused
  // configuration must not use up EPROM write cycles.
  std::vector<std::string> missing;
  for (size_t i = 0; i < joints_.size(); i++) {
    if (!present_[i]) {
      missing.emplace_back(
        "id " + std::to_string(joints_[i].id) + " (joint '" + info_.joints[i].name + "')");
    }
  }
  if (!missing.empty()) {
    const std::string list = join(missing, ", ");
    if (!allow_missing_servos_) {
      RCLCPP_FATAL(get_logger(),
        "%zu of %zu servos did not answer: %s; refusing to configure because "
        "'allow_missing_servos' is false",
        missing.size(), joints_.size(), list.c_str());
      // A configure failure gets no on_error or on_cleanup, so close the port here.
      close_port();
      return hardware_interface::CallbackReturn::FAILURE;
    }
    RCLCPP_WARN(get_logger(),
      "continuing without %zu of %zu servos because 'allow_missing_servos' is true: %s; "
      "their joints mirror their commands into their states until the servos answer",
      missing.size(), joints_.size(), list.c_str());
  }
  build_groups();
  return hardware_interface::CallbackReturn::SUCCESS;
}

void WaveshareServos::build_groups()
{
  // Command groups of the servos that answered, each in URDF order.
  p_ids_.clear();
  p_js_.clear();
  v_ids_.clear();
  v_js_.clear();
  for (size_t i = 0; i < joints_.size(); i++) {
    if (!present_[i]) {
      continue;
    }
    if (joints_[i].type == JointType::position) {
      p_ids_.emplace_back(joints_[i].id);
      p_js_.emplace_back(i);
    } else {
      v_ids_.emplace_back(joints_[i].id);
      v_js_.emplace_back(i);
    }
  }
  // one goal record per servo in each group, refilled every cycle by send_commands()
  p_goals_.assign(p_ids_.size(), GoalPosition{});
  v_goals_.assign(v_ids_.size(), GoalSpeed{});
  // set motor modes: 0 = servo, 1 = closed loop wheel
  for (size_t k = 0; k < p_ids_.size(); k++) {
    set_mode(p_ids_[k], 0);
  }
  for (size_t k = 0; k < v_ids_.size(); k++) {
    set_mode(v_ids_[k], 1);
    // A mode write clears torque enable (register 40); on_activate enables torque after this.
    // See docs/design.md, "Servo registers".
    write_wheel_acceleration(v_js_[k]);
  }
  // Size the wrapper's packet scratch now so write() does not allocate (capped at one packet).
  bus_.reserve_goal_capacity(p_ids_.size(), v_ids_.size());
  // Last, so all three groups come from the same present_. The read group has its own builder
  // because read() rebuilds it and this function does bus traffic.
  build_read_group();
}

void WaveshareServos::build_read_group()
{
  r_ids_.clear();
  r_js_.clear();
  r_slot_.assign(joints_.size(), kNoSlot);
  for (size_t i = 0; i < joints_.size(); i++) {
    if (!present_[i]) {
      continue;
    }
    r_slot_[i] = r_ids_.size();
    r_ids_.emplace_back(joints_[i].id);
    r_js_.emplace_back(i);
  }
  // assign(), not resize(): every slot starts valid == false, so a burst that never ran cannot
  // leave a previous cycle's sample readable through a slot the new group happens to reuse.
  r_blocks_.assign(r_ids_.size(), FeedbackBlock{});
}

bool WaveshareServos::probe_sync_read()
{
  // Rebuild first: on_activate calls build_groups() only when it recovers a servo.
  build_read_group();
  sync_read_active_ = false;
  // 'per_servo' sends no INST_SYNC_READ, not even a probe. With no servo present, no
  // transport is used.
  if (feedback_mode_ == "per_servo" || r_ids_.empty()) {
    return true;
  }
  // Judge only servos whose seeding read answered (measured_). Probe the whole group, not
  // one id: a firmware may answer a 1-id burst and fail a 4-id burst.
  std::vector<std::string> unanswered;
  // Two tries at most: one lost frame is noise, a second loss is evidence.
  for (int attempt = 0; attempt < 2 && (attempt == 0 || !unanswered.empty()); attempt++) {
    unanswered.clear();
    bus_.sync_read_feedback(r_ids_, r_blocks_);
    for (size_t k = 0; k < r_ids_.size(); k++) {
      if (measured_[r_js_[k]] && !r_blocks_[k].valid) {
        unanswered.emplace_back(std::to_string(static_cast<int>(r_ids_[k])));
      }
    }
  }
  if (unanswered.empty()) {
    sync_read_active_ = true;
    // Once per activation: the mode does not change later. Tests match this text.
    RCLCPP_INFO(get_logger(),
      "feedback for %zu servos travels in one sync read per cycle (INST_SYNC_READ)",
      r_ids_.size());
    return true;
  }
  const std::string ids = join(unanswered, ", ");
  if (feedback_mode_ == "sync_read") {
    // 'sync_read' rules out the per-servo path, so refuse to activate instead of falling back.
    RCLCPP_FATAL(get_logger(),
      "sync read went unanswered by motor id(s) %s, which answered a feedback read moments "
      "earlier, so the firmware ignores INST_SYNC_READ; feedback_mode is 'sync_read', which rules "
      "out the per-servo path, so refusing to activate. Use 'auto' to fall back instead",
      ids.c_str());
    return false;
  }
  // Name the ids that answered a feedback read but not INST_SYNC_READ (a firmware limit).
  // Tests match this text.
  RCLCPP_WARN(get_logger(),
    "sync read went unanswered by motor id(s) %s; falling back to one feedback read per servo "
    "for this activation", ids.c_str());
  return true;
}

void WaveshareServos::log_bus_totals()
{
  if (read_stats_reported_) {
    return;
  }
  read_stats_reported_ = true;
  // on_error can come from INACTIVE with no cycle run; an all-zero line would only be noise.
  if (read_cycles_ == 0) {
    return;
  }
  // One transaction = one wrapper read call: one sync_read_feedback() per cycle, or one
  // read_feedback_one() per polled joint. See docs/bus-timing.md, "Bus totals line".
  const double per_million = read_transactions_ ?
    1e6 * static_cast<double>(read_transaction_failures_) /
    static_cast<double>(read_transactions_) :
    0.0;
  // Tail: 'idN count' per declared servo, in id order. It counts per joint, so it sums to
  // `failed` only while no burst lost two frames.
  std::vector<size_t> by_id;
  by_id.reserve(joints_.size());
  for (size_t i = 0; i < joints_.size(); i++) {
    by_id.push_back(i);
  }
  std::sort(
    by_id.begin(), by_id.end(),
    [this](size_t a, size_t b) {return joints_[a].id < joints_[b].id;});
  std::string tail;
  for (const size_t i : by_id) {
    if (!tail.empty()) {
      tail += ", ";
    }
    tail += "id" + std::to_string(joints_[i].id) + " " + std::to_string(read_failures_[i]);
  }
  // Tests and test/hil/hil_gates.py (BUS_TOTALS) parse this line: do not change its format.
  RCLCPP_INFO(get_logger(),
    "bus totals: transactions %" PRIu64 ", failed %" PRIu64 " (%.1f per million), worst "
    "consecutive %" PRIu32 ", dropped %" PRIu32 " [%s]",
    read_transactions_, read_transaction_failures_, per_million, worst_consecutive_,
    dropped_count_,
    tail.c_str());
}

void WaveshareServos::write_wheel_acceleration(size_t i)
{
  // Skip absent servos (an unanswered write costs a full timeout) and position joints (every
  // goal record carries their acceleration).
  if (!present_[i] || joints_[i].type != JointType::velocity) {
    return;
  }
  // Write 0 too: max_accel="0" means no ramp, and skipping it would keep the servo's old value.
  if (!bus_.write_acc(joints_[i].id, joints_[i].acc_counts)) {
    // No retry: a failed write costs a full timeout, and the next call site retries. WARN,
    // because an unramped wheel has no other symptom. Tests match this text.
    RCLCPP_WARN(get_logger(),
      "could not write the acceleration register of motor id '%d'; it will run unramped",
      joints_[i].id);
  }
}

bool WaveshareServos::set_mode(u8 id, u8 mode)
{
  // Register 33 is EPROM: a write while the lock (55) is set is applied but lost at power-off,
  // and EPROM wears out, so unlock and write only when the mode is wrong.
  const int current = bus_.readByte(id, SMS_STS_MODE);
  if (current == -1) {
    RCLCPP_WARN(get_logger(),
      "could not read the mode of motor id '%d'", id);
    return false;
  }
  if (current == mode) {
    return true;
  }
  bus_.unLockEprom(id);
  bus_.Mode(id, mode);
  bus_.LockEprom(id);
  const int now = bus_.readByte(id, SMS_STS_MODE);
  if (now != mode) {
    RCLCPP_ERROR(get_logger(),
      "failed to set motor id '%d' to mode %d, it is still in mode %d", id, mode, now);
    return false;
  }
  RCLCPP_INFO(get_logger(),
    "motor id '%d' mode changed from %d to %d", id, current, mode);
  return true;
}

void WaveshareServos::cache_handles()
{
  // The framework creates the handles after on_init; look them up by name, since its lists
  // are not in URDF order. on_init already checked every name.
  handles_.assign(info_.joints.size(), JointHandles{});
  for (size_t i = 0; i < info_.joints.size(); i++) {
    const hardware_interface::ComponentInfo & joint = info_.joints[i];
    const std::string & name = joint.name;
    JointHandles & h = handles_[i];
    // in the description's order, so read() publishes in that order too
    h.states.reserve(joint.state_interfaces.size());
    for (const hardware_interface::InterfaceInfo & si : joint.state_interfaces) {
      h.states.emplace_back(
        *state_kind(si.name), get_state_interface_handle(name + "/" + si.name));
    }
    const std::string pos_cmd = name + "/" + hardware_interface::HW_IF_POSITION;
    const std::string vel_cmd = name + "/" + hardware_interface::HW_IF_VELOCITY;
    if (has_command(pos_cmd)) {
      h.position_cmd = get_command_interface_handle(pos_cmd);
    }
    if (has_command(vel_cmd)) {
      h.velocity_cmd = get_command_interface_handle(vel_cmd);
    }
  }
}

void WaveshareServos::pull_commands(size_t i, bool wait)
{
  // A busy handle (non-blocking get) keeps last cycle's command: no NaN, no jump. An
  // undeclared interface has no handle.
  const JointHandles & h = handles_[i];
  if (h.position_cmd) {
    std::ignore = get_command(h.position_cmd, pos_cmds_[i], wait);
  }
  if (h.velocity_cmd) {
    std::ignore = get_command(h.velocity_cmd, vel_cmds_[i], wait);
  }
}

void WaveshareServos::push_position_command(size_t i)
{
  // blocking: only called from lifecycle callbacks, which read() and write() cannot overlap
  const JointHandles & h = handles_[i];
  if (h.position_cmd) {
    std::ignore = set_command(h.position_cmd, pos_cmds_[i], true);
  }
}

void WaveshareServos::push_velocity_command(size_t i)
{
  // blocking: only called from lifecycle callbacks, which read() and write() cannot overlap
  const JointHandles & h = handles_[i];
  if (h.velocity_cmd) {
    std::ignore = set_command(h.velocity_cmd, vel_cmds_[i], true);
  }
}

double WaveshareServos::state_value(size_t i, StateKind kind) const
{
  switch (kind) {
    case StateKind::kPosition:
      return pos_states_[i];
    case StateKind::kVelocity:
      return vel_states_[i];
    case StateKind::kEffort:
      return eff_states_[i];
    case StateKind::kCurrent:
      return cur_states_[i];
    case StateKind::kVoltage:
      return volt_states_[i];
    case StateKind::kTemperature:
      return temp_states_[i];
    case StateKind::kLoad:
      return load_states_[i];
    case StateKind::kStatus:
      return status_states_[i];
    case StateKind::kTorque:
      return torq_states_[i];
  }
  // No `default:` label above, so a tenth StateKind is a -Wswitch warning rather than a silent
  // NaN; this return is only here to satisfy -Wreturn-type.
  return std::numeric_limits<double>::quiet_NaN();
}

void WaveshareServos::push_states(size_t i, bool wait)
{
  // A busy handle (non-blocking set) keeps the previous sample until the next read(). Only
  // the declared interfaces have a handle.
  const JointHandles & h = handles_[i];
  for (const auto & entry : h.states) {
    std::ignore = set_state(entry.second, state_value(i, entry.first), wait);
  }
}

hardware_interface::CallbackReturn WaveshareServos::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // One activation is one measurement window: reset the counters first, before the re-ping.
  read_cycles_ = 0;
  read_transactions_ = 0;
  read_transaction_failures_ = 0;
  read_stats_reported_ = false;
  worst_consecutive_ = 0;
  dropped_count_ = 0;
  read_attempts_.assign(joints_.size(), 0);
  read_failures_.assign(joints_.size(), 0);
  // Reset read_fails_ too: it defines worst_consecutive_, and it is the drop budget, which a
  // re-activation must give back in full.
  read_fails_.assign(joints_.size(), 0);
  // Off the real-time path: ping absent and dropped servos again, so deactivate + activate
  // recovers a servo without a restart.
  bool regrouped = false;
  for (size_t i = 0; i < joints_.size(); i++) {
    if (present_[i]) {
      continue;
    }
    for (int attempt = 0; attempt < ping_attempts_ && !present_[i]; attempt++) {
      present_[i] = (bus_.Ping(joints_[i].id) != -1);
    }
    if (present_[i]) {
      RCLCPP_INFO(get_logger(),
        "motor id '%d' answered on activation; adding it back", joints_[i].id);
      read_fails_[i] = 0;
      regrouped = true;
    }
  }
  if (regrouped) {
    build_groups();
  }
  // Seed the commands from the current positions so activation moves nothing. Each goes to
  // its handle at once; an active controller can still overwrite it.
  for (size_t i = 0; i < joints_.size(); i++) {
    vel_cmds_[i] = 0.0;
    push_velocity_command(i);
    hold_pos_[i] = std::numeric_limits<double>::quiet_NaN();
    // Keep a seeded wheel count across deactivate/activate (the controllers are not told); only
    // a discarded count restarts. See docs/configuration.md, "Multi-turn wheel position".
    if (!unwrap_[i].seeded()) {
      reset_unwrap(i);
    }
    if (present_[i]) {
      // A servo whose torque has been latched off -- by a protection trip, or by whatever
      // last talked to it -- accepts goal positions and quietly ignores them.
      bus_.EnableTorque(joints_[i].id, 1);
      // Write register 41 again: a wheel power-cycled while INACTIVE has lost it. A recovered
      // servo gets it twice (also in build_groups()); tests must not expect exactly one write.
      write_wheel_acceleration(i);
    }
    if (present_[i] && feedback(i)) {
      pos_cmds_[i] = pos_states_[i];
      // A seeding read that answered clears the failure count, so an old silence is not charged
      // to the first read().
      read_fails_[i] = 0;
      // A joint that starts outside its limits is held where it is, not run to the nearest
      // limit, until it is commanded inside them.
      if (outside_limits(i, pos_states_[i])) {
        hold_pos_[i] = pos_states_[i];
        RCLCPP_WARN(get_logger(),
          "joint '%s' starts at %.3f rad, outside its limits [%.3f, %.3f]; holding it there "
          "until it is commanded to a position inside them", info_.joints[i].name.c_str(),
          pos_states_[i], joints_[i].pos_min, joints_[i].pos_max);
      }
    } else {
      // nothing on the bus to read, so start from a neutral command rather than from the
      // -1 that a timed-out read would otherwise hand us
      pos_cmds_[i] = 0.0;
      pos_states_[i] = 0.0;
      measured_[i] = false;
      vel_states_[i] = 0.0;
      eff_states_[i] = 0.0;
      cur_states_[i] = 0.0;
      volt_states_[i] = 0.0;
      temp_states_[i] = 0.0;
      load_states_[i] = 0.0;
      torq_states_[i] = 0.0;
      // no reading is not "no fault"
      status_states_[i] = std::numeric_limits<double>::quiet_NaN();
      // and a fault from before this activation must not fire a clear edge on the next read
      last_error_[i] = 0;
      status_bytes_[i] = 0;
    }
    // hand these to the controllers: a controller activated next starts from these commands
    push_position_command(i);
    push_states(i, true);
  }
  // Probe after the seeding loop, when present_ and measured_ are final; inside the loop it
  // would run once per joint.
  if (!probe_sync_read()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WaveshareServos::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // On SIGINT/SIGTERM the controller manager deactivates the hardware before it shuts it down,
  // so this also runs on ctrl-C.
  stop_and_park(false);
  log_bus_totals();
  // The transport choice belongs to one activation; the next on_activate probes again.
  sync_read_active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool WaveshareServos::outside_limits(size_t i, double position) const
{
  // Only a position joint has limits; an unwrapped wheel's position is unbounded.
  if (joints_[i].type != JointType::position) {
    return false;
  }
  // allow a step of encoder rounding
  const double step = 2 * M_PI / encoder_steps_;
  return (position < joints_[i].pos_min - step) || (position > joints_[i].pos_max + step);
}

void WaveshareServos::reset_unwrap()
{
  for (size_t i = 0; i < unwrap_.size(); i++) {
    reset_unwrap(i);
  }
}

void WaveshareServos::reset_unwrap(size_t i)
{
  // The throttle counter is always reset with the count: they describe the same session, and a
  // stale counter would silence the first gap of the next one.
  unwrap_[i].reset();
  unwrap_gap_warns_[i] = 0;
}

int64_t WaveshareServos::unwrap_ticks(size_t i, int32_t raw)
{
  if (!joints_[i].unwrap) {
    return raw;
  }
  return unwrap_[i].update(raw);
}

double WaveshareServos::max_speed_rad(size_t i) const
{
  return rad_from_steps(joints_[i].max_speed_counts, encoder_steps_);
}

void WaveshareServos::note_unwrap_gap(size_t i, int missed, double period_s)
{
  if (!joints_[i].unwrap || !unwrap_[i].bridged()) {
    return;
  }
  // Elapsed time: the period if every read answered, else (missed + 1) cycles of at least
  // one io_timeout_ms each. The timeout is charged only to failed cycles.
  const double cycle = std::max(last_period_, static_cast<double>(io_timeout_ms_) / 1000.0);
  const double gap = (missed == 0) ? period_s : std::max(period_s, cycle * (missed + 1));
  // Use the speed ceiling: the speed while silent is unknown. Hence 'may be' in the WARN.
  const double possible = max_speed_rad(i) * gap;
  if (possible < M_PI) {
    return;
  }
  unwrap_gap_warns_[i]++;
  if (unwrap_gap_warns_[i] == 1 || unwrap_gap_warns_[i] % 200 == 0) {
    RCLCPP_WARN(get_logger(),
      "joint '%s' went %.3f s without a reading; at %.2f rad/s it could have turned %.2f rad while "
      "it was silent, so its unwrapped position may be off by whole revolutions (%d such gaps)",
      info_.joints[i].name.c_str(), gap, max_speed_rad(i), possible, unwrap_gap_warns_[i]);
  }
}

void WaveshareServos::stop_and_park(bool at_measured_positions)
{
  // start from the commands the controllers left in the handles
  for (size_t i = 0; i < joints_.size(); i++) {
    pull_commands(i, true);
  }
  // Zero the wheel commands and push them to the handles now; an active controller can
  // still overwrite them before write() reads the handles.
  for (size_t i = 0; i < vel_cmds_.size(); i++) {
    vel_cmds_[i] = 0.0;
    push_velocity_command(i);
  }
  // and park the position joints where they actually are, so shutdown cannot run them off to
  // a goal they had not reached yet
  for (size_t i = 0; i < joints_.size(); i++) {
    const bool measured = present_[i] && feedback(i);
    if (measured) {
      pos_cmds_[i] = pos_states_[i];
      // feedback() refreshed every state cache; only then are they published, as before
      push_states(i, true);
    }
    if (at_measured_positions && present_[i]) {
      // Park only at a measured position (this read or the last since on_configure, else NaN),
      // and hold a joint outside its limits there, as on_activate does.
      if (!measured) {
        pos_cmds_[i] = measured_[i] ? pos_states_[i] : std::numeric_limits<double>::quiet_NaN();
      }
      hold_pos_[i] = std::numeric_limits<double>::quiet_NaN();
      if (outside_limits(i, pos_cmds_[i])) {
        hold_pos_[i] = pos_cmds_[i];
      }
    }
    // the handles hold the parked command of every joint that has one; a joint that is not parked
    // keeps the controller's command in its handle, which write() below takes, as before
    if (measured || (at_measured_positions && present_[i])) {
      push_position_command(i);
    }
  }
  if (at_measured_positions) {
    // Leave out servos with no measured position or dropped by read(): send no made-up goal.
    // The port closes next, which clears the groups.
    size_t kept = 0;
    for (size_t k = 0; k < p_ids_.size(); k++) {
      const size_t j = p_js_[k];
      if (present_[j] && measured_[j] && std::isfinite(pos_cmds_[j])) {
        p_ids_[kept] = p_ids_[k];
        p_js_[kept] = p_js_[k];
        kept++;
      }
    }
    p_ids_.resize(kept);
    p_js_.resize(kept);
    // Send from these values, not the handles, so a controller that still commands cannot
    // replace the stop.
    send_commands(rclcpp::Duration(0, 0));
    return;
  }
  const rclcpp::Clock::SharedPtr clock = get_clock();
  auto now = clock ? clock->now() : rclcpp::Time(0, 0, RCL_STEADY_TIME);
  auto period = rclcpp::Duration(0, 0);    // zero duration
  this->write(now, period);
}

hardware_interface::return_type WaveshareServos::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Elapsed time of this cycle (zero or non-finite: last period). The gap warning uses it
  // to see a descheduled loop, which a failed-read count cannot show.
  double period_s = period.seconds();
  if (!(std::isfinite(period_s) && period_s > 0.0)) {
    period_s = last_period_;
  }
  // One sync-read burst for all present servos (4 servos: ~1.6 ms vs ~3.0 ms one by one).
  // Each slot's `valid` flag decides: a burst that lost one frame still has the others.
  const bool burst_issued = sync_read_active_ && !r_ids_.empty();
  if (burst_issued) {
    bus_.sync_read_feedback(r_ids_, r_blocks_);
    // The burst is one transaction, whatever the servo count; the per-servo path counts below.
    read_transactions_++;
  }
  read_cycles_++;
  // A burst is charged at most one failure, however many frames it lost.
  bool burst_failed = false;
  for (size_t i = 0; i < joints_.size(); i++) {
    if (!present_[i]) {
      // No servo on the bus for this joint. Mirror the command so the controllers see a
      // finite, self-consistent state instead of a timeout sentinel, and stay off the wire.
      pull_commands(i, false);
      pos_states_[i] = std::isfinite(pos_cmds_[i]) ? pos_cmds_[i] : 0.0;
      measured_[i] = false;
      vel_states_[i] = 0.0;
      eff_states_[i] = 0.0;
      cur_states_[i] = 0.0;
      volt_states_[i] = 0.0;
      temp_states_[i] = 0.0;
      load_states_[i] = 0.0;
      torq_states_[i] = 0.0;
      // no reading is not "no fault"
      status_states_[i] = std::numeric_limits<double>::quiet_NaN();
      // and the fault this servo last reported must not fire a clear edge when it comes back
      last_error_[i] = 0;
      status_bytes_[i] = 0;
      continue;
    }
    // After the absent branch: an absent joint is never counted.
    read_attempts_[i]++;
    // On the per-servo path each poll is a transaction of its own.
    if (!sync_read_active_) {
      read_transactions_++;
    }
    if (!feedback_from_cycle(i)) {
      // The round trip failed. Keep the last good sample: publishing the -1 that a timed
      // out read returns would look like a real measurement 0.0015 rad from the origin.
      read_failures_[i]++;
      if (!sync_read_active_) {
        // Its own transaction failed, and only its own: the other joints' unicast reads are
        // unaffected by this one, which is the difference the shared burst cannot make.
        read_transaction_failures_++;
      } else if (burst_issued) {
        // Charged once after the loop, and only if a burst went out, so failed <= transactions.
        burst_failed = true;
      }
      read_fails_[i]++;
      // Record the peak now: any success clears read_fails_.
      worst_consecutive_ = std::max(worst_consecutive_, static_cast<uint32_t>(read_fails_[i]));
      if (read_fails_[i] == 1 || read_fails_[i] % 200 == 0) {
        RCLCPP_WARN(get_logger(),
          "read failed for motor id '%d' (%d in a row)", joints_[i].id, read_fails_[i]);
      }
      if (read_fails_[i] >= max_read_fails_) {
        // Drop it after max_read_fails consecutive failures (cycles, not seconds); re-activation
        // looks for it again. See docs/operation.md, "Recovery after a servo is lost".
        RCLCPP_ERROR(get_logger(),
          "motor id '%d' stopped answering after %d attempts; dropping it from the "
          "read cycle until the hardware is re-activated", joints_[i].id, read_fails_[i]);
        present_[i] = false;
        dropped_count_++;
        // Rebuild the read group after the loop (see read_group_dirty_). The write groups keep
        // the servo: a sync write is never acknowledged, so it costs only record bytes.
        read_group_dirty_ = true;
        reset_unwrap(i);
        if (joints_[i].unwrap) {
          // A wheel's pos_cmds_ still holds its activation position: copy the last reading,
          // so the absent-branch mirror does not jump back.
          pos_cmds_[i] = pos_states_[i];
        }
      }
      continue;
    }
    // Report an ended gap before the count is cleared. Every success runs it: a descheduled
    // loop gives missed == 0 and is a gap too.
    const int missed = read_fails_[i];
    read_fails_[i] = 0;
    note_unwrap_gap(i, missed, period_s);
    if (missed > 0) {
      // Answered after a silence: a wheel may have power-cycled and lost register 41, so write
      // it again (one short write, on this abnormal cycle only).
      write_wheel_acceleration(i);
    }
    // Use the status byte captured with the sample: SCS::Error is overwritten by every Ack
    // (e.g. the EnableTorque below).
    const u8 status = status_bytes_[i];
    if (status != 0 && status != last_error_[i]) {
      RCLCPP_WARN(get_logger(), "motor id '%d' reports status 0x%02x (%s)",
        joints_[i].id, status, status_text(status).c_str());
    } else if (status == 0 && last_error_[i] != 0) {
      // The trip cleared. A protection trip latches torque off, and a servo in that state
      // still answers reads and still accepts goal positions -- it just ignores them.
      RCLCPP_INFO(get_logger(),
        "motor id '%d' cleared its fault; re-enabling torque", joints_[i].id);
      bus_.EnableTorque(joints_[i].id, 1);
      // A trip that resets the servo may clear register 41 with no missed read (not measured),
      // so write it again here too.
      write_wheel_acceleration(i);
    }
    last_error_[i] = status;
  }
  if (burst_failed) {
    read_transaction_failures_++;
  }
  if (read_group_dirty_) {
    build_read_group();
    read_group_dirty_ = false;
  }
  // publish every joint's state, including the last good sample of a joint whose read failed
  for (size_t i = 0; i < joints_.size(); i++) {
    push_states(i, false);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WaveshareServos::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // take this cycle's commands from the controllers
  for (size_t i = 0; i < joints_.size(); i++) {
    pull_commands(i, false);
  }
  send_commands(period);
  return hardware_interface::return_type::OK;
}

void WaveshareServos::send_commands(const rclcpp::Duration & period)
{
  // The servo stops dead at each goal, so pace the goal speed to arrive as the next setpoint
  // comes, using the real time since the last write. See docs/design.md, "Goal speed".
  double dt = period.seconds();
  if (std::isfinite(dt) && dt > 0.0) {
    last_period_ = dt;
  } else {
    dt = last_period_;
  }
  // stop_and_park(true) may shrink p_ids_; resize the records to match (no allocation in
  // steady state).
  p_goals_.resize(p_ids_.size());
  v_goals_.resize(v_ids_.size());
  for (size_t k = 0; k < p_ids_.size(); k++) {
    const size_t j = p_js_[k];
    // Until the first controller update the command is still NaN; hold station rather than
    // converting NaN to a garbage step count.
    double cmd = pos_cmds_[j];
    if (!std::isfinite(cmd)) {
      cmd = std::isfinite(pos_states_[j]) ? pos_states_[j] : 0.0;
    }
    // Clamp to the joint limits (the servo's own angle limits are raw ticks without the
    // offset). A joint held outside them stays put until commanded inside.
    if (std::isfinite(hold_pos_[j]) &&
      ((cmd < joints_[j].pos_min) || (cmd > joints_[j].pos_max)))
    {
      cmd = hold_pos_[j];
    } else {
      hold_pos_[j] = std::numeric_limits<double>::quiet_NaN();
      cmd = std::clamp(cmd, joints_[j].pos_min, joints_[j].pos_max);
    }
    // The exact inverse of the read path: sign, then offset, then the tick scale.
    const double goal_steps =
      (joints_[j].sign * cmd + joints_[j].offset) * encoder_steps_ / (2 * M_PI);
    const s16 goal_ticks = static_cast<s16>(std::lround(std::clamp(goal_steps, -32767.0, 32767.0)));
    // Speed = |goal - measured| / dt (chord plus lag). Not the trajectory velocity: on an
    // accelerating segment the servo would arrive early and stall.
    double speed = 0.0;
    if (std::isfinite(pos_states_[j])) {
      const double now_steps =
        (joints_[j].sign * pos_states_[j] + joints_[j].offset) * encoder_steps_ / (2 * M_PI);
      speed = std::fabs(goal_steps - now_steps) / dt;
    } else if (std::isfinite(vel_cmds_[j])) {
      // no usable measurement this cycle, so fall back to the commanded velocity
      speed = std::fabs(vel_cmds_[j]) * encoder_steps_ / (2 * M_PI);
    }
    if (!std::isfinite(speed)) {
      speed = 0.0;
    }
    // Goal speed 0 means full speed, so floor it at 1 (0 made the servo lurch at trajectory
    // ends). Direction comes from the goal position; too high a speed only arrives early.
    const u16 goal_speed = static_cast<u16>(
      std::clamp(speed, 1.0, static_cast<double>(joints_[j].max_speed_counts)));
    // Acceleration rides in every position record (0 too), so a restarted position servo
    // gets it back on the next cycle.
    p_goals_[k] = GoalPosition{p_ids_[k], goal_ticks, goal_speed, joints_[j].acc_counts};
  }
  for (size_t k = 0; k < v_ids_.size(); k++) {
    const size_t j = v_js_[k];
    const double vel = std::isfinite(vel_cmds_[j]) ? vel_cmds_[j] : 0.0;
    const double speed = std::clamp(joints_[j].sign * vel * encoder_steps_ / (2 * M_PI),
      -static_cast<double>(joints_[j].max_speed_counts),
      static_cast<double>(joints_[j].max_speed_counts));
    // No acceleration here: register 41 is written at four edges, not every cycle (saves
    // about 0.57 ms per wheel per cycle). See docs/design.md, "Wheel acceleration".
    v_goals_[k] = GoalSpeed{v_ids_[k], static_cast<s16>(std::lround(speed))};
  }
  // ServoBus splits each group into packets (30 position / 82 speed records) that fit the
  // 255-byte txBuf; an empty group sends nothing.
  const WriteResult positions = bus_.write_goal_positions(p_goals_);
  const WriteResult speeds = bus_.write_goal_speeds(v_goals_);
  if (!positions || !speeds) {
    // Refused: nothing was sent and a wheel keeps turning, so ERROR (throttled). Still return
    // OK: on_error's park-and-close is too heavy for a closed port or a bad id.
    write_refusals_++;
    if (write_refusals_ == 1 || write_refusals_ % 200 == 0) {
      RCLCPP_ERROR(get_logger(), "sync write refused: positions %s, speeds %s (%d in a row)",
        to_string(positions.status), to_string(speeds.status), write_refusals_);
    }
  } else {
    write_refusals_ = 0;
  }
}

hardware_interface::CallbackReturn WaveshareServos::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close_port();
  return hardware_interface::CallbackReturn::SUCCESS;
}

void WaveshareServos::close_port()
{
  // ServoBus::close() is safe in any state: it clears the exclusive flag, closes the library's
  // descriptor and releases the lock, each only if it holds it.
  bus_.close();
  // the continuous count belongs to a measurement session on an open port; there is none now
  reset_unwrap();
  // release the command buffers together with the groups that index them; on_configure rebuilds
  // both
  p_ids_.clear();
  p_js_.clear();
  v_ids_.clear();
  v_js_.clear();
  // The wrapper's packet scratch keeps its capacity: it holds no state between calls.
  p_goals_.clear();
  v_goals_.clear();
  // Clear the read group and the transport flag too; r_slot_ keeps one entry per joint.
  r_ids_.clear();
  r_js_.clear();
  r_blocks_.clear();
  r_slot_.assign(joints_.size(), kNoSlot);
  read_group_dirty_ = false;
  sync_read_active_ = false;
}

void WaveshareServos::log_open_failure(const OpenResult & result) const
{
  // No `default:`, so a new BusStatus warns until it has a message. ALREADY_OPEN and the
  // two values on_init already checked are internal errors.
  switch (result.status) {
    case BusStatus::OK:
      return;
    case BusStatus::ALREADY_OPEN:
      RCLCPP_FATAL(get_logger(),
        "internal error: the bus was already open when on_configure tried to take '%s'",
        port_.c_str());
      return;
    case BusStatus::UNSUPPORTED_BAUDRATE:
      RCLCPP_FATAL(get_logger(),
        "internal error: the bus refused baud rate %d, which on_init should have rejected",
        baudrate_);
      return;
    case BusStatus::INVALID_TIMEOUT:
      RCLCPP_FATAL(get_logger(),
        "internal error: the bus refused io timeout %u ms, which on_init should have rejected",
        io_timeout_ms_);
      return;
    case BusStatus::OPEN_FAILED:
    case BusStatus::LOCK_OPEN_FAILED:
      switch (result.error) {
        case EBUSY:
          RCLCPP_FATAL(get_logger(),
            "port '%s' is already held exclusively by another process%s; refusing to share the "
            "bus. Two programs on one servo bus produce garbage reads and a joint that drifts.",
            port_.c_str(), holder_suffix(port_).c_str());
          return;
        case EACCES:
          RCLCPP_FATAL(get_logger(),
            "not allowed to open port '%s': %s. The port usually belongs to group 'dialout'; "
            "'sudo usermod -a -G dialout $USER' and a new login session fixes that.",
            port_.c_str(), std::strerror(result.error));
          return;
        case ENOENT:
        case ENXIO:
          RCLCPP_FATAL(get_logger(),
            "port '%s' does not exist: %s. 'ls /dev/ttyACM* /dev/ttyUSB*' lists what is plugged "
            "in; the <param name=\"port\"> of the <hardware> block chooses between them.",
            port_.c_str(), std::strerror(result.error));
          return;
        case 0:
          RCLCPP_FATAL(get_logger(),
            "could not open port '%s' at %d baud; the serial layer printed the reason on stderr",
            port_.c_str(), baudrate_);
          return;
        default:
          RCLCPP_FATAL(get_logger(), "could not open port '%s' at %d baud: %s",
            port_.c_str(), baudrate_, std::strerror(result.error));
          return;
      }
    case BusStatus::NOT_A_TTY:
      RCLCPP_FATAL(get_logger(),
        "port '%s' is not a serial device (%s); expected a tty such as '/dev/ttyACM0'",
        port_.c_str(), std::strerror(result.error));
      return;
    case BusStatus::LOCK_FAILED:
      RCLCPP_FATAL(get_logger(),
        "another process holds the lock on port '%s'%s; refusing to share the bus. Two programs "
        "on one servo bus produce garbage reads and a joint that drifts.",
        port_.c_str(), holder_suffix(port_).c_str());
      return;
    case BusStatus::TERMIOS_FAILED:
      RCLCPP_FATAL(get_logger(),
        "port '%s' could not be configured for %d baud 8N1; the serial layer printed the reason "
        "on stderr. A USB adapter that has just been unplugged reports this.",
        port_.c_str(), baudrate_);
      return;
    case BusStatus::EXCLUSIVE_FAILED:
      RCLCPP_FATAL(get_logger(),
        "port '%s' refused exclusive access (%s); refusing to run without it, because nothing "
        "would then stop a second program from writing to the same servo bus.",
        port_.c_str(), std::strerror(result.error));
      return;
  }
}

hardware_interface::CallbackReturn WaveshareServos::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reached from UNCONFIGURED or INACTIVE (the controller manager deactivates an active component
  // first).
  park_and_close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WaveshareServos::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  // From INACTIVE or ACTIVE on an error: park, close the port, and return SUCCESS so the
  // component goes to UNCONFIGURED (any other return finalizes it).
  RCLCPP_ERROR(get_logger(), "error in state '%s'; stopping the servos and closing the port",
    previous_state.label().c_str());
  // Log before the port closes: an error ends the most useful measurement. One line only,
  // even if on_shutdown follows.
  log_bus_totals();
  park_and_close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

void WaveshareServos::park_and_close()
{
  // Park only if the port is open. A failed park must not escape: the resource manager
  // calls on_error from read()/write() outside its exception handling.
  if (bus_.is_open()) {
    try {
      stop_and_park(true);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "could not stop and park the servos: %s", e.what());
    }
  }
  close_port();
}

FeedbackScales WaveshareServos::scales_of(size_t i) const
{
  FeedbackScales k;
  k.encoder_steps = encoder_steps_;
  k.offset = joints_[i].offset;
  k.sign = joints_[i].sign;
  k.current_per_count_a = current_per_count_a_;
  k.torque_constant_nm_per_a = torque_constant_nm_per_a_;
  k.torque_constant_kgfcm_per_a = torque_constant_kgfcm_per_a_;
  return k;
}

bool WaveshareServos::feedback(size_t i)
{
  // One Read of registers 56..70 (15 bytes, the request FeedBack() sends), decoded by the
  // shared decoder, not the ReadX(-1) accessors. See docs/design.md, "Feedback block".
  FeedbackBlock b;
  if (!bus_.read_feedback_one(joints_[i].id, b)) {
    return false;
  }
  apply_feedback(i, b);
  return true;
}

bool WaveshareServos::feedback_from_cycle(size_t i)
{
  if (!sync_read_active_) {
    return feedback(i);
  }
  const size_t k = r_slot_[i];
  // kNoSlot should not occur for a present joint; treat it as no answer, never as slot 0.
  if (k == kNoSlot || !r_blocks_[k].valid) {
    return false;
  }
  apply_feedback(i, r_blocks_[k]);
  return true;
}

void WaveshareServos::apply_feedback(size_t i, const FeedbackBlock & b)
{
  // Shared by both transports. moving_raw is decoded by the wrapper but not published.
  FeedbackSample s;
  s.status = b.status;
  s.position_ticks = b.position_ticks;
  s.speed_ticks = b.speed_ticks;
  s.load_raw = b.load_raw;
  s.current_counts = b.current_counts;
  s.voltage_raw = b.voltage_raw;
  s.temperature_raw = b.temperature_raw;
  status_bytes_[i] = s.status;
  // The only unwrap_ticks() call: only an arrived sample advances the count. Identity if
  // the joint does not unwrap.
  const int64_t uticks = unwrap_ticks(i, static_cast<int32_t>(s.position_ticks));
  const JointStates v = decode(s, uticks, scales_of(i));
  // Plain copy: every unit conversion and the `inverted` sign live in decode().
  pos_states_[i] = v.position;
  vel_states_[i] = v.velocity;
  eff_states_[i] = v.effort;
  cur_states_[i] = v.current;
  volt_states_[i] = v.voltage;
  temp_states_[i] = v.temperature;
  load_states_[i] = v.load;
  status_states_[i] = v.status;
  torq_states_[i] = v.torque;
  measured_[i] = true;
}

}  // namespace waveshare_servos

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  waveshare_servos::WaveshareServos, hardware_interface::SystemInterface)
