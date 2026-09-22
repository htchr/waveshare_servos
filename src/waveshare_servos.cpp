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

// Every <param> name the <hardware> block may carry. Anything else is warned about and ignored:
// rejecting would break every description that carries documentation params, and staying silent
// would make a typo invisible.
constexpr std::array<const char *, 11> kKnownHardwareParams = {
  "port", "baudrate", "io_timeout_ms", "ping_attempts", "max_read_fails",
  "allow_missing_servos", "protocol", "feedback_mode", "encoder_steps", "current_per_count_a",
  "torque_constant_nm_per_a"};

// Every <param> name a <joint> may carry. Like the hardware table above, an unknown name is
// warned about and ignored: 'invert' instead of 'inverted' is a servo that turns the wrong way,
// and silence is the one response that makes that invisible.
constexpr std::array<const char *, 7> kKnownJointParams = {
  "id", "type", "offset", "inverted", "max_speed", "max_accel", "unwrap"};

// std::lround of a double outside long's range is unspecified -- on this target it is LONG_MIN --
// so an absurd but finite max_speed would be reported as smaller than one encoder step, and an
// enormous positive offset as an enormous negative tick. Saturating first keeps the magnitude
// huge and the verdict right. 2^62 is exactly representable as a double, so the saturated value
// prints exactly; a NaN takes the first branch, which no caller can reach (every input here is
// checked for finiteness first).
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

// The nine state interfaces this driver serves, indexed by StateKind. A joint may declare any
// subset in any order (PHASE2_SPEC 7.2), so this is the whole vocabulary: a name that is not here
// is a FATAL at on_init. `moving` (register 66) is deliberately absent -- adding an interface later
// is purely additive, taking one away is not.
//
// The table lives here rather than in units.hpp because it needs the hardware_interface::HW_IF_*
// constants, which units.hpp deliberately does not pull in.
constexpr std::array<const char *, kStateKindCount> kStateKindNames = {
  hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
  hardware_interface::HW_IF_EFFORT, hardware_interface::HW_IF_CURRENT,
  HW_IF_VOLTAGE, hardware_interface::HW_IF_TEMPERATURE, HW_IF_LOAD, HW_IF_STATUS,
  hardware_interface::HW_IF_TORQUE};

// Exact match, no aliasing: the framework already strips leading and trailing whitespace from the
// name attribute (component_parser.cpp:388-389).
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

  // baudrate is a set, not a range: get_int only decides well-formedness (so kEmpty and kMalformed
  // use the shared templates), and membership is tested separately with one hand-built sentence,
  // which is why a negative, a zero and an unmapped rate all read the same.
  int64_t baudrate = baudrate_;
  st = params::get_int(p, "baudrate", INT64_MIN, INT64_MAX, baudrate);
  if (rejected(st)) {
    return fail(params::message(
      st, subject_of("baudrate"), params::raw(p, "baudrate"), "an integer"));
  }
  if (st == params::Status::kOk) {
    // ServoBus::is_supported_baudrate is the single source of truth, shared with ServoBus::open().
    // The int range check in front of it is not redundant: get_int accepts the whole int64 range
    // here (membership is what decides), and narrowing 2^32 + 9600 to an int would wrap onto a
    // rate that is mapped.
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

  // Lower-cased like protocol above, so 'Per_Servo' and 'per_servo' are one value and the
  // configuration line has one spelling to print (PHASE3 2.71).
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

  // 1 ms is refused rather than warned about (PHASE3 R8): at 1 ms a sync read of four servos fails
  // 98.55 % [P3 Q2] and, worse, 8 of 3000 "clean" reads completed faster than the 0.96 ms physical
  // airtime -- the PREVIOUS cycle's frames, with correct headers, ids in their request-order
  // slots, lengths and checksums [P3 Q4/Q6]. No flush can discard bytes that have not arrived, so
  // that is a silently-wrong-data setting, not a slow one. 1000 ms is a control-loop sanity bound,
  // not a kernel one. Whether the user chose the value decides what happens below the
  // servo-count-aware floor further down, so the status is kept.
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

  // The goal tick is at most encoder_steps - 1 = 32767, exactly the 15-bit magnitude
  // SyncWritePosEx encodes. Even is required so encoder_steps / 2 is exact when a wrapped
  // difference is compared against half a revolution.
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

  // PHASE3 2.82 (R10): io_timeout_ms carries the whole sync-read BURST, not one transaction, and
  // the burst's cost grows with the number of servos in it -- 0.476 ms fixed plus 0.290 ms each
  // [P1 Q3] -- so the bound is a function of the joint count rather than a constant, which is why
  // it cannot live in the range check above. info_.joints is already populated here
  // (SystemInterface::on_init runs before this is called), and the argument is the CHUNK size and
  // not the joint count, because a longer id list is split into chunks of sync_read_max_ids
  // (2.80). What happens below the floor depends on who chose the value and on which transport
  // was asked for: the four rungs are per_servo (no question to answer), defaulted, user-set with
  // 'sync_read' and user-set with 'auto'.
  const size_t chunk_servos = std::min(info_.joints.size(), ServoBus::sync_read_max_ids);
  const uint32_t floor_ms = ServoBus::min_io_timeout_ms(chunk_servos);
  if (feedback_mode_ != "per_servo" && io_timeout_ms_ < floor_ms) {
    if (!io_timeout_user_set) {
      // Nobody chose this number, so raising it overrides nobody -- and it is what keeps a stock
      // ten-joint description on the fast path instead of silently demoting it to four times the
      // bus time. The 5 ms default is deliberately silent up to nine servos (5.19).
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
      // The value is the user's, and it is also what one dead servo will cost per cycle [P3 Q3],
      // so it is theirs to keep: the transport moves instead. A single FeedBack waits for one
      // 21-byte reply and measured 0 failures in 2000 at every setting down to 1 ms [P3 Q2], so it
      // is the path that still works down here. Frozen string L3 (PHASE3 0.4).
      RCLCPP_WARN(get_logger(),
        "io_timeout_ms %u is below the %u ms a sync read of %zu servos needs here; using one "
        "feedback read per servo", io_timeout_ms_, floor_ms, chunk_servos);
      feedback_mode_ = "per_servo";
    }
  }

  // The other end of the same budget (PHASE3 2.85, R11). The threshold is max(8, floor) and not a
  // flat 8 so that a bus big enough to legitimately need more -- min_io_timeout_ms(30) is 12 -- is
  // never scolded for obeying the floor above. 8 is a 10 ms period minus the 2 ms drain a failed
  // read pays on the error path. This says a setting is expensive, never that it is wrong: a
  // slower loop or a deliberately patient bus is a real configuration.
  const uint32_t stall_warn_ms = std::max(8u, floor_ms);
  if (feedback_mode_ != "per_servo" && io_timeout_ms_ > stall_warn_ms) {
    RCLCPP_WARN(get_logger(),
      "io_timeout_ms is %u ms and a failed read pays the %u ms drain on top of it; that is what "
      "one non-answering servo costs in every cycle that polls it (measured at 1.00-1.03x the "
      "setting across 2..50 ms [P1 Q7]), and a 100 Hz loop has 10 ms in all. A slower loop or a "
      "deliberately patient bus makes that legitimate; it is a problem only if a servo goes "
      "silent and 100 Hz still has to be met",
      io_timeout_ms_, static_cast<uint32_t>(ServoBus::sync_read_drain_ms));
  }

  // The parsed configuration, once per component load. %.7g and not %g: %g prints 0.8825985 as
  // 0.882599. A repeated <param> silently keeps the last value, so this line is the user's only
  // check of what the driver actually read -- and it reports the EFFECTIVE values, so a timeout
  // raised or a mode latched by the ladder above appears here rather than the word that was
  // parsed. It sits after the ladder for that reason.
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
    // 0. the unknown-parameter scan, first for the same reason the hardware one is: a typo stays
    // visible even when a later value is fatal. joint.parameters is unordered, so only sorting
    // makes the log reproducible.
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
    // 1. id. Required, and unchecked in Phase 1: the bare find("id")->second below was undefined
    // behaviour on a joint that declares none, and a measured SIGSEGV.
    int64_t id = 0;
    params::Status st = params::get_int(jp, "id", 1, 253, id);
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
      // kOutOfRange. The declared text is quoted rather than the parsed number so a value past
      // int64 reads the same as 0 or 254; 254 is the broadcast address the sync writes use
      // (src/SCS.cpp:132) and 255 is the packet header byte (src/SCS.cpp:130-131).
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
    // 2. the state interfaces: free-form. Any subset of the nine, in any order, and the empty set
    // is legal -- a joint the driver only commands publishes nothing. What is checked is that
    // every name is one the driver serves, that its data_type is 'double', and that it is not
    // declared twice. data_type is an XML attribute, not a <param>
    // (component_parser.cpp:237-250,433), and it defaults to "double"; checking it here turns a
    // std::runtime_error out of set_state<double>, thrown from read() on a live bus, into a
    // load-time FATAL.
    // `seen_kinds`, not the spec's `seen`: the id block above already binds `seen` to the
    // duplicate-id lookup, and one loop body cannot have two.
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
      // D2: the alias keeps kg cm, frozen at the Phase 1 number, so it is never silently re-united
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
    // 4. how the joint is driven: a position joint runs the servo's own profile to a goal position
    // (mode 0), a velocity joint is a closed-loop wheel (mode 1). The command groups are built
    // from this. A declared type wins; otherwise it is inferred, and either way it then has to
    // agree with the command interfaces.
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
    // 5. save the position command limits: write() keeps every goal inside them, since nothing
    // upstream does by default (the controller manager only clamps with enforce_command_limits).
    // hardware_interface::stod is locale-independent and rejects a non-finite result, so a limit
    // of "nan" is reported here instead of silently switching that side of the clamp off.
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
    // 6. offset, a SERVO-frame constant: which servo tick is joint zero. It applies to a wheel
    // too -- the read path is uniform, so a wheel also reports zero at its calibrated mark -- and
    // only the single-turn check at the end of this loop is position-only.
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
    // 7. inverted, stored as a double sign so the hot path multiplies instead of branching and an
    // uninverted joint reproduces the Phase 1 arithmetic bit for bit. '1' and '0' are deliberately
    // not accepted. It flips exactly position, velocity and load, and the two commands (D4).
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
    // 8. max_speed (rad/s) and max_accel (rad/s^2), both SI in the joint frame and both
    // magnitudes, so `inverted` never touches them. An ABSENT one keeps the Phase 1 register
    // value and is never routed through a conversion: the acceleration scale is the one number
    // here that is neither in the vendored sources nor measured, and a default through it would
    // silently change the acceleration of every robot that already works.
    //
    // Coverage owed, and now discharged: the converted counts have no load-time observable -- a
    // position joint's reach the wire as the speed and acceleration fields of the GoalPosition
    // records send_commands() builds, a wheel's acceleration through write_wheel_acceleration() --
    // so the rejections and the two cap WARNs below pin the conversion and the clamping, and
    // test_lifecycle_over_pty's the_acceleration_record_carries_the_per_joint_max_accel and
    // configure_writes_each_wheel_acc_once pin that the CONVERTED values, not the 6000/150
    // defaults, are what reach the bus.
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
        // bit 15 of the goal speed field is the direction sign (src/SMS_STS.cpp:89-92), and on the
        // position path the value goes into the register un-encoded, where it would read as a
        // negative goal speed
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
          // a small but positive limit that rounds to 0 counts would mean "no limit" by accident,
          // which is the inversion of meaning this phase removes
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
    // 9. unwrap, defaulting to the resolved type: a wheel turns past a revolution, a position
    // joint cannot. Asking for it on a position joint is a FATAL rather than a silently ignored
    // parameter, because the goal-speed pacing, the limit check and the activation seed all read
    // the position back as an absolute tick count.
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
    // 10. the single-turn range check, position joints only and last, because it needs the
    // offset, the sign and both limits. It uses the expression the write path uses, so a goal the
    // servo would silently ignore is refused here instead of being sent every cycle.
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
        // One warning per joint, not one per side. Requiring finite limits outright would reject
        // every stock ros2_control description, and skipping silently is the failure this check
        // exists to remove.
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
  // the deprecated `torque` interface keeps kg cm (D2), so the configured N m/A is converted once
  // here rather than per sample
  torque_constant_kgfcm_per_a_ = kgfcm_per_amp(torque_constant_nm_per_a_);
  // init vectors for state interfaces. All nine start at NaN, `status` included: NaN is what the
  // interface reads until a servo actually answers, and 0.0 means "it answered and reported no
  // fault" (PHASE2_SPEC 3.5).
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
  // One unwrapper per joint, whether it unwraps or not, so every per-joint vector is indexed the
  // same way and unwrap_ticks() needs no second mapping. They are sized from encoder_steps_, which
  // read_hardware_parameters() resolved above.
  unwrap_.assign(joints_.size(), PositionUnwrapper(encoder_steps_));
  unwrap_gap_warns_.assign(joints_.size(), 0);
  // The failed-transaction counters of PHASE3 3.34, sized with the other per-joint vectors and
  // zeroed again by every on_activate, which is what makes one activation one measurement window.
  read_attempts_.assign(joints_.size(), 0);
  read_failures_.assign(joints_.size(), 0);
  // The read group's three parallel vectors and its joint->slot map, reserved to the full joint
  // count once. build_read_group() runs from read() on the drop edge, so it must not allocate
  // there; reserving for every joint covers the largest group it can ever build (PHASE3 2.59).
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
// One handle per name, in the order of the names: the handles the description asks for, in the
// order it lists them. A handle that no name asks for is left out, and a name listed twice takes
// the same handle twice, which is exactly what the driver exported before the handles belonged to
// the framework.
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
  // The framework creates a handle for every interface of the description, including those of a
  // <gpio> or <sensor> this driver knows nothing about, and hands them over in the order of its
  // hash map. The resource manager lists them in the order they are exported, and so do
  // list_hardware_components and the controllers that claim every interface
  // (/dynamic_joint_states). Export what the driver has always exported: the state interfaces of
  // the joints, joint by joint, in the order the description lists them. An interface the driver
  // does not serve is left out rather than handed to a controller as a value nothing ever writes;
  // the resource manager then refuses the description, as it did before.
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
  // Take the bus exclusively. open() also puts the io timeout in force before the first
  // transaction: a servo replies in well under a millisecond at this baud rate, so the library's
  // stock 100 ms timeout only ever costs time -- with it, every absent servo burned a tenth of a
  // second of every control cycle.
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
      "take it. 'ros2 run waveshare_servos set_id', screen and minicom do not take it.",
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
  // The gate sits here, between the ping loop and build_groups(), on purpose: build_groups() calls
  // set_mode(), which can unlock and rewrite EPROM register 33, and a configuration that is about
  // to be refused must not spend one of that cell's write cycles.
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
      // Every failure path in on_configure logs one FATAL, closes the port and returns FAILURE: a
      // component refused out of UNCONFIGURED gets neither on_error nor on_cleanup, so this is the
      // only place the port can be released.
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
  // Build the command groups from the servos that answered. An absent servo must never end up
  // in a sync write or a poll: it cannot answer, so it costs a full timeout every cycle.
  // One pass over the joints in URDF order fills both groups, each in the order the description
  // lists its joints, which is the order the two separate passes produced before.
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
    // Site 1 of PHASE3 1.24, and it must stay in this order: set_mode() writes register 33, and
    // writing the mode register clears the torque-enable register 40 -- measured on the bench when
    // the probe tripped over it and the servos free-wheeled. set_mode() does not restore torque;
    // the driver is safe only because on_activate reaches EnableTorque after build_groups(), so
    // nothing here may be reordered. The acceleration write itself touches neither 33 nor 40.
    //
    // Being inside build_groups() is what makes this site cover the dropped-then-recovered servo
    // too: recovery is ping -> regrouped -> build_groups() (on_activate, below).
    write_wheel_acceleration(v_js_[k]);
  }
  // Size the wrapper's packet scratch once per rebuild, so the first write() of an activation
  // allocates nothing. Over-sizing is free; the wrapper clamps at its per-packet maxima, so a
  // hundred-servo group reserves one chunk and not a hundred records (PHASE3 1.7).
  bus_.reserve_goal_capacity(p_ids_.size(), v_ids_.size());
  // Last, so the three groups are always built together and present_ is read once per rebuild by
  // all three. The read group has its own builder rather than sharing this one because read()
  // calls it on the drop edge and this function does bus traffic -- a set_mode() per id, and an
  // EPROM unlock/write/lock whenever a mode differs (PHASE3 2.64).
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
  // Step 0 of PHASE3 2.74, and unconditional: on_activate calls build_groups() only when a servo
  // was recovered, so the read group cannot be assumed fresh at this point.
  build_read_group();
  sync_read_active_ = false;
  // 'per_servo' answers the question without asking the bus, which is what makes it the A/B
  // baseline of the bench comparison: not one INST_SYNC_READ is emitted, probe included (5.4).
  // An empty group is the same answer for a different reason -- with nothing present, every joint
  // takes read()'s absent-servo mirror and no transport is used at all (PHASE3 2.78).
  if (feedback_mode_ == "per_servo" || r_ids_.empty()) {
    return true;
  }
  // Judged against the servos that answered their SEEDING feedback() in the loop above, not
  // against every id in the group: a servo that pings but does not read has already taken that
  // loop's else branch, and condemning the whole transport for it would be wrong. measured_ is
  // exactly that set (PHASE3 2.74 step 3). Probing with one servo, as jazzy.md item 4 literally
  // suggests, is rejected: a firmware that answers a 1-id burst and mishandles a 4-id one is
  // invisible to it, and the full probe costs one extra transaction once per activation.
  std::vector<std::string> unanswered;
  // Twice at most: one lost frame must not condemn the mode, and a second loss on the same
  // activation is evidence rather than noise (PHASE3 2.74 step 5).
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
    // Frozen string L1 (PHASE3 0.4). Once per activation and never per cycle: the mode is decided
    // here and nowhere else, so there is nothing later to report (2.75).
    RCLCPP_INFO(get_logger(),
      "feedback for %zu servos travels in one sync read per cycle (INST_SYNC_READ)",
      r_ids_.size());
    return true;
  }
  const std::string ids = join(unanswered, ", ");
  if (feedback_mode_ == "sync_read") {
    // The value exists so that this case is loud. Demoting here would run the transport the user
    // explicitly ruled out and would be measured as though it were the fast one (PHASE3 2.72).
    RCLCPP_FATAL(get_logger(),
      "sync read went unanswered by motor id(s) %s, which answered a feedback read moments "
      "earlier, so the firmware ignores INST_SYNC_READ; feedback_mode is 'sync_read', which rules "
      "out the per-servo path, so refusing to activate. Use 'auto' to fall back instead",
      ids.c_str());
    return false;
  }
  // Frozen string L2 (PHASE3 0.4). "Answered FeedBack but not INST_SYNC_READ" is exactly the
  // firmware condition jazzy.md item 4 asks to verify, which is why the ids are named.
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
  // on_error is reachable from INACTIVE, where no cycle ever ran. An all-zero line there is noise
  // a grep-based gate would have to learn to ignore, so say nothing (PHASE3 3.36).
  if (read_cycles_ == 0) {
    return;
  }
  // One transaction is one of the wrapper's own read entry points (PHASE3 5.14): the sync path
  // issues one sync_read_feedback() per control cycle however many servos are on the bus, the
  // per-servo path one read_feedback_one() per polled joint. Not read_cycles_, which would make
  // the per-servo arm's denominator a quarter of its real traffic on this bench and its
  // per-million rate four times the sync arm's for an identically healthy bus -- and those two
  // arms are exactly what 5.6 runs and 5.26 prints side by side. hil_gates.py's H11 arithmetic
  // (`expected = 100.0 * soak`, SOAK_MIN_TRANSACTIONS = 30000 as five minutes at 100 Hz) is
  // written against the sync path, where the two quantities are the same number.
  const double per_million = read_transactions_ ?
    1e6 * static_cast<double>(read_transaction_failures_) /
    static_cast<double>(read_transactions_) :
    0.0;
  // The bracketed tail is required, one pair per DECLARED servo in id order: jazzy.md:204 asks for
  // the count per servo, and hil_gates.py parses it so a failing soak can say which ids the
  // failures fell on. Id order, not URDF order, because that is the order a human reads a bus in.
  // It is per JOINT and the aggregate above is per CYCLE, so the tail sums to `failed` only while
  // no burst ever lost two frames at once; that is the point of printing both.
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
  // Frozen string L5 (PHASE3 0.4, 5.14). The grammar is what test/hil/hil_gates.py's BUS_TOTALS
  // regex matches; it may not drift.
  RCLCPP_INFO(get_logger(),
    "bus totals: transactions %" PRIu64 ", failed %" PRIu64 " (%.1f per million), worst "
    "consecutive %" PRIu32 ", dropped %" PRIu32 " [%s]",
    read_transactions_, read_transaction_failures_, per_million, worst_consecutive_,
    dropped_count_,
    tail.c_str());
}

void WaveshareServos::write_wheel_acceleration(size_t i)
{
  // A servo that is not on the bus would cost one whole io_timeout_ms for a write nobody can ack,
  // and a position joint carries its acceleration in every goal record already (PHASE3 1.21).
  if (!present_[i] || joints_[i].type != JointType::velocity) {
    return;
  }
  // acc_counts of 0 is written, never skipped: max_accel="0" is the documented "no ramp" opt-out
  // (cpp:717-719), and skipping would leave whatever the servo held from its EPROM default or a
  // previous run -- silently inverting the meaning of the parameter (PHASE3 1.23).
  if (!bus_.write_acc(joints_[i].id, joints_[i].acc_counts)) {
    // Not fatal, and no retry here: a failed write costs a full io_timeout_ms each time, and two
    // of the four sites run inside read(). The next of the four sites is the retry (PHASE3 1.25).
    // It is a WARN rather than an INFO because an unramped wheel is a physical surprise with no
    // other symptom. The text is frozen (PHASE3 L4); 4.T55 matches it literally.
    RCLCPP_WARN(get_logger(),
      "could not write the acceleration register of motor id '%d'; it will run unramped",
      joints_[i].id);
  }
}

bool WaveshareServos::set_mode(u8 id, u8 mode)
{
  // Register 33 lives in EPROM. It is write protected until the lock register is cleared --
  // without that the write is silently dropped -- and the cell has a limited write endurance,
  // so only touch it when the mode is actually wrong.
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
  // The framework creates one handle per <state_interface>/<command_interface> of the URDF right
  // after on_init, so on_configure is the first place they can be looked up. Look them up by name:
  // joint_states_/joint_commands_ are not in URDF order. on_init has already checked every state
  // interface name, so these lookups cannot throw and state_kind() cannot be empty here.
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
  // A non-blocking get that finds the handle busy returns false and leaves the cache untouched, so
  // the joint keeps the command it had last cycle: never NaN, never a jump. A command interface the
  // URDF does not declare has no handle, and its cache entry stays internal to the driver.
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
  // A non-blocking set that finds the handle busy leaves the previous sample in it; the caches are
  // published again every cycle, so it catches up on the next read(). Only the interfaces the
  // description declares have a handle here, so nothing null is ever touched.
  const JointHandles & h = handles_[i];
  for (const auto & entry : h.states) {
    std::ignore = set_state(entry.second, state_value(i, entry.first), wait);
  }
}

hardware_interface::CallbackReturn WaveshareServos::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // One activation is one measurement window, so the failed-transaction counters start again
  // here -- at the top, before the re-ping loop, so nothing this activation does falls outside
  // the window it is charged to (PHASE3 3.35, 5.14). H9 cycles the component inactive->active and
  // must not carry its counts across.
  read_cycles_ = 0;
  read_transactions_ = 0;
  read_transaction_failures_ = 0;
  read_stats_reported_ = false;
  worst_consecutive_ = 0;
  dropped_count_ = 0;
  read_attempts_.assign(joints_.size(), 0);
  read_failures_.assign(joints_.size(), 0);
  // read_fails_ with them, because worst_consecutive_ is defined as the largest value it reaches
  // since activation (R16) and a run that began in the previous window would otherwise be reported
  // against this one. It is also the drop budget, and the drop's own ERROR text promises that
  // re-activating the hardware looks for the servo again -- a budget carried across would spend
  // what is left of it on the first failed read instead
  // (re_activating_hands_a_still_silent_servo_its_whole_drop_budget_again). The two clears further
  // down (a servo that pinged back, a servo whose seeding read answered) are left where they are:
  // they are what PHASE2_SPEC 8.4/8.5 pin, and they are now reinforcement rather than the only
  // reset. The third consequence is deliberate too: note_unwrap_gap()'s `missed` starts this
  // window at 0 even for a servo that was silent right across the transition, which is correct
  // because reset_unwrap() in the seeding loop below clears, in the same activation, the very
  // accumulator that gap would have been charged against (PHASE2_SPEC 8.4).
  read_fails_.assign(joints_.size(), 0);
  // Activation is off the real-time path, so it is the right place to look again for a servo
  // that was absent at configure time or was dropped after it stopped answering. Deactivating
  // and re-activating the hardware is therefore enough to recover one, with no restart.
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
  // set position commands to current positions before any movement to not move on start. Each
  // command reaches its handle at the point the value is set, as it did when the controllers shared
  // this memory: a controller that is still active can overwrite it before the next write().
  for (size_t i = 0; i < joints_.size(); i++) {
    vel_cmds_[i] = 0.0;
    push_velocity_command(i);
    hold_pos_[i] = std::numeric_limits<double>::quiet_NaN();
    // A joint whose count is still seeded KEEPS it across an inactive->active cycle; only a count
    // that has already been thrown away is started again here, and the seeding feedback() below
    // then bridges the gap instead of re-zeroing (PHASE2_SPEC 8.4, narrowed in Phase 4).
    //
    // 8.4 originally reset unconditionally, on the grounds that "a consumer that integrates wheel
    // position has to re-zero on hardware activation, exactly as it does on controller activation".
    // That premise does not hold: a controller stays ACTIVE across a hardware-component cycle and
    // is never told it happened, so it has no re-zero to perform. Measured on the bench (hil_check
    // H9.g1c) and reproduced on the shipped example, the reset dropped a wheel's position by
    // exactly the whole turns it had accumulated -- 6.000 turns in H9 -- which
    // diff_drive_controller with position_feedback: true differences into a 0.314 m odometry
    // teleport per turn at the example's wheel_radius, with no reset service in 4.42.1 to undo it.
    //
    // There is nothing left to protect here: `unwrap` is rejected for a `pos` joint (read_joints),
    // so only wheels carry a count, and a wheel has no position command to seed, no position limits
    // to be outside of (outside_limits is false for every non-position joint) and no activation
    // hold -- the three things the old comment said the plain register reading was needed for. For
    // a joint that does not unwrap, unwrap_ticks() returns `raw` without consulting the accumulator
    // at all, so this is bit-for-bit unchanged for `pos` joints either way.
    //
    // The two resets that DO fire are the ones where continuity was genuinely lost: close_port()
    // clears every count (a new session on a newly opened port), and read()'s dropping branch
    // clears the count of a servo it has stopped believing -- so a rejoining servo still starts
    // fresh, which is what a_rejoining_wheel_starts_a_new_unwrapped_count pins. Carried from a live
    // count, the bridge is bounded to whole revolutions and has the same envelope as any sample
    // gap; a wheel hand-turned more than half a revolution while INACTIVE aliases exactly as one
    // turned that far between two reads does. Re-seeding instead makes the error the full travel,
    // which position_unwrapper.hpp's own "Rejected alternatives" already calls strictly worse.
    if (!unwrap_[i].seeded()) {
      reset_unwrap(i);
    }
    if (present_[i]) {
      // A servo whose torque has been latched off -- by a protection trip, or by whatever
      // last talked to it -- accepts goal positions and quietly ignores them.
      bus_.EnableTorque(joints_[i].id, 1);
      // Site 2 of PHASE3 1.24, after EnableTorque because register 41 is unaffected by the torque
      // register either way and keeping EnableTorque on its own line keeps the H-gate log ordering
      // readable. It covers what site 1 cannot: an activation with no recovered servo never calls
      // build_groups(), and a wheel that power-cycled while the component was INACTIVE answers
      // every ping and every read with its SRAM acceleration register lost [P2 Q2].
      //
      // An activation that DID recover a servo writes register 41 twice, once here and once in
      // build_groups() above. That is accepted: the register is SRAM, so the second write costs
      // 0.594 ms and nothing else (PHASE3 3.20), and no test may assert "exactly one" per
      // activation -- only a delta across one transition.
      write_wheel_acceleration(i);
    }
    if (present_[i] && feedback(i)) {
      pos_cmds_[i] = pos_states_[i];
      // A successful read is a successful read, off the real-time path as much as on it: clear the
      // failure count this servo may have carried in from before the deactivation. Otherwise a
      // silence that ENDED at this activation is charged to the first read() after it -- a gap a
      // whole activation old, reported against a count that was re-seeded from the very sample
      // above (PHASE2_SPEC 8.4, 8.5) -- and the drop budget is spent by a servo that is answering.
      read_fails_[i] = 0;
      // A joint that starts outside its limits (moved by hand with the torque off, say) is not
      // run to the nearest one: write() holds it where it is until it is commanded to a
      // position inside them.
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
  // Last, after the seeding loop, because this is the only placement that judges the transport
  // against the servos that actually answered a feedback read: present_ and measured_ are both
  // final here, and a servo that pings but does not read has already taken the else branch above
  // (PHASE3 2.74, R6). It must never sit inside the loop -- EnableTorque and the seeding read are
  // two lines of the same per-joint body, so a probe between them would run once per joint.
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
  // After the park and not before it, so the parking round trips fall inside the window they were
  // issued in rather than silently outside it (PHASE3 5.14).
  log_bus_totals();
  // The transport decision belongs to one activation. Clearing it here means the next on_activate
  // probes again rather than inheriting a verdict taken against a bus that may have changed
  // (PHASE3 2.73).
  sync_read_active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool WaveshareServos::outside_limits(size_t i, double position) const
{
  // Only a position-controlled joint has limits to be outside of. A continuous joint's position
  // grows without bound once it is unwrapped and must never read as "outside its limits": the
  // example xacro declares the continuous joint3 with <limit lower="-3.14" upper="3.14">, so the
  // day anything derives driver limits from that tag, an unwrapped wheel would otherwise latch a
  // hold on its first revolution (PHASE2_SPEC 8.7).
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
  // At least one control period passed, and a failed round trip costs at least the read timeout.
  // The timeout is charged only to the cycles that actually FAILED: with missed == 0 every read
  // answered, so the elapsed time is the control period and nothing else. (PHASE2_SPEC 8.5's code
  // block reads `cycle * (missed + 1)` unconditionally, which at a legal io_timeout_ms of 342 or
  // more reports a gap of a whole timeout on every healthy cycle of a 100 Hz loop; the deviation
  // is declared, and it leaves every gap with missed >= 1 exactly as the spec computes it.)
  const double cycle = std::max(last_period_, static_cast<double>(io_timeout_ms_) / 1000.0);
  const double gap = (missed == 0) ? period_s : std::max(period_s, cycle * (missed + 1));
  // The joint's speed CEILING, not its last measured speed: what a servo did while it was not
  // answering is unknown by construction, and the margin this warning has comes from bounding it.
  // That is also why the wording is "may be": it fires for a slowly turning wheel that could not
  // have aliased, and staying silent there would mean guessing.
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
  // set velocities to 0 on close. The zeros reach the handles right here, before the feedback
  // round, as they did when the controllers shared this memory: a controller that is still active
  // (a deactivate requested through the service) can write its command back before write() below
  // takes the commands from the handles, and then the stop is lost, as it was before the handles.
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
      // The handles may hold no command of a controller, only NaN or one from before a cleanup,
      // and hold_pos_ may be stale. Park at a measurement alone, this one or else the last one
      // since on_configure (NaN if there is none: the 0.0 on_activate stands in for a servo that
      // did not answer is not a position), and hold a joint found outside its limits there, as
      // on_activate does, instead of running it to the nearest limit at full speed.
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
    // A servo that has not returned a position since on_configure has nowhere to be parked: leave
    // it out of this last write rather than send it a made-up goal. So is one that read() dropped
    // after it stopped answering: it is still in the group, but its entry holds the command left
    // in its handle, not a position. The port is closed next, which clears the groups, and
    // on_configure builds them again.
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
    // Send the park from the values just decided, not from the handles: a controller that still
    // commands (a finalize requested while it is active) must not be able to replace the stop, and
    // the port is closed right after this.
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
  // The elapsed time this cycle covers, guarded exactly as send_commands() guards it: a zero or
  // non-finite period falls back to the last non-zero one. It is the gap warning's evidence that a
  // control loop was descheduled, which a failed-read count alone cannot see (PHASE2_SPEC 8.5).
  double period_s = period.seconds();
  if (!(std::isfinite(period_s) && period_s > 0.0)) {
    period_s = last_period_;
  }
  // One transaction for every present servo, before any per-joint work (PHASE3 2.62). All four
  // samples now come out of one ~1.6 ms window instead of being spread over ~3.0 ms of sequential
  // reads [P1 Q3], and the fault-clear EnableTorque in the loop below can no longer land between
  // two servos' reads. The return value is not consulted: the per-slot `valid` flag is the only
  // gate, because a burst that lost one frame still carries the other three (PHASE3 2.33, R3).
  const bool burst_issued = sync_read_active_ && !r_ids_.empty();
  if (burst_issued) {
    bus_.sync_read_feedback(r_ids_, r_blocks_);
    // One wrapper read entry point, whatever the servo count: the sync path's whole transaction
    // (PHASE3 5.14). The per-servo path counts its own, one per polled joint, below.
    read_transactions_++;
  }
  read_cycles_++;
  // The burst is ONE transaction, so however many frames it lost it is charged one failure: the
  // flag is raised by any polled joint and charged once after the loop, which is what keeps the
  // rate at or below a million per million (PHASE3 5.14).
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
    // After the absent branch's continue, so an absent joint never enters its own poll count
    // (PHASE3 3.34, F14), never costs a transaction and never raises a failure either.
    read_attempts_[i]++;
    // On the per-servo path this joint's poll IS a transaction of its own -- one
    // read_feedback_one() unicast -- so it enters the denominator here rather than sharing the
    // burst's single entry above (PHASE3 5.14, R15).
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
        // Charged once after the loop, and only against a burst that really went out, so
        // `failed <= transactions` holds by construction rather than by the read group and
        // present_ being rebuilt together -- which they are, but that is a second invariant.
        burst_failed = true;
      }
      read_fails_[i]++;
      // read_fails_ is consecutive and any success clears it, so the peak has to be remembered as
      // it happens: a bus that loses one reply in every other cycle would otherwise report a
      // worst run of 1 forever (PHASE3 5.14).
      worst_consecutive_ = std::max(worst_consecutive_, static_cast<uint32_t>(read_fails_[i]));
      if (read_fails_[i] == 1 || read_fails_[i] % 200 == 0) {
        RCLCPP_WARN(get_logger(),
          "read failed for motor id '%d' (%d in a row)", joints_[i].id, read_fails_[i]);
      }
      if (read_fails_[i] >= max_read_fails_) {
        // It answered at configure time and has now gone quiet -- a brown-out, a
        // protection trip, a pulled connector. Stop polling it: otherwise it costs a
        // full timeout in every control period from here on and drags the whole loop
        // down, which is the failure this driver started with. Deactivate and activate
        // the hardware to look for it again. The budget is in cycles, not seconds: this is
        // max_read_fails CONSECUTIVE failed reads, so the wall clock before a drop stretches with
        // a lower rw_rate on the <ros2_control> tag -- half a second at 100 Hz, a full second at
        // rw_rate="50". The policy is unchanged by that attribute; only its clock is (README,
        // "Running the bus slower, or off the control thread").
        RCLCPP_ERROR(get_logger(),
          "motor id '%d' stopped answering after %d attempts; dropping it from the "
          "read cycle until the hardware is re-activated", joints_[i].id, read_fails_[i]);
        present_[i] = false;
        dropped_count_++;
        // The read group is rebuilt after the loop and not here: the joints after i have not been
        // visited yet and still index r_slot_, so compacting now would re-point them at another
        // servo's sample (PHASE3 2.65, 3.28). The WRITE groups are deliberately not rebuilt at
        // all -- a broadcast sync write is never acked, so a dropped servo costs it three bytes,
        // while a dead id in the read list costs a whole io_timeout_ms every cycle (F21).
        read_group_dirty_ = true;
        reset_unwrap(i);
        if (joints_[i].unwrap) {
          // Keep the absent-branch mirror continuous with the last unwrapped reading. A `vel`
          // joint declares no position command interface, so pos_cmds_[i] still holds the
          // ACTIVATION-time position and read()'s absent branch would publish it next cycle: an
          // unbounded backward jump, strictly worse than the bounded 2 pi discontinuity unwrapping
          // removes (PHASE2_SPEC 8.6). Safe: p_js_ is built from position joints only, so a `vel`
          // joint's pos_cmds_ never reaches send_commands(), and the guard leaves `pos` joints
          // (where unwrap is always false) bit-for-bit unchanged.
          pos_cmds_[i] = pos_states_[i];
        }
      }
      continue;
    }
    // The sample arrived. Report a gap that has now ENDED, before the failure count is cleared:
    // this runs on EVERY successful cycle, not only when missed > 0, because a control-loop
    // deschedule produces missed == 0 and is exactly the gap the warning exists for.
    const int missed = read_fails_[i];
    read_fails_[i] = 0;
    note_unwrap_gap(i, missed, period_s);
    if (missed > 0) {
      // Site 3 of PHASE3 1.24. This joint answered again after at least one silent cycle, and the
      // cheapest explanation that matters is a wheel that power-cycled and came back inside the
      // drop budget: it reads fine and has silently lost its SRAM acceleration register [P2 Q2].
      // A power cycle takes far longer than one control period, so it always shows up as a missed
      // read -- which makes this the only place that catches the loss DURING an activation, now
      // that nothing rewrites register 41 per cycle. One 0.594 ms unicast [P2 Q1] on a cycle that
      // was already abnormal.
      write_wheel_acceleration(i);
    }
    // Every reply carries the servo's status byte -- overload, over-temperature, over-voltage
    // and friends. A latched protection trip is exactly the sort of thing that makes a servo stop
    // responding to goals. Use the byte feedback() captured rather than re-reading SCS::Error:
    // EnableTorque below is a writeByte -> Ack, which rewrites it (src/SCS.cpp:292), so reading it
    // here would make the edges depend on what the previous joint's branch did.
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
      // Site 4 of PHASE3 1.24 (R12). A protection trip that resets the servo's controller can
      // clear SRAM with no missed read at all, so site 3 does not subsume this one; the inference
      // that it does is not a measurement (UNMEASURED, PHASE3 section 8 Q3). The branch already
      // pays the EnableTorque round trip above, so the insurance costs one more 0.594 ms write on
      // an edge that is abnormal by definition.
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
  // The servo runs its own trapezoidal profile to the goal position and stops dead on arrival,
  // so the goal speed is what decides whether a stream of setpoints comes out as continuous
  // motion or as a sequence of sprints and dwells. Pace it to arrive just as the next setpoint
  // is written.
  //
  // `period` is the controller manager's write period by default, and this component's own once an
  // rw_rate on the <ros2_control> tag decimates the loop: ros2_control 4.48 then passes the time
  // since this component's last write, so at half the rate dt doubles along with the chord each
  // setpoint has to cover and the goal speed comes out the same rad/s. Use it; never substitute
  // 1/update_rate. The 0.01 seed behind the guard is only reached by the park, which passes a zero
  // period on purpose (README, "Running the bus slower, or off the control thread").
  double dt = period.seconds();
  if (std::isfinite(dt) && dt > 0.0) {
    last_period_ = dt;
  } else {
    dt = last_period_;
  }
  // stop_and_park(true) compacts p_ids_/p_js_ in place before its single send, so the record array
  // can be longer than the group it describes. Resizing here makes the pruned path correct by
  // construction and never allocates in steady state (PHASE3 1.27).
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
    // Keep the goal inside the joint's limits. The servo's own angle limits cannot stand in for
    // them: they are raw encoder steps (0..4095 by default) and know nothing of the offset.
    // A joint that started outside them is the exception: it stays where it started until it
    // is commanded to a position inside them, instead of being run to the nearest limit.
    if (std::isfinite(hold_pos_[j]) &&
      ((cmd < joints_[j].pos_min) || (cmd > joints_[j].pos_max)))
    {
      cmd = hold_pos_[j];
    } else {
      hold_pos_[j] = std::numeric_limits<double>::quiet_NaN();
      cmd = std::clamp(cmd, joints_[j].pos_min, joints_[j].pos_max);
    }
    // the exact inverse of the read path: sign first, then the offset, then the tick scale, in
    // the baseline's operator order
    const double goal_steps =
      (joints_[j].sign * cmd + joints_[j].offset) * encoder_steps_ / (2 * M_PI);
    const s16 goal_ticks = static_cast<s16>(std::lround(std::clamp(goal_steps, -32767.0, 32767.0)));
    // Pace the chord this setpoint actually adds, plus whatever the servo still owes:
    // goal(k) - measured(k) is exactly chord + lag, so one term covers both. Deliberately
    // NOT the trajectory's instantaneous velocity -- on an accelerating segment that is
    // larger than the chord's mean slope, which makes the servo finish the chord early and
    // then stand still for the rest of the period. That is the stutter we are removing.
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
    // The goal speed register is an unsigned magnitude -- travel direction comes from the
    // goal position, not from this field -- and 0 in it means "no speed limit", i.e. full
    // speed. Never let a value land on 0 by accident: that is what made the servo lurch at
    // the start and the end of every trajectory, where the commanded velocity is zero.
    // Sending the goal position faster than the servo can reach it is harmless: the goal
    // position bounds the travel, so an over-large speed can only make it arrive early.
    const u16 goal_speed = static_cast<u16>(
      std::clamp(speed, 1.0, static_cast<double>(joints_[j].max_speed_counts)));
    // The acceleration keeps travelling in byte 0 of the record, value 0 included: on the position
    // path it costs nothing extra, so a position servo that restarts mid-run repairs its own
    // register on the next cycle and needs none of the wheel policy (PHASE3 1.21).
    p_goals_[k] = GoalPosition{p_ids_[k], goal_ticks, goal_speed, joints_[j].acc_counts};
  }
  for (size_t k = 0; k < v_ids_.size(); k++) {
    const size_t j = v_js_[k];
    const double vel = std::isfinite(vel_cmds_[j]) ? vel_cmds_[j] : 0.0;
    const double speed = std::clamp(joints_[j].sign * vel * encoder_steps_ / (2 * M_PI),
      -static_cast<double>(joints_[j].max_speed_counts),
      static_cast<double>(joints_[j].max_speed_counts));
    // No acceleration here: register 41 is SRAM and is written at the four edges of PHASE3 1.24
    // instead of once per cycle. That removal is the 0.567 ms per wheel per cycle Phase 3 item 1
    // buys back [P2 Q1] -- 1.6x the estimate jazzy.md:202 made.
    v_goals_[k] = GoalSpeed{v_ids_[k], static_cast<s16>(std::lround(speed))};
  }
  // One frame per group, chunked by the wrapper at 30 position records and 82 speed records so the
  // unchecked 255-byte txBuf cannot overflow. An empty group puts nothing on the wire (PHASE3
  // 1.16), so the two emptiness guards that used to protect the vendored helpers' zero-length VLAs
  // are gone: one guard in one place cannot fall out of step with a second call site (R13).
  const WriteResult positions = bus_.write_goal_positions(p_goals_);
  const WriteResult speeds = bus_.write_goal_speeds(v_goals_);
  if (!positions || !speeds) {
    // Both refusals mean nothing reached the bus and every servo kept the previous cycle's goals
    // -- on the velocity path, a turning wheel keeps turning -- so this is an ERROR and not a
    // WARN. write() still returns OK: an ERROR return sends the component to on_error(), which
    // parks and closes the port, and that is far too heavy for a closed port (already fatal
    // elsewhere) or a bad id that on_init should have caught (PHASE3 1.27, R14). Throttled the
    // same way read()'s failing-servo WARN is, because a refusal repeats every cycle until
    // something else changes.
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
  // The wrapper's own packet scratch keeps its capacity: it holds no state between calls, and
  // on_configure would only have to reserve it again (PHASE3 1.28).
  p_goals_.clear();
  v_goals_.clear();
  // The read group goes with them, and so does the transport flag: a cleanup must not leave the
  // driver believing a sync path is live over an empty group (PHASE3 2.70, 3.23). r_slot_ is
  // assigned rather than cleared so it stays one entry per joint, which is what read() indexes.
  r_ids_.clear();
  r_js_.clear();
  r_blocks_.clear();
  r_slot_.assign(joints_.size(), kNoSlot);
  read_group_dirty_ = false;
  sync_read_active_ = false;
}

void WaveshareServos::log_open_failure(const OpenResult & result) const
{
  // No `default:` label: a new BusStatus must be given a sentence here or the build warns.
  // UNSUPPORTED_BAUDRATE, INVALID_TIMEOUT and ALREADY_OPEN are reported as internal errors --
  // on_init validated both values already, and the user must never get two different complaints
  // about one number.
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
  // Reached from INACTIVE or ACTIVE when a transition, read() or write() reports an error. Stop and
  // park the servos if the port is open, close it, and return SUCCESS: the component then goes to
  // UNCONFIGURED and can be configured again (any other return finalizes it).
  RCLCPP_ERROR(get_logger(), "error in state '%s'; stopping the servos and closing the port",
    previous_state.label().c_str());
  // Before park_and_close(), which is where the port goes: an activation that ends in an error is
  // the one whose failure counts are most worth having, and read_stats_reported_ keeps an
  // error-then-shutdown sequence to one line (PHASE3 3.36, R16).
  log_bus_totals();
  park_and_close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

void WaveshareServos::park_and_close()
{
  // Only talk to the servos if on_configure opened the port: the library would otherwise select()
  // on an invalid descriptor. A failed park must still close the port and must not escape: the
  // resource manager calls on_error from its read() and write() outside its exception handling,
  // and the handles throw if the URDF gives an interface a data_type other than double.
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
  // One round trip fills the servo's whole feedback block (registers 56..70), so all nine state
  // values come from a single transaction instead of one blocking read each.
  //
  // read_feedback_one() issues Read(id, 56, ., 15), which is byte for byte what FeedBack(id)
  // issues (src/SMS_STS.cpp:123, sizeof(Mem) == 15) -- the wire is unchanged (PHASE3 F5). What
  // changed is that the wrapper decodes the block with the same decoder the sync path uses
  // instead of with the ReadX(-1) accessors, which retires the Err-gated sign trap from both
  // paths: those accessors suppress every sign decode when SCS::Err is set by an unrelated
  // earlier call (src/SMS_STS.cpp:146,168,188,254). With no ReadX(-1) call left anywhere in the
  // package, SMS_STS::Mem going stale cannot hurt either (PHASE3 R15, 2.61a).
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
  // kNoSlot cannot reach here for a present joint -- present_ and the read group are rebuilt
  // together -- but a missing slot has to read as "did not answer" rather than as slot 0's data,
  // which is the one way a joint could publish another servo's sample (PHASE3 3.27).
  if (k == kNoSlot || !r_blocks_[k].valid) {
    return false;
  }
  apply_feedback(i, r_blocks_[k]);
  return true;
}

void WaveshareServos::apply_feedback(size_t i, const FeedbackBlock & b)
{
  // The shared tail of both transports, and the reason they publish the same nine doubles for the
  // same fifteen bytes by construction rather than by inspection (PHASE3 2.60, 2.63, F9).
  // moving_raw is decoded by the wrapper because the block carries it and is dropped here: no
  // tenth state interface is added by Phase 3 (PHASE3 2.12, F8).
  FeedbackSample s;
  s.status = b.status;
  s.position_ticks = b.position_ticks;
  s.speed_ticks = b.speed_ticks;
  s.load_raw = b.load_raw;
  s.current_counts = b.current_counts;
  s.voltage_raw = b.voltage_raw;
  s.temperature_raw = b.temperature_raw;
  status_bytes_[i] = s.status;
  // The only call site of unwrap_ticks(), and it is reached only for a sample that really
  // arrived -- the `valid` flag is the only gate: the accumulator is advanced by an arrived
  // sample and by nothing else, so a frozen publication is a frozen interface and not a frozen
  // count (PHASE2_SPEC 8.9, PHASE3 F17). It is the identity for a joint that does not unwrap.
  const int64_t uticks = unwrap_ticks(i, static_cast<int32_t>(s.position_ticks));
  const JointStates v = decode(s, uticks, scales_of(i));
  // A plain copy. Every unit conversion and every `sign` lives in decode(), so there is exactly
  // one place that decides which quantities `inverted` flips (D4, PHASE2_SPEC 3.2) -- and at
  // sign == 1.0 decode() is the Phase 1 arithmetic with a `1.0 *` in front, which is exact in
  // IEEE-754, so a joint that is not inverted publishes bit-identical numbers.
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
