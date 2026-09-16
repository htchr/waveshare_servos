#include "waveshare_servos.hpp"

#include <vector>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <tuple>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace waveshare_servos
{
hardware_interface::CallbackReturn WaveshareServos::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // check urdf definitions
  pos_offsets_.resize(info_.joints.size(), 0.0);
  size_t i = 0;
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    all_ids_.emplace_back(std::stoul(joint.parameters.find("id")->second));
    // check num, order, and type of state interfaces
    if (joint.state_interfaces.size() != 4) {
      RCLCPP_FATAL(get_logger(),
        "joint has the wrong number of state interfaces");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(get_logger(),
        "a joint does not have the position state interface first");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(get_logger(),
        "a joint does not have the velocity state interface second");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[2].name != "torque") {
      RCLCPP_FATAL(get_logger(),
        "a joint does not have the torque state interface third");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[3].name != "temperature") {
      RCLCPP_FATAL(get_logger(),
        "a joint does not have the temperature state interface fourth");
      return hardware_interface::CallbackReturn::ERROR;
    }
    // check presence and types of command interfaces
    if (joint.command_interfaces.size() < 1) {
      RCLCPP_FATAL(get_logger(),
        "a joint does not have a command interfaces");
      return hardware_interface::CallbackReturn::ERROR;
    }
    for (size_t ci = 0; ci < joint.command_interfaces.size(); ci++) {
      if (joint.command_interfaces[ci].name != hardware_interface::HW_IF_POSITION &&
        joint.command_interfaces[ci].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(get_logger(),
          "a joint is using a command interface that isn't position or velocity");
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    // store ids in different vectors by type
    if (joint.parameters.find("type")->second == "pos") {
      pos_ids_.emplace_back(std::stoul(joint.parameters.find("id")->second));
      pos_is_.emplace_back(i);
    } else if (joint.parameters.find("type")->second == "vel") {
      vel_ids_.emplace_back(std::stoul(joint.parameters.find("id")->second));
      vel_is_.emplace_back(i);
    } else {
      RCLCPP_FATAL(get_logger(),
        "a joint has the wrong type, it should be vel or pos");
      return hardware_interface::CallbackReturn::ERROR;
    }
    // save pose offsets to work around motor movement limitations
    auto offset = joint.parameters.find("offset");
    if (offset != joint.parameters.end()) {
      pos_offsets_[i] = std::stod(offset->second);
    }
    // save the position command limits: write() keeps every goal inside them, since nothing
    // upstream does by default (the controller manager only clamps with enforce_command_limits)
    double pos_min = -std::numeric_limits<double>::infinity();
    double pos_max = std::numeric_limits<double>::infinity();
    for (const hardware_interface::InterfaceInfo & ci : joint.command_interfaces) {
      if (ci.name != hardware_interface::HW_IF_POSITION) {
        continue;
      }
      try {
        if (!ci.min.empty()) {
          pos_min = std::stod(ci.min);
        }
        if (!ci.max.empty()) {
          pos_max = std::stod(ci.max);
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
    pos_mins_.emplace_back(pos_min);
    pos_maxs_.emplace_back(pos_max);
    i++;
  }
  // init vectors for state interfaces
  pos_states_.resize(all_ids_.size(), std::numeric_limits<double>::quiet_NaN());
  vel_states_.resize(all_ids_.size(), std::numeric_limits<double>::quiet_NaN());
  torq_states_.resize(all_ids_.size(), std::numeric_limits<double>::quiet_NaN());
  temp_states_.resize(all_ids_.size(), std::numeric_limits<double>::quiet_NaN());
  // create vectors for command interfaces
  pos_cmds_.resize(all_ids_.size(), std::numeric_limits<double>::quiet_NaN());
  vel_cmds_.resize(all_ids_.size(), std::numeric_limits<double>::quiet_NaN());
  // no joint is held outside its limits until on_activate (or a park) finds one there
  hold_pos_.resize(all_ids_.size(), std::numeric_limits<double>::quiet_NaN());
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
  // start servo communication
  if (!sm_st.begin(baudrate_, port_.c_str())) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  port_open_ = true;
  // A servo replies in well under a millisecond at this baud rate, so the library's stock
  // 100 ms timeout only ever costs us time: with it, every absent servo burned a tenth of a
  // second of every control cycle.
  sm_st.IOTimeOut = io_timeout_ms_;
  // ping motors and remember which ones are actually on the bus
  present_.assign(all_ids_.size(), false);
  read_fails_.assign(all_ids_.size(), 0);
  last_error_.assign(all_ids_.size(), 0);
  // nothing is measured on this port yet: a sample from before a cleanup may be stale
  measured_.assign(all_ids_.size(), false);
  for (size_t i = 0; i < all_ids_.size(); i++) {
    for (int attempt = 0; attempt < ping_attempts_ && !present_[i]; attempt++) {
      present_[i] = (sm_st.Ping(all_ids_[i]) != -1);
    }
    if (!present_[i]) {
      RCLCPP_WARN(get_logger(),
        "unable to ping motor id '%d'; joint '%s' will be skipped on the bus",
        all_ids_[i], info_.joints[i].name.c_str());
    }
  }
  build_groups();
  return hardware_interface::CallbackReturn::SUCCESS;
}

void WaveshareServos::build_groups()
{
  // Build the command groups from the servos that answered. An absent servo must never end up
  // in a sync write or a poll: it cannot answer, so it costs a full timeout every cycle.
  p_ids_.clear();
  p_js_.clear();
  for (size_t k = 0; k < pos_ids_.size(); k++) {
    if (present_[pos_is_[k]]) {
      p_ids_.emplace_back(pos_ids_[k]);
      p_js_.emplace_back(pos_is_[k]);
    }
  }
  v_ids_.clear();
  v_js_.clear();
  for (size_t k = 0; k < vel_ids_.size(); k++) {
    if (present_[vel_is_[k]]) {
      v_ids_.emplace_back(vel_ids_[k]);
      v_js_.emplace_back(vel_is_[k]);
    }
  }
  // arrays for servo commands
  p_pos_ar_.assign(p_ids_.size(), 0);
  p_vel_ar_.assign(p_ids_.size(), 0);
  p_acc_ar_.assign(p_ids_.size(), 0);
  v_vel_ar_.assign(v_ids_.size(), 0);
  v_acc_ar_.assign(v_ids_.size(), 0);
  // set motor modes: 0 = servo, 1 = closed loop wheel; set max acceleration
  for (size_t k = 0; k < p_ids_.size(); k++) {
    set_mode(p_ids_[k], 0);
    p_acc_ar_[k] = max_acc_;
  }
  for (size_t k = 0; k < v_ids_.size(); k++) {
    set_mode(v_ids_[k], 1);
    v_acc_ar_[k] = max_acc_;
  }
}

bool WaveshareServos::set_mode(u8 id, u8 mode)
{
  // Register 33 lives in EPROM. It is write protected until the lock register is cleared --
  // without that the write is silently dropped -- and the cell has a limited write endurance,
  // so only touch it when the mode is actually wrong.
  const int current = sm_st.readByte(id, SMS_STS_MODE);
  if (current == -1) {
    RCLCPP_WARN(get_logger(),
      "could not read the mode of motor id '%d'", id);
    return false;
  }
  if (current == mode) {
    return true;
  }
  sm_st.unLockEprom(id);
  sm_st.Mode(id, mode);
  sm_st.LockEprom(id);
  const int now = sm_st.readByte(id, SMS_STS_MODE);
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
  // joint_states_/joint_commands_ are not in URDF order. on_init has already checked that all four
  // state interfaces exist, so these lookups cannot throw.
  handles_.assign(info_.joints.size(), JointHandles{});
  for (size_t i = 0; i < info_.joints.size(); i++) {
    const std::string & name = info_.joints[i].name;
    JointHandles & h = handles_[i];
    h.position = get_state_interface_handle(name + "/" + hardware_interface::HW_IF_POSITION);
    h.velocity = get_state_interface_handle(name + "/" + hardware_interface::HW_IF_VELOCITY);
    h.torque = get_state_interface_handle(name + "/torque");
    h.temperature = get_state_interface_handle(name + "/temperature");
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

void WaveshareServos::push_states(size_t i, bool wait)
{
  // A non-blocking set that finds the handle busy leaves the previous sample in it; the caches are
  // published again every cycle, so it catches up on the next read().
  const JointHandles & h = handles_[i];
  std::ignore = set_state(h.position, pos_states_[i], wait);
  std::ignore = set_state(h.velocity, vel_states_[i], wait);
  std::ignore = set_state(h.torque, torq_states_[i], wait);
  std::ignore = set_state(h.temperature, temp_states_[i], wait);
}

hardware_interface::CallbackReturn WaveshareServos::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Activation is off the real-time path, so it is the right place to look again for a servo
  // that was absent at configure time or was dropped after it stopped answering. Deactivating
  // and re-activating the hardware is therefore enough to recover one, with no restart.
  bool regrouped = false;
  for (size_t i = 0; i < all_ids_.size(); i++) {
    if (present_[i]) {
      continue;
    }
    for (int attempt = 0; attempt < ping_attempts_ && !present_[i]; attempt++) {
      present_[i] = (sm_st.Ping(all_ids_[i]) != -1);
    }
    if (present_[i]) {
      RCLCPP_INFO(get_logger(),
        "motor id '%d' answered on activation; adding it back", all_ids_[i]);
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
  for (size_t i = 0; i < all_ids_.size(); i++) {
    vel_cmds_[i] = 0.0;
    push_velocity_command(i);
    hold_pos_[i] = std::numeric_limits<double>::quiet_NaN();
    if (present_[i]) {
      // A servo whose torque has been latched off -- by a protection trip, or by whatever
      // last talked to it -- accepts goal positions and quietly ignores them.
      sm_st.EnableTorque(all_ids_[i], 1);
    }
    if (present_[i] && feedback(i)) {
      pos_cmds_[i] = pos_states_[i];
      // A joint that starts outside its limits (moved by hand with the torque off, say) is not
      // run to the nearest one: write() holds it where it is until it is commanded to a
      // position inside them.
      if (outside_limits(i, pos_states_[i])) {
        hold_pos_[i] = pos_states_[i];
        RCLCPP_WARN(get_logger(),
          "joint '%s' starts at %.3f rad, outside its limits [%.3f, %.3f]; holding it there "
          "until it is commanded to a position inside them", info_.joints[i].name.c_str(),
          pos_states_[i], pos_mins_[i], pos_maxs_[i]);
      }
    } else {
      // nothing on the bus to read, so start from a neutral command rather than from the
      // -1 that a timed-out read would otherwise hand us
      pos_cmds_[i] = 0.0;
      pos_states_[i] = 0.0;
      measured_[i] = false;
      vel_states_[i] = 0.0;
      torq_states_[i] = 0.0;
      temp_states_[i] = 0.0;
    }
    // hand these to the controllers: a controller activated next starts from these commands
    push_position_command(i);
    push_states(i, true);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WaveshareServos::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // On SIGINT/SIGTERM the controller manager deactivates the hardware before it shuts it down,
  // so this also runs on ctrl-C.
  stop_and_park(false);
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool WaveshareServos::outside_limits(size_t i, double position) const
{
  // allow a step of encoder rounding
  const double step = 2 * M_PI / steps_;
  return (position < pos_mins_[i] - step) || (position > pos_maxs_[i] + step);
}

void WaveshareServos::stop_and_park(bool at_measured_positions)
{
  // start from the commands the controllers left in the handles
  for (size_t i = 0; i < all_ids_.size(); i++) {
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
  for (size_t i = 0; i < all_ids_.size(); i++) {
    const bool measured = present_[i] && feedback(i);
    if (measured) {
      pos_cmds_[i] = pos_states_[i];
      // feedback() refreshed all four states; only then are they published, as before
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
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < all_ids_.size(); i++) {
    if (!present_[i]) {
      // No servo on the bus for this joint. Mirror the command so the controllers see a
      // finite, self-consistent state instead of a timeout sentinel, and stay off the wire.
      pull_commands(i, false);
      pos_states_[i] = std::isfinite(pos_cmds_[i]) ? pos_cmds_[i] : 0.0;
      measured_[i] = false;
      vel_states_[i] = 0.0;
      torq_states_[i] = 0.0;
      temp_states_[i] = 0.0;
      continue;
    }
    if (!feedback(i)) {
      // The round trip failed. Keep the last good sample: publishing the -1 that a timed
      // out read returns would look like a real measurement 0.0015 rad from the origin.
      read_fails_[i]++;
      if (read_fails_[i] == 1 || read_fails_[i] % 200 == 0) {
        RCLCPP_WARN(get_logger(),
          "read failed for motor id '%d' (%d in a row)", all_ids_[i], read_fails_[i]);
      }
      if (read_fails_[i] >= max_read_fails_) {
        // It answered at configure time and has now gone quiet -- a brown-out, a
        // protection trip, a pulled connector. Stop polling it: otherwise it costs a
        // full timeout in every control period from here on and drags the whole loop
        // down, which is the failure this driver started with. Deactivate and activate
        // the hardware to look for it again.
        RCLCPP_ERROR(get_logger(),
          "motor id '%d' stopped answering after %d attempts; dropping it from the "
          "read cycle until the hardware is re-activated", all_ids_[i], read_fails_[i]);
        present_[i] = false;
      }
      continue;
    }
    read_fails_[i] = 0;
    // Every reply carries the servo's status byte -- overload, over-temperature, over-voltage
    // and friends. The packet layer decodes it and then throws it away; a latched protection
    // trip is exactly the sort of thing that makes a servo stop responding to goals.
    if (sm_st.Error != 0 && sm_st.Error != last_error_[i]) {
      RCLCPP_WARN(get_logger(),
        "motor id '%d' reports status byte 0x%02x", all_ids_[i], sm_st.Error);
    } else if (sm_st.Error == 0 && last_error_[i] != 0) {
      // The trip cleared. A protection trip latches torque off, and a servo in that state
      // still answers reads and still accepts goal positions -- it just ignores them.
      RCLCPP_INFO(get_logger(),
        "motor id '%d' cleared its fault; re-enabling torque", all_ids_[i]);
      sm_st.EnableTorque(all_ids_[i], 1);
    }
    last_error_[i] = sm_st.Error;
  }
  // publish every joint's state, including the last good sample of a joint whose read failed
  for (size_t i = 0; i < all_ids_.size(); i++) {
    push_states(i, false);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WaveshareServos::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // take this cycle's commands from the controllers
  for (size_t i = 0; i < all_ids_.size(); i++) {
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
  double dt = period.seconds();
  if (std::isfinite(dt) && dt > 0.0) {
    last_period_ = dt;
  } else {
    dt = last_period_;
  }
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
    if (std::isfinite(hold_pos_[j]) && ((cmd < pos_mins_[j]) || (cmd > pos_maxs_[j]))) {
      cmd = hold_pos_[j];
    } else {
      hold_pos_[j] = std::numeric_limits<double>::quiet_NaN();
      cmd = std::clamp(cmd, pos_mins_[j], pos_maxs_[j]);
    }
    const double goal_steps = (cmd + pos_offsets_[j]) * steps_ / (2 * M_PI);
    p_pos_ar_[k] = static_cast<s16>(std::lround(std::clamp(goal_steps, -32767.0, 32767.0)));
    // Pace the chord this setpoint actually adds, plus whatever the servo still owes:
    // goal(k) - measured(k) is exactly chord + lag, so one term covers both. Deliberately
    // NOT the trajectory's instantaneous velocity -- on an accelerating segment that is
    // larger than the chord's mean slope, which makes the servo finish the chord early and
    // then stand still for the rest of the period. That is the stutter we are removing.
    double speed = 0.0;
    if (std::isfinite(pos_states_[j])) {
      const double now_steps = (pos_states_[j] + pos_offsets_[j]) * steps_ / (2 * M_PI);
      speed = std::fabs(goal_steps - now_steps) / dt;
    } else if (std::isfinite(vel_cmds_[j])) {
      // no usable measurement this cycle, so fall back to the commanded velocity
      speed = std::fabs(vel_cmds_[j]) * steps_ / (2 * M_PI);
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
    p_vel_ar_[k] = static_cast<u16>(std::clamp(speed, 1.0, static_cast<double>(max_speed_)));
    p_acc_ar_[k] = max_acc_;
  }
  for (size_t k = 0; k < v_ids_.size(); k++) {
    const size_t j = v_js_[k];
    const double vel = std::isfinite(vel_cmds_[j]) ? vel_cmds_[j] : 0.0;
    const double speed = std::clamp(vel * steps_ / (2 * M_PI),
      -static_cast<double>(max_speed_), static_cast<double>(max_speed_));
    v_vel_ar_[k] = static_cast<s16>(std::lround(speed));
    v_acc_ar_[k] = max_acc_;
  }
  // Both sync writes size a variable length array from the count, so never call them with none.
  if (!p_ids_.empty()) {
    sm_st.SyncWritePosEx(p_ids_.data(), static_cast<u8>(p_ids_.size()),
      p_pos_ar_.data(), p_vel_ar_.data(), p_acc_ar_.data());
  }
  if (!v_ids_.empty()) {
    sm_st.SyncWriteSpe(v_ids_.data(), static_cast<u8>(v_ids_.size()),
      v_vel_ar_.data(), v_acc_ar_.data());
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
  // SCSerial::end() only closes a descriptor that is open and then invalidates it, so this never
  // closes twice
  sm_st.end();
  port_open_ = false;
  // release the command buffers together with the groups that index them; on_configure rebuilds
  // both
  p_ids_.clear();
  p_js_.clear();
  v_ids_.clear();
  v_js_.clear();
  p_pos_ar_.clear();
  p_vel_ar_.clear();
  p_acc_ar_.clear();
  v_vel_ar_.clear();
  v_acc_ar_.clear();
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
  park_and_close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

void WaveshareServos::park_and_close()
{
  // Only talk to the servos if on_configure opened the port: the library would otherwise select()
  // on an invalid descriptor. A failed park must still close the port and must not escape: the
  // resource manager calls on_error from its read() and write() outside its exception handling,
  // and the handles throw if the URDF gives an interface a data_type other than double.
  if (port_open_) {
    try {
      stop_and_park(true);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "could not stop and park the servos: %s", e.what());
    }
  }
  close_port();
}

bool WaveshareServos::feedback(size_t i)
{
  // One round trip fills the servo's whole feedback block (registers 56..70), so every state
  // interface comes from a single transaction instead of four separate blocking reads.
  if (sm_st.FeedBack(all_ids_[i]) == -1) {
    return false;
  }
  pos_states_[i] = sm_st.ReadPos(-1) * 2 * M_PI / steps_ - pos_offsets_[i];
  vel_states_[i] = sm_st.ReadSpeed(-1) * 2 * M_PI / steps_;
  // ReadCurrent is unitless; 6 mA per count, then the torque constant
  torq_states_[i] = sm_st.ReadCurrent(-1) * 6.0 / 1000.0 * KT_;
  temp_states_[i] = static_cast<double>(sm_st.ReadTemper(-1));
  measured_[i] = true;
  return true;
}

}  // namespace waveshare_servos

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  waveshare_servos::WaveshareServos, hardware_interface::SystemInterface)
