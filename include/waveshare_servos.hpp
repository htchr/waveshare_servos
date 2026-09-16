#ifndef WAVESHARE_SERVOS_HPP_
#define WAVESHARE_SERVOS_HPP_

#include <cstddef>
#include <vector>
#include <string>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "visibility_controls.h"  // NOLINT(build/include_subdir)
#include "SCServo.h"

namespace waveshare_servos
{
class WaveshareServos : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(WaveshareServos)

  WAVESHARE_SERVOS_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  WAVESHARE_SERVOS_PUBLIC
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces()
  override;

  WAVESHARE_SERVOS_PUBLIC
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces()
  override;

  WAVESHARE_SERVOS_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  WAVESHARE_SERVOS_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  WAVESHARE_SERVOS_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  WAVESHARE_SERVOS_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  WAVESHARE_SERVOS_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  WAVESHARE_SERVOS_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  WAVESHARE_SERVOS_PUBLIC
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  WAVESHARE_SERVOS_PUBLIC
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  // one round trip that refreshes every state interface of joint i from the servo's
  // feedback block; returns false if the servo did not answer
  bool feedback(size_t i);
  // set the servo's operating mode, unlocking the EPROM only if it needs changing
  bool set_mode(u8 id, u8 mode);
  // (re)build the present-only command groups and their command arrays
  void build_groups();
  // look up the framework-owned interface handles of every joint once (on_configure), so read()
  // and write() never hash an interface name
  void cache_handles();
  // copy the controllers' commands of joint i from the command handles into pos_cmds_/vel_cmds_;
  // wait=false on the real-time path, where a busy handle leaves the previous value in place
  void pull_commands(size_t i, bool wait);
  // publish pos_cmds_[i] / vel_cmds_[i] to the position / velocity command handle of joint i, if
  // the joint has one (blocking: lifecycle only)
  void push_position_command(size_t i);
  void push_velocity_command(size_t i);
  // publish the four state caches of joint i to its state handles
  void push_states(size_t i, bool wait);
  // turn pos_cmds_/vel_cmds_ into goals for the servos in the command groups and send them; write()
  // after it has taken the controllers' commands from the handles
  void send_commands(const rclcpp::Duration & period);
  // zero the wheel commands, park the position joints where they are and send that once.
  // Without at_measured_positions (on_deactivate) every command reaches its handle where it is set
  // and write() takes the commands from the handles, as when the controllers shared this memory.
  // at_measured_positions is for on_shutdown and on_error, which can run on a component that was
  // not activated since it was configured: each position joint is parked only at a measured
  // position, held there even outside its limits, a servo that has not returned a position since
  // on_configure or that read() dropped gets no goal at all, and the goals are sent from these
  // values rather than taken back from the handles. That drops such servos from the command groups,
  // so close the port next.
  void stop_and_park(bool at_measured_positions);
  // stop_and_park(true) if the port is open, then close_port(); a failure to park is logged and
  // never keeps the port open or escapes
  void park_and_close();
  // whether a position of joint i lies more than one encoder step outside its position limits
  bool outside_limits(size_t i, double position) const;
  // close the serial port and release the command buffers; safe to call in any state
  void close_port();

  // framework-owned handles of one joint, looked up once by name
  struct JointHandles
  {
    hardware_interface::StateInterface::SharedPtr position;
    hardware_interface::StateInterface::SharedPtr velocity;
    hardware_interface::StateInterface::SharedPtr torque;
    hardware_interface::StateInterface::SharedPtr temperature;
    // null when the joint does not declare that command interface in the URDF
    hardware_interface::CommandInterface::SharedPtr position_cmd;
    hardware_interface::CommandInterface::SharedPtr velocity_cmd;
  };
  std::vector<JointHandles> handles_;
  // true between a successful begin() in on_configure and close_port()
  bool port_open_ = false;

  // motor variables
  int baudrate_ = 1000000;
  std::string port_ = "/dev/ttyACM0";   // /dev/ttyTHS1 if using UART
  SMS_STS sm_st;
  double KT_ = 9.0;   // torque constant (kg*cm / A)
  int steps_ = 4096;
  u16 max_speed_ = 6000;   // 6000;
  u8 max_acc_ = 150;   // 150;
  // A servo answers in well under a millisecond at 1 Mbaud. The library's stock 100 ms
  // timeout turned every absent servo into a tenth of a second of dead time in every
  // control cycle, which is what held the loop down to ~1.1 Hz.
  u32 io_timeout_ms_ = 20;  // same type as SCSerial::IOTimeOut
  int ping_attempts_ = 3;
  int max_read_fails_ = 50;
  // id group variables
  std::vector<u8> all_ids_;
  std::vector<u8> pos_ids_;
  std::vector<u8> vel_ids_;
  std::vector<size_t> pos_is_;
  std::vector<size_t> vel_is_;
  // command variables: the last commands pulled from the command handles (and what the
  // hardware itself commands in on_activate/on_deactivate); write() and read() use only these
  std::vector<double> pos_cmds_;
  std::vector<double> vel_cmds_;
  // state variables: the latest measurement, published to the state handles
  std::vector<double> pos_states_;
  std::vector<double> vel_states_;
  std::vector<double> torq_states_;
  std::vector<double> temp_states_;
  // vector for position offsets
  std::vector<double> pos_offsets_;
  // position command limits from the ros2_control min/max params, +/-inf where none is given
  std::vector<double> pos_mins_;
  std::vector<double> pos_maxs_;
  // where a joint that on_activate (or the park of on_shutdown/on_error) found outside those
  // limits is held until it is commanded to a position inside them; NaN for every other joint
  std::vector<double> hold_pos_;
  // which servos answered Ping; absent ones are never put on the bus
  std::vector<bool> present_;
  std::vector<int> read_fails_;
  std::vector<u8> last_error_;
  // whether pos_states_ holds a position the servo returned since on_configure, rather than a
  // stand-in (NaN, on_activate's 0.0, or read()'s mirror of the command of an absent servo)
  std::vector<bool> measured_;
  // command groups containing only servos that are present, with the joint index each maps to
  std::vector<u8> p_ids_;
  std::vector<size_t> p_js_;
  std::vector<u8> v_ids_;
  std::vector<size_t> v_js_;
  // last non-zero control period, used to pace the goal speed
  double last_period_ = 0.01;
  // array variables for motors, one entry per servo in p_ids_ / v_ids_; the sync writes
  // rewrite the position and speed arrays in place, so write() refills them every cycle
  std::vector<s16> p_pos_ar_;
  std::vector<u16> p_vel_ar_;
  std::vector<u8> p_acc_ar_;
  std::vector<s16> v_vel_ar_;
  std::vector<u8> v_acc_ar_;
};

}  // namespace waveshare_servos

#endif  // WAVESHARE_SERVOS_HPP_
