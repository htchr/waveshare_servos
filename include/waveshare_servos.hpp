#ifndef WAVESHARE_SERVOS_HPP_
#define WAVESHARE_SERVOS_HPP_

#include <vector>
#include <string>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "visibility_controls.h"
#include "SCServo.h"

namespace waveshare_servos
{
class WaveshareServos : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(WaveshareServos)

    WAVESHARE_SERVOS_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;
    
    WAVESHARE_SERVOS_PUBLIC
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    WAVESHARE_SERVOS_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    WAVESHARE_SERVOS_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

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

private:
    // one round trip that refreshes every state interface of joint i from the servo's
    // feedback block; returns false if the servo did not answer
    bool feedback(int i);
    // set the servo's operating mode, unlocking the EPROM only if it needs changing
    bool set_mode(u8 id, u8 mode);
    // (re)build the present-only command groups and their command arrays
    void build_groups();

    // motor variables
    int baudrate_ = 1000000;
    std::string port_ = "/dev/ttyACM0"; // /dev/ttyTHS1 if using UART
    SMS_STS sm_st;
    double KT_ = 9.0; // torque constant (kg*cm / A)
    int steps_ = 4096;
    u16 max_speed_ = 6000; // 6000;
    u8 max_acc_ = 150; // 150;
    // A servo answers in well under a millisecond at 1 Mbaud. The library's stock 100 ms
    // timeout turned every absent servo into a tenth of a second of dead time in every
    // control cycle, which is what held the loop down to ~1.1 Hz.
    unsigned long io_timeout_ms_ = 20;
    int ping_attempts_ = 3;
    int max_read_fails_ = 50;
    // id group variables
    std::vector<u8> all_ids_;
	std::vector<u8> pos_ids_;
	std::vector<u8> vel_ids_;
    std::vector<int> pos_is_;
    std::vector<int> vel_is_;
    // command interface variables
    std::vector<double> pos_cmds_;
    std::vector<double> vel_cmds_;
    // state interface variables
    std::vector<double> pos_states_;
    std::vector<double> vel_states_;
    std::vector<double> torq_states_;
    std::vector<double> temp_states_;
    // vector for position offsets
    std::vector<double> pos_offsets_;
    // position command limits from the ros2_control min/max params, +/-inf where none is given
    std::vector<double> pos_mins_;
    std::vector<double> pos_maxs_;
    // where a joint that started outside those limits is held until it is commanded to a
    // position inside them; NaN for every other joint
    std::vector<double> hold_pos_;
    // which servos answered Ping; absent ones are never put on the bus
    std::vector<bool> present_;
    std::vector<int> read_fails_;
    std::vector<u8> last_error_;
    // command groups containing only servos that are present, with the joint index each maps to
    std::vector<u8>  p_ids_;
    std::vector<int> p_js_;
    std::vector<u8>  v_ids_;
    std::vector<int> v_js_;
    // last non-zero control period, used to pace the goal speed
    double last_period_ = 0.01;
    // array variables for motors
    s16* p_pos_ar_ = nullptr;
    u16* p_vel_ar_ = nullptr;
    u8*  p_acc_ar_ = nullptr;
    s16* v_vel_ar_ = nullptr;
    u8*  v_acc_ar_ = nullptr;
};

} // namespace waveshare_servos

#endif // WAVESHARE_SERVOS_HPP_
