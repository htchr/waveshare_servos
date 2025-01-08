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
#include "waveshare_servos/visibility_controls.h"
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
    // helper motor functions
    double get_position(int ID);
    double get_velocity(int ID);
    double get_torque(int ID);
    double get_temperature(int ID);
    void write_pos();
    void write_vel();

    // motor variables
    int baudrate_ = 1000000;
    std::string port_ = "/dev/ttyACM0"; // /dev/ttyTHS1 if using UART
    SMS_STS sm_st;
    double KT_ = 9.0; // torque constant (kg*cm / A)
    int steps_ = 4096;
    u16 max_speed_ = 6000; // 6000;
    u8 max_acc_ = 150; // 150;
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
    // array variables for motors
    u8*  p_ids_pnt_;
    u8*  v_ids_pnt_;
    s16* p_pos_ar_;
    u16* p_vel_ar_;
    u8*  p_acc_ar_;
    s16* v_vel_ar_;
    u8*  v_acc_ar_;
};

} // namespace waveshare_servos

#endif // WAVESHARE_SERVOS_HPP_
