#include "waveshare_servos/waveshare_servos.hpp"

#include <vector>
#include <algorithm>
#include <cmath>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace waveshare_servos
{
hardware_interface::CallbackReturn WaveshareServos::on_init(
	const hardware_interface::HardwareInfo & info)
{
	if (
		hardware_interface::SystemInterface::on_init(info) !=
    	hardware_interface::CallbackReturn::SUCCESS)
  	{
    	return hardware_interface::CallbackReturn::ERROR;
  	}
	// init vectors for state interfaces
	pos_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
	vel_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
	torq_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
	temp_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
	// check urdf definitions
	for (const hardware_interface::ComponentInfo & joint : info_.joints)
	{
		all_ids_.emplace_back(std::stoul(joint.parameters.find("id")->second));
		if (joint.command_interfaces.size() != 1)
		{
			RCLCPP_FATAL(
				rclcpp::get_logger("waveshare_servos"), 
				"joint has the wrong number of cmd interfaces");
			return hardware_interface::CallbackReturn::ERROR;
		}
		if (joint.state_interfaces.size() != 4)
		{
			RCLCPP_FATAL(
				rclcpp::get_logger("waveshare_servos"),
				"joint has the wrong number of state interfaces");
			return hardware_interface::CallbackReturn::ERROR;
		}
		if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
		{
			RCLCPP_FATAL(
				rclcpp::get_logger("waveshare_servos"),
				"a joint does not have the position state interface first");
			return hardware_interface::CallbackReturn::ERROR;
		}
		if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
		{
			RCLCPP_FATAL(
				rclcpp::get_logger("waveshare_servos"),
				"a joint does not have the velocity state interface second");
			return hardware_interface::CallbackReturn::ERROR;
		}
		if (joint.state_interfaces[2].name != "torque")
		{
			RCLCPP_FATAL(
				rclcpp::get_logger("waveshare_servos"),
				"a joint does not have the torque state interface third");
			return hardware_interface::CallbackReturn::ERROR;
		}
		if (joint.state_interfaces[3].name != "temperature")
		{
			RCLCPP_FATAL(
				rclcpp::get_logger("waveshare_servos"),
				"a joint does not have the temperature state interface fourth");
			return hardware_interface::CallbackReturn::ERROR;
		}
		if (joint.parameters.find("type")->second == "pos")
		{
			pos_ids_.emplace_back(std::stoul(joint.parameters.find("id")->second));
			if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
			{
				RCLCPP_FATAL(
					rclcpp::get_logger("waveshare_servos"),
					"a position joint is not using the position command interface");
				return hardware_interface::CallbackReturn::ERROR;
			}
		} 
		else if (joint.parameters.find("type")->second == "vel") 
		{
			vel_ids_.emplace_back(std::stoul(joint.parameters.find("id")->second));
			if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
			{
				RCLCPP_FATAL(
					rclcpp::get_logger("waveshare_servos"),
					"a velocity joint is not using the velocity command interface");
				return hardware_interface::CallbackReturn::ERROR;
			}
		}
		else 
		{
			RCLCPP_FATAL(
				rclcpp::get_logger("waveshare_servos"), 
				"a joint has the wrong type, it should be vel or pos");
			return hardware_interface::CallbackReturn::ERROR;
		}
	}
	// create vectors for command interfaces
	pos_cmds_.resize(pos_ids_.size(), std::numeric_limits<double>::quiet_NaN());
	vel_cmds_.resize(vel_ids_.size(), std::numeric_limits<double>::quiet_NaN());
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WaveshareServos::on_configure(
  	const rclcpp_lifecycle::State & /*previous_state*/)
{
	// set motor modes: 0 = servo, 1 = closed loop wheel
	for (u8 i = 0; i < pos_ids_.size(); i++)
	{
		sm_st.Mode(pos_ids_[i], 0); 
	}
	for (u8 i = 0; i < vel_ids_.size(); i++)
	{
		sm_st.Mode(vel_ids_[i], 1);
	}
	// pointers to ids for motor control
	pos_ids_pnt_ = &pos_ids_[0];
	vel_ids_pnt_ = &vel_ids_[0];
	// arrays for motor control
	// pos_ar_ = new s16[pos_ids_.size()];
	pos_speed_ar_ = new u16[pos_ids_.size()];
	pos_acc_ar_ = new u8[pos_ids_.size()];
	// vel_speed_ar_ = new s16[vel_ids_.size()];
	vel_acc_ar_ = new u8[vel_ids_.size()];
	for (u8 i = 0; i < pos_ids_.size(); i++) 
	{
		pos_speed_ar_[i] = max_speed_;
		pos_acc_ar_[i] = max_acc_;
	}
	for (u8 i = 0; i< vel_ids_.size(); i++)
	{
		vel_acc_ar_[i] = max_acc_;
	}
	return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> WaveshareServos::export_state_interfaces()
{
	std::vector<hardware_interface::StateInterface> state_interfaces;
	for (u8 i = 0; i < all_ids_.size(); i++)
	{
		state_interfaces.emplace_back(hardware_interface::StateInterface(
			info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_states_[i]));
		state_interfaces.emplace_back(hardware_interface::StateInterface(
			info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
		state_interfaces.emplace_back(hardware_interface::StateInterface(
			info_.joints[i].name, "torque", &torq_states_[i]));
		state_interfaces.emplace_back(hardware_interface::StateInterface(
			info_.joints[i].name, "temperature", &temp_states_[i]));
	}
	return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> WaveshareServos::export_command_interfaces()
{
	std::vector<hardware_interface::CommandInterface> command_interfaces;
	u8 p = 0;
	u8 v = 0;
	for (const hardware_interface::ComponentInfo & joint : info_.joints)
	{
		if (joint.parameters.find("type")->second == "pos")
		{
			command_interfaces.emplace_back(hardware_interface::CommandInterface(
				joint.name, hardware_interface::HW_IF_POSITION, &pos_cmds_[p]));
			p++;
		}
		else if (joint.parameters.find("type")->second == "vel")
		{
			command_interfaces.emplace_back(hardware_interface::CommandInterface(
				joint.name, hardware_interface::HW_IF_VELOCITY, &vel_cmds_[v]));
			v++;
		}
	}
	return command_interfaces;
}

hardware_interface::CallbackReturn WaveshareServos::on_activate(
	const rclcpp_lifecycle::State & /*previous_state*/)
{
	if (!sm_st.begin(baudrate_, port_.c_str()))
	{
		return hardware_interface::CallbackReturn::ERROR;
	}
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WaveshareServos::on_deactivate(
	const rclcpp_lifecycle::State & /*previous_state*/)
{
	sm_st.end();
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type WaveshareServos::read(
	const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
	RCLCPP_INFO(
		rclcpp::get_logger("waveshare_servos"), "read");
	RCLCPP_INFO(
		rclcpp::get_logger("waveshare_servos"), "pos");
	RCLCPP_INFO(
		rclcpp::get_logger("waveshare_servos"), std::to_string(pos_states_[0]).c_str());
	RCLCPP_INFO(
		rclcpp::get_logger("waveshare_servos"), std::to_string(pos_states_[1]).c_str());
	RCLCPP_INFO(
		rclcpp::get_logger("waveshare_servos"), std::to_string(pos_states_[2]).c_str());
	RCLCPP_INFO(
		rclcpp::get_logger("waveshare_servos"), "vel");
	RCLCPP_INFO(
		rclcpp::get_logger("waveshare_servos"), std::to_string(vel_states_[0]).c_str());
	RCLCPP_INFO(
		rclcpp::get_logger("waveshare_servos"), std::to_string(vel_states_[1]).c_str());
	RCLCPP_INFO(
		rclcpp::get_logger("waveshare_servos"), std::to_string(vel_states_[2]).c_str());
	for (size_t i = 0; i < all_ids_.size(); i++)
	{
		pos_states_[i] = get_position(all_ids_[i]);
		vel_states_[i] = get_velocity(all_ids_[i]);
		torq_states_[i] = get_torque(all_ids_[i]);
		temp_states_[i] = get_temperature(all_ids_[i]);
	}
	return hardware_interface::return_type::OK;
}

hardware_interface::return_type WaveshareServos::write(
	const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
	RCLCPP_INFO(
		rclcpp::get_logger("waveshare_servos"), "write");
	RCLCPP_INFO(
		rclcpp::get_logger("waveshare_servos"), std::to_string(pos_cmds_.size()).c_str());
	RCLCPP_INFO(
		rclcpp::get_logger("waveshare_servos"), std::to_string(pos_cmds_[0]).c_str());
	RCLCPP_INFO(
		rclcpp::get_logger("waveshare_servos"), std::to_string(pos_cmds_[1]).c_str());
	RCLCPP_INFO(
		rclcpp::get_logger("waveshare_servos"), std::to_string(vel_cmds_[0]).c_str());
	
	if (pos_ids_.size())
	{
		write_pos();
	}
	if (vel_ids_.size())
	{
		write_vel();
	}
	return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn WaveshareServos::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
	delete[] pos_ar_;
	delete[] pos_speed_ar_;
	delete[] pos_acc_ar_;
	delete[] vel_speed_ar_;
	delete[] vel_acc_ar_;
	return hardware_interface::CallbackReturn::SUCCESS;
}

double WaveshareServos::get_position(int ID)
{
    double pos = sm_st.ReadPos(ID) * 2 * M_PI / steps_;
    return pos;
}

double WaveshareServos::get_velocity(int ID)
{
    // rads / s
    double vel = sm_st.ReadSpeed(ID) * 2 * M_PI / steps_;
    return vel;
}

double WaveshareServos::get_torque(int ID)
{
    // ReadCurrent(ID) return unitless value, multiply by static current (6mA)
    int current = sm_st.ReadCurrent(ID) * 6.0 / 1000.0;
    double torque = current * KT_;
    return torque;
}

double WaveshareServos::get_temperature(int ID)
{
    double temp = static_cast<double>(sm_st.ReadTemper(ID));
    return temp;
}

void WaveshareServos::write_pos()
{
	pos_ar_ = new s16[pos_ids_.size()];
    std::transform(pos_cmds_.begin(), pos_cmds_.end(), pos_ar_,
        [this](const auto& cmd) { return (cmd * steps_) / (2 * M_PI); });
    sm_st.SyncWritePosEx(pos_ids_pnt_, static_cast<u8>(pos_ids_.size()), 
		pos_ar_, pos_speed_ar_, pos_acc_ar_); 
	delete[] pos_ar_;
}

void WaveshareServos::write_vel()
{
	vel_speed_ar_ = new s16[vel_ids_.size()];
	std::transform(vel_cmds_.begin(), vel_cmds_.end(), vel_speed_ar_,
        [](const auto& cmd) { return cmd * 0; }); // TODO convert rads/s to -3400-3400
	sm_st.SyncWriteSpe(vel_ids_pnt_, static_cast<u8>(vel_ids_.size()), 
		vel_speed_ar_, vel_acc_ar_); 
	delete[] vel_speed_ar_;
}

}  // namespace waveshare_servos

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
	waveshare_servos::WaveshareServos, hardware_interface::SystemInterface)
