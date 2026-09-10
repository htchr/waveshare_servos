#include "waveshare_servos.hpp"

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
	// check urdf definitions
	pos_offsets_.resize(info_.joints.size(), 0.0);
	int i = 0;
	for (const hardware_interface::ComponentInfo & joint : info_.joints)
	{
		all_ids_.emplace_back(std::stoul(joint.parameters.find("id")->second));
		// check num, order, and type of state interfaces
		if (joint.state_interfaces.size() != 4)
		{
			RCLCPP_FATAL(rclcpp::get_logger("waveshare_servos"),
				"joint has the wrong number of state interfaces");
			return hardware_interface::CallbackReturn::ERROR;
		}
		if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
		{
			RCLCPP_FATAL(rclcpp::get_logger("waveshare_servos"),
				"a joint does not have the position state interface first");
			return hardware_interface::CallbackReturn::ERROR;
		}
		if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
		{
			RCLCPP_FATAL(rclcpp::get_logger("waveshare_servos"),
				"a joint does not have the velocity state interface second");
			return hardware_interface::CallbackReturn::ERROR;
		}
		if (joint.state_interfaces[2].name != "torque")
		{
			RCLCPP_FATAL(rclcpp::get_logger("waveshare_servos"),
				"a joint does not have the torque state interface third");
			return hardware_interface::CallbackReturn::ERROR;
		}
		if (joint.state_interfaces[3].name != "temperature")
		{
			RCLCPP_FATAL(rclcpp::get_logger("waveshare_servos"),
				"a joint does not have the temperature state interface fourth");
			return hardware_interface::CallbackReturn::ERROR;
		}
		// check presence and types of command interfaces
		if (joint.command_interfaces.size() < 1)
		{
			RCLCPP_FATAL(rclcpp::get_logger("waveshare_servos"), 
				"a joint does not have a command interfaces");
			return hardware_interface::CallbackReturn::ERROR;
		}
		for (long unsigned int ci = 0; ci < joint.command_interfaces.size(); ci++)
		{
			if (joint.command_interfaces[ci].name != hardware_interface::HW_IF_POSITION &&
				joint.command_interfaces[ci].name != hardware_interface::HW_IF_VELOCITY)
			{
				RCLCPP_FATAL(rclcpp::get_logger("waveshare_servos"),
					"a joint is using a command interface that isn't position or velocity");
				return hardware_interface::CallbackReturn::ERROR;
			}
		}
		// store ids in different vectors by type
		if (joint.parameters.find("type")->second == "pos")
		{
			pos_ids_.emplace_back(std::stoul(joint.parameters.find("id")->second));
			pos_is_.emplace_back(i);
		} 
		else if (joint.parameters.find("type")->second == "vel") 
		{
			vel_ids_.emplace_back(std::stoul(joint.parameters.find("id")->second));
			vel_is_.emplace_back(i);
		}
		else 
		{
			RCLCPP_FATAL(rclcpp::get_logger("waveshare_servos"), 
				"a joint has the wrong type, it should be vel or pos");
			return hardware_interface::CallbackReturn::ERROR;
		}
		// save pose offsets to work around motor movement limitations
		auto offset = joint.parameters.find("offset");
    	if (offset != joint.parameters.end())
    	{
      		pos_offsets_[i] = std::stod(offset->second);  
    	}
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
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WaveshareServos::on_configure(
  	const rclcpp_lifecycle::State & /*previous_state*/)
{
	// start servo communication
	if (!sm_st.begin(baudrate_, port_.c_str()))
	{
		return hardware_interface::CallbackReturn::ERROR;
	}
	// A servo replies in well under a millisecond at this baud rate, so the library's stock
	// 100 ms timeout only ever costs us time: with it, every absent servo burned a tenth of a
	// second of every control cycle.
	sm_st.IOTimeOut = io_timeout_ms_;
	// ping motors and remember which ones are actually on the bus
	present_.assign(all_ids_.size(), false);
	read_fails_.assign(all_ids_.size(), 0);
	last_error_.assign(all_ids_.size(), 0);
	for (size_t i = 0; i < all_ids_.size(); i++)
	{
		for (int attempt = 0; attempt < ping_attempts_ && !present_[i]; attempt++)
		{
			present_[i] = (sm_st.Ping(all_ids_[i]) != -1);
		}
		if (!present_[i])
		{
			RCLCPP_WARN(rclcpp::get_logger("waveshare_servos"), 
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
	for (size_t k = 0; k < pos_ids_.size(); k++)
	{
		if (present_[pos_is_[k]])
		{
			p_ids_.emplace_back(pos_ids_[k]);
			p_js_.emplace_back(pos_is_[k]);
		}
	}
	v_ids_.clear();
	v_js_.clear();
	for (size_t k = 0; k < vel_ids_.size(); k++)
	{
		if (present_[vel_is_[k]])
		{
			v_ids_.emplace_back(vel_ids_[k]);
			v_js_.emplace_back(vel_is_[k]);
		}
	}
	// arrays for servo commands
	delete[] p_pos_ar_;
	delete[] p_vel_ar_;
	delete[] p_acc_ar_;
	delete[] v_vel_ar_;
	delete[] v_acc_ar_;
	p_pos_ar_ = new s16[p_ids_.size()];
	p_vel_ar_ = new u16[p_ids_.size()];
	p_acc_ar_ = new  u8[p_ids_.size()];
	v_vel_ar_ = new s16[v_ids_.size()];
	v_acc_ar_ = new  u8[v_ids_.size()];
	// set motor modes: 0 = servo, 1 = closed loop wheel; set max acceleration
	for (size_t k = 0; k < p_ids_.size(); k++)
	{
		set_mode(p_ids_[k], 0);
		p_acc_ar_[k] = max_acc_;
	}
	for (size_t k = 0; k < v_ids_.size(); k++)
	{
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
	if (current == -1)
	{
		RCLCPP_WARN(rclcpp::get_logger("waveshare_servos"),
			"could not read the mode of motor id '%d'", id);
		return false;
	}
	if (current == mode)
	{
		return true;
	}
	sm_st.unLockEprom(id);
	sm_st.Mode(id, mode);
	sm_st.LockEprom(id);
	const int now = sm_st.readByte(id, SMS_STS_MODE);
	if (now != mode)
	{
		RCLCPP_ERROR(rclcpp::get_logger("waveshare_servos"),
			"failed to set motor id '%d' to mode %d, it is still in mode %d", id, mode, now);
		return false;
	}
	RCLCPP_INFO(rclcpp::get_logger("waveshare_servos"),
		"motor id '%d' mode changed from %d to %d", id, current, mode);
	return true;
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
	for (u8 i = 0; i < all_ids_.size(); i++)
	{
		for (long unsigned int ci = 0; ci < info_.joints[i].command_interfaces.size(); ci++)
		{
			if (info_.joints[i].command_interfaces[ci].name == hardware_interface::HW_IF_POSITION) 
			{
				command_interfaces.emplace_back(hardware_interface::CommandInterface(
					info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_cmds_[i]));
			}
			if (info_.joints[i].command_interfaces[ci].name == hardware_interface::HW_IF_VELOCITY) 
			{
				command_interfaces.emplace_back(hardware_interface::CommandInterface(
					info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_cmds_[i]));
			}
		}
	}
	return command_interfaces;
}

hardware_interface::CallbackReturn WaveshareServos::on_activate(
	const rclcpp_lifecycle::State & /*previous_state*/)
{
	// Activation is off the real-time path, so it is the right place to look again for a servo
	// that was absent at configure time or was dropped after it stopped answering. Deactivating
	// and re-activating the hardware is therefore enough to recover one, with no restart.
	bool regrouped = false;
	for (size_t i = 0; i < all_ids_.size(); i++)
	{
		if (present_[i])
		{
			continue;
		}
		for (int attempt = 0; attempt < ping_attempts_ && !present_[i]; attempt++)
		{
			present_[i] = (sm_st.Ping(all_ids_[i]) != -1);
		}
		if (present_[i])
		{
			RCLCPP_INFO(rclcpp::get_logger("waveshare_servos"),
				"motor id '%d' answered on activation; adding it back", all_ids_[i]);
			read_fails_[i] = 0;
			regrouped = true;
		}
	}
	if (regrouped)
	{
		build_groups();
	}
	// set position commands to current positions before any movement to not move on start
	for (size_t i = 0; i < all_ids_.size(); i++)
  	{
		vel_cmds_[i] = 0.0;
		if (present_[i])
		{
			// A servo whose torque has been latched off -- by a protection trip, or by whatever
			// last talked to it -- accepts goal positions and quietly ignores them.
			sm_st.EnableTorque(all_ids_[i], 1);
		}
		if (present_[i] && feedback(i))
		{
			pos_cmds_[i] = pos_states_[i];
		}
		else
		{
			// nothing on the bus to read, so start from a neutral command rather than from the
			// -1 that a timed-out read would otherwise hand us
			pos_cmds_[i] = 0.0;
			pos_states_[i] = 0.0;
			vel_states_[i] = 0.0;
			torq_states_[i] = 0.0;
			temp_states_[i] = 0.0;
		}
	}
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WaveshareServos::on_deactivate(
	const rclcpp_lifecycle::State & /*previous_state*/)
{
	// set velocities to 0 on close, doesn't work on ctrl-C
	for (size_t i = 0; i < vel_cmds_.size(); i++)
	{
    	vel_cmds_[i] = 0.0;
  	}
	// and park the position joints where they actually are, so shutdown cannot run them off to
	// a goal they had not reached yet
	for (size_t i = 0; i < all_ids_.size(); i++)
	{
		if (present_[i] && feedback(i))
		{
			pos_cmds_[i] = pos_states_[i];
		}
	}
	auto now    = rclcpp::Clock().now();
  	auto period = rclcpp::Duration(0, 0);  // zero duration
  	this->write(now, period);
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type WaveshareServos::read(
	const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
	for (size_t i = 0; i < all_ids_.size(); i++)
	{
		if (!present_[i])
		{
			// No servo on the bus for this joint. Mirror the command so the controllers see a
			// finite, self-consistent state instead of a timeout sentinel, and stay off the wire.
			pos_states_[i] = std::isfinite(pos_cmds_[i]) ? pos_cmds_[i] : 0.0;
			vel_states_[i] = 0.0;
			torq_states_[i] = 0.0;
			temp_states_[i] = 0.0;
			continue;
		}
		if (!feedback(i))
		{
			// The round trip failed. Keep the last good sample: publishing the -1 that a timed
			// out read returns would look like a real measurement 0.0015 rad from the origin.
			read_fails_[i]++;
			if (read_fails_[i] == 1 || read_fails_[i] % 200 == 0)
			{
				RCLCPP_WARN(rclcpp::get_logger("waveshare_servos"),
					"read failed for motor id '%d' (%d in a row)", all_ids_[i], read_fails_[i]);
			}
			if (read_fails_[i] >= max_read_fails_)
			{
				// It answered at configure time and has now gone quiet -- a brown-out, a
				// protection trip, a pulled connector. Stop polling it: otherwise it costs a
				// full timeout in every control period from here on and drags the whole loop
				// down, which is the failure this driver started with. Deactivate and activate
				// the hardware to look for it again.
				RCLCPP_ERROR(rclcpp::get_logger("waveshare_servos"),
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
		if (sm_st.Error != 0 && sm_st.Error != last_error_[i])
		{
			RCLCPP_WARN(rclcpp::get_logger("waveshare_servos"),
				"motor id '%d' reports status byte 0x%02x", all_ids_[i], sm_st.Error);
		}
		else if (sm_st.Error == 0 && last_error_[i] != 0)
		{
			// The trip cleared. A protection trip latches torque off, and a servo in that state
			// still answers reads and still accepts goal positions -- it just ignores them.
			RCLCPP_INFO(rclcpp::get_logger("waveshare_servos"),
				"motor id '%d' cleared its fault; re-enabling torque", all_ids_[i]);
			sm_st.EnableTorque(all_ids_[i], 1);
		}
		last_error_[i] = sm_st.Error;
	}
	return hardware_interface::return_type::OK;
}

hardware_interface::return_type WaveshareServos::write(
	const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
	// The servo runs its own trapezoidal profile to the goal position and stops dead on arrival,
	// so the goal speed is what decides whether a stream of setpoints comes out as continuous
	// motion or as a sequence of sprints and dwells. Pace it to arrive just as the next setpoint
	// is written.
	double dt = period.seconds();
	if (std::isfinite(dt) && dt > 0.0)
	{
		last_period_ = dt;
	}
	else
	{
		dt = last_period_;
	}
	for (size_t k = 0; k < p_ids_.size(); k++)
	{
		const int j = p_js_[k];
		// Until the first controller update the command is still NaN; hold station rather than
		// converting NaN to a garbage step count.
		double cmd = pos_cmds_[j];
		if (!std::isfinite(cmd))
		{
			cmd = std::isfinite(pos_states_[j]) ? pos_states_[j] : 0.0;
		}
		const double goal_steps = (cmd + pos_offsets_[j]) * steps_ / (2 * M_PI);
		p_pos_ar_[k] = static_cast<s16>(std::lround(std::clamp(goal_steps, -32767.0, 32767.0)));
		// Pace the chord this setpoint actually adds, plus whatever the servo still owes:
		// goal(k) - measured(k) is exactly chord + lag, so one term covers both. Deliberately
		// NOT the trajectory's instantaneous velocity -- on an accelerating segment that is
		// larger than the chord's mean slope, which makes the servo finish the chord early and
		// then stand still for the rest of the period. That is the stutter we are removing.
		double speed = 0.0;
		if (std::isfinite(pos_states_[j]))
		{
			const double now_steps = (pos_states_[j] + pos_offsets_[j]) * steps_ / (2 * M_PI);
			speed = std::fabs(goal_steps - now_steps) / dt;
		}
		else if (std::isfinite(vel_cmds_[j]))
		{
			// no usable measurement this cycle, so fall back to the commanded velocity
			speed = std::fabs(vel_cmds_[j]) * steps_ / (2 * M_PI);
		}
		if (!std::isfinite(speed))
		{
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
	for (size_t k = 0; k < v_ids_.size(); k++)
	{
		const int j = v_js_[k];
		const double vel = std::isfinite(vel_cmds_[j]) ? vel_cmds_[j] : 0.0;
		const double speed = std::clamp(vel * steps_ / (2 * M_PI),
			-static_cast<double>(max_speed_), static_cast<double>(max_speed_));
		v_vel_ar_[k] = static_cast<s16>(std::lround(speed));
		v_acc_ar_[k] = max_acc_;
	}
	// Both sync writes size a variable length array from the count, so never call them with none.
	if (!p_ids_.empty())
	{
		sm_st.SyncWritePosEx(p_ids_.data(), static_cast<u8>(p_ids_.size()), 
			p_pos_ar_, p_vel_ar_, p_acc_ar_); 
	}
	if (!v_ids_.empty())
	{
		sm_st.SyncWriteSpe(v_ids_.data(), static_cast<u8>(v_ids_.size()), 
			v_vel_ar_, v_acc_ar_); 
	}
	return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn WaveshareServos::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
	sm_st.end();
	delete[] p_pos_ar_;
	delete[] p_vel_ar_;
	delete[] p_acc_ar_;
	delete[] v_vel_ar_;
	delete[] v_acc_ar_;
	p_pos_ar_ = nullptr;
	p_vel_ar_ = nullptr;
	p_acc_ar_ = nullptr;
	v_vel_ar_ = nullptr;
	v_acc_ar_ = nullptr;
	return hardware_interface::CallbackReturn::SUCCESS;
}

bool WaveshareServos::feedback(int i)
{
	// One round trip fills the servo's whole feedback block (registers 56..70), so every state
	// interface comes from a single transaction instead of four separate blocking reads.
	if (sm_st.FeedBack(all_ids_[i]) == -1)
	{
		return false;
	}
	pos_states_[i] = sm_st.ReadPos(-1) * 2 * M_PI / steps_ - pos_offsets_[i];
	vel_states_[i] = sm_st.ReadSpeed(-1) * 2 * M_PI / steps_;
	// ReadCurrent is unitless; 6 mA per count, then the torque constant
	torq_states_[i] = sm_st.ReadCurrent(-1) * 6.0 / 1000.0 * KT_;
	temp_states_[i] = static_cast<double>(sm_st.ReadTemper(-1));
	return true;
}

}  // namespace waveshare_servos

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
	waveshare_servos::WaveshareServos, hardware_interface::SystemInterface)
