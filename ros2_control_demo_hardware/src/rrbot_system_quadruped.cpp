// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros2_control_demo_hardware/rrbot_system_quadruped.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_hardware
{


return_type RRBotSystemQuadrupedHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);

  // For each joint.
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    hw_states_[joint.name] =
      {std::numeric_limits<double>::quiet_NaN(),
       std::numeric_limits<double>::quiet_NaN(),
       std::numeric_limits<double>::quiet_NaN(),
       std::numeric_limits<double>::quiet_NaN(),
       std::numeric_limits<double>::quiet_NaN()};
    hw_commands_[joint.name] =
      {std::numeric_limits<double>::quiet_NaN(),
       std::numeric_limits<double>::quiet_NaN(),
       std::numeric_limits<double>::quiet_NaN(),
       std::numeric_limits<double>::quiet_NaN(),
       std::numeric_limits<double>::quiet_NaN()};
    control_mode_[joint.name] = control_mode_t::NO_VALID_MODE;


    // RRBotSystemQuadruped has exactly 5 doubles for the state and
    // 5 doubles for the command interface on each joint
    if (joint.command_interfaces.size() !=
      quad_list_of_cmd_inter.size())
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
        "Joint '%s' has %d command interfaces found. 5 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return return_type::ERROR;
    }

    // For each command interface of the joint
    for (const auto & a_joint_cmd_inter : joint.command_interfaces)
    {
      // Check if the command interface is inside the list
      if (quad_list_of_cmd_inter.find(a_joint_cmd_inter.name) ==
        quad_list_of_cmd_inter.end())
      {
        // If not then generate an error message
        RCLCPP_FATAL(
          rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
          "Joint '%s' have %s command interfaces found. One of the following values is expected",
          joint.name.c_str(),
          a_joint_cmd_inter.name.c_str());
        for (const auto & a_cmd_inter : quad_list_of_cmd_inter)
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
            "'%s' expected.", a_cmd_inter.c_str());
        }
        return return_type::ERROR;
      }
    }

    // Check if the state interface list is of the right size
    if (joint.state_interfaces.size() !=
      quad_list_of_state_inter.size())
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
        "Joint '%s' has %d state interface. 5 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return return_type::ERROR;
    }

    // For each state interface of the joint
    for (const auto & a_joint_state_inter : joint.state_interfaces)
    {
      std::string joint_state_inter_name = a_joint_state_inter.name;

      // Check if the state interface is inside the list
      if (quad_list_of_state_inter.find(joint_state_inter_name) ==
        quad_list_of_state_inter.end())
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
          "Joint '%s' have %s state interface. One of the following was expected: ",
          joint.name.c_str(),
          a_joint_state_inter.name.c_str());

        for (const auto & a_state_inter : quad_list_of_state_inter)
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
            "'%s' expected.", a_state_inter.c_str());
        }
        return return_type::ERROR;
      }
    }
  }

  status_ = hardware_interface::status::CONFIGURED;
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
    "Finished configure() %p",this);

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
      "Joint '%s' configuration.",
      joint.name.c_str());
  }
  return return_type::OK;
}


return_type
RRBotSystemQuadrupedHardware::prepare_command_mode_switch
(
 const std::vector<std::string> & start_interfaces,
 const std::vector<std::string> & stop_interfaces
 )
{

  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
    "Going through RRBotSystemQuadrupedHardware::prepare_command_mode_switch %d",
    start_interfaces.size());

  // Initialize new modes.
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
    new_modes_[joint.name] = control_mode_t::NO_VALID_MODE;

  /// Check that the key interfaces are coherent
  for (auto & key : start_interfaces) {
    RCLCPP_INFO(rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
		"prepare_command_mode_switch %s",key.c_str());

    /// For each joint
    for (const hardware_interface::ComponentInfo & joint : info_.joints) {
      if (key == joint.name + "/" + hardware_interface::HW_IF_POSITION)
	{
	  new_modes_[joint.name]=control_mode_t::POSITION;
	  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
		      "%s switch to position",key.c_str());

	}
      if (key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY)
	{
	  new_modes_[joint.name]=control_mode_t::VELOCITY;
	}
      if (key == joint.name + "/" + hardware_interface::HW_IF_EFFORT)
	{
	  new_modes_[joint.name]=control_mode_t::EFFORT;
	}
      if (key == joint.name + "/" + ros2_control_demo_hardware::HW_IF_GAIN_KP)
	{
	  new_modes_[joint.name]=control_mode_t::POS_VEL_EFF_GAINS;
	}
      if (key == joint.name + "/" + ros2_control_demo_hardware::HW_IF_GAIN_KD)
	{
	  new_modes_[joint.name]=control_mode_t::POS_VEL_EFF_GAINS;
	}
    }
  }
  // Stop motion on all relevant joints that are stopping
  for (std::string key : stop_interfaces)
  {
    for (const hardware_interface::ComponentInfo & joint : info_.joints) {
      if (key.find(joint.name) != std::string::npos)
      {
        hw_commands_[joint.name].velocity = 0.0;
        hw_commands_[joint.name].effort = 0.0;
        control_mode_[joint.name] = control_mode_t::NO_VALID_MODE;  // Revert to undefined
      }
    }
  }
  // Set the new command modes
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if ((control_mode_[joint.name] == control_mode_t::NO_VALID_MODE) &&
	(new_modes_[joint.name]==control_mode_t::NO_VALID_MODE))
    {
      // Something else is using the joint! Abort!
      RCLCPP_ERROR(
        rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
	"Joint '%s' has no valid control mode %d %d",
	joint.name.c_str(),
	control_mode_[joint.name],
	new_modes_[joint.name]

      );
      return return_type::ERROR;
    }
    control_mode_[joint.name] = new_modes_[joint.name];
  }

  return return_type::OK;
}

std::vector<hardware_interface::StateInterface>
RRBotSystemQuadrupedHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_POSITION,
        &hw_states_[joint.name].position));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY,
        &hw_states_[joint.name].velocity));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_EFFORT,
        &hw_states_[joint.name].effort));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, HW_IF_GAIN_KP,
        &hw_states_[joint.name].effort));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, HW_IF_GAIN_KD,
        &hw_states_[joint.name].effort));
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
      "Joint '%s' export_state_interface.",
      joint.name.c_str());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RRBotSystemQuadrupedHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_POSITION,
        &hw_commands_[joint.name].position));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY,
        &hw_commands_[joint.name].velocity));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_EFFORT,
        &hw_commands_[joint.name].effort));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, HW_IF_GAIN_KP,
        &hw_commands_[joint.name].Kp));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, HW_IF_GAIN_KD,
        &hw_commands_[joint.name].Kd));
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
      "Joint '%s' export_command_interface.",
      joint.name.c_str());
  }

  return command_interfaces;
}


return_type RRBotSystemQuadrupedHardware::start()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
    "Starting ...please wait...");

  for (auto i = 0; i <= hw_start_sec_; ++i) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
      "%.1f seconds left...", hw_start_sec_ - i);
  }

  // set some default values
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (std::isnan(hw_states_[joint.name].position)) {
      hw_states_[joint.name] = {0.0, 0.0, 0.0, 0.0, 0.0};
      hw_commands_[joint.name] = {0.0, 0.0, 0.0, 0.0, 0.0};
    }
  }

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
    "System Sucessfully started!");

  return return_type::OK;
}

return_type RRBotSystemQuadrupedHardware::stop()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
    "Stopping ...please wait...");

  for (int i = 0; i <= hw_stop_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
      "%.1f seconds left...", hw_stop_sec_ - i);
  }

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
    "System sucessfully stopped!");

  return return_type::OK;
}

hardware_interface::return_type RRBotSystemQuadrupedHardware::read()
{
  // RCLCPP_INFO(
  //   rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
  //   "Reading...");
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    switch(control_mode_[joint.name]) {
    case control_mode_t::POS_VEL_EFF_GAINS:
      // Simulate RRBotQuadruped's PD+ computation
      hw_states_[joint.name].effort = hw_commands_[joint.name].effort +
	hw_commands_[joint.name].Kp*
	(hw_states_[joint.name].position -
	 hw_commands_[joint.name].position) +
	hw_commands_[joint.name].Kd*
	(hw_states_[joint.name].velocity -
	 hw_commands_[joint.name].velocity);
      break;
    case control_mode_t::POSITION:
      hw_states_[joint.name].position =
	hw_commands_[joint.name].position;
      break;
    case control_mode_t::VELOCITY:
      hw_states_[joint.name].position =
	hw_states_[joint.name].position +
	hw_slowdown_ * hw_states_[joint.name].velocity;
      hw_states_[joint.name].velocity =
	hw_commands_[joint.name].velocity;
      break;
    case control_mode_t::EFFORT:
      hw_states_[joint.name].effort =
	hw_commands_[joint.name].effort;
      break;
    case control_mode_t::NO_VALID_MODE:
      hw_states_[joint.name].effort = 0.0;
      break;
    }
    //
    // RCLCPP_INFO(
    //   rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
    //   "Got state (%.5f,%.5f,%.5f) for joint %d!",
    //   hw_states_[i].position, hw_states_[i].velocity,
    //   hw_states_[i].effort, i);
  }
  // RCLCPP_INFO(
  //   rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
  //   "Joints sucessfully read!");

  return return_type::OK;
}

hardware_interface::return_type
RRBotSystemQuadrupedHardware::write()
{
  // This part of the code sends command to the real robot.

  // RCLCPP_INFO(
  //   rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
  //   "Writing...");
  //  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // Simulate sending commands to the hardware
    // RCLCPP_INFO(
    //   rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
    //   "Got command (%.5f,%.5f,%.5f,%.5f,%.5f) for joint %d!",
    //   hw_commands_[i].position,
    //   hw_commands_[i].velocity,
    //   hw_commands_[i].effort,
    //   hw_commands_[i].Kp,
    //   hw_commands_[i].Kd,
    //   i);
  // }
  // RCLCPP_INFO(
  //   rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
  //   "Joints sucessfully written!");

  return return_type::OK;
}

}  // namespace ros2_control_demo_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_hardware::RRBotSystemQuadrupedHardware,
  hardware_interface::SystemInterface
)
