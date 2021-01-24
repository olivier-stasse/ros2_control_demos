// Copyright 2020 ros2_control Development Team
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
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 8) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
        "Joint '%s' has %d command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != "pos_vel_effort_gains") {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        "pos_vel_effort_gains");
      return return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 8) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
        "Joint '%s' has %d state interface. 8 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != HW_PVEG) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
        "Joint '%s' have %s state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        HW_PVEG);
      return return_type::ERROR;
    }
  }

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface>
RRBotSystemQuadrupedHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, HW_PVEG, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RRBotSystemQuadrupedHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, HW_PVEG, &hw_commands_[i]));
  }

  return command_interfaces;
}


return_type RRBotSystemQuadrupedHardware::start()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
    "Starting ...please wait...");

  for (int i = 0; i <= hw_start_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
      "%.1f seconds left...", hw_start_sec_ - i);
  }

  // set some default values
  for (uint i = 0; i < hw_states_.size(); i++) {
    if (std::isnan(hw_states_[i])) {
      hw_states_[i] = 0;
      hw_commands_[i] = 0;
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
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
    "Reading...");

  for (uint i = 0; i < hw_states_.size(); i++) {
    // Simulate RRBot's movement
    hw_states_[i] = hw_commands_[i] + (hw_states_[i] - hw_commands_[i]) / hw_slowdown_;
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
      "Got state %.5f for joint %d!", hw_states_[i], i);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
    "Joints sucessfully read!");

  return return_type::OK;
}

hardware_interface::return_type ros2_control_demo_hardware::RRBotSystemQuadrupedHardware::write()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
    "Writing...");

  for (uint i = 0; i < hw_commands_.size(); i++) {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
      "Got command %.5f for joint %d!", hw_commands_[i], i);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemQuadrupedHardware"),
    "Joints sucessfully written!");

  return return_type::OK;
}

}  // namespace ros2_control_demo_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_hardware::RRBotSystemQuadrupedHardware,
  hardware_interface::SystemInterface
)
