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

#ifndef ROS2_CONTROL_DEMO_HARDWARE__RRBOT_SYSTEM_QUADRUPED_HPP_
#define ROS2_CONTROL_DEMO_HARDWARE__RRBOT_SYSTEM_QUADRUPED_HPP_

#include <memory>
#include <set>
#include <string>
#include <vector>
#include <map>
#include "rclcpp/macros.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "ros2_control_demo_hardware/visibility_control.h"

using hardware_interface::return_type;

namespace ros2_control_demo_hardware
{


struct PosVelEffortGains
{
  double position;
  double velocity;
  double effort;
  double Kp;
  double Kd;
};

constexpr const auto HW_IF_GAIN_KP = "gain_kp";
constexpr const auto HW_IF_GAIN_KD = "gain_kd";

std::set<std::string> quad_list_of_cmd_inter {
  "position",
  "velocity",
  "effort",
  "gain_kp",
  "gain_kd"
};

std::set<std::string> quad_list_of_state_inter {
  "position",
  "velocity",
  "effort",
  "gain_kp",
  "gain_kd"
};

enum control_mode_t {
  POSITION,
  VELOCITY,
  EFFORT,
  POS_VEL_EFF_GAINS,
  NO_VALID_MODE
};

class RRBotSystemQuadrupedHardware : public
  hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RRBotSystemQuadrupedHardware);

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo & info) override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  return_type start() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  return_type stop() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  return_type read() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  return_type write() override;

private:
  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  // Store the command for the simulated robot
  std::map<std::string,PosVelEffortGains> hw_commands_;
  std::map<std::string,PosVelEffortGains> hw_states_;
  std::map<std::string,control_mode_t> control_mode_;
};

}  // namespace ros2_control_demo_hardware

#endif  // ROS2_CONTROL_DEMO_HARDWARE__RRBOT_SYSTEM_QUADRUPED_HPP_
