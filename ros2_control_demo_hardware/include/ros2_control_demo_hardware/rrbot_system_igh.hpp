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

#ifndef ROS2_CONTROL_DEMO_HARDWARE__RRBOT_SYSTEM_IGH_HPP_
#define ROS2_CONTROL_DEMO_HARDWARE__RRBOT_SYSTEM_IGH_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"
#include "ros2_control_demo_hardware/visibility_control.h"

#include <shmringbuf.h>

/* Shm related data types
 * s_buf, handle_s: data sent from RT domain
 * r_buf, handle_r: data sent to RT domain
 */
#define MSG_LEN 500
#define JOINT_NUM 2

struct JointCommand
{
  double positions[JOINT_NUM] = {0};
  double velocities[JOINT_NUM] = {0};
  double accelerations[JOINT_NUM] = {0};
  double efforts[JOINT_NUM] = {0};
};

struct JointState
{
  double positions[JOINT_NUM] = {0};
  double velocities[JOINT_NUM] = {0};
  double accelerations[JOINT_NUM] = {0};
  double efforts[JOINT_NUM] = {0};
};

namespace ros2_control_demo_hardware
{
class RRBotSystemIghHardware
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RRBotSystemIghHardware);

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  hardware_interface::return_type start() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  hardware_interface::return_type stop() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  hardware_interface::return_type read() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  hardware_interface::return_type write() override;

  bool shm_read();
  bool shm_write();

private:
  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  char s_buf_[MSG_LEN];
  char r_buf_[MSG_LEN];
  shm_handle_t handle_s_, handle_r_;
  JointCommand joint_cmd_;
  JointState joint_state_;
};

}  // namespace ros2_control_demo_hardware

#endif  // ROS2_CONTROL_DEMO_HARDWARE__RRBOT_SYSTEM_IGH_HPP_
