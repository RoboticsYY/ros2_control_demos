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

#include "ros2_control_demo_hardware/rrbot_system_igh.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_hardware
{
hardware_interface::return_type RRBotSystemIghHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }

  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemIghHardware"),
        "Joint '%s' has %d command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemIghHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemIghHardware"),
        "Joint '%s' has %d state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemIghHardware"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }
  }

  handle_s_ = shm_blkbuf_open((char*)"rtsend");
  handle_r_ = shm_blkbuf_open((char*)"rtread");

  status_ = hardware_interface::status::CONFIGURED;
  return shm_read() ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

std::vector<hardware_interface::StateInterface>
RRBotSystemIghHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RRBotSystemIghHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type RRBotSystemIghHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemIghHardware"), "Starting ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemIghHardware"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }

  // set some default values when starting the first time
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = joint_state_.positions[i];
    hw_commands_[i] = hw_states_[i];
  }

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemIghHardware"), "System Successfully started!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotSystemIghHardware::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemIghHardware"), "Stopping ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemIghHardware"), "%.1f seconds left...",
      hw_stop_sec_ - i);
  }

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemIghHardware"), "System successfully stopped!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotSystemIghHardware::read()
{
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemIghHardware"), "Reading...");

  if (!shm_read())
    return hardware_interface::return_type::ERROR;

  for (uint i = 0; i < hw_states_.size(); i++)
  {
    // Simulate RRBot's movement
    hw_states_[i] = joint_state_.positions[i];
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemIghHardware"), "Got state %.5f for joint %d!",
      hw_states_[i], i);
  }
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemIghHardware"), "Joints successfully read!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotSystemIghHardware::write()
{
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemIghHardware"), "Writing...");

  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    joint_cmd_.positions[i] = hw_commands_[i];

    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemIghHardware"), "Got command %.5f for joint %d!",
      hw_commands_[i], i);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemIghHardware"), "Joints successfully written!");

  return shm_write() ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

bool RRBotSystemIghHardware::shm_read()
{
  if (shm_blkbuf_empty(handle_s_))
    return false;

  if (!shm_blkbuf_read(handle_s_, s_buf_, sizeof(s_buf_)))
    return false;

  memcpy(&joint_state_, s_buf_, sizeof(joint_state_));

  return true;
}

bool RRBotSystemIghHardware::shm_write()
{
  if (shm_blkbuf_full(handle_r_))
    return false;

  memcpy(r_buf_, &joint_cmd_, sizeof(joint_cmd_));

  return shm_blkbuf_write(handle_r_, r_buf_, sizeof(r_buf_));
}

}  // namespace ros2_control_demo_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_hardware::RRBotSystemIghHardware, hardware_interface::SystemInterface)