// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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


#ifndef SCOUT_HARDWARE__SCOUT_HARDWARE_HPP_
#define SCOUT_HARDWARE__SCOUT_HARDWARE_HPP_

// std
#include <atomic>

// romea
#include "romea_common_utils/ros_versions.hpp"
#include "romea_mobile_base_hardware/hardware_system_interface.hpp"

// agilex
#include "ugv_sdk/mobile_robot/scout_robot.hpp"

namespace romea
{
namespace ros2
{

class ScoutHardwareBase : public HardwareSystemInterface<HardwareInterface4WD>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ScoutHardwareBase);

  ScoutHardwareBase(ProtocolVersion protocol, bool is_mini_model);

  virtual ~ScoutHardwareBase();

#if ROS_DISTRO == ROS_GALACTIC
  hardware_interface::return_type read()override;

  hardware_interface::return_type write()override;
#else
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)override;
#endif

private:
  hardware_interface::return_type connect_() override;

  hardware_interface::return_type disconnect_() override;

  hardware_interface::return_type load_info_(
    const hardware_interface::HardwareInfo & hardware_info) override;

  void send_null_command_();

  void get_hardware_command_();

  void set_hardware_state_();

private:
  westonrobot::ScoutRobot robot_;

  float wheel_radius_;
  float track_;

  std::atomic<float> front_left_wheel_angular_speed_measure_;
  std::atomic<float> front_right_wheel_angular_speed_measure_;
  std::atomic<float> rear_left_wheel_angular_speed_measure_;
  std::atomic<float> rear_right_wheel_angular_speed_measure_;

  float front_left_wheel_angular_speed_command_;
  float front_right_wheel_angular_speed_command_;
  float rear_left_wheel_angular_speed_command_;
  float rear_right_wheel_angular_speed_command_;
};

class ScoutMiniHardware : public ScoutHardwareBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ScoutMiniHardware);

  ScoutMiniHardware();

  virtual ~ScoutMiniHardware() = default;
};

class ScoutV2Hardware : public ScoutHardwareBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ScoutV2Hardware);

  ScoutV2Hardware();

  virtual ~ScoutV2Hardware() = default;
};

}  // namespace ros2
}  // namespace romea

#endif  // SCOUT_HARDWARE__SCOUT_HARDWARE_HPP_
