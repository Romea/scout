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


// ros2
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

// romea
#include "romea_mobile_base_utils/ros2_control/info/hardware_info_common.hpp"

// local
#include "scout_hardware/scout_hardware.hpp"


namespace
{
size_t FRONT_LEFT_SPINNING_MOTOR_ID_ = 4;
size_t FRONT_RIGHT_SPINNING_MOTOR_ID_ = 1;
size_t REAR_LEFT_SPINNING_MOTOR_ID_ = 3;
size_t REAR_RIGHT_SPINNING_MOTOR_ID_ = 2;
}


namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
ScoutHardwareBase::ScoutHardwareBase(ProtocolVersion protocol, bool is_mini_model)
: HardwareSystemInterface<HardwareInterface4WD>(),
  robot_(protocol, is_mini_model),
  wheel_radius_(0),
  track_(0),
  front_left_wheel_angular_speed_measure_(0),
  front_right_wheel_angular_speed_measure_(0),
  rear_left_wheel_angular_speed_measure_(0),
  rear_right_wheel_angular_speed_measure_(0),
  front_left_wheel_angular_speed_command_(0),
  front_right_wheel_angular_speed_command_(0),
  rear_left_wheel_angular_speed_command_(0),
  rear_right_wheel_angular_speed_command_(0)
{
}

//-----------------------------------------------------------------------------
ScoutHardwareBase::~ScoutHardwareBase()
{
  // force deactive when interface has not been deactivated by controller manager but by ctrl-c
  if (lifecycle_state_.id() == 3) {
    on_deactivate(lifecycle_state_);
  }
}


//-----------------------------------------------------------------------------
hardware_interface::return_type ScoutHardwareBase::connect_()
{
  // RCLCPP_ERROR(rclcpp::get_logger("ScoutHardware"), "Init communication with robot");
  // To be implememented

  if (robot_.Connect("can0")) {
    robot_.EnableCommandedMode();
    send_null_command_();
    return hardware_interface::return_type::OK;
  } else {
    return hardware_interface::return_type::ERROR;
  }
}

//-----------------------------------------------------------------------------
hardware_interface::return_type ScoutHardwareBase::disconnect_()
{
  // RCLCPP_ERROR(rclcpp::get_logger("ScoutHardware"), "Close communication with robot");
  // To be implememented

  send_null_command_();
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type ScoutHardwareBase::load_info_(
  const hardware_interface::HardwareInfo & hardware_info)
{
  try {
    // Get some info from ros2_control item of robot urdf file
    // wheelbase_ = get_parameter<double>(hardware_info, "wheelbase");
    // auto front_track_ = get_parameter<double>(hardware_info, "front_track");
    // auto rear_track_ = get_parameter<double>(hardware_info, "rear_track");
    auto front_track_ = get_front_track(hardware_info);
    auto rear_track_ = get_rear_track(hardware_info);
    assert(front_track_ == rear_track_);
    track_ = front_track_;

    // auto front_wheel_radius_ = get_parameter<float>(hardware_info, "front_wheel_radius");
    // auto rear_wheel_radius_ = get_parameter<float>(hardware_info, "rear_wheel_radius");
    auto front_wheel_radius_ = get_front_wheel_radius(hardware_info);
    auto rear_wheel_radius_ = get_rear_wheel_radius(hardware_info);
    assert(front_wheel_radius_ == rear_wheel_radius_);
    wheel_radius_ = front_wheel_radius_;

    return hardware_interface::return_type::OK;
  } catch (std::runtime_error & e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("ScoutHardware"), e.what());
    return hardware_interface::return_type::ERROR;
  }
}

//-----------------------------------------------------------------------------
void ScoutHardwareBase::send_null_command_()
{
  robot_.SetMotionCommand(0, 0);
}


//-----------------------------------------------------------------------------
#if ROS_DISTRO == ROS_GALACTIC
hardware_interface::return_type ScoutHardwareBase::read()
#else
hardware_interface::return_type ScoutHardwareBase::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
#endif
{
  try {
    set_hardware_state_();
    return hardware_interface::return_type::OK;
  } catch (std::runtime_error & e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("ScoutHardware"), e.what());
    return hardware_interface::return_type::ERROR;
  }

  try {
    //TODO (JEAN) to be finished
    auto actuator = robot_.GetActuatorState();
    // front_left_wheel_angular_speed_measure_ =
    //   actuator.actuator_hs_state[FRONT_LEFT_SPINNING_MOTOR_ID_].rpm * ? / wheel_radius_ ;
    // front_right_wheel_angular_speed_measure_ =
    //   actuator.actuator_hs_state[FRONT_RIGHT_SPINNING_MOTOR_ID_].rpm * ? / wheel_radius_ ;
    // rear_left_wheel_angular_speed_measure_ =
    //   actuator.actuator_hs_state[REAR_LEFT_SPINNING_MOTOR_ID_].rpm * ? / wheel_radius_;
    // rear_right_wheel_angular_speed_measure_ =
    //   actuator.actuator_hs_state[REAR_LEFT_SPINNING_MOTOR_ID_].rpm * ? / wheel_radius_;
    set_hardware_state_();
    return hardware_interface::return_type::OK;
  } catch (std::runtime_error & e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("HunterHardware"), e.what());
    return hardware_interface::return_type::ERROR;
  }
}


//-----------------------------------------------------------------------------
#if ROS_DISTRO == ROS_GALACTIC
hardware_interface::return_type ScoutHardwareBase::write()
# else
hardware_interface::return_type ScoutHardwareBase::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
#endif
{
  // RCLCPP_ERROR(rclcpp::get_logger("ScoutHardware"), "Send command to robot");
  get_hardware_command_();

  auto left_linear_speed_command = 0.5 *
    (front_left_wheel_angular_speed_command_ * wheel_radius_ +
    rear_left_wheel_angular_speed_command_ * wheel_radius_);
  auto right_linear_speed_command = 0.5 *
    (front_right_wheel_angular_speed_command_ * wheel_radius_ +
    rear_right_wheel_angular_speed_command_ * wheel_radius_);

  auto linear_speed_command = 0.5 * (left_linear_speed_command + right_linear_speed_command);
  auto angular_speed_command = (right_linear_speed_command - left_linear_speed_command) / track_;
  robot_.SetMotionCommand(linear_speed_command, angular_speed_command);

  return hardware_interface::return_type::OK;
}


//-----------------------------------------------------------------------------
void ScoutHardwareBase::set_hardware_state_()
{
  core::HardwareState4WD state;
  state.frontLeftWheelSpinningMotion.velocity = front_left_wheel_angular_speed_measure_;
  state.frontRightWheelSpinningMotion.velocity = front_right_wheel_angular_speed_measure_;
  state.rearLeftWheelSpinningMotion.velocity = rear_left_wheel_angular_speed_measure_;
  state.rearRightWheelSpinningMotion.velocity = rear_right_wheel_angular_speed_measure_;
  this->hardware_interface_->set_feedback(state);
}

//-----------------------------------------------------------------------------
void ScoutHardwareBase::get_hardware_command_()
{
  core::HardwareCommand4WD command = hardware_interface_->get_hardware_command();
  front_left_wheel_angular_speed_command_ = command.frontLeftWheelSpinningSetPoint;
  front_right_wheel_angular_speed_command_ = command.frontRightWheelSpinningSetPoint;
  rear_left_wheel_angular_speed_command_ = command.rearLeftWheelSpinningSetPoint;
  rear_right_wheel_angular_speed_command_ = command.rearRightWheelSpinningSetPoint;
}

//-----------------------------------------------------------------------------
ScoutMiniHardware::ScoutMiniHardware()
: ScoutHardwareBase(ProtocolVersion::AGX_V2, true)
{
}

//-----------------------------------------------------------------------------
ScoutV2Hardware::ScoutV2Hardware()
: ScoutHardwareBase(ProtocolVersion::AGX_V2, false)
{
}

}  // namespace ros2
}  // namespace romea


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(romea::ros2::ScoutMiniHardware, hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(romea::ros2::ScoutV2Hardware, hardware_interface::SystemInterface)
