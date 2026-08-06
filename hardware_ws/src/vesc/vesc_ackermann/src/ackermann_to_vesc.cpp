// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/ackermann_to_vesc.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using std::placeholders::_1;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions & options)
: Node("ackermann_to_vesc_node", options),
  actual_speed_(0.0),
  last_vesc_state_time_(0, 0, RCL_ROS_TIME),
  has_vesc_state_(false),
  braking_active_(false)
{
  // get conversion parameters
  speed_to_erpm_gain_ = declare_parameter<double>("speed_to_erpm_gain");
  speed_to_erpm_offset_ = declare_parameter<double>("speed_to_erpm_offset");
  steering_to_servo_gain_ =
    declare_parameter<double>("steering_angle_to_servo_gain");
  steering_to_servo_offset_ =
    declare_parameter<double>("steering_angle_to_servo_offset");
  brake_control_enabled_ = declare_parameter<bool>("brake_control_enabled");
  ackermann_brake_max_ = declare_parameter<double>("ackermann_brake_max");
  brake_speed_error_gain_ = declare_parameter<double>("brake_speed_error_gain");
  brake_decel_threshold_ = declare_parameter<double>("brake_decel_threshold");
  brake_entry_speed_error_ = declare_parameter<double>("brake_entry_speed_error");
  brake_overspeed_error_ = declare_parameter<double>("brake_overspeed_error");
  brake_release_speed_error_ = declare_parameter<double>("brake_release_speed_error");
  brake_min_vehicle_speed_ = declare_parameter<double>("brake_min_vehicle_speed");
  vesc_state_timeout_ = declare_parameter<double>("vesc_state_timeout");

  if (
    speed_to_erpm_gain_ == 0.0 || ackermann_brake_max_ <= 0.0 ||
    brake_speed_error_gain_ < 0.0 || brake_decel_threshold_ < 0.0 ||
    brake_entry_speed_error_ < 0.0 || brake_overspeed_error_ < 0.0 ||
    brake_release_speed_error_ < 0.0 ||
    brake_release_speed_error_ >= brake_entry_speed_error_ ||
    brake_min_vehicle_speed_ < 0.0 || vesc_state_timeout_ <= 0.0)
  {
    throw std::invalid_argument("Invalid VESC braking parameters");
  }

  // create publishers to VESC motor and servo commands
  erpm_pub_ = create_publisher<Float64>("commands/motor/speed", 10);
  current_pub_ = create_publisher<Float64>("commands/motor/current", 10);
  brake_pub_ = create_publisher<Float64>("commands/motor/brake", 10);
  servo_pub_ = create_publisher<Float64>("commands/servo/position", 10);

  // subscribe to ackermann topic
  ackermann_sub_ = create_subscription<AckermannDriveStamped>(
    "ackermann_cmd", 10, std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));
  vesc_state_sub_ = create_subscription<VescStateStamped>(
    "sensors/core", 10, std::bind(&AckermannToVesc::vescStateCallback, this, _1));
}

void AckermannToVesc::vescStateCallback(const VescStateStamped::SharedPtr state)
{
  actual_speed_ = (state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
  last_vesc_state_time_ = now();
  has_vesc_state_ = std::isfinite(actual_speed_);
}

void AckermannToVesc::ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd)
{
  // calc vesc electric RPM (speed)
  Float64 erpm_msg;
  erpm_msg.data = speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;

  // calc steering angle (servo)
  Float64 servo_msg;
  servo_msg.data = steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

  // publish
  if (rclcpp::ok()) {
    servo_pub_->publish(servo_msg);

    if (!brake_control_enabled_) {
      braking_active_ = false;
      erpm_pub_->publish(erpm_msg);
      return;
    }

    const double state_age = has_vesc_state_ ?
      (now() - last_vesc_state_time_).seconds() : vesc_state_timeout_ + 1.0;
    const bool state_is_fresh =
      has_vesc_state_ && state_age >= 0.0 && state_age <= vesc_state_timeout_;
    const bool command_is_finite =
      std::isfinite(cmd->drive.speed) && std::isfinite(cmd->drive.acceleration);

    Float64 current_msg;
    current_msg.data = 0.0;

    if (!state_is_fresh || !command_is_finite) {
      braking_active_ = false;
      current_pub_->publish(current_msg);
      return;
    }

    // Preserve the legacy speed path for reverse commands. The explicit brake
    // controller is intended for forward autonomous driving only.
    if (cmd->drive.speed < 0.0 || actual_speed_ < 0.0) {
      braking_active_ = false;
      erpm_pub_->publish(erpm_msg);
      return;
    }

    const double speed_error = actual_speed_ - cmd->drive.speed;
    const bool above_brake_speed = actual_speed_ >= brake_min_vehicle_speed_;
    const bool planned_deceleration =
      cmd->drive.acceleration <= -brake_decel_threshold_ &&
      speed_error >= brake_entry_speed_error_;
    const bool excessive_overspeed = speed_error >= brake_overspeed_error_;

    if (braking_active_) {
      braking_active_ =
        above_brake_speed && speed_error > brake_release_speed_error_;
    } else {
      braking_active_ =
        above_brake_speed && (planned_deceleration || excessive_overspeed);
    }

    if (braking_active_) {
      Float64 brake_msg;
      brake_msg.data = std::min(
        ackermann_brake_max_, std::max(0.0, brake_speed_error_gain_ * speed_error));
      brake_pub_->publish(brake_msg);
    } else if (speed_error >= brake_entry_speed_error_) {
      // Keep the VESC speed PID active for small positive errors. Enter COAST
      // only once the overspeed is large enough to warrant intervention but
      // neither explicit-brake entry condition is active.
      current_pub_->publish(current_msg);
    } else {
      erpm_pub_->publish(erpm_msg);
    }
  }
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::AckermannToVesc)
