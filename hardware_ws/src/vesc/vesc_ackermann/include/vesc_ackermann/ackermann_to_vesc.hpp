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

#ifndef VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_
#define VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_

#include <cstdint>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

class AckermannToVesc : public rclcpp::Node
{
public:
  explicit AckermannToVesc(const rclcpp::NodeOptions & options);

private:
  // ROS parameters
  // conversion gain and offset
  double speed_to_erpm_gain_, speed_to_erpm_offset_;
  double steering_to_servo_gain_, steering_to_servo_offset_;
  bool brake_control_enabled_;
  double ackermann_brake_max_, brake_speed_error_gain_;
  double brake_decel_threshold_, brake_entry_speed_error_;
  double brake_overspeed_error_, brake_release_speed_error_;
  double brake_min_vehicle_speed_, vesc_state_timeout_;
  bool launch_ramp_enabled_;
  double launch_arm_speed_mps_, launch_exit_speed_mps_;
  double launch_accel_limit_mps2_, drive_ramp_erpm_per_sec_;
  double fault_clear_hold_sec_;

  /** @todo consider also providing an interpolated look-up table conversion */

  // ROS services
  rclcpp::Publisher<Float64>::SharedPtr erpm_pub_;
  rclcpp::Publisher<Float64>::SharedPtr current_pub_;
  rclcpp::Publisher<Float64>::SharedPtr brake_pub_;
  rclcpp::Publisher<Float64>::SharedPtr servo_pub_;
  rclcpp::Subscription<AckermannDriveStamped>::SharedPtr ackermann_sub_;
  rclcpp::Subscription<VescStateStamped>::SharedPtr vesc_state_sub_;

  // Latest VESC feedback and longitudinal mode state
  double actual_speed_;
  rclcpp::Time last_vesc_state_time_;
  rclcpp::Time last_fault_time_;
  rclcpp::Time last_drive_command_time_;
  bool has_vesc_state_;
  bool braking_active_;
  bool fault_seen_;
  bool launch_armed_;
  bool launch_ramp_active_;
  bool was_above_launch_arm_speed_;
  double limited_drive_speed_;
  int32_t fault_code_;

  // ROS callbacks
  void ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd);
  void vescStateCallback(const VescStateStamped::SharedPtr state);
};

}  // namespace vesc_ackermann

#endif  // VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_
