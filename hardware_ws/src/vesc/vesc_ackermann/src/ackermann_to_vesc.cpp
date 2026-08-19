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
  last_ackermann_cmd_time_(0, 0, RCL_ROS_TIME),
  has_vesc_state_(false),
  has_last_ackermann_cmd_time_(false),
  braking_active_(false),
  accel_speed_lead_(0.0)
{
  // get conversion parameters
  speed_to_erpm_gain_ = declare_parameter<double>("speed_to_erpm_gain");
  speed_to_erpm_offset_ = declare_parameter<double>("speed_to_erpm_offset");
  steering_to_servo_gain_ =
    declare_parameter<double>("steering_angle_to_servo_gain");
  steering_to_servo_offset_ =
    declare_parameter<double>("steering_angle_to_servo_offset");
  accel_speed_lead_enabled_ = declare_parameter<bool>("accel_speed_lead_enabled");
  accel_speed_lead_time_s_ = declare_parameter<double>("accel_speed_lead_time_s");
  accel_speed_lead_max_mps_ = declare_parameter<double>("accel_speed_lead_max_mps");
  accel_speed_lead_target_max_mps_ =
    declare_parameter<double>("accel_speed_lead_target_max_mps");
  accel_speed_lead_rise_rate_mps2_ =
    declare_parameter<double>("accel_speed_lead_rise_rate_mps2");
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
    speed_to_erpm_gain_ == 0.0 || accel_speed_lead_time_s_ < 0.0 ||
    accel_speed_lead_max_mps_ < 0.0 || accel_speed_lead_target_max_mps_ <= 0.0 ||
    accel_speed_lead_rise_rate_mps2_ <= 0.0 ||
    ackermann_brake_max_ <= 0.0 ||
    brake_speed_error_gain_ < 0.0 || brake_decel_threshold_ < 0.0 ||
    brake_entry_speed_error_ < 0.0 || brake_overspeed_error_ < 0.0 ||
    brake_release_speed_error_ < 0.0 ||
    brake_release_speed_error_ >= brake_entry_speed_error_ ||
    brake_min_vehicle_speed_ < 0.0 || vesc_state_timeout_ <= 0.0)
  {
    throw std::invalid_argument("Invalid VESC longitudinal control parameters");
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
  // calc steering angle (servo)
  Float64 servo_msg;
  servo_msg.data = steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

  const rclcpp::Time command_time = now();
  const double state_age = has_vesc_state_ ?
    (command_time - last_vesc_state_time_).seconds() : vesc_state_timeout_ + 1.0;
  const bool state_is_fresh =
    has_vesc_state_ && state_age >= 0.0 && state_age <= vesc_state_timeout_;
  const bool command_is_finite =
    std::isfinite(cmd->drive.speed) && std::isfinite(cmd->drive.acceleration);

  double command_dt = 0.0;
  if (has_last_ackermann_cmd_time_) {
    command_dt = (command_time - last_ackermann_cmd_time_).seconds();
  }
  last_ackermann_cmd_time_ = command_time;
  has_last_ackermann_cmd_time_ = true;

  const bool command_dt_is_valid =
    std::isfinite(command_dt) && command_dt > 0.0 && command_dt <= vesc_state_timeout_;
  if (!command_dt_is_valid) {
    accel_speed_lead_ = 0.0;
  }

  // 계획된 양의 가속도를 이용해 DRIVE 목표 속도를 선행시킨다. 실제 속도가
  // 원래 명령 속도에 가까워질수록 보정값을 줄이고, 설정된 최고속도를 넘지 않는다.
  double drive_speed = cmd->drive.speed;
  const bool speed_lead_is_allowed =
    accel_speed_lead_enabled_ && state_is_fresh && command_is_finite &&
    command_dt_is_valid && !braking_active_ &&
    cmd->drive.speed >= 0.0 && actual_speed_ >= 0.0 &&
    cmd->drive.acceleration > 0.0 && actual_speed_ < cmd->drive.speed;
  if (speed_lead_is_allowed) {
    const double requested_lead = accel_speed_lead_time_s_ * cmd->drive.acceleration;
    const double speed_deficit = cmd->drive.speed - actual_speed_;
    const double target_headroom =
      std::max(0.0, accel_speed_lead_target_max_mps_ - cmd->drive.speed);
    const double target_speed_lead = std::min(
      accel_speed_lead_max_mps_,
      std::min(requested_lead, std::min(speed_deficit, target_headroom)));
    const double max_lead_rise = accel_speed_lead_rise_rate_mps2_ * command_dt;
    accel_speed_lead_ = std::min(target_speed_lead, accel_speed_lead_ + max_lead_rise);
  } else {
    // 가속 종료, BRAKE, 과속, 후진 또는 센서 이상 시에는 하강 제한을 적용하지
    // 않고 보정값을 즉시 제거해 원래 목표 속도로 돌아간다.
    accel_speed_lead_ = 0.0;
  }
  drive_speed += accel_speed_lead_;

  // calc vesc electric RPM (speed)
  Float64 erpm_msg;
  erpm_msg.data = speed_to_erpm_gain_ * drive_speed + speed_to_erpm_offset_;

  // publish
  if (rclcpp::ok()) {
    servo_pub_->publish(servo_msg);

    if (!brake_control_enabled_) {
      braking_active_ = false;
      erpm_pub_->publish(erpm_msg);
      return;
    }

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
    // 2026-08-19: 계획(cmd.acceleration) 기반 진입 + 피드포워드 전류를 넣었다가
    // 도로 뺐다. accel_speed_lead(계획 가속도 리드)를 실차에서 체감이 안
    // 좋다고 이미 껐는데, 이 브레이크 변경도 같은 계열(오차를 기다리지 않고
    // 계획값을 바로 액추에이터에 먹인다)이라 실차 검증 없이 그대로 남겨두는
    // 위험을 사용자가 원치 않음 — 리액티브 전용(사후 speed_error 기반)으로
    // 되돌린다. deployment_profiles.VescLongitudinalPlant.step() 이 폐루프
    // 시뮬에서 쓰는 모델도 원래 이 리액티브 로직 그대로였다(피드포워드로
    // 갱신된 적이 없다) — 즉 이 되돌림은 랩타임 시뮬 결과에 영향이 없다.
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
