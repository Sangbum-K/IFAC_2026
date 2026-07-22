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

#include "vesc_ackermann/vesc_to_odom.hpp"

#include <cmath>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

namespace vesc_ackermann
{

using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::Imu;
using std::placeholders::_1;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

namespace
{

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

VescToOdom::VescToOdom(const rclcpp::NodeOptions & options)
: Node("vesc_to_odom_node", options),
  odom_frame_("odom"),
  base_frame_("base_link"),
  imu_topic_("sensors/imu/raw"),
  use_servo_cmd_(true),
  use_imu_yaw_(false),
  publish_tf_(false),
  x_(0.0),
  y_(0.0),
  yaw_(0.0),
  last_speed_(0.0),
  has_imu_yaw_(false),
  last_imu_yaw_raw_(0.0),
  imu_yaw_unwrapped_(0.0),
  imu_angular_velocity_z_(0.0)
{
  // get ROS parameters
  odom_frame_ = declare_parameter("odom_frame", odom_frame_);
  base_frame_ = declare_parameter("base_frame", base_frame_);
  use_servo_cmd_ = declare_parameter("use_servo_cmd_to_calc_angular_velocity", use_servo_cmd_);
  use_imu_yaw_ = declare_parameter("use_imu_yaw", use_imu_yaw_);
  imu_topic_ = declare_parameter("imu_topic", imu_topic_);
  imu_yaw_timeout_ = declare_parameter("imu_yaw_timeout", 0.20);
  // The current hardware_ws VESC driver publishes gyro in deg/s. Keep the conversion
  // explicit as a parameter so a future standards-compliant rad/s driver can use 1.0.
  imu_angular_velocity_scale_ = declare_parameter(
    "imu_angular_velocity_scale", M_PI / 180.0);
  imu_yaw_variance_ = declare_parameter("imu_yaw_variance", 0.003);
  max_integration_dt_ = declare_parameter("max_integration_dt", 0.20);

  speed_to_erpm_gain_ = declare_parameter<double>("speed_to_erpm_gain");
  speed_to_erpm_offset_ = declare_parameter<double>("speed_to_erpm_offset");

  if (use_servo_cmd_) {
    steering_to_servo_gain_ =
      declare_parameter<double>("steering_angle_to_servo_gain");
    steering_to_servo_offset_ =
      declare_parameter<double>("steering_angle_to_servo_offset");
    wheelbase_ = declare_parameter<double>("wheelbase");
  }

  publish_tf_ = declare_parameter("publish_tf", publish_tf_);

  // create odom publisher
  odom_pub_ = create_publisher<Odometry>("odom", 10);

  // create tf broadcaster
  if (publish_tf_) {
    tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
  }

  // subscribe to VESC state and the selected heading source
  vesc_state_sub_ = create_subscription<VescStateStamped>(
    "sensors/core", 10, std::bind(&VescToOdom::vescStateCallback, this, _1));

  if (!use_imu_yaw_ && use_servo_cmd_) {
    servo_sub_ = create_subscription<Float64>(
      "sensors/servo_position_command", 10, std::bind(&VescToOdom::servoCmdCallback, this, _1));
  }

  if (use_imu_yaw_) {
    imu_sub_ = create_subscription<Imu>(
      imu_topic_, 20, std::bind(&VescToOdom::imuCallback, this, _1));
    RCLCPP_INFO(
      get_logger(), "Odometry heading source: relative IMU yaw (%s)", imu_topic_.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Odometry heading source: commanded steering model");
  }
}

void VescToOdom::vescStateCallback(const VescStateStamped::SharedPtr state)
{
  // check that we have a last servo command if we are depending on it for angular velocity
  if (!use_imu_yaw_ && use_servo_cmd_ && !last_servo_cmd_) {
    return;
  }

  // convert to engineering units
  double current_speed = (state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
  if (std::fabs(current_speed) < 0.05) {
    current_speed = 0.0;
  }
  double current_steering_angle(0.0), current_angular_velocity(0.0);
  if (!use_imu_yaw_ && use_servo_cmd_) {
    current_steering_angle =
      (last_servo_cmd_->data - steering_to_servo_offset_) / steering_to_servo_gain_;
    current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase_;
  }

  double imu_yaw = 0.0;
  if (use_imu_yaw_) {
    builtin_interfaces::msg::Time imu_stamp;
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      if (!has_imu_yaw_) {
        last_state_ = state;
        last_speed_ = current_speed;
        return;
      }
      imu_yaw = imu_yaw_unwrapped_;
      current_angular_velocity = imu_angular_velocity_z_ * imu_angular_velocity_scale_;
      imu_stamp = last_imu_stamp_;
    }

    const double imu_age =
      (rclcpp::Time(state->header.stamp) - rclcpp::Time(imu_stamp)).seconds();
    if (imu_age > imu_yaw_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IMU yaw is stale (age %.3f s); odometry propagation paused", imu_age);
      last_state_ = state;
      last_speed_ = current_speed;
      return;
    }
  }

  // calc elapsed time
  double dt_seconds = 0.0;
  if (last_state_) {
    dt_seconds =
      (rclcpp::Time(state->header.stamp) - rclcpp::Time(last_state_->header.stamp)).seconds();
    if (dt_seconds <= 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring non-positive VESC state dt: %.6f s", dt_seconds);
      return;
    }
  }

  const double previous_yaw = yaw_;
  double next_yaw = previous_yaw;
  if (use_imu_yaw_) {
    next_yaw = imu_yaw;
  } else if (use_servo_cmd_) {
    next_yaw += current_angular_velocity * dt_seconds;
  }

  if (dt_seconds > 0.0 && dt_seconds <= max_integration_dt_) {
    // Trapezoidal speed plus exact constant-curvature integration over the measured
    // yaw change. This removes the first-order Euler error in corners.
    const double travel = 0.5 * (last_speed_ + current_speed) * dt_seconds;
    const double delta_yaw = next_yaw - previous_yaw;
    if (std::fabs(delta_yaw) > 1.0e-9) {
      const double radius = travel / delta_yaw;
      x_ += radius * (std::sin(next_yaw) - std::sin(previous_yaw));
      y_ -= radius * (std::cos(next_yaw) - std::cos(previous_yaw));
    } else {
      x_ += travel * std::cos(previous_yaw);
      y_ += travel * std::sin(previous_yaw);
    }
  } else if (dt_seconds > max_integration_dt_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "VESC state gap %.3f s exceeds max_integration_dt; translation skipped", dt_seconds);
  }
  yaw_ = next_yaw;

  // save state for next time
  last_state_ = state;
  last_speed_ = current_speed;

  // publish odometry message
  Odometry odom;
  odom.header.frame_id = odom_frame_;
  odom.header.stamp = state->header.stamp;
  odom.child_frame_id = base_frame_;

  // Position
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = sin(yaw_ / 2.0);
  odom.pose.pose.orientation.w = cos(yaw_ / 2.0);

  // Position uncertainty
  /** @todo Think about position uncertainty, perhaps get from parameters? */
  odom.pose.covariance[0] = 0.2;   ///< x
  odom.pose.covariance[7] = 0.2;   ///< y
  odom.pose.covariance[35] = use_imu_yaw_ ? imu_yaw_variance_ : 0.4;  ///< yaw

  // Velocity ("in the coordinate frame given by the child_frame_id")
  odom.twist.twist.linear.x = current_speed;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = current_angular_velocity;
  odom.twist.covariance[35] = use_imu_yaw_ ? 0.01 : 0.4;

  // Velocity uncertainty
  /** @todo Think about velocity uncertainty */

  if (publish_tf_) {
    TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    // Keep TF and Odometry on the same sensor timestamp so slam_toolbox can look up
    // the pose that actually corresponds to each laser scan.
    tf.header.stamp = state->header.stamp;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;

    if (rclcpp::ok()) {
      tf_pub_->sendTransform(tf);
    }
  }

  if (rclcpp::ok()) {
    odom_pub_->publish(odom);
  }
}

void VescToOdom::servoCmdCallback(const Float64::SharedPtr servo)
{
  last_servo_cmd_ = servo;
}

void VescToOdom::imuCallback(const Imu::SharedPtr imu)
{
  const auto & q = imu->orientation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(norm) || norm < 1.0e-9) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Ignoring invalid IMU orientation quaternion");
    return;
  }

  const double x = q.x / norm;
  const double y = q.y / norm;
  const double z = q.z / norm;
  const double w = q.w / norm;
  const double raw_yaw = std::atan2(
    2.0 * (w * z + x * y),
    1.0 - 2.0 * (y * y + z * z));

  std::lock_guard<std::mutex> lock(imu_mutex_);
  if (!has_imu_yaw_) {
    // The odom frame starts at yaw zero regardless of the AHRS magnetic/global datum.
    last_imu_yaw_raw_ = raw_yaw;
    imu_yaw_unwrapped_ = 0.0;
    has_imu_yaw_ = true;
  } else {
    imu_yaw_unwrapped_ += normalizeAngle(raw_yaw - last_imu_yaw_raw_);
    last_imu_yaw_raw_ = raw_yaw;
  }
  imu_angular_velocity_z_ = imu->angular_velocity.z;
  last_imu_stamp_ = imu->header.stamp;
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdom)
