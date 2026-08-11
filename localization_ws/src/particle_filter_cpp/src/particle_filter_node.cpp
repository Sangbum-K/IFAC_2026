// Copyright 2026 IFAC F1TENTH Team
// SPDX-License-Identifier: MIT

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "particle_filter_cpp/mcl_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "includes/RangeLib.h"

namespace particle_filter_cpp
{
namespace
{
using namespace std::chrono_literals;

int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

builtin_interfaces::msg::Time to_stamp(const int64_t nanoseconds)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  return stamp;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(const double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion result;
  result.x = quaternion.x();
  result.y = quaternion.y();
  result.z = quaternion.z();
  result.w = quaternion.w();
  return result;
}

bool finite_pose_message(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

double mean_or_zero(const std::deque<double> & values)
{
  if (values.empty()) {
    return 0.0;
  }
  double total = 0.0;
  for (const double value : values) {
    total += value;
  }
  return total / static_cast<double>(values.size());
}

void append_bounded(std::deque<double> & values, const double value, const std::size_t capacity)
{
  values.push_back(value);
  while (values.size() > capacity) {
    values.pop_front();
  }
}
}  // namespace

class ParticleFilterNode : public rclcpp::Node
{
public:
  ParticleFilterNode()
  : Node("particle_filter"),
    odom_buffer_(declare_parameter<int>("odom_buffer_size", 150)),
    imu_buffer_(declare_parameter<int>("imu_buffer_size", 150)),
    quality_tracker_(
      declare_parameter<double>("quality_ok_inlier_fraction", 0.70),
      declare_parameter<double>("quality_lost_inlier_fraction", 0.40),
      declare_parameter<int>("quality_bad_updates_to_lost", 3),
      declare_parameter<int>("quality_good_updates_to_recover", 5))
  {
    read_parameters();
    validate_supported_configuration();
    slip_detector_ = std::make_unique<SlipDetector>(
      slip_translation_innovation_m_, slip_yaw_innovation_rad_,
      slip_enter_score_, slip_recover_score_, slip_enter_updates_,
      slip_recover_updates_, slip_immediate_score_);

    const rclcpp::QoS status_qos = rclcpp::QoS(1).reliable().transient_local();
    ready_pub_ = create_publisher<std_msgs::msg::Bool>(ready_topic_, status_qos);
    quality_pub_ = create_publisher<std_msgs::msg::String>(quality_topic_, status_qos);
    initial_received_pub_ = create_publisher<std_msgs::msg::Bool>(
      initial_received_topic_, status_qos);
    initial_accepted_pub_ = create_publisher<std_msgs::msg::Bool>(
      initial_accepted_topic_, status_qos);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/pf/viz/inferred_pose", 1);
    particle_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/pf/viz/particles", 1);
    fake_scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "/pf/viz/fake_scan", 1);
    polygon_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
      "/pf/viz/poly1", 1);
    if (publish_odom_) {
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/pf/pose/odom", 1);
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::QoS(1),
      std::bind(&ParticleFilterNode::scan_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      std::bind(&ParticleFilterNode::odom_callback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::QoS(50),
      std::bind(&ParticleFilterNode::imu_callback, this, std::placeholders::_1));
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(1),
      std::bind(&ParticleFilterNode::initial_pose_callback, this, std::placeholders::_1));
    initial_pose_latched_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", status_qos,
      std::bind(&ParticleFilterNode::initial_pose_callback, this, std::placeholders::_1));
    clicked_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", rclcpp::QoS(1),
      [this](geometry_msgs::msg::PointStamped::SharedPtr) {initialize_global();});

    map_client_ = create_client<nav_msgs::srv::GetMap>("/map_server/map");
    map_retry_timer_ = create_wall_timer(250ms, std::bind(&ParticleFilterNode::request_map, this));
    initial_pose_timer_ = create_wall_timer(
      100ms, std::bind(&ParticleFilterNode::retry_initial_pose, this));
    publish_status();
    worker_ = std::thread(&ParticleFilterNode::worker_loop, this);
  }

  ~ParticleFilterNode() override
  {
    stop_worker_.store(true);
    scan_condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  struct ScanSample
  {
    int64_t stamp_ns{0};
    std::chrono::steady_clock::time_point received;
    std::vector<float> ranges;
    std::vector<float> angles;
  };

  void read_parameters()
  {
    angle_step_ = declare_parameter<int>("angle_step", 15);
    max_particles_ = declare_parameter<int>("max_particles", 2000);
    max_viz_particles_ = declare_parameter<int>("max_viz_particles", 60);
    squash_factor_ = declare_parameter<double>("squash_factor", 2.2);
    // localize.yaml intentionally stores this legacy parameter as `10`
    // (INTEGER). Python accepted it dynamically and converted with float();
    // rclcpp is strict, so preserve the wire type and convert internally.
    max_range_m_ = static_cast<double>(declare_parameter<int>("max_range", 10));
    theta_discretization_ = declare_parameter<int>("theta_discretization", 112);
    range_method_name_ = declare_parameter<std::string>("range_method", "glt");
    rangelib_variant_ = declare_parameter<int>("rangelib_variant", 0);
    profile_label_ = declare_parameter<std::string>("pf_profile_label", "unprofiled");
    random_seed_ = declare_parameter<int>("random_seed", -1);
    publish_odom_ = declare_parameter<int>("publish_odom", 1) != 0;
    visualize_ = declare_parameter<int>("viz", 1) != 0;
    z_short_ = declare_parameter<double>("z_short", 0.01);
    z_max_ = declare_parameter<double>("z_max", 0.07);
    z_rand_ = declare_parameter<double>("z_rand", 0.12);
    z_hit_ = declare_parameter<double>("z_hit", 0.75);
    sigma_hit_ = declare_parameter<double>("sigma_hit", 4.0);
    motion_dispersion_x_ = declare_parameter<double>("motion_dispersion_x", 0.05);
    motion_dispersion_y_ = declare_parameter<double>("motion_dispersion_y", 0.025);
    motion_dispersion_theta_ = declare_parameter<double>("motion_dispersion_theta", 0.08);
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    odom_topic_ = declare_parameter<std::string>("odometry_topic", "/odom");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/sensors/imu/raw");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    laser_frame_ = declare_parameter<std::string>("laser_frame", "laser");
    pose_jump_guard_enabled_ = declare_parameter<bool>("pose_jump_guard_enabled", true);
    max_pose_jump_translation_m_ = declare_parameter<double>(
      "max_pose_jump_translation_m", 0.30);
    max_pose_jump_yaw_rad_ = declare_parameter<double>("max_pose_jump_yaw_rad", 0.35);
    low_speed_threshold_mps_ = declare_parameter<double>("low_speed_threshold_mps", 0.2);
    low_speed_xy_deadband_m_ = declare_parameter<double>(
      "low_speed_odom_xy_deadband_m", 0.01);
    low_speed_yaw_deadband_rad_ = declare_parameter<double>(
      "low_speed_odom_yaw_deadband_rad", 0.02);
    low_speed_odom_scale_ = declare_parameter<double>("low_speed_odom_scale", 0.25);
    low_speed_jump_translation_m_ = declare_parameter<double>(
      "low_speed_jump_guard_translation_m", 0.50);
    low_speed_jump_yaw_rad_ = declare_parameter<double>(
      "low_speed_jump_guard_yaw_rad", 0.60);
    max_consecutive_jump_rejects_ = declare_parameter<int>(
      "max_consecutive_jump_rejects", 10);
    odom_sync_wait_sec_ = declare_parameter<double>("odom_sync_wait_sec", 0.04);
    max_odom_interp_gap_sec_ = declare_parameter<double>(
      "max_odom_interp_gap_sec", 0.06);
    max_pending_scan_age_sec_ = declare_parameter<double>(
      "max_pending_scan_age_sec", 0.10);
    max_update_rate_hz_ = declare_parameter<double>("max_update_rate_hz", 15.0);
    diagnostics_period_sec_ = declare_parameter<double>(
      "sync_diagnostics_period_sec", 1.0);
    max_localization_update_age_sec_ = declare_parameter<double>(
      "max_localization_update_age_sec", 0.50);
    motion_noise_min_scale_ = declare_parameter<double>("motion_noise_min_scale", 0.08);
    motion_noise_reference_translation_m_ = declare_parameter<double>(
      "motion_noise_reference_translation_m", 0.25);
    motion_noise_reference_yaw_rad_ = declare_parameter<double>(
      "motion_noise_reference_yaw_rad", 0.15);
    resample_ess_ratio_ = declare_parameter<double>("resample_ess_ratio", 0.50);
    beam_skip_enabled_ = declare_parameter<bool>("beam_skip_enabled", true);
    beam_skip_distance_m_ = declare_parameter<double>("beam_skip_distance_m", 0.20);
    beam_skip_min_particle_fraction_ = declare_parameter<double>(
      "beam_skip_min_particle_fraction", 0.30);
    beam_skip_max_fraction_ = declare_parameter<double>("beam_skip_max_fraction", 0.50);
    beam_skip_short_only_ = declare_parameter<bool>("beam_skip_short_only", true);
    quality_gate_enabled_ = declare_parameter<bool>("quality_gate_enabled", false);
    ready_topic_ = declare_parameter<std::string>(
      "localization_ready_topic", "/pf/localization_ready");
    quality_topic_ = declare_parameter<std::string>(
      "localization_quality_topic", "/pf/localization_quality");
    initial_received_topic_ = declare_parameter<std::string>(
      "initialpose_received_topic", "/pf/initialpose_received");
    initial_accepted_topic_ = declare_parameter<std::string>(
      "initialpose_accepted_topic", "/pf/initialpose_accepted");
    freeze_stationary_filter_ = declare_parameter<bool>("freeze_stationary_filter", true);
    deskew_enabled_ = declare_parameter<bool>("deskew_enabled", true);
    ground_filter_enabled_ = declare_parameter<bool>("ground_filter_enabled", true);
    scan_match_enabled_ = declare_parameter<bool>("scan_match_enabled", true);
    imu_angular_velocity_scale_ = declare_parameter<double>(
      "imu_angular_velocity_scale", 0.017453292519943295);
    imu_linear_acceleration_scale_ = declare_parameter<double>(
      "imu_linear_acceleration_scale", 9.80665);
    max_imu_interp_gap_sec_ = declare_parameter<double>("max_imu_interp_gap_sec", 0.06);
    stationary_imu_yaw_rate_rad_s_ = declare_parameter<double>(
      "stationary_imu_yaw_rate_rad_s", 0.06);
    stationary_imu_accel_mps2_ = declare_parameter<double>(
      "stationary_imu_horizontal_accel_mps2", 1.60);
    stationary_scan_translation_m_ = declare_parameter<double>(
      "stationary_scan_translation_m", 0.04);
    stationary_scan_yaw_rad_ = declare_parameter<double>("stationary_scan_yaw_rad", 0.04);
    scan_match_max_correspondence_m_ = declare_parameter<double>(
      "scan_match_max_correspondence_m", 0.40);
    scan_match_trim_fraction_ = declare_parameter<double>(
      "scan_match_trim_fraction", 0.90);
    scan_match_iterations_ = declare_parameter<int>("scan_match_iterations", 4);
    scan_match_min_points_ = declare_parameter<int>("scan_match_min_points", 12);
    scan_match_min_inlier_fraction_ = declare_parameter<double>(
      "scan_match_min_inlier_fraction", 0.55);
    scan_match_max_rmse_m_ = declare_parameter<double>("scan_match_max_rmse_m", 0.15);
    slip_translation_innovation_m_ = declare_parameter<double>(
      "slip_translation_innovation_m", 0.15);
    slip_yaw_innovation_rad_ = declare_parameter<double>("slip_yaw_innovation_rad", 0.15);
    slip_enter_score_ = declare_parameter<double>("slip_enter_score", 1.0);
    slip_immediate_score_ = declare_parameter<double>("slip_immediate_score", 2.0);
    slip_recover_score_ = declare_parameter<double>("slip_recover_score", 0.50);
    slip_enter_updates_ = declare_parameter<int>("slip_enter_updates", 2);
    slip_recover_updates_ = declare_parameter<int>("slip_recover_updates", 5);
    slip_motion_noise_multiplier_ = declare_parameter<double>(
      "slip_motion_noise_multiplier", 2.5);
    slip_bypass_min_fit_ = declare_parameter<double>(
      "slip_jump_bypass_min_scan_fit", 0.75);
    slip_bypass_max_translation_m_ = declare_parameter<double>(
      "slip_jump_bypass_max_translation_m", 0.80);
    slip_bypass_max_yaw_rad_ = declare_parameter<double>(
      "slip_jump_bypass_max_yaw_rad", 0.80);
  }

  void validate_supported_configuration() const
  {
    if (ground_filter_enabled_ || deskew_enabled_) {
      throw std::runtime_error(
              "particle_filter_cpp currently supports the validated "
              "no-ground/no-deskew profiles only; refusing to ignore enabled geometry features");
    }
    if (range_method_name_ != "glt") {
      throw std::runtime_error("particle_filter_cpp currently requires range_method=glt");
    }
    if (rangelib_variant_ < 0 || rangelib_variant_ > 5) {
      throw std::runtime_error("rangelib_variant must be in [0, 5]");
    }
    if (angle_step_ <= 0 || max_particles_ <= 0 || max_range_m_ <= 0.0 ||
      theta_discretization_ <= 0 || max_update_rate_hz_ <= 0.0 || random_seed_ < -1)
    {
      throw std::runtime_error("invalid particle_filter_cpp parameters");
    }
  }

  void request_map()
  {
    if (map_request_sent_.load() || map_initialized_.load()) {
      return;
    }
    if (!map_client_->service_is_ready()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000, "Get map service not available, waiting...");
      return;
    }
    bool expected = false;
    if (!map_request_sent_.compare_exchange_strong(expected, true)) {
      return;
    }
    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
    map_client_->async_send_request(
      request,
      [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
        try {
          initialize_map(future.get()->map);
        } catch (const std::exception & error) {
          map_request_sent_.store(false);
          RCLCPP_ERROR(get_logger(), "Failed to initialize C++ PF map: %s", error.what());
        }
      });
  }

  void initialize_map(const nav_msgs::msg::OccupancyGrid & map)
  {
    RCLCPP_INFO(get_logger(), "Initializing C++ RangeLib method: glt");
    if (map.info.width == 0U || map.info.height == 0U || map.info.resolution <= 0.0F ||
      map.data.size() != static_cast<std::size_t>(map.info.width * map.info.height))
    {
      throw std::runtime_error("invalid occupancy grid");
    }
    auto occupancy_map = std::make_unique<ranges::OMap>(
      static_cast<int>(map.info.height), static_cast<int>(map.info.width));
    for (std::size_t row = 0; row < map.info.height; ++row) {
      for (std::size_t column = 0; column < map.info.width; ++column) {
        occupancy_map->grid[row][column] =
          map.data[row * map.info.width + column] > 10;
      }
    }
    const double origin_yaw = yaw_from_quaternion(map.info.origin.orientation);
    occupancy_map->world_scale = map.info.resolution;
    occupancy_map->world_angle = static_cast<float>(-origin_yaw);
    occupancy_map->world_origin_x = static_cast<float>(map.info.origin.position.x);
    occupancy_map->world_origin_y = static_cast<float>(map.info.origin.position.y);
    occupancy_map->world_sin_angle = std::sin(occupancy_map->world_angle);
    occupancy_map->world_cos_angle = std::cos(occupancy_map->world_angle);
    max_range_px_ = static_cast<int>(max_range_m_ / map.info.resolution);
    auto range_method = std::make_unique<ranges::GiantLUTCast>(
      *occupancy_map, static_cast<float>(max_range_px_),
      static_cast<unsigned int>(theta_discretization_));

    MclParameters parameters;
    parameters.particle_count = static_cast<std::size_t>(max_particles_);
    parameters.motion_dispersion_x = motion_dispersion_x_;
    parameters.motion_dispersion_y = motion_dispersion_y_;
    parameters.motion_dispersion_theta = motion_dispersion_theta_;
    parameters.motion_noise_min_scale = motion_noise_min_scale_;
    parameters.motion_noise_reference_translation_m = motion_noise_reference_translation_m_;
    parameters.motion_noise_reference_yaw_rad = motion_noise_reference_yaw_rad_;
    parameters.resample_ess_ratio = resample_ess_ratio_;
    parameters.beam_skip_enabled = beam_skip_enabled_;
    parameters.beam_skip_distance_m = beam_skip_distance_m_;
    parameters.beam_skip_min_particle_fraction = beam_skip_min_particle_fraction_;
    parameters.beam_skip_max_fraction = beam_skip_max_fraction_;
    parameters.beam_skip_short_only = beam_skip_short_only_;
    parameters.squash_factor = squash_factor_;
    parameters.map_resolution = map.info.resolution;
    parameters.max_range_px = max_range_px_;
    parameters.z_short = z_short_;
    parameters.z_max = z_max_;
    parameters.z_rand = z_rand_;
    parameters.z_hit = z_hit_;
    parameters.sigma_hit = sigma_hit_;
    parameters.random_seed = random_seed_;
    auto core = std::make_unique<MclCore>(parameters);

    std::vector<Pose2d> permissible;
    permissible.reserve(map.data.size());
    const double cosine = std::cos(origin_yaw);
    const double sine = std::sin(origin_yaw);
    for (std::size_t row = 0; row < map.info.height; ++row) {
      for (std::size_t column = 0; column < map.info.width; ++column) {
        if (map.data[row * map.info.width + column] != 0) {
          continue;
        }
        const double grid_x = static_cast<double>(column) * map.info.resolution;
        const double grid_y = static_cast<double>(row) * map.info.resolution;
        permissible.push_back({
          map.info.origin.position.x + cosine * grid_x - sine * grid_y,
          map.info.origin.position.y + sine * grid_x + cosine * grid_y,
          0.0});
      }
    }
    core->initialize_global(permissible);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      map_info_ = map.info;
      occupancy_map_ = std::move(occupancy_map);
      range_method_ = std::move(range_method);
      core_ = std::move(core);
      permissible_poses_ = std::move(permissible);
      inferred_pose_.reset();
      map_initialized_ = true;
      readiness_tracker_.clear_initial_pose();
      reset_filter_reference_locked(true);
    }
    RCLCPP_INFO(
      get_logger(),
      "Finished initializing PF profile %s: C++ RangeLib variant=%d, "
      "ground-filter=OFF, deskew=OFF, scan-match/slip=%s; seed=%s; "
      "odom/IMU buffer=%zu/%zu, latest-scan worker<=%.1fHz; "
      "autonomy waits for /initialpose and quality OK",
      profile_label_.c_str(), rangelib_variant_, scan_match_enabled_ ? "ON" : "OFF",
      random_seed_ >= 0 ? std::to_string(random_seed_).c_str() : "random",
      odom_buffer_.capacity(), imu_buffer_.capacity(), max_update_rate_hz_);
    retry_initial_pose();
  }

  bool lookup_laser_to_base()
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (laser_to_base_) {
        return true;
      }
    }
    try {
      const auto transform = tf_buffer_->lookupTransform(
        laser_frame_, base_frame_, tf2::TimePointZero);
      const Pose2d laser_to_base{
        transform.transform.translation.x,
        transform.transform.translation.y,
        yaw_from_quaternion(transform.transform.rotation)};
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!laser_to_base_) {
        laser_to_base_ = laser_to_base;
      }
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    if (message->ranges.empty()) {
      return;
    }
    std::vector<float> angles;
    std::vector<float> ranges;
    angles.reserve((message->ranges.size() + static_cast<std::size_t>(angle_step_) - 1U) /
      static_cast<std::size_t>(angle_step_));
    ranges.reserve(angles.capacity());
    if (expected_scan_size_ == 0U) {
      expected_scan_size_ = message->ranges.size();
      RCLCPP_INFO(get_logger(), "...Received first LiDAR message");
    } else if (message->ranges.size() != expected_scan_size_) {
      ++scan_shape_drop_count_;
      return;
    }
    const float sensor_max = std::isfinite(message->range_max) ?
      message->range_max : static_cast<float>(max_range_m_);
    const float sensor_min = std::isfinite(message->range_min) ?
      std::max(0.0F, message->range_min) : 0.0F;
    for (std::size_t index = 0; index < message->ranges.size();
      index += static_cast<std::size_t>(angle_step_))
    {
      const float value = message->ranges[index];
      const bool valid = std::isfinite(value) && value >= sensor_min && value > 0.0F &&
        value <= sensor_max && value < 65.5F && value < max_range_m_ * 0.995;
      ranges.push_back(valid ? std::clamp(value, 0.0F, static_cast<float>(max_range_m_)) :
        static_cast<float>(max_range_m_));
      double angle = message->angle_min;
      if (std::isfinite(message->angle_increment) && message->angle_increment != 0.0F) {
        angle += static_cast<double>(index) * message->angle_increment;
      } else if (message->ranges.size() > 1U) {
        angle += (message->angle_max - message->angle_min) * static_cast<double>(index) /
          static_cast<double>(message->ranges.size() - 1U);
      }
      angles.push_back(static_cast<float>(angle));
    }
    int64_t timestamp = stamp_ns(message->header.stamp);
    if (timestamp <= 0) {
      timestamp = now().nanoseconds();
    }
    if (timestamp <= last_received_scan_stamp_ns_) {
      if (last_received_scan_stamp_ns_ - timestamp < 1000000000LL) {
        ++scan_out_of_order_count_;
        return;
      }
    }
    last_received_scan_stamp_ns_ = timestamp;
    if (offered_scan_count_ == 0U) {
      RCLCPP_INFO(
        get_logger(), "PF C++ raycast budget: %d particles x %zu beams = %zu/update",
        max_particles_, ranges.size(), static_cast<std::size_t>(max_particles_) * ranges.size());
    }
    ScanSample sample{timestamp, std::chrono::steady_clock::now(), std::move(ranges), std::move(angles)};
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      ++offered_scan_count_;
      if (pending_scan_) {
        ++overwritten_scan_count_;
      }
      pending_scan_ = std::move(sample);
    }
    scan_condition_.notify_one();
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    int64_t timestamp = stamp_ns(message->header.stamp);
    if (timestamp <= 0) {
      timestamp = now().nanoseconds();
    }
    OdomSample sample{
      timestamp,
      {
        message->pose.pose.position.x,
        message->pose.pose.position.y,
        yaw_from_quaternion(message->pose.pose.orientation)},
      message->twist.twist.linear.x,
      message->twist.twist.angular.z};
    const std::size_t reset_count = odom_buffer_.time_reset_count();
    if (!odom_buffer_.append(sample)) {
      ++odom_reject_count_;
      return;
    }
    if (odom_buffer_.time_reset_count() != reset_count) {
      RCLCPP_WARN(get_logger(), "Odom time moved backward; resetting C++ PF reference");
      std::lock_guard<std::mutex> lock(state_mutex_);
      reset_filter_reference_locked(true);
      last_published_odom_stamp_ns_ = 0;
    }
    odom_initialized_ = true;
    if (timestamp > last_published_odom_stamp_ns_) {
      publish_propagated_pose(sample);
      last_published_odom_stamp_ns_ = timestamp;
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    int64_t timestamp = stamp_ns(message->header.stamp);
    if (timestamp <= 0) {
      timestamp = now().nanoseconds();
    }
    const std::array<double, 6> values{
      message->angular_velocity.x, message->angular_velocity.y,
      message->angular_velocity.z, message->linear_acceleration.x,
      message->linear_acceleration.y, message->linear_acceleration.z};
    if (!std::all_of(values.begin(), values.end(), [](const double value) {
        return std::isfinite(value);
      }))
    {
      ++imu_reject_count_;
      return;
    }
    const std::size_t reset_count = imu_buffer_.time_reset_count();
    if (!imu_buffer_.append({
        timestamp,
        values[2] * imu_angular_velocity_scale_,
        values[3] * imu_linear_acceleration_scale_,
        values[4] * imu_linear_acceleration_scale_}))
    {
      ++imu_reject_count_;
      return;
    }
    if (imu_buffer_.time_reset_count() != reset_count) {
      RCLCPP_WARN(get_logger(), "IMU time moved backward; reset C++ integration history");
    }
    imu_initialized_ = true;
  }

  void initial_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    if (!finite_pose_message(message->pose.pose)) {
      RCLCPP_ERROR(get_logger(), "Ignoring /initialpose containing non-finite values");
      return;
    }
    const std::string key = std::to_string(message->header.stamp.sec) + ":" +
      std::to_string(message->header.stamp.nanosec) + ":" +
      std::to_string(message->pose.pose.position.x) + ":" +
      std::to_string(message->pose.pose.position.y) + ":" +
      std::to_string(message->pose.pose.orientation.z) + ":" +
      std::to_string(message->pose.pose.orientation.w);
    {
      std::lock_guard<std::mutex> lock(initial_pose_mutex_);
      if (key == last_initial_pose_key_) {
        return;
      }
      last_initial_pose_key_ = key;
      pending_initial_pose_ = message->pose.pose;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      readiness_tracker_.receive_initial_pose();
    }
    publish_status();
    RCLCPP_INFO(get_logger(), "/initialpose received by C++ PF; waiting for map/TF if needed");
    retry_initial_pose();
  }

  void retry_initial_pose()
  {
    std::optional<geometry_msgs::msg::Pose> pending;
    {
      std::lock_guard<std::mutex> lock(initial_pose_mutex_);
      pending = pending_initial_pose_;
    }
    if (!pending || !map_initialized_) {
      return;
    }
    if (!lookup_laser_to_base()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "/initialpose cached: waiting for laser<->base_link TF");
      return;
    }
    const Pose2d base_pose{
      pending->position.x, pending->position.y,
      yaw_from_quaternion(pending->orientation)};
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!core_ || !laser_to_base_) {
        return;
      }
      const Pose2d laser_pose = compose(base_pose, inverse(*laser_to_base_));
      core_->initialize_pose(laser_pose, 0.5, 0.4);
      inferred_pose_ = laser_pose;
      readiness_tracker_.accept_initial_pose();
      reset_filter_reference_locked(true);
    }
    {
      std::lock_guard<std::mutex> lock(initial_pose_mutex_);
      pending_initial_pose_.reset();
    }
    publish_status();
    RCLCPP_INFO(
      get_logger(), "/initialpose accepted by C++ PF; autonomy waits for quality OK");
  }

  void initialize_global()
  {
    if (!map_initialized_) {
      return;
    }
    {
      std::lock_guard<std::mutex> initial_lock(initial_pose_mutex_);
      pending_initial_pose_.reset();
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!core_ || permissible_poses_.empty()) {
        return;
      }
      core_->initialize_global(permissible_poses_);
      inferred_pose_.reset();
      readiness_tracker_.clear_initial_pose();
      reset_filter_reference_locked(true);
    }
    publish_status();
    RCLCPP_INFO(get_logger(), "C++ PF global particle initialization complete");
  }

  Pose2d low_speed_action(Pose2d action, const double speed) const
  {
    if (std::abs(speed) >= low_speed_threshold_mps_) {
      return action;
    }
    if (translation_norm(action) < low_speed_xy_deadband_m_) {
      action.x = 0.0;
      action.y = 0.0;
    }
    if (std::abs(action.yaw) < low_speed_yaw_deadband_rad_) {
      action.yaw = 0.0;
    }
    action.x *= low_speed_odom_scale_;
    action.y *= low_speed_odom_scale_;
    action.yaw *= low_speed_odom_scale_;
    return action;
  }

  void worker_loop()
  {
    auto next_allowed = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration<double>(1.0 / max_update_rate_hz_);
    while (!stop_worker_.load()) {
      ScanSample scan;
      {
        std::unique_lock<std::mutex> lock(scan_mutex_);
        scan_condition_.wait_for(lock, 100ms, [this] {
          return stop_worker_.load() || pending_scan_.has_value();
        });
        if (stop_worker_.load()) {
          break;
        }
        if (!pending_scan_) {
          continue;
        }
        scan = std::move(*pending_scan_);
        pending_scan_.reset();
      }
      const auto current = std::chrono::steady_clock::now();
      if (current < next_allowed) {
        std::this_thread::sleep_until(next_allowed);
      }
      std::optional<OdomSample> odom;
      const auto lookup_deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(odom_sync_wait_sec_);
      do {
        odom = odom_buffer_.lookup(scan.stamp_ns, max_odom_interp_gap_sec_);
        if (odom || std::chrono::steady_clock::now() >= lookup_deadline) {
          break;
        }
        std::this_thread::sleep_for(1ms);
      } while (!stop_worker_.load());
      if (!odom) {
        ++scan_sync_drop_count_;
        log_diagnostics();
        continue;
      }
      const auto age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - scan.received).count();
      if (age > max_pending_scan_age_sec_) {
        ++scan_stale_drop_count_;
        log_diagnostics();
        continue;
      }
      const auto started = std::chrono::steady_clock::now();
      try {
        if (!process_scan(scan, *odom)) {
          continue;
        }
      } catch (const std::exception & error) {
        ++worker_error_count_;
        RCLCPP_ERROR(get_logger(), "C++ PF worker error: %s", error.what());
        continue;
      }
      const auto finished = std::chrono::steady_clock::now();
      const double compute_ms = std::chrono::duration<double, std::milli>(
        finished - started).count();
      const double turnaround_ms = std::chrono::duration<double, std::milli>(
        finished - scan.received).count();
      {
        std::lock_guard<std::mutex> lock(metrics_mutex_);
        append_bounded(compute_ms_, compute_ms, 20U);
        append_bounded(turnaround_ms_, turnaround_ms, 20U);
        processed_times_.push_back(finished);
        while (processed_times_.size() > 40U) {
          processed_times_.pop_front();
        }
      }
      ++scan_processed_count_;
      next_allowed = started + std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      log_diagnostics();
    }
  }

  bool process_scan(const ScanSample & scan, const OdomSample & odom)
  {
    if (!map_initialized_ || !odom_initialized_) {
      return false;
    }
    if (!lookup_laser_to_base()) {
      return false;
    }
    std::unique_lock<std::mutex> lock(state_mutex_);
    if (!core_ || !range_method_) {
      return false;
    }
    Pose2d raw_action;
    if (previous_filter_odom_) {
      raw_action = compose(inverse(previous_filter_odom_->pose), odom.pose);
    }
    const Pose2d action = low_speed_action(raw_action, odom.linear_speed);
    ScanMotionEstimate scan_motion;
    if (scan_match_enabled_ && previous_scan_ranges_ && previous_scan_angles_) {
      scan_motion = estimate_scan_motion(
        *previous_scan_ranges_, scan.ranges, *previous_scan_angles_, scan.angles,
        raw_action, max_range_m_, scan_match_max_correspondence_m_,
        scan_match_trim_fraction_, scan_match_iterations_, scan_match_min_points_);
      scan_motion.valid = scan_motion.valid &&
        scan_motion.inlier_fraction >= scan_match_min_inlier_fraction_ &&
        scan_motion.rmse_m <= scan_match_max_rmse_m_;
      if (!scan_motion.valid) {
        ++scan_match_invalid_count_;
      }
    }
    ImuIntegration imu;
    if (previous_scan_stamp_ns_ && imu_initialized_ && scan.stamp_ns > *previous_scan_stamp_ns_) {
      imu = imu_buffer_.integrate(
        *previous_scan_stamp_ns_, scan.stamp_ns, max_imu_interp_gap_sec_);
      if (!imu.valid) {
        ++imu_sync_miss_count_;
      }
    }
    const bool imu_stationary = imu.valid &&
      imu.max_abs_yaw_rate <= stationary_imu_yaw_rate_rad_s_ &&
      imu.max_horizontal_acceleration <= stationary_imu_accel_mps2_;
    SlipUpdate slip = scan_match_enabled_ ? slip_detector_->update(
      raw_action, imu.delta_yaw, imu.valid, scan_motion.action, scan_motion.valid) :
      slip_detector_->disabled_update();
    if (slip.entered) {
      ++slip_event_count_;
    }
    if (slip.recovered) {
      ++slip_recovery_count_;
    }
    last_slip_ = slip;
    last_scan_motion_ = scan_motion;

    const auto prior_particles = core_->particles();
    const auto prior_weights = core_->weights();
    const auto prior_pose = inferred_pose_;
    Proposal proposal = core_->prepare_proposal(
      action, slip.active ? slip_motion_noise_multiplier_ : 1.0);
    auto queries = core_->particle_query_array(proposal);
    std::vector<float> predicted(
      proposal.particles.size() * scan.ranges.size(), static_cast<float>(max_range_m_));
    range_method_->numpy_calc_range_angles(
      queries.data(), const_cast<float *>(scan.angles.data()), predicted.data(),
      static_cast<int>(proposal.particles.size()), static_cast<int>(scan.ranges.size()));
    SensorEvaluation sensor = core_->evaluate_sensor(
      proposal, scan.ranges, predicted);
    bool correction_accepted = false;
    bool stationary_hold = false;
    if (sensor.valid) {
      last_prior_ess_ratio_ = proposal.prior_ess_ratio;
      if (proposal.resampled) {
        ++resample_count_;
      }
      if (sensor.beams.fallback_to_all_beams) {
        ++beam_fallback_count_;
      }
      last_beam_selection_ = sensor.beams;
      core_->accept(std::move(proposal), std::move(sensor.posterior_weights));
      const Pose2d candidate = core_->expected_pose();
      const bool wheel_stationary = std::abs(odom.linear_speed) < low_speed_threshold_mps_ &&
        translation_norm(action) <= 1.0e-12 && std::abs(action.yaw) <= 1.0e-12;
      const bool lidar_stationary = scan_motion.valid &&
        translation_norm(scan_motion.action) <= stationary_scan_translation_m_ &&
        std::abs(scan_motion.action.yaw) <= stationary_scan_yaw_rad_;
      stationary_hold = freeze_stationary_filter_ && wheel_stationary &&
        imu_stationary && lidar_stationary && !slip.active;
      if (stationary_hold) {
        ++stationary_hold_count_;
        consecutive_jump_rejects_ = 0;
        core_->restore_motion_only(prior_particles, prior_weights, action);
      } else if (pose_jump_guard_enabled_ && prior_pose) {
        const Pose2d predicted_pose = compose(*prior_pose, action);
        const Pose2d correction = compose(inverse(predicted_pose), candidate);
        const double correction_xy = translation_norm(correction);
        const double correction_yaw = std::abs(correction.yaw);
        double max_xy = max_pose_jump_translation_m_;
        double max_yaw = max_pose_jump_yaw_rad_;
        if (std::abs(odom.linear_speed) < low_speed_threshold_mps_) {
          max_xy = low_speed_jump_translation_m_;
          max_yaw = low_speed_jump_yaw_rad_;
        }
        bool reject = correction_xy > max_xy || correction_yaw > max_yaw;
        const bool bypass = reject && slip.active &&
          sensor.beams.weighted_inlier_fraction >= slip_bypass_min_fit_ &&
          correction_xy <= slip_bypass_max_translation_m_ &&
          correction_yaw <= slip_bypass_max_yaw_rad_;
        if (bypass) {
          ++slip_bypass_count_;
          reject = false;
        }
        if (reject && max_consecutive_jump_rejects_ > 0 &&
          consecutive_jump_rejects_ + 1 >= max_consecutive_jump_rejects_)
        {
          reject = false;
        }
        if (reject) {
          ++jump_reject_count_;
          ++consecutive_jump_rejects_;
          core_->restore_motion_only(prior_particles, prior_weights, action);
        } else {
          consecutive_jump_rejects_ = 0;
          inferred_pose_ = candidate;
          correction_accepted = true;
        }
      } else {
        inferred_pose_ = candidate;
        correction_accepted = true;
      }
    } else {
      core_->restore_motion_only(prior_particles, prior_weights, action);
      last_beam_selection_.reset();
    }
    if (!stationary_hold && !correction_accepted) {
      inferred_pose_ = core_->expected_pose();
    }
    last_posterior_ess_ratio_ = core_->effective_sample_size_ratio();
    if (last_beam_selection_) {
      const double quality_score = correction_accepted || stationary_hold ?
        last_beam_selection_->weighted_inlier_fraction : 0.0;
      const QualityState previous = quality_tracker_.state();
      quality_tracker_.update(quality_score);
      readiness_tracker_.update(quality_tracker_.state());
      if (quality_tracker_.state() == QualityState::kLost) {
        ++quality_lost_update_count_;
      }
      if (previous != QualityState::kLost && quality_tracker_.state() == QualityState::kLost) {
        ++quality_lost_transition_count_;
      }
    }
    if (inferred_pose_ && !(quality_gate_enabled_ && quality_tracker_.state() == QualityState::kLost)) {
      set_localization_transform_locked(*inferred_pose_, odom);
    }
    previous_filter_odom_ = odom;
    previous_scan_stamp_ns_ = scan.stamp_ns;
    if (scan_match_enabled_) {
      previous_scan_ranges_ = scan.ranges;
      previous_scan_angles_ = scan.angles;
    } else {
      previous_scan_ranges_.reset();
      previous_scan_angles_.reset();
    }
    current_scan_ranges_ = scan.ranges;
    current_scan_angles_ = scan.angles;
    last_processed_stamp_ns_ = scan.stamp_ns;
    visualize_locked();
    lock.unlock();
    publish_status();
    return true;
  }

  void set_localization_transform_locked(const Pose2d & laser_pose, const OdomSample & odom)
  {
    const Pose2d map_to_base = compose(laser_pose, *laser_to_base_);
    const Pose2d map_to_odom = compose(map_to_base, inverse(odom.pose));
    std::array<double, 36> covariance{};
    const auto covariance3 = core_->covariance_3x3();
    const std::array<std::size_t, 3> indices{0U, 1U, 5U};
    for (std::size_t row = 0; row < 3U; ++row) {
      for (std::size_t column = 0; column < 3U; ++column) {
        covariance[indices[row] * 6U + indices[column]] = covariance3[row * 3U + column];
      }
    }
    std::lock_guard<std::mutex> transform_lock(transform_mutex_);
    map_to_odom_ = map_to_odom;
    localized_covariance_ = covariance;
    correction_updated_ = std::chrono::steady_clock::now();
  }

  void publish_propagated_pose(const OdomSample & odom)
  {
    std::optional<Pose2d> transform;
    std::array<double, 36> covariance{};
    std::chrono::steady_clock::time_point updated;
    {
      std::lock_guard<std::mutex> lock(transform_mutex_);
      transform = map_to_odom_;
      covariance = localized_covariance_;
      updated = correction_updated_;
    }
    if (!transform) {
      return;
    }
    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - updated).count();
    if (age > max_localization_update_age_sec_) {
      return;
    }
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = to_stamp(odom.stamp_ns);
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    tf.transform.translation.x = transform->x;
    tf.transform.translation.y = transform->y;
    tf.transform.rotation = quaternion_from_yaw(transform->yaw);
    tf_broadcaster_->sendTransform(tf);
    if (!publish_odom_ || !odom_pub_) {
      return;
    }
    const Pose2d base_pose = compose(*transform, odom.pose);
    nav_msgs::msg::Odometry output;
    output.header = tf.header;
    output.child_frame_id = base_frame_;
    output.pose.pose.position.x = base_pose.x;
    output.pose.pose.position.y = base_pose.y;
    output.pose.pose.orientation = quaternion_from_yaw(base_pose.yaw);
    output.pose.covariance = covariance;
    output.twist.twist.linear.x = odom.linear_speed;
    output.twist.twist.angular.z = odom.angular_speed;
    odom_pub_->publish(output);
  }

  void publish_status()
  {
    std_msgs::msg::Bool ready;
    std_msgs::msg::String quality;
    std_msgs::msg::Bool received;
    std_msgs::msg::Bool accepted;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      ready.data = readiness_tracker_.ready();
      quality.data = quality_state_name(quality_tracker_.state());
      received.data = readiness_tracker_.initial_pose_received();
      accepted.data = readiness_tracker_.initial_pose_accepted();
    }
    ready_pub_->publish(ready);
    quality_pub_->publish(quality);
    initial_received_pub_->publish(received);
    initial_accepted_pub_->publish(accepted);
  }

  void visualize_locked()
  {
    if (!visualize_ || !inferred_pose_ || !laser_to_base_) {
      return;
    }
    const Pose2d base_pose = compose(*inferred_pose_, *laser_to_base_);
    if (pose_pub_->get_subscription_count() > 0U) {
      geometry_msgs::msg::PoseStamped message;
      message.header.stamp = to_stamp(last_processed_stamp_ns_);
      message.header.frame_id = map_frame_;
      message.pose.position.x = base_pose.x;
      message.pose.position.y = base_pose.y;
      message.pose.orientation = quaternion_from_yaw(base_pose.yaw);
      pose_pub_->publish(message);
    }
    if (particle_pub_->get_subscription_count() > 0U) {
      geometry_msgs::msg::PoseArray message;
      message.header.stamp = to_stamp(last_processed_stamp_ns_);
      message.header.frame_id = map_frame_;
      const auto & particles = core_->particles();
      const std::size_t count = std::min<std::size_t>(
        static_cast<std::size_t>(max_viz_particles_), particles.size());
      message.poses.reserve(count);
      for (std::size_t output = 0; output < count; ++output) {
        const std::size_t index = output * particles.size() / count;
        geometry_msgs::msg::Pose pose;
        pose.position.x = particles[index].x;
        pose.position.y = particles[index].y;
        pose.orientation = quaternion_from_yaw(particles[index].yaw);
        message.poses.push_back(pose);
      }
      particle_pub_->publish(message);
    }
    if (fake_scan_pub_->get_subscription_count() > 0U && !current_scan_angles_.empty()) {
      std::vector<float> query{
        static_cast<float>(inferred_pose_->x), static_cast<float>(inferred_pose_->y),
        static_cast<float>(inferred_pose_->yaw)};
      std::vector<float> ranges(current_scan_angles_.size());
      range_method_->numpy_calc_range_angles(
        query.data(), current_scan_angles_.data(), ranges.data(), 1,
        static_cast<int>(current_scan_angles_.size()));
      sensor_msgs::msg::LaserScan message;
      message.header.stamp = to_stamp(last_processed_stamp_ns_);
      message.header.frame_id = laser_frame_;
      message.angle_min = current_scan_angles_.front();
      message.angle_max = current_scan_angles_.back();
      message.angle_increment = current_scan_angles_.size() > 1U ?
        std::abs(current_scan_angles_[1] - current_scan_angles_[0]) : 0.0F;
      message.range_min = 0.0F;
      message.range_max = static_cast<float>(max_range_m_);
      message.ranges = std::move(ranges);
      fake_scan_pub_->publish(message);
    }
  }

  void reset_filter_reference_locked(const bool clear_transform)
  {
    previous_filter_odom_.reset();
    previous_scan_stamp_ns_.reset();
    previous_scan_ranges_.reset();
    previous_scan_angles_.reset();
    quality_tracker_.reset();
    readiness_tracker_.reset_quality();
    if (slip_detector_) {
      slip_detector_->reset();
    }
    if (clear_transform) {
      std::lock_guard<std::mutex> lock(transform_mutex_);
      map_to_odom_.reset();
      localized_covariance_.fill(0.0);
    }
  }

  void log_diagnostics()
  {
    const auto current = std::chrono::steady_clock::now();
    if (last_diagnostic_time_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(current - last_diagnostic_time_).count() <
      diagnostics_period_sec_)
    {
      return;
    }
    last_diagnostic_time_ = current;
    double rate = 0.0;
    double compute_ms = 0.0;
    double turnaround_ms = 0.0;
    {
      std::lock_guard<std::mutex> lock(metrics_mutex_);
      if (processed_times_.size() >= 2U) {
        const double span = std::chrono::duration<double>(
          processed_times_.back() - processed_times_.front()).count();
        rate = span > 0.0 ? static_cast<double>(processed_times_.size() - 1U) / span : 0.0;
      }
      compute_ms = mean_or_zero(compute_ms_);
      turnaround_ms = mean_or_zero(turnaround_ms_);
    }
    double correction_age_ms = -1.0;
    {
      std::lock_guard<std::mutex> lock(transform_mutex_);
      if (map_to_odom_) {
        correction_age_ms = std::chrono::duration<double, std::milli>(
          current - correction_updated_).count();
      }
    }
    double scan_fit = -1.0;
    double beam_skip = -1.0;
    std::string quality = "STARTUP";
    int good = 0;
    int bad = 0;
    bool ready = false;
    bool initial_received = false;
    bool initial_accepted = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (last_beam_selection_) {
        scan_fit = last_beam_selection_->weighted_inlier_fraction;
        beam_skip = 100.0 * last_beam_selection_->skipped_fraction;
      }
      quality = quality_state_name(quality_tracker_.state());
      good = quality_tracker_.consecutive_good();
      bad = quality_tracker_.consecutive_lost();
      ready = readiness_tracker_.ready();
      initial_received = readiness_tracker_.initial_pose_received();
      initial_accepted = readiness_tracker_.initial_pose_accepted();
    }
    const double overwrite_percent = 100.0 * static_cast<double>(overwritten_scan_count_) /
      static_cast<double>(std::max<std::size_t>(1U, offered_scan_count_));
    const double core_load = rate * compute_ms / 10.0;
    const double scan_translation = translation_norm(last_scan_motion_.action);
    RCLCPP_INFO(
      get_logger(),
      "PF sync: %.1fHz, compute %.1fms, scan turnaround %.1fms, correction-age %.0fms, "
      "CPU-core %.0f%%, scans %zu/%zu, latest-overwrite %zu (%.0f%%), "
      "sync/stale/order-drop %zu/%zu/%zu, worker-error %zu, odom-buffer %zu/%zu, "
      "ESS-prior/post %.2f/%.2f, scan-fit %.2f, beam-skip %.0f%%, beam-fallback %zu, "
      "quality %s good/bad %d/%d, ready/initialpose-received/accepted %d/%d/%d, "
      "resampled/lost-updates/lost-transitions %zu/%zu/%zu, "
      "stationary-hold %zu, stationary wheel/IMU/scan 0/0/0, "
      "IMU-buffer %zu/%zu, imu-miss/reject %zu/%zu, scan-match-invalid %zu, "
      "scan-motion %s %.3fm/%.1fdeg rmse %.3fm inlier %.2f, "
      "slip %s score %.2f, events/recoveries/bypass %zu/%zu/%zu, "
      "slip-age/last-duration 0.00/0.00s, slip-max-correction 0.00m/0.0deg, "
      "deskew OFF applied 0, timing/odom/imu-fallback 0/0/0, scan-span 0.0ms, "
      "translation-scale 0.00, deskew-max-motion 0.000m/0.0deg, "
      "geometry disabled ground/offplane/degraded/sync-miss/fallback 0/0/0/0/0, "
      "attitude-buffer 0/%zu level waiting, shape-drop %zu, profile %s, engine cpp",
      rate, compute_ms, turnaround_ms, correction_age_ms, core_load,
      scan_processed_count_.load(), offered_scan_count_.load(), overwritten_scan_count_.load(),
      overwrite_percent, scan_sync_drop_count_.load(), scan_stale_drop_count_.load(),
      scan_out_of_order_count_.load(), worker_error_count_.load(), odom_buffer_.size(),
      odom_buffer_.capacity(), last_prior_ess_ratio_, last_posterior_ess_ratio_,
      scan_fit, beam_skip, beam_fallback_count_.load(), quality.c_str(), good, bad,
      ready ? 1 : 0, initial_received ? 1 : 0, initial_accepted ? 1 : 0,
      resample_count_.load(), quality_lost_update_count_.load(),
      quality_lost_transition_count_.load(), stationary_hold_count_.load(),
      imu_buffer_.size(), imu_buffer_.capacity(), imu_sync_miss_count_.load(),
      imu_reject_count_.load(), scan_match_invalid_count_.load(),
      last_scan_motion_.valid ? "OK" : "INVALID", scan_translation,
      std::abs(last_scan_motion_.action.yaw) * 180.0 / M_PI,
      last_scan_motion_.valid ? last_scan_motion_.rmse_m : -1.0,
      last_scan_motion_.valid ? last_scan_motion_.inlier_fraction : -1.0,
      last_slip_.active ? "ACTIVE" : "OK", last_slip_.score,
      slip_event_count_.load(), slip_recovery_count_.load(), slip_bypass_count_.load(),
      imu_buffer_.capacity(), scan_shape_drop_count_.load(), profile_label_.c_str());
  }

  int angle_step_{15};
  int max_particles_{2000};
  int max_viz_particles_{60};
  double squash_factor_{2.2};
  double max_range_m_{10.0};
  int max_range_px_{200};
  int theta_discretization_{112};
  std::string range_method_name_;
  int rangelib_variant_{0};
  std::string profile_label_;
  int random_seed_{-1};
  bool publish_odom_{true};
  bool visualize_{true};
  double z_short_{0.01};
  double z_max_{0.07};
  double z_rand_{0.12};
  double z_hit_{0.75};
  double sigma_hit_{4.0};
  double motion_dispersion_x_{0.05};
  double motion_dispersion_y_{0.025};
  double motion_dispersion_theta_{0.08};
  std::string scan_topic_;
  std::string odom_topic_;
  std::string imu_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string laser_frame_;
  bool pose_jump_guard_enabled_{true};
  double max_pose_jump_translation_m_{0.30};
  double max_pose_jump_yaw_rad_{0.35};
  double low_speed_threshold_mps_{0.2};
  double low_speed_xy_deadband_m_{0.01};
  double low_speed_yaw_deadband_rad_{0.02};
  double low_speed_odom_scale_{0.25};
  double low_speed_jump_translation_m_{0.50};
  double low_speed_jump_yaw_rad_{0.60};
  int max_consecutive_jump_rejects_{10};
  double odom_sync_wait_sec_{0.04};
  double max_odom_interp_gap_sec_{0.06};
  double max_pending_scan_age_sec_{0.10};
  double max_update_rate_hz_{15.0};
  double diagnostics_period_sec_{1.0};
  double max_localization_update_age_sec_{0.50};
  double motion_noise_min_scale_{0.08};
  double motion_noise_reference_translation_m_{0.25};
  double motion_noise_reference_yaw_rad_{0.15};
  double resample_ess_ratio_{0.50};
  bool beam_skip_enabled_{true};
  double beam_skip_distance_m_{0.20};
  double beam_skip_min_particle_fraction_{0.30};
  double beam_skip_max_fraction_{0.50};
  bool beam_skip_short_only_{true};
  bool quality_gate_enabled_{false};
  std::string ready_topic_;
  std::string quality_topic_;
  std::string initial_received_topic_;
  std::string initial_accepted_topic_;
  bool freeze_stationary_filter_{true};
  bool deskew_enabled_{true};
  bool ground_filter_enabled_{true};
  bool scan_match_enabled_{true};
  double imu_angular_velocity_scale_{0.017453292519943295};
  double imu_linear_acceleration_scale_{9.80665};
  double max_imu_interp_gap_sec_{0.06};
  double stationary_imu_yaw_rate_rad_s_{0.06};
  double stationary_imu_accel_mps2_{1.60};
  double stationary_scan_translation_m_{0.04};
  double stationary_scan_yaw_rad_{0.04};
  double scan_match_max_correspondence_m_{0.40};
  double scan_match_trim_fraction_{0.90};
  int scan_match_iterations_{4};
  int scan_match_min_points_{12};
  double scan_match_min_inlier_fraction_{0.55};
  double scan_match_max_rmse_m_{0.15};
  double slip_translation_innovation_m_{0.15};
  double slip_yaw_innovation_rad_{0.15};
  double slip_enter_score_{1.0};
  double slip_immediate_score_{2.0};
  double slip_recover_score_{0.50};
  int slip_enter_updates_{2};
  int slip_recover_updates_{5};
  double slip_motion_noise_multiplier_{2.5};
  double slip_bypass_min_fit_{0.75};
  double slip_bypass_max_translation_m_{0.80};
  double slip_bypass_max_yaw_rad_{0.80};

  OdomBuffer odom_buffer_;
  ImuBuffer imu_buffer_;
  QualityTracker quality_tracker_;
  ReadinessTracker readiness_tracker_;
  std::unique_ptr<SlipDetector> slip_detector_;
  std::unique_ptr<MclCore> core_;
  std::vector<Pose2d> permissible_poses_;
  std::unique_ptr<ranges::OMap> occupancy_map_;
  std::unique_ptr<ranges::RangeMethod> range_method_;
  std::optional<nav_msgs::msg::MapMetaData> map_info_;
  std::atomic<bool> map_initialized_{false};
  std::atomic<bool> map_request_sent_{false};
  std::atomic<bool> odom_initialized_{false};
  std::atomic<bool> imu_initialized_{false};
  std::optional<Pose2d> laser_to_base_;
  std::optional<Pose2d> inferred_pose_;
  std::optional<OdomSample> previous_filter_odom_;
  std::optional<int64_t> previous_scan_stamp_ns_;
  std::optional<std::vector<float>> previous_scan_ranges_;
  std::optional<std::vector<float>> previous_scan_angles_;
  std::vector<float> current_scan_ranges_;
  std::vector<float> current_scan_angles_;
  std::optional<BeamSelection> last_beam_selection_;
  ScanMotionEstimate last_scan_motion_;
  SlipUpdate last_slip_;
  double last_prior_ess_ratio_{1.0};
  double last_posterior_ess_ratio_{1.0};
  int consecutive_jump_rejects_{0};

  std::mutex state_mutex_;
  std::mutex transform_mutex_;
  std::optional<Pose2d> map_to_odom_;
  std::array<double, 36> localized_covariance_{};
  std::chrono::steady_clock::time_point correction_updated_{};
  std::mutex initial_pose_mutex_;
  std::optional<geometry_msgs::msg::Pose> pending_initial_pose_;
  std::string last_initial_pose_key_;
  std::mutex scan_mutex_;
  std::condition_variable scan_condition_;
  std::optional<ScanSample> pending_scan_;
  std::atomic<bool> stop_worker_{false};
  std::thread worker_;
  std::mutex metrics_mutex_;
  std::deque<double> compute_ms_;
  std::deque<double> turnaround_ms_;
  std::deque<std::chrono::steady_clock::time_point> processed_times_;
  std::chrono::steady_clock::time_point last_diagnostic_time_{};

  std::size_t expected_scan_size_{0};
  int64_t last_received_scan_stamp_ns_{0};
  int64_t last_published_odom_stamp_ns_{0};
  int64_t last_processed_stamp_ns_{0};
  std::atomic<std::size_t> offered_scan_count_{0};
  std::atomic<std::size_t> overwritten_scan_count_{0};
  std::atomic<std::size_t> scan_processed_count_{0};
  std::atomic<std::size_t> scan_sync_drop_count_{0};
  std::atomic<std::size_t> scan_stale_drop_count_{0};
  std::atomic<std::size_t> scan_out_of_order_count_{0};
  std::atomic<std::size_t> scan_shape_drop_count_{0};
  std::atomic<std::size_t> worker_error_count_{0};
  std::atomic<std::size_t> odom_reject_count_{0};
  std::atomic<std::size_t> imu_reject_count_{0};
  std::atomic<std::size_t> imu_sync_miss_count_{0};
  std::atomic<std::size_t> scan_match_invalid_count_{0};
  std::atomic<std::size_t> resample_count_{0};
  std::atomic<std::size_t> beam_fallback_count_{0};
  std::atomic<std::size_t> quality_lost_update_count_{0};
  std::atomic<std::size_t> quality_lost_transition_count_{0};
  std::atomic<std::size_t> stationary_hold_count_{0};
  std::atomic<std::size_t> jump_reject_count_{0};
  std::atomic<std::size_t> slip_event_count_{0};
  std::atomic<std::size_t> slip_recovery_count_{0};
  std::atomic<std::size_t> slip_bypass_count_{0};

  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_client_;
  rclcpp::TimerBase::SharedPtr map_retry_timer_;
  rclcpp::TimerBase::SharedPtr initial_pose_timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr quality_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr initial_received_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr initial_accepted_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr fake_scan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_latched_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace particle_filter_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<particle_filter_cpp::ParticleFilterNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3U);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("particle_filter_cpp"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
