#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "f1tenth_planning_msgs/msg/wpnt.hpp"
#include "f1tenth_planning_msgs/msg/wpnt_array.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "example_interfaces/srv/trigger.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
constexpr double kEpsilon = 1.0e-9;

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double norm = std::sqrt(
    q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(norm) || norm <= kEpsilon) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double x = q.x / norm;
  const double y = q.y / norm;
  const double z = q.z / norm;
  const double w = q.w / norm;
  return std::atan2(
    2.0 * (w * z + x * y),
    1.0 - 2.0 * (y * y + z * z));
}
}  // namespace

// C++ port of steering_lookup/lookup_steer_angle.py. The first CSV row holds
// velocities, the first column holds steering angles, and the body holds the
// lateral acceleration available at each (steering, velocity) pair.
class LookupSteerAngle
{
public:
  bool load(const std::string & file_path, const rclcpp::Logger & logger)
  {
    table_.clear();
    std::ifstream file(file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(logger, "Steering lookup table not found: %s", file_path.c_str());
      return false;
    }

    std::vector<std::vector<double>> parsed;
    std::string line;
    std::size_t expected_columns = 0;
    while (std::getline(file, line)) {
      if (line.empty()) {
        continue;
      }

      std::vector<double> row;
      std::stringstream line_stream(line);
      std::string cell;
      while (std::getline(line_stream, cell, ',')) {
        try {
          const auto first = cell.find_first_not_of(" \t\r");
          const auto last = cell.find_last_not_of(" \t\r");
          if (first == std::string::npos) {
            throw std::invalid_argument("empty cell");
          }
          cell = cell.substr(first, last - first + 1U);
          std::size_t parsed = 0;
          const double value = std::stod(cell, &parsed);
          if (parsed != cell.size()) {
            throw std::invalid_argument("trailing characters");
          }
          row.push_back(value);
        } catch (const std::exception &) {
          RCLCPP_ERROR(logger, "Invalid value in steering lookup table: '%s'", cell.c_str());
          return false;
        }
      }

      if (expected_columns == 0) {
        expected_columns = row.size();
      }
      if (row.size() != expected_columns) {
        RCLCPP_ERROR(logger, "Steering lookup table has inconsistent row lengths");
        return false;
      }
      parsed.push_back(std::move(row));
    }

    if (parsed.size() < 2 || expected_columns < 2) {
      RCLCPP_ERROR(logger, "Steering lookup table must contain at least two rows and columns");
      return false;
    }

    if (!std::isfinite(parsed[0][0]) || std::abs(parsed[0][0]) > kEpsilon ||
      !std::isfinite(parsed[1][0]) || std::abs(parsed[1][0]) > kEpsilon)
    {
      RCLCPP_ERROR(logger, "Steering lookup table must start at zero velocity/steering");
      return false;
    }
    for (std::size_t column = 1; column < expected_columns; ++column) {
      if (!std::isfinite(parsed[0][column]) || parsed[0][column] <= 0.0 ||
        (column > 1 && parsed[0][column] <= parsed[0][column - 1]))
      {
        RCLCPP_ERROR(logger, "Steering lookup velocity header is invalid/non-monotonic");
        return false;
      }
      bool seen_non_finite = false;
      std::size_t finite_count = 0;
      double previous_acceleration = -std::numeric_limits<double>::infinity();
      for (std::size_t row = 1; row < parsed.size(); ++row) {
        const double value = parsed[row][column];
        if (!std::isfinite(value)) {
          seen_non_finite = true;
          continue;
        }
        if (seen_non_finite || value < 0.0 ||
          value + kEpsilon < previous_acceleration)
        {
          RCLCPP_ERROR(
            logger,
            "Steering lookup column %zu has an invalid/non-monotonic finite region",
            column);
          return false;
        }
        previous_acceleration = value;
        ++finite_count;
      }
      if (finite_count < 2U) {
        RCLCPP_ERROR(
          logger, "Steering lookup column %zu has fewer than two finite samples", column);
        return false;
      }
    }
    for (std::size_t row = 2; row < parsed.size(); ++row) {
      if (!std::isfinite(parsed[row][0]) ||
        parsed[row][0] + kEpsilon < parsed[row - 1][0])
      {
        RCLCPP_ERROR(logger, "Steering lookup angle axis is invalid/non-monotonic");
        return false;
      }
    }

    table_ = std::move(parsed);
    RCLCPP_INFO(
      logger, "Loaded steering lookup table: %s (%zu x %zu)",
      file_path.c_str(), table_.size(), table_.front().size());
    return true;
  }

  bool loaded() const
  {
    return !table_.empty();
  }

  double lookup_steer_angle(double acceleration, double velocity) const
  {
    if (!loaded()) {
      return 0.0;
    }

    const double sign = acceleration > 0.0 ? 1.0 : -1.0;
    acceleration = std::abs(acceleration);

    // Match the Python implementation: use the single closest velocity column.
    std::size_t velocity_column = 1;
    double closest_velocity_error = std::numeric_limits<double>::max();
    for (std::size_t column = 1; column < table_.front().size(); ++column) {
      if (!std::isfinite(table_.front()[column])) {
        continue;
      }
      const double error = std::abs(table_.front()[column] - velocity);
      if (error < closest_velocity_error) {
        closest_velocity_error = error;
        velocity_column = column;
      }
    }

    std::vector<std::size_t> finite_rows;
    finite_rows.reserve(table_.size() - 1);
    for (std::size_t row = 1; row < table_.size(); ++row) {
      if (std::isfinite(table_[row][velocity_column]) && std::isfinite(table_[row][0])) {
        finite_rows.push_back(row);
      }
    }
    if (finite_rows.empty()) {
      return 0.0;
    }

    std::size_t closest_position = 0;
    double closest_acceleration_error = std::numeric_limits<double>::max();
    for (std::size_t position = 0; position < finite_rows.size(); ++position) {
      const double error =
        std::abs(table_[finite_rows[position]][velocity_column] - acceleration);
      if (error < closest_acceleration_error) {
        closest_acceleration_error = error;
        closest_position = position;
      }
    }

    // This is the zero-steering row in the ETH lookup implementation.
    if (closest_position == 0) {
      return 0.0;
    }

    std::size_t second_position = closest_position;
    if (closest_position + 1 >= finite_rows.size()) {
      second_position = closest_position;
    } else {
      const std::size_t before = closest_position - 1;
      const std::size_t after = closest_position + 1;
      const double before_error =
        std::abs(table_[finite_rows[before]][velocity_column] - acceleration);
      const double after_error =
        std::abs(table_[finite_rows[after]][velocity_column] - acceleration);
      second_position = before_error <= after_error ? before : after;
    }

    const std::size_t first_row = finite_rows[closest_position];
    const std::size_t second_row = finite_rows[second_position];
    double steering_angle = table_[first_row][0];
    if (first_row != second_row) {
      const double first_acceleration = table_[first_row][velocity_column];
      const double second_acceleration = table_[second_row][velocity_column];
      if (std::abs(second_acceleration - first_acceleration) > kEpsilon) {
        const double ratio = std::clamp(
          (acceleration - first_acceleration) /
          (second_acceleration - first_acceleration), 0.0, 1.0);
        steering_angle = table_[first_row][0] +
          ratio * (table_[second_row][0] - table_[first_row][0]);
      }
    }
    return steering_angle * sign;
  }

private:
  std::vector<std::vector<double>> table_;
};

class ControlNode : public rclcpp::Node
{
public:
  ControlNode()
  : Node("control")
  {
    // ROS interfaces. The planning branch publishes WpntArray on /local_waypoints.
    control_rate_ = this->declare_parameter<double>("control_rate", 40.0);
    const auto waypoint_topic = this->declare_parameter<std::string>(
      "local_waypoints_topic", "/local_waypoints");
    const auto odom_topic = this->declare_parameter<std::string>(
      "odom_topic", "/pf/pose/odom");
    const auto frenet_odom_topic = this->declare_parameter<std::string>(
      "frenet_odom_topic", "/car_state/frenet/odom");
    const auto imu_topic = this->declare_parameter<std::string>(
      "imu_topic", "/sensors/imu/raw");
    const auto output_topic = this->declare_parameter<std::string>(
      "ackermann_topic", "/ackermann_cmd");
    const auto estop_topic = this->declare_parameter<std::string>(
      "estop_topic", "/safety/emergency_stop");
    localization_gate_enabled_ = this->declare_parameter<bool>(
      "localization_gate/enabled", false);
    const auto localization_ready_topic = this->declare_parameter<std::string>(
      "localization_gate/ready_topic", "/pf/localization_ready");
    localization_ready_timeout_ = this->declare_parameter<double>(
      "localization_gate/status_timeout_sec", 0.50);
    const auto recovery_state_topic = this->declare_parameter<std::string>(
      "recovery/state_topic", "/planner/recovery/state");
    const auto recovery_target_speed_topic = this->declare_parameter<std::string>(
      "recovery/target_speed_topic", "/planner/recovery/target_speed");
    const auto service_name = this->declare_parameter<std::string>(
      "service_name", "/control/set_mode");
    autonomous_ = this->declare_parameter<bool>("autonomous_on_start", false);

    waypoint_timeout_ = this->declare_parameter<double>("waypoint_timeout_sec", 0.5);
    odom_timeout_ = this->declare_parameter<double>("odom_timeout_sec", 0.5);
    frenet_odom_timeout_ = this->declare_parameter<double>("frenet_odom_timeout_sec", 0.5);
    imu_acceleration_sign_ = this->declare_parameter<double>("imu_acceleration_sign", -1.0);
    require_source_stamps_ = this->declare_parameter<bool>(
      "source_freshness/require_header_stamp", true);
    source_stamp_future_tolerance_ = this->declare_parameter<double>(
      "source_freshness/future_tolerance_sec", 0.1);

    // Recovery state와 target_speed는 header가 없는 두 토픽이므로 수신 시각을
    // 한 쌍의 짧은 lease로 검증한다. 공용/bench 설정만 기본 false이고
    // real_car/race_map/sim 환경은 true로 덮어쓴다. 한쪽이라도 관측한 뒤에는
    // 연결이 끊겨도 nominal 주행으로 되돌아가지 않고 fail-closed한다.
    recovery_require_supervisor_ = this->declare_parameter<bool>(
      "recovery/require_supervisor", false);
    recovery_command_timeout_ = this->declare_parameter<double>(
      "recovery/command_timeout_sec", 0.25);
    recovery_pair_max_skew_ = this->declare_parameter<double>(
      "recovery/pair_max_skew_sec", 0.10);
    recovery_allow_reverse_ = this->declare_parameter<bool>(
      "recovery/allow_reverse", false);
    recovery_max_reverse_speed_ = this->declare_parameter<double>(
      "recovery/max_reverse_speed_mps", 0.30);
    recovery_max_forward_override_ = this->declare_parameter<double>(
      "recovery/max_forward_override_mps", 1.0);

    // Parameters from the supplied ETH Forza MAP_Controller.
    t_clip_min_ = this->declare_parameter<double>("map/t_clip_min", 0.7);
    t_clip_max_ = this->declare_parameter<double>("map/t_clip_max", 8.0);
    m_l1_ = this->declare_parameter<double>("map/m_l1", 0.47);
    q_l1_ = this->declare_parameter<double>("map/q_l1", -0.2);
    curvature_factor_ = this->declare_parameter<double>("map/curvature_factor", 0.0);
    speed_lookahead_ = this->declare_parameter<double>("map/speed_lookahead", 0.25);
    // Keep the ETH speed correction available, but disable it for initial
    // steering validation so planner target speed passes through unchanged.
    lat_err_coeff_ = this->declare_parameter<double>("map/lat_err_coeff", 0.0);
    acc_scaler_for_steer_ = this->declare_parameter<double>(
      "map/acc_scaler_for_steer", 1.0);
    dec_scaler_for_steer_ = this->declare_parameter<double>(
      "map/dec_scaler_for_steer", 1.0);
    start_scale_speed_ = this->declare_parameter<double>("map/start_scale_speed", 6.5);
    end_scale_speed_ = this->declare_parameter<double>("map/end_scale_speed", 10.0);
    downscale_factor_ = this->declare_parameter<double>("map/downscale_factor", 0.5);
    speed_lookahead_for_steer_ = this->declare_parameter<double>(
      "map/speed_lookahead_for_steer", 0.0);
    max_lateral_error_ = this->declare_parameter<double>("map/max_lateral_error_m", 0.5);
    // 추종 실패 백스톱 (근거는 사용처 주석 참조).
    backstop_enabled_ = this->declare_parameter<bool>(
      "map/backstop_enabled", true);
    backstop_err_m_ = this->declare_parameter<double>(
      "map/backstop_path_error_m", 0.25);
    backstop_sat_ratio_ = this->declare_parameter<double>(
      "map/backstop_steer_sat_ratio", 0.85);
    backstop_min_speed_ = this->declare_parameter<double>(
      "map/backstop_min_speed_mps", 1.5);
    backstop_sat_min_err_m_ = this->declare_parameter<double>(
      "map/backstop_sat_min_path_error_m", 0.12);
    backstop_steering_reference_angle_ = this->declare_parameter<double>(
      "map/backstop_steering_reference_rad", 0.3578);
    max_curvature_reference_ = this->declare_parameter<double>(
      "map/max_curvature_reference", 0.8);
    velocity_steer_gain_max_ = this->declare_parameter<double>(
      "map/velocity_steer_gain_max", 1.0);
    nearest_search_horizon_ = this->declare_parameter<double>(
      "map/nearest_search_horizon_m", 3.0);
    // L1 거리를 실측 속도와 경로 계획 속도 중 **긴 쪽**으로 잡는다.
    // false 로 두면 종전(실측 전용) 동작 — A/B 용으로만 끌 것.
    l1_speed_uses_path_plan_ = this->declare_parameter<bool>(
      "map/l1_speed_uses_path_plan", true);

    // Vehicle limits and the only non-ETH speed multiplier.
    wheelbase_ = this->declare_parameter<double>("vehicle/wheelbase_m", 0.33);
    max_speed_ = this->declare_parameter<double>("vehicle/max_speed_mps", 4.0);
    max_left_steering_angle_ = this->declare_parameter<double>(
      "vehicle/max_left_steering_angle_rad", 0.4189);
    max_right_steering_angle_ = this->declare_parameter<double>(
      "vehicle/max_right_steering_angle_rad", 0.4189);
    // Conservative initial value. Replace with a measured steering-system rate
    // before increasing real-car speed.
    max_steering_rate_ = this->declare_parameter<double>(
      "vehicle/max_steering_rate_radps", 2.0);
    max_acceleration_ = this->declare_parameter<double>(
      "vehicle/max_accel_mps2", 5.5);
    max_deceleration_ = this->declare_parameter<double>(
      "vehicle/max_decel_mps2", 7.0);
    speed_scale_ = this->declare_parameter<double>("vehicle/speed_scale", 1.0);

    validate_parameters();
    RCLCPP_INFO(
      this->get_logger(),
      "Steering limits: left=%.4frad right=%.4frad, backstop reference=%.4frad",
      max_left_steering_angle_, max_right_steering_angle_,
      backstop_steering_reference_angle_);
    load_steering_lookup();

    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    waypoint_sub_ = this->create_subscription<f1tenth_planning_msgs::msg::WpntArray>(
      waypoint_topic, reliable_qos,
      std::bind(&ControlNode::waypoint_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, reliable_qos,
      std::bind(&ControlNode::odom_callback, this, std::placeholders::_1));
    frenet_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      frenet_odom_topic, reliable_qos,
      std::bind(&ControlNode::frenet_odom_callback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, reliable_qos,
      std::bind(&ControlNode::imu_callback, this, std::placeholders::_1));
    estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      estop_topic, reliable_qos,
      std::bind(&ControlNode::estop_callback, this, std::placeholders::_1));
    escape_active_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/planner/escape_active", reliable_qos,
      std::bind(&ControlNode::escape_active_callback, this, std::placeholders::_1));
    if (localization_gate_enabled_) {
      const auto localization_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
      localization_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        localization_ready_topic, localization_qos,
        std::bind(
          &ControlNode::localization_ready_callback, this,
          std::placeholders::_1));
    }
    recovery_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      recovery_state_topic, reliable_qos,
      std::bind(&ControlNode::recovery_state_callback, this, std::placeholders::_1));
    recovery_target_speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      recovery_target_speed_topic, reliable_qos,
      std::bind(
        &ControlNode::recovery_target_speed_callback, this, std::placeholders::_1));

    command_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      output_topic, reliable_qos);
    lookahead_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/control/lookahead_point", reliable_qos);
    mode_service_ = this->create_service<example_interfaces::srv::Trigger>(
      service_name,
      std::bind(
        &ControlNode::set_mode_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    control_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_),
      std::bind(&ControlNode::main_loop, this));

    RCLCPP_INFO(
      this->get_logger(),
      "ETH MAP controller ready: %s + %s -> %s @ %.1f Hz, autonomous=%s",
      waypoint_topic.c_str(), odom_topic.c_str(), output_topic.c_str(), control_rate_,
      autonomous_ ? "ON" : "OFF");
    RCLCPP_INFO(
      this->get_logger(),
      "Recovery lease: required=%s timeout=%.2fs reverse=%s max_reverse=%.2fm/s",
      recovery_require_supervisor_ ? "yes" : "no", recovery_command_timeout_,
      recovery_allow_reverse_ ? "OPT-IN" : "OFF", recovery_max_reverse_speed_);
    RCLCPP_INFO(
      this->get_logger(),
      "Localization autonomy gate: %s topic=%s timeout=%.2fs",
      localization_gate_enabled_ ? "ENABLED" : "disabled",
      localization_ready_topic.c_str(), localization_ready_timeout_);
    RCLCPP_INFO(
      this->get_logger(),
      "Unicorn L1 curvature preview: waypoint offsets [10, 20), factor=%.3f",
      curvature_factor_);
  }

private:
  void validate_parameters()
  {
    const std::array<double, 41> finite_values = {
      control_rate_, waypoint_timeout_, odom_timeout_, frenet_odom_timeout_,
      imu_acceleration_sign_, source_stamp_future_tolerance_,
      localization_ready_timeout_,
      recovery_command_timeout_, recovery_pair_max_skew_,
      recovery_max_reverse_speed_, recovery_max_forward_override_,
      t_clip_min_, t_clip_max_, m_l1_, q_l1_, curvature_factor_, speed_lookahead_,
      lat_err_coeff_, acc_scaler_for_steer_, dec_scaler_for_steer_,
      start_scale_speed_, end_scale_speed_, downscale_factor_,
      speed_lookahead_for_steer_, max_lateral_error_,
      backstop_err_m_, backstop_sat_ratio_, backstop_min_speed_,
      backstop_sat_min_err_m_, backstop_steering_reference_angle_,
      max_curvature_reference_, velocity_steer_gain_max_,
      nearest_search_horizon_, wheelbase_, max_speed_,
      max_left_steering_angle_, max_right_steering_angle_,
      max_steering_rate_, max_acceleration_, max_deceleration_, speed_scale_};
    if (!std::all_of(
        finite_values.begin(), finite_values.end(),
        [](double value) {return std::isfinite(value);}) ||
      control_rate_ < 1.0 || control_rate_ > 200.0 ||
      waypoint_timeout_ < 0.01 || waypoint_timeout_ > 2.0 ||
      odom_timeout_ < 0.01 || odom_timeout_ > 2.0 ||
      frenet_odom_timeout_ < 0.01 || frenet_odom_timeout_ > 2.0 ||
      source_stamp_future_tolerance_ < 0.0 ||
      source_stamp_future_tolerance_ > 0.5 ||
      localization_ready_timeout_ < 0.05 || localization_ready_timeout_ > 2.0 ||
      recovery_command_timeout_ < 0.01 || recovery_command_timeout_ > 2.0 ||
      recovery_pair_max_skew_ < 0.0 ||
      recovery_pair_max_skew_ > recovery_command_timeout_ ||
      recovery_max_reverse_speed_ < 0.0 || recovery_max_reverse_speed_ > 1.0 ||
      recovery_max_forward_override_ < 0.0 ||
      recovery_max_forward_override_ > 3.0 ||
      t_clip_min_ < 0.05 || t_clip_max_ < t_clip_min_ || t_clip_max_ > 20.0 ||
      m_l1_ <= 0.0 || m_l1_ > 5.0 || q_l1_ < -5.0 || q_l1_ > 10.0 ||
      curvature_factor_ < 0.0 || curvature_factor_ > 5.0 ||
      speed_lookahead_ < 0.0 || speed_lookahead_ > 2.0 ||
      speed_lookahead_for_steer_ < 0.0 || speed_lookahead_for_steer_ > 2.0 ||
      lat_err_coeff_ < 0.0 || lat_err_coeff_ > 1.0 ||
      acc_scaler_for_steer_ < 0.0 || acc_scaler_for_steer_ > 5.0 ||
      dec_scaler_for_steer_ < 0.0 || dec_scaler_for_steer_ > 5.0 ||
      start_scale_speed_ < 0.0 || end_scale_speed_ <= start_scale_speed_ ||
      end_scale_speed_ > 30.0 ||
      downscale_factor_ < 0.0 || downscale_factor_ > 1.0 ||
      max_lateral_error_ <= 0.0 || max_lateral_error_ > 5.0 ||
      backstop_err_m_ <= 0.0 || backstop_err_m_ > 5.0 ||
      backstop_sat_ratio_ <= 0.0 || backstop_sat_ratio_ > 1.0 ||
      backstop_min_speed_ < 0.0 || backstop_min_speed_ > 10.0 ||
      // 0 = 비활성(포화 단독 발화). 경로오차 임계 이상으로 두면 이 분기가
      // 경로오차 분기에 흡수돼 무의미해지므로 그 위는 막는다.
      backstop_sat_min_err_m_ < 0.0 ||
      backstop_sat_min_err_m_ >= backstop_err_m_ ||
      backstop_steering_reference_angle_ <= 0.0 ||
      backstop_steering_reference_angle_ > 0.8 ||
      backstop_steering_reference_angle_ >
      std::min(max_left_steering_angle_, max_right_steering_angle_) ||
      max_curvature_reference_ <= 0.0 || max_curvature_reference_ > 5.0 ||
      velocity_steer_gain_max_ < 1.0 || velocity_steer_gain_max_ > 3.0 ||
      nearest_search_horizon_ < 0.5 || nearest_search_horizon_ > 20.0 ||
      wheelbase_ < 0.1 || wheelbase_ > 1.0 ||
      max_speed_ < 0.0 || max_speed_ > 20.0 ||
      max_left_steering_angle_ <= 0.0 || max_left_steering_angle_ > 0.8 ||
      max_right_steering_angle_ <= 0.0 || max_right_steering_angle_ > 0.8 ||
      max_steering_rate_ <= 0.0 || max_steering_rate_ > 10.0 ||
      max_acceleration_ <= 0.0 || max_acceleration_ > 20.0 ||
      max_deceleration_ <= 0.0 || max_deceleration_ > 20.0 ||
      speed_scale_ < 0.0 || speed_scale_ > 1.0)
    {
      throw std::runtime_error(
              "Invalid/non-finite control safety parameter; refusing to command vehicle");
    }
  }

  void load_steering_lookup()
  {
    auto lookup_file = this->declare_parameter<std::string>("map/steering_lut_file", "");
    if (lookup_file.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "map/steering_lut_file is empty; using the bicycle-model inverse until a "
        "vehicle-specific ETH steering LUT is supplied");
      return;
    }

    if (lookup_file.front() != '/') {
      try {
        lookup_file = ament_index_cpp::get_package_share_directory("control_pkg") +
          "/config/steering_lookup/" + lookup_file;
      } catch (const std::exception & exception) {
        RCLCPP_ERROR(
          this->get_logger(), "Could not resolve control_pkg share directory: %s",
          exception.what());
        throw std::runtime_error(
                std::string("Could not resolve steering LUT package path: ") +
                exception.what());
      }
    }
    if (!steer_lookup_.load(lookup_file, this->get_logger())) {
      throw std::runtime_error(
              "Configured steering LUT is missing or malformed: " + lookup_file);
    }
  }

  void waypoint_callback(const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    waypoints_ = msg->wpnts;
    waypoint_frame_ = msg->header.frame_id;
    waypoint_stamp_ = this->now();
    waypoint_source_stamp_ =
      rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    odom_ = *msg;
    odom_stamp_ = this->now();
    odom_source_stamp_ =
      rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
    has_odom_ = true;
  }

  void frenet_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    frenet_odom_ = *msg;
    frenet_odom_stamp_ = this->now();
    frenet_odom_source_stamp_ =
      rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
    has_frenet_odom_ = true;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (std::size_t index = acceleration_samples_.size() - 1; index > 0; --index) {
      acceleration_samples_[index] = acceleration_samples_[index - 1];
    }
    acceleration_samples_[0] = imu_acceleration_sign_ * msg->linear_acceleration.x;
  }

  // 플래너가 원호 이탈(BuildEscapeArc) 중이라고 알려 오는 신호.
  // 그 기동은 설계상 조향이 포화 근처로 가므로, 백스톱의 **조향포화 분기**를
  // 그대로 두면 감속으로 회피와 싸운다. 경로오차 분기는 그대로 살린다 —
  // 그건 "계획을 못 따라가고 있다"는 뜻이라 원호 중에도 유효한 신호다.
  // stamp 를 같이 저장해 신호가 끊기면 자동으로 원복한다 (fail-closed).
  void escape_active_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    escape_active_ = msg->data;
    escape_active_stamp_ = this->now();
  }

  void estop_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      estop_ = msg->data;
    }
    if (msg->data) {
      publish_stop();
    }
  }

  void localization_ready_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      localization_ready_ = msg->data;
      localization_ready_stamp_ = this->now();
      has_localization_ready_ = true;
    }
    if (!msg->data) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Localization not ready -> autonomy stop (teleop/estop remain available)");
      publish_stop();
    }
  }

  void recovery_state_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    recovery_state_ = msg->data;
    recovery_state_stamp_ = this->now();
    has_recovery_state_ = true;
  }

  void recovery_target_speed_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    recovery_target_speed_ = msg->data;
    recovery_target_speed_stamp_ = this->now();
    has_recovery_target_speed_ = true;
  }

  void set_mode_callback(
    const std::shared_ptr<example_interfaces::srv::Trigger::Request> /*request*/,
    std::shared_ptr<example_interfaces::srv::Trigger::Response> response)
  {
    autonomous_ = !autonomous_;
    curr_steering_angle_ = 0.0;
    last_control_time_.reset();
    if (!autonomous_) {
      publish_stop();
    }
    response->success = true;
    response->message = autonomous_ ? "autonomous ON" : "autonomous OFF";
    RCLCPP_WARN(this->get_logger(), "Control mode: %s", response->message.c_str());
  }

  bool input_stamp_fresh(
    const rclcpp::Time & now,
    const rclcpp::Time & receipt_stamp,
    const rclcpp::Time & source_stamp,
    double timeout_sec) const
  {
    const auto valid_age = [&](const rclcpp::Time & stamp) {
        if (stamp.nanoseconds() <= 0) {
          return false;
        }
        const double age = (now - stamp).seconds();
        return std::isfinite(age) &&
               age >= -source_stamp_future_tolerance_ &&
               age <= timeout_sec;
      };
    if (!valid_age(receipt_stamp)) {
      return false;
    }
    if (source_stamp.nanoseconds() <= 0) {
      return !require_source_stamps_;
    }
    return valid_age(source_stamp);
  }

  bool inputs_finite(
    const std::vector<f1tenth_planning_msgs::msg::Wpnt> & waypoints,
    const nav_msgs::msg::Odometry & odom,
    const nav_msgs::msg::Odometry & frenet_odom) const
  {
    const auto & p = odom.pose.pose.position;
    const auto & q = odom.pose.pose.orientation;
    const auto & v = odom.twist.twist.linear;
    const auto & fp = frenet_odom.pose.pose.position;
    const auto & fv = frenet_odom.twist.twist.linear;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
      !std::isfinite(q.x) || !std::isfinite(q.y) ||
      !std::isfinite(q.z) || !std::isfinite(q.w) ||
      !std::isfinite(v.x) || !std::isfinite(v.y) ||
      !std::isfinite(fp.x) || !std::isfinite(fp.y) ||
      !std::isfinite(fv.x) || !std::isfinite(fv.y))
    {
      return false;
    }
    const double quaternion_norm =
      q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(quaternion_norm) || quaternion_norm < kEpsilon) {
      return false;
    }
    for (const auto & waypoint : waypoints) {
      if (!std::isfinite(waypoint.x_m) || !std::isfinite(waypoint.y_m) ||
        !std::isfinite(waypoint.vx_mps) ||
        !std::isfinite(waypoint.kappa_radpm))
      {
        return false;
      }
    }
    return true;
  }

  void main_loop()
  {
    std::vector<f1tenth_planning_msgs::msg::Wpnt> waypoints;
    nav_msgs::msg::Odometry odom;
    nav_msgs::msg::Odometry frenet_odom;
    std::array<double, 10> acceleration_samples;
    std::string waypoint_frame;
    rclcpp::Time waypoint_stamp(0, 0, RCL_ROS_TIME);
    rclcpp::Time odom_stamp(0, 0, RCL_ROS_TIME);
    rclcpp::Time frenet_odom_stamp(0, 0, RCL_ROS_TIME);
    rclcpp::Time waypoint_source_stamp(0, 0, RCL_ROS_TIME);
    rclcpp::Time odom_source_stamp(0, 0, RCL_ROS_TIME);
    rclcpp::Time frenet_odom_source_stamp(0, 0, RCL_ROS_TIME);
    rclcpp::Time recovery_state_stamp(0, 0, RCL_ROS_TIME);
    rclcpp::Time recovery_target_speed_stamp(0, 0, RCL_ROS_TIME);
    rclcpp::Time localization_ready_stamp(0, 0, RCL_ROS_TIME);
    std::string recovery_state;
    double recovery_target_speed = std::numeric_limits<double>::quiet_NaN();
    bool has_odom = false;
    bool has_frenet_odom = false;
    bool has_recovery_state = false;
    bool has_recovery_target_speed = false;
    bool has_localization_ready = false;
    bool localization_ready = false;
    bool estop = false;
    bool escape_active = false;
    rclcpp::Time escape_active_stamp{0, 0, RCL_ROS_TIME};
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      waypoints = waypoints_;
      odom = odom_;
      frenet_odom = frenet_odom_;
      acceleration_samples = acceleration_samples_;
      waypoint_frame = waypoint_frame_;
      waypoint_stamp = waypoint_stamp_;
      odom_stamp = odom_stamp_;
      frenet_odom_stamp = frenet_odom_stamp_;
      waypoint_source_stamp = waypoint_source_stamp_;
      odom_source_stamp = odom_source_stamp_;
      frenet_odom_source_stamp = frenet_odom_source_stamp_;
      has_odom = has_odom_;
      has_frenet_odom = has_frenet_odom_;
      recovery_state = recovery_state_;
      recovery_target_speed = recovery_target_speed_;
      recovery_state_stamp = recovery_state_stamp_;
      recovery_target_speed_stamp = recovery_target_speed_stamp_;
      has_recovery_state = has_recovery_state_;
      has_recovery_target_speed = has_recovery_target_speed_;
      localization_ready_stamp = localization_ready_stamp_;
      has_localization_ready = has_localization_ready_;
      localization_ready = localization_ready_;
      estop = estop_;
      escape_active = escape_active_;
      escape_active_stamp = escape_active_stamp_;
    }

    const auto now = this->now();
    // E-stop은 recovery와 autonomous mode를 포함한 모든 명령보다 최우선이다.
    if (estop) {
      publish_stop();
      return;
    }

    if (localization_gate_enabled_) {
      double status_age = std::numeric_limits<double>::infinity();
      if (has_localization_ready && localization_ready_stamp.nanoseconds() > 0) {
        status_age = (now - localization_ready_stamp).seconds();
      }
      const bool status_fresh =
        std::isfinite(status_age) && status_age >= 0.0 &&
        status_age <= localization_ready_timeout_;
      if (!has_localization_ready || !localization_ready || !status_fresh) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Localization autonomy gate blocked: received=%s ready=%s age=%.3fs",
          has_localization_ready ? "yes" : "no",
          localization_ready ? "true" : "false", status_age);
        publish_stop();
        return;
      }
    }

    bool recovery_stop = false;
    bool recovery_reverse = false;
    bool recovery_forward_override = false;
    const bool recovery_contract_active =
      recovery_require_supervisor_ || has_recovery_state || has_recovery_target_speed;
    if (recovery_contract_active) {
      const auto lease_fresh = [&](const rclcpp::Time & stamp) {
          if (stamp.nanoseconds() <= 0) {
            return false;
          }
          const double age = (now - stamp).seconds();
          return std::isfinite(age) &&
                 age >= -source_stamp_future_tolerance_ &&
                 age <= recovery_command_timeout_;
        };
      const bool pair_complete = has_recovery_state && has_recovery_target_speed;
      const bool pair_fresh =
        pair_complete && lease_fresh(recovery_state_stamp) &&
        lease_fresh(recovery_target_speed_stamp) &&
        std::abs((recovery_state_stamp - recovery_target_speed_stamp).seconds()) <=
        recovery_pair_max_skew_;

      if (!pair_fresh) {
        recovery_stop = true;
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Recovery command pair missing/stale/skewed -> stop");
      } else if (
        recovery_state == "WAITING_FOR_INPUTS" ||
        recovery_state == "STUCK_DETECTED" ||
        recovery_state == "COLLISION_STOP" ||
        recovery_state == "RECOVER_STOP" ||
        recovery_state == "STOP")
      {
        // State alone is sufficient to stop. A malformed target cannot weaken it.
        recovery_stop = true;
        if (!std::isfinite(recovery_target_speed)) {
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Recovery stop state has non-finite target (still stopping)");
        }
      } else if (recovery_state == "DRIVING") {
        // NaN is an intentional, state-qualified sentinel meaning "no override".
        // +/-inf or a finite target in DRIVING indicates an incoherent pair.
        if (!std::isnan(recovery_target_speed)) {
          recovery_stop = true;
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Recovery DRIVING state must carry NaN no-override target -> stop");
        }
      } else if (recovery_state == "RECOVER_REVERSE") {
        if (!recovery_allow_reverse_ || !std::isfinite(recovery_target_speed) ||
          recovery_target_speed >= 0.0 || recovery_max_reverse_speed_ <= 0.0)
        {
          recovery_stop = true;
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Recovery reverse rejected (not opted-in/invalid target) -> stop");
        } else {
          recovery_reverse = true;
        }
      } else if (recovery_state == "RECOVER_ALIGN") {
        if (!std::isfinite(recovery_target_speed) || recovery_target_speed < 0.0) {
          recovery_stop = true;
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Recovery align target invalid -> stop");
        } else {
          recovery_forward_override = true;
        }
      } else {
        recovery_stop = true;
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Unknown recovery state '%s' -> stop", recovery_state.c_str());
      }
    }

    if (recovery_stop) {
      publish_stop();
      return;
    }
    if (!autonomous_) {
      publish_stop();
      return;
    }

    if (waypoints.size() < 2 ||
      !input_stamp_fresh(
        now, waypoint_stamp, waypoint_source_stamp, waypoint_timeout_))
    {
      warn_and_stop("Local waypoints missing or receipt/source stale");
      return;
    }
    if (!has_odom ||
      !input_stamp_fresh(now, odom_stamp, odom_source_stamp, odom_timeout_))
    {
      warn_and_stop("Map odometry missing or receipt/source stale");
      return;
    }
    if (!has_frenet_odom ||
      !input_stamp_fresh(
        now, frenet_odom_stamp, frenet_odom_source_stamp,
        frenet_odom_timeout_))
    {
      warn_and_stop("Frenet odometry missing or receipt/source stale");
      return;
    }
    if (!inputs_finite(waypoints, odom, frenet_odom)) {
      warn_and_stop("Planner/odometry input contains non-finite values");
      return;
    }

    if (recovery_reverse) {
      // 전용 reverse path/steering 계약은 아직 없다. 양쪽 노드에서 명시적으로
      // opt-in된 경우에도 forward L1 조향을 역주행에 재사용하지 않고 직진
      // 저속 후진만 허용한다. Supervisor가 매 cycle 경계와 후방을 재검사한다.
      const double reverse_speed = -std::min(
        std::abs(recovery_target_speed), recovery_max_reverse_speed_);
      if (!std::isfinite(reverse_speed) || !(reverse_speed < 0.0)) {
        warn_and_stop("Recovery reverse command is invalid");
        return;
      }
      curr_steering_angle_ = 0.0;
      last_speed_command_ = 0.0;
      last_control_time_.reset();
      ackermann_msgs::msg::AckermannDriveStamped command;
      command.header.stamp = now;
      command.header.frame_id = "base_link";
      command.drive.speed = static_cast<float>(reverse_speed);
      command.drive.acceleration = 0.0F;
      command.drive.jerk = 0.0F;
      command.drive.steering_angle = 0.0F;
      command_pub_->publish(command);
      return;
    }

    const Point2D position{odom.pose.pose.position.x, odom.pose.pose.position.y};
    const double yaw = yaw_from_quaternion(odom.pose.pose.orientation);
    const double speed_now = odom.twist.twist.linear.x;
    const Point2D velocity{
      std::cos(yaw) * speed_now,
      std::sin(yaw) * speed_now};

    const double nominal_dt = 1.0 / control_rate_;
    double control_dt = nominal_dt;
    if (last_control_time_.has_value()) {
      const double measured_dt = (now - *last_control_time_).seconds();
      if (std::isfinite(measured_dt) && measured_dt > 0.0) {
        // Bound timer anomalies while retaining normal scheduling jitter.
        control_dt = std::clamp(measured_dt, 0.25 * nominal_dt, 4.0 * nominal_dt);
      }
    }
    last_control_time_ = now;

    const auto lateral_error_result = calc_lateral_error_norm(frenet_odom);
    const double lat_error_norm = lateral_error_result.first;
    const double lateral_error = lateral_error_result.second;

    // LONGITUDINAL CONTROL: same order as MAP_Controller.main_loop().
    double speed_command = calc_speed_command(
      position, velocity, waypoints, lat_error_norm);

    // LATERAL CONTROL: L1 point -> lateral acceleration -> steering lookup.
    const auto l1_result = calc_l1_point(
      position, waypoints, speed_now, lateral_error, lat_error_norm);
    const Point2D l1_point = l1_result.first;
    const double l1_distance = l1_result.second;
    double steering_angle = calc_steering_angle(
      position, l1_point, l1_distance, yaw, lat_error_norm, velocity,
      waypoints, speed_now, acceleration_samples, control_dt);

    if (!std::isfinite(speed_command) || !std::isfinite(steering_angle)) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "MAP controller produced a non-finite command -> stop");
      publish_stop();
      return;
    }

    if (recovery_forward_override) {
      speed_command = std::clamp(
        recovery_target_speed, 0.0,
        std::min(max_speed_, recovery_max_forward_override_));
    } else {
      speed_command = std::clamp(speed_command * speed_scale_, 0.0, max_speed_);
      // ── 추종 실패 백스톱 ────────────────────────────────────────────────
      // 플래너의 속도 계획은 smoothstep 기하 + 가정한 그립(a_lat)으로 나온다.
      // 그 모델이 틀리면(먼지/타이어 온도/배터리 새그, 또는 계획 자체의
      // 오버슛) 계획은 성립하는데 차가 못 따라간다. rosbag_0806 run_235014
      // 42~44s가 그 경우다 — 계획 apex는 d=-0.59였는데 차는 d=-1.16까지
      // 밀려 우벽(-0.80)을 0.26m 관통했다. 추종 오차 0.57m.
      //
      // 트리거를 조향각(>10deg)으로 잡으면 안 된다: 같은 4런에서 장애물이
      // 없을 때도 27~32% 발화하고(레이스라인 기하만으로 10deg를 넘는 구간은
      // 9%뿐인데 pure pursuit 룩어헤드+추종오차가 나머지를 만든다), 정작
      // 35초 정지가 난 run_234734에서는 직선 제동이라 조향이 내내 0이었다.
      //
      // 실제로 앞서는 신호는 **경로 추종 오차**다. 위 구간 실측:
      //   41.43s  조향 0.00deg  오차 0.22   <- 이미 벌어지는 중
      //   41.62s  조향 0.00deg  오차 0.40
      //   41.84s  조향 11.5deg  오차 0.67   <- 조향 트리거는 여기서야 발화
      //   42.42s  조향 15.2deg  오차 1.17   <- 벽 관통
      // 오차가 조향보다 0.4s / 0.45m 앞선다.
      //
      // 주의: 여기서 쓰는 오차는 raceline 기준 d(frenet_odom.y)가 아니라
      // **지금 따라가는 로컬 경로**까지의 거리다. 회피 중에는 d가 커지는 게
      // 정상이므로 d로 트리거하면 회피할 때마다 오발화한다.
      if (backstop_enabled_) {
        const std::size_t near_i = nearest_waypoint(position, waypoints);
        const double path_err = std::hypot(
          position.x - waypoints[near_i].x_m,
          position.y - waypoints[near_i].y_m);
        const double sat = std::abs(steering_angle) /
          std::max(backstop_steering_reference_angle_, 1e-3);
        double backstop_cap = max_speed_;
        if (std::isfinite(path_err) && path_err > backstop_err_m_) {
          // 오차가 임계를 넘은 만큼 선형으로 감속. 오차가 임계의 2배면
          // 하한까지 내려간다.
          const double over = std::min(
            (path_err - backstop_err_m_) / std::max(backstop_err_m_, 1e-3), 1.0);
          backstop_cap = std::min(
            backstop_cap, speed_now - over * (speed_now - backstop_min_speed_));
        }
        // 원호 이탈 중이면 조향포화 분기를 건너뛴다. 이 기동은 "조향을 최대로
        // 쓰는 것"이 목적이라, 포화를 감속 사유로 읽으면 회피와 정면으로
        // 싸운다 (플래너는 회피를 계획하고 컨트롤러는 그 속도를 깎는다).
        // 신호가 끊기면 즉시 원복 — 게이팅은 fail-closed 여야 한다.
        const bool escape_now = escape_active &&
          (now - escape_active_stamp).seconds() < kEscapeActiveTimeoutSec;
        // ── 2026-08-12: 조향포화 분기에 경로오차 조건을 AND 로 묶는다 ──────
        // 이 분기의 전제는 "조향이 포화면 기하를 못 만들고 있다"이지만,
        // **추종이 완벽한데도** 계획된 레이스라인 자체가 큰 조향을 요구하면
        // 포화만 뜬다. 그때 깎는 것은 방어가 아니라 순손실이다.
        //   run_0812_013630 실측 오발화 12건 (경로오차 2~10cm = 추종 완벽):
        //     경로오차 0.06m / 포화 99% -> 3.80 -> 1.54 m/s  (-59%)
        //     경로오차 0.10m / 포화 92% -> 3.61 -> 2.53      (-30%)
        //     경로오차 0.07m / 포화 93% -> 3.05 -> 2.28      (-25%)
        //     경로오차 0.02m / 포화 90% -> 2.95 -> 2.62      (-11%)
        //   (0808 run_202157 의 apex 5곳 오발화와 같은 패턴이 램프 전환 후에도
        //    남아 있었다 — 램프가 낙폭을 줄였을 뿐 발화 자체는 그대로다.)
        // 포화가 **실제 추종 실패로 나타나기 시작할 때**만 개입하게 한다.
        // 임계는 경로오차 분기(backstop_path_error_m 0.25)의 절반 아래로 둬서
        // 그 분기보다 항상 먼저 열리게 한다 — 포화는 선행 지표로 남는다.
        //   ⚠️ 이 값을 0 으로 두면 옛 동작(포화 단독 발화)으로 돌아간다.
        //   ⚠️ 반대로 backstop_path_error_m 이상으로 올리면 이 분기가 경로오차
        //      분기에 완전히 흡수돼 사실상 비활성이 된다.
        const bool sat_err_ok =
          !std::isfinite(backstop_sat_min_err_m_) ||
          backstop_sat_min_err_m_ <= 0.0 ||
          (std::isfinite(path_err) && path_err > backstop_sat_min_err_m_);
        if (sat > backstop_sat_ratio_ && !escape_now && sat_err_ok) {
          // 조향이 포화면 기하를 못 만들고 있다. 다만 경로오차 분기와 같은
          // **비례 램프**로 깎는다. 0808 run_202157에서 하한 즉시 캡은
          // 오발화했다: apex 5곳에서 경로오차가 3~7cm(= 추종은 완벽)인데
          // 계획된 레이스라인이 20°를 요구해 포화만 87~96%로 떴고, 그때마다
          // 2.2~2.5 m/s를 1.5로 통째로 잘랐다. 145s 중 109s가 발화 상태였고
          // 그중 87%가 하한 클램프였다.
          //
          // 절벽이 특히 해로웠던 이유는 **이미 하한 근처인지를 무시**했기
          // 때문이다. 6.5에서 자르는 것과 2.2에서 자르는 것을 같게 취급했다.
          // 낙폭을 (speed_now - 하한)에 비례시키면 저속에서 감속이 자연히
          // 소멸한다. 위 5개 apex 기준 평균 낙폭 0.74 -> 0.34 m/s.
          //
          // 부수 효과로 해제도 연속이 된다. 절벽일 때는 sat이 임계를 스치기만
          // 해도 relay처럼 on/off가 뒤집혀 헤어핀에서 1.4s를 1.50에 붙어 있다
          // 풀리자마자 1.5 -> 6.5로 튀었다. 램프에는 되돌아갈 계단이 없다.
          //
          // 좌우 물리 한계와 별개인 보수적 reference 로 포화율을 계산한다.
          // 좌측 추가 조향을 쓰면 sat이 1.0을 넘을 수 있지만 over_sat을 1.0으로
          // 제한하므로 감속 램프는 기존 최대 강도 이상으로 커지지 않는다.
          //
          // 주의: 램프가 좁을수록(임계가 1.0에 가까울수록) 게인이 커진다.
          // LUT가 speed_now를 받아 조향각을 내므로 감속 -> 요구조향 감소 ->
          // sat 감소의 폐루프가 있다. apex 속도가 떨면 임계를 **낮춰서**
          // 램프를 넓힐 것 — 직관과 반대 방향이다.
          const double over_sat = std::min(
            (sat - backstop_sat_ratio_) /
            std::max(1.0 - backstop_sat_ratio_, 1e-3), 1.0);
          backstop_cap = std::min(
            backstop_cap, speed_now - over_sat * (speed_now - backstop_min_speed_));
        }
        // 하한을 두는 이유: BlendDist의 조향 기하 하한 때문에 약 2.4 m/s
        // 아래로는 필요 조주거리가 더 이상 줄지 않는다. 무한정 깎으면 회피는
        // 안 쉬워지고 조우 시간만 길어져 정지 라치 영역으로 들어간다.
        backstop_cap = std::max(backstop_cap, backstop_min_speed_);
        if (backstop_cap < speed_command) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "추종 백스톱: 경로오차 %.2fm / 조향포화 %.0f%% -> 속도 %.2f -> %.2f",
            path_err, 100.0 * sat, speed_command, backstop_cap);
          speed_command = backstop_cap;
        }
      }
    }
    // 정상 TRACK/ALIGN 명령은 물리 가감속 한계 안에서만 바뀐다. E-stop,
    // stale, recovery STOP은 위에서 publish_stop()으로 이 limiter를 우회해 즉시 0.
    const double previous_speed_command = last_speed_command_;
    const double speed_delta = speed_command - previous_speed_command;
    const double max_speed_delta = (
      speed_delta >= 0.0 ? max_acceleration_ : max_deceleration_) * control_dt;
    speed_command = previous_speed_command +
      std::clamp(speed_delta, -max_speed_delta, max_speed_delta);
    if (recovery_forward_override) {
      // Supervisor의 저속 align 상한은 감속 slew보다 우선한다. 정상 상태에서
      // RECOVER_STOP을 거치지만, 중간 메시지를 놓쳐도 이전 고속 명령이 남지 않는다.
      speed_command = std::min(
        speed_command, std::min(max_speed_, recovery_max_forward_override_));
    }
    const double target_acceleration = std::clamp(
      (speed_command - previous_speed_command) / control_dt,
      -max_deceleration_, max_acceleration_);
    last_speed_command_ = speed_command;
    steering_angle = std::clamp(
      steering_angle, -max_right_steering_angle_, max_left_steering_angle_);

    ackermann_msgs::msg::AckermannDriveStamped command;
    command.header.stamp = now;
    command.header.frame_id = "base_link";
    command.drive.speed = static_cast<float>(speed_command);
    command.drive.acceleration = static_cast<float>(target_acceleration);
    command.drive.jerk = 0.0F;
    command.drive.steering_angle = static_cast<float>(steering_angle);
    command_pub_->publish(command);

    geometry_msgs::msg::PointStamped lookahead;
    lookahead.header.stamp = now;
    lookahead.header.frame_id = waypoint_frame.empty() ? odom.header.frame_id : waypoint_frame;
    lookahead.point.x = l1_point.x;
    lookahead.point.y = l1_point.y;
    lookahead_pub_->publish(lookahead);
  }

  std::pair<double, double> calc_lateral_error_norm(
    const nav_msgs::msg::Odometry & frenet_odom) const
  {
    // Original implementation: lateral_error = abs(position_in_map_frenet[1]).
    const double lateral_error = std::abs(frenet_odom.pose.pose.position.y);
    const double clipped_error = std::clamp(lateral_error, 0.0, max_lateral_error_);
    const double lat_error_norm = 0.5 * clipped_error / max_lateral_error_;
    return {lat_error_norm, lateral_error};
  }

  double calc_speed_command(
    const Point2D & position,
    const Point2D & velocity,
    const std::vector<f1tenth_planning_msgs::msg::Wpnt> & waypoints,
    double lat_error_norm) const
  {
    const Point2D lookahead_position{
      position.x + velocity.x * speed_lookahead_,
      position.y + velocity.y * speed_lookahead_};
    const std::size_t lookahead_index = nearest_waypoint(lookahead_position, waypoints);
    double speed_command = waypoints[lookahead_index].vx_mps;
    speed_command = speed_adjust_lat_err(speed_command, lat_error_norm);
    return speed_command;
  }

  std::pair<Point2D, double> calc_l1_point(
    const Point2D & position,
    const std::vector<f1tenth_planning_msgs::msg::Wpnt> & waypoints,
    double speed_now,
    double lateral_error,
    double lat_error_norm)
  {
    idx_nearest_waypoint_ = nearest_waypoint(position, waypoints);

    // Match Unicorn's curvature preview: average |kappa| over waypoint
    // offsets [nearest + 10, nearest + 20). The local path is forward-only;
    // near a short horizon, use the available suffix instead of producing an
    // empty/NaN mean like an unchecked array slice would.
    constexpr std::size_t kCurvaturePreviewBeginOffset = 10U;
    constexpr std::size_t kCurvaturePreviewEndOffset = 20U;
    const std::size_t remaining = waypoints.size() - idx_nearest_waypoint_;
    if (remaining > kCurvaturePreviewBeginOffset) {
      const std::size_t preview_begin =
        idx_nearest_waypoint_ + kCurvaturePreviewBeginOffset;
      const std::size_t preview_end = idx_nearest_waypoint_ + std::min(
        remaining, kCurvaturePreviewEndOffset);
      double curvature_sum = 0.0;
      for (std::size_t index = preview_begin; index < preview_end; ++index) {
        curvature_sum += std::abs(waypoints[index].kappa_radpm);
      }
      curvature_waypoints_ = curvature_sum /
        static_cast<double>(preview_end - preview_begin);
    } else {
      curvature_waypoints_ = 0.0;
    }

    // Unicorn adaptive L1 (normal-driving branch): one final distance, not
    // competing speed- and curvature-selected target points.
    //   L1 = m*v + q - curvature_factor*mean(|kappa_preview|)*v^2
    const auto l1_from = [this](double speed) {
        return m_l1_ * speed + q_l1_ -
               curvature_factor_ * curvature_waypoints_ * speed * speed;
      };
    double l1_distance = l1_from(speed_now);

    // ── L1 거리와 횡가속 명령이 같은 속도를 써야 한다 (2026-08-17) ──────────
    // l1_distance 는 **실측** 속도(odom)로 잡는데, calc_steering_angle 의
    //   lateral_acceleration = 2*speed_for_lookup^2 / L1 * sin(eta)
    // 는 **경로 계획** 속도를 쓴다. 발진 순간 이 둘이 최대로 벌어진다 —
    // 실측 0 m/s 가 L1 을 하한(= max(t_clip_min, sqrt(2)*lat_err)) 으로
    // 눌러 놓는데 분자는 계획 5.4 m/s 의 제곱이다.
    //   run_0815_191056 실측: 그리드에서 레이스라인 대비 0.40m 옆에 놓고
    //   정지해 있던 7.2초 내내 조향이 -0.358 rad (한계 0.36 의 99.4%) 에
    //   고정됐고, 발진 후 4.8초간 d 가 peak-to-peak 1.30m 로 진동했다
    //   (부호 변화 4회). 포화가 백스톱을 깨워 속도가 1.5 로 묶이고, 낮은
    //   속도가 L1 을 다시 줄이는 자기강화 루프였다.
    //   같은 모델로 오프셋 0.10m 만 돼도 요구 횡가속이 22.9 m/s^2 —
    //   실측 그립 포락선(a_lat p99 = 8.76) 의 2.6배다. 즉 **레이스라인 위에
    //   정확히 놓아도 포화한다.** 0814 세 런(오프셋 0.03~0.13m)이 정지 중
    //   조향 66% 까지 갔던 것도 같은 원인이다.
    // 두 곳이 같은 속도를 쓰면 L1 ~= m*v, sin(eta) ~= e/L1 이므로 요구
    // 횡가속이 2e/m^2 = 9.05*e 로 **속도와 무관하게 오프셋에만 비례**한다
    // (0.40m -> 4.2 m/s^2, 포락선의 48%). 포화 없는 한계는 약 0.85m.
    //
    // 두 속도의 max 가 아니라 두 **거리**의 max 를 쓴다. L1(v) 는 곡률항
    // 때문에 v 에 대해 단조가 아닐 수 있어서(dL1/dv = m - 2*cf*kappa*v),
    // 거리로 비교해야 "L1 은 길어지기만 한다"가 곡률과 무관하게 성립한다.
    // L1 이 길어지면 1/L1 과 sin(eta)~=e/L1 양쪽으로 횡가속이 줄어드므로
    // 이 변경은 **새로운 포화를 만들 수 없다**. 제동/트레일링처럼 계획이
    // 실측보다 느린 구간(run_0815 기준 전체 사이클의 34%)에서는 종전과
    // 완전히 동일하게 동작한다.
    if (l1_speed_uses_path_plan_ && !waypoints.empty()) {
      const double path_speed = waypoints[idx_nearest_waypoint_].vx_mps;
      if (std::isfinite(path_speed) && path_speed > 0.0) {
        // calc_steering_angle 이 실제로 분자에 넣는 값과 같은 보정을 건다.
        // (lat_err_coeff_ = 0 이면 no-op, curvature_waypoints_ 는 위에서 갱신됨)
        l1_distance = std::max(
          l1_distance, l1_from(speed_adjust_lat_err(path_speed, lat_error_norm)));
      }
    }

    const double lower_bound = std::min(
      std::max(t_clip_min_, std::sqrt(2.0) * lateral_error), t_clip_max_);
    l1_distance = std::clamp(l1_distance, lower_bound, t_clip_max_);

    return {
      waypoint_at_distance_before_car(l1_distance, waypoints, idx_nearest_waypoint_),
      l1_distance};
  }

  double calc_steering_angle(
    const Point2D & position,
    const Point2D & l1_point,
    double l1_distance,
    double yaw,
    double lat_error_norm,
    const Point2D & velocity,
    const std::vector<f1tenth_planning_msgs::msg::Wpnt> & waypoints,
    double speed_now,
    const std::array<double, 10> & acceleration_samples,
    double control_dt)
  {
    const Point2D steering_lookahead_position{
      position.x + velocity.x * speed_lookahead_for_steer_,
      position.y + velocity.y * speed_lookahead_for_steer_};
    const std::size_t steering_lookahead_index =
      nearest_waypoint(steering_lookahead_position, waypoints);
    const double steering_lookahead_speed = waypoints[steering_lookahead_index].vx_mps;
    const double speed_for_lookup =
      speed_adjust_lat_err(steering_lookahead_speed, lat_error_norm);

    const Point2D l1_vector{l1_point.x - position.x, l1_point.y - position.y};
    const double l1_vector_norm = std::hypot(l1_vector.x, l1_vector.y);
    double eta = 0.0;
    if (l1_vector_norm <= kEpsilon) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Norm of L1 vector is zero; eta set to zero");
    } else {
      const double sin_eta = std::clamp(
        (-std::sin(yaw) * l1_vector.x + std::cos(yaw) * l1_vector.y) /
        l1_vector_norm, -1.0, 1.0);
      eta = std::asin(sin_eta);
    }

    double lateral_acceleration = 0.0;
    if (l1_distance > kEpsilon && std::abs(std::sin(eta)) > kEpsilon) {
      lateral_acceleration =
        2.0 * speed_for_lookup * speed_for_lookup / l1_distance * std::sin(eta);
    }

    double steering_angle = lookup_steer_angle(lateral_acceleration, speed_for_lookup);
    steering_angle = acc_scaling(steering_angle, acceleration_samples);
    steering_angle = speed_steer_scaling(steering_angle, speed_for_lookup);
    steering_angle *= std::clamp(
      1.0 + speed_now / 10.0, 1.0, velocity_steer_gain_max_);

    const double max_steering_delta = max_steering_rate_ * control_dt;
    if (std::abs(steering_angle - curr_steering_angle_) > max_steering_delta) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "MAP steering rate clipped");
    }
    steering_angle = std::clamp(
      steering_angle,
      curr_steering_angle_ - max_steering_delta,
      curr_steering_angle_ + max_steering_delta);
    steering_angle = std::clamp(
      steering_angle, -max_right_steering_angle_, max_left_steering_angle_);
    curr_steering_angle_ = steering_angle;
    return steering_angle;
  }

  double lookup_steer_angle(double lateral_acceleration, double speed) const
  {
    if (steer_lookup_.loaded()) {
      return steer_lookup_.lookup_steer_angle(lateral_acceleration, speed);
    }

    // a_y = v^2 * tan(delta) / wheelbase. This temporary inverse replaces only
    // the vehicle-specific LUT, not the ETH MAP/L1 guidance calculation.
    if (std::abs(speed) <= 0.05) {
      return 0.0;
    }
    return std::atan(wheelbase_ * lateral_acceleration / (speed * speed));
  }

  double acc_scaling(
    double steering_angle,
    const std::array<double, 10> & acceleration_samples) const
  {
    double mean_acceleration = 0.0;
    for (const double acceleration : acceleration_samples) {
      mean_acceleration += acceleration;
    }
    mean_acceleration /= static_cast<double>(acceleration_samples.size());

    if (mean_acceleration >= 1.0) {
      steering_angle *= acc_scaler_for_steer_;
    } else if (mean_acceleration <= -1.0) {
      steering_angle *= dec_scaler_for_steer_;
    }
    return steering_angle;
  }

  double speed_steer_scaling(double steering_angle, double speed) const
  {
    const double speed_difference = std::max(0.1, end_scale_speed_ - start_scale_speed_);
    const double factor = 1.0 - std::clamp(
      (speed - start_scale_speed_) / speed_difference, 0.0, 1.0) *
      downscale_factor_;
    return steering_angle * factor;
  }

  double speed_adjust_lat_err(double global_speed, double lat_error_norm) const
  {
    lat_error_norm *= 2.0;
    const double curvature = std::clamp(
      2.0 * (curvature_waypoints_ / max_curvature_reference_) - 2.0,
      0.0, 1.0);
    global_speed *=
      1.0 - lat_err_coeff_ + lat_err_coeff_ * std::exp(-lat_error_norm * curvature);
    return global_speed;
  }

  std::size_t nearest_waypoint(
    const Point2D & position,
    const std::vector<f1tenth_planning_msgs::msg::Wpnt> & waypoints) const
  {
    std::size_t nearest_index = 0;
    double nearest_squared_distance = std::numeric_limits<double>::max();
    double forward_progress = 0.0;
    for (std::size_t index = 0; index < waypoints.size(); ++index) {
      if (index > 0) {
        forward_progress += std::hypot(
          waypoints[index].x_m - waypoints[index - 1].x_m,
          waypoints[index].y_m - waypoints[index - 1].y_m);
        if (forward_progress > nearest_search_horizon_) {
          break;
        }
      }
      const double dx = position.x - waypoints[index].x_m;
      const double dy = position.y - waypoints[index].y_m;
      const double squared_distance = dx * dx + dy * dy;
      if (squared_distance < nearest_squared_distance) {
        nearest_squared_distance = squared_distance;
        nearest_index = index;
      }
    }
    return nearest_index;
  }

  Point2D waypoint_at_distance_before_car(
    double requested_distance,
    const std::vector<f1tenth_planning_msgs::msg::Wpnt> & waypoints,
    std::size_t waypoint_behind_car) const
  {
    // /local_waypoints is an ordered, forward-only window. Accumulate its
    // actual Cartesian segment lengths and deliberately do not wrap the local
    // vector: the planner already handles global-track wrap-around when it
    // constructs this window.
    const double target_distance = std::max(0.0, requested_distance);
    if (target_distance <= kEpsilon) {
      return {
        waypoints[waypoint_behind_car].x_m,
        waypoints[waypoint_behind_car].y_m};
    }
    double accumulated_distance = 0.0;
    for (std::size_t index = waypoint_behind_car;
      index + 1 < waypoints.size(); ++index)
    {
      accumulated_distance += std::hypot(
        waypoints[index + 1].x_m - waypoints[index].x_m,
        waypoints[index + 1].y_m - waypoints[index].y_m);
      if (accumulated_distance >= target_distance) {
        return {waypoints[index + 1].x_m, waypoints[index + 1].y_m};
      }
    }

    // The local planner did not provide enough horizon for the requested L1.
    // Use its final forward point rather than wrapping to a point behind the car.
    return {waypoints.back().x_m, waypoints.back().y_m};
  }

  void warn_and_stop(const char * reason)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000, "%s -> stop", reason);
    publish_stop();
  }

  void publish_stop()
  {
    if (!command_pub_) {
      return;
    }
    curr_steering_angle_ = 0.0;
    last_speed_command_ = 0.0;
    last_control_time_.reset();
    ackermann_msgs::msg::AckermannDriveStamped command;
    command.header.stamp = this->now();
    command.header.frame_id = "base_link";
    command_pub_->publish(command);
  }

  // Parameters
  double control_rate_{40.0};
  double waypoint_timeout_{0.5};
  double odom_timeout_{0.5};
  double frenet_odom_timeout_{0.5};
  double imu_acceleration_sign_{-1.0};
  bool require_source_stamps_{true};
  double source_stamp_future_tolerance_{0.1};
  bool localization_gate_enabled_{false};
  double localization_ready_timeout_{0.50};
  bool recovery_require_supervisor_{false};
  double recovery_command_timeout_{0.25};
  double recovery_pair_max_skew_{0.10};
  bool recovery_allow_reverse_{false};
  double recovery_max_reverse_speed_{0.30};
  double recovery_max_forward_override_{1.0};
  double t_clip_min_{0.7};
  double t_clip_max_{8.0};
  double m_l1_{0.47};
  double q_l1_{-0.2};
  double curvature_factor_{0.0};
  double speed_lookahead_{0.25};
  double lat_err_coeff_{0.0};
  double acc_scaler_for_steer_{1.0};
  double dec_scaler_for_steer_{1.0};
  double start_scale_speed_{6.5};
  double end_scale_speed_{10.0};
  double downscale_factor_{0.5};
  double speed_lookahead_for_steer_{0.0};
  double max_lateral_error_{0.5};
  bool backstop_enabled_{true};
  double backstop_err_m_{0.25};
  double backstop_sat_ratio_{0.85};
  double backstop_min_speed_{1.5};
  // 조향포화 분기가 열리기 위한 최소 경로오차 [m]. 0 이면 옛 동작(포화 단독).
  double backstop_sat_min_err_m_{0.12};
  double backstop_steering_reference_angle_{0.3578};
  double max_curvature_reference_{0.8};
  double velocity_steer_gain_max_{1.0};
  double nearest_search_horizon_{3.0};
  // L1 거리를 실측/계획 속도 중 긴 쪽으로 잡는다 (calc_l1_point 주석 참조).
  bool l1_speed_uses_path_plan_{true};
  double wheelbase_{0.33};
  double max_speed_{4.0};
  double max_left_steering_angle_{0.4189};
  double max_right_steering_angle_{0.4189};
  double max_steering_rate_{2.0};
  double max_acceleration_{5.5};
  double max_deceleration_{7.0};
  double speed_scale_{1.0};

  // Controller state
  bool autonomous_{false};
  bool estop_{false};
  bool escape_active_{false};
  rclcpp::Time escape_active_stamp_{0, 0, RCL_ROS_TIME};
  // 이 시간 넘게 신호가 없으면 원호 중이 아니라고 본다 (플래너 20 Hz).
  static constexpr double kEscapeActiveTimeoutSec{0.30};
  bool has_odom_{false};
  bool has_frenet_odom_{false};
  bool has_recovery_state_{false};
  bool has_recovery_target_speed_{false};
  bool has_localization_ready_{false};
  bool localization_ready_{false};
  double curr_steering_angle_{0.0};
  double last_speed_command_{0.0};
  double curvature_waypoints_{0.0};
  std::size_t idx_nearest_waypoint_{0};
  std::optional<rclcpp::Time> last_control_time_;
  LookupSteerAngle steer_lookup_;

  // Shared input data
  std::mutex data_mutex_;
  std::vector<f1tenth_planning_msgs::msg::Wpnt> waypoints_;
  nav_msgs::msg::Odometry odom_;
  nav_msgs::msg::Odometry frenet_odom_;
  std::array<double, 10> acceleration_samples_{};
  std::string waypoint_frame_;
  std::string recovery_state_;
  double recovery_target_speed_{std::numeric_limits<double>::quiet_NaN()};
  rclcpp::Time waypoint_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time frenet_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time waypoint_source_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time odom_source_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time frenet_odom_source_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time recovery_state_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time recovery_target_speed_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time localization_ready_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr frenet_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr escape_active_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr recovery_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr recovery_target_speed_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr lookahead_pub_;
  rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr mode_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
