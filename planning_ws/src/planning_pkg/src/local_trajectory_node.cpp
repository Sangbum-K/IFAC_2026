#include "planning_pkg/local_trajectory_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "planning_pkg/pitwall.hpp"

namespace
{
constexpr double kEpsilon = 1.0e-7;

double PositiveModulo(double value, double modulus)
{
  if (!(modulus > 0.0) || !std::isfinite(value)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double result = std::fmod(value, modulus);
  return result < 0.0 ? result + modulus : result;
}

double NormalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool IsZeroStamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec == 0 && stamp.nanosec == 0U;
}

bool IsFiniteWaypoint(const f1tenth_planning_msgs::msg::Wpnt & wpnt)
{
  return std::isfinite(wpnt.s_m) && std::isfinite(wpnt.d_m) &&
         std::isfinite(wpnt.x_m) && std::isfinite(wpnt.y_m) &&
         std::isfinite(wpnt.d_right) && std::isfinite(wpnt.d_left) &&
         std::isfinite(wpnt.psi_rad) && std::isfinite(wpnt.kappa_radpm) &&
         std::isfinite(wpnt.vx_mps) && std::isfinite(wpnt.ax_mps2);
}

bool ComputeTrackLength(
  const std::vector<f1tenth_planning_msgs::msg::Wpnt> & wpnts,
  double & track_length)
{
  if (wpnts.size() < 3U) {
    return false;
  }
  for (std::size_t i = 0; i < wpnts.size(); ++i) {
    if (!IsFiniteWaypoint(wpnts[i]) ||
      (i > 0U && !(wpnts[i].s_m > wpnts[i - 1U].s_m)))
    {
      return false;
    }
  }
  const double closure = std::hypot(
    wpnts.back().x_m - wpnts.front().x_m,
    wpnts.back().y_m - wpnts.front().y_m);
  track_length = wpnts.back().s_m - wpnts.front().s_m + closure;
  return closure > kEpsilon && std::isfinite(track_length) && track_length > 0.0;
}
}  // namespace

LocalTrajectoryNode::LocalTrajectoryNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  pitwall::Init(this);

  this->declare_parameter("local_trajectory/loop_rate_hz", 40.0);
  this->declare_parameter("local_trajectory/num_local_waypoints", 80);
  this->declare_parameter("local_trajectory/avoidance_timeout_sec", 0.5);
  this->declare_parameter("local_trajectory/require_avoidance_heartbeat", true);
  this->declare_parameter("local_trajectory/frenet_odom_timeout_sec", 0.20);
  this->declare_parameter("local_trajectory/frenet_odom_source_timeout_sec", 0.20);
  this->declare_parameter("local_trajectory/future_stamp_tolerance_sec", 0.05);
  this->declare_parameter("local_trajectory/min_frenet_jacobian", 0.20);
  this->declare_parameter("local_trajectory/max_avoidance_step_m", 1.0);
  this->declare_parameter("local_trajectory/minimum_horizon_sec", 1.5);
  this->declare_parameter("local_trajectory/minimum_braking_decel_mps2", 2.5);
  this->declare_parameter("local_trajectory/minimum_stop_buffer_m", 1.5);
  // ── 회피 경로 상실 시 속도 홀드 (2026-08-12) ─────────────────────────────
  // otwpnts 가 **빈 배열**로 오면(스플라이너는 살아 있고 회피만 못 만든 상태)
  // 예전에는 순수 레이스라인을 그대로 발행했다. 그러면 장애물을 잃은 순간
  // 계획 속도가 그 구간 레이스라인 값으로 **올라간다** — fail-open 이다.
  //   run_0812_013630 3:14 실측: 인지가 0.25s(5프레임) 빈 배열을 발행하는 동안
  //   local 선두 속도가 5.00 -> 6.50 m/s 로 올랐고 실측 속도도 4.84 -> 5.27 로
  //   따라 올랐다. 장애물이 1.99m 앞에서 돌아왔을 때는 통과 가능 속도(3.67)를
  //   훨씬 넘겨 어떤 회피도 성립하지 않았다. 위험 드롭아웃 8건 관측
  //   (공백 중앙 275ms / 최대 745ms).
  // 이제 직전 회피 경로의 선두 속도를 이 시간만큼 상한으로 유지한다.
  // **감속이 아니다** — 직전에 성립했던 속도를 넘지 못하게만 한다.
  //   0 으로 두면 이 홀드가 꺼지고 옛 fail-open 동작으로 돌아간다.
  //   길게 두면 유령 때문에 잃은 경우에도 오래 눌려 랩타임을 먹는다. 관측된
  //   공백 중앙값(275ms)의 약 2배인 0.5s 가 기본값이다.
  this->declare_parameter("local_trajectory/avoidance_loss_hold_sec", 0.5);
  // ── 홀드 상한의 바닥 (2026-08-17) ────────────────────────────────────────
  // 위 홀드의 목적은 "장애물 상실이 **가속** 신호가 되지 않게"인데, 기억해 둔
  // 선두 속도가 0 에 가까우면 홀드가 감속을 넘어 **정지 계획**을 130점 전부에
  // 퍼뜨린다. 그건 이 홀드의 권한 밖이다.
  //   run_0817_123650 실측 (좌벽 충돌): 1프레임 유령(id=8, nobs=0)에
  //   스플라이너가 trailing v_target=0.1 을 냈고 그 경로의 선두 5점 최솟값이
  //   0.042 였다. 다음 프레임 유령이 사라지자
  //     "회피 경로 상실 0.02s — 속도 상한 0.00 m/s 유지 (130점 캡)"
  //   가 12.58~13.05s (0.47초) 걸렸고, 그동안 실측 차속은 4.39 m/s 였다.
  //   전 점 vx=0 은 MAP 컨트롤러 조향을 죽인다(spliner_node.cpp 의 트레일링
  //   램프 가드에 같은 근거가 적혀 있다) → 조향 0.000 rad 개루프 직진 → 벽.
  //   같은 런의 발진(t=3.08)에서도 상한 0.00 이 130점에 걸렸다.
  // 이 바닥은 홀드가 **감속 장치로 변질되는 것만** 막는다. 홀드 상한은
  // 여전히 레이스라인 계획보다 훨씬 낮으므로 0812 의 fail-open 폭주
  // (5.00 -> 6.50 m/s)는 그대로 차단된다.
  //   run_0817 홀드 발동 16건 중 바닥에 걸리는 건 2건뿐 (0.00 두 번).
  //   나머지 14건은 1.80~4.63 m/s 라 이 값과 무관하다.
  //   0 으로 두면 이 바닥이 꺼지고 종전 동작으로 돌아간다.
  // 기본값은 spliner/trailing_retry_speed_degraded, control 의
  // backstop_min_speed_mps 와 같은 1.5 로 맞춘다.
  this->declare_parameter("local_trajectory/avoidance_loss_hold_floor_mps", 1.5);

  loop_rate_hz_ = this->get_parameter("local_trajectory/loop_rate_hz").as_double();
  num_local_waypoints_ =
    this->get_parameter("local_trajectory/num_local_waypoints").as_int();
  avoidance_timeout_sec_ =
    this->get_parameter("local_trajectory/avoidance_timeout_sec").as_double();
  require_avoidance_heartbeat_ =
    this->get_parameter("local_trajectory/require_avoidance_heartbeat").as_bool();
  frenet_odom_timeout_sec_ =
    this->get_parameter("local_trajectory/frenet_odom_timeout_sec").as_double();
  frenet_odom_source_timeout_sec_ =
    this->get_parameter("local_trajectory/frenet_odom_source_timeout_sec").as_double();
  future_stamp_tolerance_sec_ =
    this->get_parameter("local_trajectory/future_stamp_tolerance_sec").as_double();
  min_frenet_jacobian_ =
    this->get_parameter("local_trajectory/min_frenet_jacobian").as_double();
  max_avoidance_step_m_ =
    this->get_parameter("local_trajectory/max_avoidance_step_m").as_double();
  minimum_horizon_sec_ =
    this->get_parameter("local_trajectory/minimum_horizon_sec").as_double();
  minimum_braking_decel_mps2_ =
    this->get_parameter("local_trajectory/minimum_braking_decel_mps2").as_double();
  minimum_stop_buffer_m_ =
    this->get_parameter("local_trajectory/minimum_stop_buffer_m").as_double();
  avoidance_loss_hold_sec_ =
    this->get_parameter("local_trajectory/avoidance_loss_hold_sec").as_double();
  avoidance_loss_hold_floor_ = this->get_parameter(
    "local_trajectory/avoidance_loss_hold_floor_mps").as_double();

  const std::array<double, 9> safety_values = {
    loop_rate_hz_, avoidance_timeout_sec_, frenet_odom_timeout_sec_,
    frenet_odom_source_timeout_sec_, future_stamp_tolerance_sec_,
    max_avoidance_step_m_, minimum_horizon_sec_, minimum_braking_decel_mps2_,
    min_frenet_jacobian_};
  if (!std::all_of(
      safety_values.begin(), safety_values.end(),
      [](double value) {return std::isfinite(value);}) ||
    !std::isfinite(minimum_stop_buffer_m_) ||
    !std::isfinite(avoidance_loss_hold_floor_) ||
    // 0 = 비활성. 위로 열면 홀드가 "가속 억제"가 아니라 최저속도 보장이 돼
    // 인지 드롭아웃 중 레이스라인 속도까지 올려 줄 수 있다.
    avoidance_loss_hold_floor_ < 0.0 || avoidance_loss_hold_floor_ > 3.0 ||
    loop_rate_hz_ < 1.0 || loop_rate_hz_ > 100.0 ||
    num_local_waypoints_ < 2 || num_local_waypoints_ > 10000 ||
    avoidance_timeout_sec_ < 0.01 || avoidance_timeout_sec_ > 1.0 ||
    frenet_odom_timeout_sec_ < 0.01 || frenet_odom_timeout_sec_ > 1.0 ||
    frenet_odom_source_timeout_sec_ < 0.01 ||
    frenet_odom_source_timeout_sec_ > 1.0 ||
    future_stamp_tolerance_sec_ < 0.0 || future_stamp_tolerance_sec_ > 0.5 ||
    max_avoidance_step_m_ < 0.05 || max_avoidance_step_m_ > 5.0 ||
    minimum_horizon_sec_ <= 0.0 || minimum_horizon_sec_ > 10.0 ||
    minimum_braking_decel_mps2_ <= 0.0 ||
    minimum_braking_decel_mps2_ > 20.0 ||
    minimum_stop_buffer_m_ < 0.0 || minimum_stop_buffer_m_ > 20.0)
  {
    throw std::runtime_error(
            "Invalid/non-finite local trajectory safety parameter");
  }

  s_global_waypoints_ =
    this->create_subscription<f1tenth_planning_msgs::msg::WpntArray>(
    "/global_waypoints", qos_profile,
    std::bind(&LocalTrajectoryNode::CallbackGlobalWaypoints, this, std::placeholders::_1));
  s_frenet_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/car_state/frenet/odom", qos_profile,
    std::bind(&LocalTrajectoryNode::CallbackFrenetOdom, this, std::placeholders::_1));
  s_avoidance_waypoints_ =
    this->create_subscription<f1tenth_planning_msgs::msg::WpntArray>(
    "/planner/avoidance/otwpnts", qos_profile,
    std::bind(
      &LocalTrajectoryNode::CallbackAvoidanceWaypoints, this,
      std::placeholders::_1));

  p_local_waypoints_ =
    this->create_publisher<f1tenth_planning_msgs::msg::WpntArray>(
    "/local_waypoints", qos_profile);
  t_publish_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / loop_rate_hz_),
    [this]() {this->Run();});

  RCLCPP_INFO(
    this->get_logger(),
    "Local trajectory: %.1f Hz, base=%d points, Frenet receive/source timeout=%.3f/%.3fs",
    loop_rate_hz_, num_local_waypoints_, frenet_odom_timeout_sec_,
    frenet_odom_source_timeout_sec_);
}

LocalTrajectoryNode::~LocalTrajectoryNode() = default;

void LocalTrajectoryNode::CallbackGlobalWaypoints(
  const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg)
{
  double track_length = 0.0;
  if (!ComputeTrackLength(msg->wpnts, track_length)) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *get_clock(), 1000,
      "Rejecting invalid/non-monotonic global waypoints");
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_global_waypoints_);
  global_waypoints_ = msg->wpnts;
  has_global_waypoints_ = true;
  RCLCPP_INFO_ONCE(
    this->get_logger(), "Received %zu global waypoints, L=%.3fm",
    global_waypoints_.size(), track_length);
}

void LocalTrajectoryNode::CallbackFrenetOdom(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (IsZeroStamp(msg->header.stamp) ||
    !std::isfinite(msg->pose.pose.position.x) ||
    !std::isfinite(msg->pose.pose.position.y) ||
    !std::isfinite(msg->twist.twist.linear.x) ||
    !std::isfinite(msg->twist.twist.linear.y))
  {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *get_clock(), 1000,
      "Rejecting Frenet odom with zero stamp or non-finite state");
    return;
  }

  const rclcpp::Time source_stamp(msg->header.stamp, get_clock()->get_clock_type());
  const double source_age = (get_clock()->now() - source_stamp).seconds();
  if (!std::isfinite(source_age) ||
    source_age > frenet_odom_source_timeout_sec_ ||
    source_age < -future_stamp_tolerance_sec_)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *get_clock(), 1000,
      "Rejecting Frenet odom with source age %.3fs", source_age);
    return;
  }

  const int64_t source_stamp_ns = source_stamp.nanoseconds();
  std::lock_guard<std::mutex> lock(mutex_frenet_odom_);
  if (last_frenet_source_stamp_ns_ != 0 &&
    source_stamp_ns <= last_frenet_source_stamp_ns_)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *get_clock(), 1000,
      "Rejecting duplicate/out-of-order Frenet timestamp (%ld <= %ld)",
      static_cast<long>(source_stamp_ns),
      static_cast<long>(last_frenet_source_stamp_ns_));
    return;
  }
  current_frenet_odom_ = *msg;
  frenet_odom_arrival_time_ = std::chrono::steady_clock::now();
  last_frenet_source_stamp_ns_ = source_stamp_ns;
  has_frenet_odom_ = true;
}

void LocalTrajectoryNode::CallbackAvoidanceWaypoints(
  const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg)
{
  if (IsZeroStamp(msg->header.stamp)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *get_clock(), 1000,
      "Rejecting avoidance path with zero source timestamp");
    return;
  }
  const rclcpp::Time source_stamp(msg->header.stamp, get_clock()->get_clock_type());
  const double source_age = (get_clock()->now() - source_stamp).seconds();
  if (!std::isfinite(source_age) || source_age > avoidance_timeout_sec_ ||
    source_age < -future_stamp_tolerance_sec_)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *get_clock(), 1000,
      "Rejecting avoidance path with source age %.3fs", source_age);
    return;
  }

  const int64_t source_stamp_ns = source_stamp.nanoseconds();
  std::lock_guard<std::mutex> lock(mutex_avoidance_);
  if (last_avoidance_source_stamp_ns_ != 0 &&
    source_stamp_ns <= last_avoidance_source_stamp_ns_)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *get_clock(), 1000,
      "Rejecting duplicate/out-of-order avoidance timestamp");
    return;
  }
  avoidance_waypoints_ = msg->wpnts;
  avoidance_arrival_time_ = std::chrono::steady_clock::now();
  avoidance_source_stamp_ns_ = source_stamp_ns;
  last_avoidance_source_stamp_ns_ = source_stamp_ns;
  has_avoidance_message_ = true;
}

int LocalTrajectoryNode::FindClosestWaypointIndex(
  double s_coordinate,
  const std::vector<f1tenth_planning_msgs::msg::Wpnt> & global_waypoints,
  double track_length)
{
  if (global_waypoints.empty() || !(track_length > 0.0)) {
    return -1;
  }
  const double wrapped_s = PositiveModulo(s_coordinate, track_length);
  if (!std::isfinite(wrapped_s)) {
    return -1;
  }

  double best_distance = std::numeric_limits<double>::infinity();
  int best_index = -1;
  for (std::size_t i = 0; i < global_waypoints.size(); ++i) {
    double delta = std::abs(
      PositiveModulo(global_waypoints[i].s_m - wrapped_s, track_length));
    delta = std::min(delta, track_length - delta);
    if (delta < best_distance) {
      best_distance = delta;
      best_index = static_cast<int>(i);
    }
  }
  return best_index;
}

void LocalTrajectoryNode::Run()
{
  std::vector<f1tenth_planning_msgs::msg::Wpnt> global_waypoints_copy;
  {
    std::lock_guard<std::mutex> lock(mutex_global_waypoints_);
    if (!has_global_waypoints_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *get_clock(), 1000, "Waiting for global waypoints");
      return;
    }
    global_waypoints_copy = global_waypoints_;
  }

  nav_msgs::msg::Odometry frenet_odom_copy;
  std::chrono::steady_clock::time_point arrival_time;
  {
    std::lock_guard<std::mutex> lock(mutex_frenet_odom_);
    if (!has_frenet_odom_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *get_clock(), 1000, "Waiting for Frenet odometry");
      return;
    }
    frenet_odom_copy = current_frenet_odom_;
    arrival_time = frenet_odom_arrival_time_;
  }

  const double receive_age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - arrival_time).count();
  const rclcpp::Time source_stamp(
    frenet_odom_copy.header.stamp, get_clock()->get_clock_type());
  const double source_age = (get_clock()->now() - source_stamp).seconds();
  if (!std::isfinite(receive_age) || receive_age > frenet_odom_timeout_sec_ ||
    !std::isfinite(source_age) || source_age > frenet_odom_source_timeout_sec_ ||
    source_age < -future_stamp_tolerance_sec_)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *get_clock(), 1000,
      "Frenet stale: receive age=%.3fs, source age=%.3fs; local publish stopped",
      receive_age, source_age);
    return;
  }

  PublishLocalWaypoints(global_waypoints_copy, frenet_odom_copy);
}

void LocalTrajectoryNode::PublishLocalWaypoints(
  const std::vector<f1tenth_planning_msgs::msg::Wpnt> & global_waypoints_copy,
  const nav_msgs::msg::Odometry & frenet_odom_copy)
{
  double track_length = 0.0;
  if (!ComputeTrackLength(global_waypoints_copy, track_length)) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *get_clock(), 1000,
      "Global path became invalid; local publish stopped");
    return;
  }

  const double current_s = frenet_odom_copy.pose.pose.position.x;
  const double current_d = frenet_odom_copy.pose.pose.position.y;
  const double coordinate_s_dot = frenet_odom_copy.twist.twist.linear.x;
  const double coordinate_d_dot = frenet_odom_copy.twist.twist.linear.y;
  if (!std::isfinite(current_s) || !std::isfinite(current_d) ||
    !std::isfinite(coordinate_s_dot) || !std::isfinite(coordinate_d_dot))
  {
    return;
  }
  const int start_idx = FindClosestWaypointIndex(
    current_s, global_waypoints_copy, track_length);
  if (start_idx < 0) {
    return;
  }
  const double jacobian =
    1.0 - global_waypoints_copy[start_idx].kappa_radpm * current_d;
  if (!std::isfinite(jacobian) || jacobian <= min_frenet_jacobian_) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *get_clock(), 1000,
      "Unsafe Frenet Jacobian %.3f; local publish stopped", jacobian);
    return;
  }
  // Frenet twist.x is ds/dt. Convert it back to physical tangent velocity
  // before calculating a braking horizon.
  const double current_speed = std::hypot(
    jacobian * coordinate_s_dot, coordinate_d_dot);

  const double braking_distance =
    current_speed * current_speed / (2.0 * minimum_braking_decel_mps2_) +
    minimum_stop_buffer_m_;
  const double time_horizon_distance = current_speed * minimum_horizon_sec_;
  const double required_horizon_distance =
    std::max(braking_distance, time_horizon_distance);

  f1tenth_planning_msgs::msg::WpntArray local_waypoints;
  // Propagate localization source time rather than making stale pose-derived
  // geometry look new at every local-planner timer tick.
  local_waypoints.header.stamp = frenet_odom_copy.header.stamp;
  local_waypoints.header.frame_id = "map";

  const int trajectory_size = static_cast<int>(global_waypoints_copy.size());
  const int base_count = std::min(num_local_waypoints_, trajectory_size);
  local_waypoints.wpnts.reserve(trajectory_size);
  double covered_distance = 0.0;
  for (int offset = 0; offset < trajectory_size; ++offset) {
    const int index = (start_idx + offset) % trajectory_size;
    auto waypoint = global_waypoints_copy[index];
    waypoint.id = static_cast<int32_t>(local_waypoints.wpnts.size());
    if (!local_waypoints.wpnts.empty()) {
      const double ds = PositiveModulo(
        waypoint.s_m - local_waypoints.wpnts.back().s_m, track_length);
      if (!(ds > kEpsilon) || ds > track_length * 0.5) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *get_clock(), 1000,
          "Invalid global waypoint progress %.3fm; local publish stopped", ds);
        return;
      }
      covered_distance += ds;
    }
    local_waypoints.wpnts.push_back(waypoint);
    if (static_cast<int>(local_waypoints.wpnts.size()) >= base_count &&
      covered_distance >= required_horizon_distance)
    {
      break;
    }
  }

  if (covered_distance + kEpsilon < required_horizon_distance) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *get_clock(), 1000,
      "Closed track cannot provide required %.2fm stopping horizon (available %.2fm); "
      "local publish stopped",
      required_horizon_distance, covered_distance);
    return;
  }

  std::vector<f1tenth_planning_msgs::msg::Wpnt> avoidance_copy;
  bool avoidance_is_fresh = false;
  {
    std::lock_guard<std::mutex> lock(mutex_avoidance_);
    if (has_avoidance_message_) {
      const double receive_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - avoidance_arrival_time_).count();
      const rclcpp::Time source_stamp(
        avoidance_source_stamp_ns_, get_clock()->get_clock_type());
      const double source_age = (get_clock()->now() - source_stamp).seconds();
      avoidance_is_fresh =
        std::isfinite(receive_age) && receive_age <= avoidance_timeout_sec_ &&
        std::isfinite(source_age) && source_age <= avoidance_timeout_sec_ &&
        source_age >= -future_stamp_tolerance_sec_;
      if (avoidance_is_fresh) {
        avoidance_copy = avoidance_waypoints_;
      }
    }
  }

  if (require_avoidance_heartbeat_ && !avoidance_is_fresh) {
    // Spliner가 죽으면 회피 토픽이 끊기지만, 예전 동작은 전속 global path를
    // 계속 발행했다. heartbeat를 safety lease로 삼아 local publish를 멈추면
    // control/recovery waypoint timeout이 실제 차량을 정지시킨다.
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *get_clock(), 1000,
      "Avoidance planner heartbeat missing/stale; local publish stopped");
    return;
  }

  int spliced_count = 0;
  if (avoidance_is_fresh && !avoidance_copy.empty() &&
    !SpliceAvoidanceWaypoints(
      local_waypoints.wpnts, avoidance_copy, track_length, spliced_count,
      frenet_odom_copy.pose.pose.position.y))
  {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *get_clock(), 1000,
      "Invalid avoidance trajectory rejected; local publish stopped");
    return;
  }

  // ── 회피 상실 시 속도 홀드 (fail-closed) ────────────────────────────────
  // 회피 경로가 있을 때는 그 선두 속도를 기억하고, 빈 배열로 바뀌면 잠시
  // 그 값을 상한으로 유지한다. 장애물을 잃은 것이 **가속 신호가 되지 않게**
  // 하는 것이 목적이다 (근거는 위 declare_parameter 주석).
  // 감속은 하지 않는다 — 직전에 성립했던 속도를 넘지 못하게만 한다.
  if (avoidance_loss_hold_sec_ > 0.0) {
    const auto now_steady = std::chrono::steady_clock::now();
    if (avoidance_is_fresh && !avoidance_copy.empty()) {
      // 선두 몇 점의 최솟값을 쓴다 — 선두 한 점만 보면 그 점이 이미 지나간
      // 뒤의 값일 수 있어 상한이 실제 계획보다 높아진다.
      //
      // ── 기준점은 반드시 **차 앞**이어야 한다 (2026-08-17) ────────────────
      // 커밋된 회피 경로는 s 가 고정이라 차가 전진해도 머리가 뒤에 남는다.
      // 배열 선두를 그대로 쓰면 이미 한참 지나온 점의 속도가 상한이 된다.
      // 정지 중 만들어진 커밋이면 그 점의 vx 는 0 이다 (ApplySpeedProfile 이
      // 머리를 cur_vs_ 에 앵커하므로) — run_0815_191056 실측:
      //   "회피 경로 상실 0.02s — 속도 상한 0.00 m/s 유지 (130점 캡)"
      //   명령 속도 0 이 10.366~11.175s (0.81초), 차속 2.09 -> 0.90 m/s.
      //   그 커밋의 머리는 s=0.14 인데 차는 s=7.5 에 있었다.
      // 이 홀드의 목적은 "장애물 상실이 **가속** 신호가 되지 않게"이므로
      // 기준은 지금 차가 향하는 구간이어야 한다. 뒤쪽 점을 건너뛰면
      // 같은 상황에서 상한이 3.9 m/s (경로 끝 계획속도) 로 나온다.
      std::size_t begin = 0U;
      while (begin + 1U < avoidance_copy.size() &&
        PositiveModulo(current_s - avoidance_copy[begin].s_m, track_length) <
        track_length * 0.5)
      {
        ++begin;
      }
      const std::size_t look =
        std::min<std::size_t>(avoidance_copy.size() - begin, 5U);
      double head = std::numeric_limits<double>::infinity();
      for (std::size_t i = begin; i < begin + look; ++i) {
        const double v = avoidance_copy[i].vx_mps;
        if (std::isfinite(v) && v >= 0.0) {
          head = std::min(head, v);
        }
      }
      if (std::isfinite(head)) {
        last_avoidance_head_speed_ = head;
        last_avoidance_nonempty_time_ = now_steady;
        has_avoidance_head_speed_ = true;
      }
    } else if (has_avoidance_head_speed_ && last_avoidance_head_speed_ >= 0.0) {
      const double since = std::chrono::duration<double>(
        now_steady - last_avoidance_nonempty_time_).count();
      if (std::isfinite(since) && since >= 0.0 &&
        since <= avoidance_loss_hold_sec_)
      {
        // 바닥을 적용한 값으로 캡한다. 기억해 둔 선두 속도 자체는 건드리지
        // 않는다 — 로그가 "무엇을 기억했는지"와 "무엇을 걸었는지"를 둘 다
        // 남겨야 사후 rosbag 분석에서 인과를 되짚을 수 있다.
        // (검증에서 avoidance_loss_hold_floor_ >= 0 이므로 max 0 은 불필요하다.)
        const double cap =
          std::max(last_avoidance_head_speed_, avoidance_loss_hold_floor_);
        int capped = 0;
        for (auto & w : local_waypoints.wpnts) {
          if (std::isfinite(w.vx_mps) && w.vx_mps > cap) {
            w.vx_mps = cap;
            ++capped;
          }
        }
        if (capped > 0) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *get_clock(), 1000,
            "회피 경로 상실 %.2fs — 속도 상한 %.2f m/s 유지 (%d점 캡, "
            "기억 %.2f / 바닥 %.2f). 장애물 상실이 가속으로 이어지지 않게 한다",
            since, cap, capped, last_avoidance_head_speed_,
            avoidance_loss_hold_floor_);
        }
        pitwall::Log("avoidance_loss_hold", 1.0);
        // 불리언만으로는 사후 분석이 안 된다 — 위 WARN 은 1Hz throttle 이라
        // 0.5s 짜리 홀드 한 건이 한 줄만 남고, 그 한 줄의 숫자는 그 사이클
        // 것뿐이다. 캡/기억을 수치 채널로 내보내 매 사이클 백에 남긴다
        // (기존 스크립트가 /pitwall/avoidance_loss_hold 를 이미 읽는다:
        //  tools/bag_analysis_0813/quant.py, bag_analysis_0817/rej.py).
        pitwall::Log("avoidance_loss_hold_cap", cap);
        pitwall::Log("avoidance_loss_hold_memory", last_avoidance_head_speed_);
      } else {
        has_avoidance_head_speed_ = false;
        pitwall::Log("avoidance_loss_hold", 0.0);
      }
    }
  }

  // 실제로 치환된 개수를 보고한다. 예전엔 수신한 회피 배열의 크기를 그대로
  // 실었기 때문에, s 구간이 겹치지 않아 한 점도 치환되지 않은 사이클도
  // "치환 130점"으로 기록돼 사후 rosbag 분석에서 인과를 뒤집었다.
  pitwall::Log("spliced_wpnts", static_cast<double>(spliced_count));
  p_local_waypoints_->publish(local_waypoints);

  RCLCPP_DEBUG(
    this->get_logger(),
    "Published %zu local waypoints, horizon %.2fm, start index %d",
    local_waypoints.wpnts.size(), covered_distance, start_idx);
}

bool LocalTrajectoryNode::SpliceAvoidanceWaypoints(
  std::vector<f1tenth_planning_msgs::msg::Wpnt> & local_wpnts,
  const std::vector<f1tenth_planning_msgs::msg::Wpnt> & avoidance_wpnts,
  double track_length,
  int & replaced_out,
  double ego_d)
{
  replaced_out = 0;
  if (local_wpnts.size() < 2U || avoidance_wpnts.size() < 2U ||
    !(track_length > 0.0))
  {
    return false;
  }

  std::vector<double> avoidance_progress(avoidance_wpnts.size(), 0.0);
  for (std::size_t i = 0; i < avoidance_wpnts.size(); ++i) {
    if (!IsFiniteWaypoint(avoidance_wpnts[i])) {
      return false;
    }
    if (i == 0U) {
      continue;
    }
    const double raw_ds =
      avoidance_wpnts[i].s_m - avoidance_wpnts[i - 1U].s_m;
    // Accept a normal forward step or one normal closed-track wrap. Generic
    // modulo would incorrectly disguise a corrupt +one-lap jump as a 0.1m step.
    const double ds = raw_ds > 0.0 ? raw_ds : raw_ds + track_length;
    const double cartesian_step = std::hypot(
      avoidance_wpnts[i].x_m - avoidance_wpnts[i - 1U].x_m,
      avoidance_wpnts[i].y_m - avoidance_wpnts[i - 1U].y_m);
    const double max_cartesian_step =
      std::max(2.0 * max_avoidance_step_m_, 3.0 * ds + 0.2);
    if (!(ds > kEpsilon) || ds > max_avoidance_step_m_ ||
      !std::isfinite(cartesian_step) || cartesian_step > max_cartesian_step)
    {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *get_clock(), 1000,
        "Avoidance path is non-monotonic/discontinuous at %zu: ds=%.3f, dxy=%.3f",
        i, ds, cartesian_step);
      return false;
    }
    avoidance_progress[i] = avoidance_progress[i - 1U] + ds;
  }

  // local_wpnts 는 이 호출 직전에 PublishLocalWaypoints 가 global_waypoints_copy
  // (ComputeTrackLength 에서 전 원소 IsFiniteWaypoint 통과)에서 복사해 만들고
  // 그 자리에서 ds 단조성까지 검사한 것이다 — 여기서 다시 검증하지 않는다.
  // 누적 progress 만 계산한다.
  std::vector<double> local_progress(local_wpnts.size(), 0.0);
  for (std::size_t i = 1; i < local_wpnts.size(); ++i) {
    const double ds = PositiveModulo(
      local_wpnts[i].s_m - local_wpnts[i - 1U].s_m, track_length);
    local_progress[i] = local_progress[i - 1U] + ds;
  }

  const double avoidance_start =
    PositiveModulo(avoidance_wpnts.front().s_m, track_length);
  const double local_start = PositiveModulo(local_wpnts.front().s_m, track_length);
  double query_at_local_start = local_start - avoidance_start;
  query_at_local_start -=
    std::round(query_at_local_start / track_length) * track_length;

  const double total_avoidance_progress = avoidance_progress.back();
  const double match_tolerance =
    std::min(0.5 * avoidance_progress[1U], 0.10) + kEpsilon;
  int replaced = 0;
  int first_replaced = -1;
  for (std::size_t local_index = 0; local_index < local_wpnts.size(); ++local_index) {
    double query = query_at_local_start + local_progress[local_index];
    if (query < -match_tolerance ||
      query > total_avoidance_progress + match_tolerance)
    {
      continue;
    }
    query = std::clamp(query, 0.0, total_avoidance_progress);

    const auto upper = std::upper_bound(
      avoidance_progress.begin(), avoidance_progress.end(), query);
    std::size_t upper_index =
      static_cast<std::size_t>(std::distance(avoidance_progress.begin(), upper));
    if (upper_index == 0U) {
      upper_index = 1U;
    } else if (upper_index >= avoidance_progress.size()) {
      upper_index = avoidance_progress.size() - 1U;
    }
    const std::size_t lower_index = upper_index - 1U;
    const double span =
      avoidance_progress[upper_index] - avoidance_progress[lower_index];
    if (!(span > kEpsilon)) {
      return false;
    }
    const double ratio = std::clamp(
      (query - avoidance_progress[lower_index]) / span, 0.0, 1.0);
    const auto & lower = avoidance_wpnts[lower_index];
    const auto & upper_wpnt = avoidance_wpnts[upper_index];

    auto replacement = lower;
    const auto lerp = [ratio](double a, double b) {
        return a + ratio * (b - a);
      };
    replacement.d_m = lerp(lower.d_m, upper_wpnt.d_m);
    replacement.x_m = lerp(lower.x_m, upper_wpnt.x_m);
    replacement.y_m = lerp(lower.y_m, upper_wpnt.y_m);
    replacement.d_right = lerp(lower.d_right, upper_wpnt.d_right);
    replacement.d_left = lerp(lower.d_left, upper_wpnt.d_left);
    replacement.psi_rad = NormalizeAngle(
      lower.psi_rad + ratio * NormalizeAngle(upper_wpnt.psi_rad - lower.psi_rad));
    replacement.kappa_radpm =
      lerp(lower.kappa_radpm, upper_wpnt.kappa_radpm);
    replacement.vx_mps = lerp(lower.vx_mps, upper_wpnt.vx_mps);
    replacement.ax_mps2 = lerp(lower.ax_mps2, upper_wpnt.ax_mps2);

    // Keep the ordered local reference coordinate. Avoidance s may wrap one
    // or several times; copying it would make /local_waypoints non-monotonic.
    replacement.s_m = local_wpnts[local_index].s_m;
    replacement.id = local_wpnts[local_index].id;
    if (!IsFiniteWaypoint(replacement)) {
      return false;
    }
    local_wpnts[local_index] = replacement;
    if (first_replaced < 0) {
      first_replaced = static_cast<int>(local_index);
    }
    ++replaced;
  }

  replaced_out = replaced;
  if (replaced > 0) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *get_clock(), 2000,
      "Spliced %d/%zu local waypoints using ordered avoidance progress %.2fm",
      replaced, local_wpnts.size(), total_avoidance_progress);
  }
  // ── 진단 (2026-08-14) ──────────────────────────────────────────────────
  // 회피 경로는 유효한데 **차 바로 앞 구간이 raceline 그대로** 남는 경우가
  // 있고, 그때 컨트롤러는 자기 위치에서 1 m 떨어진 경로를 받는다:
  //   run_0813_221645 t=161.71 — ego d=+0.97, otwpnts 선두 d=+0.97(정상),
  //   그런데 /local_waypoints 선두 11점이 전부 d=0.00. 추종오차 0.97 m 로
  //   백스톱이 5.3~6.3 -> 1.50 m/s 로 잘랐다 (직선 18랩 중 5랩).
  // 실패 사유를 남기는 로그가 없어 원인을 못 좁혔다. 정렬 수치를 그대로
  // 찍어 다음 백에서 판정한다 — 동작은 바꾸지 않는다.
  //   query_at_local_start < -match_tolerance 면 회피 경로가 로컬 시작보다
  //   **앞에서** 시작한다는 뜻이고(스플라이너가 늦게 앵커),
  //   > 0 이면 회피 경로가 이미 차 뒤에서 시작한다는 뜻이다.
  //
  // 선두를 못 덮는 것 자체는 정상일 수 있다 — free spline 은 원래 차보다
  // 앞에서 시작한다. 해로운 경우는 **차는 라인을 벗어나 있는데 선두가
  // raceline 위**일 때 하나뿐이므로 그 조건에서만 찍는다. 임계 0.25 m 는
  // control 백스톱의 경로오차 램프 시작점(map/backstop_path_error_m)과 같은
  // 값이다 — 그 아래면 어차피 백스톱이 안 문다.
  const double head_gap = local_wpnts.empty()
    ? 0.0 : std::abs(ego_d - local_wpnts.front().d_m);
  if ((replaced == 0 || first_replaced > 0) && head_gap > 0.25) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *get_clock(), 1000,
      "회피 스플라이스가 선두를 못 덮음: ego_d=%.2f head_d=%.2f gap=%.2fm | "
      "replaced=%d first=%d/%zu | query@local_start=%.3f tol=%.3f "
      "avoid_progress=%.2fm local_start_s=%.2f avoid_start_s=%.2f",
      ego_d, local_wpnts.empty() ? 0.0 : local_wpnts.front().d_m, head_gap,
      replaced, first_replaced, local_wpnts.size(),
      query_at_local_start, match_tolerance, total_avoidance_progress,
      local_start, avoidance_start);
  }
  return true;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalTrajectoryNode>("local_trajectory_node"));
  rclcpp::shutdown();
  return 0;
}
