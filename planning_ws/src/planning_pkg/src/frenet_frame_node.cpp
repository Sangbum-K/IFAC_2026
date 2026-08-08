#include "planning_pkg/frenet_frame_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "planning_pkg/pitwall.hpp"

namespace
{
constexpr double kEpsilon = 1.0e-9;

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

bool IsFiniteOdom(const nav_msgs::msg::Odometry & odom)
{
  const auto & p = odom.pose.pose.position;
  const auto & q = odom.pose.pose.orientation;
  const auto & v = odom.twist.twist.linear;
  return std::isfinite(p.x) && std::isfinite(p.y) &&
         std::isfinite(q.x) && std::isfinite(q.y) &&
         std::isfinite(q.z) && std::isfinite(q.w) &&
         std::isfinite(v.x) && std::isfinite(v.y);
}

bool IsZeroStamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec == 0 && stamp.nanosec == 0U;
}
}  // namespace

FrenetFrameNode::FrenetFrameNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  pitwall::Init(this);

  this->declare_parameter("frenet_frame/loop_rate_hz", 40.0);
  this->declare_parameter("frenet_frame/odom_mode", "sim_gt");
  this->declare_parameter("frenet_frame/odom_receive_timeout_sec", 0.20);
  this->declare_parameter("frenet_frame/odom_source_timeout_sec", 0.20);
  this->declare_parameter("frenet_frame/odom_future_tolerance_sec", 0.05);
  this->declare_parameter("frenet_frame/projection_search_segments", 40);
  this->declare_parameter("frenet_frame/projection_reacquire_distance_m", 2.0);
  this->declare_parameter("frenet_frame/projection_heading_weight_m2", 0.25);
  this->declare_parameter("frenet_frame/projection_progress_weight", 0.02);
  this->declare_parameter("frenet_frame/min_frenet_jacobian", 0.20);

  loop_rate_hz_ = this->get_parameter("frenet_frame/loop_rate_hz").as_double();
  odom_mode_ = this->get_parameter("frenet_frame/odom_mode").as_string();
  odom_receive_timeout_sec_ =
    this->get_parameter("frenet_frame/odom_receive_timeout_sec").as_double();
  odom_source_timeout_sec_ =
    this->get_parameter("frenet_frame/odom_source_timeout_sec").as_double();
  odom_future_tolerance_sec_ =
    this->get_parameter("frenet_frame/odom_future_tolerance_sec").as_double();
  projection_search_segments_ =
    this->get_parameter("frenet_frame/projection_search_segments").as_int();
  projection_reacquire_distance_m_ =
    this->get_parameter("frenet_frame/projection_reacquire_distance_m").as_double();
  projection_heading_weight_m2_ =
    this->get_parameter("frenet_frame/projection_heading_weight_m2").as_double();
  projection_progress_weight_ =
    this->get_parameter("frenet_frame/projection_progress_weight").as_double();
  min_frenet_jacobian_ =
    this->get_parameter("frenet_frame/min_frenet_jacobian").as_double();

  const auto require_range = [](double value, double minimum, double maximum,
      const char * name) {
      if (!std::isfinite(value) || value < minimum || value > maximum) {
        throw std::invalid_argument(
                std::string(name) + " must be finite and in [" +
                std::to_string(minimum) + ", " + std::to_string(maximum) + "]");
      }
    };
  require_range(loop_rate_hz_, 1.0, 500.0, "frenet_frame/loop_rate_hz");
  require_range(
    odom_receive_timeout_sec_, 0.01, 5.0,
    "frenet_frame/odom_receive_timeout_sec");
  require_range(
    odom_source_timeout_sec_, 0.01, 5.0,
    "frenet_frame/odom_source_timeout_sec");
  require_range(
    odom_future_tolerance_sec_, 0.0, 1.0,
    "frenet_frame/odom_future_tolerance_sec");
  if (projection_search_segments_ < 2 || projection_search_segments_ > 100000) {
    throw std::invalid_argument(
            "frenet_frame/projection_search_segments must be in [2, 100000]");
  }
  require_range(
    projection_reacquire_distance_m_, 0.1, 100.0,
    "frenet_frame/projection_reacquire_distance_m");
  require_range(
    projection_heading_weight_m2_, 0.0, 100.0,
    "frenet_frame/projection_heading_weight_m2");
  require_range(
    projection_progress_weight_, 0.0, 100.0,
    "frenet_frame/projection_progress_weight");
  require_range(
    min_frenet_jacobian_, 0.01, 0.95,
    "frenet_frame/min_frenet_jacobian");

  RCLCPP_INFO(
    this->get_logger(),
    "Frenet: %.1f Hz, odom_mode=%s, receive/source timeout=%.3f/%.3fs",
    loop_rate_hz_, odom_mode_.c_str(), odom_receive_timeout_sec_, odom_source_timeout_sec_);

  const auto odom_callback =
    std::bind(&FrenetFrameNode::CallbackOdom, this, std::placeholders::_1);
  if (odom_mode_ == "sim_gt") {
    s_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/ego_racecar/odom", qos_profile, odom_callback);
  } else if (odom_mode_ == "real") {
    s_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/pf/pose/odom", qos_profile, odom_callback);
  } else {
    // sim_pf 는 real 과 구독 토픽도 이후 경로도 완전히 같아서 이름만 다른
    // 세 번째 모드였고, 지정하는 yaml/launch 도 없었다 — 조용히 real 로
    // 도는 대신 명시적으로 거부한다.
    throw std::invalid_argument(
            "Invalid frenet_frame/odom_mode '" + odom_mode_ +
            "' (expected sim_gt or real)");
  }

  s_global_waypoints_ =
    this->create_subscription<f1tenth_planning_msgs::msg::WpntArray>(
    "/global_waypoints", qos_profile,
    std::bind(&FrenetFrameNode::CallbackGlobalWaypoints, this, std::placeholders::_1));

  p_frenet_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "/car_state/frenet/odom", qos_profile);

  t_process_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / loop_rate_hz_),
    [this]() {this->Run();});
}

FrenetFrameNode::~FrenetFrameNode() = default;

void FrenetFrameNode::CallbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!IsFiniteOdom(*msg) || IsZeroStamp(msg->header.stamp)) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *get_clock(), 1000,
      "Rejecting odom with non-finite state or zero source timestamp");
    return;
  }

  const rclcpp::Time source_stamp(msg->header.stamp, get_clock()->get_clock_type());
  const rclcpp::Time now = get_clock()->now();
  const double source_age = (now - source_stamp).seconds();
  if (!std::isfinite(source_age) ||
    source_age > odom_source_timeout_sec_ ||
    source_age < -odom_future_tolerance_sec_)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *get_clock(), 1000,
      "Rejecting odom: source timestamp age %.3fs outside [-%.3f, %.3f]s",
      source_age, odom_future_tolerance_sec_, odom_source_timeout_sec_);
    return;
  }

  const int64_t source_stamp_ns = source_stamp.nanoseconds();
  std::lock_guard<std::mutex> lock(mutex_odom_);
  if (odom_source_stamp_ns_ != 0 &&
    source_stamp_ns <= odom_source_stamp_ns_)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *get_clock(), 1000,
      "Rejecting duplicate/out-of-order odom timestamp (%ld <= %ld)",
      static_cast<long>(source_stamp_ns),
      static_cast<long>(odom_source_stamp_ns_));
    return;
  }

  current_odom_ = *msg;
  odom_arrival_time_ = std::chrono::steady_clock::now();
  // (last_accepted_odom_stamp_ns_ 제거: 이 한 곳에서만 쓰기가 일어나 절대
  //  odom_source_stamp_ns_ 와 갈라지지 않는 중복 멤버였다 — 하나로 합쳤다.)
  odom_source_stamp_ns_ = source_stamp_ns;
  has_odom_ = true;
}

void FrenetFrameNode::CallbackGlobalWaypoints(
  const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg)
{
  if (msg->wpnts.size() < 3U) {
    RCLCPP_ERROR(this->get_logger(), "Global trajectory requires at least three waypoints");
    return;
  }

  std::vector<std::vector<double>> candidate;
  candidate.reserve(msg->wpnts.size());
  double previous_s = -std::numeric_limits<double>::infinity();
  for (const auto & wpnt : msg->wpnts) {
    if (!std::isfinite(wpnt.s_m) || !std::isfinite(wpnt.x_m) ||
      !std::isfinite(wpnt.y_m) || !std::isfinite(wpnt.psi_rad) ||
      !std::isfinite(wpnt.kappa_radpm) || wpnt.s_m <= previous_s)
    {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *get_clock(), 1000,
        "Rejecting non-finite or non-monotonic global trajectory");
      return;
    }
    previous_s = wpnt.s_m;
    candidate.push_back({
      wpnt.s_m, wpnt.x_m, wpnt.y_m, wpnt.d_right, wpnt.d_left,
      wpnt.psi_rad, wpnt.kappa_radpm, wpnt.vx_mps, wpnt.ax_mps2});
  }

  const double closing_length = std::hypot(
    candidate.back()[1] - candidate.front()[1],
    candidate.back()[2] - candidate.front()[2]);
  const double candidate_track_length =
    candidate.back()[0] - candidate.front()[0] + closing_length;
  if (!(closing_length > kEpsilon) || !(candidate_track_length > 0.0) ||
    !std::isfinite(candidate_track_length))
  {
    RCLCPP_ERROR(this->get_logger(), "Rejecting global trajectory with invalid closure");
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_trajectory_);
  const bool geometry_changed =
    !has_trajectory_ || global_trajectory_.size() != candidate.size() ||
    std::abs(track_length_ - candidate_track_length) > 1.0e-3 ||
    std::hypot(
      global_trajectory_.front()[1] - candidate.front()[1],
      global_trajectory_.front()[2] - candidate.front()[2]) > 1.0e-3;

  global_trajectory_ = std::move(candidate);
  track_length_ = candidate_track_length;
  has_trajectory_ = true;
  if (geometry_changed) {
    projection_reset_requested_.store(true);
  }

  RCLCPP_INFO_ONCE(
    this->get_logger(), "Received closed global trajectory: %zu waypoints, L=%.3fm",
    global_trajectory_.size(), track_length_);
}

void FrenetFrameNode::Run()
{
  nav_msgs::msg::Odometry odom_copy;
  std::chrono::steady_clock::time_point odom_arrival;
  int64_t source_stamp_ns = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_odom_);
    if (!has_odom_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *get_clock(), 1000, "Waiting for source odometry");
      return;
    }
    odom_copy = current_odom_;
    odom_arrival = odom_arrival_time_;
    source_stamp_ns = odom_source_stamp_ns_;
  }

  std::vector<std::vector<double>> trajectory_copy;
  double track_length = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_trajectory_);
    if (!has_trajectory_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *get_clock(), 1000, "Waiting for global trajectory");
      return;
    }
    trajectory_copy = global_trajectory_;
    track_length = track_length_;
  }

  const double receive_age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - odom_arrival).count();
  const rclcpp::Time source_stamp(
    odom_copy.header.stamp, get_clock()->get_clock_type());
  const double source_age = (get_clock()->now() - source_stamp).seconds();
  if (!std::isfinite(receive_age) || receive_age > odom_receive_timeout_sec_ ||
    !std::isfinite(source_age) || source_age > odom_source_timeout_sec_ ||
    source_age < -odom_future_tolerance_sec_)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *get_clock(), 1000,
      "Odom stale: receive age=%.3fs, source age=%.3fs; Frenet publish stopped",
      receive_age, source_age);
    return;
  }

  // A timer must not turn one localization sample into a stream of apparently
  // fresh samples. Each source odometry timestamp is published at most once.
  if (source_stamp_ns <= last_published_odom_stamp_ns_) {
    return;
  }

  if (projection_reset_requested_.exchange(false)) {
    has_projection_ = false;
    last_segment_index_ = 0;
  }

  if (ProcessFrenetConversion(odom_copy, trajectory_copy, track_length)) {
    last_published_odom_stamp_ns_ = source_stamp_ns;
  }
}

bool FrenetFrameNode::ProcessFrenetConversion(
  const nav_msgs::msg::Odometry & odom_copy,
  const std::vector<std::vector<double>> & trajectory_copy,
  double track_length)
{
  const auto & q_in = odom_copy.pose.pose.orientation;
  const double q_norm = std::sqrt(
    q_in.x * q_in.x + q_in.y * q_in.y + q_in.z * q_in.z + q_in.w * q_in.w);
  if (!(q_norm > kEpsilon) || !std::isfinite(q_norm)) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *get_clock(), 1000, "Rejecting odom with invalid quaternion");
    return false;
  }
  const double qx = q_in.x / q_norm;
  const double qy = q_in.y / q_norm;
  const double qz = q_in.z / q_norm;
  const double qw = q_in.w / q_norm;
  const double vehicle_theta = std::atan2(
    2.0 * (qw * qz + qx * qy),
    1.0 - 2.0 * (qy * qy + qz * qz));

  const FrenetCoordinates frenet_coords = CartesianToFrenet(
    odom_copy.pose.pose.position.x, odom_copy.pose.pose.position.y,
    vehicle_theta, trajectory_copy, track_length);
  if (!frenet_coords.valid) {
    return false;
  }

  double s_dot = 0.0;
  double d_dot = 0.0;
  if (!CartesianVelocityToFrenet(
      odom_copy.twist.twist.linear.x, odom_copy.twist.twist.linear.y,
      vehicle_theta, frenet_coords, s_dot, d_dot))
  {
    return false;
  }

  PublishFrenetOdometry(
    odom_copy.header.stamp, frenet_coords, s_dot, d_dot);
  pitwall::Log("lat_err", frenet_coords.d);
  return true;
}

FrenetFrameNode::FrenetCoordinates FrenetFrameNode::CartesianToFrenet(
  double x, double y, double vehicle_theta,
  const std::vector<std::vector<double>> & trajectory,
  double track_length)
{
  FrenetCoordinates result;
  const int size = static_cast<int>(trajectory.size());
  if (size < 3 || !(track_length > 0.0)) {
    return result;
  }

  struct Candidate
  {
    double score{std::numeric_limits<double>::infinity()};
    double distance_sq{std::numeric_limits<double>::infinity()};
    double s{0.0};
    double unwrapped_s{0.0};
    double d{0.0};
    double psi{0.0};
    double kappa{0.0};
    int segment{-1};
  };

  auto evaluate_segment =
    [&](int segment, bool use_progress_continuity, Candidate & best)
    {
      const int next = (segment + 1) % size;
      const double ax = trajectory[segment][1];
      const double ay = trajectory[segment][2];
      const double bx = trajectory[next][1];
      const double by = trajectory[next][2];
      const double ex = bx - ax;
      const double ey = by - ay;
      const double length_sq = ex * ex + ey * ey;
      if (!(length_sq > kEpsilon) || !std::isfinite(length_sq)) {
        return;
      }

      const double t = std::clamp(
        ((x - ax) * ex + (y - ay) * ey) / length_sq, 0.0, 1.0);
      const double px = ax + t * ex;
      const double py = ay + t * ey;
      const double dx = x - px;
      const double dy = y - py;
      const double distance_sq = dx * dx + dy * dy;
      const double segment_length = std::sqrt(length_sq);
      const double tx = ex / segment_length;
      const double ty = ey / segment_length;
      const double psi = std::atan2(ty, tx);

      double segment_s_span = 0.0;
      if (next > segment) {
        segment_s_span = trajectory[next][0] - trajectory[segment][0];
      } else {
        segment_s_span =
          track_length - (trajectory[segment][0] - trajectory.front()[0]);
      }
      if (!(segment_s_span > kEpsilon) || !std::isfinite(segment_s_span)) {
        return;
      }

      const double wrapped_s = PositiveModulo(
        trajectory[segment][0] + t * segment_s_span, track_length);
      if (!std::isfinite(wrapped_s)) {
        return;
      }
      double unwrapped_s = wrapped_s;
      if (has_projection_) {
        unwrapped_s += std::round(
          (last_unwrapped_s_ - wrapped_s) / track_length) * track_length;
      }

      const double heading_error = NormalizeAngle(vehicle_theta - psi);
      double score =
        distance_sq + projection_heading_weight_m2_ * heading_error * heading_error;
      if (use_progress_continuity && has_projection_) {
        const double progress_error = unwrapped_s - last_unwrapped_s_;
        score += projection_progress_weight_ * progress_error * progress_error;
        const double tangent_change = NormalizeAngle(psi - last_track_psi_);
        score += 0.25 * projection_heading_weight_m2_ *
          tangent_change * tangent_change;
      }

      if (score < best.score) {
        best.score = score;
        best.distance_sq = distance_sq;
        best.s = wrapped_s;
        best.unwrapped_s = unwrapped_s;
        best.d = -ty * dx + tx * dy;
        best.psi = psi;
        best.kappa =
          (1.0 - t) * trajectory[segment][6] + t * trajectory[next][6];
        best.segment = segment;
      }
    };

  Candidate best;
  if (has_projection_) {
    const int search_half_width = std::min(projection_search_segments_, size);
    for (int offset = -search_half_width; offset <= search_half_width; ++offset) {
      int segment = (last_segment_index_ + offset) % size;
      if (segment < 0) {
        segment += size;
      }
      evaluate_segment(segment, true, best);
    }
  } else {
    for (int segment = 0; segment < size; ++segment) {
      evaluate_segment(segment, false, best);
    }
  }

  const double reacquire_distance_sq =
    projection_reacquire_distance_m_ * projection_reacquire_distance_m_;
  // 투영을 아직 못 잡은 상태(시동 직후 / 궤적 재수신)에서는 위 else 가지가
  // 이미 use_progress_continuity=false 로 전 세그먼트를 훑었다. 같은 인자로
  // 한 번 더 돌면 비트 단위로 같은 결과가 나오고(중복 O(N) 스캔),
  // 게다가 "pose jump" 이벤트가 매 시동마다 허위로 찍혔다.
  if (has_projection_ &&
    (best.segment < 0 || best.distance_sq > reacquire_distance_sq))
  {
    Candidate global_best;
    for (int segment = 0; segment < size; ++segment) {
      evaluate_segment(segment, false, global_best);
    }
    if (global_best.segment < 0) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *get_clock(), 1000,
        "No valid segment in global trajectory");
      return result;
    }
    best = global_best;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *get_clock(), 1000,
      "Projection reacquired globally at segment %d (distance %.2fm)",
      best.segment, std::sqrt(best.distance_sq));
    pitwall::Event("pose jump - segment projection reacquired");
  }
  if (best.segment < 0) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *get_clock(), 1000,
      "No valid segment in global trajectory");
    return result;
  }

  result.s = best.s;
  result.d = best.d;
  result.unwrapped_s = best.unwrapped_s;
  result.track_psi = best.psi;
  result.kappa = best.kappa;
  result.segment_index = best.segment;
  result.valid =
    std::isfinite(result.s) && std::isfinite(result.d) &&
    std::isfinite(result.unwrapped_s) && std::isfinite(result.track_psi) &&
    std::isfinite(result.kappa);
  if (!result.valid) {
    return result;
  }

  last_segment_index_ = result.segment_index;
  last_unwrapped_s_ = result.unwrapped_s;
  last_track_psi_ = result.track_psi;
  has_projection_ = true;

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *get_clock(), 1000,
    "[Frenet] XY(%.2f, %.2f) -> s=%.2f, d=%.2f, segment=%d",
    x, y, result.s, result.d, result.segment_index);
  return result;
}

bool FrenetFrameNode::CartesianVelocityToFrenet(
  double vx, double vy, double vehicle_theta,
  const FrenetCoordinates & frenet_coords,
  double & s_dot, double & d_dot)
{
  // nav_msgs/Odometry twist is expressed in child_frame_id. The supported
  // odometry sources use base_link, so rotate it into map before projecting.
  const double vx_map = vx * std::cos(vehicle_theta) - vy * std::sin(vehicle_theta);
  const double vy_map = vx * std::sin(vehicle_theta) + vy * std::cos(vehicle_theta);
  const double tangential_velocity =
    vx_map * std::cos(frenet_coords.track_psi) +
    vy_map * std::sin(frenet_coords.track_psi);
  const double normal_velocity =
    -vx_map * std::sin(frenet_coords.track_psi) +
    vy_map * std::cos(frenet_coords.track_psi);

  // For r(s,d) = reference(s) + d*n(s):
  //   v_tangent = (1 - kappa*d) * ds/dt, v_normal = dd/dt.
  // twist.linear.x in /car_state/frenet/odom therefore means coordinate rate
  // ds/dt, not merely the Cartesian velocity projected onto the tangent.
  const double jacobian = 1.0 - frenet_coords.kappa * frenet_coords.d;
  if (!std::isfinite(jacobian) || jacobian <= min_frenet_jacobian_) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *get_clock(), 1000,
      "Frenet transform singular/unsafe: 1-kappa*d=%.3f (kappa=%.3f, d=%.3f)",
      jacobian, frenet_coords.kappa, frenet_coords.d);
    return false;
  }

  s_dot = tangential_velocity / jacobian;
  d_dot = normal_velocity;
  return std::isfinite(s_dot) && std::isfinite(d_dot);
}

void FrenetFrameNode::PublishFrenetOdometry(
  const builtin_interfaces::msg::Time & timestamp,
  const FrenetCoordinates & frenet_coords,
  double s_dot, double d_dot)
{
  nav_msgs::msg::Odometry frenet_odom;
  // Preserve the localization source timestamp. Downstream freshness checks
  // must observe sensor age, not this node's timer execution time.
  frenet_odom.header.stamp = timestamp;
  frenet_odom.header.frame_id = "frenet";
  frenet_odom.child_frame_id = "base_link_frenet";
  frenet_odom.pose.pose.position.x = frenet_coords.s;
  frenet_odom.pose.pose.position.y = frenet_coords.d;
  frenet_odom.pose.pose.orientation.w = 1.0;
  frenet_odom.twist.twist.linear.x = s_dot;
  frenet_odom.twist.twist.linear.y = d_dot;
  p_frenet_odom_->publish(frenet_odom);

  RCLCPP_DEBUG(
    this->get_logger(), "Published Frenet odom: s=%.3f, d=%.3f, ds/dt=%.3f, dd/dt=%.3f",
    frenet_coords.s, frenet_coords.d, s_dot, d_dot);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrenetFrameNode>("frenet_frame_node"));
  rclcpp::shutdown();
  return 0;
}
