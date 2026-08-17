/*
 * LiDAR 장애물 탐지 노드 (C++).
 *
 * 파이프라인: 전처리 -> TF(laser->target_frame) -> 경계/벽/트랙 필터 ->
 *             adaptive breakpoint 클러스터링 -> 매칭+KF 트래킹 -> 정적/동적 판별
 *
 * 설계상 지켜야 하는 계약 (전부 실차 사고로 얻은 것):
 *  1) frame_id를 라벨만 바꿔치기하지 않는다. target_frame으로 **실제 좌표 변환**을
 *     한 뒤 그 프레임으로 발행한다.
 *  2) boundary_polygon_xy는 target_frame(보통 map) 기준. 고정 지형은 map에 한 번만
 *     정의하면 차량 위치와 무관하게 항상 맞는다.
 *  3) LaserScan의 **원본 시각**으로 TF를 조회한다. 움직이는 차량에서 "최신 TF"로
 *     과거 스캔을 변환하면 장애물 위치가 속도x지연만큼 어긋난다.
 *  4) TF 조회에 실패하면 그 스캔은 스킵한다 (잘못된 좌표로 발행하지 않는다).
 *
 * 실행 모델: TransformListener는 자체 스레드(spin_thread=true)로 /tf를 소비하고,
 * 노드 자신은 MultiThreadedExecutor로 돈다. on_scan은 MutuallyExclusive 그룹이라
 * 여전히 한 번에 하나만 실행되어 트래커 상태가 보호되고, /map·/global_waypoints
 * 콜백은 전용 Reentrant 그룹이라 스캔 큐가 밀려도 굶지 않는다
 * (run_0803_213313: latched /map을 78초간 못 받아 벽 필터가 통째로 죽었던 건).
 */
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "f1tenth_perception_msgs/msg/obstacle.hpp"
#include "f1tenth_perception_msgs/msg/obstacle_array.hpp"
#include "f1tenth_planning_msgs/msg/wpnt_array.hpp"

#include "perception_pkg/classifier.hpp"
#include "perception_pkg/clustering.hpp"
#include "perception_pkg/distance_transform.hpp"
#include "perception_pkg/preprocessing.hpp"
#include "perception_pkg/track_filter.hpp"
#include "perception_pkg/tracker.hpp"
#include "perception_pkg/wall_distance_map.hpp"

namespace perception_pkg
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

std::string trim(const std::string & s)
{
  const auto begin = s.find_first_not_of(" \t\n\r\f\v");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\n\r\f\v");
  return s.substr(begin, end - begin + 1);
}

std::string strip_leading_slash(const std::string & s)
{
  std::size_t i = 0;
  while (i < s.size() && s[i] == '/') {
    ++i;
  }
  return s.substr(i);
}

double stamp_to_sec(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

}  // namespace


class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode()
  : Node("obstacle_detector_node")
  {
    declare_all_parameters();
    read_and_validate_parameters();

    tracker_ = std::make_unique<Tracker>(
      gating_distance_, track_timeout_, reid_distance_, reid_timeout_,
      pos_hist_len_, radius_ratio_max_, dynamic_reacquire_gate_multiplier_,
      dynamic_reacquire_timeout_sec_, dynamic_frenet_reid_timeout_sec_,
      dynamic_frenet_reid_lateral_gate_m_, dynamic_frenet_reid_progress_slack_m_,
      dynamic_frenet_reid_backtrack_m_);

    // ---- TF2 ----
    // spin_thread=true: TransformListener가 전용 executor 스레드로 /tf를 소비한다.
    // (rclpy에서는 같은 옵션이 노드 소유권 다툼으로 조용히 무력화됐지만, rclcpp의
    //  구현은 콜백 그룹만 따로 떼어 add_callback_group 하므로 실제로 병렬로 돈다.)
    // 이 스레드가 있어야 tf_timeout_sec > 0인 lookup이 의미를 갖는다
    // (setUsingDedicatedThread(true)가 여기서 켜진다).
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    // on_scan은 기본(MutuallyExclusive) 그룹 — 트래커 상태를 직렬로 보호한다.
    scan_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions scan_options;
    scan_options.callback_group = scan_cb_group_;
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ObstacleDetectorNode::on_scan, this, std::placeholders::_1), scan_options);
    pub_ = this->create_publisher<f1tenth_perception_msgs::msg::ObstacleArray>(obstacle_topic_, 10);

    // on_scan과 같은 시계(스캔 stamp와 비교 가능)로 대기시간을 잰다.
    start_time_ = this->get_clock()->now().seconds();

    if (use_map_filter_) {
      // map_server는 latched(TRANSIENT_LOCAL) 발행 — 기본 QoS로는 못 받는다.
      // 콜백 그룹을 따로 떼지 않으면 40Hz 스캔 큐가 비지 않는 한 순번을 못 받는다.
      rclcpp::QoS map_qos(rclcpp::KeepLast(1));
      map_qos.reliable().transient_local();
      map_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      rclcpp::SubscriptionOptions map_options;
      map_options.callback_group = map_cb_group_;
      map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, map_qos,
        std::bind(&ObstacleDetectorNode::cb_map, this, std::placeholders::_1), map_options);
      awaiting_map_ = true;
    }

    // 트랙 경계(Frenet) 필터 구독. 맵 필터와 달리 발행을 보류하지 않는다:
    // 벽거리 필터 위에 얹는 2차 기준이라 없어도 파이프라인이 성립하고,
    // 웨이포인트는 전역 플래너가 뜬 뒤에야 나오므로 보류하면 기동 순서에 묶인다.
    if (use_track_filter_) {
      // 전역 플래너가 타이머로 주기 재발행(~2Hz)하므로 latch가 필요 없다.
      rclcpp::QoS wpnt_qos(rclcpp::KeepLast(1));
      wpnt_qos.reliable();
      wpnt_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      rclcpp::SubscriptionOptions wpnt_options;
      wpnt_options.callback_group = wpnt_cb_group_;
      wpnt_sub_ = this->create_subscription<f1tenth_planning_msgs::msg::WpntArray>(
        global_waypoints_topic_, wpnt_qos,
        std::bind(&ObstacleDetectorNode::cb_global_waypoints, this, std::placeholders::_1),
        wpnt_options);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "obstacle_detector_node started: %s -> %s (target_frame=%s)",
      scan_topic_.c_str(), obstacle_topic_.c_str(), target_frame_.c_str());
    if (!boundary_polygon_.empty()) {
      RCLCPP_INFO(
        this->get_logger(),
        "boundary polygon enabled with %zu vertices (in %s frame)",
        boundary_polygon_.size(), target_frame_.c_str());
    }
  }

private:
  // ── 파라미터 ────────────────────────────────────────────────────────────
  void declare_all_parameters()
  {
    this->declare_parameter<double>("range_min", 0.05);
    this->declare_parameter<double>("range_max", 10.0);
    this->declare_parameter<double>("fov_min_deg", -135.0);
    this->declare_parameter<double>("fov_max_deg", 135.0);
    this->declare_parameter<int>("median_kernel", 3);

    // adaptive breakpoint 클러스터링: 임계값이 센서 거리에 따라 커지므로
    // 구 cluster_eps(고정 거리)는 필요 없다.
    this->declare_parameter<double>("lambda_angle_deg", 10.0);
    this->declare_parameter<double>("sigma", 0.015);
    this->declare_parameter<int>("min_cluster_size", 3);
    // 벽에서 떨어진(열린 공간) 클러스터에만 적용하는 완화된 최소 크기.
    // min_cluster_size를 올린 이유는 벽 fringe 억제였는데, 그 대가로 벽에서
    // 먼 작은 물체까지 함께 잘린다 — run_0806_155621에서 폭 0.2m 물체(스캔 4점)가
    // 15프레임 미검출되어 하류에 0.48~0.63m짜리 '가짜 틈'이 생겼다. 실제로 막힌
    // 통로를 열렸다고 판단하면 그 틈으로 회피를 시도하므로 위험하다.
    // 벽에서 open_space_dist_m 이상 떨어진 클러스터는 벽 fringe일 수 없다. 0이면 비활성.
    this->declare_parameter<int>("min_cluster_size_open", 0);
    this->declare_parameter<double>("open_space_dist_m", 0.50);

    this->declare_parameter<double>("gating_distance", 0.5);
    // 크기 일관성 게이트: 트랙이 보던 반경 대비 후보 클러스터 반경의 비가
    // 이 배수를 넘으면 매칭하지 않는다. 0/1 이하면 비활성.
    this->declare_parameter<double>("radius_ratio_max", 4.0);
    this->declare_parameter<int>("track_timeout", 10);

    // 표준 association을 넓히지 않고, primary match에 실패한 이미 확정된
    // 동적 트랙만 마지막 실측 위치로 짧게 재획득한다.
    this->declare_parameter<double>("dynamic_reacquire_gate_multiplier", 1.6);
    this->declare_parameter<double>("dynamic_reacquire_timeout_sec", 0.20);
    // retire 뒤에야 보이는 코너/FOV 이탈은 map 좌표 CV 대신 레이스라인 Frenet
    // s/d로 한 번 더 보수적으로 재식별한다. TrackBounds를 못 받으면 비활성이다.
    this->declare_parameter<double>("dynamic_frenet_reid_timeout_sec", 1.25);
    this->declare_parameter<double>("dynamic_frenet_reid_lateral_gate_m", 0.45);
    this->declare_parameter<double>("dynamic_frenet_reid_progress_slack_m", 0.80);
    this->declare_parameter<double>("dynamic_frenet_reid_backtrack_m", 0.30);


    this->declare_parameter<double>("v_thresh", 0.25);
    this->declare_parameter<int>("n_confirm", 5);
    this->declare_parameter<int>("n_release", 8);
    // 정적/동적 위치분산 근거 (map 프레임)
    this->declare_parameter<double>("static_pos_std", 0.08);
    this->declare_parameter<double>("dynamic_pos_std", 0.20);
    this->declare_parameter<int>("dynamic_release", 40);
    // 재식별(occlusion 후 같은 id 부활) + 미확정 트랙 게이팅
    this->declare_parameter<double>("reid_distance", 0.7);
    this->declare_parameter<double>("reid_timeout_sec", 1.0);
    this->declare_parameter<int>("pos_hist_len", 20);
    this->declare_parameter<int>("min_hits", 3);
    // ── 초 단위 창 (프레임 상수 대체) ────────────────────────────────────
    // 위 track_timeout / n_confirm / n_release / dynamic_release / pos_hist_len
    // 은 전부 프레임 수라, 발행 주기가 바뀌면 뜻하는 벽시계 창이 같이 바뀐다
    // (params.yaml 의 2.33x 재스케일 주석 참조). 실측 출력 주기는 스로틀과
    // LiDAR 지터의 맥놀이라 12.5~20 Hz 사이를 오가므로, 같은 설정에서도
    // ±30% 밴드가 생긴다.
    //
    // 여기서는 창을 **초**로 받고, 노드가 자기 실측 처리 주기로 프레임 수를
    // 매 사이클 환산한다. 0 이하면 비활성 = 위 프레임 값을 그대로 쓴다
    // (기존 튜닝을 그대로 재현하고 싶을 때).
    //
    // ⚠️ min_hits 는 의도적으로 프레임으로 남긴다 — 이건 시간 창이 아니라
    //    "센서 노이즈는 프레임 단위로 뜬다"는 프레임 도메인 게이트다
    //    (params.yaml 의 근거 주석 참조).
    this->declare_parameter<double>("track_timeout_sec", 0.0);
    this->declare_parameter<double>("n_confirm_sec", 0.0);
    this->declare_parameter<double>("n_release_sec", 0.0);
    this->declare_parameter<double>("dynamic_release_sec", 0.0);
    this->declare_parameter<double>("pos_hist_sec", 0.0);
    // 실측 주기 EMA 계수. 0에 가까울수록 느리게 반응한다.
    this->declare_parameter<double>("rate_ema_alpha", 0.05);

    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<std::string>("obstacle_topic", "/perception/obstacles");
    this->declare_parameter<std::string>("target_frame", "map");
    this->declare_parameter<double>("tf_timeout_sec", 0.05);
    this->declare_parameter<bool>("require_source_stamp", true);
    this->declare_parameter<double>("max_source_age_sec", 0.20);
    this->declare_parameter<double>("source_future_tolerance_sec", 0.05);
    // 고속주행에서는 latest TF fallback이 위치를 체계적으로 틀리게 하므로 기본 금지.
    this->declare_parameter<bool>("tf_allow_latest_fallback", false);
    this->declare_parameter<double>("tf_max_age_sec", 0.05);
    // time-travel lookup의 경유(fixed) 프레임. ''(기본) = 비활성.
    this->declare_parameter<std::string>("tf_fixed_frame", "");
    // target_frame(map) 기준. 기본 [0.0]은 "비활성" sentinel (파이썬 판과 동일 규약).
    this->declare_parameter<std::vector<double>>("boundary_polygon_xy", {0.0});

    // ---- 맵 벽 포인트 필터 ----
    this->declare_parameter<bool>("use_map_filter", false);
    this->declare_parameter<std::string>("map_topic", "/map");
    this->declare_parameter<double>("wall_dist_min_m", 0.15);
    this->declare_parameter<double>("map_wait_timeout_sec", 10.0);

    // ---- 레이 일관성 필터 (맵 필터와 같은 WallDistanceMap 을 쓴다) ----
    // 벽거리 필터의 **반대편**을 막는다: 저쪽은 벽에 너무 가까운 점을,
    // 이쪽은 벽보다 뒤에서 온 점을 버린다. 근거는 ray_consistency_mask 주석.
    // use_map_filter 가 켜져 있어야 동작한다 (맵이 없으면 기대거리가 없다).
    this->declare_parameter<bool>("use_ray_filter", false);
    // 기대 벽거리 대비 이만큼 넘어야 기각. 접선 입사 민감도 때문에 작게 잡으면
    // 원거리 실물을 버린다 — 실측 근거는 ray_consistency_mask 주석 (1)번.
    this->declare_parameter<double>("ray_filter_margin_m", 1.5);
    // 안정성 판정용 빔 방향 섭동각 [deg]. 자세 오차 대표값.
    this->declare_parameter<double>("ray_filter_probe_deg", 1.5);
    // 섭동으로 기대거리가 이만큼 이상 흔들리면 그 빔은 판단하지 않는다.
    this->declare_parameter<double>("ray_filter_unstable_m", 0.5);

    // ---- 트랙 경계(Frenet) 필터 ----
    this->declare_parameter<bool>("use_track_filter", false);
    this->declare_parameter<std::string>("global_waypoints_topic", "/global_waypoints");
    this->declare_parameter<double>("track_bound_inflation_m", 0.10);
    this->declare_parameter<double>("max_view_ahead_m", 6.0);
    this->declare_parameter<double>("max_view_behind_m", 2.0);

    // 물리적으로 불가능한 크기의 클러스터 거부. 0.0 = 비활성.
    this->declare_parameter<double>("max_cluster_radius_m", 0.0);
    // 스캔 처리율 상한.
    this->declare_parameter<double>("process_rate_hz", 20.0);
  }

  void read_and_validate_parameters()
  {
    range_min_ = this->get_parameter("range_min").as_double();
    range_max_ = this->get_parameter("range_max").as_double();
    fov_min_ = this->get_parameter("fov_min_deg").as_double() * kPi / 180.0;
    fov_max_ = this->get_parameter("fov_max_deg").as_double() * kPi / 180.0;
    median_kernel_ = static_cast<int>(this->get_parameter("median_kernel").as_int());

    lambda_angle_ = this->get_parameter("lambda_angle_deg").as_double() * kPi / 180.0;
    sigma_ = this->get_parameter("sigma").as_double();
    min_cluster_size_ = static_cast<int>(this->get_parameter("min_cluster_size").as_int());
    min_cluster_size_open_ =
      static_cast<int>(this->get_parameter("min_cluster_size_open").as_int());
    open_space_dist_ = this->get_parameter("open_space_dist_m").as_double();

    v_thresh_ = this->get_parameter("v_thresh").as_double();
    n_confirm_ = static_cast<int>(this->get_parameter("n_confirm").as_int());
    n_release_ = static_cast<int>(this->get_parameter("n_release").as_int());
    static_pos_std_ = this->get_parameter("static_pos_std").as_double();
    dynamic_pos_std_ = this->get_parameter("dynamic_pos_std").as_double();
    dynamic_release_ = static_cast<int>(this->get_parameter("dynamic_release").as_int());
    min_hits_ = static_cast<int>(this->get_parameter("min_hits").as_int());
    gating_distance_ = this->get_parameter("gating_distance").as_double();
    radius_ratio_max_ = this->get_parameter("radius_ratio_max").as_double();
    track_timeout_ = static_cast<int>(this->get_parameter("track_timeout").as_int());
    dynamic_reacquire_gate_multiplier_ =
      this->get_parameter("dynamic_reacquire_gate_multiplier").as_double();
    dynamic_reacquire_timeout_sec_ =
      this->get_parameter("dynamic_reacquire_timeout_sec").as_double();
    dynamic_frenet_reid_timeout_sec_ =
      this->get_parameter("dynamic_frenet_reid_timeout_sec").as_double();
    dynamic_frenet_reid_lateral_gate_m_ =
      this->get_parameter("dynamic_frenet_reid_lateral_gate_m").as_double();
    dynamic_frenet_reid_progress_slack_m_ =
      this->get_parameter("dynamic_frenet_reid_progress_slack_m").as_double();
    dynamic_frenet_reid_backtrack_m_ =
      this->get_parameter("dynamic_frenet_reid_backtrack_m").as_double();
    reid_distance_ = this->get_parameter("reid_distance").as_double();
    reid_timeout_ = this->get_parameter("reid_timeout_sec").as_double();
    pos_hist_len_ = static_cast<int>(this->get_parameter("pos_hist_len").as_int());
    track_timeout_sec_ = this->get_parameter("track_timeout_sec").as_double();
    n_confirm_sec_ = this->get_parameter("n_confirm_sec").as_double();
    n_release_sec_ = this->get_parameter("n_release_sec").as_double();
    dynamic_release_sec_ = this->get_parameter("dynamic_release_sec").as_double();
    pos_hist_sec_ = this->get_parameter("pos_hist_sec").as_double();
    rate_ema_alpha_ = this->get_parameter("rate_ema_alpha").as_double();

    const double rate = this->get_parameter("process_rate_hz").as_double();
    const double tf_timeout_sec = this->get_parameter("tf_timeout_sec").as_double();
    require_source_stamp_ = this->get_parameter("require_source_stamp").as_bool();
    max_source_age_ = this->get_parameter("max_source_age_sec").as_double();
    source_future_tolerance_ = this->get_parameter("source_future_tolerance_sec").as_double();
    tf_allow_latest_fallback_ = this->get_parameter("tf_allow_latest_fallback").as_bool();
    tf_max_age_ = this->get_parameter("tf_max_age_sec").as_double();
    tf_fixed_frame_ = trim(this->get_parameter("tf_fixed_frame").as_string());
    use_map_filter_ = this->get_parameter("use_map_filter").as_bool();
    wall_dist_min_ = this->get_parameter("wall_dist_min_m").as_double();
    max_cluster_radius_ = this->get_parameter("max_cluster_radius_m").as_double();
    map_wait_timeout_ = this->get_parameter("map_wait_timeout_sec").as_double();
    use_ray_filter_ = this->get_parameter("use_ray_filter").as_bool();
    ray_filter_margin_ = this->get_parameter("ray_filter_margin_m").as_double();
    ray_filter_probe_deg_ = this->get_parameter("ray_filter_probe_deg").as_double();
    ray_filter_unstable_ = this->get_parameter("ray_filter_unstable_m").as_double();
    use_track_filter_ = this->get_parameter("use_track_filter").as_bool();
    track_bound_inflation_ = this->get_parameter("track_bound_inflation_m").as_double();
    max_view_ahead_ = this->get_parameter("max_view_ahead_m").as_double();
    max_view_behind_ = this->get_parameter("max_view_behind_m").as_double();

    if (!std::isfinite(track_bound_inflation_) ||
      !(track_bound_inflation_ >= 0.0 && track_bound_inflation_ <= 1.0) ||
      !std::isfinite(max_view_ahead_) ||
      !(max_view_ahead_ >= 0.5 && max_view_ahead_ <= 1000.0) ||
      !std::isfinite(max_view_behind_) ||
      !(max_view_behind_ >= 0.0 && max_view_behind_ <= 1000.0))
    {
      throw std::invalid_argument(
              "invalid track_bound_inflation_m/max_view_ahead_m/max_view_behind_m");
    }
    if (!std::isfinite(wall_dist_min_) || !(wall_dist_min_ >= 0.0 && wall_dist_min_ <= 2.0) ||
      !std::isfinite(max_cluster_radius_) ||
      !(max_cluster_radius_ >= 0.0 && max_cluster_radius_ <= 100.0) ||
      !std::isfinite(map_wait_timeout_) ||
      !(map_wait_timeout_ >= 0.0 && map_wait_timeout_ <= 120.0))
    {
      throw std::invalid_argument(
              "invalid wall_dist_min_m/max_cluster_radius_m/map_wait_timeout_sec");
    }
    // 여유 하한 0.3m: 그 아래로 내리면 접선 입사 빔의 기대거리 흔들림(실측
    // 4~5m 에서 p90 0.70m)이 여유를 넘어 원거리 실물을 버리기 시작한다.
    // 섭동각 상한 10도: 그보다 크면 이웃한 다른 구조물을 재게 되어
    // 안정성 판정 자체가 무의미해진다.
    if (!std::isfinite(ray_filter_margin_) ||
      !(ray_filter_margin_ >= 0.3 && ray_filter_margin_ <= 20.0) ||
      !std::isfinite(ray_filter_probe_deg_) ||
      !(ray_filter_probe_deg_ >= 0.0 && ray_filter_probe_deg_ <= 10.0) ||
      !std::isfinite(ray_filter_unstable_) ||
      !(ray_filter_unstable_ >= 0.0 && ray_filter_unstable_ <= 20.0))
    {
      throw std::invalid_argument(
              "invalid ray_filter_margin_m/ray_filter_probe_deg/ray_filter_unstable_m");
    }
    // 레이 필터는 맵에서 기대 벽거리를 얻는다 — 맵 구독은 use_map_filter 가
    // 건다. 조용히 아무것도 안 하는 대신 기동 때 세운다 (그 조합은 설정 실수다).
    if (use_ray_filter_ && !use_map_filter_) {
      throw std::invalid_argument(
              "use_ray_filter=true requires use_map_filter=true (기대 벽거리 출처가 맵이다)");
    }
    if (!std::isfinite(rate) || !std::isfinite(tf_timeout_sec) ||
      !std::isfinite(max_source_age_) || !std::isfinite(source_future_tolerance_) ||
      !std::isfinite(tf_max_age_) ||
      !(rate >= 1.0 && rate <= 500.0) ||
      !(tf_timeout_sec >= 0.0 && tf_timeout_sec <= 5.0) ||
      !(max_source_age_ >= 0.01 && max_source_age_ <= 5.0) ||
      !(source_future_tolerance_ >= 0.0 && source_future_tolerance_ <= 1.0) ||
      !(tf_max_age_ >= 0.0 && tf_max_age_ <= 1.0))
    {
      throw std::invalid_argument("invalid detector timing/freshness parameters");
    }
    const double continuous[] = {
      range_min_, range_max_, fov_min_, fov_max_, lambda_angle_, sigma_, v_thresh_,
      static_pos_std_, dynamic_pos_std_, gating_distance_, radius_ratio_max_,
      dynamic_reacquire_gate_multiplier_, dynamic_reacquire_timeout_sec_,
      dynamic_frenet_reid_timeout_sec_, dynamic_frenet_reid_lateral_gate_m_,
      dynamic_frenet_reid_progress_slack_m_, dynamic_frenet_reid_backtrack_m_,
      reid_distance_, reid_timeout_,
    };
    const bool all_finite =
      std::all_of(
      std::begin(continuous), std::end(continuous),
      [](double v) {return std::isfinite(v);});
    if (!all_finite ||
      !(range_min_ >= 0.0 && range_min_ < range_max_ && range_max_ <= 100.0) ||
      !(fov_min_ >= -2.0 * kPi && fov_min_ < fov_max_ && fov_max_ <= 2.0 * kPi) ||
      median_kernel_ < 1 || median_kernel_ % 2 == 0 ||
      // lambda는 angle_increment보다 확실히 커야 한다. 작거나 같으면 d_max가
      // 음수가 되어 모든 포인트가 쪼개지고 장애물이 통째로 사라진다. 하한 1deg는
      // 실사용 라이다 증분(UST-10LX 0.25deg)의 4배 여유. 상한 45deg 이상은
      // 사실상 무한 병합이라 의미가 없다.
      !(lambda_angle_ >= 1.0 * kPi / 180.0 && lambda_angle_ <= 45.0 * kPi / 180.0) ||
      !(sigma_ >= 0.0 && sigma_ <= 1.0) ||
      !(min_cluster_size_ >= 1 && min_cluster_size_ <= 100000) ||
      !(min_cluster_size_open_ >= 0 && min_cluster_size_open_ <= min_cluster_size_) ||
      !(open_space_dist_ >= 0.0 && open_space_dist_ <= 10.0) ||
      !(gating_distance_ >= 0.001 && gating_distance_ <= 100.0) ||
      !(radius_ratio_max_ >= 0.0 && radius_ratio_max_ <= 1000.0) ||
      !(track_timeout_ >= 1 && track_timeout_ <= 100000) ||
      !(dynamic_reacquire_gate_multiplier_ >= 1.0 &&
      dynamic_reacquire_gate_multiplier_ <= 10.0) ||
      !(dynamic_reacquire_timeout_sec_ >= 0.0 &&
      dynamic_reacquire_timeout_sec_ <= 5.0) ||
      !(dynamic_frenet_reid_timeout_sec_ >= 0.0 &&
      dynamic_frenet_reid_timeout_sec_ <= 10.0) ||
      !(dynamic_frenet_reid_lateral_gate_m_ >= 0.0 &&
      dynamic_frenet_reid_lateral_gate_m_ <= 5.0) ||
      !(dynamic_frenet_reid_progress_slack_m_ >= 0.0 &&
      dynamic_frenet_reid_progress_slack_m_ <= 20.0) ||
      !(dynamic_frenet_reid_backtrack_m_ >= 0.0 &&
      dynamic_frenet_reid_backtrack_m_ <= 5.0) ||
      !(v_thresh_ >= 0.0 && v_thresh_ <= 100.0) ||
      !(n_confirm_ >= 1 && n_confirm_ <= 100000) ||
      !(n_release_ >= 1 && n_release_ <= 100000) ||
      !(static_pos_std_ >= 0.0 && static_pos_std_ <= dynamic_pos_std_ &&
      dynamic_pos_std_ <= 100.0) ||
      !(dynamic_release_ >= 1 && dynamic_release_ <= 100000) ||
      !(reid_distance_ >= 0.0 && reid_distance_ <= 100.0) ||
      !(reid_timeout_ >= 0.0 && reid_timeout_ <= 60.0) ||
      !(pos_hist_len_ >= 2 && pos_hist_len_ <= 100000) ||
      !(min_hits_ >= 1 && min_hits_ <= track_timeout_))
    {
      throw std::invalid_argument("invalid detector geometry/tracking parameters");
    }

    min_period_ = 1.0 / rate;
    tf_timeout_ = tf2::durationFromSec(tf_timeout_sec);

    target_frame_ = trim(this->get_parameter("target_frame").as_string());
    if (target_frame_.empty()) {
      throw std::invalid_argument("target_frame must not be empty");
    }
    scan_topic_ = trim(this->get_parameter("scan_topic").as_string());
    obstacle_topic_ = trim(this->get_parameter("obstacle_topic").as_string());
    if (scan_topic_.empty() || obstacle_topic_.empty()) {
      throw std::invalid_argument("scan_topic and obstacle_topic must not be empty");
    }
    boundary_polygon_ = parse_boundary_polygon(
      this->get_parameter("boundary_polygon_xy").as_double_array());

    if (use_map_filter_) {
      map_topic_ = trim(this->get_parameter("map_topic").as_string());
      if (map_topic_.empty()) {
        throw std::invalid_argument("map_topic must not be empty when use_map_filter");
      }
    }
    if (use_track_filter_) {
      global_waypoints_topic_ = trim(this->get_parameter("global_waypoints_topic").as_string());
      if (global_waypoints_topic_.empty()) {
        throw std::invalid_argument(
                "global_waypoints_topic must not be empty when use_track_filter");
      }
    }
  }

  /// [x1, y1, x2, y2, ...] 형식 파라미터를 폴리곤으로 변환. 미설정/sentinel이면 빈 값.
  static std::vector<Point2> parse_boundary_polygon(const std::vector<double> & flat_xy)
  {
    std::vector<Point2> polygon;
    if (flat_xy.size() < 2) {   // 미설정/sentinel -> 비활성
      return polygon;
    }
    if (flat_xy.size() < 6 || flat_xy.size() % 2 != 0) {
      throw std::invalid_argument(
              "boundary_polygon_xy must contain an even number of values with at least 3 vertices");
    }
    for (std::size_t i = 0; i < flat_xy.size(); i += 2) {
      if (!std::isfinite(flat_xy[i]) || !std::isfinite(flat_xy[i + 1])) {
        throw std::invalid_argument("boundary_polygon_xy contains non-finite coordinates");
      }
      polygon.push_back({{flat_xy[i], flat_xy[i + 1]}});
    }
    return polygon;
  }

  // ── 콜백: /map ──────────────────────────────────────────────────────────
  /// 노드 수명 중 1회: 셀별 최근접 벽거리 테이블(EDT) 계산.
  void cb_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      if (wall_map_) {
        return;
      }
    }
    const std::string grid_frame = strip_leading_slash(trim(msg->header.frame_id));
    if (!grid_frame.empty() && grid_frame != strip_leading_slash(target_frame_)) {
      // 좌표계가 다르면 EDT 인덱싱이 통째로 틀린다 — 켜지 않는 게 낫다.
      RCLCPP_ERROR(
        this->get_logger(),
        "맵 프레임(%s)이 target_frame(%s)과 다름 — 벽 필터 비활성",
        grid_frame.c_str(), target_frame_.c_str());
      awaiting_map_ = false;   // 영구 불가 — 발행 보류를 풀고 fail-open
      return;
    }
    if (msg->info.width == 0 || msg->info.height == 0 ||
      !std::isfinite(msg->info.resolution) || msg->info.resolution <= 0.0f)
    {
      RCLCPP_ERROR(this->get_logger(), "invalid occupancy grid; 벽 필터 비활성");
      awaiting_map_ = false;
      return;
    }
    const auto & q = msg->info.origin.orientation;
    const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    const double origin_values[] = {
      msg->info.origin.position.x, msg->info.origin.position.y, q.x, q.y, q.z, q.w};
    const bool origin_finite = std::all_of(
      std::begin(origin_values), std::end(origin_values),
      [](double v) {return std::isfinite(v);});
    if (msg->data.size() !=
      static_cast<std::size_t>(msg->info.width) * static_cast<std::size_t>(msg->info.height) ||
      !origin_finite || q_norm < 1e-9)
    {
      RCLCPP_ERROR(this->get_logger(), "occupancy grid data/origin이 malformed; 벽 필터 비활성");
      awaiting_map_ = false;
      return;
    }

    // unknown(-1)도 free로 증명되지 않았으므로 벽으로 취급한다 — 맵 밖 영역의
    // 스캔 리턴(트랙 밖으로 새어나간 빔)이 장애물이 되는 것을 막는다.
    std::vector<uint8_t> occupied(msg->data.size());
    for (std::size_t i = 0; i < msg->data.size(); ++i) {
      occupied[i] = (msg->data[i] < 0 || msg->data[i] >= 65) ? 1 : 0;
    }
    auto wall_map = std::make_shared<WallDistanceMap>();
    wall_map->width = static_cast<int>(msg->info.width);
    wall_map->height = static_cast<int>(msg->info.height);
    wall_map->resolution = static_cast<double>(msg->info.resolution);
    wall_map->dist = euclidean_distance_transform(
      occupied, wall_map->width, wall_map->height, wall_map->resolution);
    if (wall_map->dist.empty()) {
      RCLCPP_ERROR(this->get_logger(), "EDT 계산 실패; 벽 필터 비활성");
      awaiting_map_ = false;
      return;
    }
    wall_map->origin_x = msg->info.origin.position.x;
    wall_map->origin_y = msg->info.origin.position.y;
    const double qx = q.x / q_norm;
    const double qy = q.y / q_norm;
    const double qz = q.z / q_norm;
    const double qw = q.w / q_norm;
    wall_map->origin_yaw =
      std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      wall_map_ = wall_map;
    }
    awaiting_map_ = false;
    RCLCPP_INFO(
      this->get_logger(),
      "맵 벽거리 필터 활성: %dx%d res=%.3f wall_dist_min=%.2fm",
      wall_map->width, wall_map->height, wall_map->resolution, wall_dist_min_);
  }

  // ── 콜백: /global_waypoints ─────────────────────────────────────────────
  /// 전역 레이스라인 수신 -> Frenet 투영/트랙 폭 테이블 갱신.
  ///
  /// 전역 플래너는 주기적으로 재발행하지만 내용은 거의 항상 같으므로 웨이포인트
  /// 개수가 바뀔 때만 다시 만든다. 섹터 재계산으로 개수가 바뀌면 따라간다.
  void cb_global_waypoints(f1tenth_planning_msgs::msg::WpntArray::ConstSharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(bounds_mutex_);
      if (track_bounds_ && track_bounds_->size() == msg->wpnts.size()) {
        return;
      }
    }
    const std::string frame = strip_leading_slash(trim(msg->header.frame_id));
    if (!frame.empty() && frame != strip_leading_slash(target_frame_)) {
      // 좌표계가 다르면 투영이 통째로 틀린다 — 켜지 않는 게 낫다.
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000,
        "웨이포인트 프레임(%s)이 target_frame(%s)과 다름 — 트랙 필터 비활성",
        frame.c_str(), target_frame_.c_str());
      return;
    }
    std::vector<Waypoint> wpnts;
    wpnts.reserve(msg->wpnts.size());
    for (const auto & w : msg->wpnts) {
      wpnts.push_back(Waypoint{w.x_m, w.y_m, w.s_m, w.d_left, w.d_right, w.psi_rad});
    }
    std::shared_ptr<TrackBounds> bounds;
    try {
      bounds = std::make_shared<TrackBounds>(wpnts);
    } catch (const std::invalid_argument & ex) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000,
        "전역 웨이포인트가 트랙 필터에 부적합함 (%s) — 트랙 필터 비활성", ex.what());
      return;
    }
    bool first = false;
    {
      std::lock_guard<std::mutex> lock(bounds_mutex_);
      first = (track_bounds_ == nullptr);
      track_bounds_ = bounds;
    }
    if (first) {
      RCLCPP_INFO(
        this->get_logger(),
        "트랙 경계 필터 활성: %zu점, 트랙길이 %.1fm, inflation=%.2fm, view=%.1f~+%.1fm",
        bounds->size(), bounds->length(), track_bound_inflation_,
        -max_view_behind_, max_view_ahead_);
    }
  }

  // ── 초 단위 창 -> 프레임 수 환산 ───────────────────────────────────────
  // 실측 처리 주기(measured_dt_)가 바뀔 때만 다시 계산한다. 매 프레임
  // 재계산해도 결과는 같지만, 로그를 한 번만 남기기 위해 게이트를 둔다.
  void refresh_frame_windows()
  {
    if (measured_dt_ <= 1e-6) {
      return;
    }
    const bool any_sec = track_timeout_sec_ > 0.0 || n_confirm_sec_ > 0.0 ||
      n_release_sec_ > 0.0 || dynamic_release_sec_ > 0.0 || pos_hist_sec_ > 0.0;
    if (!any_sec) {
      return;
    }
    // dt 가 5% 이상 움직였을 때만 갱신 — EMA 미세 진동으로 창이 떨리지 않게.
    if (last_window_dt_ > 0.0 &&
      std::abs(measured_dt_ - last_window_dt_) < 0.05 * last_window_dt_)
    {
      return;
    }
    last_window_dt_ = measured_dt_;
    auto frames = [&](double sec, int fallback, int lo) {
      if (sec <= 0.0) {
        return fallback;
      }
      const double f = sec / measured_dt_;
      // 100000 은 read_and_validate_parameters 의 상한과 같다.
      return static_cast<int>(std::clamp(std::lround(f),
        static_cast<long>(lo), static_cast<long>(100000)));
    };
    // min_hits 는 프레임 도메인 게이트라 환산 대상이 아니지만,
    // track_timeout >= min_hits 불변식은 여기서도 지켜야 한다.
    track_timeout_ = std::max(
      frames(track_timeout_sec_, track_timeout_, 1), min_hits_);
    n_confirm_ = frames(n_confirm_sec_, n_confirm_, 1);
    n_release_ = frames(n_release_sec_, n_release_, 1);
    dynamic_release_ = frames(dynamic_release_sec_, dynamic_release_, 1);
    pos_hist_len_ = frames(pos_hist_sec_, pos_hist_len_, 2);
    if (tracker_) {
      tracker_->set_track_timeout(track_timeout_);
      tracker_->set_pos_hist_len(pos_hist_len_);
    }
    RCLCPP_INFO(
      this->get_logger(),
      "프레임 창 재환산 (실측 %.1f Hz, dt %.1f ms): track_timeout=%d "
      "n_confirm=%d n_release=%d dynamic_release=%d pos_hist_len=%d",
      1.0 / measured_dt_, 1000.0 * measured_dt_, track_timeout_, n_confirm_,
      n_release_, dynamic_release_, pos_hist_len_);
  }

  // ── 콜백: /scan ────────────────────────────────────────────────────────
  void on_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  {
    const double stamp = stamp_to_sec(msg->header.stamp);
    const std::string source_frame = trim(msg->header.frame_id);
    const double now = this->get_clock()->now().seconds();

    if (source_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "LaserScan has an empty frame_id; skipping.");
      return;
    }
    if (!std::isfinite(stamp) || (require_source_stamp_ && stamp <= 0.0)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "LaserScan has a missing/non-finite source stamp; skipping.");
      return;
    }
    const double age = now - stamp;
    if (!std::isfinite(age) || age > max_source_age_ || age < -source_future_tolerance_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "LaserScan source stamp is outside the freshness window (age=%.3fs); skipping.", age);
      return;
    }
    if (stamp <= last_seen_stamp_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Duplicate or out-of-order LaserScan stamp; skipping.");
      return;
    }
    last_seen_stamp_ = stamp;
    if (stamp - last_processed_ < min_period_) {
      return;
    }
    // last_processed_ 갱신은 TF 변환 성공 이후로 미룬다 (아래 ①.5 끝).
    // 여기서 갱신하면 맵 대기/TF 미준비로 return 하는 프레임까지 스로틀 예산을
    // 먹어서, 조건이 풀린 직후에도 min_period_ 만큼 더 굶는다.

    // 맵 필터가 켜져 있고 맵을 기다리는 중이면 발행 자체를 보류한다.
    // fail-open으로 돌리면 벽 파편이 conf 1.0 장애물로 나간다 (run_0803_213313:
    // 유령 416건, 전부 벽거리 <=0.10m). 하류 spliner는 장애물 프레임을 한 번도
    // 못 받은 동안에는 정상 주행하므로 보류가 정지 라치보다 안전하다.
    // 단, 무한정 보류하면 맵서버가 죽었을 때 회피 자체를 못 하는 상태로 조용히
    // 굳는다 — 제한 시간 뒤에는 시끄럽게 fail-open으로 내려간다.
    if (awaiting_map_) {
      if (now - start_time_ < map_wait_timeout_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "use_map_filter=true, /map 수신 대기 중 — 장애물 발행 보류");
        return;
      }
      awaiting_map_ = false;
      RCLCPP_ERROR(
        this->get_logger(),
        "/map을 %.0fs 내 못 받음 — 벽 필터 없이 발행 시작 "
        "(벽 파편이 유령 장애물로 나갈 수 있음)", map_wait_timeout_);
    }

    if (!scan_is_usable(*msg)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Malformed LaserScan metadata/ranges; skipping.");
      return;
    }

    // ① 전처리 (laser frame 기준, range/FOV/median filter만).
    // point_ranges는 센서 원점 기준 거리 — 아래에서 points를 map frame으로 옮겨도
    // 클러스터링 임계값은 이 값을 써야 한다.
    ScanPoints scan = preprocess_scan(
      msg->ranges, msg->angle_min, msg->angle_increment,
      range_min_, range_max_, fov_min_, fov_max_, median_kernel_);

    // ①.5 TF 변환: laser frame -> target_frame (보통 map).
    // sensor_xy는 트랙 필터의 s 원점(자차 위치, target_frame 기준).
    bool have_sensor_xy = false;
    Point2 sensor_xy{{0.0, 0.0}};
    if (target_frame_ != source_frame) {
      geometry_msgs::msg::TransformStamped tf_stamped;
      if (!lookup_scan_transform(*msg, stamp, source_frame, tf_stamped)) {
        return;
      }
      const auto & t = tf_stamped.transform.translation;
      const auto & q = tf_stamped.transform.rotation;
      const double values[] = {t.x, t.y, q.x, q.y, q.z, q.w};
      const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
      const bool finite = std::all_of(
        std::begin(values), std::end(values), [](double v) {return std::isfinite(v);});
      if (!finite || q_norm < 1e-9) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "TF contains non-finite/invalid values; skipping.");
        return;
      }
      const Rot2 rot = rotation_matrix_2d_from_quaternion(
        q.x / q_norm, q.y / q_norm, q.z / q_norm, q.w / q_norm);
      transform_points(scan.points, t.x, t.y, rot);
      sensor_xy = {{t.x, t.y}};
      have_sensor_xy = true;
    }
    // 여기까지 온 프레임만 실제로 처리된다 — 스로틀 예산은 이제 소비한다.
    // 실측 처리 주기를 EMA 로 추적한다. 초 단위 창(*_sec)이 프레임으로
    // 환산될 때 쓰는 유일한 기준이며, 이 값이 흔들리면 창의 벽시계 길이가
    // 흔들리는 게 아니라 **일정하게 유지**된다 (그게 이 기능의 목적이다).
    if (last_processed_ > -1e17) {
      const double dt = stamp - last_processed_;
      // 스로틀 하한(min_period_)보다 짧거나 1초를 넘는 간격은 이상치로 본다
      // (백 재생 점프, 스캔 드롭). EMA 를 오염시키지 않는다.
      if (dt >= min_period_ * 0.5 && dt <= 1.0) {
        measured_dt_ = (measured_dt_ <= 0.0)
          ? dt
          : (1.0 - rate_ema_alpha_) * measured_dt_ + rate_ema_alpha_ * dt;
      }
    }
    last_processed_ = stamp;
    refresh_frame_windows();

    // ②.5 경계 필터 (target_frame 기준이므로 정적 폴리곤을 그대로 쓸 수 있다).
    if (!boundary_polygon_.empty()) {
      apply_mask(scan.points, scan.ranges, polygon_mask(scan.points, boundary_polygon_));
    }

    // ②.6 맵 벽 포인트 제거 — 반드시 클러스터링 '전'이어야 한다.
    // 클러스터가 만들어진 뒤에 centroid로 판정하면(브릿지의 map filter) 벽 전체가
    // 병합된 클러스터의 centroid가 트랙 한복판이라 통과한다.
    std::shared_ptr<const WallDistanceMap> wall_map;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      wall_map = wall_map_;
    }
    if (use_map_filter_) {
      if (!wall_map) {
        // 여기 도달 = 맵이 '영구 불가'(프레임 불일치/malformed) 또는 대기 만료.
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "use_map_filter=true지만 벽 필터를 쓸 수 없음 — 필터 없이 진행 "
          "(벽 파편이 유령 장애물로 나갈 수 있음)");
      } else {
        apply_mask(scan.points, scan.ranges, wall_point_mask(*wall_map, scan.points));
      }
    }

    // ②.62 레이 일관성 필터 — 맵 벽보다 '뒤'에서 온 빔 제거.
    // ②.6 과 같은 맵을 쓰지만 반대편을 막는다. 반드시 ②.6 **뒤**여야 한다:
    // 벽에 붙은 점이 먼저 빠져야 남은 점만 레이캐스트하면 되고, 순서를 바꾸면
    // 벽 표면 점들이 전부 "기대거리 근처"라 헛돌기만 한다.
    // 자차 위치를 모르면(target_frame == 스캔 프레임) 레이의 원점이 없다.
    if (use_ray_filter_ && wall_map) {
      if (!have_sensor_xy) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "use_ray_filter=true지만 target_frame이 스캔 프레임과 같아 자차 위치를 "
          "알 수 없음 — 레이 필터 미적용");
      } else {
        const std::size_t before = scan.points.size();
        std::size_t unstable = 0;
        apply_mask(
          scan.points, scan.ranges,
          ray_consistency_mask(*wall_map, scan.points, sensor_xy, &unstable));
        const std::size_t dropped = before - scan.points.size();
        if (dropped > 0) {
          RCLCPP_DEBUG_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "레이 일관성 필터: %zu/%zu 포인트 기각 (벽 뒤에서 반사), "
            "접선 입사로 판단 보류 %zu",
            dropped, before, unstable);
        }
      }
    }

    // ②.65 트랙 경계(Frenet) 필터 — ForzaETH laserPointOnTrack 이식.
    // 벽거리(②.6)와 직교하는 기준이라 벽 fringe가 남아도 여기서 걸린다.
    // 웨이포인트 미수신 중에는 적용하지 않는다(fail-open) — 여기서 스캔을 버리면
    // 스트림이 끊겨 spliner가 fail-safe 정지한다.
    std::shared_ptr<const TrackBounds> bounds;
    {
      std::lock_guard<std::mutex> lock(bounds_mutex_);
      bounds = track_bounds_;
    }
    if (use_track_filter_ && bounds) {
      if (!have_sensor_xy) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "use_track_filter=true지만 target_frame이 스캔 프레임과 같아 자차 위치를 "
          "알 수 없음 — 트랙 필터 미적용");
      } else {
        apply_mask(
          scan.points, scan.ranges,
          bounds->on_track_mask(
            scan.points, sensor_xy, track_bound_inflation_,
            max_view_ahead_, max_view_behind_));
      }
    }

    // ② 클러스터링
    if (msg->angle_increment >= lambda_angle_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "angle_increment (%.3f deg) >= lambda_angle_deg (%.3f); clustering will merge "
        "everything. Raise lambda_angle_deg.",
        msg->angle_increment * 180.0 / kPi, lambda_angle_ * 180.0 / kPi);
    }
    // 열린 공간 완화가 켜져 있으면 낮은 임계로 뽑은 뒤 벽 거리로 선별한다.
    // (클러스터링이 내부에서 잘라버리면 되살릴 수 없으므로 낮은 쪽으로 먼저 뽑는다.)
    const bool open_relax =
      min_cluster_size_open_ > 0 && min_cluster_size_open_ < min_cluster_size_ &&
      wall_map != nullptr;
    std::vector<Cluster> clusters = adaptive_cluster_points(
      scan.points, scan.ranges, msg->angle_increment, lambda_angle_, sigma_,
      open_relax ? min_cluster_size_open_ : min_cluster_size_);
    if (open_relax) {
      std::vector<Cluster> selected;
      selected.reserve(clusters.size());
      for (auto & c : clusters) {
        if (static_cast<int>(c.points.size()) >= min_cluster_size_) {
          selected.push_back(std::move(c));
          continue;
        }
        // 작은 클러스터는 벽에서 충분히 떨어져 있을 때만 살린다
        double wd = 0.0;
        if (wall_map->distance_at(c.centroid[0], c.centroid[1], wd) && wd >= open_space_dist_) {
          selected.push_back(std::move(c));
        }
      }
      if (selected.size() != clusters.size()) {
        RCLCPP_DEBUG_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "열린공간 완화: %zu개 중 %zu개 채택 (min %d->%d, 벽거리 >= %.2fm)",
          clusters.size(), selected.size(), min_cluster_size_open_, min_cluster_size_,
          open_space_dist_);
      }
      clusters = std::move(selected);
    }

    // ②.7 물리적으로 불가능한 크기의 클러스터 거부 (맵 필터 2차 방어선).
    // 트래킹 '전'에 버려야 한다 — 트래커에 들어가면 재식별/코스팅으로 살아남아
    // 계속 발행된다.
    if (max_cluster_radius_ > 0.0) {
      double largest = 0.0;
      for (const auto & c : clusters) {
        largest = std::max(largest, c.radius);
      }
      const std::size_t before = clusters.size();
      clusters.erase(
        std::remove_if(
          clusters.begin(), clusters.end(),
          [&](const Cluster & c) {return c.radius > max_cluster_radius_;}),
        clusters.end());
      if (clusters.size() != before) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "%zu개 초대형 클러스터 거부 (max_radius=%.2fm, 최대 %.2fm) — 벽/구조물로 판단",
          before - clusters.size(), max_cluster_radius_, largest);
      }
    }

    // ③④ 매칭 + 칼만필터 트래킹
    const std::vector<TrackPtr> active_tracks = tracker_->step(clusters, stamp, bounds.get());

    // ⑤ 정적/동적 판별
    f1tenth_perception_msgs::msg::ObstacleArray obstacle_array;
    obstacle_array.header = msg->header;
    obstacle_array.header.frame_id = target_frame_;   // 실제로 변환된 프레임과 항상 일치

    for (const auto & track : active_tracks) {
      classify_track(
        *track, track->misses == 0, v_thresh_, n_confirm_, n_release_,
        static_pos_std_, dynamic_pos_std_, dynamic_release_);

      // 관측이 부족한(확정 전) 트랙은 노이즈 클러스터일 수 있어 발행하지 않는다.
      // 단, 한 번 동적 확정된 상대는 occlusion 코스팅 중에도 계속 발행해
      // 다운스트림(GP/회피)이 상대를 놓치지 않게 한다.
      if (track->hits < min_hits_ && !track->ever_dynamic) {
        continue;
      }

      // 코스팅(미관측 외삽) 중이면 신뢰도를 감쇠 — 다운스트림이 측정값과 외삽값을
      // 구분할 수 있게 한다 (misses가 timeout에 가까울수록 0).
      double coast = 1.0;
      if (track->misses > 0) {
        coast = std::max(
          0.0,
          1.0 - static_cast<double>(track->misses) /
          static_cast<double>(tracker_->track_timeout()));
      }

      f1tenth_perception_msgs::msg::Obstacle obs;
      obs.id = track->id;
      obs.position.x = track->kf.x();
      obs.position.y = track->kf.y();
      obs.position.z = 0.0;
      obs.velocity.x = track->kf.vx();
      obs.velocity.y = track->kf.vy();
      obs.velocity.z = 0.0;
      obs.is_dynamic = track->is_dynamic;
      // 이번 프레임에 측정이 매칭됐는지 그대로 넘긴다. confidence 감쇠만으로는
      // 하류가 "저신뢰 측정"과 "외삽"을 구분할 수 없다 — 둘의 대응이 다르다
      // (전자는 여유를 키우면 되지만, 후자는 존재 자체가 불확실하다).
      obs.is_visible = (track->misses == 0);
      obs.confidence = static_cast<float>(track->confidence * coast);
      obs.radius = static_cast<float>(track->radius);
      obstacle_array.obstacles.push_back(obs);
    }

    pub_->publish(obstacle_array);
  }

  // ── 보조 ────────────────────────────────────────────────────────────────
  static bool scan_is_usable(const sensor_msgs::msg::LaserScan & msg)
  {
    const double meta[] = {
      msg.angle_min, msg.angle_max, msg.angle_increment, msg.range_min, msg.range_max};
    if (msg.ranges.empty() ||
      !std::all_of(std::begin(meta), std::end(meta), [](double v) {return std::isfinite(v);}) ||
      msg.angle_increment <= 0.0f ||
      msg.range_min < 0.0f || msg.range_max <= msg.range_min)
    {
      return false;
    }
    // 유효 리턴(유한 양수) 또는 +inf(미반사)가 하나도 없으면 쓸 수 없는 스캔이다.
    return std::any_of(
      msg.ranges.begin(), msg.ranges.end(),
      [](float r) {
        return (std::isfinite(r) && r > 0.0f) || (std::isinf(r) && r > 0.0f);
      });
  }

  /// 맵 벽에서 wall_dist_min 이상 떨어진 포인트의 마스크 (target_frame 좌표).
  ///
  /// **마스크를 돌려주는 이유**: 호출부는 points와 ranges를 1:1로 유지해야 한다.
  /// points만 잘라내면 클러스터링이 길이 불일치로 예외를 던지고, 그 예외는
  /// executor 밖으로 전파되어 노드가 그대로 죽는다
  /// (-> /perception/obstacles 끊김 -> spliner fail-safe 정지).
  std::vector<uint8_t> wall_point_mask(
    const WallDistanceMap & map, const std::vector<Point2> & points) const
  {
    std::vector<uint8_t> keep(points.size(), 0);   // 맵 밖 포인트는 버린다
    for (std::size_t i = 0; i < points.size(); ++i) {
      double d = 0.0;
      if (map.distance_at(points[i][0], points[i][1], d) && d >= wall_dist_min_) {
        keep[i] = 1;
      }
    }
    return keep;
  }

  /// 레이 일관성 필터 — "맵 벽보다 **뒤**에서 돌아온 빔"을 버린다.
  ///
  /// 2D 세계에서 빔은 벽에 막힌다. 벽보다 먼 거리를 보고했다면 그 빔은 경계를
  /// 통과한 것이고, 되돌아온 점은 트랙 밖 구조물이다. 벽거리 필터(wall_dist_min)
  /// 는 이걸 못 잡는다 — 그 점은 정의상 **어느 벽에서도 멀리** 떨어져 있다.
  ///
  /// run_0815_191056 실측 근거 (tools/bag_analysis_0815/diagonal.py):
  ///   · 벽보다 0.4m 이상 뒤에서 온 빔이 중앙 2.5% / p90 12.6% / 최대 52.8%
  ///   · 초과 거리 중앙 2.64m, 넘긴 벽까지 거리 중앙 3.36m
  ///   · 거리 의존: 0.5~1m 에서 0.84% -> 5.5~6m 에서 35.6%
  ///   · 피치 의존: 0.5~1m 에서 -0.05%p(무관), 6~6.5m 에서 +23.8%p
  ///     -> 원인은 차체 피치로 스캔 평면이 떠서 낮은 배리어를 넘는 것.
  ///        (근본 대책은 배리어 높이. 13.7cm 라이다 기준 20cm 배리어는 4m 앞을
  ///         0.90도에 뚫린다 — 브레이킹 피치로 충분한 각도다.)
  ///   · 그렇게 들어온 점들이 코리도를 가로지르는 직선으로 찍힌다
  ///     (t=46.49 실측: 직선 잔차 1.1cm, 길이 2.5m, s=31.2 d=-0.37 -> s=33.2 d=+1.15)
  ///
  /// **여유를 작게 잡으면 안 된다.** 접선 입사(빔이 벽과 거의 나란한 경우)에서는
  /// 자세가 조금만 틀어져도 기대 벽거리가 미터 단위로 튄다. 같은 백에서 자세를
  /// ±0.10m / ±1.5도 흔들었을 때 기대거리가 0.5m 넘게 흔들린 빔이
  ///   4~5m 15.7% · 5~6m 31.7% · 6~7m 53.0%
  /// 였고, 하필 거기가 실물 최초 검출이 몰린 구간이다(중앙 4.82m, 94%가 4m 이상).
  /// 그래서 두 겹으로 막는다:
  ///   (1) 여유 ray_filter_margin_m 를 넉넉히(기본 1.5m). 넘김 초과가 중앙
  ///       2.64m 라 이 여유로도 절반 이상은 여전히 걸린다.
  ///   (2) **안정성 게이트** — 빔 방향을 ±ray_filter_probe_deg 흔들어 기대거리가
  ///       ray_filter_unstable_m 이상 흔들리면 그 빔은 판단하지 않고 살린다.
  ///       (각도 섭동은 거리 R 에서 R*dtheta 의 횡변위를 만든다. 5m·1.5도 = 13cm 로
  ///        위치오차 섭동을 이미 덮으므로 원점 섭동은 따로 두지 않는다.)
  ///
  /// 진짜 장애물은 **언제나 뒤 벽보다 가깝다** — 원리상 이 필터에 걸리지 않는다.
  std::vector<uint8_t> ray_consistency_mask(
    const WallDistanceMap & map, const std::vector<Point2> & points,
    const Point2 & sensor_xy, std::size_t * out_unstable) const
  {
    std::vector<uint8_t> keep(points.size(), 1);   // 기본은 살린다 (fail-open)
    const double probe = ray_filter_probe_deg_ * kPi / 180.0;
    const double cp = std::cos(probe);
    const double sp = std::sin(probe);
    // 레이캐스트 상한. 이보다 먼 벽은 "모른다"로 두고 판단하지 않는다.
    const double max_range = range_max_ + ray_filter_margin_;
    std::size_t unstable = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
      const double dx = points[i][0] - sensor_xy[0];
      const double dy = points[i][1] - sensor_xy[1];
      const double r = std::hypot(dx, dy);
      // 여유보다 가까운 빔은 어떤 기대거리(>=0)로도 기각될 수 없다 — 조기 탈출.
      // 넘김이 근거리에서 0.84% 뿐이라는 실측과도 맞고, 연산의 대부분을 없앤다.
      if (!(r > ray_filter_margin_) || !std::isfinite(r)) {
        continue;
      }
      const double ux = dx / r;
      const double uy = dy / r;
      const double e0 = map.raycast(sensor_xy[0], sensor_xy[1], ux, uy, max_range);
      if (e0 >= max_range) {
        continue;                            // 맵 밖/벽 없음 — 판단 보류
      }
      if (r <= e0 + ray_filter_margin_) {
        continue;                            // 벽 앞에서 온 정상 빔
      }
      // 기각 후보다. 이 빔의 기대거리가 자세 흔들림에 얼마나 민감한지 본다.
      const double e1 =
        map.raycast(sensor_xy[0], sensor_xy[1], cp * ux - sp * uy, sp * ux + cp * uy, max_range);
      const double e2 =
        map.raycast(sensor_xy[0], sensor_xy[1], cp * ux + sp * uy, -sp * ux + cp * uy, max_range);
      const double lo = std::min(e0, std::min(e1, e2));
      const double hi = std::max(e0, std::max(e1, e2));
      if (hi - lo > ray_filter_unstable_) {
        ++unstable;
        continue;                            // 접선 입사 — 판단하지 않는다
      }
      keep[i] = 0;
    }
    if (out_unstable != nullptr) {
      *out_unstable = unstable;
    }
    return keep;
  }

  /// 스캔 원본 시각 기준 TF 조회. 성공하면 true.
  /// 실패 경로마다 파이썬 판과 동일한 폴백 순서/게이트를 따른다.
  bool lookup_scan_transform(
    const sensor_msgs::msg::LaserScan & msg, double stamp,
    const std::string & source_frame, geometry_msgs::msg::TransformStamped & out)
  {
    const tf2::TimePoint scan_time = tf2::TimePoint(
      std::chrono::nanoseconds(rclcpp::Time(msg.header.stamp).nanoseconds()));

    try {
      out = tf_buffer_->lookupTransform(target_frame_, source_frame, scan_time, tf_timeout_);
      return true;   // exact-time 성공 — 추가 게이트 없음 (파이썬 판과 동일)
    } catch (const tf2::TransformException & exact_ex) {
      // 1순위 폴백 — time-travel: laser->fixed(odom)는 스캔 시각(또는 odom 레그가
      // 아직 스캔 시각을 못 덮으면 그 레그의 latest 시각으로 클램프),
      // fixed->target(map)은 latest로 합성. 스캔 도착 시점엔 스캔 시각을 덮는
      // odom TF가 보통 수 ms 뒤에 오므로 클램프가 없으면 이 경로를 거의 못 탄다.
      // 클램프하면 ego-motion 오차가 그 수 ms 공백만큼으로 줄고, map 레그는
      // PF 보정 드리프트만 남는다 (run_0801_175634 재생 실측: 전방 3m 기준
      // p50 29cm -> 5cm, p90 65cm -> 18cm).
      bool have = false;
      if (!tf_fixed_frame_.empty()) {
        try {
          const auto odom_leg = tf_buffer_->lookupTransform(
            tf_fixed_frame_, source_frame, tf2::TimePointZero, tf_timeout_);
          const double odom_stamp = stamp_to_sec(odom_leg.header.stamp);
          if (std::isfinite(odom_stamp) && odom_stamp > 0.0 &&
            std::abs(stamp - odom_stamp) <= tf_max_age_)
          {
            const tf2::TimePoint src_time = (odom_stamp >= stamp) ?
              scan_time :
              tf2::TimePoint(
              std::chrono::nanoseconds(rclcpp::Time(odom_leg.header.stamp).nanoseconds()));
            out = tf_buffer_->lookupTransform(
              target_frame_, tf2::TimePointZero, source_frame, src_time,
              tf_fixed_frame_, tf_timeout_);
            have = true;
          }
        } catch (const tf2::TransformException &) {
          have = false;
        }
      }
      if (!have) {
        if (!tf_allow_latest_fallback_) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Exact-time TF lookup %s -> %s failed: %s. Skipping this scan.",
            source_frame.c_str(), target_frame_.c_str(), exact_ex.what());
          return false;
        }
        try {
          out = tf_buffer_->lookupTransform(
            target_frame_, source_frame, tf2::TimePointZero, tf_timeout_);
        } catch (const tf2::TransformException & latest_ex) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "TF lookup %s -> %s failed: %s. Skipping this scan.",
            source_frame.c_str(), target_frame_.c_str(), latest_ex.what());
          return false;
        }
      }
      const double tf_stamp = stamp_to_sec(out.header.stamp);
      if (!std::isfinite(tf_stamp) || tf_stamp <= 0.0 ||
        std::abs(tf_stamp - stamp) > tf_max_age_)
      {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "TF is not close enough to the LaserScan source stamp (|tf-scan|=%.3fs, "
          "limit=%.3fs); skipping.", std::abs(tf_stamp - stamp), tf_max_age_);
        return false;
      }
      return true;
    }
  }

  // ── 상태 ────────────────────────────────────────────────────────────────
  // 파라미터
  double range_min_{0.0}, range_max_{0.0}, fov_min_{0.0}, fov_max_{0.0};
  int median_kernel_{3};
  double lambda_angle_{0.0}, sigma_{0.0};
  int min_cluster_size_{1}, min_cluster_size_open_{0};
  double open_space_dist_{0.0};
  double v_thresh_{0.0};
  int n_confirm_{1}, n_release_{1};
  double static_pos_std_{0.0}, dynamic_pos_std_{0.0};
  int dynamic_release_{1}, min_hits_{1};
  double gating_distance_{0.0}, radius_ratio_max_{0.0};
  int track_timeout_{1};
  double dynamic_reacquire_gate_multiplier_{1.0};
  double dynamic_reacquire_timeout_sec_{0.0};
  double dynamic_frenet_reid_timeout_sec_{0.0};
  double dynamic_frenet_reid_lateral_gate_m_{0.0};
  double dynamic_frenet_reid_progress_slack_m_{0.0};
  double dynamic_frenet_reid_backtrack_m_{0.0};
  double reid_distance_{0.0}, reid_timeout_{0.0};
  int pos_hist_len_{2};
  // 초 단위 창 (0 이하 = 비활성, 위 프레임 값 사용)
  double track_timeout_sec_{0.0}, n_confirm_sec_{0.0}, n_release_sec_{0.0};
  double dynamic_release_sec_{0.0}, pos_hist_sec_{0.0};
  double rate_ema_alpha_{0.05};
  double measured_dt_{0.0};        ///< 처리 간 실측 dt EMA [s] (0 = 미측정)
  double last_window_dt_{0.0};     ///< 마지막으로 프레임 창을 환산한 dt
  double min_period_{0.0};
  bool require_source_stamp_{true};
  double max_source_age_{0.0}, source_future_tolerance_{0.0};
  bool tf_allow_latest_fallback_{false};
  double tf_max_age_{0.0};
  tf2::Duration tf_timeout_{tf2::durationFromSec(0.0)};
  std::string tf_fixed_frame_, target_frame_, scan_topic_, obstacle_topic_;
  std::string map_topic_, global_waypoints_topic_;
  std::vector<Point2> boundary_polygon_;
  bool use_map_filter_{false};
  double wall_dist_min_{0.0}, max_cluster_radius_{0.0}, map_wait_timeout_{0.0};
  bool use_ray_filter_{false};
  double ray_filter_margin_{0.0}, ray_filter_probe_deg_{0.0}, ray_filter_unstable_{0.0};
  bool use_track_filter_{false};
  double track_bound_inflation_{0.0}, max_view_ahead_{0.0}, max_view_behind_{0.0};

  // 런타임
  double last_processed_{-1e18};
  double last_seen_stamp_{-1e18};
  double start_time_{0.0};
  /// 맵 구독을 걸었지만 아직 첫 맵을 못 받은 상태. true인 동안 on_scan은 발행을
  /// 보류한다 — fail-open으로 벽 파편을 흘리는 것보다 안전하다.
  std::atomic<bool> awaiting_map_{false};

  std::unique_ptr<Tracker> tracker_;
  std::mutex map_mutex_;
  std::shared_ptr<const WallDistanceMap> wall_map_;
  std::mutex bounds_mutex_;
  std::shared_ptr<const TrackBounds> track_bounds_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::CallbackGroup::SharedPtr scan_cb_group_, map_cb_group_, wpnt_cb_group_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr wpnt_sub_;
  rclcpp::Publisher<f1tenth_perception_msgs::msg::ObstacleArray>::SharedPtr pub_;
};

}  // namespace perception_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<perception_pkg::ObstacleDetectorNode>();
    // on_scan(MutuallyExclusive)과 /map·/global_waypoints(Reentrant)를 병렬로 돌린다.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("obstacle_detector_node"), "%s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
