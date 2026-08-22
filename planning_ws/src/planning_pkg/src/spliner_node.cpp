#include "planning_pkg/spliner_node.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>

#include "planning_pkg/pitwall.hpp"

namespace {
// 음수 안전 modulo (트랙 s 좌표 wrap-around 용)
double Mod(double a, double m) {
    double r = std::fmod(a, m);
    return (r < 0.0) ? r + m : r;
}

double WrapToPi(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

}  // namespace

SplinerNode::SplinerNode(const std::string &node_name, const rclcpp::NodeOptions &options)
    : Node(node_name, options) {

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    pitwall::Init(this);

    // Parameters - declare and get values
    this->declare_parameter("spliner/loop_rate_hz", 20.0);
    this->declare_parameter("spliner/lookahead_m", 10.0);
    this->declare_parameter("spliner/evasion_dist", 0.65);
    this->declare_parameter("spliner/obs_traj_tresh", 0.3);
    this->declare_parameter("spliner/spline_bound_mindist", 0.2);
    this->declare_parameter("spliner/pre_apex_dists", std::vector<double>{-4.0, -3.0, -1.5});
    this->declare_parameter("spliner/post_apex_dists", std::vector<double>{2.0, 3.0, 4.0});
    this->declare_parameter("spliner/fixed_pred_time", 0.15);
    this->declare_parameter("spliner/outside_spline_scale", 1.75);
    // 사이클 트레이스 (TraceCycle 주석 참조). 장애물이 있는 사이클에만 찍힌다.
    this->declare_parameter("spliner/trace_cycles", true);
    this->declare_parameter("spliner/side_switch_d_tresh", 0.25);
    this->declare_parameter("spliner/obs_timeout_sec", 0.5);
    this->declare_parameter("spliner/trailing_gap_m", 2.5);
    this->declare_parameter("spliner/trailing_decel", 2.5);
    // 컨트롤러가 wpnt 속도에 곱하는 스케일 (sim tracker 0.5, 실차 1.0).
    // 장애물 실측 속도를 wpnt 속도 단위로 환산할 때 필요 — 안 맞추면
    // 동적 상대 trailing 시 상대보다 느리게 따라가 격차가 벌어진다.
    this->declare_parameter("spliner/cmd_speed_scale", 1.0);
    // 고속 회피 예산 — 상수 거리 대신 그립으로 기하를 정한다
    this->declare_parameter("spliner/blend_lat_acc_max", 3.0);
    this->declare_parameter("spliner/blend_dist_max_m", 8.0);
    this->declare_parameter("spliner/blend_use_spare_grip", true);
    this->declare_parameter("spliner/path_lat_acc_max", 7.0);
    // 0 = path_lat_acc_max 를 그대로 사용(종전 동작과 비트 단위로 동일).
    this->declare_parameter("spliner/offset_grip_cap", true);
    this->declare_parameter("spliner/path_decel_max", 6.0);
    this->declare_parameter("spliner/plant_decel_mps2", 2.38);
    this->declare_parameter("spliner/path_accel_max", 3.0);
    this->declare_parameter("spliner/path_jerk_max", 8.0);
    this->declare_parameter("spliner/planning_delay_sec", 0.10);
    this->declare_parameter("spliner/vehicle_half_length_m", 0.275);
    this->declare_parameter("spliner/vehicle_half_width_m", 0.15);
    this->declare_parameter("spliner/boundary_tracking_margin_m", 0.05);
    // 기본값은 gain 0 = 예전(속도 무관 고정) 동작. 실차 값은 real_car.yaml.
    this->declare_parameter("spliner/boundary_tracking_speed_ref", 3.5);
    this->declare_parameter("spliner/boundary_tracking_speed_gain", 0.0);
    this->declare_parameter("spliner/boundary_tracking_margin_max", 0.05);
    // 벽 여유를 "장애물 옆을 지날 때" 속도로 재기 위한 감속 크레딧 [m/s^2].
    // 0 = 예전 동작(현재 속도 그대로)과 비트 단위로 같다. 근거는
    // ArrivalSpeed()/RequiredBoundaryMarginAt() 주석.
    this->declare_parameter("spliner/boundary_arrival_decel_mps2", 0.0);
    // 트레일링 횡 램프의 장애물 검사 on/off. false = 예전 동작(벽만 검사).
    this->declare_parameter("spliner/trailing_ramp_obstacle_check", false);
    this->declare_parameter("spliner/collision_sigma", 2.0);
    // 정적 장애물의 속도분산 시간 성장 상한 [s] / 회피 기하 계획 속도 하한 [m/s]
    this->declare_parameter("spliner/static_time_uncertainty_cap", 0.3);
    this->declare_parameter("spliner/max_uncertainty_m", 0.25);
    this->declare_parameter("spliner/plan_speed_floor", 1.5);
    this->declare_parameter("spliner/path_check_margin", 0.15);
    this->declare_parameter("spliner/cluster_gap_s_m", 8.0);
    this->declare_parameter("spliner/trailing_engage_dist_m", 8.0);
    this->declare_parameter("spliner/dyn_follow_gap_m", 1.5);
    this->declare_parameter("spliner/dyn_follow_gap_min_m", 0.6);
    this->declare_parameter("spliner/dyn_follow_kp", 1.0);
    this->declare_parameter("spliner/ot_overtake", true);
    this->declare_parameter("spliner/ot_side", std::string("left"));
    this->declare_parameter("spliner/ot_max_gap_m", 2.5);
    this->declare_parameter("spliner/ot_inside_min_kappa", 0.15);
    this->declare_parameter("spliner/trailing_lag_sec", 0.3);
    this->declare_parameter("spliner/trailing_ramp_decel", 2.0);
    this->declare_parameter("spliner/trailing_retry_speed", 1.5);
    this->declare_parameter("spliner/trailing_retry_speed_degraded", 1.5);
    this->declare_parameter("spliner/trailing_vs_filter", true);
    this->declare_parameter("spliner/apex_clearance_eps_m", 0.05);
    this->declare_parameter("spliner/commit_break_min_obs", 3);
    this->declare_parameter("spliner/commit_break_requires_visible", true);
    this->declare_parameter("spliner/commit_min_obs", 7);
    this->declare_parameter("spliner/commit_min_speed_mps", 0.3);
    this->declare_parameter("spliner/commit_release_empty_sec", 1.0);
    this->declare_parameter("spliner/commit_tail_release_lead_m", 3.0);
    this->declare_parameter("spliner/commit_break_confirm_cycles", 2);
    this->declare_parameter("spliner/commit_break_lat_dev_m", 0.40);
    this->declare_parameter("spliner/commit_break_lat_dev_cycles", 3);
    this->declare_parameter("spliner/creep_escape_speed", 0.6);
    // 원호 이탈 (BuildEscapeArc) — 근거는 spliner_node.hpp 선언부 주석.
    this->declare_parameter("spliner/escape_arc_enabled", true);
    this->declare_parameter("spliner/escape_arc_lat_acc", 5.5);
    this->declare_parameter("spliner/escape_arc_min_speed", 1.2);
    this->declare_parameter("spliner/escape_arc_max_heading_rad", 0.70);
    // 정지 워치독 — 근거는 spliner_node.hpp 선언부 주석.
    this->declare_parameter("spliner/standstill_watchdog_sec", 2.0);
    this->declare_parameter("spliner/standstill_speed_eps", 0.20);
    this->declare_parameter("spliner/cluster_gap_d_m", 0.8);
    // 정적/동적 재분류 (unicorn perception/multi_tracking.py 이식)
    this->declare_parameter("spliner/cls_min_nb_meas", 5);
    this->declare_parameter("spliner/cls_min_std", 0.16);
    this->declare_parameter("spliner/cls_max_std", 0.2);
    this->declare_parameter("spliner/cls_vs_reset", 0.1);
    this->declare_parameter("spliner/cls_vs_hold", 10);
    this->declare_parameter("spliner/cls_inherit_dist_m", 0.6);
    this->declare_parameter("spliner/cls_inherit_timeout_sec", 0.5);
    this->declare_parameter("spliner/ever_moved_min_disp_m", 0.5);
    this->declare_parameter("spliner/corridor_block_stop_enabled", true);
    this->declare_parameter("spliner/corridor_block_stop_min_speed", 3.0);
    this->declare_parameter("spliner/corridor_block_stop_hold_sec", 0.15);
    this->declare_parameter("spliner/corridor_block_stop_release_sec", 0.5);
    this->declare_parameter("spliner/sector_gating", true);
    this->declare_parameter("spliner/ot_min_clearance", 0.95);
    this->declare_parameter("spliner/ot_max_kappa", 0.1);
    // 라인 스위치 (기본 maneuver): true면 오프셋 레인 스위치를 1순위로 시도하고
    // 모든 레인이 막힐 때만 DoSpline을 폴백으로 호출한다. false면 기존 동작(DoSpline 우선).
    this->declare_parameter("spliner/use_lane_switch", true);
    this->declare_parameter("spliner/lane_blend_out_m", 5.0);
    this->declare_parameter("spliner/early_switch_kappa_max", 0.10);
    this->declare_parameter("spliner/vehicle_wheelbase_m", 0.33);
    this->declare_parameter("spliner/vehicle_max_steering_rad", 0.3578);
    this->declare_parameter("spliner/vehicle_max_steering_rate_radps", 2.0);
    // 조향 기하 게이트 완화 노브 3종. 기본값은 예전 하드코딩과 동일하므로
    // yaml에 없으면 동작이 바뀌지 않는다. 의미는 헤더 멤버 주석 참조.
    this->declare_parameter("spliner/steer_kappa_scale", 1.0);
    this->declare_parameter("spliner/geom_floor_margin", 0.8);
    this->declare_parameter("spliner/geom_floor_kappa_ratio", 0.15);
    this->declare_parameter("spliner/steer_kappa_reject_ratio", 1.0);
    // d(s) 스플라인 샘플 간격. 예전엔 선언 없는 하드코딩 상수였고, 0이면
    // DoSpline 의 샘플 루프가 무한이 된다 — 선언하는 이상 범위 검증이 필수다.
    this->declare_parameter("spliner/spline_resolution", 0.1);
    // 추월 완료 판정 s 간격: 상대 앞머리가 ego 뒤로 이만큼 벌어져야
    // 라인 복귀 시작 (그 전 복귀는 반 차신도 안 앞선 상대 앞 끼어들기)
    this->declare_parameter("spliner/ot_sector_min_len_m", 8.0);
    this->declare_parameter("spliner/ot_sectors", std::vector<double>{});

    loop_rate_hz_ = this->get_parameter("spliner/loop_rate_hz").as_double();
    lookahead_m_ = this->get_parameter("spliner/lookahead_m").as_double();
    evasion_dist_ = this->get_parameter("spliner/evasion_dist").as_double();
    obs_traj_tresh_ = this->get_parameter("spliner/obs_traj_tresh").as_double();
    spline_bound_mindist_ = this->get_parameter("spliner/spline_bound_mindist").as_double();
    pre_apex_dists_ = this->get_parameter("spliner/pre_apex_dists").as_double_array();
    post_apex_dists_ = this->get_parameter("spliner/post_apex_dists").as_double_array();
    fixed_pred_time_ = this->get_parameter("spliner/fixed_pred_time").as_double();
    outside_spline_scale_ = this->get_parameter("spliner/outside_spline_scale").as_double();
    trace_cycles_ = this->get_parameter("spliner/trace_cycles").as_bool();
    side_switch_d_tresh_ = this->get_parameter("spliner/side_switch_d_tresh").as_double();
    obs_timeout_sec_ = this->get_parameter("spliner/obs_timeout_sec").as_double();
    trailing_gap_m_ = this->get_parameter("spliner/trailing_gap_m").as_double();
    trailing_decel_ = this->get_parameter("spliner/trailing_decel").as_double();
    cmd_speed_scale_ = this->get_parameter("spliner/cmd_speed_scale").as_double();
    blend_lat_acc_max_ = this->get_parameter("spliner/blend_lat_acc_max").as_double();
    blend_dist_max_m_ = this->get_parameter("spliner/blend_dist_max_m").as_double();
    blend_use_spare_grip_ =
        this->get_parameter("spliner/blend_use_spare_grip").as_bool();
    path_lat_acc_max_ = this->get_parameter("spliner/path_lat_acc_max").as_double();
    active_lat_budget_ = path_lat_acc_max_;
    offset_grip_cap_ = this->get_parameter("spliner/offset_grip_cap").as_bool();
    path_decel_max_ = this->get_parameter("spliner/path_decel_max").as_double();
    plant_decel_mps2_ = this->get_parameter("spliner/plant_decel_mps2").as_double();
    path_accel_max_ = this->get_parameter("spliner/path_accel_max").as_double();
    path_jerk_max_ = this->get_parameter("spliner/path_jerk_max").as_double();
    planning_delay_sec_ = this->get_parameter("spliner/planning_delay_sec").as_double();
    vehicle_half_length_m_ =
        this->get_parameter("spliner/vehicle_half_length_m").as_double();
    vehicle_half_width_m_ =
        this->get_parameter("spliner/vehicle_half_width_m").as_double();
    boundary_tracking_margin_m_ =
        this->get_parameter("spliner/boundary_tracking_margin_m").as_double();
    boundary_tracking_speed_ref_ =
        this->get_parameter("spliner/boundary_tracking_speed_ref").as_double();
    boundary_tracking_speed_gain_ =
        this->get_parameter("spliner/boundary_tracking_speed_gain").as_double();
    boundary_tracking_margin_max_ =
        this->get_parameter("spliner/boundary_tracking_margin_max").as_double();
    // 상한이 기본값보다 작으면 속도 의존분이 통째로 죽는다 — 조용히 무효인
    // 노브가 되는 것이 이 파일에서 반복된 사고라 여기서 올려 맞춘다.
    boundary_tracking_margin_max_ =
        std::max(boundary_tracking_margin_max_, boundary_tracking_margin_m_);
    boundary_arrival_decel_mps2_ = this->get_parameter(
        "spliner/boundary_arrival_decel_mps2").as_double();
    trailing_ramp_obstacle_check_ = this->get_parameter(
        "spliner/trailing_ramp_obstacle_check").as_bool();
    collision_sigma_ = this->get_parameter("spliner/collision_sigma").as_double();
    static_time_uncertainty_cap_ =
        this->get_parameter("spliner/static_time_uncertainty_cap").as_double();
    max_uncertainty_m_ =
        this->get_parameter("spliner/max_uncertainty_m").as_double();
    plan_speed_floor_ = this->get_parameter("spliner/plan_speed_floor").as_double();
    path_check_margin_ = this->get_parameter("spliner/path_check_margin").as_double();
    cluster_gap_s_m_ = this->get_parameter("spliner/cluster_gap_s_m").as_double();
    dyn_follow_gap_m_ =
        this->get_parameter("spliner/dyn_follow_gap_m").as_double();
    dyn_follow_gap_min_m_ =
        this->get_parameter("spliner/dyn_follow_gap_min_m").as_double();
    dyn_follow_kp_ = this->get_parameter("spliner/dyn_follow_kp").as_double();
    ot_overtake_enabled_ = this->get_parameter("spliner/ot_overtake").as_bool();
    ot_side_ = this->get_parameter("spliner/ot_side").as_string();
    ot_max_gap_m_ = this->get_parameter("spliner/ot_max_gap_m").as_double();
    ot_inside_min_kappa_ =
        this->get_parameter("spliner/ot_inside_min_kappa").as_double();
    trailing_engage_dist_m_ =
        this->get_parameter("spliner/trailing_engage_dist_m").as_double();
    trailing_lag_sec_ =
        this->get_parameter("spliner/trailing_lag_sec").as_double();
    trailing_ramp_decel_ =
        this->get_parameter("spliner/trailing_ramp_decel").as_double();
    trailing_retry_speed_ =
        this->get_parameter("spliner/trailing_retry_speed").as_double();
    trailing_retry_speed_degraded_ = this->get_parameter(
        "spliner/trailing_retry_speed_degraded").as_double();
    trailing_vs_filter_ =
        this->get_parameter("spliner/trailing_vs_filter").as_bool();
    apex_clearance_eps_m_ =
        this->get_parameter("spliner/apex_clearance_eps_m").as_double();
    commit_break_min_obs_ = static_cast<int>(
        this->get_parameter("spliner/commit_break_min_obs").as_int());
    commit_break_requires_visible_ =
        this->get_parameter("spliner/commit_break_requires_visible").as_bool();
    commit_min_obs_ = static_cast<int>(
        this->get_parameter("spliner/commit_min_obs").as_int());
    commit_min_speed_ =
        this->get_parameter("spliner/commit_min_speed_mps").as_double();
    commit_release_empty_sec_ =
        this->get_parameter("spliner/commit_release_empty_sec").as_double();
    commit_tail_release_lead_m_ = this->get_parameter(
        "spliner/commit_tail_release_lead_m").as_double();
    commit_break_confirm_cycles_ = static_cast<int>(this->get_parameter(
        "spliner/commit_break_confirm_cycles").as_int());
    commit_break_lat_dev_m_ =
        this->get_parameter("spliner/commit_break_lat_dev_m").as_double();
    commit_break_lat_dev_cycles_ = static_cast<int>(this->get_parameter(
        "spliner/commit_break_lat_dev_cycles").as_int());
    creep_escape_speed_ =
        this->get_parameter("spliner/creep_escape_speed").as_double();
    escape_arc_enabled_ =
        this->get_parameter("spliner/escape_arc_enabled").as_bool();
    escape_arc_lat_acc_ =
        this->get_parameter("spliner/escape_arc_lat_acc").as_double();
    escape_arc_min_speed_ =
        this->get_parameter("spliner/escape_arc_min_speed").as_double();
    escape_arc_max_heading_ =
        this->get_parameter("spliner/escape_arc_max_heading_rad").as_double();
    standstill_watchdog_sec_ =
        this->get_parameter("spliner/standstill_watchdog_sec").as_double();
    standstill_speed_eps_ =
        this->get_parameter("spliner/standstill_speed_eps").as_double();
    cluster_gap_d_m_ = this->get_parameter("spliner/cluster_gap_d_m").as_double();
    cls_min_nb_meas_ = static_cast<int>(this->get_parameter("spliner/cls_min_nb_meas").as_int());
    cls_min_std_ = this->get_parameter("spliner/cls_min_std").as_double();
    cls_max_std_ = this->get_parameter("spliner/cls_max_std").as_double();
    cls_vs_reset_ = this->get_parameter("spliner/cls_vs_reset").as_double();
    cls_vs_hold_ =
        static_cast<int>(this->get_parameter("spliner/cls_vs_hold").as_int());
    cls_inherit_dist_m_ =
        this->get_parameter("spliner/cls_inherit_dist_m").as_double();
    cls_inherit_timeout_sec_ =
        this->get_parameter("spliner/cls_inherit_timeout_sec").as_double();
    ever_moved_min_disp_m_ =
        this->get_parameter("spliner/ever_moved_min_disp_m").as_double();
    corridor_block_stop_enabled_ =
        this->get_parameter("spliner/corridor_block_stop_enabled").as_bool();
    corridor_block_stop_min_speed_ = this->get_parameter(
        "spliner/corridor_block_stop_min_speed").as_double();
    corridor_block_stop_hold_sec_ = this->get_parameter(
        "spliner/corridor_block_stop_hold_sec").as_double();
    corridor_block_stop_release_sec_ = this->get_parameter(
        "spliner/corridor_block_stop_release_sec").as_double();
    sector_gating_ = this->get_parameter("spliner/sector_gating").as_bool();
    ot_min_clearance_ = this->get_parameter("spliner/ot_min_clearance").as_double();
    ot_max_kappa_ = this->get_parameter("spliner/ot_max_kappa").as_double();
    use_lane_switch_ = this->get_parameter("spliner/use_lane_switch").as_bool();
    lane_blend_out_m_ = this->get_parameter("spliner/lane_blend_out_m").as_double();
    early_switch_kappa_max_ =
        this->get_parameter("spliner/early_switch_kappa_max").as_double();
    vehicle_wheelbase_m_ =
        this->get_parameter("spliner/vehicle_wheelbase_m").as_double();
    vehicle_max_steering_rad_ =
        this->get_parameter("spliner/vehicle_max_steering_rad").as_double();
    vehicle_max_steering_rate_radps_ =
        this->get_parameter("spliner/vehicle_max_steering_rate_radps").as_double();
    steer_kappa_scale_ =
        this->get_parameter("spliner/steer_kappa_scale").as_double();
    geom_floor_margin_ =
        this->get_parameter("spliner/geom_floor_margin").as_double();
    geom_floor_kappa_ratio_ =
        this->get_parameter("spliner/geom_floor_kappa_ratio").as_double();
    steer_kappa_reject_ratio_ =
        this->get_parameter("spliner/steer_kappa_reject_ratio").as_double();
    spline_resolution_ =
        this->get_parameter("spliner/spline_resolution").as_double();
    ot_sector_min_len_m_ = this->get_parameter("spliner/ot_sector_min_len_m").as_double();
    ot_sectors_param_ = this->get_parameter("spliner/ot_sectors").as_double_array();

    // NaN에 std::max를 적용하면 그대로 남아 모든 비교를 우회할 수 있다.
    // local_trajectory가 이 노드의 heartbeat를 필수 lease로 사용하므로,
    // 잘못된 설정은 명확히 기동 거부 -> downstream timeout 정지로 처리한다.
    const std::vector<double> safety_values = {
        loop_rate_hz_, lookahead_m_, evasion_dist_, obs_traj_tresh_,
        spline_bound_mindist_, fixed_pred_time_, outside_spline_scale_,
        side_switch_d_tresh_, obs_timeout_sec_,
        trailing_gap_m_, trailing_decel_, cmd_speed_scale_,
        blend_lat_acc_max_, blend_dist_max_m_, path_lat_acc_max_,
        path_decel_max_, plant_decel_mps2_, path_accel_max_, path_jerk_max_,
        planning_delay_sec_,
        vehicle_half_length_m_, vehicle_half_width_m_,
        boundary_tracking_margin_m_, boundary_tracking_speed_ref_,
        boundary_tracking_speed_gain_, boundary_tracking_margin_max_,
        boundary_arrival_decel_mps2_,
        collision_sigma_, path_check_margin_,
        static_time_uncertainty_cap_, max_uncertainty_m_, plan_speed_floor_,
        cluster_gap_s_m_, cluster_gap_d_m_,
        trailing_engage_dist_m_, dyn_follow_gap_m_,
        dyn_follow_gap_min_m_, dyn_follow_kp_,
        ot_max_gap_m_, ot_inside_min_kappa_,
        cls_min_std_, cls_max_std_, cls_vs_reset_,
        ot_min_clearance_, ot_max_kappa_, lane_blend_out_m_,
        vehicle_wheelbase_m_,
        vehicle_max_steering_rad_, vehicle_max_steering_rate_radps_,
        steer_kappa_scale_, geom_floor_margin_, geom_floor_kappa_ratio_,
        steer_kappa_reject_ratio_,
        ot_sector_min_len_m_,
        // 아래는 범위 비교(x < lo || x > hi)로만 검사되던 것들이다. NaN은
        // 두 비교 모두 false라 그 검사를 그냥 통과하므로 isfinite 목록에 반드시
        // 있어야 위 주석의 계약("잘못된 설정은 기동 거부")이 실제로 성립한다.
        trailing_lag_sec_, trailing_ramp_decel_, trailing_retry_speed_,
        apex_clearance_eps_m_, creep_escape_speed_,
        escape_arc_lat_acc_, escape_arc_min_speed_, escape_arc_max_heading_,
        // 아래 3개가 정확히 그 누락 사례였다. trailing_retry_speed_degraded_ 는
        // BuildTrailing 에서 목표속도로 **직접 대입**되므로 NaN 이 들어오면
        // 그대로 vx 로 흘러간다 (BuildTrailing 은 ApplySpeedProfile 의
        // 반환값을 쓰지 않아 그쪽 isfinite 검사가 발행을 막지 못한다).
        trailing_retry_speed_degraded_, early_switch_kappa_max_,
        // 래치 타이머가 NaN 이면 (now - since) 비교가 전부 false 라 래치가
        // **영원히 안 걸린다** = 이 기능이 조용히 죽는다.
        corridor_block_stop_min_speed_, corridor_block_stop_hold_sec_,
        corridor_block_stop_release_sec_,
        spline_resolution_,
        // 준정적 추월 접근속도 문턱. NaN 이면 아래 비교가 전부 false 가 되어
        // "모든 동적 상대가 준정적"이 되고, 커밋/retry_d 가 근거 없이 열린다.
        };
    if (!std::all_of(
            safety_values.begin(), safety_values.end(),
            [](double value) { return std::isfinite(value); }) ||
        loop_rate_hz_ < 1.0 || loop_rate_hz_ > 100.0 ||
        lookahead_m_ < 0.5 || lookahead_m_ > 100.0 ||
        evasion_dist_ <= 0.0 || evasion_dist_ > 5.0 ||
        obs_traj_tresh_ < 0.0 || obs_traj_tresh_ > 5.0 ||
        spline_bound_mindist_ < 0.0 || spline_bound_mindist_ > 2.0 ||
        fixed_pred_time_ < 0.0 || fixed_pred_time_ > 2.0 ||
        outside_spline_scale_ <= 0.0 || outside_spline_scale_ > 5.0 ||
        side_switch_d_tresh_ < 0.0 || side_switch_d_tresh_ > 5.0 ||
        obs_timeout_sec_ < 0.05 || obs_timeout_sec_ > 1.0 ||
        trailing_gap_m_ <= 0.0 || trailing_gap_m_ > lookahead_m_ ||
        trailing_decel_ <= 0.0 || trailing_decel_ > 20.0 ||
        cmd_speed_scale_ <= 0.0 || cmd_speed_scale_ > 1.0 ||
        blend_lat_acc_max_ <= 0.0 || blend_lat_acc_max_ > 15.0 ||
        blend_dist_max_m_ <= 0.0 || blend_dist_max_m_ > 50.0 ||
        path_lat_acc_max_ <= 0.0 || path_lat_acc_max_ > 15.0 ||
        // 0 = 비활성(정적과 동일). 켤 때 정적 예산보다 낮으면 의미가 없고
        // (동적이 정적보다 보수적이 될 뿐) 상한은 마찰원 물리 한계다.
        path_accel_max_ <= 0.0 || path_accel_max_ > 20.0 ||
        path_decel_max_ <= 0.0 || path_decel_max_ > 20.0 ||
        plant_decel_mps2_ <= 0.0 || plant_decel_mps2_ > 20.0 ||
        // 벽 여유의 감속 크레딧은 **플랜트가 실제로 낼 수 있는 감속** 이상을
        // 주장할 수 없다. 그 위는 "브레이크가 실제보다 잘 듣는다"는 가정이라
        // 도착 속도를 과소평가하고, 그만큼 벽 예비가 얇아진다.
        boundary_arrival_decel_mps2_ < 0.0 ||
        boundary_arrival_decel_mps2_ > plant_decel_mps2_ ||
        path_jerk_max_ <= 0.0 || path_jerk_max_ > 100.0 ||
        planning_delay_sec_ < 0.0 || planning_delay_sec_ > 1.0 ||
        vehicle_half_length_m_ < 0.05 || vehicle_half_length_m_ > 2.0 ||
        vehicle_half_width_m_ < 0.05 || vehicle_half_width_m_ > 1.0 ||
        boundary_tracking_margin_m_ < 0.0 || boundary_tracking_margin_m_ > 1.0 ||
        collision_sigma_ < 1.0 || collision_sigma_ > 5.0 ||
        static_time_uncertainty_cap_ < 0.0 ||
        static_time_uncertainty_cap_ > 5.0 ||
        max_uncertainty_m_ < 0.0 || max_uncertainty_m_ > 5.0 ||
        plan_speed_floor_ < 0.0 || plan_speed_floor_ > 10.0 ||
        path_check_margin_ < 0.0 || path_check_margin_ > 2.0 ||
        cluster_gap_s_m_ < 0.0 || cluster_gap_s_m_ > lookahead_m_ ||
        cluster_gap_d_m_ < 0.0 || cluster_gap_d_m_ > 5.0 ||
        trailing_engage_dist_m_ <= 0.0 || trailing_engage_dist_m_ > lookahead_m_ ||
        // 목표 간격은 안전 하한보다 커야 하고 lookahead 안이어야 한다.
        dyn_follow_gap_min_m_ < 0.0 ||
        dyn_follow_gap_m_ <= dyn_follow_gap_min_m_ ||
        dyn_follow_gap_m_ > lookahead_m_ ||
        dyn_follow_kp_ <= 0.0 || dyn_follow_kp_ > 5.0 ||
        // 추월 시도 간격은 추종 개입 안이어야 의미가 있다.
        ot_max_gap_m_ <= 0.0 || ot_max_gap_m_ > lookahead_m_ ||
        ot_inside_min_kappa_ < 0.0 || ot_inside_min_kappa_ > 2.0 ||
        (ot_side_ != "left" && ot_side_ != "right" && ot_side_ != "auto") ||
        trailing_lag_sec_ < 0.0 || trailing_lag_sec_ > 2.0 ||
        trailing_ramp_decel_ <= 0.0 || trailing_ramp_decel_ > 20.0 ||
        trailing_retry_speed_ < 0.0 || trailing_retry_speed_ > 10.0 ||
        trailing_retry_speed_degraded_ < 0.0 ||
        trailing_retry_speed_degraded_ > trailing_retry_speed_ ||
        apex_clearance_eps_m_ < 0.0 || apex_clearance_eps_m_ > 0.5 ||
        // 하한을 vs 노이즈 위에 둔다. 실측 vs 표준편차가 p90 0.42 m/s 라
        // 문턱이 그 아래면 노이즈만으로 준정적 판정이 켜진다 (run_0815).
        commit_break_min_obs_ < 1 || commit_break_min_obs_ > 20 ||
        // 만들기 게이트가 깨기 게이트보다 느슨하면 "커밋은 만들 수 있는데
        // 깰 수는 없는" 트랙이 생긴다 — run_0815 에서 전체 트랙의 71%가
        // 그 부류였고 그게 10.7초 유령 커밋의 원인이다. 대칭을 강제한다.
        commit_min_obs_ < commit_break_min_obs_ || commit_min_obs_ > 20 ||
        commit_min_speed_ < 0.0 || commit_min_speed_ > 3.0 ||
        commit_release_empty_sec_ < 0.0 || commit_release_empty_sec_ > 10.0 ||
        // 0 = 꼬리 절단 비활성. 상한은 lookahead — 그보다 큰 여유를 요구하면
        // 절단점이 항상 기동 구간 안으로 들어와 조건이 성립할 수 없다.
        commit_tail_release_lead_m_ < 0.0 ||
        commit_tail_release_lead_m_ > lookahead_m_ ||
        // 1 = 종전 동작(즉시 파기). 상한은 인지 드롭아웃(0812 실측 최대
        // 745 ms)보다 훨씬 짧게 — 확정 지연은 프레임 단위여야 한다.
        commit_break_confirm_cycles_ < 1 || commit_break_confirm_cycles_ > 5 ||
        commit_break_lat_dev_m_ < 0.0 || commit_break_lat_dev_m_ > 2.0 ||
        commit_break_lat_dev_cycles_ < 1 || commit_break_lat_dev_cycles_ > 30 ||
        creep_escape_speed_ < 0.0 || creep_escape_speed_ > 3.0 ||
        cls_min_nb_meas_ < 2 || cls_min_nb_meas_ > 100 ||
        cls_min_std_ <= 0.0 || cls_max_std_ < cls_min_std_ ||
        cls_vs_reset_ < 0.0 || cls_vs_reset_ > 5.0 ||
        cls_vs_hold_ < 1 || cls_vs_hold_ > 1000 ||
        cls_inherit_dist_m_ < 0.0 || cls_inherit_dist_m_ > 5.0 ||
        cls_inherit_timeout_sec_ < 0.0 || cls_inherit_timeout_sec_ > 5.0 ||
        ever_moved_min_disp_m_ < 0.0 || ever_moved_min_disp_m_ > 10.0 ||
        // 0 = 속도 게이트 없음(모든 속도에서 무장). 상한은 레이스라인 최고속.
        corridor_block_stop_min_speed_ < 0.0 ||
        corridor_block_stop_min_speed_ > 10.0 ||
        // 게이트가 트레일링 바닥보다 낮으면 무장해도 바꿀 것이 없다 — 그
        // 아래에서는 바닥이 애초에 구속이 아니다. 오발화만 늘린다.
        (corridor_block_stop_enabled_ &&
         corridor_block_stop_min_speed_ > 0.0 &&
         corridor_block_stop_min_speed_ < trailing_retry_speed_) ||
        // hold 0 은 1프레임 유령까지 래치한다. 상한은 물리 예산(0.14~0.4s,
        // [[planning-full-block-estop-0821]]) 밖으로 못 나가게.
        corridor_block_stop_hold_sec_ <= 0.0 ||
        corridor_block_stop_hold_sec_ > 0.5 ||
        // 해제가 hold 보다 짧으면 래치가 채터링한다.
        corridor_block_stop_release_sec_ < corridor_block_stop_hold_sec_ ||
        corridor_block_stop_release_sec_ > 5.0 ||
        ot_min_clearance_ <= 0.0 || ot_min_clearance_ > 10.0 ||
        ot_max_kappa_ <= 0.0 || ot_max_kappa_ > 5.0 ||
        lane_blend_out_m_ <= 0.0 || lane_blend_out_m_ > lookahead_m_ ||
        early_switch_kappa_max_ < 0.0 || early_switch_kappa_max_ > 5.0 ||
        vehicle_wheelbase_m_ < 0.1 || vehicle_wheelbase_m_ > 1.0 ||
        vehicle_max_steering_rad_ <= 0.0 || vehicle_max_steering_rad_ > 0.8 ||
        vehicle_max_steering_rate_radps_ <= 0.0 ||
        vehicle_max_steering_rate_radps_ > 10.0 ||
        // 상한 1.5 = 최소 선회반경 0.58 m 주장. 측정 기하값이 0.87 m 이므로
        // (TUNING_STATUS §1) 그보다 33% 안쪽까지가 슬립/Ackermann 내륜각으로
        // 변호할 수 있는 최대치다. 그 위는 설정 실수로 본다.
        steer_kappa_scale_ < 0.5 || steer_kappa_scale_ > 1.5 ||
        // 1.0 초과는 "근사식보다 실제 피크가 작다"는 뜻이라 근거가 없다.
        geom_floor_margin_ <= 0.0 || geom_floor_margin_ > 1.0 ||
        geom_floor_kappa_ratio_ <= 0.0 || geom_floor_kappa_ratio_ > 1.0 ||
        // 1.0 미만이면 조향 한계 안쪽의 정상 경로를 버리게 된다.
        steer_kappa_reject_ratio_ < 1.0 || steer_kappa_reject_ratio_ > 1.5 ||
        // 0이면 DoSpline의 샘플 루프가 끝나지 않는다.
        spline_resolution_ < 0.01 || spline_resolution_ > 1.0 ||
        ot_sector_min_len_m_ <= 0.0 || ot_sector_min_len_m_ > lookahead_m_ ||
        // 1.0 초과 = 섹터 내 부스트. 예전 상한 1.0은 헤더/params 주석이
        // 약속한 "섹터 안 공격 모드"를 원리적으로 불가능하게 만들었다.
        // 실제 상한은 gb_vmax_(레이스라인 최고속)이므로 여기서는 2.0까지 허용.
        // 원호는 마지막 회피 수단이라 통상 전환보다 큰 횡가속을 쓰지만,
        // 경로 곡률 검증(path_lat_acc_max_)이 쓰는 그립 예산을 넘으면
        // 자기가 만든 경로를 자기 검증기가 거부한다.
        escape_arc_lat_acc_ <= 0.0 || escape_arc_lat_acc_ > path_lat_acc_max_ ||
        escape_arc_min_speed_ < 0.0 || escape_arc_min_speed_ > 5.0 ||
        // 상한 1.0 rad(57도). 그 이상 남기면 복귀가 트랙 폭을 넘는다.
        escape_arc_max_heading_ <= 0.0 || escape_arc_max_heading_ > 1.0 ||
        // 워치독이 0.5s 미만이면 정지 판정 전에 노이즈로 발동하고,
        // eps 가 크면 저속 주행이 전부 "정지"가 된다.
        standstill_watchdog_sec_ < 0.5 || standstill_watchdog_sec_ > 30.0 ||
        standstill_speed_eps_ < 0.02 || standstill_speed_eps_ > 1.0) {
        throw std::runtime_error(
            "Invalid/non-finite spliner safety parameter; planner heartbeat disabled");
    }
    // 절점 거리는 반드시 순증가여야 한다 (스플라인 x축). 어긋나면 DoSpline이
    // 매번 실패해 회피가 통째로 죽으므로 부팅 시점에 알린다.
    auto strictly_increasing = [](const std::vector<double> &v) {
        for (size_t i = 0; i + 1 < v.size(); ++i) {
            if (!(v[i + 1] > v[i])) return false;
        }
        return true;
    };
    const auto all_finite = [](const std::vector<double> &values) {
        return std::all_of(values.begin(), values.end(),
            [](double value) { return std::isfinite(value); });
    };
    if (pre_apex_dists_.empty() || !strictly_increasing(pre_apex_dists_) ||
        !all_finite(pre_apex_dists_) || !(pre_apex_dists_.back() < 0.0) ||
        post_apex_dists_.empty() || !strictly_increasing(post_apex_dists_) ||
        !all_finite(post_apex_dists_) || !(post_apex_dists_.front() > 0.0) ||
        ot_sectors_param_.size() % 2U != 0U ||
        !all_finite(ot_sectors_param_)) {
        throw std::runtime_error(
            "Invalid spliner apex/sector sequence; planner heartbeat disabled");
    }

    RCLCPP_INFO(this->get_logger(),
        "spliner params: lookahead=%.1fm evasion_dist=%.2fm obs_traj_tresh=%.2fm",
        lookahead_m_, evasion_dist_, obs_traj_tresh_);
    // 2026-08-19 에 들어온 두 스위치는 트랙에서 되돌릴 일이 생긴다. 되돌렸는지
    // 를 ros2 param get 없이 launch 로그 한 줄로 확인할 수 있어야 한다
    // (둘 다 off = 이 커밋 이전과 비트 단위로 같은 동작).
    RCLCPP_INFO(this->get_logger(),
        "spliner 회피 스위치: trailing_ramp_obstacle_check=%s "
        "boundary_arrival_decel=%.2f m/s^2 %s",
        trailing_ramp_obstacle_check_ ? "ON" : "off",
        boundary_arrival_decel_mps2_,
        (!trailing_ramp_obstacle_check_ && boundary_arrival_decel_mps2_ <= 0.0)
            ? "(둘 다 off — 2026-08-19 이전 동작)" : "");


    // Subscriber init
    // 실차: perception 팀 원시 토픽(/perception/obstacles, f1tenth_perception_msgs,
    // laser frame)을 perception_obstacle_bridge_node가 frenet으로 변환해
    // /perception/obstacles_frenet으로 재발행 -> real_car.yaml이 이 토픽으로 override.
    // sim은 virtual_obstacle_node가 f1tenth_planning_msgs 타입을 기본 토픽에 직접 발행.
    this->declare_parameter("spliner/obstacle_topic", "/perception/obstacles");
    const auto obstacle_topic =
        this->get_parameter("spliner/obstacle_topic").as_string();
    s_obstacles_ = this->create_subscription<f1tenth_planning_msgs::msg::ObstacleArray>(
        obstacle_topic, qos_profile,
        std::bind(&SplinerNode::CallbackObstacles, this, std::placeholders::_1));

    s_frenet_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/car_state/frenet/odom", qos_profile,
        std::bind(&SplinerNode::CallbackFrenetOdom, this, std::placeholders::_1));

    s_global_waypoints_ = this->create_subscription<f1tenth_planning_msgs::msg::WpntArray>(
        "/global_waypoints", qos_profile,
        std::bind(&SplinerNode::CallbackGlobalWaypoints, this, std::placeholders::_1));

    // Publisher init
    p_evasion_wpnts_ = this->create_publisher<f1tenth_planning_msgs::msg::WpntArray>(
        "/planner/avoidance/otwpnts", qos_profile);
    p_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/planner/avoidance/markers", qos_profile);
    p_sector_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/planner/ot_sectors/markers",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
    p_escape_active_ = this->create_publisher<std_msgs::msg::Bool>(
        "/planner/escape_active", qos_profile);
    p_corridor_blocked_ = this->create_publisher<std_msgs::msg::Bool>(
        "/planner/corridor_blocked", qos_profile);
    p_evade_committed_ = this->create_publisher<std_msgs::msg::Bool>(
        "/planner/avoidance/evade_committed", qos_profile);

    // Timer init
    t_run_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / loop_rate_hz_),
        [this]() { this->Run(); });

    RCLCPP_INFO(this->get_logger(), "SplinerNode initialized successfully");
}

void SplinerNode::CallbackObstacles(const f1tenth_planning_msgs::msg::ObstacleArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_obstacles_);
    obstacles_ = *msg;
    obstacles_stamp_ = this->now();
    has_obstacles_ = true;
}

void SplinerNode::CallbackFrenetOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // (mutex_frenet_odom_ 제거: cur_s_/cur_d_/cur_vs_ 는 Run() 이하 수십 곳에서
    //  잠금 없이 읽혔기 때문에 이 락은 상호배제를 만들지 못했다. main() 이
    //  rclcpp::spin(단일 스레드 executor)이라 콜백과 타이머가 동시 실행되지도
    //  않는다. MultiThreadedExecutor 로 바꾸려면 읽기 쪽 스냅샷부터 만들 것.)
    cur_s_ = msg->pose.pose.position.x;
    cur_d_ = msg->pose.pose.position.y;
    cur_vs_ = msg->twist.twist.linear.x;
    has_frenet_odom_ = true;
}

void SplinerNode::CallbackGlobalWaypoints(const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_gb_wpnts_);
    gb_wpnts_ = msg->wpnts;
    if (gb_wpnts_.size() >= 2) {
        wpnt_dist_ = gb_wpnts_[1].s_m - gb_wpnts_[0].s_m;
        if (!(wpnt_dist_ > 1e-6)) {
            // s가 단조증가하지 않는 CSV. wpnt_dist_는 s->index 변환의 분모라
            // 0이면 전 노드가 NaN 궤적을 발행한다 — 라인을 통째로 거부한다.
            RCLCPP_ERROR(this->get_logger(),
                "global waypoints의 s 간격이 %.6f (>0이어야 함) — 라인 무시",
                wpnt_dist_);
            has_gb_wpnts_ = false;
            gb_wpnts_.clear();
            return;
        }
        // 트랙 길이 = (마지막 s - 첫 s) + 폐합 구간 길이.
        // local_trajectory_node::ComputeTrackLength / frenet_frame_node /
        // perception 브릿지가 쓰는 정의와 **같은 식**이어야 wrap 근처에서
        // s 가 어긋나지 않는다. 예전엔 여기만 back_s + 균일 간격을 썼는데,
        // ValidateTrajectory 가 점간격 비균일을 1.25배까지, 폐합 구간을
        // 최대 간격의 2.5배까지 허용하므로 새 맵에서 두 값이 갈라질 수 있다
        // (현행 CSV 에서는 1e-8 수준으로 일치한다).
        const double closure = std::hypot(
            gb_wpnts_.front().x_m - gb_wpnts_.back().x_m,
            gb_wpnts_.front().y_m - gb_wpnts_.back().y_m);
        track_len_ = gb_wpnts_.back().s_m - gb_wpnts_.front().s_m + closure;
        gb_vmax_ = 0.0;
        for (const auto &w : gb_wpnts_) {
            gb_vmax_ = std::max(gb_vmax_, w.vx_mps);
        }
        // ot_sectors_ 는 **이 배열의 인덱스 기준**으로 만들어진 파생물이다.
        // 예전에는 첫 수신에만 만들고 이후 재발행에는 gb_wpnts_ 만 갈아끼웠는데,
        // 그러면 섹터가 옛 s 좌표를 가리킨 채 남는다. 레이아웃이 바뀌면
        // 다시 만든다. (SP 라인 파생물은 2026-08-22 에 제거됐다.)
        const bool layout_changed =
            !has_gb_wpnts_ || gb_wpnts_.size() != gb_layout_size_ ||
            std::abs(track_len_ - gb_layout_len_) > 1e-6;
        gb_layout_size_ = gb_wpnts_.size();
        gb_layout_len_ = track_len_;
        has_gb_wpnts_ = true;
        if (layout_changed) {
            ComputeOtSectors();
            PublishSectorMarkers();
        }
    }
}

void SplinerNode::ComputeOtSectors() {
    ot_sectors_.clear();
    // 수동 지정이 있으면 그대로 사용
    if (ot_sectors_param_.size() >= 2) {
        for (size_t i = 0; i + 1 < ot_sectors_param_.size(); i += 2) {
            ot_sectors_.emplace_back(ot_sectors_param_[i], ot_sectors_param_[i + 1]);
        }
        RCLCPP_INFO(this->get_logger(), "OT sectors: %zu manual", ot_sectors_.size());
        return;
    }

    // 자동 산출: 한쪽 벽 여유가 ot_min_clearance 이상이고 코너가 아닌
    // (|kappa| <= ot_max_kappa) 연속 구간. 코너 아웃라인 추월은 경로가
    // 길어져 페이스 우위가 상쇄되므로 섹터에서 제외 — 코너에선 TRAILING으로
    // 붙었다가 직선에서 승부한다.
    int n = static_cast<int>(gb_wpnts_.size());
    std::vector<bool> feasible(n);
    for (int i = 0; i < n; ++i) {
        double clearance = std::max(std::abs(gb_wpnts_[i].d_left),
                                    std::abs(gb_wpnts_[i].d_right));
        feasible[i] = clearance >= ot_min_clearance_ &&
                      std::abs(gb_wpnts_[i].kappa_radpm) <= ot_max_kappa_;
    }

    // wrap을 고려한 연속 run 추출: s=0을 걸치는 run은 시작 인덱스를 회전시켜 처리
    int start = 0;
    while (start < n && feasible[start]) {
        ++start;
    }
    if (start == n) {
        // 트랙 전체가 feasible -> 전 구간 섹터 하나
        ot_sectors_.emplace_back(0.0, track_len_);
    } else {
        int run_begin = -1;
        for (int k = 0; k <= n; ++k) {
            int idx = (start + k) % n;
            bool f = (k < n) ? feasible[idx] : false;
            if (f && run_begin < 0) {
                run_begin = idx;
            } else if (!f && run_begin >= 0) {
                int run_end = (start + k - 1) % n;
                double len = Mod((run_end - run_begin) * wpnt_dist_, track_len_);
                if (len >= ot_sector_min_len_m_) {
                    ot_sectors_.emplace_back(gb_wpnts_[run_begin].s_m,
                                             gb_wpnts_[run_end].s_m);
                }
                run_begin = -1;
            }
        }
    }

    double total = 0.0;
    for (const auto &[s0, s1] : ot_sectors_) {
        total += Mod(s1 - s0, track_len_);
    }
    double pct = 100.0 * total / std::max(track_len_, 0.1);
    RCLCPP_INFO(this->get_logger(),
        "OT sectors: %zu auto (%.0f%% of track, clearance>=%.2fm, min_len=%.0fm)",
        ot_sectors_.size(), pct, ot_min_clearance_, ot_sector_min_len_m_);
    char buf[80];
    std::snprintf(buf, sizeof(buf), "추월 섹터 %zu개 산출 (트랙 %.0f%%)",
                  ot_sectors_.size(), pct);
    pitwall::Event(buf);
}

bool SplinerNode::InOtSector(double s) const {
    for (const auto &[s0, s1] : ot_sectors_) {
        if (s0 <= s1) {
            if (s >= s0 && s <= s1) return true;
        } else {
            if (s >= s0 || s <= s1) return true;  // s=0 wrap 구간
        }
    }
    return false;
}

double SplinerNode::StraightEndAhead(double s) const {
    for (const auto &[s0, s1] : ot_sectors_) {
        const bool inside = (s0 <= s1) ? (s >= s0 && s <= s1)
                                       : (s >= s0 || s <= s1);
        if (inside) {
            return s1;
        }
    }
    return -1.0;
}

std::string SplinerNode::CornerInsideSide(double s_from, double span) const {
    const int gb_size = static_cast<int>(gb_wpnts_.size());
    if (gb_size < 2 || !(track_len_ > 0.0) || !(span > 0.0)) {
        return "";
    }
    const int steps = std::max(1, static_cast<int>(span / std::max(wpnt_dist_, 0.01)));
    double sum = 0.0;
    const int start = static_cast<int>(Mod(s_from, track_len_) / wpnt_dist_) % gb_size;
    for (int i = 0; i < steps; ++i) {
        sum += gb_wpnts_[(start + i) % gb_size].kappa_radpm;
    }
    const double mean = sum / static_cast<double>(steps);
    if (!std::isfinite(mean) || std::abs(mean) < ot_inside_min_kappa_) {
        return "";
    }
    // 좌선회(kappa > 0)의 안쪽은 왼쪽(d > 0).
    return mean > 0.0 ? "left" : "right";
}

void SplinerNode::PublishSectorMarkers() {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);

    int id = 0;
    for (const auto &[s0, s1] : ot_sectors_) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = this->now();
        m.ns = "ot_sectors";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.scale.x = 0.25;
        m.color.g = 1.0;
        m.color.a = 0.35;
        m.pose.orientation.w = 1.0;
        double len = Mod(s1 - s0, track_len_);
        if (len <= 0.0) len = track_len_;
        for (double ds = 0.0; ds <= len; ds += 0.5) {
            auto [x, y] = FrenetToCartesian(Mod(s0 + ds, track_len_), 0.0);
            geometry_msgs::msg::Point p;
            p.x = x;
            p.y = y;
            p.z = 0.02;
            m.points.push_back(p);
        }
        arr.markers.push_back(m);
    }
    p_sector_markers_->publish(arr);
}

void SplinerNode::Run() {
    if (!has_gb_wpnts_ || !has_frenet_odom_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
                             "Waiting for global waypoints / frenet odom...");
        return;
    }

    // 경계 abort는 사이클마다 재발생하므로 상승엣지에서만 알린다 (아래 참조)
    boundary_abort_now_ = false;
    // 회피 실패 사유 플래그 (Tier3에서 "정지 vs 감속 재시도"를 가른다)
    lane_switch_geom_infeasible_ = false;
    free_spline_geom_infeasible_ = false;
    // 준정적 판정은 **이번 사이클의 최근접 장애물**에 대한 값이다. 장애물이
    // 사라진 사이클에도 반드시 내려가야 한다 — 안 그러면 true 로 굳어서
    // 다음 장애물이 근거 없이 커밋/retry_d 를 물려받는다 (try_evade_now 와
    // 같은 이유. 그쪽은 굳는 바람에 상승엣지를 영영 놓친 전례가 있다).
    // ── 추월 속도 배율도 매 사이클 내려야 한다 (2026-08-19) ──────────────
    // 이 값은 장애물 분기 안에서만 대입된다. 장애물이 **없는** 사이클에는
    // 그 대입이 실행되지 않으므로, 리셋이 없으면 직전 동적 사이클의 배율이
    // 그대로 남는다. ot_speed_scale 이 1.0 이던 동안에는 남아도 결과가 같아
    // 드러나지 않았지만, 1.25 로 켠 뒤에는 **정적 경로/커밋 재생에 25% 부스트가
    // 새는 통로**가 된다.
    // ── 그립 예산도 같은 이유로 매 사이클 내려야 한다 (2026-08-20) ───────
    // 이 값은 아래 장애물 분기에서만 올라간다. 리셋이 없으면 직전 동적
    // 사이클의 높은 예산이 장애물 없는 사이클·정적 회피·커밋 재생으로
    // 그대로 새어 나가, 정적 기하가 튜닝된 예산 밖에서 계산된다.
    active_lat_budget_ = path_lat_acc_max_;
    cycle_rejects_.clear();

    // ── 정지 워치독 (hpp 선언부 주석) ────────────────────────────────────
    // v=0 에서 나갈 수단을 하나는 남긴다: 정지가 이만큼 이어지면
    // BuildEscapeArc 의 is_visible 요구를 해제한다. 기하·충돌·경계 검증은
    // 전부 그대로라, 성립하는 원호가 없으면 지금과 똑같이 서 있는다.
    if (std::abs(cur_vs_) > standstill_speed_eps_) {
        standstill_since_valid_ = false;
        if (standstill_override_) {
            standstill_override_ = false;   // 움직이기 시작 — 정상 게이트 복원
        }
    } else if (!standstill_since_valid_) {
        standstill_since_ = this->now();
        standstill_since_valid_ = true;
    } else if (!standstill_override_ &&
               (this->now() - standstill_since_).seconds() >=
                   standstill_watchdog_sec_) {
        standstill_override_ = true;
        RCLCPP_WARN(this->get_logger(),
            "정지 워치독: %.1fs 정지 — escape 가시성 게이트 해제 "
            "(s=%.2f, 기하/충돌 검증은 유지)",
            standstill_watchdog_sec_, cur_s_);
        pitwall::Event("정지 워치독 — escape 가시성 게이트 해제");
    }

    f1tenth_planning_msgs::msg::ObstacleArray obs_copy;
    bool perception_stale = false;
    {
        std::lock_guard<std::mutex> lock(mutex_obstacles_);
        // perception stream이 한 번 시작된 뒤 멈추면 빈 장애물로 간주하지
        // 않는다. 빈 배열로의 전환은 전속 raceline 복귀이므로 아래에서
        // fail-safe stop trajectory를 발행한다.
        if (has_obstacles_ &&
            (this->now() - obstacles_stamp_).seconds() < obs_timeout_sec_) {
            obs_copy = obstacles_;
        } else if (has_obstacles_) {
            perception_stale = true;
        }
        for (const auto &obs : obs_copy.obstacles) {
            if (!std::isfinite(obs.s_start) || !std::isfinite(obs.s_end) ||
                !std::isfinite(obs.d_right) || !std::isfinite(obs.d_left) ||
                !std::isfinite(obs.vs) || !std::isfinite(obs.vd) ||
                obs.d_right > obs.d_left) {
                perception_stale = true;  // malformed도 동일하게 fail-closed
                obs_copy.obstacles.clear();
                break;
            }
        }
        // 정적/동적 재분류 (unicorn multi_tracking.py 방식): perception의
        // is_static 플래그를 관측 이력(위치 std 투표 + vs_reset)으로 덮어쓴다.
        // 파킹한 상대가 dynamic으로 남으면 예측/섹터게이팅/커밋이 전부
        // 동적 경로를 타서 회피가 무너진다 (파킹 상대 미회피 실측).
        ClassifyObstacles(obs_copy.obstacles);
    }

    f1tenth_planning_msgs::msg::WpntArray evasion_wpnts;
    evasion_wpnts.header.stamp = this->now();
    evasion_wpnts.header.frame_id = "map";
    visualization_msgs::msg::MarkerArray mrks;

    bool evasion_valid = false;
    bool used_trailing = false;
    // 이번 사이클의 트레일링이 **정지를 거부한** 트레일링인가 (allow_stop=false).
    // 컨트롤러 정지 발진 하한의 게이트가 이 값을 함께 본다 — 근거는
    // spliner_node.hpp 의 p_evade_committed_ 선언부 주석.
    bool trailing_no_stop = false;
    // 이번 사이클에 회피를 시도했는가. 장애물이 사라진 사이클에도 반드시
    // 갱신돼야 한다 — 안 그러면 in_ot_sector_가 true로 굳어 다음 장애물의
    // "회피 시도 진입" 상승엣지를 영원히 놓친다.
    bool try_evade_now = false;
    // 같은 이유로 아래 두 엣지 플래그도 사이클마다 갱신한다. 예전에는
    // if (!close_obs.empty()) 블록 안에서만 대입해서, 장애물이 사라지면
    // true로 굳어 두 번째 발생부터 pitwall 이벤트가 통째로 사라졌다.
    bool spline_fallback_now = false;
    // 이번 사이클에 동적 추종 캡이 걸렸는가 (pitwall 상승엣지용).
    bool follow_now = false;
    // 이번 사이클에 직선 인코스 추월 경로를 발행했는가.
    bool ot_now = false;
    bool static_stop_now = false;
    bool escape_arc_now = false;
    {
        std::lock_guard<std::mutex> lock(mutex_gb_wpnts_);
        // 붙어 있는 장애물들을 먼저 하나의 박스로 병합 — 이후의 최근접 선택,
        // 회피, 경로 검증이 모두 클러스터 단위로 일관되게 동작한다.
        // 이 안에서 "합치면 트랙이 막힌다" 판정이 나오면 봉쇄 근거가 선다.
        corridor_block_evidence_ = false;
        obs_copy.obstacles = ClusterObstacles(obs_copy.obstacles);
        // perception stale/malformed 사이클은 obstacles 가 비어 근거가 안 서고,
        // 해제 타이머가 정상적으로 래치를 푼다 — 별도 처리가 필요 없다.
        UpdateCorridorBlockLatch();
        auto all_ahead_now = ObstaclesInLookahead(obs_copy.obstacles);

        // 커밋 해제 판정용 타이머. 관측수/가시성 게이트를 거치지 않은 **원시**
        // 전방 목록으로 갱신한다 — stable_ahead 를 쓰면 실물이 잠깐 가려진
        // 것만으로 "아무것도 없다"가 되어 0811 에서 막은 coasting fail-open 이
        // 되살아난다. 인지가 stale 인 사이클은 근거가 없으므로 세지 않는다
        // (그 분기는 어차피 커밋을 즉시 지운다).
        if (perception_stale || !all_ahead_now.empty()) {
            last_obs_ahead_time_ = this->now();
        }

        if (perception_stale) {
            // 마지막 장애물 위치를 추측해 계속 회피하는 대신 즉시 최대 제동
            // 프로파일을 발행한다. sensor가 복구되어 fresh empty array가 오면
            // 다음 cycle에 정상 raceline으로 돌아간다.
            f1tenth_planning_msgs::msg::Obstacle stop;
            stop.id = std::numeric_limits<int32_t>::min();
            stop.is_static = true;
            // unwrapped 값으로 두어 (s_start-gap-cur)==0이 wrap 부동소수 오차로
            // 한 바퀴가 되는 것을 막는다. BuildTrailing 내부에서 경로 s만 wrap.
            stop.s_start = cur_s_ + trailing_gap_m_;
            stop.s_center = stop.s_start;
            stop.s_end = stop.s_start;
            stop.d_right = -vehicle_half_width_m_;
            stop.d_left = vehicle_half_width_m_;
            evasion_valid = BuildTrailing(stop, evasion_wpnts, mrks);
            used_trailing = evasion_valid;
            committed_until_s_ = -1.0;
            committed_obs_end_s_ = -1.0;
            committed_lat_budget_ = -1.0;
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "Perception obstacle stream stale/malformed: publishing fail-safe stop");
        }

        // 진행 중인 정적 회피는 경로 끝까지 커밋 — 매 사이클 재계획(ego-anchor
        // 재생성)과 '장애물 지남' 판정으로 회피가 도중에 사라지면, 복귀 중인
        // 차(d 큼)가 raceline 전속 경로로 순간 전환돼 반대편 벽을 친다 (실측).
        // 새 장애물이 커밋 경로를 막으면 즉시 폐기하고 재계획한다.
        if (!perception_stale && committed_until_s_ >= 0.0) {
            double span = Mod(committed_until_s_ - committed_start_s_, track_len_);
            double pos = Mod(cur_s_ - committed_start_s_, track_len_);
            // 커밋 경로를 무효화하는 장애물은 **안정된 트랙**이어야 한다.
            // 갓 뜬 1프레임짜리 유령 하나가 이미 성립한 회피를 파기하면
            // 그대로 정지 궤적으로 떨어진다 — 0807 run_164819에서 t=3.09에
            // 확보한 여유 0.35m 회피가 t=4.321 유령 등장 25ms 만에 파기됐고,
            // 유령 소멸(4.878) -> 회복(5.051) -> 재등장(5.600) -> 재거부로
            // 인과가 3번 확인됐다. 그 사이 vx=0 경로가 조향을 죽였다.
            // 안정성 판정은 관측 이력(ClassifyObstacles가 채우는 cls_tracks_)의
            // 관측 수로 한다 — perception의 min_hits와 같은 성격의 게이트다.
            const auto stable_ahead = StableObstacles(all_ahead_now);
            // ── 커밋 조기 해제 (2026-08-13) ───────────────────────────────
            // 커밋 유지 조건이 "커밋 경로 끝에 물리적으로 도달할 때까지"였다.
            // 그런데 CheckPathCollision 은 **커밋 경로 위**의 충돌만 본다 —
            // 커밋 경로가 끝나는 지점보다 더 앞에 있는 새 장애물은 충돌로
            // 잡히지 않으므로, 그 장애물은 커밋이 만료될 때까지 통째로
            // 무시된다. run_0813_165312 실측: S/F 장애물 #166/#178/#191 이
            // 전부 5.7~5.8m 에서 처음 보였는데 committed_hold 가 유지돼
            // 2.7~3.1m 에서야 처음 평가됐다(0.5초 · 3m 손실). 그 3m 가
            // 라인스위치 성립/불성립을 가르는 구간이다.
            //
            // 그래서 "커밋 경로가 덮지 못하는 안정 장애물이 전방
            // lookahead 안에 있으면" 커밋을 즉시 해제한다. 다만 커밋을 만든
            // 장애물 옆을 **아직 지나는 중**이면 해제하지 않는다 — 기동
            // 중간에 raceline 으로 전환되는 것이 커밋을 도입한 이유다.
            //
            // 판정 대상은 stable_ahead 가 아니라 all_ahead_now + 관측수 게이트다.
            // stable_ahead 는 commit_break_requires_visible 로 코스팅/기억
            // 장애물을 뺀 목록인데, 그 게이트의 목적은 "외삽 유령이 이미 성립한
            // 회피를 깨는 것"을 막는 것이다. 여기는 반대 상황 — 이미 지나간
            // 장애물의 커밋을 놓아 주고 **다음** 장애물을 계획하자는 것이므로
            // 그 게이트가 걸리면 안 된다 (랩 기억 장애물은 정의상 is_visible=0).
            auto stable_for_release =
                [&](const f1tenth_planning_msgs::msg::Obstacle &o) {
                    auto it = cls_tracks_.find(o.id);
                    const int seen = (it == cls_tracks_.end())
                        ? 0 : static_cast<int>(it->second.s_hist.size());
                    return seen >= commit_break_min_obs_;
                };
            bool new_obs_beyond_commit = false;
            if (committed_obs_end_s_ >= 0.0) {
                const double past_obs = Mod(cur_s_ - committed_obs_end_s_, track_len_);
                const bool committed_obs_passed =
                    past_obs > 0.0 && past_obs < track_len_ * 0.5;
                if (committed_obs_passed) {
                    for (const auto &o : all_ahead_now) {
                        if (!stable_for_release(o)) {
                            continue;
                        }
                        const double start_ahead =
                            Mod(o.s_start - cur_s_, track_len_);
                        const double beyond =
                            Mod(o.s_start - committed_until_s_, track_len_);
                        if (start_ahead <= lookahead_m_ &&
                            beyond > 0.0 && beyond < track_len_ * 0.5)
                        {
                            new_obs_beyond_commit = true;
                            break;
                        }
                    }
                }
            }
            // ── 장애물 소멸 해제 (2026-08-17) ─────────────────────────────
            // 해제 경로가 perception_stale / 경로충돌 / new_obs_beyond_commit /
            // path_end 넷뿐이었다. 장애물이 0개면 CheckPathCollision 은 루프를
            // 한 번도 돌지 않고 "충돌 없음"을 돌려주고 new_obs_beyond_commit 은
            // 장애물이 있어야 성립하므로, **유일한 출구가 경로 끝까지 주행하는
            // 것**이었다. 정지 상태에서는 원리적으로 영원히 안 풀린다.
            //   run_0815_191056: 장애물 0개인 커밋이 정지 7.2초 + 주행 7.3m =
            //   10.7초 유지 → 그동안 발진 속도계획이 동결됐고, 만료 순간
            //   회피상실 홀드가 0 m/s 캡을 걸어 0.81초 정지로 이어졌다.
            // 커밋이 막으려던 위험("복귀 중 raceline 전속 경로로 급전환")은
            // 정의상 장애물이 있을 때만 존재하므로, 전방이 완전히 빈 상태가
            // 충분히 이어졌으면 놓아 주는 것이 안전하다. 임계는 관측된 인지
            // 드롭아웃(0812 실측 중앙 275ms / 최대 745ms)보다 크게 잡는다.
            // ── 자차 횡이탈 해제 (2026-08-22) ─────────────────────────────
            // 해제 사유 넷(경로충돌 / 새 장애물 / 장애물 소멸 / path_end)이
            // 전부 **경로와 장애물**만 본다. 자차가 그 경로에서 얼마나
            // 벗어났는지는 아무도 보지 않아, 못 타는 경로를 끝까지 재생한다.
            //   run_0822_000933 t=30.06~31.56 실측: 경로오차 0.51 -> 1.01 m
            //   로 벌어지는 1.5 초 동안 committed_hold 가 유지됐고, 속도는
            //   백스톱이 1.9 -> 0.00 으로 깎아 차가 섰다. 커밋은 결국
            //   `nothing_ahead`(장애물 소멸)로만 풀렸다.
            // 문턱은 같은 백에서 갈랐다 (발행 회피경로 대비 자차 |Δd|):
            //   정상 회피 1299 사이클  p50 0.002 / p90 0.072 / p99 0.188 / max 0.532
            //   사건  3구간 261 사이클  p50 0.331 / p90 0.724 / max 1.006
            //   문턱 0.40 m -> 정상 구간 발화 **1 사이클**, 사건 구간 120 사이클.
            //   연속 3사이클(30 Hz = 0.1 s)을 요구하면 그 1 건도 살아남지 못한다.
            // 파기하면 다음 사이클에 티어가 **현재 위치에서** 다시 계획한다 —
            // BlendSpeedCap 이 "남은 횡이동을 남은 거리에 넣을 수 있는 속도"를
            // 풀어 필요한 만큼만 감속하고, 실패해도 트레일링이 retry_d 목표를
            // 유지하므로 raceline 전속으로 튀지 않는다.
            bool lat_deviated = false;
            if (commit_break_lat_dev_m_ > 0.0 && !committed_wpnts_.wpnts.empty()) {
                double best = std::numeric_limits<double>::infinity();
                double d_plan = 0.0;
                for (const auto &w : committed_wpnts_.wpnts) {
                    const double ds = std::abs(
                        Mod(w.s_m - cur_s_ + track_len_ / 2.0, track_len_) -
                        track_len_ / 2.0);
                    if (ds < best) {
                        best = ds;
                        d_plan = w.d_m;
                    }
                }
                // 경로가 자차 s 를 덮지 않으면 판정하지 않는다 (커밋 꼬리를
                // 이미 지난 경우 — 그건 path_end 가 처리한다).
                if (best <= 0.5 && std::isfinite(cur_d_)) {
                    const double dev = std::abs(cur_d_ - d_plan);
                    pitwall::Log("commit_lat_dev", dev);
                    if (dev > commit_break_lat_dev_m_) {
                        ++commit_lat_dev_run_;
                    } else {
                        commit_lat_dev_run_ = 0;
                    }
                } else {
                    commit_lat_dev_run_ = 0;
                }
                lat_deviated =
                    commit_lat_dev_run_ >= commit_break_lat_dev_cycles_;
            } else {
                commit_lat_dev_run_ = 0;
            }
            const double empty_for =
                (this->now() - last_obs_ahead_time_).seconds();
            const bool nothing_ahead =
                commit_release_empty_sec_ > 0.0 && all_ahead_now.empty() &&
                std::isfinite(empty_for) && empty_for >= commit_release_empty_sec_;
            // ── 꼬리 충돌은 커밋 파기 사유가 아니다 (2026-08-18) ──────────────
            // 재검증이 커밋 경로 **전체**를 봤다. 그런데 커밋이 지키는 것은
            // 기동(진입 블렌드 + 대상 장애물 통과)이고, 그 뒤에 붙는
            // raceline 복귀 꼬리는 보호 대상이 아니다 — 거기에 있는 장애물은
            // 어차피 다음 기동으로 따로 계획한다. 그런데 그 꼬리는 정의상
            // d≈0 (raceline) 이므로 **하류 콘에 반드시 걸린다**.
            //   run_0817_194156 실측: committed_break 180 회 중 path_collision
            //   115. 오프라인으로 111 건을 재현해 어느 장애물이 깼는지 보면
            //     · 40% 가 4 m 이상 앞, 25% 가 6 m 이상 앞 (중앙 3.5 m)
            //     · 5 m 이상 앞인 경우 충돌점의 경로상 위치 0.83(꼬리),
            //       그중 49% 가 |d| < 0.10 = 복귀 구간
            //     · 깬 장애물의 47% 는 관측 나이 0.4 초 미만 —
            //       commit_break_min_obs(7) 를 그 순간 막 넘긴 트랙이다
            //   파기 시점은 pos/span 중앙 0.14, 즉 기동의 14% 지점이다.
            // 그리고 그 대가가 트레일링이다: 트레일링 에피소드 71 개 중
            // 62 개(87%)가 직전 0.4 초 안의 committed_break 뒤에 시작했고,
            // 재계획 시점의 장애물 거리는 중앙 0.56 m 였다(성공한 lane_switch
            // 는 중앙 4.19 m). 그 거리에서는 lane_geom_floor 가 옳게 거부한다
            // — 회랑에 자리는 있었다(정적 트레일링 1339 중 1337 에서 한쪽
            // 레인이 성립, 여유 중앙 0.59 m, 필요 횡이동 중앙 0.22 m).
            //   즉 "자리가 없어서" 가 아니라 "0.5 m 앞에서 처음 계획해서" 다.
            //
            // 그래서 충돌점이 기동 구간 **밖**이고 재계획 여유가 남아 있으면
            // 파기하지 않고 경로를 그 앞에서 **자른다**. 커밋은 잘린 끝에서
            // path_end 로 정상 만료되므로, 다음 장애물은 0.5 m 가 아니라
            // plan_lead 만큼 앞에서 처음 계획된다. 자른 뒤에는 충돌하던 꼬리가
            // 경로에 없으므로 다음 사이클의 재검증이 그대로 통과한다(수렴).
            // 충돌점이 기동 안이거나 여유가 없으면 종전대로 파기한다 — 그건
            // "지금 하는 기동 자체가 더는 안전하지 않다" 이므로 사유가 다르다.
            double blocking_s = -1.0;
            const f1tenth_planning_msgs::msg::Obstacle *blocking_obs = nullptr;
            double contact_slack = std::numeric_limits<double>::infinity();
            const bool path_clear = CheckPathCollision(
                committed_wpnts_.wpnts, stable_ahead, &blocking_s,
                &blocking_obs, &contact_slack);
            // ── 파기 확정: 깊이와 지속을 본다 (2026-08-18) ────────────────────
            // 종전에는 **한 사이클에 한 번** 박스에 걸리기만 하면 즉시 파기였다.
            // 어느 트랙을 세느냐에는 관측 7회 게이트(commit_break_min_obs)가
            // 있는데, "얼마나 깊이" "몇 번" 걸렸는지에는 문턱이 하나도 없었다.
            // run_0817_194156 실측 (path_collision 파기 115 건):
            //   · 파기 시점부터 거슬러 세면 **70 건(61%)이 딱 한 프레임**짜리
            //     충돌이다. 다음 프레임이면 사라진다. 6 사이클 이상 지속된
            //     진짜배기는 32 건(28%)뿐이다.
            //   · 기동 구간 내 파기 50 건 중 46 건(92%)이 **커밋의 대상 장애물
            //     자신**이고, 47 건 중 46 건(98%)에서 커밋 생성 이후 그 박스가
            //     넓어졌다(가장자리 이동 중앙 10~12 cm).
            //   · 그 결과 파기 시점 관측 폭 중앙 54 cm — 실제 콘은 30 cm 다.
            //     정적 장애물 관측 폭은 중앙 30 cm 로 정확하지만 0~5 m 에서
            //     17~20% 의 프레임이 45 cm 를 넘고, 인접 프레임 폭이 10 cm 넘게
            //     튀는 경우가 9% 다. 박스가 5 프레임에 1 번꼴로 부푼다.
            //   · 침투 깊이 중앙 5.3 cm 이고, **44 건(88%)은 차체가 닿지도
            //     않는다** (박스 27 cm 중 12 cm 는 분산 여유라 차체는 그만큼
            //     늦게 닿는다).
            // 그래서 둘을 나눈다:
            //   · 차체 접촉선을 넘었으면(contact_slack <= 0) **즉시** 파기한다.
            //     여기에는 지연을 두지 않는다 — 물리적으로 닿는 상황이다.
            //   · 그 아래(= 분산 여유만 갉아먹은 상태)면 연속 N 사이클일 때만
            //     파기한다. N=2 면 확정이 33 ms(5 m/s 에서 0.17 m) 늦어지는
            //     대신 115 건 중 74 건(64%)이 살아남는다. N=3 으로 늘려도 4 건
            //     밖에 더 못 얻으므로 2 가 그 계단이다.
            // ⚠️ 이 완화는 **이미 성립한 커밋의 유지 판정**에만 적용된다.
            //    회피 경로를 새로 만드는 쪽(BuildLaneSwitch/DoSpline 의
            //    CheckPathCollision)은 종전 그대로 한 번에 거부한다.
            // 잔여 위험: 유예 56 건 중 다음 사이클에도 충돌이 남은 30 건에서
            // 2 건이 그 33 ms 동안 접촉 여유 -1.1 cm 까지 내려간다. 즉시파기
            // 문턱을 양수로 올려도 안 잡힌다 (0/1/2/3/5 cm -> 2/2/2/1/1 건).
            // 근거는 params.yaml 주석 참조 — 요약하면 그 -1.1 cm 는 5 프레임에
            // 1 번꼴로 25 cm 부푸는 **관측** 박스 기준이고, 한 사이클 일찍
            // 파기해도 폴백이 그 안에 차의 횡위치를 못 바꾼다.
            const bool contact = !path_clear && !(contact_slack > 0.0);
            if (path_clear) {
                commit_collision_run_ = 0;
            } else {
                ++commit_collision_run_;
            }
            const bool collision_confirmed =
                !path_clear &&
                (contact || commit_collision_run_ >= commit_break_confirm_cycles_);
            if (!path_clear && !collision_confirmed) {
                NoteReject("commit_collision_deferred");
            }
            bool tail_blocked = false;
            double tail_release_s = -1.0;
            if (!path_clear && commit_tail_release_lead_m_ > 0.0 &&
                committed_obs_end_s_ >= 0.0 && blocking_s >= 0.0 &&
                blocking_obs != nullptr)
            {
                // 대상 장애물을 이미 지났으면 커밋의 보호 목적은 끝났다 —
                // 남은 충돌은 전부 "다음 것"이므로 정상적으로 파기한다.
                const double past_obs_end =
                    Mod(cur_s_ - committed_obs_end_s_, track_len_);
                const bool obs_end_passed =
                    past_obs_end > 0.0 && past_obs_end < track_len_ * 0.5;
                const double blocking_ahead =
                    Mod(blocking_s - cur_s_, track_len_);
                // 기동 구간의 끝. hold 창과 같은 규약(장애물 뒤 0.5 m).
                const double maneuver_end =
                    Mod(committed_obs_end_s_ - cur_s_, track_len_) + 0.5;
                // 다음 사이클 티어가 실제로 기동을 만들 수 있는 거리.
                // ── 진폭은 **차단 장애물에서 직접** 뽑는다 (2026-08-18) ────────
                // 처음엔 evasion_dist_(0.35) 를 상수 진폭으로 썼는데,
                // BlendDist 는 v 에 정비례하므로 고속에서 모자란다.
                //   tools/bag_analysis_0817/commit_tail.py (lead 3.0 고정,
                //   진폭 evasion_dist): 유예 38 건 중 **8 건이 만료 시점에
                //   회피도 제동도 불가**했다 — 전부 v 4.85~5.87 구간이고
                //   실제 필요 진입거리가 3.02~3.74 m 인데 lead 는 3.00~3.14
                //   였다. 상수 진폭으로는 이 절단을 실차에 올릴 수 없다.
                // 재계획이 실제로 옮겨야 할 양은 그 장애물의 가까운 쪽
                // 가장자리 + 요구 간격이고, 기준점은 raceline(d=0) 이다 —
                // 꼬리에서 이미 복귀 중이므로 그때 자차 d 는 0 근처다.
                const double blk_clear =
                    RequiredObstacleClearance(*blocking_obs);
                const double shift = std::max(
                    std::min(std::abs(blocking_obs->d_left + blk_clear),
                             std::abs(blocking_obs->d_right - blk_clear)),
                    evasion_dist_);
                // 속도는 "지금"이 아니라 "만료 지점까지 가는 동안 낼 속도"다.
                // 지금 속도로 재면 그 사이의 가속만큼 리드가 모자란다 —
                // commit_tail.py 에서 남은 fail-open 2 건이 정확히 그 차이
                // (파기 5.66 -> 만료 5.80 m/s, 부족 0.09 m)였다.
                // BuildLaneSwitch 의 carried_in 이 같은 이유로 쓰는 값이다.
                const double lead_v =
                    BlendPlanSpeed(Mod(cur_s_, track_len_), blocking_ahead);
                const double plan_lead = std::max(
                    commit_tail_release_lead_m_,
                    BlendDist(shift, lead_v) + vehicle_half_length_m_);
                if (!obs_end_passed && blocking_ahead < track_len_ * 0.5 &&
                    blocking_ahead - plan_lead > maneuver_end)
                {
                    tail_blocked = true;
                    tail_release_s = Mod(
                        cur_s_ + (blocking_ahead - plan_lead), track_len_);
                }
            }
            if (pos < span && !new_obs_beyond_commit && !nothing_ahead &&
                !lat_deviated &&
                (!collision_confirmed || tail_blocked)) {
                // 절단은 **확정된** 충돌에만 한다. 한 프레임짜리 유령으로
                // 커밋을 짧게 만들면 만료가 근거 없이 앞당겨진다.
                if (tail_blocked && collision_confirmed) {
                    TruncateCommitAt(tail_release_s, blocking_s);
                    span = Mod(committed_until_s_ - committed_start_s_,
                               track_len_);
                }
                evasion_wpnts.wpnts = committed_wpnts_.wpnts;
                // ── 커밋 동안 그립 예산을 얼린다 (2026-08-20) ─────────────
                // 이 블록이 서면 아래 close_obs 가 비므로 장애물 분기의
                // 예산 선택이 실행되지 않아 active_lat_budget_ 는 리셋값
                // (= 정적 예산)으로 남는다. 그 상태로 아래 재생을 돌리면
                // **기하는 동적 예산으로 만든 경로인데 전방 가속 envelope 만
                // 정적 예산으로 다시 푸는** 불일치가 생긴다. 분류가 뒤집혀도
                // 같은 문제가 생기므로 생성 시점 값을 그대로 되돌린다.
                if (committed_lat_budget_ > 0.0) {
                    active_lat_budget_ = committed_lat_budget_;
                }
                // ── 기하는 얼리되 속도는 갱신한다 (2026-08-15) ─────────────
                // 커밋은 "매 사이클 재계획하면 복귀 중인 차가 raceline 전속
                // 경로로 순간 전환돼 반대편 벽을 친다"를 막으려고 도입됐다.
                // 그 이유는 **기하**에만 해당하는데 속도까지 같이 얼어 있었다:
                //   run_0814_191443 committed_hold 6,906 사이클 실측 —
                //   동결된 머리 속도와 *커밋 생성 시점* 차속의 상관 0.99,
                //   *지금* 차속과는 0.56. 유지 1.5초 이상이면 머리 2.76 인데
                //   차는 3.94 로 달린다. 최장 동결 16.8초, 141개 에피소드 중
                //   110개가 속도 배열 비트 단위로 동일했다.
                //   그 동결이 발목을 잡은 사이클이 44.5%, 손실 41.1초
                //   (랩당 1.21초) — trailing 전체(33.9초)보다 크다.
                // 보관해 둔 기하 상한에서 시작해 현재 cur_vs_ 로 전방 가속
                // envelope 만 다시 푼다. x·y·d 를 건드리지 않으므로 기하는
                // 완전히 동일하다. 실패하면 동결값을 그대로 쓴다.
                if (!committed_geom_ceiling_.empty() &&
                    committed_geom_ceiling_.size() ==
                        committed_wpnts_.wpnts.size())
                {
                    auto refreshed = committed_wpnts_.wpnts;
                    if (ApplySpeedProfile(refreshed, false, nullptr,
                                          &committed_geom_ceiling_)) {
                        evasion_wpnts.wpnts = std::move(refreshed);
                    } else {
                        NoteReject("commit_speed_refresh_failed");
                    }
                }
                mrks = committed_mrks_;
                evasion_valid = true;
                // 커밋 유지 중에는 섹터 판정을 다시 하지 않으므로 in_sector는
                // 의미가 없다 (false로 고정, outcome이 상태를 말해준다).
                TraceCycle(all_ahead_now.empty() ? nullptr : &all_ahead_now.front(),
                           true, true, false, "committed_hold");
            } else {
                // 커밋 파기는 드물지만 그대로 정지/급조향으로 이어지므로 반드시
                // 남긴다 (가설 (c) "성립했다가 깨진다"의 유일한 직접 증거).
                RCLCPP_WARN(this->get_logger(),
                    "[trace] committed_break pos=%.2f/%.2f stable_obs=%zu reason=%s",
                    pos, span, stable_ahead.size(),
                    lat_deviated         ? "lat_deviation"
                        : new_obs_beyond_commit ? "new_obs_beyond_commit"
                        : nothing_ahead   ? "nothing_ahead"
                        : (pos < span ? "path_collision" : "path_end"));
                if (lat_deviated) {
                    NoteReject("commit_released_lat_deviation");
                    RCLCPP_WARN(this->get_logger(),
                        "커밋 파기: 자차가 커밋 경로에서 %.2f m 이상 벗어남 "
                        "(%d 사이클 연속) — 현재 위치에서 재계획한다",
                        commit_break_lat_dev_m_, commit_lat_dev_run_);
                    pitwall::Event("커밋 파기 — 경로 이탈, 재계획");
                }
                if (new_obs_beyond_commit) {
                    NoteReject("commit_released_new_obs");
                }
                if (nothing_ahead) {
                    NoteReject("commit_released_nothing_ahead");
                }
                // 꼬리 절단으로 만료된 경우(= 잘린 끝까지 주행)를 따로 센다.
                // 이 경로로 만료되는 것이 설계 의도이고, 그 뒤 티어가 다시
                // 도는 시점의 장애물 거리가 이 수정의 성패다.
                if (!new_obs_beyond_commit && !nothing_ahead &&
                    pos >= span && committed_tail_truncated_)
                {
                    NoteReject("commit_released_tail_lead");
                }
                committed_tail_truncated_ = false;
                commit_collision_run_ = 0;
                commit_lat_dev_run_ = 0;
                committed_until_s_ = -1.0;
                committed_obs_end_s_ = -1.0;
                committed_lat_budget_ = -1.0;
            }
        }

        // 디버그: lookahead 내 장애물 구성 (동적 상대 상호작용 분석용)
        for (const auto &o : all_ahead_now) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "[obs] id=%d s=[%.1f,%.1f] d=[%.2f,%.2f] vs=%.1f static=%d",
                o.id, o.s_start, o.s_end, o.d_right, o.d_left, o.vs,
                o.is_static ? 1 : 0);
        }

        // ── 회피 대상과 추종 대상을 분리한다 (2026-08-22, 5/8단계) ────────
        // 1단계는 "동적을 회피 티어에 넣지 않는다"까지만 했고, closest 를
        // 여전히 정적/동적 섞인 목록에서 골랐다. 그래서 상대가 콘보다 가까우면
        // **그 콘이 이번 사이클에 평가조차 되지 않았다** (1단계 주석의 '남은
        // 구멍'). 여기서 목록 자체를 갈라 그 구멍을 없앤다:
        //   · 정적 -> 회피 티어(횡)
        //   · 동적 -> 종방향 추종(아래 별도 단계). 회피 경로가 있으면 그
        //             경로의 속도만 캡하므로 "회피가 최우선"이 성립한다.
        // 추종 목록은 evasion_valid(커밋 재생 중) 여부와 무관하게 만든다 —
        // 커밋을 재생하는 동안에도 상대는 앞에 있고 캡은 걸려야 한다.
        const auto filtered = perception_stale
            ? std::vector<f1tenth_planning_msgs::msg::Obstacle>{}
            : FilterObstacles(obs_copy.obstacles);
        std::vector<f1tenth_planning_msgs::msg::Obstacle> static_close;
        std::vector<f1tenth_planning_msgs::msg::Obstacle> dyn_close;
        for (const auto &o : filtered) {
            (o.is_static ? static_close : dyn_close).push_back(o);
        }
        // wrap-around을 고려한 전방 최근접 선택
        const auto nearest_ahead =
            [this](const std::vector<f1tenth_planning_msgs::msg::Obstacle> &v) {
                return *std::min_element(
                    v.begin(), v.end(),
                    [this](const auto &a, const auto &b) {
                        return Mod(a.s_end - cur_s_, track_len_) <
                               Mod(b.s_end - cur_s_, track_len_);
                    });
            };
        if (!evasion_valid && !static_close.empty()) {
            auto closest = nearest_ahead(static_close);
            // SP 레인체인지 검증용: lookahead 내 전체 장애물.
            // obs_copy.obstacles 는 ClusterObstacles 이후 수정되지 않으므로
            // 위에서 만든 all_ahead_now 를 그대로 쓴다 (장애물마다
            // 장애물별 전처리를 다시 도는 중복 계산 제거).
            const auto &all_ahead = all_ahead_now;
            // 섹터 게이팅: 장애물(예측 장애물이면 따라잡는 지점)이 추월 섹터
            // 안일 때만 회피 시도. 섹터 안은 공격 모드(경계 마진 완화 + 속도
            // 부스트), 밖은 TRAILING으로 따라가다 섹터에서 승부한다.
            // 동적 상대를 이미 추월 중(|d| 큼)이면 섹터 이탈로 공격을 끊지
            // 않는다 — 끊기면 trailing d램프가 차를 상대 위치(raceline)로
            // 도로 끌어당겨 접촉한다 (섹터 조각 경계에서 진동 실측).
            bool mid_pass = std::abs(cur_d_) > side_switch_d_tresh_ &&
                            !closest.is_static;
            bool in_sector = !sector_gating_ || mid_pass ||
                             InOtSector(Mod(closest.s_center, track_len_));
            // 정적 장애물은 섹터(직선) 밖이어도 회피해야 한다 — trailing으로
            // 빠지면 v_target=0이라 그 뒤에 영원히 정지 (헤어핀 정적 장애물
            // 실측). 속도 부스트 없이 회피만 허용한다.
            // (예전엔 여기서 경계 마진을 ot_bound_mindist 로 완화했으나, 그
            //  값이 spline_bound_mindist 와 같고 모든 소비처가
            //  RequiredBoundaryMargin() = max(요청, 차반폭 + 추적여유) 를
            //  거쳐 실차에서 항상 0.25 로 수렴해 완화가 성립한 적이 없다.)
            //    준정적 추월은 "정적 회피 기구를 동적에 연다"는 장치인데,
            //    정작 그 기구로 들어가는 문(try_evade)에 조건이 없어서
            //    메인 직선(= 유일한 추월 섹터, 랩의 29%) 밖에서는 판정만
            //    하고 아무 일도 일어나지 않는 사문화 상태였다.
            //    run_0819 84.462s 실측: 분류가 STATIC->DYNAMIC 으로 뒤집힌
            //    직후부터 조우가 끝날 때까지 evade=0 — TryLaneSwitch 자체가
            //    호출되지 않았고, 그 뒤에 걸린 NoteRetryTarget(횡이동 유지)
            //    과 커밋 승격도 같이 죽었다.
            // ── 동적 상대는 회피 대상이 아니다 (2026-08-22, 1/8단계) ────────
            // H2H 실주행 뒤 동적 처리를 처음부터 다시 만든다. 새 계약:
            //   · 동적 상대는 **회피 티어에 들어가지 않는다** — 위치 트래킹만
            //     하고, 간격이 좁혀지면 종방향 추종(트레일링)만 한다.
            //   · 회피(횡)는 정적 장애물 전용이고 항상 그쪽이 우선이다.
            //   · 유일한 추월은 직선 인코스 레이트브레이크 (7단계에서 신설).
            // 여기서 문을 닫으면 Tier1(라인스위치)/Tier2(스플라인)/
            // Tier2c(원호)가 전부 정적 전용이 되고, Tier1b(SP 레인체인지)는
            // `!closest.is_static && try_evade` 라 **발화 조건이 모순**이 되어
            // 죽는다 — 2단계가 그 사문화된 코드를 지운다.
            // 동적은 그대로 Tier3 트레일링으로 내려간다(기존 경로 유지).
            //
            // ⚠️ 남은 구멍: closest 가 동적이면 그 뒤의 정적 장애물이 이번
            //    사이클에 평가되지 않는다. 이건 closest 하나로 티어를 도는
            //    **기존 구조**의 성질이지 이번 변경이 만든 것이 아니다.
            //    5~6단계에서 "횡=정적 / 종=동적"으로 분리하면서 없앤다.
            bool try_evade = closest.is_static;
            try_evade_now = try_evade;
            // ── 추월 속도 배율은 **동적 전용** 이다 (2026-08-19) ──────────
            // 이 배율은 레이스라인 속도에 곱해져 회피 경로의 기준 vx 가 된다.
            // 정적 장애물 회피 기하는 오래 튜닝된 것이라, 같은 배율을 씌우면
            // 그 튜닝이 통째로 흔들린다. 추월(= 상대보다 빨리 가서 지나감)이
            // 필요한 것은 동적/준정적뿐이므로 거기서만 연다.
            //
            // 물리 캡은 그대로 걸린다 — ApplySpeedProfile 이 곡률별
            // sqrt(a_lat/k) 와 조향 한계로 다시 누르므로, 직선에서만 실제로
            // 살아나고 코너에서는 자동으로 눌린다. run_0818 실측 a_lat p90 이
            // 포락선의 86~105% 라 코너에는 여유가 없고, 이번 조우 구간
            // (직선, kappa 0.01)에는 여유가 충분하다.
            // 2026-08-22 (4/8단계): 동적 전용 그립 예산 분리를 없앴다.
            // 회피가 정적 전용이 됐으므로 나눌 대상이 없다. 예산 상향은
            // 9단계에서 path_lat_acc_max_ 하나를 실측 포락선에 맞춰 올린다.
            // 회피 시도 진입 알림 (try_evade 상승엣지)
            if (try_evade && !evade_attempt_active_) {
                pitwall::Event(in_sector ? "추월 섹터 — 라인 스위치 시도"
                                         : "정적 장애물 — 라인 스위치 시도");
            }

            // ── Tier 1: 라인 스위치 (기본 maneuver) ─────────────────────────
            // 오프셋 레인(±apex)으로 ease-on→hold→ease-back(rule-based). 정적
            // 장애물은 항상, 동적 상대는 추월 섹터 안에서만(try_evade) 시도한다
            if (use_lane_switch_ && try_evade) {
                evasion_valid = TryLaneSwitch(closest, all_ahead,
                                              evasion_wpnts, mrks);
            }

            // ── Tier 2: free-form 스플라인 폴백 (모든 레인 막힘) ────────────
            // 고정 오프셋 레인이 막혔을 때만 DoSpline을 "호출"한다 —
            // 자연 3차 스플라인이 고정 레인이 못 덮는 틈으로 d(s)를 최적화해
            // 마지막으로 회피 해를 찾는다.
            // ── 2026-08-18: Tier 2 를 정적 전용으로 좁힌다 ────────────────
            // DoSpline 은 d(s) 를 자유롭게 최적화하므로 입력이 조금만 흔들려도
            // 사이클마다 다른 모양이 나온다. 고정 레인(Tier 1)은 후보가 좌/우
            // 둘뿐이라 같은 답으로 수렴하지만 여기는 그렇지 않다.
            // 동적 상대는 그 입력이 vs 인데 노이즈가 신호와 같은 크기다
            // (정지가 증명된 트랙에서 std 0.43~0.45 m/s, 순간 최대 1.5 —
            //  run_0815). 게다가 동적은 커밋이 없어 20 Hz 로 매 사이클
            // 처음부터 다시 푼다 -> "불필요한 라인"이 끝없이 생성된다.
            // 의존성 확인 — free_spline_geom_infeasible_ 의 소비처는
            //   · Tier 2c EscapeArc 게이트: 어차피 is_static 요구라 동적 무관
            //   · geom_blocked: pitwall 문구에만 쓰임
            // 이라 동적에서 이 티어를 빼도 다른 분기가 바뀌지 않는다.
            bool used_spline = false;
            if (!evasion_valid && try_evade && closest.is_static) {
                used_spline = DoSpline(closest, all_ahead, evasion_wpnts, mrks);
                evasion_valid = used_spline;
            }
            if (used_spline && !spline_fallback_active_) {
                pitwall::Event("모든 레인 막힘 — 스플라인 폴백");
            }
            spline_fallback_now = used_spline;

            // ── Tier 2c: 원호 이탈 (정지 직전 마지막 회피) ──────────────────
            // Tier1/2 는 둘 다 평행 복귀를 전제한 smoothstep 이라 6A/L² 을
            // 요구한다. 여기서만 평행 복귀를 포기하고 한쪽 원호(2A/L²)로
            // 비킨다 — 필요 거리가 58% 로 줄어든다.
            // **기하 하한으로 거부됐을 때만** 시도한다. 충돌/벽 같은 다른
            // 사유는 원호로도 안 풀리고, 그때 급조향하면 상황만 나빠진다.
            bool used_escape = false;
            if (!evasion_valid && try_evade &&
                (lane_switch_geom_infeasible_ || free_spline_geom_infeasible_)) {
                used_escape = BuildEscapeArc(closest, all_ahead,
                                             evasion_wpnts, mrks);
                evasion_valid = used_escape;
                if (used_escape) {
                    // 원호가 성립했다는 것은 "기하 불성립"이 더는 아니라는
                    // 뜻이다. 플래그를 내려야 Tier3 가 이걸 정지 사유로
                    // 재사용하지 않는다.
                    lane_switch_geom_infeasible_ = false;
                    free_spline_geom_infeasible_ = false;
                    if (!escape_arc_active_) {
                        pitwall::Event("원호 이탈 (정상 회피 기하 불성립)");
                    }
                }
            }
            escape_arc_now = used_escape;

            // ── Tier 3: 트레일링 / 정지 ────────────────────────────────────
            // 동적 상대의 트레일링은 "뒤에 붙어 따라가다 추월 섹터에서
            // 승부한다"는 레이싱 로직이고 출구가 있다 — 섹터에 도달하거나
            // 상대가 움직이면 상태가 바뀐다.
            //
            // 정적 장애물에는 그 출구가 없다. 장애물이 안 움직이고 자차도
            // v=0이면 다음 사이클의 입력이 이번과 완전히 같으므로 판정도
            // 같고, 그대로 영구 정지가 된다 (run_0806_234734 35초 /
            // 235014 20초 / 234247 7.7초). 게다가 rosbag_0806 4런에서
            // 트레일링이 걸린 1345프레임 중 트랙이 실제로 막혀 있던 건
            // 4프레임(0.3%)뿐이었다 — 나머지는 전부 회피가 가능했다.
            //
            // 그래서 정적은 트레일링을 태우지 않고, Tier1/2가 "어떤 속도로도
            // 불가"를 돌려준 경우(= 조향 기하 하한 초과 / 트랙 실제 막힘)에만
            // 정지 궤적을 만든다. 정지 궤적 자체는 BuildTrailing과 같은
            // 기하(raceline 복귀 램프 + 제동 프로파일)를 쓴다.
            //
            // 2026-08-07: 위 주석이 선언한 대로 코드가 사유를 구분하게 고쳤다.
            // 예전에는 `if (!evasion_valid)` 뿐이라 충돌검사
            // 거부·벽 축소·"이미 장애물 구간 내부"·스팬 초과 같은 일시적
            // 사유에도 v_target=0 정지 궤적을 발행했다. 0807 실측으로 그
            // 정지 궤적(전 점 vx=0)이 MAP 컨트롤러의 조향을 0으로 만들어
            // (control_node.cpp:1057) 코스아웃의 직접 원인이 됐다.
            if (!evasion_valid) {
                // 정지가 정당한 경우:
                //  - 동적: 트레일링은 원래 "따라가기"이므로 그대로 (v_target=상대속도).
                //    여기를 빼면 정지한 상대 뒤에서 trailing_retry_speed(1.5)를
                //    바닥으로 깔고 들이받는다 — 반드시 남긴다.
                //  - 정적: **더 이상 정지 사유가 아니다.**
                //
                // 2026-08-13: geom_blocked 를 정지 사유에서 뺀다. 옛 근거는
                // "조향 기하 하한 초과 = 어떤 속도로도 불가 = 서라" 였는데,
                // 그건 트레일링이 d 를 raceline 으로 되돌릴 때만 성립했다.
                // 이제 트레일링이 회피 목표를 유지하며 매 사이클 d 를 목표
                // 쪽으로 옮기므로(BuildTrailing 의 retry_d_target_ / d_profile),
                // 남은 진폭이 줄고 필요 진입거리도 같이 준다 — 기하 불가는
                // **영구 조건이 아니라 일시 조건**이 됐다. 정지시키면 그
                // 수렴을 스스로 끊고, 실측상 과도한 감속으로 나타났다.
                // 정적 장애물에서는 이제 trailing_retry_speed 감속만 걸고
                // 횡이동이 끝나는 대로 raceline 으로 복귀한다.
                //
                // ⚠️ 트레이드: 정적에 대해 "완전 정지" 경로가 사라진다.
                // 횡이동이 끝나기 전에 장애물에 도달하면 1.5 m/s 로 접촉한다.
                // 남는 방어선은 컨트롤러 백스톱과 recovery_supervisor 뿐이다.
                const bool geom_blocked = lane_switch_geom_infeasible_ ||
                                          free_spline_geom_infeasible_;
                // ── 관측수 게이트를 여기 걸지 않은 이유 (2026-08-17) ──────────
                // "is_static 이 근거를 갖기 전에 allow_stop 이 열린다"는 지적이
                // 있었다. 구조적으로는 맞다 — ClassifyObstacles 는 관측이
                // cls_min_nb_meas(5) 를 넘기 전에는 재분류를 건너뛰고 perception
                // 플래그를 그대로 통과시키고(`s_hist.size() <= cls_min_nb_meas_`
                // 에서 continue), 브릿지의 대응 가드(unsettled_class -> 정적
                // 취급)는 **confidence** 로 키가 걸려 있는데 유령은 confidence
                // 로 갈리지 않는다(브릿지 주석: 코스팅 유령 510건/2087 이
                // conf>=0.2). 그래서 `ObsSeen(closest.id) > cls_min_nb_meas_` 를
                // 붙여 보고 tools/bag_analysis_0817/stop_gate.py 로 쟀다.
                //
                // run_0817_123650 결과: **막히는 사이클이 0 건이었다.**
                //   · 트레일링 대상이 동적으로 내려온 사이클 51 (고유 트랙 1개)
                //   · 그 트랙(id=244)의 총 관측 106 프레임 — 게이트 문턱과 무관
                //   · 138초 전체에서 is_static=false 프레임을 가진 트랙이 1개뿐
                // 즉 이 백에서 allow_stop=true 는 사실상 일어나지 않는다.
                // 그리고 커밋 afb72d4 가 이 경로의 시작으로 지목한 유령
                // id=8(t=11.33, s=17.45)은 **is_static=True** 였다 — allow_stop
                // 은 이미 false 였고, v_target 도 정적 경로의 바닥
                // (trailing_retry_speed 2.5 -> 횡목표 축소 시 degraded 1.5)에
                // 걸려 0.04 가 나올 수 없다. 홀드가 기억한 0.042 의 출처는
                // 트레일링이 아니라 다른 경로다 (커밋된 회피 경로의 머리가
                // 1순위 후보 — 홀드 주석의 run_0815 사례와 같은 모양).
                //
                // 0 건을 막는 게이트를 넣고 그 대가로 진짜 동적 상대의 정지
                // 권한을 6프레임(152ms) 늦추는 것은 남는 거래가 아니다. 위
                // 1310 주석이 "반드시 남긴다"고 못박은 방향이기도 하다.
                // 다시 검토할 조건: **실제로 움직이는 상대차가 있는 백**에서
                // stop_gate.py 가 0 이 아닌 수를 내면 그때 붙인다.
                // ── 멈춘 상대 뒤에서 설 권한 (2026-08-22 복구) ──────────────
                // `allow_stop = !is_static` 하나면, 달리던 상대가 멈춰
                // ClassifyObstacles 의 vs_reset 이 STATIC 으로 뒤집는 순간
                // **정지 권한이 사라진다** — trailing_retry_speed(2.5) /
                // degraded(1.5) 바닥이 깔려 멈춘 차에 그 속도로 들어간다.
                //
                // 판단 근거를 분류기(WasEverDynamic)가 아니라 **관측된 변위**
                // (WasEverMoved)로 두는 이유는 max_disp 주석 참조 — 짧게 흔들린
                // 콘이 DYNAMIC 으로 각인되는 일이 실제로 있었다.
                //
                // ⚠️ 이 한 줄만 바꾸면 "잠깐 느려진 상대"를 정지로 오인해 급감속
                //    한다. ClassifyObstacles 가 정적 트랙의 obs.vs 를 0 으로
                //    **강제**하기 때문이다. 그래서 BuildTrailing 의 obs_vs_eff
                //    가 ever_moved 트랙에 대해 관측 창 평균(ObsVsMean)을 쓰도록
                //    같이 바꿨다 — 둘은 한 묶음이다.
                const bool allow_stop =
                    !closest.is_static || WasEverMoved(closest.id);
                // ── 회랑 봉쇄 래치의 승격 (2026-08-22) ──────────────────────
                // allow_stop 문장 자체는 손대지 않는다 — 그 계약("정지 권한은
                // 관측 변위로 연다")은 그대로 두고, 회랑이 실제로 막혔다는
                // **독립 근거**를 OR 로 얹는다. 잔해는 새 id·is_static·이동
                // 이력 0 이라 위 두 항으로는 영원히 열리지 않는다
                // (run_0822_000310: 그래서 degraded 1.5 로 들이받았다).
                const bool stop_authority =
                    allow_stop || corridor_block_latched_;
                evasion_wpnts.wpnts.clear();
                mrks.markers.clear();
                evasion_valid = BuildTrailing(closest, evasion_wpnts, mrks,
                                              stop_authority, &all_ahead);
                used_trailing = evasion_valid;
                trailing_no_stop = used_trailing && !stop_authority;
                if (used_trailing && closest.is_static &&
                    !static_stop_active_) {
                    // 정적은 기본적으로 정지가 아니라 감속이다
                    // (allow_stop = !is_static). 회랑 봉쇄 래치가 걸린
                    // 사이클만 예외로 정지까지 간다 — 문구를 거동에 맞춘다.
                    pitwall::Event(
                        corridor_block_latched_
                            ? "회랑 봉쇄 — 정적 장애물 앞 정지"
                        : geom_blocked
                            ? "정적 장애물 감속 (회피 기하 불성립 — 횡이동 유지)"
                            : "정적 장애물 감속 (회피 일시 실패 — 재시도)");
                }
                static_stop_now = used_trailing && closest.is_static;
            }

            TraceCycle(&closest, false, try_evade, in_sector,
                       !evasion_valid   ? "none"
                       : used_trailing  ? "trailing"
                       : used_escape    ? "escape_arc"
                       : used_spline    ? "free_spline"
                                        : "lane_switch");

            // 정적 장애물 회피 성공 -> 경로 끝까지 커밋 (동적/예측 상대는
            // 움직이므로 커밋하지 않고 기존처럼 매 사이클 갱신)
            //
            // ── 커밋 승격 게이트 (2026-08-17) ─────────────────────────────
            // 커밋을 **만드는** 쪽에는 게이트가 하나도 없었다. FilterObstacles
            // 는 유한성·lookahead·회랑만 보고, ClassifyObstacles 는 관측이
            // cls_min_nb_meas 에 못 미치면 perception 의 is_static 을 그대로
            // 통과시킨다 — 즉 1프레임짜리 유령 하나로 7m 커밋이 성립했다.
            // 반대로 깨는 쪽(StableObstacles)은 commit_break_min_obs 회 관측
            // + is_visible 을 요구한다. 이 비대칭이 run_0815_191056 의 원인이다:
            //   플래너가 받은 트랙 94개 중 1프레임 21개, 7프레임 미만 67개(71%).
            //   이들은 커밋을 만들 수는 있어도 깰 수는 없다.
            // 회피 **경로 생성**은 이 게이트 앞에 그대로 두므로 회피 반응
            // 지연은 0 이다. 얼리는 것만 미룬다 — 커밋의 목적("복귀 중
            // raceline 전속 경로로 급전환 방지")은 기동이 이미 진행 중일
            // 때만 의미가 있다. 실측 비용: 7번째 관측까지 중앙 151ms / 0.43m
            // (lookahead 10m 의 4.3%). 진짜 장애물은 39.4Hz 로 연속 관측되고
            // 프레임 수 분포가 1~6 / 10 이상으로 이봉형이라 7 이 그 골짜기다.
            //
            // 정지 중 커밋도 막는다. 정지 상태에서 만들어진 커밋은 머리 속도가
            // vx=0 으로 굳어 local_trajectory 의 회피상실 홀드가 전 구간
            // 0 m/s 캡을 걸고(run_0815 실측 0.81초 정지), pos < span 이 계속
            // 참이라 path_end 로도 만료되지 않는다.
            // 2026-08-18: 준정적 추월도 커밋 대상에 넣는다. 동적에 커밋이
            // 없어서 회피 경로가 20 Hz 로 매 사이클 재생성되던 것이 churn 의
            // 근본 원인이었다. 재검증 기구는 새로 만들지 않는다 — 위 커밋
            // 유지 블록이 매 사이클 StableObstacles + CheckPathCollision 으로
            // 다시 검사하고, 그 충돌검사는 동적 장애물을 도착시각만큼
            // 전파(motion_s)하므로 상대가 움직여 커밋 경로를 침범하면 그대로
            // 파기된다. 즉 "얼리되 매 사이클 반증 가능"이 이미 성립한다.
            const bool commit_candidate =
                evasion_valid && !used_trailing &&
                closest.is_static &&
                !evasion_wpnts.wpnts.empty();
            const bool commit_obs_ok = ObsSeen(closest.id) >= commit_min_obs_;
            const bool commit_speed_ok = std::abs(cur_vs_) >= commit_min_speed_;
            if (commit_candidate && !commit_obs_ok) {
                NoteReject("commit_deferred_unstable_obs");
            } else if (commit_candidate && !commit_speed_ok) {
                NoteReject("commit_deferred_standstill");
            } else if (commit_candidate) {
                committed_wpnts_ = evasion_wpnts;
                committed_mrks_ = mrks;
                // 기하 전용 속도 상한을 같이 보관한다. 마지막으로 성공한
                // ApplySpeedProfile 이 이 경로를 만든 것이므로 점 수가 같다 —
                // 다르면(빌더가 뒤에 점을 손댄 경우) 비워서 종전 동작으로
                // 안전하게 떨어진다.
                if (last_geom_ceiling_.size() == evasion_wpnts.wpnts.size()) {
                    committed_geom_ceiling_ = last_geom_ceiling_;
                } else {
                    committed_geom_ceiling_.clear();
                }
                committed_lat_budget_ = active_lat_budget_;
                committed_start_s_ = evasion_wpnts.wpnts.front().s_m;
                committed_until_s_ = evasion_wpnts.wpnts.back().s_m;
                committed_tail_truncated_ = false;
                commit_collision_run_ = 0;
                // 조기 해제 판정용: 이 커밋이 대상으로 삼은 장애물의 뒷 끝.
                committed_obs_end_s_ = Mod(closest.s_end, track_len_);
            }
        }

        // ── 동적 상대 종방향 추종 (2026-08-22, 6/8단계) ────────────────────
        // 새 동적 계약 전체가 여기 있다. 횡은 한 줄도 정하지 않는다.
        //   · 간격이 개입 거리 밖이면 아무것도 하지 않는다 (레이스라인 속도).
        //   · 안쪽이면 목표 간격 dyn_follow_gap_m 를 유지하는 속도 상한을
        //     계산해서 **속도만** 누른다.
        // 회피 경로(정적)가 이미 있으면 그 경로의 vx 만 캡한다 — 경로 모양을
        // 건드리지 않으므로 "정적 회피가 최우선"이 구조적으로 지켜진다.
        // 회피 경로가 없으면 BuildTrailing 이 제동 프로파일을 만든다 (그쪽은
        // 거리별 곡선이라 예고 제동이 되고 캡은 스칼라라 반응 제어다 —
        // 횡 기동이 없을 때는 예고 쪽이 낫다).
        if (!perception_stale && !dyn_close.empty()) {
            const auto opponent = nearest_ahead(dyn_close);
            const double cap = OpponentFollowCap(opponent);
            const bool engaged = std::isfinite(cap);
            // ── 직선 인코스 추월 (7/8단계) ────────────────────────────────
            // 이 스택의 **유일한** 추월 시도다. 추종 중일 때만 열리고
            // (= "모든 회피 조건은 trailing 중"), 정적 회피 경로가 이미 있으면
            // 열지 않는다 — 회피가 우선이기 때문이다.
            // 성립하면 추종 캡을 **걸지 않는다**. 그것이 레이트 브레이크다:
            // 상대 속도에 묶이면 애초에 지나갈 수가 없다.
            if (engaged && !evasion_valid &&
                TryStraightOvertake(opponent, all_ahead_now,
                                    evasion_wpnts, mrks)) {
                ot_now = true;
                evasion_valid = true;
                if (!ot_active_) {
                    pitwall::Event("직선 인코스 추월 시도 (레이트 브레이크)");
                }
            }
            if (engaged && !ot_now) {
                follow_now = true;
                if (evasion_valid) {
                    const double cap_w =
                        cap / std::max(cmd_speed_scale_, 0.05);
                    int capped = 0;
                    for (auto &w : evasion_wpnts.wpnts) {
                        if (std::isfinite(w.vx_mps) && w.vx_mps > cap_w) {
                            w.vx_mps = cap_w;
                            ++capped;
                        }
                    }
                    if (capped > 0) {
                        NoteOutcome("follow_cap_on_path");
                    }
                } else {
                    evasion_valid = BuildTrailing(opponent, evasion_wpnts,
                                                  mrks, true, &all_ahead_now);
                    used_trailing = evasion_valid;
                }
                pitwall::Log("follow_cap", cap);
            }
            pitwall::Log("follow_gap",
                         Mod(opponent.s_start - cur_s_, track_len_));
        }
        pitwall::Log("follow_active", follow_now ? 1.0 : 0.0);
        pitwall::Log("overtake_active", ot_now ? 1.0 : 0.0);
    }
    if (follow_now && !follow_active_) {
        pitwall::Event("동적 상대 추종 시작 (간격 유지)");
    }
    follow_active_ = follow_now;
    // 추월 기동은 장애물이 사라진 사이클에도 반드시 내려가야 한다 —
    // 굳으면 다음 조우의 상승엣지를 영영 놓친다 (try_evade_now 와 같은 이유).
    ot_active_ = ot_now;
    if (used_trailing && !trailing_active_) {
        pitwall::Event("TRAILING 폴백 (회피 불가 — 감속 추종)");
    }
    trailing_active_ = used_trailing;
    // 트랙경계 회피 포기: 같은 장애물에 대해 30Hz로 재발생하므로(실측 8초에
    // 207회) 상승엣지에서만 발행한다. 회피가 성공하거나 장애물이 사라져
    // abort가 멎으면 플래그가 내려가 다음 발생을 다시 알린다.
    if (boundary_abort_now_ && !boundary_abort_active_) {
        pitwall::Event("회피 포기(트랙경계)");
    }
    boundary_abort_active_ = boundary_abort_now_;
    evade_attempt_active_ = try_evade_now;
    spline_fallback_active_ = spline_fallback_now;
    static_stop_active_ = static_stop_now;
    escape_arc_active_ = escape_arc_now;
    {
        // 컨트롤러 백스톱 게이팅용. 매 사이클 발행해야 노드가 죽었을 때
        // 컨트롤러가 마지막 true 에 붙어 있지 않는다 (아래 stale 처리 참조).
        std_msgs::msg::Bool escape_msg;
        escape_msg.data = escape_arc_now;
        p_escape_active_->publish(escape_msg);
        // "이번 사이클에 차를 세워 둘 의도가 없다". 컨트롤러의 정지 발진
        // 하한 게이트다 — 근거는 spliner_node.hpp 의 p_evade_committed_
        // 선언부 주석. 정지를 거부한 트레일링(allow_stop=false)도 포함한다.
        std_msgs::msg::Bool evade_msg;
        evade_msg.data = evasion_valid && (!used_trailing || trailing_no_stop);
        p_evade_committed_->publish(evade_msg);
    }

    if (!evasion_valid) {
        evasion_wpnts.wpnts.clear();
        visualization_msgs::msg::Marker del_mrk;
        del_mrk.header.stamp = this->now();
        del_mrk.action = visualization_msgs::msg::Marker::DELETEALL;
        mrks.markers.clear();
        mrks.markers.push_back(del_mrk);
    }

    p_evasion_wpnts_->publish(evasion_wpnts);
    p_markers_->publish(mrks);
    PublishRejectCounts();
}

std::vector<f1tenth_planning_msgs::msg::Obstacle> SplinerNode::FilterObstacles(
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> &obstacles) const {
    std::vector<f1tenth_planning_msgs::msg::Obstacle> close_obs;
    for (const auto &raw_obs : obstacles) {
        if (!std::isfinite(raw_obs.s_start) || !std::isfinite(raw_obs.s_end) ||
            !std::isfinite(raw_obs.d_right) || !std::isfinite(raw_obs.d_left)) {
            continue;
        }
        // 2026-08-22 (4/8단계): 등속 전파(PredictObstacleMovement)를 없앴다.
        // 전파는 상대를 **앞으로** 밀어 간격을 실제보다 크게 보이게 하므로
        // 추종에서는 안전과 반대 방향이다. 인지-계획 지연은 6단계 follow cap 의
        // 반응시간 항이 명시적으로 처리하고, 경로 충돌검사는 도착시각 기준
        // 전파(CheckPathCollision 의 motion_s/motion_d)를 그대로 쓴다.
        auto obs = raw_obs;
        double span = Mod(obs.s_end - obs.s_start, track_len_);
        if (span < 1e-3 && obs.size > 0.0) {
            span = obs.size;
            obs.s_start = Mod(obs.s_center - span / 2.0, track_len_);
            obs.s_end = Mod(obs.s_center + span / 2.0, track_len_);
        }
        const double longitudinal_uncertainty =
            vehicle_half_length_m_ + UncertaintyMargin(obs.s_var, 0.0, 0.0);
        const bool ego_inside =
            Mod(cur_s_ - obs.s_start, track_len_) <= span + longitudinal_uncertainty;
        const double start_ahead = Mod(obs.s_start - cur_s_, track_len_);
        if (!ego_inside &&
            start_ahead > lookahead_m_ + longitudinal_uncertainty) {
            continue;
        }

        // 차량 반폭, 관측 분산, 경로 도달시점까지의 횡이동을 corridor에
        // 포함한다. 현재는 경로 밖이어도 ETA 때 안으로 들어오는 긴 물체를
        // 초기에 누락하지 않는다.
        const double eta = planning_delay_sec_ +
            (ego_inside ? 0.0 :
                start_ahead / std::max(std::abs(cur_vs_), 0.5));
        const double d_var =
            (std::isfinite(obs.d_var) && obs.d_var > 0.0) ? obs.d_var : 0.0;
        const double vd_var =
            (std::isfinite(obs.vd_var) && obs.vd_var > 0.0) ? obs.vd_var : 0.0;
        // 정적 장애물은 시간이 지나도 안 움직이므로 vd_var의 시간 성장분을
        // 자른다 (static_time_uncertainty_cap_ 주석 참조). 위치 분산 d_var는
        // 그대로 — 관측 오차는 정적이든 아니든 존재한다.
        const double uncertainty_eta =
            obs.is_static ? std::min(eta, static_time_uncertainty_cap_) : eta;
        const double lateral_uncertainty =
            UncertaintyMargin(d_var, vd_var, uncertainty_eta);
        const double lateral_motion = obs.is_static ? 0.0 : obs.vd * eta;
        const double corridor = obs_traj_tresh_ +
            std::max(path_check_margin_, vehicle_half_width_m_) +
            lateral_uncertainty;
        if (obs.d_right + lateral_motion <= corridor &&
            obs.d_left + lateral_motion >= -corridor) {
            close_obs.push_back(obs);
        }
    }
    return close_obs;
}

std::pair<std::string, double> SplinerNode::MoreSpace(
    const f1tenth_planning_msgs::msg::Obstacle &obs) const {
    // 트랙 좌/우 빈 공간과 회피에 필요한 최소 공간 비교.
    // 목표 간격은 evasion_dist지만, 벽이 허용하는 만큼 apex를 줄여서라도
    // 아래 passable_width 이상 확보되면 회피를 시도한다 — 고정 간격을 못
    // 맞추면 바로 포기하던 것이 "회피포기 남발"의 원인 (특히 클러스터).
    // 부호 있는 여유를 그대로 쓴다 — abs를 씌우면 장애물이 벽을 넘어 걸친
    // (라이다 벽 반사 클러스터) 경우 음수 여유가 큰 양수로 뒤집혀 막힌 쪽을
    // "공간 있음"으로 오판한다.
    // 벽 여유는 apex 한 점이 아니라 hold 구간 최소값 (ApexFor와 동일 근거) —
    // 사전 필터가 낙관적으로 통과시킨 뒤 매 사이클 검증기에서 죽는 것을 막는다.
    double wall_left, wall_right;
    MinWallInSpan(Mod(obs.s_start - 0.5, track_len_),
                  Mod(obs.s_end + 0.5, track_len_), wall_left, wall_right);
    double left_gap = wall_left - obs.d_left;
    double right_gap = wall_right + obs.d_right;
    // 판정 기준은 "그 한쪽 방향으로 실제로 지나갈 수 있는가" 하나다.
    // 통과에 물리적으로 필요한 폭 = 장애물쪽 여유 + 벽쪽 여유이고, 이 둘은
    // 아래 두 검증기가 이미 강제한다:
    //   CheckPathCollision : max(path_check_margin, 차반폭)  (+ sigma_d)
    //   WithinTrackBoundary: RequiredBoundaryMargin(...)
    // 예전에는 여기서 evasion_dist_min(0.20)+경계여유(0.25)=0.45를 요구했는데,
    // 검증기가 실제로 필요로 하는 값은 0.15+0.25=0.40이라 **5cm 더 엄격했다**.
    // 그 5cm 때문에 통과 가능한 회피가 사전 필터에서 잘려 TRAILING으로 떨어졌다
    // (실폭 1.2m 구간에서 중앙 장애물이 정확히 이 경계에 걸린다).
    // 이제 사전 필터를 검증기와 같은 예산으로 맞추고, 최종 가부는 검증기가
    // 판단한다 — 통과 가능성이 있으면 일단 회피를 시도한다. 실제로 불가능하면
    // Tier2(스플라인) / Tier3(트레일링)가 그대로 받아낸다.
    // sigma_d는 여기서 빼둔다(장애물마다 다름) — 사전 필터는 낙관적이어야 한다.
    const double min_space =
        RequiredObstacleClearance(obs) +
        RequiredBoundaryMargin(spline_bound_mindist_);

    double apex_left = ApexFor("left", obs);
    double apex_right = ApexFor("right", obs);

    if (right_gap > min_space && left_gap < min_space) {
        return {"right", apex_right};
    }
    if (left_gap > min_space && right_gap < min_space) {
        return {"left", apex_left};
    }
    // 양쪽 다 가능(또는 불가능)하면 raceline에서 덜 벗어나는 쪽 선택.
    //
    // 2026-08-11: 그 전에 "실제로 비켜 가지는가"를 먼저 본다.
    // ApexFor는 apex를 벽 안쪽으로 clamp하므로, **막힌 쪽일수록 apex의 절댓값이
    // 작아진다**. 그대로 |apex| 비교에 넣으면 "덜 벗어나는 쪽"이 곧 "막힌 쪽"이
    // 되어, 통과 불가능한 사이드를 제안하게 된다.
    //   run_0810_233521 실측: side_switch_blocked_mid_evasion 2291프레임 중
    //   2258개(99%)가 이 패턴이었다. 대표 프레임 —
    //     장애물 d[-0.45, +0.13], 좌벽 1.34 / 우벽 0.47
    //     apex_left  = +0.58 (통과 가능)
    //     apex_right = -0.22 (우벽 0.47-0.25 에 clamp. 통과하려면 -0.85 필요)
    //   |0.58| > |0.22| 라 right 가 선택됐고, 상위에서 "이미 left로 회피 중"과
    //   충돌해 회피를 버리고 감속했다 — 왼쪽이 1.2 m 비어 있는데도.
    const double clear_need = RequiredObstacleClearance(obs);
    const bool left_clears = apex_left >= obs.d_left + clear_need;
    const bool right_clears = apex_right <= obs.d_right - clear_need;
    if (left_clears != right_clears) {
        return left_clears ? std::pair<std::string, double>{"left", apex_left}
                           : std::pair<std::string, double>{"right", apex_right};
    }
    if (!left_clears && !right_clears) {
        // 둘 다 안 되는 경우에도 |apex| 를 쓰면 안 된다 — 위와 같은 이유로
        // "더 심하게 막힌 쪽"이 이긴다. 여기서 고른 사이드는 last_ot_side_ 로
        // 커밋되어 다음 사이클의 사이드 유지 히스테리시스를 좌우하므로, 지금
        // 실패하더라도 **가장 덜 막힌 쪽**을 남겨야 재시도가 그쪽으로 수렴한다.
        // (위 두 branch 가 gap 으로 이미 한쪽을 골라내지만, MinWallInSpan 이
        //  장애물 s-구간 전체의 최소 벽폭을 쓰기 때문에 클러스터가 길면 양쪽
        //  gap 이 동시에 min_space 아래로 내려와 여기로 떨어진다 — 관측된
        //  2258 프레임이 그 경로다.)
        //
        // 2026-08-16: 비교 대상을 left_gap/right_gap 에서 **기동 전체의 최악
        // 벽여유**로 바꿨다. gap 은 장애물 s-구간의 벽만 보므로, 장애물 직후에
        // 좁아지는 코너에서는 "덜 막힌 쪽"이 곧 "빠져나올 수 없는 쪽"이 된다.
        //   run_0815_191056 s≈27.6 (헤어핀2 출구, 장애물 d[-0.41,+0.10]):
        //     left_gap 0.32 / right_gap 0.43  -> right 선택
        //     그런데 우벽은 1.2m 뒤 0.84 -> 0.47 로 줄어 복귀 불가:
        //       right: 기동 최악여유 -0.02 m, 복귀 도달 불가
        //       left : 기동 최악여유 +0.10 m, 복귀 도달 가능
        //   9랩 중 5랩이 이 11cm 차이로 right 를 골랐고, 마지막 랩에서
        //   2.9초 벽 접촉(s=28.9)으로 끝났다.
        // 도달 불가한 사이드는 큰 페널티로 강등한다 — 여유가 조금 더 넓어도
        // 조향 한계상 복귀할 수 없으면 그 사이드는 선택지가 아니다.
        bool reach_left = true, reach_right = true;
        const double score_left =
            ManeuverWorstMargin(apex_left, obs, &reach_left) -
            (reach_left ? 0.0 : 1.0);
        const double score_right =
            ManeuverWorstMargin(apex_right, obs, &reach_right) -
            (reach_right ? 0.0 : 1.0);
        return (score_left >= score_right)
            ? std::pair<std::string, double>{"left", apex_left}
            : std::pair<std::string, double>{"right", apex_right};
    }
    // 둘 다 통과 가능 — 이때만 "raceline 에서 덜 벗어나는 쪽"이 의미가 있다.
    // 단, 복귀가 조향 한계로 불가능한 사이드는 제외한다. raceline 에 가깝다는
    // 이유로 빠져나올 수 없는 코리도를 고르면 위(둘 다 불가) 분기와 같은
    // 사고가 난다 — 판정 근거는 ManeuverWorstMargin 주석.
    const bool prefer_left = std::abs(apex_left) <= std::abs(apex_right);
    bool reach_pref = true;
    ManeuverWorstMargin(prefer_left ? apex_left : apex_right, obs, &reach_pref);
    if (!reach_pref) {
        bool reach_alt = true;
        ManeuverWorstMargin(prefer_left ? apex_right : apex_left, obs,
                            &reach_alt);
        if (reach_alt) {
            return prefer_left
                ? std::pair<std::string, double>{"right", apex_right}
                : std::pair<std::string, double>{"left", apex_left};
        }
    }
    if (prefer_left) {
        return {"left", apex_left};
    }
    return {"right", apex_right};
}

std::vector<f1tenth_planning_msgs::msg::Obstacle> SplinerNode::EvasionGroup(
    const f1tenth_planning_msgs::msg::Obstacle &closest,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> &all_ahead) const {
    std::vector<f1tenth_planning_msgs::msg::Obstacle> group{closest};
    if (track_len_ <= 0.0) {
        return group;
    }
    // 자차 기준 상대거리로 unwrap — 뒤에 있는 장애물은 ~track_len 이 되어
    // 아래 reach/lookahead 비교에서 자연히 걸러진다.
    auto rel = [&](double s) { return Mod(s - cur_s_, track_len_); };
    // 처음 도달범위 = closest 를 지난 뒤 복귀 블렌드가 끝나는 지점.
    // 그 안에 다른 장애물이 있으면 "복귀하다 들이받는다".
    double reach = std::min(rel(closest.s_end) + lane_blend_out_m_, lookahead_m_);
    std::vector<char> taken(all_ahead.size(), 0);
    // 매 반복 **가장 가까운** 후보 하나만 넣는다.
    // 예전에는 한 패스에서 조건을 만족하는 것을 전부 push 하고 크기 검사는
    // while 조건에만 있어서, 사정권에 5개가 있으면 kEvasionGroupMax(4)를 넘겨
    // 한 번에 6개가 됐다 — 상한이 강제되지 않았다. 게다가 all_ahead 는 s 로
    // 정렬돼 있지 않아 잘라내면 임의의 부분집합이 남는다. 가까운 것부터 넣으면
    // 상한이 실제로 걸리고, reach 가 자라는 순서도 물리적으로 맞다.
    while (group.size() < kEvasionGroupMax) {
        size_t best = all_ahead.size();
        double best_a = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < all_ahead.size(); ++i) {
            if (taken[i]) {
                continue;
            }
            const auto &o = all_ahead[i];
            if (!std::isfinite(o.s_start) || !std::isfinite(o.s_end) ||
                !std::isfinite(o.d_left) || !std::isfinite(o.d_right)) {
                taken[i] = 1;
                continue;
            }
            if (o.id == closest.id) {
                taken[i] = 1;      // closest 본인
                continue;
            }
            const double a = rel(o.s_start);
            if (a > reach || a > lookahead_m_) {
                continue;          // 아직 사정권 밖 (reach 가 자라면 다시 본다)
            }
            if (a < best_a) {
                best_a = a;
                best = i;
            }
        }
        if (best >= all_ahead.size()) {
            break;                 // 더 넣을 것이 없다
        }
        taken[best] = 1;
        group.push_back(all_ahead[best]);
        reach = std::min(
            std::max(reach, rel(all_ahead[best].s_end) + lane_blend_out_m_),
            lookahead_m_);
    }
    return group;
}

double SplinerNode::GroupApex(
    const std::string &side,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> &group,
    bool *clears_all) const {
    // 가장 요구가 큰 apex 하나로 그룹 전체를 덮는다. 왼쪽이면 최대, 오른쪽이면
    // 최소 — ApexFor 가 이미 각 장애물의 벽 클램프를 반영하므로, 이 max/min 은
    // "벽 안에서 모두를 비켜 가는 가장 가까운 오프셋"이 된다.
    bool ok = !group.empty();
    // sentinel 은 반드시 infinity 여야 아래 !isfinite 가드가 동작한다.
    // DBL_MAX 는 isfinite 가 true 라, 모든 apex 가 비정상이면 1.8e308 이
    // d_hold 후보로 그대로 반환됐다.
    double d = (side == "left") ? -std::numeric_limits<double>::infinity()
                                : std::numeric_limits<double>::infinity();
    for (const auto &o : group) {
        const double a = ApexFor(side, o);
        if (!std::isfinite(a)) {
            ok = false;
            continue;
        }
        d = (side == "left") ? std::max(d, a) : std::min(d, a);
    }
    if (!std::isfinite(d) || group.empty()) {
        if (clears_all) {
            *clears_all = false;
        }
        return 0.0;
    }
    // 그 오프셋이 **정말로** 전부를 비켜 가는가. ApexFor 는 벽에 클램프되므로
    // max/min 을 취해도 어떤 장애물은 여전히 못 비켜 갈 수 있다 — 그때는
    // 이 사이드로 한 번에 통과할 수 없다는 뜻이고, 지그재그가 필요하다.
    for (const auto &o : group) {
        const double need = RequiredObstacleClearance(o);
        const bool cleared = (side == "left") ? (d >= o.d_left + need)
                                              : (d <= o.d_right - need);
        if (!cleared) {
            ok = false;
        }
    }
    if (clears_all) {
        *clears_all = ok;
    }
    return d;
}

bool SplinerNode::HoldClearsOthers(
    double d_hold,
    const f1tenth_planning_msgs::msg::Obstacle &closest_obs,
    double hold_through_s,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> &all_ahead,
    f1tenth_planning_msgs::msg::Obstacle *blocker) const {
    if (!std::isfinite(d_hold) || track_len_ <= 0.0) {
        return true;
    }
    // hold 구간은 BuildLaneSwitch 가 실제로 오프셋을 유지하는 범위와 같아야
    // 한다 — 여기서 낙관적으로 좁게 잡으면 사전 필터를 통과한 뒤 검증기에서
    // 죽는, 지금 고치려는 것과 같은 형태의 구멍이 생긴다.
    // (장애물 앞뒤 0.5m 여유까지 포함. BuildLaneSwitch 의 hold_start/hold_end.)
    const double span = Mod(closest_obs.s_end - closest_obs.s_start, track_len_);
    const double behind = Mod(cur_s_ - closest_obs.s_start, track_len_);
    const bool alongside = behind < span + 1.0;
    const double dist_front =
        alongside ? 0.0 : Mod(closest_obs.s_start - cur_s_, track_len_);
    double dist_back = alongside
        ? Mod(closest_obs.s_end - cur_s_, track_len_)
        : dist_front + span;
    if (std::isfinite(hold_through_s)) {
        const double through = Mod(hold_through_s - cur_s_, track_len_);
        if (through <= lookahead_m_) {
            dist_back = std::max(dist_back, through);
        }
    }
    const double hold_start = std::max(dist_front - 0.5, 0.0);
    const double hold_end = dist_back + 0.5;

    for (const auto &o : all_ahead) {
        if (o.id == closest_obs.id) {
            continue;   // 호출자가 사이드까지 따져서 따로 본다
        }
        if (!std::isfinite(o.s_start) || !std::isfinite(o.s_end) ||
            !std::isfinite(o.d_left) || !std::isfinite(o.d_right)) {
            continue;
        }
        // 자차 기준 상대거리로 unwrap — 뒤에 있는 장애물은 ~track_len 이 되어
        // 아래 겹침 비교에서 자연히 걸러진다 (EvasionGroup 과 같은 규약).
        const double a = Mod(o.s_start - cur_s_, track_len_);
        const double b = a + Mod(o.s_end - o.s_start, track_len_);
        if (a > hold_end || b < hold_start) {
            continue;
        }
        // 사이드를 묻지 않는다 — d_hold 가 이 박스의 어느 쪽이든 요구 간격
        // 밖이면 지나갈 수 있다. 통로가 **레이스라인이 아닌 곳**에 있는
        // 배치(장애물 사이 게이트)에서 사이드를 강제하면 정답을 잃는다.
        const double need = RequiredObstacleClearance(o);
        if (d_hold >= o.d_left + need || d_hold <= o.d_right - need) {
            continue;
        }
        if (blocker != nullptr) {
            *blocker = o;
        }
        return false;
    }
    return true;
}

double SplinerNode::ApexFor(
    const std::string &side,
    const f1tenth_planning_msgs::msg::Obstacle &obs) const {
    // apex 는 hold 구간(장애물 앞뒤 0.5 m)에서 유지하는 오프셋이므로, 벽 여유도
    // **그 구간에 도달했을 때** 속도로 재야 한다. 아직 도달 전이면 그만큼
    // 감속 크레딧을 준다 (RequiredBoundaryMarginAt 주석 — 이 한 곳이
    // run_0819_180345 t=267.05 의 6 mm 를 만든 자리다).
    // 이미 옆에 들어왔거나 사정권 밖이면 크레딧 0 = 종전과 같다.
    double dist_to_hold =
        Mod(Mod(obs.s_start - 0.5, track_len_) - cur_s_, track_len_);
    if (!std::isfinite(dist_to_hold) || dist_to_hold > lookahead_m_) {
        dist_to_hold = 0.0;
    }
    const double boundary_margin =
        RequiredBoundaryMarginAt(spline_bound_mindist_, dist_to_hold);
    // apex 오프셋은 장애물 옆을 지나는 hold 구간 내내 유지되므로, 벽 여유도
    // 그 구간에서 가장 좁은 값을 써야 한다. 한 점(gb_idx)의 벽으로 정하면
    // 같은 구간의 더 좁은 지점에서 경계 클램프가 걸리고, 그 클램프가 장애물
    // 간격을 침범해 회피가 통째로 포기된다 (run_0806_155621: apex의 우벽
    // 0.595 기준으로 뽑은 d_apex가 진입부 우벽 0.445에서 클램프 -> 매 사이클 abort).
    // 범위는 DoSpline/BuildLaneSwitch의 hold 구간과 동일한 장애물 앞뒤 0.5m.
    double wall_left, wall_right;
    MinWallInSpan(Mod(obs.s_start - 0.5, track_len_),
                  Mod(obs.s_end + 0.5, track_len_), wall_left, wall_right);
    // 목표/하한 모두 검증기(CheckPathCollision)가 요구하는 간격 이상이어야
    // 한다 — 아니면 계획한 회피를 자기 검증기가 매번 충돌로 거부한다.
    //
    // "이상"으로는 부족하다. need와 정확히 같으면 목표 apex가 검증기의
    // 충돌 박스 **경계 위**에 놓이고, 샘플 좌표의 수 mm 오차만으로 뒤집힌다.
    // 0807 run_164326에서 매 랩 16회 "회피 실패"가 났는데 재구성상 관통량이
    // -0.11 ~ -10.3 mm, 즉 전부 이 동률이었다. 5cm 에폭실론이면 15/15가
    // 뒤집힌다. (근본적으로는 진입 예산과 검증기 종방향 박스를 같은 기준으로
    // 맞추는 것이 옳고, BuildLaneSwitch/DoSpline의 entry_avail이 그 몫이다.)
    const double need =
        RequiredObstacleClearance(obs) + apex_clearance_eps_m_;
    // 하한은 검증기 요구치 그 자체다. (예전엔 max(evasion_dist_min_, need)
    // 였는데 evasion_dist_min_(0.20)이 need(0.35~0.45)를 이긴 적이 없어
    // 항등이었다 — 파라미터 자체를 제거했다.)
    const double dist_min = need;
    // 목표는 evasion_dist 지만, 그것도 need 보다 작으면 의미가 없다.
    // ⚠️ dist_target == dist_min 이 되면 아래 balanced(코리도 여유를
    //    장애물/벽에 반씩 배분) 로직이 항등이 된다 — evasion_dist 를
    //    need 보다 유의미하게 크게 잡아야 그 배분이 실제로 동작한다.
    const double dist_target = std::max(evasion_dist_, need);
    if (side == "left") {
        double wall = wall_left;
        // 부호를 살린다 (MoreSpace 의 left_gap/right_gap 과 같은 규약).
        // abs 를 씌우면 장애물이 벽을 넘어 걸친 경우(라이다 벽 반사) 음수
        // corridor 가 큰 양수로 뒤집혀 surplus 가 부풀었다. 최종 clamp 가
        // 흡수해 출력은 같았지만, 식이 반대 방향을 계산하고 있었다.
        double corridor = wall_left - obs.d_left;
        double surplus =
            std::max(corridor - dist_min - boundary_margin, 0.0);
        // corridor 여유를 장애물/벽에 반씩 배분한 지점 (넓으면 목표값이 더 작음)
        double balanced = obs.d_left + dist_min + surplus / 2.0;
        double target = obs.d_left + dist_target;
        return std::clamp(std::min(target, balanced), 0.0,
                          std::max(wall - boundary_margin, 0.0));
    }
    double wall = wall_right;
    double corridor = wall_right + obs.d_right;   // 부호 유지 (위와 같은 이유)
    double surplus =
        std::max(corridor - dist_min - boundary_margin, 0.0);
    double balanced = obs.d_right - dist_min - surplus / 2.0;
    double target = obs.d_right - dist_target;
    return std::clamp(std::max(target, balanced),
                      -std::max(wall - boundary_margin, 0.0), 0.0);
}

double SplinerNode::ManeuverWorstMargin(
    double apex, const f1tenth_planning_msgs::msg::Obstacle &obs,
    bool *reachable) const {
    if (reachable) {
        *reachable = true;
    }
    const int gb_size = static_cast<int>(gb_wpnts_.size());
    if (gb_size < 2 || !(track_len_ > 0.0) || !std::isfinite(apex)) {
        return 0.0;
    }
    const double step = std::max(wpnt_dist_, 0.01);
    const double a = std::abs(apex);
    const bool left = apex >= 0.0;
    auto wall_at = [&](int gi) {
        return left ? std::abs(gb_wpnts_[gi].d_left)
                    : std::abs(gb_wpnts_[gi].d_right);
    };
    auto idx_at = [&](double s) {
        return static_cast<int>(Mod(s, track_len_) / step) % gb_size;
    };

    // (1) hold — 장애물 앞뒤 0.5 m. ApexFor/DoSpline/BuildLaneSwitch 와 같은 창.
    const double hold_s0 = Mod(obs.s_start - 0.5, track_len_);
    const double hold_span = std::min(
        Mod((obs.s_end + 0.5) - (obs.s_start - 0.5), track_len_),
        track_len_ - step);
    double worst = std::numeric_limits<double>::infinity();
    const int hold_steps = std::max(1, static_cast<int>(hold_span / step));
    for (int i = 0; i <= hold_steps; ++i) {
        worst = std::min(
            worst,
            wall_at(idx_at(hold_s0 + i * step)) - a - vehicle_half_width_m_);
    }

    // (2) 복귀 블렌드 — TryLaneSwitch 의 blend_out 초기값과 같은 예산.
    //     여기서만 hold 창 밖을 본다: 사고가 난 곳이 정확히 이 구간이다.
    const double ret_s0 = Mod(obs.s_end + 0.5, track_len_);
    const double blend_out = std::min(
        std::max(lane_blend_out_m_,
                 BlendDist(apex, PlanSpeed(), blend_lat_acc_max_, 0.0)),
        track_len_ - step);
    const int out_steps = std::max(1, static_cast<int>(blend_out / step));
    const double kappa_steer_max = SteerKappaMax();
    const double reserve = TrackingReserve(PlanSpeed());
    for (int i = 0; i <= out_steps; ++i) {
        const double dist = i * step;
        const double w = static_cast<double>(i) / static_cast<double>(out_steps);
        const double prof = a * (1.0 - w * w * (3.0 - 2.0 * w));
        const int gi = idx_at(ret_s0 + dist);
        const double wall = wall_at(gi);
        worst = std::min(worst, wall - prof - vehicle_half_width_m_);

        // 복귀 도달성: 그 지점의 벽이 허용하는 |d| 상한까지 dist 안에
        // 내려오려면 피크 곡률 6*Δd/dist² 이 필요하다. 조향 한계(트랙 곡률
        // 차감)를 넘으면 "계획은 그려도 차가 못 그리는" 복귀다.
        // 판정식은 BlendGeomFloor 의 스칼라 경로와 같은 식을 쓴다.
        if (reachable == nullptr || dist < step || kappa_steer_max <= 1e-3) {
            continue;
        }
        const double limit =
            std::max(wall - vehicle_half_width_m_ - reserve, 0.0);
        const double drop = a - limit;
        if (drop <= 0.0) {
            continue;
        }
        double kt = gb_wpnts_[gi].kappa_radpm;
        if (!std::isfinite(kt)) {
            kt = 0.0;
        }
        const double spare = geom_floor_margin_ *
            std::max(kappa_steer_max - std::abs(kt),
                     geom_floor_kappa_ratio_ * kappa_steer_max);
        if (spare <= 1e-6 || 6.0 * drop / (dist * dist) > spare) {
            *reachable = false;
        }
    }
    return std::isfinite(worst) ? worst : 0.0;
}

bool SplinerNode::TryLaneSwitch(
    const f1tenth_planning_msgs::msg::Obstacle &closest_obs,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> &all_ahead,
    f1tenth_planning_msgs::msg::WpntArray &out_wpnts,
    visualization_msgs::msg::MarkerArray &out_mrks) {

    int gb_size = static_cast<int>(gb_wpnts_.size());
    if (gb_size < 2) {
        return false;
    }

    // apex s = 장애물 중심 (s wrap 처리, DoSpline과 동일. 로그용)
    double s_apex;
    if (closest_obs.s_end < closest_obs.s_start) {
        s_apex = (closest_obs.s_end + track_len_ + closest_obs.s_start) / 2.0;
    } else {
        s_apex = (closest_obs.s_end + closest_obs.s_start) / 2.0;
    }

    // ── 사이드/오프셋을 "연쇄 그룹" 전체로 정한다 (2026-08-11) ────────────
    // closest 하나만 보면, 그 회피가 끝나기 전에 뒤 장애물이 closest 가 되며
    // 같은 상황의 답이 뒤집힌다. 그러면 아래 committed_side 가드가 발동해
    // 회피가 죽고 Tier3(정지)로 떨어진다 — 재생 실측으로 이것이 남은
    // side_switch_blocked 의 96%였다 (EvasionGroup 주석).
    const auto evasion_group = EvasionGroup(closest_obs, all_ahead);
    bool left_all = false, right_all = false;
    const double group_left = GroupApex("left", evasion_group, &left_all);
    const double group_right = GroupApex("right", evasion_group, &right_all);

    // 사이드: MoreSpace(빈 공간)가 우선. 단 이미 회피 중(|d|>thresh)이면
    // 히스테리시스로 현재 사이드를 유지한다 — mid-evasion 사이드 뒤집힘은
    // 장애물 사이로 끌려드는 충돌 원인 (DoSpline과 동일 정책).
    std::string more_side = MoreSpace(closest_obs).first;
    // 그룹이 둘 이상이고 **한쪽만** 전부를 비켜 갈 수 있으면 그쪽이 답이다.
    // MoreSpace 는 closest 하나만 보므로 여기서 뒤 장애물의 정보를 반영한다.
    // (양쪽 다 되거나 둘 다 안 되면 MoreSpace 의 판단을 그대로 둔다 — 후자는
    //  지그재그가 필요한 경우이고 이 티어의 범위 밖이다.)
    if (evasion_group.size() > 1 && left_all != right_all) {
        const std::string group_side = left_all ? "left" : "right";
        if (group_side != more_side) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "LaneSwitch 사이드 재선택: %s -> %s (연쇄 장애물 %zu개를 한 번에 "
                "비켜 갈 수 있는 쪽)", more_side.c_str(), group_side.c_str(),
                evasion_group.size());
            NoteReject("lane_group_side_override");
        }
        more_side = group_side;
    }
    std::string first = more_side;
    if (SideKeepActive(closest_obs)) {
        first = last_ot_side_;
    }
    std::string second = (first == "left") ? "right" : "left";

    // ── 회피 중 사이드 전환 금지 (DoSpline과 같은 정책) ────────────────
    // 이미 한쪽으로 몸이 나가 있으면 반대편 레인 후보를 아예 만들지 않는다.
    // d=+0.68 에서 오른쪽으로 가려면 트랙을 가로질러야 하고 그 경로는 장애물
    // 정면을 지난다 — 0808 run_185049 에서 통과 직후 4랩(85.22/109.26/
    // 122.09/134.19)에 발생해 로컬 경로가 반대편으로 점프하며 control 추종
    // 백스톱을 발화시켰다(경로오차 0.37~0.63m -> 속도 1.5 m/s 캡).
    // DoSpline 에는 이 가드가 있었지만 **1순위인 여기에는 없었다**. Tier1이
    // 항상 먼저 도는 구조라 DoSpline 의 가드는 사실상 도달하지 않는 코드였다.
    // 여기서 false 를 돌려도 geom_infeasible 플래그를 세우지 않으므로 Tier3는
    // 정지가 아니라 trailing_retry_speed 감속으로 받는다 — 가로지르기보다 안전.
    // OtSideCommitFresh: 이 가드는 "지금 회피 중"일 때만 의미가 있다. 예전엔
    // last_ot_side_ 가 영구히 남아서, 장애물이 없던 구간의 추종오차(직선 |d|
    // p95 0.26 > 문턱 0.20)만으로도 반대편 후보가 통째로 사라졌다.
    // (empty() 검사는 OtSideCommitFresh 안에 포함돼 있다.)
    // SideKeepActive 를 그대로 쓰지 않는 이유: 그쪽 (1)번 같은-장애물 잠금은
    // |d| 가 작은 접근 구간에서도 참이라, 여기에 쓰면 유지 사이드가 막혔을 때
    // 반대편 후보까지 못 만들어 "모든 레인 막힘 -> 정지"가 된다
    // (0807 run_164645 헤어핀 전패와 같은 형태). 여기서는 "몸이 이미 그쪽으로
    // 나가 있다"는 원래 조건을 유지하고 수명만 건다.
    bool committed_side = OtSideCommitFresh() &&
                          std::abs(cur_d_) > side_switch_d_tresh_ &&
                          ((cur_d_ > 0.0) ? (last_ot_side_ == "left")
                                          : (last_ot_side_ == "right"));

    // 2026-08-12: **잠금은 잠긴 쪽이 아직 성립할 때만 유효하다.**
    // 잠긴 사이드가 벽 클램프로 이미 막혔는데도 잠금을 유지하면 남는 선택이
    // "막힌 쪽을 붙잡고 감속" 하나뿐이라 회피 실패로 직결된다.
    // run_0812_013630 에서 이 거부가 lane 쪽 86 프레임 / OT 쪽 78 프레임
    // 발동했고, 로그로 확인된 7회 전부 **반대쪽은 성립**했다 (1:15 / 2:20).
    // 임계값 0.20 자체가 노이즈 바닥 아래다 — 장애물 없는 구간에서도
    // |d| > 0.20 인 시간이 36.4%(p90 0.288, p95 0.330)이므로 이 조건은
    // 회피 의도가 아니라 평상시 추종 흔들림에 걸린다.
    // 아래 루프의 clears 판정과 같은 식을 미리 한 번 돌린다 (부작용 없음).
    const auto side_clears = [&](const std::string &side) {
        const bool group_ok = (side == "left") ? left_all : right_all;
        if (group_ok) {
            return true;
        }
        const double apex = ApexFor(side, closest_obs);
        const double need_c = RequiredObstacleClearance(closest_obs);
        const bool closest_ok =
            (side == "left") ? (apex >= closest_obs.d_left + need_c)
                             : (apex <= closest_obs.d_right - need_c);
        // 아래 루프와 같은 식이어야 한다 — 잠긴 사이드가 "closest 는 비켜
        // 가지만 그 레인 위에 다른 장애물이 서 있는" 상태면 그 사이드는
        // 성립하지 않는 것이고, 잠금을 유지할 이유가 없다.
        //
        // ── 검사 구간은 closest 가 아니라 **그룹 끝**까지다 (2026-08-20) ────
        // 종전에는 hold_through_s 로 closest_obs.s_end 를 넘겨, HoldClearsOthers
        // 가 보는 창이 [앞끝-0.5, closest.s_end+0.5] 뿐이었다. 그래서 그룹
        // 안의 **뒤쪽** 장애물이 잠긴 레인 위에 서 있어도 창 밖이라 안 보였고,
        // "잠긴 쪽이 깨끗하다"로 판정돼 양보가 열리지 않았다.
        //   run_0820_201714 4:45 (대각 배치): 박스 A s=35.2~35.6 d[-0.16,+0.18],
        //   박스 B s=37.95~38.3 d[+0.47,+0.81]. 좌측 apex 0.53 은 A 는 비켜
        //   가지만 B 에 정면으로 막힌다. B 는 A 끝에서 2.35 m 뒤 — 창(0.5 m)
        //   밖이라 잠금이 유지됐고, 215 사이클이 lane_side_switch_blocked_mid_
        //   evasion 으로 트레일링에 떨어졌다(그중 211 = 98%). 같은 런에서
        //   양보(lane_side_switch_yielded)는 327 초 동안 2 번 열렸다.
        // 아래 실제 레인 생성 루프가 hold 를 그룹 끝까지 늘리는 것과 같은 식을
        // 쓴다 — 사전 판정과 생성이 같은 구간을 봐야 한다.
        double lock_through_s = closest_obs.s_end;
        if (evasion_group.size() > 1 && track_len_ > 0.0) {
            double far = Mod(closest_obs.s_end - cur_s_, track_len_);
            for (const auto &o : evasion_group) {
                far = std::max(far, Mod(o.s_end - cur_s_, track_len_));
            }
            lock_through_s = Mod(cur_s_ + far, track_len_);
        }
        return closest_ok &&
               HoldClearsOthers(apex, closest_obs, lock_through_s,
                                all_ahead, nullptr);
    };
    if (committed_side && !side_clears(last_ot_side_)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "LaneSwitch 사이드 잠금 양보: 잠긴 %s가 막혔다 (cur_d=%.2f) — "
            "반대쪽 후보를 허용한다", last_ot_side_.c_str(), cur_d_);
        NoteReject("lane_side_switch_yielded");
        committed_side = false;
    }

    // 후보 레인: 우선 사이드 → 반대 사이드. 목표 오프셋 d_hold = ApexFor
    // (장애물 가장자리 + evasion_dist, 벽 클램프). 벽 클램프로 검증기 요구
    // 간격조차 못 챙기면 그 레인은 막힌 것 → 스킵. 첫 성공 레인을 반환한다.
    for (const std::string &side : {first, second}) {
        if (committed_side && side != last_ot_side_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "LaneSwitch 사이드 전환 거부: 이미 %s로 회피 중 (cur_d=%.2f) — "
                "가로지르지 않고 감속", last_ot_side_.c_str(), cur_d_);
            NoteReject("lane_side_switch_blocked_mid_evasion");
            continue;
        }
        // 오프셋도 그룹 전체 기준. 그룹이 하나면 ApexFor(closest) 와 같다.
        const bool group_clears = (side == "left") ? left_all : right_all;
        double d_hold = (side == "left") ? group_left : group_right;
        // 그룹 전체는 못 비켜 가더라도 closest 하나는 비켜 갈 수 있으면 그건
        // 시도할 값어치가 있다 — 뒤 장애물은 다음 사이클에 다시 판단한다.
        // (예전 동작으로의 폴백. 이때는 hold 연장도 하지 않는다.)
        bool clears = group_clears;
        double hold_through_s = closest_obs.s_end;
        if (group_clears && evasion_group.size() > 1) {
            // hold 를 그룹 마지막 장애물 뒤까지 늘린다 (복귀 중 충돌 방지)
            double far = Mod(closest_obs.s_end - cur_s_, track_len_);
            for (const auto &o : evasion_group) {
                far = std::max(far, Mod(o.s_end - cur_s_, track_len_));
            }
            hold_through_s = Mod(cur_s_ + far, track_len_);
        }
        if (!group_clears) {
            d_hold = ApexFor(side, closest_obs);
            const double need_c = RequiredObstacleClearance(closest_obs);
            clears = (side == "left") ? (d_hold >= closest_obs.d_left + need_c)
                                      : (d_hold <= closest_obs.d_right - need_c);
        }
        if (!clears) {
            const double need_clear = RequiredObstacleClearance(closest_obs);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "LaneSwitch %s 레인 불가: d_hold=%.2f (필요 %.2f, 장애물 d[%.2f,%.2f])",
                side.c_str(), d_hold,
                (side == "left") ? closest_obs.d_left + need_clear
                                 : closest_obs.d_right - need_clear,
                closest_obs.d_right, closest_obs.d_left);
            NoteReject("lane_wall_no_room");
            continue;
        }
        // ── 레인 위의 **다른** 장애물 (2026-08-18) ────────────────────────
        // 위 clears 는 closest_obs(또는 GroupApex 가 본 그룹) 하나만 본
        // 판정이다. 그 하나를 비켜 갔다고 그 오프셋이 빈 것은 아니다.
        //   run_0818_205422 25.31s 충돌 — s=35.5 직선에 콘 두 개가
        //   #13 d[-0.25,+0.10] (레이스라인 위) / #14 d[+0.68,+1.02] (좌벽 옆),
        //   사이 통로 0.58~0.64 m. closest 는 s_end 가 6 cm 작은 #14 였고,
        //   ApexFor("right", #14) 는 상한 클램프 0.0 때문에 **d_hold=0.00**,
        //   즉 레이스라인 그 자체를 돌려줬다. 0.00 <= 0.68-0.27 이라 이
        //   clears 를 통과했고("LaneSwitch right d_hold=0.00" 로그),
        //   ok_lane_switch 로 커밋까지 먹었다. 그 레이스라인 위에 #13 이
        //   서 있다는 사실은 이 단계에서 아무도 보지 않았다.
        //   0.24초 뒤 committed_break(path_collision) 로 깨졌지만, 그 사이
        //   **#13 이 closest 가 된 유일한 프레임(22.60s)** 이 커밋 홀드로
        //   지나가 버렸다 — 그 프레임의 답은 d=+0.36 으로 통과였다.
        //   (다음 랩 37.77s 에 같은 프레임이 커밋 없이 와서 6.2 m/s 통과.)
        // 그래서 "이 오프셋이 hold 구간 전체에서 비어 있는가"를 여기서 본다.
        // 검증기(CheckPathCollision)가 어차피 잡아내지만, 여기서 걸러야
        // 가짜 성공이 커밋/retry_d_target_ 을 오염시키지 않는다.
        f1tenth_planning_msgs::msg::Obstacle hold_blocker;
        if (!HoldClearsOthers(d_hold, closest_obs, hold_through_s, all_ahead,
                              &hold_blocker)) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "LaneSwitch %s 레인 불가: d_hold=%.2f 위에 다른 장애물 "
                "(id %d, s[%.1f,%.1f], d[%.2f,%.2f], 필요 %.2f)",
                side.c_str(), d_hold, hold_blocker.id,
                hold_blocker.s_start, hold_blocker.s_end,
                hold_blocker.d_right, hold_blocker.d_left,
                RequiredObstacleClearance(hold_blocker));
            NoteReject("lane_hold_blocked_by_other");
            continue;
        }
        // 이 레인은 장애물 여유를 만족한다 — 뒤이은 속도/충돌 검증에서
        // 떨어지더라도 "여기로 가려던 중"이라는 사실은 남긴다. 트레일링이
        // raceline으로 되돌리는 대신 이 목표를 유지하는 근거가 된다
        // (retry_d_target_ 주석 참조).
        NoteRetryTarget(d_hold);
        if (BuildLaneSwitch(d_hold, closest_obs, hold_through_s, all_ahead,
                            out_wpnts, out_mrks)) {
            CommitOtSide(side, closest_obs);
            NoteOutcome("ok_lane_switch");
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "LaneSwitch %s d_hold=%.2f s_apex=%.1f (%zu wpnts)",
                side.c_str(), d_hold, Mod(s_apex, track_len_),
                out_wpnts.wpnts.size());
            return true;
        }
        out_wpnts.wpnts.clear();
        out_mrks.markers.clear();
    }
    return false;
}

bool SplinerNode::BuildLaneSwitch(
    double d_hold,
    const f1tenth_planning_msgs::msg::Obstacle &closest_obs,
    double hold_through_s,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> &all_ahead,
    f1tenth_planning_msgs::msg::WpntArray &out_wpnts,
    visualization_msgs::msg::MarkerArray &out_mrks) {

    int gb_size = static_cast<int>(gb_wpnts_.size());
    if (gb_size < 2) {
        return false;
    }

    // 장애물까지의 상대 거리. 추월 중이면 ego가 상대 몸통 옆까지 진입(alongside).
    double span = Mod(closest_obs.s_end - closest_obs.s_start, track_len_);
    double behind = Mod(cur_s_ - closest_obs.s_start, track_len_);
    bool alongside = behind < span + 1.0;
    // 옆에 붙었는데 아직 회피 오프셋이 없으면(=이제 막 감지) 새 레인스위치
    // 기하가 안 나온다 — 진입 블렌드가 장애물을 관통한다. DoSpline/trailing 몫.
    if (alongside &&
        std::abs(cur_d_) < RequiredObstacleClearance(closest_obs)) {
        // "어떤 속도로도 불가"에 해당한다 — 이미 장애물 옆에 들어와 있어서
        // 감속으로 해결되지 않는다. 플래그를 세우지 않으면 Tier3가 이걸
        // "일시적 실패"로 보고 정지 대신 trailing_retry_speed(1.5 m/s)로
        // 전진시킨다.
        lane_switch_geom_infeasible_ = true;
        return false;
    }
    double dist_front, dist_back;
    if (alongside) {
        dist_front = 0.0;
        dist_back = Mod(closest_obs.s_end - cur_s_, track_len_);
        // alongside 판정은 s_start 기준(behind < span + 1.0)이라 **뒤끝을 이미
        // 지난** 0.5~1.0m 창까지 참이 된다. 그 창에서 dist_back 은 wrap 되어
        // 거의 한 바퀴가 되고, hold_end = dist_back + 0.5 로 계획 스팬이
        // 트랙 길이를 넘어 매번 lane_span_over_lap 으로 죽었다 — 사유가
        // 엉뚱할 뿐 아니라 프로파일을 다 만든 뒤에 버리는 낭비였다.
        // 지난 장애물은 Tier1 이 할 일이 없다. 여기서 바로 접는다.
        // (기하 불성립 플래그는 세우지 않는다 — 정지 사유가 아니다.)
        if (dist_back > span + vehicle_half_length_m_) {
            NoteReject("lane_obs_passed");
            return false;
        }
    } else {
        dist_front = Mod(closest_obs.s_start - cur_s_, track_len_);
        dist_back = dist_front + span;
        if (dist_front > lookahead_m_ + 5.0) {
            return false;   // 너무 멀다
        }
    }

    // 연쇄 장애물이 있으면 hold 를 그 끝까지 늘린다 (호출자가 hold_through_s
    // 로 지정). 늘리지 않으면 복귀 블렌드가 뒤 장애물을 관통해 경로 전체가
    // 거부되고, 다음 사이클에 그 장애물이 closest 가 되며 사이드가 뒤집힌다.
    // 상한은 lookahead — 콘이 줄지어 있어도 hold 가 무한정 늘지 않게.
    if (std::isfinite(hold_through_s)) {
        const double through = Mod(hold_through_s - cur_s_, track_len_);
        if (through <= lookahead_m_) {
            dist_back = std::max(dist_back, through);
        }
    }

    // hold 구간: 장애물 앞뒤 0.5m 여유 (pure pursuit 대각선 커팅 방지, DoSpline 동일).
    double hold_start = std::max(dist_front - 0.5, 0.0);
    double hold_end = dist_back + 0.5;

    // 진입 예산은 hold 여유(0.5m)가 아니라 "충돌 박스에 닿기까지"로 잡는다.
    // hold 0.5m는 pure pursuit 대각선 커팅을 막기 위한 **경로 모양** 제약이지
    // 횡전환을 끝내야 하는 지점이 아니다. 실제 안전 경계는 CheckPathCollision의
    // 종방향 박스(obs.s_start - (vehicle_half_length + sigma_s))이고,
    // 진입 블렌드는 blend_start 로직상 이미 hold 구간까지 뻗을 수 있다.
    //   0807 run_164216 t=5.54: 필요 1.795 / 가용 1.67 로 0.125m 부족 판정.
    //   이 기준으로는 가용 = 2.07 - 0.275 + 0.1 = 1.895 로 성립한다.
    //   (그 런은 아래 kappa 항 때문에 여전히 불가로 남는다 — 그게 옳다.)
    // ── 접근속도 보정 (2026-08-20) ───────────────────────────────────────
    // dist_front 는 **지금** 상대까지의 거리다. 상대가 정지해 있으면 그것이
    // 곧 내가 진입 블렌드에 쓸 수 있는 경로 길이지만, 상대가 같은 방향으로
    // 달아나고 있으면 아니다. 간격이 닫히는 데 걸리는 시간은
    //   t = dist_front / (v_ego - v_obs)
    // 이고 그동안 내가 실제로 달리는 거리는 v_ego * t 다. 즉 가용 활주로는
    //   entry_avail * v_ego / (v_ego - v_obs)
    // 로, 접근속도가 느릴수록 크게 늘어난다. 종전 코드는 이 배율을 빼고
    // "상대 정지" 기준으로만 판정해서, 실제보다 30~330% 작은 활주로로
    // 회피 가부를 결정하고 있었다 (run_0819: v_ego 4.67 / v_obs 1.6 ->
    // 배율 1.52, 헤어핀 B 는 더 크다).
    //
    // 안전 방향: 배율은 항상 >= 1 이므로 이 보정은 "가능"을 늘리기만 한다.
    // 그래서 게이트를 세 겹 건다 —
    //   (1) 정적은 제외 (배율 = 1, 종전과 비트 단위로 동일).
    // 2026-08-22 (3/8단계): 접근속도 보정 항을 제거했다. 이 티어는 이제
    // 정적 전용이라(try_evade = closest.is_static) 상대가 다가오면서 활주로가
    // 줄어드는 경우가 존재하지 않는다. 동적 추월의 활주로는 7단계의 직선
    // 추월이 자기 기준으로 따로 계산한다.
    const double entry_avail =
        std::max(dist_front - vehicle_half_length_m_, 0.0);
    // 진입 블렌드의 예산은 **한 곳에서만** 정의한다. 아래 두 소비처가
    // 어긋나 있으면 안 된다:
    //   (1) BlendSpeedCap 의 available — "이 거리 안에서 되는 속도"를 구한다.
    //   (2) blend_in_cap — 실제로 블렌드에 허용하는 길이 상한.
    // 예전에는 (1)만 + wpnt_dist_ 를 달고 (2)는 hold_start + hold_overrun_max
    // (= entry_avail) 라서 정확히 wpnt_dist_ 만큼 (1) > (2) 였다. 캡이 물리는
    // 순간 need_in = cap*sqrt(6A/a_lat) = (1) 이므로 need_in 은 (2)를 **정의상**
    // 초과한다 → lane_blend_truncated 가 매 사이클 확정 발동하고, blend_in 이
    // 설계 의도인 need_in*1.2 대신 need_in 아래로 눌려 기동 피크 횡가속이
    // a_lat/1.44 대신 a_lat 를 그대로 먹는다(여유 20% -> 0%).
    const double entry_budget = entry_avail + wpnt_dist_;

    // 진입/복귀 거리는 상수가 아니라 그립 예산으로 정한다. 상수(1.5m 하한 /
    // 2.5m 복귀)를 쓰면 횡가속이 v^2로 커져 6 m/s에서 1.5g를 요구한다.
    //  - 진입 진폭은 d_hold 전체가 아니라 "현재 d에서 남은 양" — 매 사이클
    //    재생성되므로 이미 옮겨간 만큼은 다시 옮길 필요가 없다.
    //  - 복귀 진폭은 d_hold 전체 (raceline까지 되돌아와야 함).
    // DoSpline과 같은 이유로 실측이 아닌 PlanSpeed()를 쓴다 — 정지/저속에서
    // BlendDist가 0이 되면 진입 블렌드가 0m로 접혀 장애물 안에서 횡전환이
    // 끝나거나 조향 한계를 넘는 기하가 나온다.
    // 이 회피 구간에서 횡 전환에 쓸 수 있는 그립 (마찰원 잔여).
    const double a_lat = BlendLatAcc(
        Mod(cur_s_, track_len_), Mod(cur_s_ + hold_end, track_len_));
    // ── 곡률 기준 통일 (2026-08-10) ────────────────────────────────────
    // 조향 기하 하한(BlendGeomFloor)에 더할 트랙 곡률항은 **그 지점에서 차가
    // 실제로 있을 d 의 유효 곡률**이어야 한다. 구간마다 d 가 다르므로 나눠 본다:
    //   [cur_s, hold_start) 진입 블렌드 — d 가 cur_d_ 에서 d_hold 로 램프 중.
    //       기동 곡률(6A/L²)은 BlendGeomFloor 가 이미 더하므로 여기서는
    //       레이스라인 원본 곡률을 쓴다.
    //   [hold_start, hold_end] hold — d = d_hold 로 고정. 여기서는 오프셋
    //       경로의 유효 곡률 OffsetKappa(k, d_hold) 가 실제 조향량이다.
    //       "그 오프셋을 그 구간에서 유지할 수 있는가"라는 별개의 가부 조건.
    //
    // ⚠️ 처음엔 전 구간에 d_hold 를 먹였는데 그건 틀렸다. 진입 구간은 아직
    //    라인 위인데 hold 오프셋을 적용하는 셈이라, my_track0809 의 s=7.5
    //    헤어핀(k=+0.790)에서 좌 0.5m 오프셋이 OffsetKappa 1.307 로 계산돼
    //    조향 한계(1.141)를 넘고, 검출 6~8m 에서 성립하던 회피가 전부
    //    "어떤 속도로도 불가"로 뒤집혔다 (오프라인 재현으로 확인).
    const double kappa_in = std::max(
        BlendKappaMax(Mod(cur_s_, track_len_),
                      Mod(cur_s_ + hold_start, track_len_)),
        BlendKappaMax(Mod(cur_s_ + hold_start, track_len_),
                      Mod(cur_s_ + hold_end, track_len_), d_hold));
    // 복귀 블렌드도 d 가 d_hold -> 0 으로 램프하는 구간이라 원본 곡률.
    const double kappa_out = BlendKappaMax(
        Mod(cur_s_ + hold_end, track_len_),
        Mod(cur_s_ + hold_end + lane_blend_out_m_, track_len_));

    // 회피에 **실제로 필요한 최소 오프셋**. 목표 레인 d_hold까지 다 옮기는
    // 것과 "충돌을 벗어나는 것"은 다르다 — 현재 d가 이미 장애물 밖이면
    // 필요 횡이동은 0이다. 예전에는 d_hold - cur_d_ 를 그대로 진폭으로 써서
    // 추종 잔차 13cm를 못 지운다는 이유로 성립하는 회피를 통째로 버렸다
    // (0807 run_164819 t=4.815: 필요 0.13m / 하한 0.92m / 가용 0.78m -> 거부.
    //  실제로는 자차가 이미 d=+0.13 이라 횡이동 0으로 8cm 여유 통과 가능).
    const double clear_margin = RequiredObstacleClearance(closest_obs);
    const bool pass_left = (d_hold >= closest_obs.d_left);
    const double d_hold_min = pass_left
        ? closest_obs.d_left + clear_margin
        : closest_obs.d_right - clear_margin;
    // d_hold_min이 cur_d_ 쪽으로 더 가까울 때만 의미가 있다 (이상 레인을
    // 넘어서까지 줄이지는 않는다).
    const bool relax_possible =
        pass_left ? (d_hold_min < d_hold) : (d_hold_min > d_hold);

    // 진입 속도 상한. 무한대(=횡이동 0)면 제약 없음.
    double entry_speed_cap = std::numeric_limits<double>::infinity();
    double plan_v = PlanSpeed();
    // 크리프 탈출이 발동했나 — 이때만 진입 블렌드가 hold 구간을 침범해도 된다.
    bool creep_relaxed = false;
    // 진입 블렌드는 지금 위치에서 시작한다 — 지점별 기하 검사의 기준 s.
    const double blend_s0 = Mod(cur_s_, track_len_);
    if (!alongside) {
        // ── 이진 거부가 아니라 "되는 속도"를 구한다 ─────────────────────
        // 예전에는 need_in(현재 속도 기준)이 가용 거리를 넘으면 그대로
        // return false → Tier3 트레일링(v_target=0) → 정지였다. 그런데
        // 정적 장애물에서는 속도가 자유 변수다: 남은 거리 안에서 성립하는
        // 속도를 계산해 프로파일에 실으면 회피가 그대로 성립한다.
        //   rosbag_0806 주행중 검출 32건 중 22건(69%)은 감속조차 필요
        //   없었고(검출 5.5m에서 5.5~5.7 m/s까지 허용), 나머지도 2.4~5.7
        //   m/s면 성립했다. 정지가 필요했던 건 0건이다.
        // 감속으로도 못 살리는 경우는 조향 기하 하한(속도 무관)을 못 채울
        // 때뿐이고, 그때만 false를 돌려 상위 정지 로직으로 넘긴다.
        double available = entry_budget;
        double cap = BlendSpeedCap(d_hold - cur_d_, available, a_lat, kappa_in,
                                   blend_s0);
        if (!(cap > 0.0) && relax_possible) {
            // 이상 레인까지는 못 가더라도 "충돌만 벗어나는" 최소 오프셋으로는
            // 성립할 수 있다. 성립하면 목표를 그쪽으로 낮춰 회피를 살린다.
            const double relaxed_cap =
                BlendSpeedCap(d_hold_min - cur_d_, available, a_lat, kappa_in,
                              blend_s0);
            if (relaxed_cap > 0.0) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                    "LaneSwitch 최소오프셋 완화: d_hold %.2f -> %.2f "
                    "(횡 %.2f -> %.2fm, 가용 %.2fm)",
                    d_hold, d_hold_min, d_hold - cur_d_, d_hold_min - cur_d_,
                    available);
                d_hold = d_hold_min;
                cap = relaxed_cap;
            }
        }
        if (!(cap > 0.0) && std::abs(cur_vs_) <= creep_escape_speed_) {
            // ── 크리프 탈출 ────────────────────────────────────────────────
            // 정지/크리프 상태에서 장애물이 조향 기하 하한 안쪽에 있으면
            // "장애물 앞에서 횡전환을 끝낸다"는 전제로는 해가 영원히 없다:
            //   run_0806_234247 55.62s — "0.6m lateral transition, 0.4m
            //   available (v=0.0, dd=-0.06)" — 6cm 옮기는 데 0.6m가 필요한데
            //   0.4m뿐이라는 판정이 30Hz로 무한 반복됐다.
            // 실제 차는 그렇게 못 하는 게 아니다. 조향을 넣고 천천히 나가면
            // 횡이동이 장애물 옆을 지나는 동안 누적된다. 그러니 hold 구간까지
            // 전환에 쓸 수 있게 허용하고, 실제 안전(차체가 장애물을 스치는가)은
            // CheckPathCollision이 판정하게 한다 — 통과 못 하면 거기서 거부되고
            // 상위 정지 로직으로 정상적으로 넘어간다.
            // 크리프 속도 이하에서만 허용한다. 주행 중에는 이 완화가
            // "장애물 옆에서 횡전환이 끝나는" 위험한 기하를 만든다.
            const double creep_available = hold_end;
            const double creep_cap =
                BlendSpeedCap(d_hold - cur_d_, creep_available, a_lat, kappa_in,
                              blend_s0);
            if (creep_cap > 0.0) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                    "LaneSwitch 크리프 탈출: 가용 %.2f -> %.2fm (hold 구간 사용), "
                    "횡 %.2fm, v=%.2f", available, creep_available,
                    d_hold - cur_d_, cur_vs_);
                available = creep_available;
                cap = creep_cap;
                creep_relaxed = true;
            }
        }
        if (!(cap > 0.0)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "LaneSwitch infeasible: %.2fm 횡이동에 기하 하한 %.2fm 필요, "
                "가용 %.2fm (v=%.1f, kappa=%.2f) — 어떤 속도로도 불가",
                d_hold - cur_d_,
                BlendGeomFloor(d_hold - cur_d_, kappa_in, false, blend_s0),
                available, cur_vs_, kappa_in);
            NoteReject("lane_geom_floor");
            lane_switch_geom_infeasible_ = true;
            return false;
        }
        // 캡의 비교 대상은 "지금 속도"가 아니라 "진입 블렌드를 지나는 동안
        // 낼 속도"다. 예전에는 plan_v(현재 속도)와만 비교해서, **가속 구간
        // 에서는 캡이 아예 걸리지 않았다**:
        //   run_0815_191056 t=109.5 (직선 s=32.3, 장애물 s=36.4, 횡 0.72m)
        //     가용 3.8m -> cap 5.43 m/s, 그런데 현재 속도는 3.53 m/s 였다.
        //     cap < plan_v 가 거짓이라 캡 미적용 -> 차는 블렌드 중 5.9 m/s
        //     까지 가속했고, 피크 횡가속이 예산 8.83 의 2배가 됐다.
        //     조향 -14~-18도, 계획 대비 횡오차 +0.22m 로 벌어진 채 복귀에
        //     들어가 오차 +0.57m 로 커지며 좌벽 접촉(s≈1.5).
        // cap 자체는 그대로다 — 바뀐 것은 "언제 거는가" 뿐이다.
        const double carried_in = BlendPlanSpeed(blend_s0, available);
        if (cap < std::max(plan_v, carried_in)) {
            entry_speed_cap = std::max(cap, plan_speed_floor_);
            plan_v = std::min(plan_v, entry_speed_cap);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "LaneSwitch 속도 캡 %.2f m/s (횡 %.2fm, 가용 %.2fm, a_lat %.1f, "
                "현재 %.2f, 블렌드중 %.2f)",
                entry_speed_cap, d_hold - cur_d_, available, a_lat,
                PlanSpeed(), carried_in);
        }
    }
    // 진입/복귀 거리는 상수가 아니라 그립 예산으로 정한다.
    //  - 진입 진폭은 d_hold 전체가 아니라 "현재 d에서 남은 양" — 매 사이클
    //    재생성되므로 이미 옮겨간 만큼은 다시 옮길 필요가 없다.
    //  - 복귀 진폭은 d_hold 전체 (raceline까지 되돌아와야 함).
    //
    // 속도는 "지금 속도"가 아니라 "블렌드를 지나는 동안 낼 속도"다
    // (BlendPlanSpeed 주석 — 가속 중 직선에서 피크 횡가속이 예산의 1.5배가
    //  되던 것). 길이<->속도가 서로를 참조하므로 한 번 반복해 수렴시킨다.
    // entry_speed_cap 이 걸려 있으면 그 위로는 올라갈 수 없다 — 그 캡은
    // ApplySpeedProfile 이 진입 구간에 실제로 강제하는 값이다.
    // alongside(이미 장애물 옆)는 제외한다 — 그 경로의 blend_in 은 남은
    // 거리가 없어 "잔차 보정에 필요한 만큼만" 쓰는 것이 설계이고, 여기서
    // 길이를 키우면 횡전환이 장애물 옆에서 끝나는 기하가 된다.
    double entry_v = plan_v;
    for (int it = 0; it < 2 && !alongside; ++it) {
        const double guess =
            BlendDist(d_hold - cur_d_, entry_v, a_lat, kappa_in, blend_s0);
        entry_v = std::min(BlendPlanSpeed(blend_s0, guess), entry_speed_cap);
        entry_v = std::max(entry_v, plan_v);
    }
    const double need_in =
        BlendDist(d_hold - cur_d_, entry_v, a_lat, kappa_in, blend_s0);
    // ── 복귀(return) 쪽 그립 예산 통일 ─────────────────────────────────
    // 예전에는 복귀 **거리**를 a_lat(마찰원 잔여)로 잡고 복귀 **속도 캡**만
    // 고정 blend_lat_acc_max_ 로 잡아, 같은 기동의 거리와 속도가 다른 예산에서
    // 나왔다 (코너에서 실제보다 1.6배 빠른 통과를 허용). 바로 위 need_out
    // 주석이 "a_lat 은 blend_out 초기값과 반드시 같은 것을 써야 한다"고
    // 선언했는데 return_speed_cap 만 안 고쳐져 있었다.
    // 두 값을 하나로 묶되, **큰 쪽**을 쓴다 — 복귀는 장애물을 이미 지난 뒤의
    // 기동이라 보수적으로 갈 이유가 적고, 실제 물리 검증은 그 뒤의
    // ApplySpeedProfile(오프셋 곡률 캡 + 마찰원 감속)이 다시 한다.
    const double a_lat_out = std::max(a_lat, blend_lat_acc_max_);
    // 복귀 속도도 같은 이유로 "그 구간에서 낼 속도" — 복귀는 장애물을 지난
    // 뒤라 대개 가속 구간이고, 사고(run_0815_191056 t=110.4)가 난 곳이
    // 정확히 여기다.
    const double ret_s0 = Mod(cur_s_ + hold_end, track_len_);
    double out_v = PlanSpeed();
    for (int it = 0; it < 2; ++it) {
        const double guess = std::max(
            lane_blend_out_m_,
            BlendDist(d_hold, out_v, a_lat_out, kappa_out));
        out_v = BlendPlanSpeed(ret_s0, guess);
    }
    // 아래에서 벽에 맞춰 줄일 수 있다 (복귀 블렌드 단축)
    double blend_out = std::max(
        lane_blend_out_m_,
        BlendDist(d_hold, out_v, a_lat_out, kappa_out));
    double blend_in;
    if (alongside) {
        // 이미 장애물 옆 — 남은 거리가 없으므로 잔차 보정에 필요한 만큼만.
        blend_in = std::max(1.0, need_in);
    } else {
        // 필요한 만큼만 쓰고(여유 20%) 시작을 늦춘다 — 남은 거리 전부를
        // 진입에 쓰면 상류의 좁은 구간을 오프셋 상태로 통과하게 된다.
        //
        // ── hold 침범 상한 ────────────────────────────────────────────
        // 속도 캡의 예산은 entry_avail(= dist_front - 차 반길이)이고
        // hold_start 는 dist_front - 0.5 이므로, 그 차이(0.5 - 0.275 =
        // 0.225 m)까지는 진입이 hold 구간으로 넘어가는 것이 원래 설계다.
        // 그런데 예전 clamp 는 상한을 max(..., need_in) 으로 둬서 need_in 이
        // 크면 **상한이 사라졌다**. 그 경로로 들어가면 blend_start 가 0이 되고
        // 횡전환이 장애물 옆에서 끝난다 — 크리프 탈출(creep_escape_speed,
        // 0.6 m/s 이하)이 위험하다고 막아 놓은 바로 그 기하가 주행 중에
        // 속도와 무관하게 만들어진다. entry_speed_cap 이 물리 캡을
        // plan_speed_floor(1.5)로 되올리는 순간 need_in 이 가용을 넘기 때문에
        // 드문 경로도 아니다.
        // 이제 상한을 명시적으로 두고, 크리프 탈출일 때만 예외로 푼다.
        // 2026-08-13: 상한을 entry_budget 으로 통일한다. hold_start +
        // hold_overrun_max 는 dist_front > 0.5 에서 정확히 entry_avail 과 같아
        // 속도 캡 예산보다 wpnt_dist_ 만큼 작았다 (위 entry_budget 주석 참조).
        const double blend_in_cap = creep_relaxed
            ? std::max(hold_end, wpnt_dist_)
            : std::min(entry_budget, blend_dist_max_m_);
        blend_in = std::clamp(need_in * 1.2,
                              std::min(need_in, blend_in_cap),
                              std::max(blend_in_cap, wpnt_dist_));
        if (need_in > blend_in_cap + 1e-6) {
            // 필요한 전환거리를 다 못 준 채로 진행한다 — 기하 곡률이 커지므로
            // ApplySpeedProfile 이 거부할 수 있다. 사유를 남겨 둔다.
            NoteReject("lane_blend_truncated");
        }
        // ── 조기 라인 스위치 ────────────────────────────────────────────
        // 위 clamp 는 "최소 전환거리 x 1.2" 이고 아래 blend_start 가
        // hold_start 에서 딱 그만큼만 물러나므로, 회피는 항상 **최소
        // 거리에서 최대 횡가속**으로 실행된다. 늦게 시작하는 근거는
        // run_0806_155621(긴 진입이 상류의 좁은 구간을 오프셋 상태로 통과해
        // 벽에 걸림)인데, 그건 "상류가 좁을 때"만 성립한다.
        //
        // 상류가 넓고 곧으면 늦출 이유가 없다. 0808 run_185049 실측:
        // 장애물 s=31.7 앞 직선(kappa~0, 좌벽 1.07m)이 4.7m 비어 있는데
        // 횡이동은 마지막 1.0~2.5m 에서 일어났고, 6랩 중 4랩에서 조향이
        // 하드웨어 한계(20.6도)에 포화하고 rate 까지 clip 됐다.
        // d=0.55 / v=4.5 기준 피크 횡가속 6A/L²: 2.5m -> 5.3 m/s²,
        // 4.5m -> 1.6 m/s². 같은 회피가 조향 20도에서 7도가 된다.
        //
        // 안전 조건 두 가지를 모두 만족할 때만 연장한다:
        //  (1) 진입 구간 **내내**(MinWallInSpan) 목표 오프셋이 벽 안 —
        //      즉 오프셋 상태로 통과해도 되는 구간이다. 이게 위 옛 사례가
        //      요구하던 바로 그 조건이다.
        //  (2) 그 구간에서 **오프셋 경로가 실제로 그리는 곡률**이
        //      early_switch_kappa_max_ 이하.
        //      2026-08-10 수정: 예전엔 레이스라인 원본 곡률을 봤다. 그러면
        //      "코너 근처"라는 이유만으로 방향을 안 보고 전부 막혀서, 코너
        //      **바깥쪽**으로 비키는(= 반경이 커져 원래보다 완만한) 회피까지
        //      차단됐다. my_track0809 s=13.5 정적 장애물 실측: 좌(바깥) 회피는
        //      통로 0.75 m / 간격 0.35 m 로 여유가 있는데도 진입 구간 원본
        //      max|k| 0.500(헤어핀 정점) 때문에 조기 전환이 항상 거부됐고,
        //      결국 최소거리 급조향 -> geom_floor 거부 -> TRAILING 으로 떨어졌다.
        //      오프셋 곡률로 보면 좌 0.227 / 우 0.283 이라 방향이 구분된다 —
        //      원 주석이 막으려던 "안쪽 오프셋이 그립을 흔드는" 경우는 그대로 걸린다.
        // 속도 캡(entry_speed_cap)은 이미 available(= 장애물까지 전체 거리)
        // 기준으로 계산됐으므로 여기서 더 느려지지 않는다.
        // ── 2026-08-12: 전부-아니면-전무 -> 통과하는 최대 스팬 탐색 ──────
        // 예전에는 **가장 긴 스팬 하나만** 시험하고 실패하면 조기 전환을 통째로
        // 포기했다. BlendKappaMax 가 구간 **최댓값**이라 스팬 끝에 코너가 하나만
        // 걸려도 전부 취소된다 — run_0812_013630 실측 ok_early_switch 2회 / 254초.
        //
        // 이게 "일찍 시작하면 코너에서 오히려 회피를 포기한다"는 관찰의 정체다.
        // 스팬을 늘리면 kappa_track 최댓값이 올라가고, 기하 하한
        // 6A/L² + |kappa_track| <= 0.92*SteerKappaMax(=1.049) 에서 기동에 남는
        // 예산이 줄어 **요구 거리가 오히려 길어진다.** 실측 환산:
        //   kappa 0.01 -> 남는 예산 1.039(99%), 0.5m 이동에 1.70 m 필요
        //   kappa 0.30 -> 0.749(71%), 2.00 m
        //   kappa 0.52 -> 0.529(50%), 2.38 m
        //   kappa 0.75 -> 0.299(29%), 3.17 m
        // 즉 1.5 m 뒤로 늘려 활주로를 벌어도 kappa 0.75 구간을 끌어들이면
        // 요구가 1.47 m 늘어 순이득이 0이 된다. 늘리는 것 자체가 아니라
        // **코너를 스팬에 끌어들이는 것**이 문제였다.
        //
        // 이제 긴 쪽에서 짧은 쪽으로 내려가며 두 게이트를 통과하는 첫 스팬을
        // 쓴다. 코너가 앞에 있으면 그 앞까지만 늘어나고, 직선이면 전부 늘어난다.
        // 게이트 자체는 그대로 — 통과 기준을 약화시키지 않고 후보만 늘린다.
        //
        // TODO(후속): 벽 게이트 `room >= |d_hold|` 는 구조적으로 과보수다.
        // 진입 구간은 d 가 cur_d_ -> d_hold 로 **램프**하므로 상류에서는 |d| 가
        // 훨씬 작은데, min(벽) 과 max(|d|) 를 서로 다른 위치에서 뽑아 비교한다.
        // 점별(|d(s)| <= wall(s) - boundary) 검사로 바꾸면 이 트랙(폭 중앙 1.2m)
        // 에서 조기 전환이 더 열린다. 이번 수정 범위에서는 건드리지 않았다.
        if (early_switch_kappa_max_ > 0.0) {
            const double early_span_max =
                std::max(std::min(hold_start, blend_dist_max_m_), 0.0);
            // 스팬 후보 간격. wpnt_dist_ 가 촘촘하면 반복이 과해지므로 하한 0.25m.
            const double span_step = std::max(wpnt_dist_, 0.25);
            for (double early_span = early_span_max;
                 early_span > blend_in + 1e-6; early_span -= span_step) {
                const double s_from = Mod(cur_s_, track_len_);
                const double s_to = Mod(cur_s_ + early_span, track_len_);
                double wall_l, wall_r;
                MinWallInSpan(s_from, s_to, wall_l, wall_r);
                const double boundary =
                    RequiredBoundaryMargin(spline_bound_mindist_);
                const double room =
                    ((d_hold >= 0.0) ? wall_l : wall_r) - boundary;
                // 이 구간은 d 가 cur_d_ -> d_hold 로 램프하는 **진입** 구간이라
                // 아직 오프셋이 성립하지 않았다. 08-10 에 d_hold 를 먹여 봤으나
                // kappa_in 과 같은 오류였고(진입 구간에 hold 오프셋 적용),
                // 게다가 실측상 코너에서는 원본이든 오프셋이든 임계값 0.10 을
                // 크게 넘어(0.500 vs 0.665) 판정이 바뀌지 않는다 — 이 게이트가
                // 실제로 갈리는 곳은 양쪽 다 ~0 인 직선뿐이다. 원본으로 되돌린다.
                const double kappa_early = BlendKappaMax(s_from, s_to);
                if (room >= std::abs(d_hold) &&
                    kappa_early <= early_switch_kappa_max_) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                        "LaneSwitch 조기 전환: 진입 %.2f -> %.2fm / 최대 %.2fm "
                        "(구간 벽여유 %.2f >= |d_hold| %.2f, kappa %.3f)",
                        blend_in, early_span, early_span_max, room,
                        std::abs(d_hold), kappa_early);
                    NoteOutcome(early_span >= early_span_max - 1e-6
                                   ? "ok_early_switch"
                                   : "ok_early_switch_partial");
                    blend_in = early_span;
                    break;
                }
            }
        }
    }
    // 진입 블렌드가 hold 구간을 넘어가면 d 프로파일이 끊긴다.
    blend_in = std::min(blend_in, std::max(hold_end, wpnt_dist_));
    double total = hold_end + blend_out;
    if (!PlanSpanAllowed(total)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "LaneSwitch 거부: 계획 스팬 %.1fm (한 바퀴 초과)", total);
        NoteReject("lane_span_over_lap");
        return false;
    }

    int n = static_cast<int>(total / wpnt_dist_);
    if (n < 8) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "LaneSwitch 거부: 경로 %d점 (최소 8)", n);
        NoteReject("lane_too_few_points");
        return false;
    }

    // 2026-08-22 (4/8단계): 이 티어는 정적 전용이라 추월 페이스 하한이
    // 존재하지 않는다 (raceline 프로파일 그대로).
    const double v_floor = 0.0;

    // 진입 블렌드는 "가능한 한 늦게" 시작한다 (blend_start 이후). 예전에는
    // 남은 거리 전체(hold_start)를 진입에 썼는데, 그러면 장애물과 무관한
    // 상류 구간까지 오프셋 상태로 지나 그곳 벽에 걸린다 — 폭이 1.7~2.0m인
    // 이 트랙에서는 그것만으로 좌우 레인이 모두 막혔다 (run_0806_155621).
    // 늦게 시작하면 레이스라인을 더 오래 타므로 벽 여유와 속도 양쪽에 유리하다.
    const double blend_start = std::max(hold_start - blend_in, 0.0);
    // d 프로파일 생성기 (레이스라인 유지 → 진입 블렌드 → hold → 복귀 블렌드).
    auto build_profile = [&](double b_out) {
        std::vector<double> d_prof(n);
        for (int i = 0; i < n; ++i) {
            const double t = i * wpnt_dist_;
            if (t < blend_start) {
                d_prof[i] = cur_d_;            // 아직 라인 유지
            } else if (t < blend_start + blend_in) {
                double w = (blend_in > 1e-6) ? (t - blend_start) / blend_in : 1.0;
                w = w * w * (3.0 - 2.0 * w);   // smoothstep
                d_prof[i] = cur_d_ * (1.0 - w) + d_hold * w;
            } else if (t < hold_end) {
                d_prof[i] = d_hold;            // 오프셋 레인 유지
            } else {
                double w = std::clamp((t - hold_end) / std::max(b_out, 1e-6),
                                      0.0, 1.0);
                w = w * w * (3.0 - 2.0 * w);
                d_prof[i] = d_hold * (1.0 - w);  // raceline 복귀
            }
        }
        return d_prof;
    };
    // 프로파일이 트랙 경계 안에 들어가는지 (들어가면 1.0, 아니면 필요한 축소율)
    auto profile_fit = [&](const std::vector<double> &d_prof, bool *width_bad) {
        double worst = 1.0;
        const double margin = RequiredBoundaryMargin(spline_bound_mindist_);
        for (int i = 0; i < n; ++i) {
            const double s = Mod(cur_s_ + i * wpnt_dist_, track_len_);
            const int gi = static_cast<int>(s / wpnt_dist_) % gb_size;
            const double d_lo = -std::abs(gb_wpnts_[gi].d_right) + margin;
            const double d_hi = std::abs(gb_wpnts_[gi].d_left) - margin;
            if (d_lo > d_hi) {
                if (width_bad) {
                    *width_bad = true;
                }
                return 0.0;
            }
            const double lim = (d_prof[i] > 0.0) ? d_hi : d_lo;
            if (std::abs(d_prof[i]) > std::abs(lim) &&
                std::abs(d_prof[i]) > 1e-6) {
                worst = std::min(worst, std::abs(lim) / std::abs(d_prof[i]));
            }
        }
        return worst;
    };

    // ApexFor가 hold 구간 최소 벽폭을 반영하지만, 복귀 블렌드는 그 구간
    // 밖까지 뻗는다 — 장애물 직후가 좁아지는 코너면 복귀 경로가 벽에 걸린다
    // (run_0806_155621: s=3.2 좌벽 수축에서 매 사이클 실패).
    // 이때 프로파일 전체를 축소하면 정작 필요한 hold 오프셋까지 사라지므로,
    // "복귀를 더 일찍 끝내는" 쪽을 먼저 시도한다. 하한은 그립 예산(need_out)
    // — 그보다 짧은 복귀는 횡가속을 넘긴다. 그래도 안 되면 전체 축소로
    // 마지막 시도를 하고, 축소 후 장애물 간격이 없으면 이 레인을 포기한다.
    // 복귀 하한은 "현재 속도의 그립 예산"(need_out)이 아니라 속도와 무관한
    // 조향 기하 하한(geom_out)이다 — 그보다 길게 필요한 부분은 아래
    // return_speed_cap 과 ApplySpeedProfile 이 감속으로 만족시킨다.
    // 복귀를 짧게 못 하면 좁아지는 코너에 걸려 회피 자체가 사라진다.
    // 복귀 예산은 blend_out 초기값과 반드시 같은 것을 써야 한다 — 이제
    // a_lat_out 하나로 통일했다 (위 주석 참조).
    // 속도도 blend_out 초기값과 같은 것을 쓴다 (out_v — BlendPlanSpeed).
    const double need_out = BlendDist(d_hold, out_v, a_lat_out, kappa_out);
    // 조향 기하 하한만 남음 (속도 0). 트랙 곡률이 여기에도 더해진다.
    const double geom_out = BlendDist(d_hold, 0.0, -1.0, kappa_out);
    bool width_bad = false;
    std::vector<double> lane_d = build_profile(blend_out);
    double fit = profile_fit(lane_d, &width_bad);
    bool return_shortened = false;
    if (!width_bad && fit < 1.0 - 1e-6) {
        for (double shrink : {0.75, 0.5, 0.35, 0.25}) {
            const double candidate = std::max(blend_out * shrink, geom_out);
            if (candidate >= blend_out - 1e-6) {
                continue;
            }
            auto trial = build_profile(candidate);
            bool bad = false;
            const double trial_fit = profile_fit(trial, &bad);
            if (!bad && trial_fit > 1.0 - 1e-6) {
                lane_d = std::move(trial);
                return_shortened = candidate < need_out - 1e-6;
                blend_out = candidate;
                fit = 1.0;
                break;
            }
        }
    }
    if (width_bad) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "LaneSwitch 거부: 통과폭이 차량 폭보다 좁은 구간 존재");
        NoteReject("lane_width_negative");
        return false;
    }
    // 벽 때문에 오프셋을 줄여야 하는 경우의 **기하** 축소. (여기에 걸려 있던
    // 배율 감속은 제거됐다 — 속도 제한은 ApplySpeedProfile 의 물리 캡 몫.)
    //
    // 2026-08-11: 전역 배율(d *= fit) -> **지점별 클램프**.
    // profile_fit 은 스팬 전체에서 가장 나쁜 비율 하나(worst)를 뽑는데, 그것을
    // 프로파일 전체에 곱하면 정작 장애물 옆의 apex 까지 같이 줄어든다.
    // run_0810_233521 에서 15회 전부 이 경로였다 — 콘 옆은 0.99 m 비어 있는데
    // 4 m 뒤 코너가 좁다는 이유로 오프셋이 0.44 -> 0.21 / 0.46 -> 0.19 로
    // 줄어 "장애물 간격 부족"으로 회피가 포기됐다.
    // 좁은 것은 그 지점이지 스팬 전체가 아니므로, 각 점을 그 점의 벽 안으로만
    // 넣는다. 클램프는 접합부에 기울기 꺾임을 만들므로 3점 평균 2회로 눌러
    // 주고, 평활 후 다시 한 번 클램프해 경계는 반드시 지킨다.
    // (꺾임이 남아 곡률 한계를 넘으면 ApplySpeedProfile 이 거부한다 — 예전과
    //  같은 결과이지 더 나쁘지 않다. 대신 apex 가 살아남는 경우가 생긴다.)
    const bool lane_wall_clamped = fit < 1.0 - 1e-6;
    if (lane_wall_clamped) {
        const double margin = RequiredBoundaryMargin(spline_bound_mindist_);
        // i=0 도 클램프한다. "그 점은 차가 실제로 있는 자리(cur_d_)이니
        // 그대로 두어야 한다"는 수정을 검토했다가 되돌렸다 — 차가 경계 밖에
        // 있을 때 lane_d[0](밖) 과 lane_d[1](클램프됨) 사이에 0.1m 종방향에
        // 위반량만큼의 횡 점프가 생기고, 아래 psi_rad 재계산이
        // atan2(Δy, Δx) 로 그 점프를 그대로 헤딩에 싣는다. 게다가
        // ApplySpeedProfile 의 기하곡률 추정은 머리 3점을 안쪽 값으로 덮으므로
        // 그 꺾임을 보지도 못한다. 경로 전체를 회랑 안에 두는 편이 하류의
        // 모든 검사(경계·충돌·곡률)가 전제하는 형태다.
        auto clamp_to_walls = [&]() {
            for (int i = 0; i < n; ++i) {
                const double s = Mod(cur_s_ + i * wpnt_dist_, track_len_);
                const int gi = static_cast<int>(s / wpnt_dist_) % gb_size;
                const double d_lo = -std::abs(gb_wpnts_[gi].d_right) + margin;
                const double d_hi = std::abs(gb_wpnts_[gi].d_left) - margin;
                lane_d[i] = std::clamp(lane_d[i], d_lo, d_hi);
            }
        };
        clamp_to_walls();
        for (int pass = 0; pass < 2; ++pass) {
            std::vector<double> smoothed = lane_d;
            for (int i = 1; i + 1 < n; ++i) {
                smoothed[i] =
                    (lane_d[i - 1] + lane_d[i] + lane_d[i + 1]) / 3.0;
            }
            lane_d = std::move(smoothed);
        }
        clamp_to_walls();

        // 축소된 오프셋으로도 장애물을 비켜 가는가 — 이제 **장애물 옆 구간**만
        // 본다. 예전에는 전역 배율을 d_hold 에 곱한 한 숫자로 판정해서, 정작
        // 장애물 옆에서는 여유가 남는데도 다른 구간의 축소 때문에 거부됐다.
        const double need_clear2 = RequiredObstacleClearance(closest_obs);
        bool have_side = false;
        double d_side = d_hold;
        for (int i = 0; i < n; ++i) {
            const double t = i * wpnt_dist_;
            if (t < dist_front || t > dist_back) {
                continue;
            }
            if (!have_side) {
                d_side = lane_d[i];
                have_side = true;
            } else {
                d_side = (d_hold > 0.0) ? std::min(d_side, lane_d[i])
                                        : std::max(d_side, lane_d[i]);
            }
        }
        const bool still_clears = (d_hold > 0.0)
            ? d_side >= closest_obs.d_left + need_clear2
            : d_side <= closest_obs.d_right - need_clear2;
        if (!still_clears) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "LaneSwitch 거부: 벽 때문에 장애물 옆 오프셋이 %.2f -> %.2f로 "
                "줄어 간격 부족 (장애물 d[%.2f,%.2f], 필요 %.2f)",
                d_hold, d_side, closest_obs.d_right, closest_obs.d_left,
                need_clear2);
            NoteReject("lane_wall_clamped");
            return false;
        }
    }

    out_wpnts.wpnts.reserve(n);
    std::vector<std::pair<double, double>> xy(n);
    // 복귀를 그립 예산보다 짧게 줄였으면 그 구간 속도를 낮춰 물리를 맞춘다
    // (6*A*v²/L² <= blend_lat_acc_max  ->  v <= L*sqrt(a/(6A))).
    double return_speed_cap = -1.0;
    if (return_shortened && std::abs(d_hold) > 1e-6) {
        // a_lat_out: 거리(blend_out)를 잡은 예산과 같은 값이어야 한다.
        // (예전엔 여기만 blend_lat_acc_max_ 고정이라 코너에서 캡이 헐거웠다.)
        // 또 조건이 d_hold > 1e-6 이라 **오른쪽 회피(d_hold < 0)에서는 이 캡이
        // 통째로 꺼져 있었다** — 복귀를 그립 예산보다 짧게 줄여 놓고 속도는
        // 안 줄이는, 좌우 비대칭 버그. abs 로 고친다.
        return_speed_cap = blend_out *
            std::sqrt(a_lat_out / (6.0 * std::abs(d_hold)));
    }
    for (int i = 0; i < n; ++i) {
        double t = i * wpnt_dist_;
        double s = Mod(cur_s_ + t, track_len_);
        int gi = static_cast<int>(s / wpnt_dist_) % gb_size;
        double d = lane_d[i];
        xy[i] = FrenetToCartesian(s, d);

        // psi_rad 는 아래에서 실제 경로 헤딩으로 갱신된다.
        auto wpnt = MakeBaseWpnt(i, s, d, xy[i].first, xy[i].second, gi);
        double vx = std::min(gb_wpnts_[gi].vx_mps, gb_vmax_);
        if (v_floor > 0.0) {
            vx = std::clamp(vx, std::min(v_floor, gb_vmax_), gb_vmax_);
        }
        // 복귀 속도 캡은 **복귀 구간에만** 건다.
        // 예전 조건은 t >= hold_end - blend_in 이었다. blend_in 은 *진입*
        // 블렌드 길이라 복귀 캡의 시작점을 역산할 근거가 없고, 조기 전환으로
        // blend_in 이 길어질수록 캡이 앞당겨져 **장애물 옆(hold 구간)부터**
        // 속도가 깎였다 — "미리 비킬수록 느려지는" 역결합.
        // 캡 속도에 도달하기 위한 제동 run-up 은 ApplySpeedProfile 의
        // 역방향 감속 패스가 이미 담당한다.
        if (return_speed_cap > 0.0 && t >= hold_end) {
            vx = std::min(vx, return_speed_cap / std::max(cmd_speed_scale_, 0.05));
        }
        // 진입 속도 캡: 자차 위치부터 진입 블렌드가 끝나는 지점까지.
        // 여기서 미리 낮춰야 차가 블렌드 시작 시점에 그 속도에 도달해 있다.
        // (이 캡이 물리적으로 도달 불가능하면 아래 ApplySpeedProfile의
        //  require_current_feasible이 잡아 상위 폴백으로 넘긴다.)
        if (std::isfinite(entry_speed_cap) && t <= blend_start + blend_in) {
            vx = std::min(vx, entry_speed_cap / std::max(cmd_speed_scale_, 0.05));
        }
        wpnt.vx_mps = vx;   // ax_mps2 는 MakeBaseWpnt 가 이미 채웠다
        out_wpnts.wpnts.push_back(wpnt);
    }

    // 오프셋 경로의 실제 곡률로 속도 상한을 다시 잡고 역방향 감속 패스를
    // 건다. 직선(kappa~0)에서는 raceline 속도가 그대로 유지되고, 코너 진입
    // 전에서만 크게 감속된다.
    if (!ApplySpeedProfile(out_wpnts.wpnts)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "LaneSwitch 거부: 속도 프로파일 불성립 (v=%.1f) — %s",
            cur_vs_, SpeedRejectDetail().c_str());
        NoteReject("lane_speed_profile");
        out_wpnts.wpnts.clear();
        return false;
    }

    // 목표 장애물도 포함해 실제 sampled path와 시간축 footprint를 검증한다.
    double blocking_s = -1.0;
    if (!CheckPathCollision(out_wpnts.wpnts, all_ahead, &blocking_s)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "LaneSwitch 거부: 경로가 장애물과 충돌 (blocking s=%.2f)", blocking_s);
        NoteReject("lane_collision");
        out_wpnts.wpnts.clear();
        return false;
    }

    // 인접 점으로 실제 헤딩 계산 + 시각화 마커
    for (size_t i = 0; i + 1 < out_wpnts.wpnts.size(); ++i) {
        out_wpnts.wpnts[i].psi_rad = std::atan2(
            xy[i + 1].second - xy[i].second, xy[i + 1].first - xy[i].first);
    }
    out_mrks.markers.reserve(out_wpnts.wpnts.size());
    for (size_t i = 0; i < out_wpnts.wpnts.size(); ++i) {
        out_mrks.markers.push_back(MakeWpntMarker(
            xy[i].first, xy[i].second, out_wpnts.wpnts[i].vx_mps,
            static_cast<int>(i)));
    }
    return true;
}

bool SplinerNode::DoSpline(
    const f1tenth_planning_msgs::msg::Obstacle &closest_obs,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> &all_ahead,
    f1tenth_planning_msgs::msg::WpntArray &out_wpnts,
    visualization_msgs::msg::MarkerArray &out_mrks) {

    int gb_size = static_cast<int>(gb_wpnts_.size());

    // apex: 장애물 중심 (s wrap 처리)
    double s_apex;
    if (closest_obs.s_end < closest_obs.s_start) {
        s_apex = (closest_obs.s_end + track_len_ + closest_obs.s_start) / 2.0;
    } else {
        s_apex = (closest_obs.s_end + closest_obs.s_start) / 2.0;
    }

    // apex 이후 ~2m 곡률 합으로 코너 바깥쪽 판단.
    // 데드밴드 필수: 부호만 보면 직선(합≈0)에서 라이다/곡률 노이즈로 좌우가
    // 프레임마다 뒤집히고, 뒤집힐 때마다 절점 간격이 outside_spline_scale_
    // (1.75배)씩 점프해 경로 모양이 요동친다. 아래 has_outside 가 거짓이면
    // "바깥쪽 없음"으로 보고 배율을 적용하지 않는다.
    double kappa_sum = 0.0;
    int apex_idx = static_cast<int>(s_apex / wpnt_dist_) % gb_size;
    for (int i = 0; i < 20; ++i) {
        kappa_sum += gb_wpnts_[(apex_idx + i) % gb_size].kappa_radpm;
    }
    const bool has_outside = std::abs(kappa_sum) > kOutsideKappaSumMin;
    std::string outside = (kappa_sum < 0.0) ? "left" : "right";

    auto [more_space, d_apex] = MoreSpace(closest_obs);

    // 이미 회피 중(|d| > side_switch_d_tresh, 그리고 그 d의 **부호가 유지
    // 사이드와 일치**)이면 MoreSpace가 반대를 제안해도 현재 사이드를 유지한다
    // (ForzaETH danger_flag는 거부->trailing 폴백이었는데, 회피 중 d=0
    // trailing 경로로의 붕괴가 장애물 사이로 끌려 들어가는 충돌 원인이라
    // 유지-재계산으로 변경).
    //
    // 2026-08-07 수정 2가지:
    //  (1) 부호 검사 추가 — 예전에는 |cur_d_|만 봐서, 차가 오른쪽으로 밀려
    //      있는데 last_ot_side_="left"면 "왼쪽 유지"로 고정했다
    //      (run_164216 t=5.95 "Keep OT side left (proposed right, d=-0.27)").
    //      게다가 그 "left"는 직전 유령을 피하려고 세팅된 값이었다.
    //  (2) 유지 사이드가 불가능하면 **제안 사이드로 폴백**한다. 예전에는
    //      그대로 return false -> 모든 레인 막힘 -> trailing(v=0) -> 정지였다.
    //      run_164645 헤어핀에서 왼쪽 여유 0.67m가 비어 있는데 "right 유지"에
    //      걸려 전패했고 그대로 충돌했다. 실제 안전은 아래 CheckPathCollision이
    //      판정하므로, 여기서 후보를 지울 이유가 없다.
    std::string desired = more_space;
    if (SideKeepActive(closest_obs) && more_space != last_ot_side_) {
        desired = last_ot_side_;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "Keep OT side %s (proposed %s, mid-evasion d=%.2f)",
            desired.c_str(), more_space.c_str(), cur_d_);
    }
    if (desired != more_space) {
        // 요구 간격은 TryLaneSwitch/BuildLaneSwitch와 같은 기준을 쓴다.
        // 예전엔 여기와 아래 proposed_ok, 그리고 "이미 장애물 구간 내부"
        // 판정만 (지금은 제거된) evasion_dist_min_(0.20)이라 검증기 요구치
        // 보다 느슨했고, 그 사이 간격에서 DoSpline이 통과시킨 경로를
        // CheckPathCollision이 다시 거부하는 헛사이클이 생겼다.
        const double need_clear =
            RequiredObstacleClearance(closest_obs);
        const double kept_apex = ApexFor(desired, closest_obs);
        const bool kept_ok = (desired == "left")
            ? kept_apex >= closest_obs.d_left + need_clear
            : kept_apex <= closest_obs.d_right - need_clear;
        if (kept_ok) {
            more_space = desired;
            d_apex = kept_apex;
        } else {
            // 유지 사이드가 막혔다 — 회피를 버리지 말고 원래 제안(넓은 쪽)으로
            // 되돌린다. 제안 쪽도 최소 간격이 안 나오면 그때만 포기한다.
            const bool proposed_ok = (more_space == "left")
                ? d_apex >= closest_obs.d_left + need_clear
                : d_apex <= closest_obs.d_right - need_clear;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "OT side %s infeasible (d_apex=%.2f) -> %s 폴백 (%s)",
                desired.c_str(), kept_apex, more_space.c_str(),
                proposed_ok ? "성립" : "양쪽 불가");
            if (!proposed_ok) {
                pitwall::Event("사이드 유지 불가 (" + desired + ")");
                NoteReject("side_both_blocked");
                return false;
            }
            // 이미 그 사이드로 몸이 나가 있으면 반대편으로 넘기지 않는다.
            // d=+0.68에서 오른쪽으로 가려면 트랙을 가로질러야 하고, 그
            // 경로는 장애물 정면을 지난다. 0808 run_185049에서 이 전환이
            // 통과 직후 4랩(85.22 / 109.26 / 122.09 / 134.19)에 발생했고,
            // 로컬 경로가 반대편으로 점프하면서 control 추종 백스톱을
            // 발화시켰다(경로오차 0.37~0.63m -> 속도 1.5 m/s).
            //
            // ⚠️ 2026-08-12: **이 분기에서는 그 가드를 적용하지 않는다.**
            // 여기는 `kept_ok == false` 분기다 = 잠긴 사이드가 이미 막혔고,
            // 바로 위에서 `proposed_ok == true` 로 반대쪽이 성립함을 확인했다.
            // 그 상태에서 전환을 막으면 남는 선택은 "막힌 쪽을 붙잡고 감속"
            // 하나뿐이고, 그건 회피 실패로 직결된다.
            //   run_0812_013630 실측: 이 거부가 7회 발동했고 **7회 모두**
            //   직전 로그가 "OT side ... -> 반대쪽 폴백 (성립)" 이었다.
            //   1:15 관통 / 2:20 관통(2건)이 전부 이 경로다.
            //   그리고 거부 시점 |cur_d| 가 0.20 0.25 0.22 0.20 0.26 0.20 0.21
            //   — 전부 임계값(0.20) +6cm 안이다. 장애물이 없는 구간에서도
            //   |d| > 0.20 인 시간이 36.4%(p90 0.288)이므로, 이 임계값은
            //   "회피 중"이 아니라 **평상시 추종 흔들림**을 잡고 있었다.
            // 가로지르기 자체의 위험은 여기서 막을 일이 아니다 — 아래
            // BlendGeomFloor(가로지르는 **더 큰 진폭**으로 계산됨) 와
            // CheckPathCollision 이 그대로 판정한다. 이 가드는 원래 양쪽이
            // 모두 성립할 때의 **불필요한** 가로지르기를 막는 사전 필터였다.
            // OtSideCommitFresh 는 TryLaneSwitch 의 committed_side 와 같은
            // 수명 조건이다. 여기서는 거부가 아니라 **로그/카운터 조건**이므로
            // 낡은 커밋에서 헛로그가 뜨는 것만 막는다.
            if (OtSideCommitFresh() &&
                std::abs(cur_d_) > side_switch_d_tresh_ &&
                ((cur_d_ > 0.0) ? (desired == "left") : (desired == "right"))) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                    "OT side 전환 허용: %s 막힘 + %s 성립 (cur_d=%.2f) — "
                    "기하/충돌 검증에 맡긴다",
                    desired.c_str(), more_space.c_str(), cur_d_);
                NoteReject("side_switch_crossed_while_committed");
            }
            pitwall::Event("사이드 전환 (" + desired + " -> " + more_space + ")");
            CommitOtSide(more_space, closest_obs);
        }
    }

    // 스플라인 절점: pre-apex 3점 + apex 구간 + post-apex 3점.
    // apex는 점이 아니라 장애물(클러스터)의 s 구간 [s_front, s_back] — 점
    // 장애물이면 기존 단일 apex와 사실상 동일하고, 병합된 긴 클러스터면
    // 구간 전체에서 d_apex를 유지한다 (단일 apex는 꼬리에서 d=0으로
    // 되돌아와 클러스터 뒷부분을 관통).
    // apex 유지 구간을 장애물 앞뒤로 0.5m 연장 — pure pursuit lookahead가
    // 진입/이탈 대각선을 안쪽(장애물 쪽)으로 커팅해 스치는 것을 방지
    double obs_span = Mod(closest_obs.s_end - closest_obs.s_start, track_len_);
    double s_front = s_apex - obs_span / 2.0 - 0.5;
    double s_back = s_apex + obs_span / 2.0 + 0.5;

    // 절점 거리 스케일. 기존 (1 + v/vmax)는 v = vmax/2 에서 상한 1.5에 걸려
    // 그 위로는 속도가 올라가도 진입 거리가 그대로였다 — 횡가속이 v^2로 증가.
    // 그립 예산으로 계산한 필요 거리(BlendDist)를 하한으로 함께 적용한다.
    // 속도는 실측이 아니라 PlanSpeed() — 정지/저속에서 절점이 붕괴하면
    // 기하곡률이 조향 한계를 넘어 ApplySpeedProfile이 경로를 조용히 버린다.
    // 이 회피 구간의 횡 전환 그립 (마찰원 잔여).
    const double a_lat = BlendLatAcc(Mod(s_front, track_len_),
                                     Mod(s_back, track_len_));
    // 조향 기하 하한에 더할 트랙 곡률항 (BlendGeomFloor).
    // BuildLaneSwitch의 kappa_in과 같은 규칙: 진입 구간은 아직 라인 위이므로
    // 레이스라인 원본 곡률, apex 유지 구간([s_front, s_back])만 d_apex 오프셋
    // 경로의 유효 곡률을 본다. 전 구간에 d_apex를 먹이면 헤어핀 진입에서
    // OffsetKappa가 조향 한계를 넘어 성립하던 회피가 전부 뒤집힌다.
    const double kappa_sp = std::max(
        BlendKappaMax(Mod(cur_s_, track_len_), Mod(s_front, track_len_)),
        BlendKappaMax(Mod(s_front, track_len_), Mod(s_back, track_len_),
                      d_apex));
    // 진입 가용 거리 안에서 성립하는 속도로 계획한다 (BuildLaneSwitch와 동일
    // 근거: 정적 장애물은 속도가 자유 변수이므로 이진 거부 대신 속도 캡).
    // ── s_apex 기준 **부호 있는 최단** 차이 (2026-08-22 수정) ──────────────
    // 예전 식 `s_apex - Mod(s_apex - cur_s_, track_len_)` 은 Mod 가 [0, len)
    // 이라 결과가 **항상 <= s_apex** 였다. 자차가 apex 를 조금이라도 지나면
    // 한 바퀴 뒤(track_len 만큼 음수)로 감기고, 그러면 아래 두 가드가 **둘 다
    // 조용히 건너뛰어진다**:
    //   · "이미 장애물 구간 내부" 거부  (cur_unwrapped >= s_front && <= s_back)
    //   · ego 재앵커                    (cur_unwrapped > knot_s.front())
    // 그 결과 경로가 apex-4 m(= pre_apex_dists 첫 절점)에서 시작한다 —
    // **자차보다 뒤**다. 그리고 진입 가용거리(entry_avail_pre)가 한 바퀴만큼
    // 부풀어 속도 캡도 걸리지 않는다.
    //   run_0822_000933 실측 (t=29 / 56 / 1:07 세 건 모두 같은 전개):
    //     t=30.06 자차 s=5.71 d=-0.40 인데 발행 경로가 s=1.65 에서 시작하고
    //     그 지점 계획 d=+0.11 -> 경로오차 0.51 m. 그 경로가 t=30.09 에
    //     커밋돼 **1.5 초간 얼었고**(자차 d -0.40 -> -0.96) 차가 정지했다.
    //     커밋은 이탈이 아니라 `nothing_ahead`(장애물 소멸)로만 풀렸다.
    const double cur_unwrapped_pre = s_apex +
        (Mod(cur_s_ - s_apex + track_len_ / 2.0, track_len_) -
         track_len_ / 2.0);
    // BuildLaneSwitch와 같은 기준: s_front는 장애물 앞 0.5m hold 시작점이지만
    // 횡전환을 끝내야 하는 진짜 경계는 충돌 박스(vehicle_half_length)다.
    const double entry_avail_pre =
        (s_apex - obs_span / 2.0 - vehicle_half_length_m_) - cur_unwrapped_pre;
    double entry_speed_cap = std::numeric_limits<double>::infinity();
    double plan_v = PlanSpeed();
    // 진입 블렌드 시작 s — 지점별 기하 검사 기준 (BuildLaneSwitch 와 동일).
    const double blend_s0 = Mod(cur_s_, track_len_);
    if (entry_avail_pre >= 0.0) {
        // 충돌만 벗어나는 최소 오프셋 (BuildLaneSwitch의 d_hold_min과 동일 논리)
        const double clear_margin = RequiredObstacleClearance(closest_obs);
        const bool apex_left = (d_apex >= closest_obs.d_left);
        const double d_apex_min = apex_left
            ? closest_obs.d_left + clear_margin
            : closest_obs.d_right - clear_margin;
        double cap = BlendSpeedCap(
            d_apex - cur_d_, entry_avail_pre + wpnt_dist_, a_lat, kappa_sp,
            blend_s0);
        const bool relax_possible =
            apex_left ? (d_apex_min < d_apex) : (d_apex_min > d_apex);
        if (!(cap > 0.0) && relax_possible) {
            const double relaxed = BlendSpeedCap(
                d_apex_min - cur_d_, entry_avail_pre + wpnt_dist_, a_lat,
                kappa_sp, blend_s0);
            if (relaxed > 0.0) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                    "FreeSpline 최소오프셋 완화: d_apex %.2f -> %.2f (가용 %.2fm)",
                    d_apex, d_apex_min, entry_avail_pre);
                d_apex = d_apex_min;
                cap = relaxed;
            }
        }
        if (!(cap > 0.0)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "Free spline infeasible: %.2fm 횡이동에 기하 하한 %.2fm 필요, "
                "가용 %.2fm (kappa=%.2f) — 어떤 속도로도 불가",
                d_apex - cur_d_,
                BlendGeomFloor(d_apex - cur_d_, kappa_sp, false, blend_s0),
                entry_avail_pre, kappa_sp);
            NoteReject("spline_geom_floor");
            free_spline_geom_infeasible_ = true;
            return false;
        }
        // 비교 대상은 "지금 속도"가 아니라 "진입 구간을 지나는 동안 낼 속도"다.
        // plan_v 하나와만 비교하면 **가속 구간에서 캡이 아예 안 걸린다** —
        // BuildLaneSwitch 는 이 결함을 BlendPlanSpeed 로 고쳤는데 여기만
        // 옛 비교가 남아 있어 같은 상황에서 두 티어가 다른 답을 냈다.
        const double carried_in =
            BlendPlanSpeed(blend_s0, entry_avail_pre + wpnt_dist_);
        if (cap < std::max(plan_v, carried_in)) {
            entry_speed_cap = std::max(cap, plan_speed_floor_);
            plan_v = std::min(plan_v, entry_speed_cap);
        }
        // 기하 하한은 통과했다 — 이후 속도/경계/충돌 검증에서 떨어져도
        // 이 apex가 "가려던 목표"다 (TryLaneSwitch와 같은 이유).
        NoteRetryTarget(d_apex);
    }
    // ── 두 스케일의 분모가 다른 것은 의도된 것이다 ─────────────────────────
    // need_scale 은 첫 절점까지의 **진입 활주로 전체**(|pre.front()| = 4m),
    // geom_scale 은 전환이 실제로 몰리는 **마지막 절점 간격**(|pre.back()| =
    // 1.5m) 으로 나눈다. 자연 3차 스플라인은 C² 라 d'' 가 마지막 구간에만
    // 갇히지 않고 앞 구간으로 번지므로 실효 전환길이는 1.5σ~4σ 사이다.
    //   - geom_scale = 조향 기하 **하한**(= 못 넘으면 어떤 속도로도 불가)이라
    //     짧은 쪽(1.5σ)으로 보수적으로 잡는 것이 맞다.
    //   - need_scale = 그립 예산에 따른 **완만함 선호**이고, 못 지켜도 위험이
    //     아니라 ApplySpeedProfile 이 실제 샘플 곡률로 속도를 눌러 처리한다.
    // 두 분모를 1.5σ 로 통일해 보면 need_scale 이 2.67배가 되어 speed_scale 이
    // 상한(3.0)에 붙고, 절점 스팬이 8σ = 24m 까지 자란다. 이 트랙(29~33m)에서는
    // 계획이 랩의 70~80% 를 덮으면서 경계 거부(spline_out_of_bounds)가 늘어
    // **폴백 티어의 성공률이 떨어진다** — 안전 이득 없이 손해다. 그대로 둔다.
    const double knot_base = std::abs(pre_apex_dists_.front());
    double need_scale = (knot_base > 0.1)
        ? BlendDist(d_apex, plan_v, a_lat, kappa_sp, blend_s0) / knot_base : 1.0;
    // 조향 기하 하한: 위 need_scale은 "그립 예산"만 본다 — 저속에서는 횡가속
    // 예산이 커 보여도(a/v²) 조향각 한계라는 순수 기하 제약이 남는다.
    // d_apex를 옮기는 전환은 마지막 pre-apex 절점 간격에서 일어나므로
    // 그 길이가 L >= sqrt(6*A/kappa_max)를 만족해야 한다. 안 그러면
    // ApplySpeedProfile이 곡률 초과로 경로를 통째로 거부하는데, 속도가
    // 이미 0이라 감속으로 살릴 수도 없다 — 정지 라치의 직접 원인이었다.
    // geom_floor_margin 여유는 자연 3차 스플라인의 실제 피크가 smoothstep
    // 근사보다 큰 것을 흡수한다.
    const double knot_gap = std::abs(pre_apex_dists_.back());
    double geom_scale = 1.0;
    if (knot_gap > 0.05) {
        // BlendGeomFloor와 같은 식(트랙 곡률 포함)을 쓴다 — 예전에는 여기만
        // kappa_track=0 가정이라 헤어핀에서 절점 스케일이 과소평가됐다.
        // ⚠️ 여기는 **일부러 스칼라 식**(s_from 미전달)으로 남긴다. 이 전환이
        // 일어나는 구간은 마지막 pre-apex 절점 간격인데, 그 간격 자체가
        // geom_scale 로 늘어나므로 시작 s 가 결과에 의존한다(순환). 잘못된 s 로
        // 지점별 검사를 하면 엉뚱한 곡률을 보므로, 정확한 시작 s 를 정의하기
        // 전까지는 보수적인 기존 식을 쓴다 — 오늘보다 느슨해지지 않는다.
        geom_scale = BlendGeomFloor(d_apex, kappa_sp) / knot_gap;
    }
    // 상한(kMaxKnotScale)이 geom_scale을 그냥 잘라내면, 조향 기하상 불가능한
    // 것을 알면서 그 절점으로 경로를 만들고 ApplySpeedProfile이 나중에 거부한다.
    // 그러면 거부 사유가 "기하 불성립(영구)"이 아니라 "속도 프로파일(일시)"로
    // 기록되어, Tier3의 "정지 vs 감속 재시도" 판정까지 반대로 간다.
    // 여기서 기하 불성립으로 정직하게 반환한다.
    if (geom_scale > kMaxKnotScale) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "Free spline infeasible: 절점 스케일 %.2f 필요 (상한 %.1f), "
            "d_apex=%.2f kappa=%.2f — 어떤 속도로도 불가",
            geom_scale, kMaxKnotScale, d_apex, kappa_sp);
        NoteReject("spline_geom_floor");
        free_spline_geom_infeasible_ = true;
        return false;
    }
    double speed_scale = std::clamp(
        std::max({1.0 + plan_v / std::max(gb_vmax_, 0.1), need_scale,
                  geom_scale}), 1.0, kMaxKnotScale);
    std::vector<double> knot_s, knot_d;
    for (double dst : pre_apex_dists_) {
        double scaled = dst * speed_scale;
        // 코너 바깥쪽으로 추월할 때는 스플라인을 더 완만하게
        // (has_outside: 곡률 합 데드밴드 — 직선에서 좌우가 뒤집히는 것 방지)
        if (has_outside && outside == more_space) {
            scaled *= outside_spline_scale_;
        }
        knot_s.push_back(s_front + scaled);
        knot_d.push_back(0.0);
    }
    knot_s.push_back(s_front);
    knot_d.push_back(d_apex);
    knot_s.push_back(s_back);
    knot_d.push_back(d_apex);
    // 복귀도 동일하게 완만하게 — raceline 반대편 벽 여유가 좁으면
    // (이 raceline은 한쪽이 ~0.35m) 가파른 복귀 오버슛이 반대 벽을 친다
    for (double dst : post_apex_dists_) {
        double scaled = dst * speed_scale;
        if (has_outside && outside == more_space) {
            scaled *= outside_spline_scale_;
        }
        knot_s.push_back(s_back + scaled);
        knot_d.push_back(0.0);
    }

    // 현재 위치를 장애물 apex와 같은 unwrapped 축으로 옮긴다.
    // (위 cur_unwrapped_pre 와 같은 부호 있는 최단 차이.)
    const double cur_unwrapped = s_apex +
        (Mod(cur_s_ - s_apex + track_len_ / 2.0, track_len_) -
         track_len_ / 2.0);
    // 진입 가부는 위(plan_v 계산부)에서 속도 캡으로 이미 처리했다 —
    // 기하 하한을 못 채우는 경우만 거기서 false를 돌린다.
    if (cur_unwrapped >= s_front && cur_unwrapped <= s_back) {
        // 위 kept_ok/proposed_ok와 같은 기준. (제거된) evasion_dist_min_이면
        // 장애물 몸통 안에 있는데도 already_clear가 참이 돼 스플라인 생성을
        // 계속 시도한다.
        const double already_need =
            RequiredObstacleClearance(closest_obs);
        const bool already_clear = (more_space == "left")
            ? cur_d_ >= closest_obs.d_left + already_need
            : cur_d_ <= closest_obs.d_right - already_need;
        if (!already_clear) {
            // 장애물 구간 안에서 새 횡전환을 시작하면 sampled path가 대상
            // 내부를 통과한다. 감속 명령으로 이 기하 오류를 숨기지 않는다.
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "FreeSpline 거부: 이미 장애물 구간 내부 (cur_d=%.2f, %s 필요)",
                cur_d_, more_space.c_str());
            NoteReject("spline_inside_obstacle");
            // 위 BuildLaneSwitch 의 alongside 거부와 같은 이유로 기하 불성립
            // 플래그를 세운다 — 감속으로 벗어날 수 있는 상황이 아니다.
            free_spline_geom_infeasible_ = true;
            return false;
        }
    }

    // 재계획 중 차량 뒤쪽 절점을 모두 버리고 현재 pose를 정확한 첫 절점으로
    // 넣는다. 기존 조건은 s_front 직전까지만 처리해 hold/복귀 중 뒤쪽 경로를
    // 다시 발행할 수 있었다.
    if (cur_unwrapped > knot_s.front()) {
        if (cur_unwrapped >= knot_s.back() - wpnt_dist_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "FreeSpline 거부: ego가 절점 끝을 지남 (ego %.2f, 끝 %.2f)",
                cur_unwrapped, knot_s.back());
            NoteReject("spline_ego_past_knots");
            return false;
        }
        size_t keep = 0;
        while (keep < knot_s.size() && knot_s[keep] <= cur_unwrapped + 1e-6) {
            ++keep;
        }
        knot_s.erase(knot_s.begin(), knot_s.begin() + static_cast<long>(keep));
        knot_d.erase(knot_d.begin(), knot_d.begin() + static_cast<long>(keep));
        knot_s.insert(knot_s.begin(), cur_unwrapped);
        knot_d.insert(knot_d.begin(), cur_d_);
    }
    if (knot_s.size() < 3 || !PlanSpanAllowed(knot_s.back() - knot_s.front())) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "Free spline rejected: invalid/one-lap span %.1fm",
            knot_s.empty() ? 0.0 : knot_s.back() - knot_s.front());
        return false;
    }

    // d(s) 자연 3차 스플라인을 0.1m 간격으로 샘플
    std::vector<double> evasion_s;
    for (double s = knot_s.front(); s < knot_s.back(); s += spline_resolution_) {
        evasion_s.push_back(s);
    }
    std::vector<double> evasion_d = NaturalCubicSpline(knot_s, knot_d, evasion_s);
    if (evasion_d.size() != evasion_s.size()) {
        // 절점이 단조증가가 아님 (pre/post_apex_dists 설정 오류 등) -> 회피 포기
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "스플라인 절점이 단조증가가 아님 — 회피 포기 "
            "(spliner/pre_apex_dists, post_apex_dists 확인)");
        return false;
    }
    // 클램프 범위에 첫 절점 d(ego-anchor 시 cur_d)를 포함해 시작점 연속성 유지
    double d_lo = std::min({0.0, d_apex, knot_d.front()});
    double d_hi = std::max({0.0, d_apex, knot_d.front()});
    for (auto &d : evasion_d) {
        d = std::clamp(d, d_lo, d_hi);
    }

    // 궤적 생성 + 트랙 경계 안전 검사
    out_wpnts.wpnts.reserve(evasion_s.size());
    std::vector<std::pair<double, double>> xy(evasion_s.size());
    for (size_t i = 0; i < evasion_s.size(); ++i) {
        double s_wrapped = Mod(evasion_s[i], track_len_);
        int gb_idx = static_cast<int>(s_wrapped / wpnt_dist_) % gb_size;

        // raceline에서 실제로 벗어난 구간만 경계 거리 검사.
        // 허용치를 넘쳐도 바로 포기하지 않고 그 지점만 벽 허용치로 눌러본다 —
        // 눌렀을 때 장애물 구간에서 evasion_dist_min을 침범할 때만 포기.
        // (벽 폭은 s 따라 변하므로 한 점 넘쳤다고 전체를 버리면 회피포기 남발)
        if (std::abs(evasion_d[i]) > kOffRacelineEpsM) {
            // (예전엔 장애물 corridor 구간(in_obs_span)에서만 완화된
            //  ot_bound_mindist 를 썼으나, 그 값이 spline_bound_mindist 와 같고
            //  RequiredBoundaryMargin 하한에 먹혀 구분이 성립한 적이 없다.)
            double bound = RequiredBoundaryMargin(spline_bound_mindist_);
            double d_min = -std::abs(gb_wpnts_[gb_idx].d_right) + bound;
            double d_max = std::abs(gb_wpnts_[gb_idx].d_left) - bound;
            if (d_min > d_max) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                    "FreeSpline 거부: s=%.2f 통과폭 음수 (d_min %.2f > d_max %.2f)",
                    s_wrapped, d_min, d_max);
                NoteReject("spline_width_negative");
                out_wpnts.wpnts.clear();
                return false;
            }
            if (evasion_d[i] < d_min || evasion_d[i] > d_max) {
                double clamped = std::clamp(evasion_d[i], d_min, d_max);
                // "장애물을 파고드는가"는 장애물이 실제로 있는 s 구간에서만
                // 물을 수 있다. in_obs_span은 hold 여유 0.5m + 판정 여유 0.3m을
                // 포함하므로 장애물 앞 0.8m까지 덮는데, 그 구간의 경로는 아직
                // apex로 이동하는 중이라 당연히 간격이 부족하다 — 이걸 "관통"
                // 으로 보면 진입 램프의 벽 클램프가 매 사이클 회피를 포기시킨다
                // (run_0806_155621: abort 91%가 s_front보다 앞인 진입부에서 발생).
                // 차 앞머리가 장애물에 닿는 지점부터 판정한다.
                const double body_front = s_front + 0.5 - vehicle_half_length_m_;
                const double body_back = s_back - 0.5 + vehicle_half_length_m_;
                bool in_obs_body = (evasion_s[i] > body_front) &&
                                   (evasion_s[i] < body_back);
                const double need_clear = RequiredObstacleClearance(closest_obs);
                bool cuts_obstacle = in_obs_body &&
                    ((more_space == "left")
                         ? clamped < closest_obs.d_left + need_clear
                         : clamped > closest_obs.d_right - need_clear);
                if (cuts_obstacle) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                        "Evasion trajectory too close to TRACKBOUNDS, aborting evasion");
                    NoteReject("spline_boundary_abort");
                    boundary_abort_now_ = true;
                    out_wpnts.wpnts.clear();
                    return false;
                }
                evasion_d[i] = clamped;
            }
        }
        if (!WithinTrackBoundary(gb_idx, evasion_d[i],
                                 spline_bound_mindist_)) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "FreeSpline 거부: 경계 이탈 s=%.2f d=%.2f (벽 L%.2f/R%.2f)",
                s_wrapped, evasion_d[i], gb_wpnts_[gb_idx].d_left,
                gb_wpnts_[gb_idx].d_right);
            NoteReject("spline_out_of_bounds");
            out_wpnts.wpnts.clear();
            return false;
        }

        xy[i] = FrenetToCartesian(s_wrapped, evasion_d[i]);

        // psi_rad 는 아래에서 실제 경로 헤딩으로 갱신된다.
        auto wpnt = MakeBaseWpnt(i, s_wrapped, evasion_d[i],
                                 xy[i].first, xy[i].second, gb_idx);
        // 추월 섹터 내 공격 부스트 (raceline 최고속 캡).
        {
            // (안쪽 회피의 고정 배율 inside_speed_scale 제거: cc22ca9 가
            //  offset_grip_cap(OffsetKappa + 레이스라인 그립 일관성 캡)으로
            //  대체한다고 명시했는데 이 곱셈만 남아 있었다. 배포 yaml 3개가
            //  모두 1.0 이라 궤적 변화는 없고, 코드 기본값 0.9 와의 불일치만
            //  사라진다 — yaml 없이 띄우면 offset_grip_cap 과 이중 감속이었다.)
            wpnt.vx_mps = std::min(
                gb_wpnts_[gb_idx].vx_mps, gb_vmax_);
            // 진입 속도 캡 — apex 도달 전까지. 정적 장애물에서 "지금 속도로는
            // 안 되니 포기"(-> 정지) 대신 "되는 속도로 간다"를 실현하는 부분.
            // 동적 상대에도 걸어야 한다: entry_speed_cap 은 is_static 과 무관하게
            // 계산되고 스플라인 절점 간격(need_scale)이 "캡된 속도로 간다"는
            // 전제로 잡히므로, 여기서만 정적으로 한정하면 동적 회피는 낮은 속도
            // 전제로 만든 기하를 원래 속도로 달리게 된다.
            if (std::isfinite(entry_speed_cap) && evasion_s[i] <= s_front) {
                wpnt.vx_mps = std::min(
                    wpnt.vx_mps, entry_speed_cap / std::max(cmd_speed_scale_, 0.05));
            }
        }
        // ax_mps2 는 MakeBaseWpnt 가 이미 채웠다
        out_wpnts.wpnts.push_back(wpnt);
    }

    // 오프셋 곡률 기반 속도 상한 + 역방향 감속 패스 (레인스위치와 동일 기준)
    if (!ApplySpeedProfile(out_wpnts.wpnts)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "FreeSpline 거부: 속도 프로파일 불성립 (v=%.1f) — %s",
            cur_vs_, SpeedRejectDetail().c_str());
        NoteReject("spline_speed_profile");
        out_wpnts.wpnts.clear();
        return false;
    }

    // free-form도 목표/2차 장애물을 모두 검사한다. 이 검사가 없으면 고정
    // 레인을 막아 Tier 2로 보낸 두 번째 장애물을 그대로 관통했다.
    double blocking_s = -1.0;
    if (!CheckPathCollision(out_wpnts.wpnts, all_ahead, &blocking_s)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "FreeSpline 거부: 경로가 장애물과 충돌 (blocking s=%.2f)", blocking_s);
        NoteReject("spline_collision");
        out_wpnts.wpnts.clear();
        return false;
    }

    // 인접 점으로 회피 궤적의 실제 헤딩 계산
    for (size_t i = 0; i + 1 < out_wpnts.wpnts.size(); ++i) {
        out_wpnts.wpnts[i].psi_rad = std::atan2(
            xy[i + 1].second - xy[i].second, xy[i + 1].first - xy[i].first);
    }

    CommitOtSide(more_space, closest_obs);
    NoteOutcome("ok_free_spline");

    // 시각화 마커
    out_mrks.markers.reserve(out_wpnts.wpnts.size());
    for (size_t i = 0; i < out_wpnts.wpnts.size(); ++i) {
        out_mrks.markers.push_back(MakeWpntMarker(
            xy[i].first, xy[i].second, out_wpnts.wpnts[i].vx_mps, static_cast<int>(i)));
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
        "Evasion spline: side=%s d_apex=%.2f s_apex=%.1f (%zu wpnts)",
        more_space.c_str(), d_apex, Mod(s_apex, track_len_), out_wpnts.wpnts.size());
    pitwall::Log("evasion_d_apex", d_apex);
    return true;
}

bool SplinerNode::BuildEscapeArc(
    const f1tenth_planning_msgs::msg::Obstacle &closest_obs,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> &all_ahead,
    f1tenth_planning_msgs::msg::WpntArray &out_wpnts,
    visualization_msgs::msg::MarkerArray &out_mrks) {
    const int gb_size = static_cast<int>(gb_wpnts_.size());
    if (!escape_arc_enabled_ || gb_size < 2) {
        return false;
    }
    // (1) 정적(또는 준정적) + 지금 실제로 보이는 것만.
    //     외삽 유령에 급조향하면 벽으로 간다.
    //     예외: 정지 워치독(hpp 주석)이 선 뒤에는 가시성만 면제한다 —
    //     T초 정지를 버틴 정적 장애물이 유령이면 브릿지 스캔 카빙이 이미
    //     지웠고, coasting 트랙이면 track_forget(2.0s) 안에 만료됐다.
    //     그 면제는 **정적 전용** 이다.
    //
    // ── 준정적 개방 (2026-08-20) ─────────────────────────────────────────
    // 예전 주석은 "is_static 요구는 어떤 경우에도 풀지 않는다"였다. 그
    // 근거는 (a) 유령에 급조향 (b) 상대 이동을 무시한 원호 두 가지인데,
    // 준정적에서는 둘 다 이미 막혀 있다:
    //   (a) 커밋 승격 게이트가 is_visible + ObsSeen >= commit_min_obs
    //       + 예측튜브 제외를 요구한다 — 커밋 승격과 같은 관측 게이트다.
    //   (b) 이 함수는 아래에서 CheckPathCollision(all_ahead) 을 통과해야만
    //       true 를 반환하고, 그 검사는 동적 장애물을 도착시각만큼
    //       전파(motion_s/motion_d)한다. 즉 상대 이동은 검증에 살아 있다.
    // 여는 이유: 동적에는 백스톱이 하나도 없었다. Tier1 이 기하로 막히면
    // 곧장 Tier3(트레일링)로 떨어진다 — run_0819 코너 실측 동적 트레일링
    // 비율 92% vs 정적 11%. Tier 2(DoSpline, 자유 최적화)는 churn 근거가
    // 그대로라 계속 닫아 둔다. 여기만 연다.
    const bool escape_allowed = closest_obs.is_static;
    const bool visible_ok =
        closest_obs.is_visible ||
        (standstill_override_ && closest_obs.is_static);
    if (!escape_allowed || !visible_ok) {
        NoteReject("escape_not_visible_static");
        return false;
    }
    // ── 저속 거부 게이트 삭제 (2026-08-17) ──────────────────────────────
    // 예전에는 cur_vs_ < escape_arc_min_speed_ 를 여기서 거부했다
    // ("느리면 정상 회피가 성립한다"). 그 전제가 틀렸다: Tier1/2 의 기하
    // 하한은 조향 곡률 한계(속도 무관)로도 결정되므로 저속에서도 막힌다.
    // run_0817_145548 t=23.9: v=0.48, 장애물까지 1.87 m — lane/spline 전부
    // 기하 거부, 원호 하한 1.66 m 로 **유일하게 성립**하는 순간에 이 게이트가
    // 원호를 죽였고 13.8 초 정지(수동 개입)가 됐다. 같은 정지가 t=176 에
    // 재발했다. escape_arc_min_speed_ 는 아래 kappa_grip 의 속도 하한으로만
    // 남는다 — v=0 에서도 곡률 예산이 유한하게 잡힌다.
    // 장애물 앞머리까지 남은 거리. 이미 옆에 들어왔으면 원호로도 못 피한다.
    const double dist_front =
        Mod(closest_obs.s_start - cur_s_, track_len_) - vehicle_half_length_m_;
    if (!(dist_front > 0.2) || dist_front > lookahead_m_) {
        NoteReject("escape_dist_out_of_range");
        return false;
    }

    // (2) 비키는 쪽 결정 — 벽 여유가 실제로 있는 쪽. MoreSpace 와 같은 근거를
    //     쓰되, 여기서는 "통과폭이 있는가"만 본다 (apex 최적화는 몫이 아니다).
    double wall_left, wall_right;
    MinWallInSpan(Mod(closest_obs.s_start - 0.5, track_len_),
                  Mod(closest_obs.s_end + 0.5, track_len_),
                  wall_left, wall_right);
    const double need_clear = RequiredObstacleClearance(closest_obs);
    const double boundary = RequiredBoundaryMargin(spline_bound_mindist_);
    const double d_left_target = closest_obs.d_left + need_clear;
    const double d_right_target = closest_obs.d_right - need_clear;
    // MinWallInSpan 은 좌우 모두 **절대값(양수)** 으로 돌려준다. 오른쪽 경계는
    // d 축에서 음수 방향이므로 하한이 -wall_right + boundary 다
    // (WithinTrackBoundary 의 d_min 과 같은 식). 예전에는 +wall_right 로 써서
    // "자차가 트랙 왼쪽 **밖**에 있어야 참"인 조건이 됐고, right_ok 가 사실상
    // 항상 거짓이라 escape 원호는 왼쪽으로만 갈 수 있었다. 헤어핀 안쪽처럼
    // 왼쪽이 벽인 곳에서는 그대로 escape_both_blocked 로 떨어졌다
    // (run 20:47 세션: both_blocked 186회 / 원호 발동 0회).
    const bool left_ok = (d_left_target <= wall_left - boundary);
    const bool right_ok = (d_right_target >= -wall_right + boundary);
    if (!left_ok && !right_ok) {
        NoteReject("escape_both_blocked");
        return false;
    }
    // 둘 다 되면 현재 d 에서 가까운 쪽 — 옮길 양이 적을수록 필요 거리가 짧다.
    double d_target;
    if (left_ok && right_ok) {
        d_target = (std::abs(d_left_target - cur_d_) <=
                    std::abs(d_right_target - cur_d_))
            ? d_left_target : d_right_target;
    } else {
        d_target = left_ok ? d_left_target : d_right_target;
    }
    const double amplitude = d_target - cur_d_;
    const double a_amp = std::abs(amplitude);
    if (a_amp < 1e-3) {
        NoteReject("escape_already_clear");
        return false;   // 이미 비켜 있다 — 이 경로가 할 일이 없다
    }

    // (3) 곡률 예산. 조향 한계와 그립 한계 중 **작은 쪽**이 진짜 한계다.
    //     3 m/s 이상에서는 그립이 구속이라, 최대 조향을 넣으면 그냥 밀린다.
    const double kappa_track =
        BlendKappaMax(Mod(cur_s_, track_len_),
                      Mod(cur_s_ + dist_front, track_len_));
    const double kappa_steer = geom_floor_margin_ * std::max(
        SteerKappaMax() - std::abs(kappa_track),
        geom_floor_kappa_ratio_ * SteerKappaMax());
    const double v_now = std::max(cur_vs_, escape_arc_min_speed_);
    const double kappa_grip = escape_arc_lat_acc_ / (v_now * v_now);
    const double kappa_use = std::min(kappa_steer, kappa_grip);
    if (!(kappa_use > 1e-3)) {
        // 이 함수의 다른 조기 반환은 전부 카운터를 남긴다 — 여기만 조용히
        // 빠져서 "escape 가 왜 안 떴나"의 추적이 매번 이 지점에서 끊겼다.
        NoteReject("escape_no_curvature_budget");
        return false;
    }
    // ── 원호 + 기울기 완화 (2026-08-13) ──────────────────────────────────
    // 예전에는 순수 포물선 원호(A = kappa*L²/2 -> L = sqrt(2A/kappa)) 뒤에
    // **일정 d 유지(hold)** 를 바로 붙였다. 원호 끝의 횡기울기는 2A/L 인데
    // hold 의 기울기는 0 이라, 경로가 그 접합부에서 잔여 헤딩만큼 **한 점에서
    // 통째로 꺾였다.** 그 꺾임의 곡률이 1.14~1.77 로 나와 ApplySpeedProfile 의
    // speed_steer_curv(문턱 kappa_max*1.15 = 1.303)에 걸렸고, 정작 escape 가
    // 필요한 빡빡한 기하에서 **항상** 죽었다 (2026-08-12 시뮬 escape_active 0회).
    // 헤딩 게이트가 잔여 헤딩을 45.8도까지 허용해 놓고 hold 가 그걸 없애느라
    // 곡률 기각에 걸리는, 서로 모순된 구조였다.
    //
    // 이제 원호 뒤에 기울기를 0 으로 되돌리는 완화 구간을 붙인다. 전체 길이 D 를
    // 원호 0.4D / 완화 0.6D 로 나누면 두 구간의 피크 곡률이 모두 5A/D² 로
    // 같아진다(한쪽만 짧으면 그쪽이 먼저 한계에 걸리므로 이 분배가 최적).
    //   -> D >= sqrt(5A/kappa)
    // 하한이 sqrt(2A/k) 에서 sqrt(5A/k) 로 **늘었다**(1.58배). 그게 정직한 값이다
    // — 예전 하한은 실행 불가능한(꺾인) 경로를 전제로 계산된 값이었다.
    //
    // ⚠️ 이 변경으로 escape 의 원래 이점이 크게 줄어든다. "한쪽 원호는 2A/L²
    //    이라 smoothstep(6A/L²) 대비 3배 옮긴다"가 설계 근거였는데, 평행 복귀를
    //    하면 5A/D² 가 되어 **이점이 17% 뿐**이다. 3배 이점은 잔여 헤딩을 유지할
    //    때만 성립하고, 그러려면 원호 뒤를 직선으로 이어가야 하는데 그건 통과
    //    구간 내내 d 가 계속 커진다 — my_track0811 헤어핀에서 계산하면 최대 |d|
    //    0.62m 로 **벽을 0.18m 침범**한다(현행/이번 안은 0.31m). 이 트랙에서는
    //    잔여 헤딩 방식이 물리적으로 안 들어간다.
    const double arc_floor = std::sqrt(5.0 * a_amp / kappa_use);
    if (arc_floor > dist_front) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "EscapeArc 불가: %.2fm 횡이동에 원호 하한 %.2fm 필요, 가용 %.2fm "
            "(v=%.1f, kappa_use=%.2f)",
            a_amp, arc_floor, dist_front, cur_vs_, kappa_use);
        NoteReject("escape_geom_floor");
        return false;
    }
    // 실제로 쓸 길이는 가용 전체 (완만할수록 좋다).
    const double arc_len = dist_front;
    const double lead_len = 0.4 * arc_len;              // 원호 구간
    const double ease_len = arc_len - lead_len;         // 기울기 완화 구간
    // 두 구간 공통 피크 곡률. 부호는 기동 방향을 따른다.
    const double kappa_arc = 5.0 * a_amp / (arc_len * arc_len);
    const double kappa_signed = (amplitude >= 0.0) ? kappa_arc : -kappa_arc;
    // 기동이 **끝날 때** 헤딩은 0 이다(평행 복귀). 이 검사는 이제 기동 중
    // 최대 헤딩(원호가 끝나는 지점)을 본다 — 그 순간 차가 트랙 접선에서
    // 얼마나 틀어지는가. 값은 2A/D 로 예전 식과 같다.
    const double heading_left = kappa_arc * lead_len;
    if (heading_left > escape_arc_max_heading_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "EscapeArc 불가: 기동 중 최대 헤딩 %.0f도 (상한 %.0f도)",
            heading_left * 180.0 / M_PI,
            escape_arc_max_heading_ * 180.0 / M_PI);
        NoteReject("escape_heading_over");
        return false;
    }
    // 이 원호를 실제로 낼 수 있는 속도 상한 (그립).
    // 곡률은 기동분 **과 트랙분의 합**이다 — 기동분만 보면 arc_len 이
    // arc_floor 이상인 한 kappa_arc <= kappa_use <= kappa_grip 이 되어
    // v_cap >= v_now 가 항상 성립하고, 캡이 구조적으로 물릴 수 없었다.
    const double kappa_total = kappa_arc + std::abs(kappa_track);
    const double v_cap = std::sqrt(
        escape_arc_lat_acc_ / std::max(kappa_total, 1e-3));

    // (4) 경로: [0, arc_len] 포물선 원호 -> 장애물 통과까지 hold -> 복귀 블렌드.
    //     복귀는 평행 복귀이므로 통상 smoothstep 하한을 쓴다 (장애물을 지난
    //     뒤라 보통 거리가 넉넉하다).
    const double hold_end = arc_len +
        Mod(closest_obs.s_end - closest_obs.s_start, track_len_) +
        2.0 * vehicle_half_length_m_;
    const double blend_out = std::max(
        BlendGeomFloor(a_amp, kappa_track), lane_blend_out_m_);
    const double total = hold_end + blend_out;
    if (!PlanSpanAllowed(total)) {
        NoteReject("escape_span_over_lap");
        return false;
    }
    const int n = static_cast<int>(total / wpnt_dist_);
    if (n < 8) {
        NoteReject("escape_too_few_points");
        return false;
    }

    std::vector<double> arc_d(n);
    for (int i = 0; i < n; ++i) {
        const double t = i * wpnt_dist_;
        if (t < lead_len) {
            // 등곡률 원호 (포물선 근사): d = cur_d + k*t²/2, 기울기 k*t
            arc_d[i] = cur_d_ + 0.5 * kappa_signed * t * t;
        } else if (t < arc_len) {
            // 기울기 완화: slope 를 k*lead_len -> 0 으로 smoothstep 으로 내린다.
            //   d(u) = d(lead) + slope0*ease_len*(u - u³ + u⁴/2),  u∈[0,1]
            // ∫(1-smoothstep) 의 원함수이고, u=1 에서 0.5 가 되어 총 횡이동이
            //   0.5*k*lead² + 0.5*k*lead*ease = 0.5*k*lead*(lead+ease) = amplitude
            // 로 정확히 d_target 에 닿는다 (lead=0.4D, k=5A/D² 일 때).
            const double u = (ease_len > 1e-6)
                ? std::clamp((t - lead_len) / ease_len, 0.0, 1.0) : 1.0;
            const double slope0 = kappa_signed * lead_len;
            arc_d[i] = cur_d_ + 0.5 * kappa_signed * lead_len * lead_len
                     + slope0 * ease_len
                       * (u - u * u * u + 0.5 * u * u * u * u);
        } else if (t < hold_end) {
            arc_d[i] = d_target;
        } else {
            double w = std::clamp((t - hold_end) / std::max(blend_out, 1e-6),
                                  0.0, 1.0);
            w = w * w * (3.0 - 2.0 * w);
            arc_d[i] = d_target * (1.0 - w);
        }
    }
    // 경계 위반은 여기서 바로 포기한다 — 이 기동은 마지막 수단이라 축소/평활로
    // 살려 놓으면 정작 장애물 옆 간격이 사라진다.
    for (int i = 0; i < n; ++i) {
        const double s = Mod(cur_s_ + i * wpnt_dist_, track_len_);
        const int gi = static_cast<int>(s / wpnt_dist_) % gb_size;
        if (!WithinTrackBoundary(gi, arc_d[i], spline_bound_mindist_)) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "EscapeArc 불가: 경로가 트랙 경계 밖 (s=%.2f, d=%.2f)",
                s, arc_d[i]);
            NoteReject("escape_boundary");
            return false;
        }
    }

    out_wpnts.wpnts.reserve(n);
    for (int i = 0; i < n; ++i) {
        const double t = i * wpnt_dist_;
        const double s = Mod(cur_s_ + t, track_len_);
        const int gi = static_cast<int>(s / wpnt_dist_) % gb_size;
        const auto xy = FrenetToCartesian(s, arc_d[i]);

        auto wpnt = MakeBaseWpnt(i, s, arc_d[i], xy.first, xy.second, gi);
        // 추월 배율을 뺀 채로 있었다 (2026-08-11 이 함수가 신설될 때
        // 다른 네 블록은 3주 넘게 이미 쓰고 있었고, 여기만 빠진 이유가 주석에도
        // 없다 — 복제 표류로 판단). ot_speed_scale 이 1.0 이라 지금까지
        // 드러나지 않았을 뿐, 추월 섹터 부스트를 켜면 escape 만 다르게 움직인다.
        double vx = std::min(gb_wpnts_[gi].vx_mps, gb_vmax_);
        // 계획 속도 하한을 **그립 캡보다 먼저** 건다. 예전에는 순서가 반대라
        //   vx = min(vx, v_cap);  wpnt.vx_mps = max(vx, plan_speed_floor_);
        // 로 그립 캡을 걸어놓고 바로 다음 줄에서 1.5 m/s 로 되올렸다 — 원호가
        // 빡빡해 v_cap < plan_speed_floor_ 인 구간에서 **안전 캡이 무효**였다.
        // (escape_arc_max_heading 상한 때문에 실제로 물릴 확률은 낮았지만,
        //  안전 캡이 뒤에 오는 하한에 지워지는 구조 자체가 잘못이다.)
        vx = std::max(vx, plan_speed_floor_);
        // 원호 구간에는 그립 캡을 건다. 그 뒤는 ApplySpeedProfile 몫.
        if (t <= arc_len) {
            vx = std::min(vx, v_cap / std::max(cmd_speed_scale_, 0.05));
        }
        wpnt.vx_mps = vx;
        out_wpnts.wpnts.push_back(wpnt);
    }

    // 통상 회피와 **같은 검증기**를 통과해야 한다. 예전에는 여기만
    // require_current_feasible 면제 + 곡률 기각 밴드 확대를 걸 수 있는
    // 관측용 스위치(escape_relax_for_observation_)가 있었는데, 그 완화가
    // 겨냥한 접합부 꺾임(원호 끝 기울기 2A/L -> hold 기울기 0)은 위의 기울기
    // 완화 구간이 들어가며 없어졌다. 스위치는 배포 yaml 어디에도 없어 켜진
    // 적이 없고, 곡률 배율 쪽은 읽힌 적조차 없다 — 둘 다 제거했다.
    if (!ApplySpeedProfile(out_wpnts.wpnts)) {
        NoteReject("escape_speed_profile");
        out_wpnts.wpnts.clear();
        return false;
    }
    double blocking_s = -1.0;
    if (!CheckPathCollision(out_wpnts.wpnts, all_ahead, &blocking_s)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "EscapeArc 거부: 경로가 장애물과 충돌 (blocking s=%.2f)", blocking_s);
        NoteReject("escape_collision");
        out_wpnts.wpnts.clear();
        return false;
    }

    out_mrks.markers.clear();
    RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000,
        "EscapeArc: %.2fm 횡이동을 %.2fm(원호 %.2f + 완화 %.2f)로 이탈 "
        "(하한 %.2f, S자였다면 %.2f), kappa %.2f, 최대헤딩 %.0f도, v캡 %.1f",
        a_amp, arc_len, lead_len, ease_len,
        arc_floor, BlendGeomFloor(a_amp, kappa_track),
        kappa_arc, heading_left * 180.0 / M_PI, v_cap);
    NoteOutcome("ok_escape_arc");
    return true;
}

bool SplinerNode::TryStraightOvertake(
    const f1tenth_planning_msgs::msg::Obstacle &opp,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> &all_ahead,
    f1tenth_planning_msgs::msg::WpntArray &out_wpnts,
    visualization_msgs::msg::MarkerArray &out_mrks) {
    const int gb_size = static_cast<int>(gb_wpnts_.size());
    if (!ot_overtake_enabled_ || gb_size < 2 || !(track_len_ > 0.0)) {
        return false;
    }
    // ── ② 직선 안인가 / 코너 진입까지 남은 활주로 ──────────────────────
    const double s_corner = StraightEndAhead(cur_s_);
    if (s_corner < 0.0) {
        NoteReject("ot_not_in_straight");
        return false;
    }
    const double runway = Mod(s_corner - cur_s_, track_len_);
    if (!std::isfinite(runway) || runway <= 0.0 || runway > track_len_ * 0.5) {
        NoteReject("ot_no_runway");
        return false;
    }
    // ── ④ 상대가 사정거리 안인가 ───────────────────────────────────────
    const double gap = Mod(opp.s_start - cur_s_, track_len_);
    if (!std::isfinite(gap) || gap > ot_max_gap_m_ || gap > track_len_ * 0.5) {
        NoteReject("ot_gap_too_far");
        return false;
    }
    // ── ③ 직선 구간에 정적 장애물이 하나도 없어야 한다 ─────────────────
    // 있으면 회피가 우선이고, 추월 기동이 그 회피 라인을 먹어 버린다.
    for (const auto &o : all_ahead) {
        if (!o.is_static) {
            continue;
        }
        const double ahead = Mod(o.s_start - cur_s_, track_len_);
        if (ahead <= runway) {
            NoteReject("ot_static_in_straight");
            return false;
        }
    }
    // ── 인코스 사이드 결정 ─────────────────────────────────────────────
    // 코너 진입 직후 구간의 곡률로 안쪽을 구하고, 고정 설정과 어긋나면
    // 경고만 남긴다 (맵이 바뀐 것을 조용히 놓치지 않기 위한 것이다).
    const std::string derived = CornerInsideSide(s_corner, 4.0);
    std::string side = ot_side_;
    if (side == "auto") {
        if (derived.empty()) {
            NoteReject("ot_no_inside_side");
            return false;
        }
        side = derived;
    } else if (!derived.empty() && derived != side) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 5000,
            "직선 추월 인코스 설정(%s)이 코너 곡률 판정(%s)과 다르다 — "
            "맵/레이스라인이 바뀐 것이 아닌지 확인할 것",
            side.c_str(), derived.c_str());
    }
    const double sgn = (side == "left") ? 1.0 : -1.0;

    // ── ⑤ 오프셋: 상대 가장자리 + 요구 간격, 벽 여유로 클램프 ──────────
    const double need = RequiredObstacleClearance(opp) + apex_clearance_eps_m_;
    const double d_need = (side == "left") ? (opp.d_left + need)
                                           : (opp.d_right - need);
    // 벽 여유는 기동 전 구간(자차 ~ 코너 진입)의 최소값으로 본다.
    double wall_left = 0.0, wall_right = 0.0;
    MinWallInSpan(cur_s_, Mod(cur_s_ + runway, track_len_),
                  wall_left, wall_right);
    const double margin = RequiredBoundaryMarginAt(spline_bound_mindist_, gap);
    const double d_limit = (side == "left") ? (wall_left - margin)
                                            : -(wall_right - margin);
    const double d_hold = (side == "left") ? std::min(d_need, d_limit)
                                           : std::max(d_need, d_limit);
    // 인코스로 **실제로** 비켜야 하고, 요구 간격을 만족해야 한다.
    if (sgn * d_hold < sgn * d_need - 1e-6) {
        NoteReject("ot_wall_no_room");
        return false;
    }
    if (!(sgn * d_hold > 0.0)) {
        NoteReject("ot_offset_degenerate");
        return false;
    }

    // ── 기하: 진입 블렌드 + 유지 + 복귀 ────────────────────────────────
    const double amp_in = d_hold - cur_d_;
    const double kappa_in = BlendKappaMax(cur_s_, Mod(cur_s_ + runway, track_len_));
    const double a_lat = BlendLatAcc(cur_s_, Mod(cur_s_ + runway, track_len_));
    double v_in = PlanSpeed();
    double blend_in = 0.0;
    for (int it = 0; it < 2; ++it) {
        blend_in = BlendDist(amp_in, v_in, a_lat, kappa_in, cur_s_);
        v_in = std::max(BlendPlanSpeed(cur_s_, blend_in), PlanSpeed());
    }
    if (!std::isfinite(blend_in) || blend_in <= 0.0) {
        NoteReject("ot_blend_geom");
        return false;
    }
    // 상대 옆을 지나기 전에 횡이동이 끝나야 한다 — 상대 옆에서 옮기면
    // 그대로 접촉이다. 여유로 차 반길이를 더 뺀다.
    const double must_finish_by =
        std::max(gap - vehicle_half_length_m_, 0.0);
    if (blend_in > must_finish_by) {
        NoteReject("ot_blend_too_long");
        return false;
    }
    // 복귀는 코너 진입부터. 그 뒤 구간이 계획 범위 안에 들어와야 한다.
    const double blend_out = std::max(
        lane_blend_out_m_,
        BlendDist(d_hold, v_in, a_lat,
                  BlendKappaMax(Mod(cur_s_ + runway, track_len_),
                                Mod(cur_s_ + runway + lane_blend_out_m_,
                                    track_len_)),
                  Mod(cur_s_ + runway, track_len_)));
    const double span = runway + blend_out;
    if (!PlanSpanAllowed(span)) {
        NoteReject("ot_span");
        return false;
    }

    // ── 경로 생성 ──────────────────────────────────────────────────────
    const int n = std::max(static_cast<int>(span / wpnt_dist_), 2);
    auto d_at = [&](double t) {
        if (t <= blend_in) {
            const double w = std::clamp(t / std::max(blend_in, 1e-3), 0.0, 1.0);
            const double ss = w * w * (3.0 - 2.0 * w);   // smoothstep
            return cur_d_ + amp_in * ss;
        }
        if (t <= runway) {
            return d_hold;
        }
        const double w = std::clamp((t - runway) / std::max(blend_out, 1e-3),
                                    0.0, 1.0);
        const double ss = w * w * (3.0 - 2.0 * w);
        return d_hold * (1.0 - ss);
    };
    out_wpnts.wpnts.reserve(n);
    for (int i = 0; i < n; ++i) {
        const double t = i * wpnt_dist_;
        const double s = Mod(cur_s_ + t, track_len_);
        const int gi = static_cast<int>(s / wpnt_dist_) % gb_size;
        const double d = d_at(t);
        if (!WithinTrackBoundary(gi, d, spline_bound_mindist_)) {
            NoteReject("ot_boundary");
            out_wpnts.wpnts.clear();
            return false;
        }
        const auto xy = FrenetToCartesian(s, d);
        auto w = MakeBaseWpnt(i, s, d, xy.first, xy.second, gi);
        // 속도 캡을 걸지 않는다 = 레이트 브레이크. 물리는 아래
        // ApplySpeedProfile 이 오프셋 곡률로 다시 잡는다.
        w.vx_mps = std::min(gb_wpnts_[gi].vx_mps, gb_vmax_);
        out_wpnts.wpnts.push_back(w);
    }
    if (!ApplySpeedProfile(out_wpnts.wpnts)) {
        NoteReject("ot_speed_profile");
        out_wpnts.wpnts.clear();
        return false;
    }
    // ── 충돌 검증: 상대를 포함한 전방 전체와 (s,d,t) 검사 ───────────────
    double blocking_s = 0.0;
    if (!CheckPathCollision(out_wpnts.wpnts, all_ahead, &blocking_s)) {
        NoteReject("ot_collision");
        out_wpnts.wpnts.clear();
        return false;
    }
    for (size_t i = 0; i < out_wpnts.wpnts.size(); i += 5) {
        const auto &w = out_wpnts.wpnts[i];
        out_mrks.markers.push_back(
            MakeWpntMarker(w.x_m, w.y_m, w.vx_mps, static_cast<int>(i)));
    }
    NoteOutcome("ok_straight_overtake");
    return true;
}

bool SplinerNode::BuildTrailing(
    const f1tenth_planning_msgs::msg::Obstacle &obs,
    f1tenth_planning_msgs::msg::WpntArray &out_wpnts,
    visualization_msgs::msg::MarkerArray &out_mrks,
    bool allow_stop,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> *check_obs) {

    int gb_size = static_cast<int>(gb_wpnts_.size());
    if (gb_size < 2) {
        return false;
    }
    const bool emergency_stop =
        obs.id == std::numeric_limits<int32_t>::min();
    // 목표 속도: 동적(또는 예측) 장애물이면 그 진행 속도, 정적이면 0.
    // 제동 방정식은 실제 m/s 단위에서 계산하고 마지막에 wpnt 단위로 환산한다.
    const double command_scale = std::max(cmd_speed_scale_, 0.05);
    // ── 2026-08-18 (#05): 목표 속도에 관측 창 평균을 쓴다 ──────────────────
    // 단일 프레임 vs 는 **정지가 증명된** 트랙에서도 std 0.43~0.45 m/s 로
    // 흔들리고 순간 최대가 1.5 m/s 다 (run_0815, 3.3초에 0.59 m 움직인 콘).
    // 그 값을 목표로 쓰면 상대가 정속으로 가도 명령이 매 사이클 요동쳐
    // 가속·제동을 반복한다. 지금 있는 완충 v_corner_actual 은 **상한**이라
    // 하향 스파이크를 막지 못한다.
    // 정적은 ClassifyObstacles 가 이미 obs.vs=0 으로 만들어 두므로 영향이 없다
    // (vs_hist 는 덮어쓰기 **전** 원시값을 담으므로 여기서 태우면 안 된다).
    // 비상정지 합성 장애물(id=INT32_MIN)은 cls_tracks_ 에 없어 fallback 이다.
    // ever_moved 트랙은 정적으로 분류돼도 obs.vs(=0 강제) 대신 관측 창 평균을
    // 쓴다. allow_stop 복구와 한 묶음이다 — 그 한 줄만 바꾸면 "잠깐 느려진
    // 상대"의 목표 속도가 0 이 되어 급감속한다.
    //   run_0820_235246 실측 (이동 트랙이 STATIC 으로 분류된 443 프레임):
    //     obs.vs(강제 0)     -> 목표 0.00
    //     ObsVsMean(창 평균) -> 목표 p50 +0.42, 그때 실제 상대 p50 +0.64
    //     실제가 1.5 m/s 이상인데 창 평균이 0.3 미만인 프레임: **0 건**
    const bool obs_ever_moved = WasEverMoved(obs.id);
    const double obs_vs_eff =
        (trailing_vs_filter_ && !emergency_stop &&
         (!obs.is_static || obs_ever_moved))
            ? ObsVsMean(obs.id, obs.vs) : obs.vs;
    // 2026-08-22 (4/8단계): 동적 전용 감속도(trailing_decel_dyn_)를 없앴다.
    // 새 동적 계약은 "간격 1.5 m 유지하는 종방향 추종" 하나이고 그 제동 곡선은
    // 6단계의 follow cap 이 물리식으로 직접 만든다.
    const double decel_eff = trailing_decel_;
    double v_target_actual = std::max(obs_vs_eff, 0.0);

    // 정지 목표점: 장애물 앞 가장자리에서 목표 간격만큼 뒤.
    // 정적/비상정지는 trailing_gap_m_(오래 튜닝된 값), 동적 상대는
    // dyn_follow_gap_m_ 를 쓴다 — 두 계약은 서로 다르다.
    const double gap_target =
        (obs.is_static || emergency_stop) ? trailing_gap_m_ : dyn_follow_gap_m_;
    double span = Mod(obs.s_start - gap_target - cur_s_, track_len_);
    if (span > lookahead_m_ + 5.0) {
        span = 0.0;      // 이미 gap 안쪽까지 파고든 상태
        // ── ×0.85 는 **정적 전용** 이다 (2026-08-19) ────────────────────────
        // 정적 물체는 비켜주지 않으므로 "살짝 느리게 가서 간격을 다시 벌린다"가
        // 성립한다 — 시간을 버는 감속이다.
        //
        // 동적 상대에는 정반대로 작용한다. 상대가 우리보다 느린데 우리가 더
        // 느려지면 (a) 추월에 필요한 속도 우위가 사라지고 (b) 횡이동에 쓸
        // 예산이 줄고 (c) 상대는 계속 앞에 있으므로 상황이 바뀌지 않는다.
        // gap 안쪽이면 감속이 아니라 **회피**가 답이다.
        //   run_0818_205917 실측: B 구간 트레일링의 97% 가 gap 안쪽이었고
        //   그때 tail/obs_vs 중앙이 정확히 0.850 (gap 밖은 0.969) — 붙을수록
        //   뒤로 밀리는 락인이 측정으로 확인됐다.
        if (obs.is_static) {
            v_target_actual *= 0.85;
        }
    }

    // 상대는 코너에서 감속한다 — 지금 측정 속도로만 프로파일을 짜면 상대가
    // 헤어핀 진입 중 감속할 때 gap이 무너져 상대를 관통한다 (실측).
    // 상대도 코너 한계속도(raceline 속도 프로파일) 이상으로는 못 돈다고
    // 가정하고, 정지점~장애물 구간 raceline 최저 속도를 목표 속도 상한으로.
    // (raceline 대비 비율 유지 모델은 정속 주행 상대를 과잉 감속 예측해
    //  ego가 상대 절반 속도로 기어가는 부작용 — 실측 폐기)
    double v_target_prof_actual = v_target_actual;
    if (obs_vs_eff > 0.1) {
        int stop_gi = static_cast<int>(
            Mod(obs.s_start - gap_target, track_len_) / wpnt_dist_) % gb_size;
        int n_ahead = static_cast<int>(
            (gap_target + 5.0) / std::max(wpnt_dist_, 0.05));
        double v_corner_actual = gb_wpnts_[stop_gi].vx_mps * command_scale;
        for (int i = 1; i <= n_ahead; ++i) {
            v_corner_actual = std::min(
                v_corner_actual,
                gb_wpnts_[(stop_gi + i) % gb_size].vx_mps * command_scale);
        }
        v_target_prof_actual = std::min(v_target_actual, v_corner_actual);
    }
    // 기하 하한 초과가 아닌(=일시적) 실패로 들어온 트레일링은 정지가 아니라
    // 감속이다. 정지 목표를 그대로 두면 0을 명령하고, 그 0이 MAP 컨트롤러의
    // 조향까지 0으로 만든다 (control_node.cpp:1057).
    if (!allow_stop) {
        v_target_prof_actual = std::max(v_target_prof_actual,
                                        trailing_retry_speed_);
    }

    // 물리적으로 도달 가능한 최소 속도 곡선. 현재 속도 v0에서 지연 T 뒤부터
    // trailing_ramp_decel_로 감속했을 때 전방 dist 지점에서의 속도다.
    // 이 곡선 아래로는 어차피 못 내려가므로 명령해도 소용이 없고, 대신
    // "전 점 vx=0"이라는 계단이 만들어져 조향이 죽는다 — 0807 5런 실측
    // (164645: 4.25 -> 0이 단일 사이클 24ms, 전 점 0이 13.9초 누적).
    const double v0 = std::max(std::abs(cur_vs_), 0.0);
    const double ramp_a = std::max(trailing_ramp_decel_, 0.05);
    auto reachable_v = [&](double dist) {
        const double eff = std::max(dist - v0 * trailing_lag_sec_, 0.0);
        return std::sqrt(std::max(v0 * v0 - 2.0 * ramp_a * eff, 0.0));
    };

    // 정지점 뒤로도 v_target 구간을 연장한다 — 컨트롤러 lookahead(수 m)가
    // 궤적 밖의 원본 raceline 속도(전속)를 집어 재가속하는 것을 방지
    const double extend = 5.0;
    // 회피 중(|cur_d| 큼) 폴백이 걸리면 경로가 d=0으로 순간이동해 차가
    // 장애물 쪽으로 끌려간다 — 현재 d에서 목표 d(아래 d_goal)로 가는 램프.
    // 길이는 고정 3m이 아니라 그립 예산으로: 6 m/s에서 d=0.4를 3m에 되돌리면
    // 1.0g를 요구한다.
    // 램프의 종착 d. 기본은 raceline 복귀(0)지만, 회피가 **일시적으로**
    // 실패해 재시도 중이면(= allow_stop이 false) 직전에 노렸던 회피 목표를
    // 유지한다. 되돌리면 다음 사이클의 횡이동 잔량이 그대로라 회피가 영원히
    // 성립하지 않는다 — 근거는 retry_d_target_ 주석(run_0809_231809 4랩).
    double d_goal = 0.0;
    // 2026-08-13: allow_stop 을 여기서 뺀다. allow_stop 은 **속도** 결정
    // (정지까지 갈 것인가 / trailing_retry_speed 로 버틸 것인가)이지 횡 목표
    // 결정이 아니다. 예전에는 geom_blocked(= 조향 기하 하한 초과)가 서면
    // allow_stop 이 켜지면서 d_goal 까지 0(raceline 복귀)으로 떨어졌는데,
    // **기하가 모자란 상황이야말로 횡 진행을 남겨야 하는 경우다** — 되돌리면
    // 다음 사이클의 횡 잔량이 그대로라 "못 하니까 안 한다 -> 안 했으니까 계속
    // 못 한다"는 교착이 된다 (바로 위 retry_d_target_ 주석과 같은 근거).
    //   my_track0811 s=0.5 정적: 첫 탐지 5.48m/6.35m/s 에서 필요 진입 5.43m,
    //   가용 5.19m 로 4.6% 부족 -> geom_floor -> d_goal=0 -> 다음 사이클도
    //   진폭 0.83m 그대로. 랩마다 같은 자리에서 같은 실패를 반복했다.
    // 정지 보장은 아래 속도 프로파일이 그대로 한다. 정지하더라도 장애물
    // s_start **뒤에서** 서므로 그 시점의 횡 위치는 안전과 무관하고, 경계
    // 침범은 아래 ramp_ok 가 종전대로 막는다.
    // 2026-08-22 (4/8단계): 유지는 **정적 전용** 으로 되돌렸다. 회피가 정적
    // 전용이 됐으므로 retry_d 를 기록하는 것도 정적뿐이고, 동적 상대의 횡
    // 목표라는 개념 자체가 없어졌다 (동적은 종방향 추종만 한다).
    if (!emergency_stop && has_retry_d_ && obs.is_static) {
        const double age = (this->now() - retry_d_stamp_).seconds();
        if (age >= 0.0 && age <= kRetryDHoldSec) {
            d_goal = retry_d_target_;
        }
    }
    const double planned_span = std::min(
        span + extend, std::max(track_len_ - wpnt_dist_, wpnt_dist_));
    int n = std::max(static_cast<int>(planned_span / wpnt_dist_), 2);
    // 램프 길이는 실제 이동량(|d_goal - cur_d_|)의 그립 예산으로 정한다.
    auto ramp_len_for = [&](double goal) {
        return std::max(1.0, BlendDist(goal - cur_d_, cur_vs_));
    };
    // ── 오프셋 유지 구간 ──────────────────────────────────────────────
    // 2026-08-13: 회피 오프셋은 **장애물 뒤 0.5m 까지만** 유지하고 그 뒤로는
    // raceline 으로 복귀한다. 예전에는 d_goal 을 경로 끝(장애물 + extend 5m)
    // 까지 그대로 들고 갔다 — t = min(dist/len, 1) 이라 램프가 끝나면 d_goal
    // 이 상수로 남는다. 그 5m 하류가 좁아지는 구간이면 오프셋이 **필요하지도
    // 않은 곳**에서 WithinTrackBoundary 가 깨지고, 그 한 점 때문에 ramp_ok 가
    // false 를 돌려 d_goal 이 통째로 0(raceline 복귀)으로 내려갔다.
    //   my_track0811: 장애물 s=0.5, 유지 오프셋 +0.5 를 s=4~5 까지 끌고 가는데
    //   그 구간 d_left 가 0.82 -> 0.45 로 좁아진다. 실측 로그의 트레일링이
    //   v_target 과 무관하게 **전부** "d -0.2x -> 0.00" 이었던 이유가 이것이다.
    // 회피 목표를 유지하라고 만든 retry_d_target_ 이 이 한 줄 때문에 한 번도
    // 발현되지 않았다.
    const double hold_until = Mod(obs.s_end - cur_s_, track_len_) + 0.5;
    auto back_len_for = [&](double goal) {
        return std::max(1.0, BlendDist(goal, cur_vs_));
    };
    // dist 지점의 d: [진입 램프] -> [장애물 통과까지 유지] -> [raceline 복귀].
    auto d_profile = [&](double dist, double goal, double len, double back) {
        if (dist <= len) {
            const double t = std::min(dist / std::max(len, 1e-3), 1.0);
            return cur_d_ + (goal - cur_d_) * t;
        }
        if (dist <= hold_until || std::abs(goal) <= 0.02) {
            return goal;
        }
        const double t = std::min((dist - hold_until) / std::max(back, 1e-3), 1.0);
        return goal * (1.0 - t);
    };
    // 경계 판정 — **트레일링 전용**으로 "회랑 안인가" 대신 "지금보다 나빠지지
    // 않는가"를 본다. 자차의 현재 횡위치는 플래너가 이번 사이클에 바꿀 수 없는
    // **입력**인데, 종전 판정은 그것을 합격 조건에 넣었다. cur_d_ 가 회랑 밖으로
    // 1mm 라도 나가 있으면 d_profile(0) = cur_d_ 이므로 **첫 점**에서 떨어지고,
    // 그러면 목표를 아무리 줄여도(심지어 raceline 복귀조차) 같은 점에서 같은
    // 이유로 떨어져 폴백 사다리가 끝까지 무너진다 = 회피 의도 소멸.
    //   run_0817_124904 t=59.35~59.92 실측: 이 구간 d_right 가 웨이포인트마다
    //   0.45/0.47/0.49/0.52 로 튄다(맵 5cm 격자). 마진 0.20 이라 허용 하한이
    //   -0.245 ~ -0.320 사이에서 흔들리는데 차는 d=-0.20~-0.27 로 그 위를 탔다.
    //   t=59.38 은 2mm, t=59.45 는 29mm 초과. 그 1초의 트레일링 19 사이클이
    //   **전부** d_goal_zeroed -> hold_current_d 로 떨어져 0.6초 동안 횡이동이
    //   정확히 0 이었고(발행 d 가 49점 전부 cur_d_ 상수), 4.3 m/s 로 정면충돌.
    // 불변식은 유지된다 — 경로는 자차가 이미 나가 있는 쪽으로 **더** 나가지
    // 않는다. 회랑 자체가 소멸한 지점(d_min > d_max)은 종전대로 거부한다.
    // ⚠️ 트레일링(마지막 티어)에만 적용한다. Tier1/2 는 아직 대안이 있으므로
    //    엄격한 WithinTrackBoundary 를 그대로 쓴다.
    auto boundary_ok_from_here = [&](int gi, double d) {
        if (WithinTrackBoundary(gi, d, spline_bound_mindist_)) {
            return true;
        }
        const double margin = RequiredBoundaryMargin(spline_bound_mindist_);
        const double d_min = -std::abs(gb_wpnts_[gi].d_right) + margin;
        const double d_max = std::abs(gb_wpnts_[gi].d_left) - margin;
        if (!(d_min <= d_max) || !std::isfinite(d)) {
            return false;
        }
        if (d < d_min) {
            return cur_d_ < d_min && d >= cur_d_;
        }
        if (d > d_max) {
            return cur_d_ > d_max && d <= cur_d_;
        }
        return false;
    };
    // 램프 전 구간이 트랙 경계 안인가. 실패하면 구속 지점을 돌려준다
    // (bind_s / bind_d / bind_limit) — 어느 점이 물었는지가 로그에 없어서
    // run_0813_165312 분석에서 이 게이트의 인과를 재구성으로만 세울 수 있었다.
    auto ramp_ok = [&](double goal, double len, double *bind_s = nullptr,
                       double *bind_d = nullptr, double *bind_limit = nullptr) {
        const double back = back_len_for(goal);
        for (int i = 0; i < n; ++i) {
            const double dist = i * wpnt_dist_;
            const double s = Mod(cur_s_ + dist, track_len_);
            const int gi = static_cast<int>(s / wpnt_dist_) % gb_size;
            const double d = d_profile(dist, goal, len, back);
            if (!boundary_ok_from_here(gi, d)) {
                if (bind_s != nullptr) {
                    *bind_s = s;
                    *bind_d = d;
                    const double margin =
                        RequiredBoundaryMargin(spline_bound_mindist_);
                    *bind_limit = d >= 0.0
                        ? std::abs(gb_wpnts_[gi].d_left) - margin
                        : -(std::abs(gb_wpnts_[gi].d_right) - margin);
                }
                return false;
            }
        }
        return true;
    };
    // ── A (2026-08-19): 횡 램프를 장애물과도 대조한다 ──────────────────────
    // 위 ramp_ok 는 **벽만** 본다. 그래서 목표가 장애물 반대편일 때 벽 검사만
    // 통과하면 아래 되감기 사다리가 아예 돌지 않고, 장애물 정중앙을 겨눈
    // 램프가 그대로 발행됐다.
    //   run_0819_180345 t=267.05 실측: 자차 d=+0.78 에서 목표 -0.07 로 램프 ->
    //   장애물(s[0.21,0.55] d[0.21,0.55]) 지점의 경로 d = +0.45 = 박스 정중앙.
    //   0.65 초 뒤 4.0 m/s 로 정면충돌. 근거리(3 m 이내) 발행경로 2224 건 중
    //   120 건(5.4%)이 **발행 시점에 이미 관통**이었고 그중 89%가 트레일링이다.
    // 충돌 자체는 아래 check_obs 분기가 이미 감지하고 있었다 — 그런데 대응이
    // 속도 프로파일뿐이었고, 그 분기의 주석이 스스로 적어 두었듯 발동 거리
    // (0.2~5.6 m)에서는 정지 곡선이 되지 못한다. 즉 "관통하는 것을 알면서
    // 발행하고, 못 세우는 브레이크에 안전을 맡기는" 구조였다.
    //
    // 후보마다 경로를 지어 CheckPathCollision 을 부르면 이분 탐색 12회 x
    // 장애물수 만큼 든다. 여기서는 같은 판정을 Frenet 평면에서 바로 잰다 —
    // 요구 간격은 검증기와 같은 RequiredObstacleClearance 이고, 종방향 박스도
    // 검증기와 같이 차 반길이만큼 넓힌다. 양수 = 그 간격을 지키고 지나간다.
    // strict=true  -> 검증기와 같은 계획 마진(RequiredObstacleClearance, ~0.27).
    //                  되감기가 **겨누는 목표**다.
    // strict=false -> 차 반폭(0.15). 실제로 차체가 닿는가 = **개입 문턱**이다.
    // 문턱과 목표를 나눈 이유: 같은 백에서 계획 마진만 침범한(차체는 안 닿는)
    // 경로가 664 사이클 중 363(55%)이었다. 그것까지 손대면 트레일링의 55%에서
    // 속도 바닥이 2.5 -> 1.5 m/s 로 내려간다 — 대가가 이득보다 크다.
    // 문턱을 차체로 좁히면 개입은 301 사이클(트레일링의 25%)로 줄고, 정작
    // 고쳐야 할 "실제로 닿던 경로"는 그 301 이 전부다.
    auto ramp_obstacle_slack = [&](double goal, double len, double back,
                                   bool strict) {
        double worst = std::numeric_limits<double>::infinity();
        if (check_obs == nullptr) {
            return worst;
        }
        for (const auto &o : *check_obs) {
            if (!std::isfinite(o.s_start) || !std::isfinite(o.s_end) ||
                !std::isfinite(o.d_left) || !std::isfinite(o.d_right)) {
                continue;
            }
            // 자차 기준 상대거리로 unwrap — 뒤에 있는 장애물은 ~track_len 이
            // 되어 아래 사정권 비교에서 자연히 걸러진다 (HoldClearsOthers 와
            // 같은 규약).
            const double lo_s = Mod(o.s_start - cur_s_, track_len_) -
                                vehicle_half_length_m_;
            const double hi_s = lo_s + Mod(o.s_end - o.s_start, track_len_) +
                                2.0 * vehicle_half_length_m_;
            if (lo_s > planned_span) {
                continue;               // 이 경로가 닿지 않는 곳
            }
            const double need =
                strict ? RequiredObstacleClearance(o) : vehicle_half_width_m_;
            for (int i = 0; i < n; ++i) {
                const double dist = i * wpnt_dist_;
                if (dist < lo_s || dist > hi_s) {
                    continue;
                }
                const double d = d_profile(dist, goal, len, back);
                worst = std::min(
                    worst,
                    std::max(o.d_right - need - d, d - (o.d_left + need)));
            }
        }
        return worst;
    };
    // ── 후보가 "양옆이 막힌 주머니" 안인가 (2026-08-22, run_0822_144702) ────
    // ramp_obstacle_slack 은 **가장 가까운 한 면**까지의 거리만 잰다. 그래서
    // 차체가 절대 못 들어가는 틈의 정중앙이 "여유가 가장 큰 후보"로 뽑힐 수
    // 있다 — t=23.32 에서 정확히 그랬다 (두 박스 사이 4 cm 틈, 여유 -0.160 m
    // 인데 그 값이 후보 중 최대였다).
    // 이 함수는 램프 위 각 점에서 **위/아래 양쪽 모두** 장애물이 있는 경우의
    // 통로 폭을 재서 가장 좁은 값을 돌려준다. 양쪽이 다 막힌 적이 없으면
    // +inf 다 (= 판단하지 않음).
    //
    // ⚠️ 박스 **안**에 있는 점은 폭 계산에서 뺀다. 그건 주머니가 아니라 접촉이고,
    //    그 경우 "여유가 큰 후보로 옮긴다"는 run_0819_180345 실측(유지하면
    //    차체 접촉 120 -> 232, 옮기면 105)이 그대로 유효하다. 이 가드는 그
    //    경로를 건드리지 않고 **주머니만** 걸러낸다.
    auto ramp_pocket_width = [&](double goal, double len, double back) {
        double narrowest = std::numeric_limits<double>::infinity();
        if (check_obs == nullptr) {
            return narrowest;
        }
        for (int i = 0; i < n; ++i) {
            const double dist = i * wpnt_dist_;
            const double d = d_profile(dist, goal, len, back);
            double above = std::numeric_limits<double>::infinity();
            double below = -std::numeric_limits<double>::infinity();
            bool in_box = false;
            for (const auto &o : *check_obs) {
                if (!std::isfinite(o.s_start) || !std::isfinite(o.s_end) ||
                    !std::isfinite(o.d_left) || !std::isfinite(o.d_right)) {
                    continue;
                }
                const double lo_s = Mod(o.s_start - cur_s_, track_len_) -
                                    vehicle_half_length_m_;
                const double hi_s = lo_s +
                                    Mod(o.s_end - o.s_start, track_len_) +
                                    2.0 * vehicle_half_length_m_;
                if (lo_s > planned_span || dist < lo_s || dist > hi_s) {
                    continue;
                }
                if (o.d_right > d) {
                    above = std::min(above, o.d_right);
                } else if (o.d_left < d) {
                    below = std::max(below, o.d_left);
                } else {
                    in_box = true;
                    break;
                }
            }
            if (in_box || !std::isfinite(above) || !std::isfinite(below)) {
                continue;
            }
            narrowest = std::min(narrowest, above - below);
        }
        return narrowest;
    };
    // 벽 **과** 장애물을 모두 통과해야 성립이다. 어느 쪽이 물었는지는 로그에
    // 반드시 남긴다 — ramp_ok 의 bind_* 가 도입된 이유가 정확히 "어느 점이
    // 물었는지가 로그에 없어서 인과를 재구성으로만 세울 수 있었다" 였고,
    // 벽/장애물 구분 없이 bind_* 만 찍으면 장애물 구속일 때 s=0 d=0 한계=0 이
    // 나가 같은 실패를 되풀이한다.
    bool bind_obstacle = false;
    double bind_slack = 0.0;
    // 장애물 마진을 사다리의 합격 조건에 넣는 것은 **이번 사이클에 개입하기로
    // 정한 경우**뿐이다. 벽 때문에 사다리가 도는 사이클(= 예전부터 돌던 것)의
    // 합격 조건은 예전 그대로 벽 하나여야 한다 — 아니면 손댈 이유가 없던
    // 사이클까지 목표가 깎이고 속도 바닥이 내려간다.
    bool require_obstacle_margin = false;
    auto ramp_clear = [&](double goal, double len, double *bind_s = nullptr,
                          double *bind_d = nullptr, double *bind_limit = nullptr) {
        if (!ramp_ok(goal, len, bind_s, bind_d, bind_limit)) {
            return false;
        }
        if (!require_obstacle_margin) {
            return true;
        }
        const double slack =
            ramp_obstacle_slack(goal, len, back_len_for(goal), true);
        if (slack < 0.0) {
            // 진단 인자를 요구한 호출(= 바깥 1회)에서만 기록한다. 이분 탐색의
            // 내부 호출까지 쓰면 마지막 후보 값으로 덮여 원인이 흐려진다.
            if (bind_s != nullptr) {
                bind_obstacle = true;
                bind_slack = slack;
            }
            return false;
        }
        return true;
    };
    double d_ramp_len = ramp_len_for(d_goal);
    double d_back_len = back_len_for(d_goal);
    bool hold_current_d = emergency_stop;
    double bind_s = 0.0, bind_d = 0.0, bind_limit = 0.0;
    // 개입 문턱은 **차체 접촉**이다 (위 ramp_obstacle_slack 주석의 301/664).
    const bool wall_bad = !hold_current_d &&
        !ramp_ok(d_goal, d_ramp_len, &bind_s, &bind_d, &bind_limit);
    const bool body_bad = !hold_current_d && trailing_ramp_obstacle_check_ &&
        ramp_obstacle_slack(d_goal, d_ramp_len, d_back_len, false) < 0.0;
    require_obstacle_margin = body_bad;
    if (wall_bad || body_bad)
    {
        // ── 성립하는 최대 오프셋 탐색 (2026-08-13) ──────────────────────
        // 예전에는 여기서 곧장 d_goal = 0 (raceline 복귀)이었다. 그런데
        // ramp_ok 는 [진입 램프 -> 유지 -> 복귀 블렌드] 프로파일 **전 구간**을
        // 훑고 **한 점이라도** 경계를 넘으면 false 다. 복귀 블렌드는 장애물보다
        // 한참 하류까지 뻗고 그 길이는 속도에 비례하므로, 장애물이 멀수록 꼬리가
        // 길어져 좁은 구간에 닿는다 — 즉 **시간이 남아 있을수록 회피 의도가
        // 지워진다.** run_0813_165312 실측: 트레일링 28건 중 gap>=1.7m 인
        // 10건에서 9건이 d_goal=0 이 됐고(gap<=1.0m 14건은 14건 전부 유지),
        // S/F 접촉 5건이 여기서 나왔다. 재구성상 구속 지점은 7m 하류
        // s≈4.4 의 한 점이고 여유가 5mm 였다.
        //
        // 전부-아니면-전무를 없앤다: 성립하는 **최대** 오프셋을 이분 탐색으로
        // 찾는다. 검사 자체는 종전대로 전 구간을 그대로 본다 — 경로 생성
        // 루프(아래)가 같은 경계 검사로 한 점이라도 어긋나면 경로를 통째로
        // 버리므로, 검사 범위를 줄이면 "오프셋 축소"가 "경로 없음"이 된다.
        //
        // ── 축소의 종점은 raceline 이 아니라 cur_d_ 다 (2026-08-17) ────────
        // 종전 구간은 [0, full_goal] 이었다. 즉 "회피가 안 되면 회피량을 줄인다"
        // 의 극한이 **d=0 (raceline)** 이다. 그런데 이 함수가 불리는 상황은
        // 정의상 raceline 위에 장애물이 있는 상황이라, 그 종점은 안전한 곳이
        // 아니라 **제일 위험한 곳**이다.
        //   run_0817_124904 t=59.35: 장애물 #185 d[-0.35,+0.32], 자차 d=-0.25,
        //   회피 목표 +0.577. 축소가 0 으로 수렴하면 장애물 정중앙을 겨눈다.
        //   (그 런에서는 Fix A 이전이라 첫 점에서 떨어져 이 결함이 발현될
        //    기회조차 없었고 곧장 hold_current_d 로 갔다 — 잠재 결함이었다.)
        // 구간을 [cur_d_, full_goal] 로 바꾼다. 그러면
        //   · lo=0  -> cand = cur_d_  = "지금 위치 유지" (최악이 여기까지)
        //   · lo=1  -> cand = full_goal = 회피 목표 전량
        // 이고, 성립하는 만큼은 **항상 회피 방향으로** 간다. 되돌아오는 방향은
        // 후보에 아예 없다. 차 반폭이 0.15m 라 0.15~0.2m 만 옮겨도 정면 접촉이
        // 스침으로 바뀐다 — 위 사고에서 가용했던 값은 0.32m 였다.
        const double full_goal = d_goal;
        double lo = 0.0;          // 항상 성립한다고 가정하지 않는다 (아래 재확인)
        double hi = 1.0;          // 성립하지 않음이 위에서 확인된 배율
        auto goal_at = [&](double f) { return cur_d_ + (full_goal - cur_d_) * f; };
        for (int it = 0; it < 12; ++it) {
            const double mid = 0.5 * (lo + hi);
            const double cand = goal_at(mid);
            if (ramp_clear(cand, ramp_len_for(cand))) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        d_goal = goal_at(lo);
        // 수 cm 미만의 **진행량**은 유지할 값어치가 없다. 기준은 0 이 아니라
        // cur_d_ 다 — 종전처럼 |d_goal|<=0.02 로 재면 cur_d_ 근처에서 회피
        // 목표가 raceline 으로 스냅돼 위 구간 변경이 무효가 된다.
        const bool made_progress = std::abs(d_goal - cur_d_) > 0.02;
        if (!made_progress) {
            d_goal = cur_d_;
        }
        d_ramp_len = ramp_len_for(d_goal);
        d_back_len = back_len_for(d_goal);
        if (made_progress) {
            if (bind_obstacle) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000,
                    "Trailing 횡목표 축소: %.2f -> %.2f "
                    "(장애물 구속, 원목표 여유 %.3f m)",
                    full_goal, d_goal, bind_slack);
            } else {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000,
                    "Trailing 횡목표 축소: %.2f -> %.2f "
                    "(벽 구속 s=%.2f d=%.2f 한계 %.2f)",
                    full_goal, d_goal, bind_s, bind_d, bind_limit);
            }
        } else {
            if (bind_obstacle) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000,
                    "Trailing 횡목표 포기: %.2f -> 현재 %.2f 유지 "
                    "(장애물 구속, 원목표 여유 %.3f m)",
                    full_goal, cur_d_, bind_slack);
            } else {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000,
                    "Trailing 횡목표 포기: %.2f -> 현재 %.2f 유지 "
                    "(벽 구속 s=%.2f d=%.2f 한계 %.2f)",
                    full_goal, cur_d_, bind_s, bind_d, bind_limit);
            }
        }
        NoteReject(made_progress ? "trailing_d_goal_shrunk"
                                 : "trailing_d_goal_zeroed");
        // ── ★ 되감기의 바닥은 "현재 자리"가 아니다 (2026-08-19) ──────────────
        // 위 사다리는 lo=0 = cur_d_ 를 "최악이 여기까지"인 안전 바닥으로 놓고
        // 설계됐다. **벽에 대해서는** 맞다 — 경로는 자차가 이미 나가 있는 쪽으로
        // 더 나가지 않는다. 그런데 **장애물에 대해서는 틀리다**: 지금 있는 자리가
        // 곧 장애물 정면일 수 있다.
        //   run_0819_180345 재구성: 이 한 줄을 그대로 둔 채 위 장애물 검사만
        //   붙이면 차체 접촉 경로가 120 -> 232 건으로 **두 배**가 된다
        //   (근거리 3 m 이내 발행경로 2224 건 기준). 반대로 여기서 여유가 큰
        //   후보를 고르면 105 건으로 줄고, 여유가 나빠지는 프레임은 0 이다.
        // 그래서 후보가 전부 성립하지 않을 때는 "유지"가 아니라 **벽을 지키는
        // 후보 중 여유가 가장 큰 쪽**을 고른다. 벽 불변식(ramp_ok)은 그대로
        // 요구하므로 이 구제가 차를 트랙 밖으로 내보내는 일은 없다.
        // 이 스캔은 사다리가 실패하고 진행량이 0 인 사이클에서만 돈다.
        if (!made_progress && trailing_ramp_obstacle_check_) {
            // 발동 문턱도 최대화 대상도 **차체 여유**다. 계획 마진으로 재면
            // 손댈 이유가 없는 자리에서까지 옮기게 된다.
            double best_slack = ramp_obstacle_slack(
                d_goal, d_ramp_len, d_back_len, false);
            if (best_slack < 0.0) {
                double best_f = 0.0;
                // 이분 탐색이 아니라 균등 스캔이다 — 여유는 후보 f 에 대해
                // 단조가 아니다 (장애물 박스를 가로지르면 양끝이 크고 가운데가
                // 작다). 단조 가정으로 이분하면 정확히 그 가운데로 수렴한다.
                for (int it = 1; it <= 10; ++it) {
                    const double f = 0.1 * static_cast<double>(it);
                    const double cand = goal_at(f);
                    const double len = ramp_len_for(cand);
                    if (!ramp_ok(cand, len)) {
                        continue;       // 벽 불변식은 그대로 지킨다
                    }
                    // 차체가 못 들어가는 주머니는 여유가 아무리 "덜 나빠도"
                    // 후보가 아니다 — 선언부 ramp_pocket_width 주석 참조.
                    if (ramp_pocket_width(cand, len, back_len_for(cand)) <
                        2.0 * vehicle_half_width_m_) {
                        NoteReject("trailing_cand_in_pocket");
                        continue;
                    }
                    const double slack =
                        ramp_obstacle_slack(cand, len, back_len_for(cand), false);
                    if (slack > best_slack) {
                        best_slack = slack;
                        best_f = f;
                    }
                }
                if (best_f > 0.0) {
                    const double held = d_goal;
                    d_goal = goal_at(best_f);
                    d_ramp_len = ramp_len_for(d_goal);
                    d_back_len = back_len_for(d_goal);
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000,
                        "Trailing 유지가 장애물과 접촉 — 차체 여유 최대 후보로 "
                        "이동 (d %.2f -> %.2f, 차체 여유 %.3f m)",
                        held, d_goal, best_slack);
                    NoteReject("trailing_hold_pierces_obstacle");
                }
            }
        }
        // ── 횡 계획이 깎였으면 속도 바닥도 낮춘다 (2026-08-15) ─────────────
        // trailing_retry_speed_ 는 "정적 장애물을 못 피했을 때 그 속도로
        // 접촉한다"는 뜻이다 (allow_stop = !is_static 이라 정적에는 완전
        // 정지 경로가 없다 — 위 1176줄 트레이드 주석).
        // 그 값을 올리면 BlendDist = v·sqrt(6A/a_lat) 가 **속도에 정비례**해
        // 커지므로 횡 램프가 안 들어가고, 여기(횡목표 축소/포기)로 떨어지는
        // 빈도가 급증한다. run_0814_191443 실측:
        //     v<1.5  포기 0.0~0.2%   v1.5~2.5  9.7~10.0%   v>3.0  31.4%
        // 즉 바닥을 올리면 "회피를 포기하고 직진"이 같이 늘어난다 — 7:23
        // 접촉(444.27, 여유 0.000)을 만든 바로 그 분기다.
        // 그래서 **횡 계획이 온전할 때만** 높은 바닥을 쓰고, 목표가 깎이면
        // 종전 값으로 되돌린다. 빠르게 가는 것은 실제로 비켜 가고 있을
        // 때뿐이라는 뜻이다.
        v_target_prof_actual = std::min(v_target_prof_actual,
                                        std::max(trailing_retry_speed_degraded_,
                                                 v_target_actual));
        // 2026-08-17: 조건이 `d_goal == 0.0 && !ramp_ok(0.0, ...)` 이었다.
        // 이분 탐색의 종점이 cur_d_ 로 바뀌었으므로 (위 주석) 그 리터럴 비교는
        // 더 이상 성립하지 않는다 — 진행량 0 이고 그 자리 유지 기하마저 안 될
        // 때로 옮긴다. 아래 3843줄 "차가 경계 밖에 나가 램프가 검사를 못 넘는"
        // 사례는 이제 boundary_ok_from_here 가 앞단에서 흡수하므로, 여기로
        // 들어오는 것은 회랑 자체가 소멸한(d_min > d_max) 진짜 막다른 경우다.
        if (!made_progress && !ramp_ok(d_goal, d_ramp_len)) {
            // 안전한 유지 기하조차 없으면 빈 경로 대신 현재 d를
            // 유지한다. 빈 경로는 곧 전속 global path이므로 금지.
            hold_current_d = true;
            // ⚠️ 목표 속도는 allow_stop 을 따른다 (2026-08-14).
            // 이 줄은 d809831("정적 장애물의 정지를 폐지한다") **이전부터**
            // 있었고 그 커밋이 손대지 않았다 — 마무리 누락이다. 위 3242줄이
            // !allow_stop 일 때 trailing_retry_speed_ 바닥을 깔아 놓는데,
            // 여기서 무조건 0 으로 덮어써 정적 장애물이 다시 정지한다.
            //   run_0813_221645 실측: 네모코너(s 6.6) 콘 통과 14회 중 9회가
            //   1.5 m/s 아래로 내려갔고(최저 0.33), 랩 14 에서는 콘 앞
            //   0.03 m 에 0.38 m/s 로 섰다. 그 뒤 콘이 밀려나 사라졌다.
            //   이 분기로 들어오는 경로는 횡목표 포기 472 사이클 중 426 이
            //   s 5~8 — 차가 언더스티어로 경계 밖(d -0.45~-0.72)에 나가
            //   "지금 자리에서 시작하는 램프"가 경계 검사를 못 넘는 경우다.
            // 정지는 그 수렴을 스스로 끊는다는 d809831 의 논리가 여기에도
            // 그대로 적용된다. 동적/비상정지(allow_stop=true)는 종전대로 0.
            // 여기는 "복귀 기하조차 없다" = 가장 나쁜 상태이므로 낮은 바닥.
            //
            // ⚠️ ever_moved 트랙(달리다 멈춘 상대)은 allow_stop 이 열려 이
            //    분기로 들어오는데, 그때 0 을 그대로 명령하면 **아직 굴러가는
            //    상대 뒤에서 급정거**한다. 정적으로 분류돼 있어도 관측 창
            //    평균(obs_vs_eff)이 상대 속도를 들고 있으므로 그것을 쓴다.
            //    진짜로 멈춰 있으면 창 평균도 0 이라 결과가 같다.
            v_target_prof_actual =
                allow_stop ? (obs.is_static && obs_ever_moved
                                  ? std::max(obs_vs_eff, 0.0) : 0.0)
                           : trailing_retry_speed_degraded_;
        }
    }
    out_wpnts.wpnts.reserve(n);
    for (int i = 0; i < n; ++i) {
        double dist = i * wpnt_dist_;
        double s = Mod(cur_s_ + dist, track_len_);
        int gi = static_cast<int>(s / wpnt_dist_) % gb_size;
        f1tenth_planning_msgs::msg::Wpnt w = gb_wpnts_[gi];
        w.id = i;
        // gb에서 통째로 복사했으므로 s_m도 gb[gi]의 값이다 — 그대로 두면
        // 랩 경계에서 s가 중복돼 경로가 비단조가 된다. gb는 s<=back_s까지만
        // 있고 track_len_ = back_s + wpnt_dist_이므로 마지막 폐합 구간의 s는
        // 대응 gb 점이 없고, 게다가 int(s/wpnt_dist_)가 gb_size로 넘치면
        // %gb_size가 그것을 0으로 접어 바로 앞 점과 같은 gb를 가리킨다.
        // 실측(run_0803_213313): s[10]=s[11]=0.00000, dxy=0.0955 -> local_
        // trajectory가 ds=track_len 점프로 판정해 궤적을 통째로 거부하고
        // /local_waypoints 발행을 멈췄다(= 차 정지). 다른 경로 생성 루프들은
        // 모두 wpnt.s_m = s로 계산값을 쓴다 — 여기만 어긋나 있었다.
        w.s_m = s;
        // perception stale 비상정지는 알 수 없는 raceline 쪽으로 횡이동하지
        // 않고 현재 lateral line을 유지한다.
        double d;
        if (hold_current_d) {
            d = cur_d_;
        } else {
            // ramp_ok 가 검증한 것과 **같은** 프로파일을 쓴다.
            d = d_profile(dist, d_goal, d_ramp_len, d_back_len);
        }
        w.d_m = d;
        if (std::abs(d) > 0.02) {
            // ramp_ok 와 **같은** 판정이어야 한다. 달라지면 ramp_ok 가 통과시킨
            // 프로파일을 여기서 통째로 버려 "오프셋 축소"가 "경로 없음"이 된다
            // (= 전속 global path). boundary_ok_from_here 주석 참조.
            if (!hold_current_d && !boundary_ok_from_here(gi, d)) {
                out_wpnts.wpnts.clear();
                return false;
            }
            auto [x, y] = FrenetToCartesian(s, d);
            w.x_m = x;
            w.y_m = y;
            w.d_left -= d;
            w.d_right += d;
        }
        // 감속 프로파일: 정지점 앞 engage 이내에서만 감속 시작 — 감지
        // 즉시(20m 밖)부터 감속하면 접근 내내 느려져 못 따라잡는다.
        // engage는 현재 속도 비례 하한 포함: gym 1s 시정수 + 컨트롤러
        // lookahead(~3m) 때문에 실속 7.5에선 12m 고정으로는 못 서고
        // gap을 다 먹은 뒤 범퍼 추돌한다 (실측 계측).
        double engage = std::max(trailing_engage_dist_m_, cur_vs_ * 1.6 + 3.0);
        // ── 동적은 **속도 매칭 거리** 로 판정한다 (2026-08-20) ──────────────
        // 위 v*1.6 + 3.0 은 물리 유도가 아닌 heuristic 이고, 아래 v_allow
        // 곡선(지연 T + v²/(2a) 를 정확히 푼 것)과 기준이 다르다. 그래서
        // 게이트가 곡선보다 훨씬 앞에서 열려, 상대를 "따라잡는 중"인 구간이
        // 통째로 트레일링 분기로 들어간다 — 실속 6.5 기준 13.4 m 인데
        // 이 트랙의 첫 탐지 봉투는 max 8.3 m 다. 즉 **보이자마자 감속 모드**다.
        //
        // 정지 목표(정적/비상)는 종전 값을 그대로 둔다 — 그쪽은 v_target 이
        // 0 이라 곡선이 일찍 물리고, 오래 튜닝된 값이다.
        // 동적은 v_allow 가 실제로 물리기 시작하는 거리(= 지금 속도에서
        // 상대 속도까지 줄이는 데 필요한 거리)에 15% 여유를 준 값으로 하되,
        // **종전 값을 절대 넘지 않게** 위쪽으로 클램프한다. 즉 이 변경은
        // 감속 시작을 늦추기만 하고 앞당기지 않는다.
        if (!obs.is_static && !emergency_stop) {
            const double v_now = std::max(cur_vs_, 0.0);
            const double v_tgt = std::max(v_target_prof_actual, 0.0);
            const double dv2 = std::max(v_now * v_now - v_tgt * v_tgt, 0.0);
            const double need =
                v_now * trailing_lag_sec_ + dv2 / (2.0 * decel_eff);
            if (std::isfinite(need)) {
                engage = std::clamp(need * 1.15,
                                    trailing_engage_dist_m_, engage);
            }
        }
        double rem = std::max(span - dist, 0.0);
        if (hold_current_d) {
            // 최대 제동 램프. 예전에는 여기서 전 점 0을 썼는데, 장애물이
            // trailing_gap 안쪽이면 span이 modulo로 감겨 0이 되고(그러면
            // rem도 0) 정상 경로에서도 같은 계단이 나왔다.
            //
            // v_target_prof_actual 을 바닥으로 깐다. 이 분기는 위쪽
            // "복귀 기하조차 없음" 경로가 `allow_stop ? 0 : degraded` 로
            // 목표속도를 정해 놓고 들어오는데, 여기서 그 값을 읽지 않아
            // **그 대입이 아무 효과도 없었다** — 정적 장애물에서도 최대제동
            // 곡선이 그대로 나가 다시 정지했다. 비상정지(emergency_stop)는
            // v_target_prof_actual 이 0 이라 종전과 완전히 같다.
            w.vx_mps = std::min(
                w.vx_mps,
                std::max(reachable_v(dist), v_target_prof_actual) /
                    command_scale);
        } else if (rem <= engage) {
            // 순수 v²/(2a) 모델은 명령→감속 사이의 구동계 지연을 무시한다.
            // 실측(rosbag_0806, 명령 0 하강엣지 3건): v0 0.95~1.37 m/s에서
            // 정지까지 0.50~0.72m / 0.67~0.94s, 유효 감속 0.79~1.30 m/s².
            // 설정값 2.5는 그 2.5배라 프로파일이 rem이 충분히 줄 때까지
            // 감속을 시작하지 않았고, 결과적으로 trailing_gap 1.5m를
            // 0.8~1.3m 오버슛해 0.24~0.68m 앞에 섰다. 그 거리에서는
            // BlendDist의 조향 기하 하한(0.35m 횡이동에 1.51m)을 채울 수
            // 없어 탈출 기동 자체가 성립하지 않는다 = 영구 정지 라치.
            //
            // 지연 T를 포함한 정지 조건은
            //   rem >= v·T + (v² - v_target²)/(2a)
            // 이고, v에 대해 풀면 아래 닫힌 형태가 된다.
            const double aT = decel_eff * trailing_lag_sec_;
            const double v_allow = std::max(
                -aT + std::sqrt(
                    aT * aT + v_target_prof_actual * v_target_prof_actual +
                    2.0 * decel_eff * rem),
                0.0);
            // ── 목표 속도는 바닥이다 (2026-08-15) ─────────────────────────
            // 위 식은 "정지점까지 남은 거리 안에서 낼 수 있는 최고 속도"라
            // rem 이 줄면 v_target 으로 수렴해야 하는데, 지연항 -aT 때문에
            // **v_target 아래로 내려간다**. rem=0 에서 정확히
            //   -aT + sqrt(aT² + v_t²) < v_t
            // 이고, 실차 값(decel 1.0 · lag 0.3 · retry 1.5)에서는
            //   -0.3 + sqrt(0.09 + 2.25) = 1.2297
            // 이 나온다. 하한으로 깔아 둔 trailing_retry_speed(1.5)를 하한
            // 계산기가 18% 깎는 셈이고, 그 값에 **도달한 적이 한 번도 없다**.
            //   run_0814_191443 실측: 장애물 gap 안쪽(span=0 -> rem 전 구간 0)
            //   이면 경로 전체가 1.23 으로 평탄해지고, 컨트롤러는 차 밑이
            //   아니라 ~1m 앞(=몸통)을 읽으므로 명령이 1.23 에 고정된다.
            //   그 상태로 5.3초 연속(7:41~7:47), 런 전체 290 사이클 ≈ 10초.
            // ⚠️ trailing_decel 을 올리면 aT 가 커져 이 바닥이 **더** 내려간다
            //    (decel 2.4 -> 0.94). 파라미터를 손대기 전에 이 줄이 먼저다.
            w.vx_mps = std::min(
                w.vx_mps,
                std::max({v_allow, reachable_v(dist), v_target_prof_actual}) /
                    command_scale);
        }
        out_wpnts.wpnts.push_back(w);
    }

    // 횡복귀 램프로 오프셋된 구간의 곡률/그립도 반영 (위 trailing 프로파일보다
    // 낮은 값만 채택되므로 제동 프로파일을 약화시키지 않는다)
    // 이미 위험 구간이면 최대 제동 경로를 반드시 발행해야 하므로 현재 상태
    // feasibility 실패를 이유로 trailing 자체를 버리지는 않는다.
    // 다만 실패하면 **곡률/그립 속도 캡이 하나도 안 걸린 채** 발행된다는 뜻이라
    // 조용히 넘기면 안 된다 (경로는 손대인 곳 없이 온전하다 —
    // ApplySpeedProfile 이 검사를 전수로 먼저 돌고 통과 뒤에만 쓴다).
    //
    // ── 2026-08-20: 곡률 초과를 "미적용"이 아니라 "캡 적용"으로 받는다 ─────
    // 트레일링은 마지막 티어라 버릴 경로가 없다. 그런데 하드 기각은 "버려라"
    // 이므로, 반환값을 무시하고 그대로 발행하면 캡이 통째로 사라진다. 남는
    // 수동 제동 곡선은 reachable_v 바닥(= reachable_v(0)==v0)에 눌려 선두가
    // 현재 속도와 같으므로 실효 캡이 0 이다 — 기동이 급할수록 더 빨리 가는
    // 역전이 여기서 만들어졌다 (run_0820_201714 4:45, 선언부 주석 참조).
    // 소프트 모드를 쓰면 위반 지점에도 sqrt(a_lat/k) 캡이 걸리고, 그 값이
    // ApplySpeedProfile 의 역방향 감속 pass 를 타고 경로 앞쪽으로 전파된다.
    int steer_clamp_idx = -1;
    if (!ApplySpeedProfile(out_wpnts.wpnts, false, nullptr, nullptr,
                           &steer_clamp_idx)) {
        NoteReject("trailing_speed_profile_skipped");
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "Trailing 속도 프로파일 미적용 (%s) — 수동 제동 프로파일만 발행",
            SpeedRejectDetail().c_str());
    } else if (steer_clamp_idx >= 0) {
        NoteReject("trailing_steer_curv_clamped");
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "Trailing 램프가 조향 한계를 넘음 (%s) — 경로는 유지하고 "
            "횡가속 캡으로 감속 (해당 점 %.2f m/s)",
            SpeedRejectDetail().c_str(),
            (steer_clamp_idx < static_cast<int>(out_wpnts.wpnts.size()))
                ? out_wpnts.wpnts[static_cast<size_t>(steer_clamp_idx)].vx_mps
                : -1.0);
    }

    // ── 램프가 장애물을 향하면 속도 바닥을 걷고 제동한다 (2026-08-15) ──────
    // 트레일링의 횡 램프는 여기까지 **벽만** 검사했다(ramp_ok →
    // WithinTrackBoundary). 지나가려는 장애물과 부딪히는지는 아무도 안 봤고,
    // 안전은 "속도 프로파일이 장애물 전에 세워 줄 것"이라는 간접 보장에만
    // 기대고 있었다. 그런데 정적 장애물은 allow_stop=false 라 목표 속도가
    // trailing_retry_speed_ 로 **바닥이 깔려 있어** 세워 주지 않는다.
    //   run_0814_191443 7:23(t=444.27~444.42): 횡목표 포기 → 현재 d(-0.35)
    //   유지 → 장애물 d[-0.42,-0.12] 정중앙으로 직진 → 여유 0.000.
    //   같은 런에서 여유 < 차반폭(0.15m) 통과가 17건, 그중 11건이 트레일링,
    //   그리고 8건은 **횡 계획이 온전한 상태**였다 — 벽 검사만으로는 못 잡는다.
    // 여기서는 경로를 버리지 않는다(트레일링은 마지막 티어라 빈 경로 = 전속
    // global path). 대신 막는 지점 앞에서 서도록 제동 곡선을 덮어씌운다.
    // reachable_v 를 함께 max 로 두어 근거리가 전부 0 이 되는 것을 막는다 —
    // 전 점 vx=0 은 MAP 컨트롤러의 조향을 죽인다(0807 코스아웃).
    if (check_obs != nullptr && !check_obs->empty() &&
        !out_wpnts.wpnts.empty())
    {
        double blocking_s = 0.0;
        // ── 사정거리 절단은 백 검증에서 탈락했다 (2026-08-17) ─────────────────
        // "트레일링 경로는 단일 장애물(obs)용인데 충돌검사는 lookahead 전체를
        // 보니 슬라럼에서 늘 걸린다 — 첫 차단 장애물까지로 끊고 나머지는 다음
        // 사이클에 맡기자"는 안을 구현해 tools/bag_analysis_0817/reach_defer.py
        // 로 재 봤고, **틀렸다.** 미룬 차단점이 처음 사정거리 안에 들어오는
        // 순간의 제동 여유(= 남은거리 - (v*lag + v^2/2a))가 run_0817_123650 의
        // 거의 모든 사례에서 음수였다: 1.4~7.1m 부족. 0811 coasting fail-open
        // 이 "다음 사이클이 다시 잡아 준다"는 **같은 논거**로 틀렸던 그대로다.
        //
        // 다만 그 측정이 같이 드러낸 게 있다. 이 분기가 실제로 발동하는
        // 거리(차단점 0.2~5.6m)에서는 trailing_decel 1.0 으로 세우는 데 필요한
        // 거리가 이미 남은거리를 넘는다 — 즉 **여기서 v_stop 은 정지 곡선이
        // 아니고, reachable_v 바닥에 눌린 "가속 금지" 캡으로만 동작한다.**
        // 그래서 랩타임 손실의 정체는 이 검사 범위가 아니라 그 캡이다
        // (reachable_v(0) == v0 이라 명령 선두가 현재 속도를 넘지 못하는
        // 쌍안정 — 통과시간 vs 진입속도 r=-0.84). 범위를 건드리는 게 아니라
        // 그 캡을 고쳐야 하고, 그건 감속 포락선의 설계 사안이라 별건이다.
        if (!CheckPathCollision(out_wpnts.wpnts, *check_obs, &blocking_s)) {
            NoteReject("trailing_ramp_collision");
            double stop_dist =
                Mod(blocking_s - cur_s_, track_len_) - trailing_gap_m_;
            if (!std::isfinite(stop_dist) || stop_dist > lookahead_m_ + 5.0) {
                stop_dist = 0.0;   // 이미 안쪽까지 파고든 상태
            }
            stop_dist = std::max(stop_dist, 0.0);
            // ── 이 곡선도 v_target 을 목표로 삼는다 (2026-08-17) ──────────────
            // 이 분기는 d809831("정적 장애물의 정지를 폐지한다") 이전부터
            // 있었고 그 커밋이 손대지 않았다 — 3913줄과 **같은 마무리 누락**
            // 이다. 위 3994줄의 v_allow 는 정지 조건을
            //   rem >= v*T + (v^2 - v_target^2)/(2a)
            // 로 풀어 v_target 으로 수렴하는데, 여기 v_stop 만 v_target 항이
            // 빠져 있어 **allow_stop 과 무관하게 항상 완전 정지 곡선**을
            // 덮어씌웠다. 정적 장애물은 allow_stop=false 라 3657줄이
            // trailing_retry_speed(2.5) 바닥을 깔아 두는데 그게 여기서 지워진다.
            //
            // run_0817_194156 실측 (트레일링 1,408 사이클 중 이 분기 1,148 =82%):
            //   · 명령 선두 / 현재속도 비율 중앙 1.00, p10 0.56
            //   · 선두가 현재보다 낮은(=가속 금지) 사이클 45%
            //   · 정상 주행(committed_hold)은 가속 허용 70% / +0.24 m/s 인데
            //     트레일링은 4~11% 로 떨어진다
            //   · 트레일링 1회당 3.69 -> 3.15 m/s (-0.54), 지속 중앙 664 ms
            // stop_dist 1.0 m 예: 옛 곡선 -0.3+sqrt(0.09+2.0)=1.15 m/s,
            // 새 곡선 -0.3+sqrt(0.09+6.25+2.0)=2.59 m/s. 즉 차가 2.5 아래로
            // 떨어져 있으면 이제 **트레일링 중에도 재가속**한다.
            //
            // 거동이 바뀌지 않는 경우(의도됨): allow_stop=true 인 동적 상대와
            // 비상정지는 v_target_prof_actual 이 상대 속도 또는 0 이므로
            // v_target=0 인 경우 옛 식과 비트 단위로 같다.
            // ⚠️ 트레이드는 1326줄이 이미 명시한 것과 동일하다 — 정적 장애물에
            // 대한 "완전 정지" 경로가 이 분기에서도 사라진다. 남는 방어선은
            // 컨트롤러 백스톱과 recovery_supervisor 다.
            const double aT_stop = decel_eff * trailing_lag_sec_;
            for (size_t i = 0; i < out_wpnts.wpnts.size(); ++i) {
                const double dist = static_cast<double>(i) * wpnt_dist_;
                const double rem_stop = std::max(stop_dist - dist, 0.0);
                const double v_stop = std::max(
                    -aT_stop + std::sqrt(
                        aT_stop * aT_stop +
                        v_target_prof_actual * v_target_prof_actual +
                        2.0 * decel_eff * rem_stop),
                    0.0);
                // v_target 을 바닥으로 함께 깐다 — 3999줄 주석과 같은 이유로
                // 지연항 -aT 가 rem_stop=0 에서 v_target 아래로 내려간다.
                out_wpnts.wpnts[i].vx_mps = std::min(
                    out_wpnts.wpnts[i].vx_mps,
                    std::max({v_stop, reachable_v(dist),
                              v_target_prof_actual}) / command_scale);
            }
            // 속도를 덮어썼으면 ax 도 다시 계산한다. ApplySpeedProfile 은
            // 마지막에 v² 공간미분으로 ax 를 맞추는데, 이 덮어쓰기는 그
            // 뒤에 일어나므로 그대로 두면 컨트롤러가 옛 가속도를 읽는다.
            for (size_t i = 0; i + 1 < out_wpnts.wpnts.size(); ++i) {
                const double ds_i = std::max(std::hypot(
                    out_wpnts.wpnts[i + 1].x_m - out_wpnts.wpnts[i].x_m,
                    out_wpnts.wpnts[i + 1].y_m - out_wpnts.wpnts[i].y_m), 1e-3);
                out_wpnts.wpnts[i].ax_mps2 =
                    (out_wpnts.wpnts[i + 1].vx_mps *
                         out_wpnts.wpnts[i + 1].vx_mps -
                     out_wpnts.wpnts[i].vx_mps * out_wpnts.wpnts[i].vx_mps) /
                    (2.0 * ds_i);
            }
            if (out_wpnts.wpnts.size() >= 2) {
                out_wpnts.wpnts.back().ax_mps2 =
                    out_wpnts.wpnts[out_wpnts.wpnts.size() - 2].ax_mps2;
            }
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000,
                "Trailing 램프가 장애물과 충돌 (blocking s=%.2f, 정지까지 "
                "%.2fm) — 속도 바닥 해제하고 제동",
                blocking_s, stop_dist);
        }
    }

    out_mrks.markers.reserve(out_wpnts.wpnts.size());
    for (size_t i = 0; i < out_wpnts.wpnts.size(); ++i) {
        const auto &w = out_wpnts.wpnts[i];
        out_mrks.markers.push_back(
            MakeWpntMarker(w.x_m, w.y_m, w.vx_mps, static_cast<int>(i)));
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
        "Trailing fallback: obs s=%.1f gap=%.1fm v_target=%.1f "
        "d %.2f -> %.2f (ramp %.1fm) (%zu wpnts)",
        obs.s_center, span, v_target_actual, cur_d_, d_goal, d_ramp_len,
        out_wpnts.wpnts.size());
    NoteOutcome("trailing");
    return true;
}

void SplinerNode::ClassifyObstacles(
    std::vector<f1tenth_planning_msgs::msg::Obstacle> &obstacles) {
    // unicorn perception/multi_tracking.py의 정적/동적 분류 이식:
    //  1) 위치 std 투표 — 최근 관측의 std_s/std_d가 둘 다 min_std 미만이면
    //     정적 표, 하나라도 max_std 초과면 표 리셋. 정적 표 비율 >= 0.5 → 정적.
    //  2) vs_reset — 동적이라도 최근 |vs| 평균이 vs_reset 미만이면(멈춘 상대)
    //     정적으로 재분류 (multi_tracking.py update()의 avg_vs < vs_reset).
    // 관측이 min_nb_meas에 못 미치면 perception이 준 is_static을 그대로 둔다.
    rclcpp::Time now = this->now();
    for (auto &obs : obstacles) {
        // ── 2026-08-19 (#15): 재등록 시 분류 이력 승계 (동적 전용) ──────────
        // run_0818_232744 실측: 같은 물체에 **조우 도중** 새 id 가 0.7~1.4 m
        // 앞에서 7번 태어났다 (id 9/21/54/55/56/57/61). 새 id 는 이 맵에
        // 없으므로 관측 이력이 0 이 되고, cls_min_nb_meas 를 다시 채울 때까지
        // perception 플래그(대개 정적)가 그대로 통과한다 — 커밋도 함께 파기된다.
        //
        // 정적 트랙에는 적용하지 않는다. 나란히 선 콘 두 개가 서로의 이력을
        // 물려받으면 한쪽의 정적 표가 다른 쪽으로 새기 때문이다. 승계는
        // **한 번이라도 동적이었던** 트랙에서만, 그리고 방금 사라진 것에서만
        // 가져온다.
        if (cls_inherit_dist_m_ > 0.0 &&
            cls_tracks_.find(obs.id) == cls_tracks_.end())
        {
            int donor = 0;
            double best = cls_inherit_dist_m_;
            for (const auto &kv : cls_tracks_) {
                const auto &cand = kv.second;
                if (!cand.ever_dynamic || cand.s_hist.empty()) {
                    continue;
                }
                const double age = (now - cand.last_seen).seconds();
                if (!(age > 0.0) || age > cls_inherit_timeout_sec_) {
                    continue;   // 아직 살아 있거나(=별개 물체) 너무 오래됨
                }
                // 마지막 관측 위치를 그 트랙의 vs 로 지금까지 외삽해 비교한다.
                const double s_last = cand.s_ref + cand.s_hist.back();
                const double vs_last =
                    cand.vs_hist.empty() ? 0.0 : cand.vs_hist.back();
                const double ds = Mod(
                    obs.s_center - Mod(s_last + vs_last * age, track_len_) +
                    track_len_ / 2.0, track_len_) - track_len_ / 2.0;
                const double dd = obs.d_center - cand.d_hist.back();
                const double dist = std::hypot(ds, dd);
                if (dist < best) {
                    best = dist;
                    donor = kv.first;
                }
            }
            if (donor != 0) {
                cls_tracks_[obs.id] = cls_tracks_[donor];
                cls_tracks_.erase(donor);
                RCLCPP_INFO(this->get_logger(),
                    "분류 이력 승계: id %d -> %d (거리 %.2fm, 관측 %zu)",
                    donor, obs.id, best, cls_tracks_[obs.id].s_hist.size());
            }
        }
        auto &tr = cls_tracks_[obs.id];
        tr.last_seen = now;
        // s는 첫 관측 기준으로 unwrap해 원형 좌표 std 왜곡 방지
        if (tr.s_hist.empty()) {
            tr.s_ref = obs.s_center;
        }
        const double disp_now =
            Mod(obs.s_center - tr.s_ref + track_len_ / 2.0, track_len_) -
            track_len_ / 2.0;
        tr.s_hist.push_back(disp_now);
        // 관측된 최대 변위. s_ref 기준 절댓값이라 왕복해도 줄지 않는다.
        tr.max_disp = std::max(tr.max_disp, std::abs(disp_now));
        tr.d_hist.push_back(obs.d_center);
        tr.vs_hist.push_back(obs.vs);
        if (tr.s_hist.size() > kClsWindow) {
            tr.s_hist.pop_front();
            tr.d_hist.pop_front();
        }
        if (tr.vs_hist.size() > kClsVsWindow) {
            tr.vs_hist.pop_front();
        }
        if (static_cast<int>(tr.s_hist.size()) <= cls_min_nb_meas_) {
            continue;  // 표본 부족 — perception 플래그 유지
        }

        auto std_of = [](const std::deque<double> &v) {
            double mean = 0.0;
            for (double x : v) mean += x;
            mean /= static_cast<double>(v.size());
            double sq = 0.0;
            for (double x : v) sq += (x - mean) * (x - mean);
            return std::sqrt(sq / static_cast<double>(v.size()));
        };
        double std_s = std_of(tr.s_hist);
        double std_d = std_of(tr.d_hist);
        if (std_s < cls_min_std_ && std_d < cls_min_std_) {
            ++tr.static_count;
        } else if (std_s > cls_max_std_ || std_d > cls_max_std_) {
            tr.static_count = 0;
        }
        ++tr.total_count;
        bool by_vote = tr.static_count >=
            (tr.total_count + 1) / 2;   // 비율 >= 0.5
        double vs_avg = 0.0;
        for (double v : tr.vs_hist) vs_avg += std::abs(v);
        vs_avg /= static_cast<double>(tr.vs_hist.size());
        // 한 번 내려간 것만으로는 뒤집지 않는다 — cls_vs_hold_ 사이클 연속.
        const bool vs_low =
            tr.vs_hist.size() >= kClsVsWindow && vs_avg < cls_vs_reset_;
        tr.vs_low_run = vs_low ? tr.vs_low_run + 1 : 0;
        bool by_vs = tr.vs_low_run >= cls_vs_hold_;

        bool st = by_vote || by_vs;
        if (!tr.has_flag || st != tr.static_flag) {
            pitwall::Event("장애물 #" + std::to_string(obs.id) +
                           (st ? " 정적 분류" : " 동적 분류"));
            RCLCPP_INFO(this->get_logger(),
                "obstacle %d classified %s (std_s=%.2f std_d=%.2f avg|vs|=%.2f)",
                obs.id, st ? "STATIC" : "DYNAMIC", std_s, std_d, vs_avg);
        }
        tr.has_flag = true;
        tr.static_flag = st;
        if (!st) {
            tr.ever_dynamic = true;   // 한 번 동적이면 각인 (풀리지 않는다)
        }
        obs.is_static = st;
        // 관측 이력으로 위치 분산을 실측값으로 교체한다. perception 브릿지는
        // 모든 장애물에 s_var/d_var를 상수(0.01 = σ 0.1m)로 붙이는데,
        // CheckPathCollision이 3σ를 요구하므로 그것만으로 0.3m, 여기에
        // path_check_margin까지 더하면 요구 간격이 0.5m를 넘는다 —
        // 폭 1.7~2.0m 트랙에서는 회피 목표(evasion_dist 0.35)가 절대
        // 이 값을 못 넘어 계획한 회피를 자기 검증기가 매번 거부했다
        // (run_0806_155621: 정적 회피 성공 0건). 같은 자리에서 100프레임
        // 관측된 정지물의 실측 std는 0.00~0.06m다. 하한은 스캔 격자/정합
        // 오차(0.05m)로 두어 과소평가를 막는다.
        constexpr double kMinPosStd = 0.05;
        const double eff_std_s = std::max(std_s, kMinPosStd);
        const double eff_std_d = std::max(std_d, kMinPosStd);
        obs.s_var = eff_std_s * eff_std_s;
        obs.d_var = eff_std_d * eff_std_d;
        // 속도 분산도 같은 문제다 — 브릿지 상수 0.25는 σ_v 0.5 m/s로,
        // "정지물"에 붙이면 도착시각 0.3s만으로도 3σ가 0.45m가 된다.
        // 정적으로 분류된 물체는 관측된 속도 산포로 대체한다. 하한 0.1 m/s는
        // 분류 오류(사실은 아주 느린 물체)에 대한 완충이고, 브릿지 값보다
        // 크게 만들지는 않는다(std::min) — 보수적 방향은 항상 유지.
        if (st) {
            double vs_mean = 0.0;
            for (double v : tr.vs_hist) vs_mean += v;
            vs_mean /= static_cast<double>(tr.vs_hist.size());
            double vs_sq = 0.0;
            for (double v : tr.vs_hist) vs_sq += (v - vs_mean) * (v - vs_mean);
            const double vs_std =
                std::sqrt(vs_sq / static_cast<double>(tr.vs_hist.size()));
            constexpr double kMinVelStd = 0.10;   // m/s
            const double eff_vs = std::max(vs_std, kMinVelStd);
            obs.vs_var = std::min(obs.vs_var, eff_vs * eff_vs);
            obs.vd_var = std::min(obs.vd_var, kMinVelStd * kMinVelStd);
        }
        if (st) {
            obs.vs = 0.0;   // 정지 상대를 trailing할 때 목표 속도 0
            obs.vd = 0.0;
        }
    }
    // 사라진 장애물 트랙 정리
    for (auto it = cls_tracks_.begin(); it != cls_tracks_.end();) {
        if ((now - it->second.last_seen).seconds() > 2.0) {
            it = cls_tracks_.erase(it);
        } else {
            ++it;
        }
    }
}

bool SplinerNode::MergeBlocksTrack(
    const f1tenth_planning_msgs::msg::Obstacle &a,
    const f1tenth_planning_msgs::msg::Obstacle &b,
    double s_start, double s_end) const {
    // 판정식은 MoreSpace와 완전히 같은 예산을 쓴다 — 여기서만 다르면
    // "병합은 허용했는데 MoreSpace가 못 지나간다"거나 그 반대가 생긴다.
    const double d_left = std::max(a.d_left, b.d_left);
    const double d_right = std::min(a.d_right, b.d_right);
    if (!std::isfinite(d_left) || !std::isfinite(d_right)) {
        // 판단 불가 -> "병합하면 막힌다"로 본다 = **병합하지 않는다**.
        // 이 파일의 정책은 "사전 필터는 낙관적, 가부는 CheckPathCollision"이고,
        // 병합은 통로를 없애는 비관적 조작이라 그쪽이 보수적인 답이다.
        // (Run() 이 non-finite d 를 만나면 배열을 통째로 비우므로 현재는
        //  도달하지 않는다 — 입력 계약이 바뀌었을 때의 안전판이다.)
        return true;
    }
    double wall_left, wall_right;
    MinWallInSpan(Mod(s_start - 0.5, track_len_),
                  Mod(s_end + 0.5, track_len_), wall_left, wall_right);
    // 요구 간격은 두 구성원 중 큰 쪽 (보수적).
    const double need = std::max(RequiredObstacleClearance(a),
                                 RequiredObstacleClearance(b));
    const double min_space = need + RequiredBoundaryMargin(spline_bound_mindist_);
    const double left_gap = wall_left - d_left;
    const double right_gap = wall_right + d_right;
    return left_gap < min_space && right_gap < min_space;
}

void SplinerNode::UpdateCorridorBlockLatch() {
    const auto now = this->now();
    const bool evidence = corridor_block_evidence_;

    if (evidence) {
        if (!corridor_block_run_valid_) {
            // 근거 구간의 시작. 무장 여부를 **여기서 한 번** 고정한다.
            // 매 사이클 속도를 다시 보면, 래치가 걸려 차가 느려지는 순간
            // 게이트가 스스로 래치를 풀어 바닥이 되살아난다(자기모순).
            corridor_block_run_valid_ = true;
            corridor_block_run_since_ = now;
            corridor_block_run_armed_ =
                !(corridor_block_stop_min_speed_ > 0.0) ||
                std::abs(cur_vs_) >= corridor_block_stop_min_speed_;
        }
        corridor_block_last_evidence_ = now;
        if (corridor_block_stop_enabled_ && corridor_block_run_armed_ &&
            !corridor_block_latched_ &&
            (now - corridor_block_run_since_).seconds() >=
                corridor_block_stop_hold_sec_) {
            corridor_block_latched_ = true;
            NoteOutcome("corridor_block_stop_latched");
            RCLCPP_WARN(this->get_logger(),
                "회랑 봉쇄 래치: 정지 권한 개방 (s=%.2f, v=%.2f m/s, "
                "hold %.2fs) — 트레일링 속도 바닥을 푼다",
                cur_s_, cur_vs_, corridor_block_stop_hold_sec_);
            pitwall::Event("회랑 봉쇄 — 정지 권한 개방");
        }
    } else {
        corridor_block_run_valid_ = false;
        if (corridor_block_latched_) {
            // 해제에는 속도 조건을 걸지 않는다 (위 무장 주석과 같은 이유).
            // last_evidence_ 가 0 인 경우는 래치가 걸린 적이 없다는 뜻이라
            // 여기에 도달하지 않는다.
            if ((now - corridor_block_last_evidence_).seconds() >=
                corridor_block_stop_release_sec_) {
                corridor_block_latched_ = false;
                RCLCPP_INFO(this->get_logger(),
                    "회랑 봉쇄 래치 해제: 근거 %.2fs 부재 (s=%.2f)",
                    corridor_block_stop_release_sec_, cur_s_);
                pitwall::Event("회랑 봉쇄 해제");
            }
        }
    }

    // 계측용. corridor_block_stop_enabled_ 가 false 여도 근거 판정과 이
    // 발행은 그대로 돌아간다 — 백에서 발화율만 재고 거동은 종전으로 두는
    // 관측 모드가 성립해야 문턱을 옮길 근거를 모을 수 있다.
    if (p_corridor_blocked_) {
        std_msgs::msg::Bool msg;
        msg.data = corridor_block_latched_;
        p_corridor_blocked_->publish(msg);
    }
}

std::string SplinerNode::SpeedRejectDetail() const {
    if (speed_reject_idx_ < 0) {
        return "도달불가(감속)";
    }
    const double pct = (speed_reject_n_ > 1)
        ? 100.0 * speed_reject_idx_ / (speed_reject_n_ - 1) : 0.0;
    char buf[128];
    std::snprintf(buf, sizeof(buf),
                  "곡률 idx=%d/%d(%.0f%%) k=%.2f > 한계 %.2f",
                  speed_reject_idx_, speed_reject_n_, pct,
                  speed_reject_kappa_, speed_reject_limit_);
    return std::string(buf);
}

void SplinerNode::NoteReject(const char *reason) const {
    if (reason == nullptr) {
        return;
    }
    ++reject_counts_[reason];
    // 사이클 트레이스용 발생 순서 기록. 한 사이클에 좌/우 레인 + 스플라인이
    // 각각 다른 사유로 떨어질 수 있어서, 카운터만으로는 "무엇이 먼저 막았나"를
    // 복원할 수 없다. 길이는 넉넉히 잘라 로그 한 줄을 넘기지 않게 한다.
    if (cycle_rejects_.size() < 220) {
        if (!cycle_rejects_.empty()) {
            cycle_rejects_ += ',';
        }
        cycle_rejects_ += reason;
    }
}

void SplinerNode::NoteOutcome(const char *outcome) const {
    if (outcome == nullptr) {
        return;
    }
    ++outcome_counts_[outcome];
    // trace 의 마지막 필드는 그대로 한 칸을 쓰되, 거부가 아닌 항목은 '+' 를
    // 붙여 눈으로 구분되게 한다. 필드 구조를 바꾸지 않으므로 기존 파서
    // (tools/bagviz.py, bag_analysis_*/)는 그대로 동작한다.
    if (cycle_rejects_.size() < 220) {
        if (!cycle_rejects_.empty()) {
            cycle_rejects_ += ',';
        }
        cycle_rejects_ += '+';
        cycle_rejects_ += outcome;
    }
}

void SplinerNode::TraceCycle(
    const f1tenth_planning_msgs::msg::Obstacle *closest,
    bool committed, bool try_evade, bool in_sector,
    const char *outcome) const {
    if (!trace_cycles_) {
        return;
    }
    double dist = -1.0;
    double d_lo = 0.0, d_hi = 0.0;
    int id = 0;
    int is_static = -1;
    int cls_obs = -1;
    if (closest != nullptr) {
        dist = Mod(closest->s_start - cur_s_, std::max(track_len_, 1e-3));
        if (dist > track_len_ / 2.0) {
            dist = -Mod(cur_s_ - closest->s_start, track_len_);  // 이미 지나침
        }
        d_lo = closest->d_right;
        d_hi = closest->d_left;
        id = closest->id;
        is_static = closest->is_static ? 1 : 0;
        // 정적/동적 분류가 아직 확정 전이면(관측 부족) -1이 아니라 관측 수를
        // 보여준다 — "분류가 늦어서 회피를 안 한다"를 바로 구별하기 위한 것.
        auto it = cls_tracks_.find(closest->id);
        if (it != cls_tracks_.end()) {
            cls_obs = it->second.total_count;
        }
    }
    RCLCPP_INFO(this->get_logger(),
        "[trace] s=%.2f v=%.2f d=%.2f | obs id=%d dist=%.2f d[%.2f,%.2f] "
        "static=%d nobs=%d | evade=%d sector=%d commit=%d | %s | %s",
        cur_s_, cur_vs_, cur_d_, id, dist, d_lo, d_hi, is_static, cls_obs,
        try_evade ? 1 : 0, in_sector ? 1 : 0, committed ? 1 : 0,
        outcome, cycle_rejects_.empty() ? "-" : cycle_rejects_.c_str());
}

void SplinerNode::NoteRetryTarget(double d_target) {
    if (!std::isfinite(d_target)) {
        return;
    }
    retry_d_target_ = d_target;
    retry_d_stamp_ = this->now();
    has_retry_d_ = true;
}

void SplinerNode::TruncateCommitAt(double release_s, double blocking_s) {
    auto &wp = committed_wpnts_.wpnts;
    if (wp.size() < 2 || !std::isfinite(release_s) || !(track_len_ > 0.0)) {
        return;
    }
    // 경로 시작(committed_start_s_)부터 release_s 까지 남긴다. s 는 단조
    // 증가하지만 랩을 넘을 수 있으므로 시작점 기준 누적 거리로 센다.
    const double keep = Mod(release_s - committed_start_s_, track_len_);
    if (!(keep > 0.0) || keep >= track_len_ * 0.5) {
        return;      // 자를 것이 없거나 wrap 해석이 애매하다 — 손대지 않는다
    }
    size_t keep_n = wp.size();
    for (size_t i = 0; i < wp.size(); ++i) {
        if (Mod(wp[i].s_m - committed_start_s_, track_len_) > keep) {
            keep_n = i;
            break;
        }
    }
    // 최소 길이. 컨트롤러 L1(≈0.47v-0.2, 5 m/s 에서 2.15 m)이 잘린 끝을
    // 넘어 raceline 을 읽는 구간을 남기지 않으려면 기동을 덮을 만큼은
    // 있어야 한다. 못 채우면 자르지 않고 그대로 둔다 — 호출부의
    // blocking_ahead - plan_lead > maneuver_end 가 이미 이것을 보장하므로
    // 여기 도달하면 수치 이상이라는 뜻이고, 그때는 종전 파기 경로가 옳다.
    constexpr size_t kMinKeep = 5;
    if (keep_n < kMinKeep || keep_n >= wp.size()) {
        return;
    }
    const size_t was_n = wp.size();
    wp.resize(keep_n);
    if (committed_geom_ceiling_.size() > keep_n) {
        committed_geom_ceiling_.resize(keep_n);
    } else if (committed_geom_ceiling_.size() != keep_n) {
        // 크기가 어긋나면 속도 재계산이 동결값으로 안전하게 떨어진다.
        committed_geom_ceiling_.clear();
    }
    if (committed_mrks_.markers.size() > keep_n) {
        committed_mrks_.markers.resize(keep_n);
    }
    committed_until_s_ = wp.back().s_m;
    if (!committed_tail_truncated_) {
        committed_tail_truncated_ = true;
        NoteReject("commit_tail_truncated");
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000,
            "커밋 꼬리 절단: 차단 s=%.2f, 만료 s=%.2f (%zu -> %zu wpnts) — "
            "기동은 유지하고 그 앞에서 재계획한다",
            blocking_s, committed_until_s_, was_n, keep_n);
    }
}

void SplinerNode::PublishRejectCounts() {
    // 1Hz. 거부 로그가 전부 THROTTLE(2000ms)인데 루프는 20~30Hz라 최대
    // 1/40~1/60만 남는다 — 0808 run_185049 분석에서 "left 레인이 왜
    // 실패했나"가 대부분 지워져 있었다. 카운터는 전수를 세므로 랩별
    // 거부 사유 분포를 그대로 볼 수 있다 (Float32 하나당 수십 바이트).
    const auto now = this->now();
    if ((now - reject_publish_stamp_).seconds() < 1.0) {
        return;
    }
    reject_publish_stamp_ = now;
    for (const auto &kv : reject_counts_) {
        pitwall::Log("reject_" + kv.first, static_cast<double>(kv.second));
    }
    // 거부가 아닌 것은 outcome_ 로 따로 나간다 (NoteOutcome 주석 참조).
    for (const auto &kv : outcome_counts_) {
        pitwall::Log("outcome_" + kv.first, static_cast<double>(kv.second));
    }
}

std::vector<f1tenth_planning_msgs::msg::Obstacle> SplinerNode::ClusterObstacles(
    std::vector<f1tenth_planning_msgs::msg::Obstacle> obs) {
    // 사이 corridor가 cluster_gap_d보다 좁고 s 간격이 cluster_gap_s 이내인
    // 쌍을 하나의 (s,d) 박스로 반복 병합 (합쳐진 박스가 또 다른 장애물과
    // 붙으면 다시 병합 — 수렴할 때까지)
    // ego "뒤 15m"를 기준점으로 unwrap — ego가 클러스터 안(두 장애물 사이)에
    // 진입해 일부가 살짝 뒤로 가도 클러스터가 wrap 때문에 쪼개지지 않게 한다
    // (쪼개지면 남은 장애물 기준으로 사이드가 바뀌며 회피가 중간에 붕괴)
    double ref = Mod(cur_s_ - 15.0, track_len_);
    bool merged_any = true;
    while (merged_any && obs.size() > 1) {
        merged_any = false;
        for (size_t i = 0; i < obs.size() && !merged_any; ++i) {
            for (size_t j = i + 1; j < obs.size() && !merged_any; ++j) {
                double ai = Mod(obs[i].s_start - ref, track_len_);
                double bi = ai + Mod(obs[i].s_end - obs[i].s_start, track_len_);
                double aj = Mod(obs[j].s_start - ref, track_len_);
                double bj = aj + Mod(obs[j].s_end - obs[j].s_start, track_len_);
                double s_gap = std::max(ai, aj) - std::min(bi, bj);
                double d_gap = std::max(obs[i].d_right, obs[j].d_right) -
                               std::min(obs[i].d_left, obs[j].d_left);
                // s_gap만 상수(cluster_gap_s_m)로 보면 종방향으로 멀리 떨어진
                // 지그재그 배치를 하나의 축정렬 박스로 묶어 통로를 없앤다:
                //   run_0806_234734 38.20s — #42(우측 d[-0.40,0.06], s 1.68~2.14)
                //   + #44/#45(좌측 d[0.24,1.08], s≈4.1) -> d[-0.39,1.08] 폭 1.47m.
                //   그 지점 트랙 폭은 1.61m라 "벽"이 됐고 TRAILING 정지로 갔다.
                //   실제로는 s=2에서 좌측 1.09m, s=4.2에서 우측 0.71m가 비어
                //   대각선 통과가 가능했다.
                // 그래서 "그 s 간격 안에서 실제로 횡전환이 가능한가"로 바꾼다.
                // 필요 횡이동은 두 박스를 각각 비켜 가려면 옮겨야 하는 거리
                // (겹치는 d 구간의 폭)이고, 가능 여부는 BlendSpeedCap이 0보다
                // 큰지 = 조향 기하 하한을 s_gap 안에 넣을 수 있는지로 본다.
                bool mergeable = s_gap < cluster_gap_s_m_ &&
                                 d_gap < cluster_gap_d_m_;
                if (mergeable && s_gap > 0.0) {
                    // 두 장애물을 각각 반대편으로 비켜 갈 때 필요한 횡이동
                    const double need_lat = std::min(
                        std::abs(obs[i].d_left - obs[j].d_right),
                        std::abs(obs[j].d_left - obs[i].d_right));
                    // BlendLatAcc/BlendKappaMax 는 span 을 Mod(s_to - s_from)로
                    // 해석하므로 "앞선 박스의 끝 -> 뒤 박스의 시작" 순서로 넘겨야
                    // 한다. 예전엔 등록 순서 그대로 obs[i].s_end -> obs[j].s_start
                    // 를 넘겨서, 쌍의 순서가 뒤집히면 거의 한 바퀴 구간의 최악값을
                    // 보고 통과 가능한 두 장애물을 하나의 가짜 벽으로 병합했다.
                    const bool i_first = ai <= aj;
                    const double gap_from = i_first ? obs[i].s_end : obs[j].s_end;
                    const double gap_to = i_first ? obs[j].s_start : obs[i].s_start;
                    const double a_lat = BlendLatAcc(gap_from, gap_to);
                    const double kap = BlendKappaMax(gap_from, gap_to);
                    if (BlendSpeedCap(need_lat, s_gap, a_lat, kap) > 0.0) {
                        // 사이로 지나갈 기하가 성립한다 — 병합하지 않는다.
                        mergeable = false;
                    }
                }
                // 병합 결과가 "트랙 전체가 막혔다"가 되면 그 병합은 거의
                // 항상 벽 리턴/유령과 실물이 합쳐진 산물이다.
                //   0808 run_185049: 실물은 s=31.7 d[-0.22,+0.19] 폭 0.41m
                //   하나뿐인데(LiDAR 재구성), 1m 뒤 좌벽 리턴(s 0.30~0.50,
                //   d[-0.08,+0.94], 맵 벽거리 0.25m)과 합쳐져 d[-0.28,+0.97]
                //   폭 1.25m 박스가 됐다. 폭 1.47m 직선에서 좌우가 동시에
                //   막혀 매 랩 TRAILING 제동으로 떨어졌고, 같은 박스가
                //   커밋된 회피(StableObstacles 기반)까지 파기시켰다.
                // 사전 필터는 낙관적이어야 하고 실제 가부는
                // CheckPathCollision이 판정한다(이 파일의 일관된 정책) —
                // 두 박스를 따로 두면 각각을 비켜 가는 해가 검증기까지 갈 수
                // 있고, 정말 못 지나가면 거기서 정상적으로 거부된다.
                if (mergeable) {
                    const double m_start = Mod(ref + std::min(ai, aj), track_len_);
                    const double m_end = Mod(ref + std::max(bi, bj), track_len_);
                    if (MergeBlocksTrack(obs[i], obs[j], m_start, m_end)) {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(),
                            2000,
                            "클러스터 병합 보류: 합치면 통과 가능한 d가 없음 "
                            "(id %d+%d, s[%.1f,%.1f], d[%.2f,%.2f])",
                            obs[i].id, obs[j].id, m_start, m_end,
                            std::min(obs[i].d_right, obs[j].d_right),
                            std::max(obs[i].d_left, obs[j].d_left));
                        NoteOutcome("cluster_merge_blocks_track");
                        // ── 봉쇄 근거 (2026-08-22) ─────────────────────────
                        // 병합은 여전히 취소한다(위 0808 근거는 유효하다).
                        // 다만 그 판정을 버리지 않고 정지 권한 래치에 넘긴다.
                        //
                        // ⚠️ 여기에 WasEverMoved 게이트를 걸면 안 된다. 처음에
                        // "구성원 전원이 한 번도 안 움직였을 때만 근거"로 짰다가
                        // run_0822_000310 재생에서 기능이 통째로 죽었다:
                        // 잔해는 강체 하나인데 클러스터 분할이 매 프레임 조각을
                        // 재배분해 중심이 1~2 m 미끄러진다 — 사고 버스트의
                        // **93쌍 중 70쌍**이 "움직인 구성원"을 갖고 있었고,
                        // 정작 오발화 버스트(헤어핀 콘)는 46/46 이 전부
                        // "안 움직임"이었다. 게이트가 정확히 반대로 걸린다.
                        // 근거는 **회랑**의 성질이지 트랙 하나의 성질이 아니다.
                        corridor_block_evidence_ = true;
                        mergeable = false;
                    }
                }
                if (mergeable) {
                    auto &a = obs[i];
                    const auto &b = obs[j];
                    const auto merged_id = std::min(a.id, b.id);
                    double na = std::min(ai, aj);
                    double nb = std::max(bi, bj);
                    a.s_start = Mod(ref + na, track_len_);
                    a.s_end = Mod(ref + nb, track_len_);
                    a.s_center = Mod(ref + (na + nb) / 2.0, track_len_);
                    a.d_left = std::max(a.d_left, b.d_left);
                    a.d_right = std::min(a.d_right, b.d_right);
                    a.d_center = (a.d_left + a.d_right) / 2.0;
                    a.vs = std::min(a.vs, b.vs);       // trailing 목표속도 보수적
                    if (std::abs(b.vd) > std::abs(a.vd)) {
                        a.vd = b.vd;
                    }
                    a.s_var = std::max(a.s_var, b.s_var);
                    a.d_var = std::max(a.d_var, b.d_var);
                    a.vs_var = std::max(a.vs_var, b.vs_var);
                    a.vd_var = std::max(a.vd_var, b.vd_var);
                    a.size = std::max(a.size, b.size);
                    // 구성원 중 하나라도 움직이면 클러스터는 동적 (OR였을 때
                    // 예측 구간과 합쳐진 상대가 정적으로 분류돼 커밋/부스트 오동작)
                    a.is_static = a.is_static && b.is_static;
                    // 하나라도 지금 보이면 클러스터는 보이는 것이다.
                    // 이 줄이 없어서 결과 박스는 **인덱스 i 쪽 값**을 그냥
                    // 물려받았다 — i 가 코스팅 유령이고 j 가 실물이면 합친
                    // 박스가 is_visible=false 가 되어 StableObstacles 의
                    // commit_break_requires_visible_ 게이트에서 탈락하고,
                    // 실물이 커밋을 깰 자격을 잃었다. 다른 필드는 전부
                    // max/min/AND 로 명시 병합하는데 이것만 순서 의존이었다.
                    a.is_visible = a.is_visible || b.is_visible;
                    a.id = merged_id;
                    obs.erase(obs.begin() + static_cast<long>(j));
                    merged_any = true;
                }
            }
        }
    }
    // ── 지나갈 수 없는 사이 통로를 메운다 (2026-08-22, run_0822_144702) ─────
    // 위 병합은 "합치면 트랙이 막힌다"(MergeBlocksTrack)면 취소된다. 그 근거
    // (0808 run_185049 의 벽유령+실물)는 여전히 유효하지만, **취소의 부작용**
    // 까지 정당화하지는 않는다: 두 박스가 따로 남으면 그 사이의 틈이 통로로
    // 살아 있고, 하류에서 실제로 그리로 경로가 나갔다.
    //   run_0822_144702 t=23.17 "클러스터 병합 보류 (id 18+19)" ->
    //   t=23.32 트레일링 사다리가 두 박스 사이 **4 cm** 틈을 조준
    //   (d -0.12 -> -0.42, 차체 여유 -0.160 m) -> t≈24.2 접촉.
    //   같은 백 실측: 발행 경로가 0.64 m 미만 사이통로를 지난 프레임 28 개가
    //   **전부** 이 사고 구간(23.5~24.8 s)이고 92 초 주행 중 다른 곳은 0 이다.
    //   그 28 개는 100 % 가 **s 구간이 겹치는** 쌍이었다.
    // 그래서 병합은 그대로 취소하되(바깥으로 도는 해를 살린다), 사이 틈만
    // 양쪽 박스를 중앙까지 늘려 막는다. 박스를 새로 만들지 않으므로 id/관측
    // 이력/커밋 판정 등 하류 계약이 하나도 바뀌지 않는다.
    //
    // 적용 조건을 **s 겹침**으로 좁힌 이유: s 가 떨어진 쌍은 대각선 통과가
    // 실재하고(위 run_0806_234734 주석), 그 판정은 이미 BlendSpeedCap 이
    // 한다. 여기서 s 겹침만 다루면 그 경로를 건드리지 않는다.
    for (size_t i = 0; i < obs.size(); ++i) {
        for (size_t j = i + 1; j < obs.size(); ++j) {
            auto &a = obs[i];
            auto &b = obs[j];
            if (!std::isfinite(a.s_start) || !std::isfinite(a.s_end) ||
                !std::isfinite(b.s_start) || !std::isfinite(b.s_end) ||
                !std::isfinite(a.d_left) || !std::isfinite(a.d_right) ||
                !std::isfinite(b.d_left) || !std::isfinite(b.d_right)) {
                continue;
            }
            const double ai = Mod(a.s_start - ref, track_len_);
            const double bi = ai + Mod(a.s_end - a.s_start, track_len_);
            const double aj = Mod(b.s_start - ref, track_len_);
            const double bj = aj + Mod(b.s_end - b.s_start, track_len_);
            if (std::max(ai, aj) - std::min(bi, bj) > 0.0) {
                continue;      // s 가 안 겹친다 — 대각 통과 여지를 남긴다
            }
            // corridor_top 은 "왼쪽 박스의 아래 끝", corridor_bot 은
            // "오른쪽 박스의 위 끝"이다 (d_left > d_right 규약).
            const double corridor_top = std::max(a.d_right, b.d_right);
            const double corridor_bot = std::min(a.d_left, b.d_left);
            const double width = corridor_top - corridor_bot;
            if (!(width > 0.0)) {
                continue;      // d 가 이미 겹친다 — 사이에 구멍이 없다
            }
            // 통과 가능 조건은 "경로 중심이 양쪽 요구 간격을 동시에 만족"이라
            // 필요 폭은 두 요구치의 **합**이다 (MergeBlocksTrack 이 쓰는
            // max() 는 한쪽 벽까지의 거리를 재는 식이라 여기엔 안 맞는다).
            const double need = RequiredObstacleClearance(a) +
                                RequiredObstacleClearance(b);
            if (width >= need) {
                continue;      // 진짜 지나갈 수 있다 — 손대지 않는다
            }
            const double mid = 0.5 * (corridor_top + corridor_bot);
            if (a.d_right > b.d_right) {
                a.d_right = mid;   // a 가 왼쪽 박스
                b.d_left = mid;
            } else {
                b.d_right = mid;   // b 가 왼쪽 박스
                a.d_left = mid;
            }
            a.d_center = 0.5 * (a.d_left + a.d_right);
            b.d_center = 0.5 * (b.d_left + b.d_right);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "사이 통로 봉쇄: id %d+%d 사이 %.2f m < 필요 %.2f m — "
                "두 박스를 d=%.2f 까지 늘림",
                a.id, b.id, width, need, mid);
            NoteOutcome("corridor_gap_sealed");
        }
    }
    return obs;
}

std::vector<f1tenth_planning_msgs::msg::Obstacle> SplinerNode::ObstaclesInLookahead(
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> &obstacles) const {
    std::vector<f1tenth_planning_msgs::msg::Obstacle> out;
    for (const auto &raw : obstacles) {
        if (!std::isfinite(raw.s_start) || !std::isfinite(raw.s_end)) {
            continue;
        }
        auto obs = raw;
        double span = Mod(obs.s_end - obs.s_start, track_len_);
        if (span < 1e-3 && obs.size > 0.0) {
            span = obs.size;
            obs.s_start = Mod(obs.s_center - span / 2.0, track_len_);
            obs.s_end = Mod(obs.s_center + span / 2.0, track_len_);
        }
        const double margin = vehicle_half_length_m_ +
            UncertaintyMargin(obs.s_var, 0.0, 0.0);
        const bool ego_inside =
            Mod(cur_s_ - obs.s_start, track_len_) <= span + margin;
        const double start_ahead = Mod(obs.s_start - cur_s_, track_len_);
        // 장애물 몸통/불확실성 일부라도 lookahead 안이면 검증 대상.
        if (ego_inside || start_ahead <= lookahead_m_ + margin) {
            out.push_back(obs);
        }
    }
    return out;
}

int SplinerNode::ObsSeen(int32_t id) const {
    auto it = cls_tracks_.find(id);
    return (it == cls_tracks_.end())
        ? 0 : static_cast<int>(it->second.s_hist.size());
}

bool SplinerNode::WasEverDynamic(int32_t id) const {
    auto it = cls_tracks_.find(id);
    return it != cls_tracks_.end() && it->second.ever_dynamic;
}

bool SplinerNode::WasEverMoved(int32_t id) const {
    if (!(ever_moved_min_disp_m_ > 0.0)) {
        return false;   // 기능 꺼짐 = 종전 동작
    }
    auto it = cls_tracks_.find(id);
    return it != cls_tracks_.end() &&
           it->second.max_disp >= ever_moved_min_disp_m_;
}

double SplinerNode::OpponentFollowCap(
    const f1tenth_planning_msgs::msg::Obstacle &opp) const {
    constexpr double kNoCap = std::numeric_limits<double>::infinity();
    if (!(track_len_ > 0.0) || !std::isfinite(opp.s_start)) {
        return kNoCap;
    }
    // 상대가 앞에 있는가. Mod 는 항상 양수라 "바로 뒤"가 "한 바퀴 앞"으로
    // 보이므로 반 바퀴로 자른다.
    const double gap = Mod(opp.s_start - cur_s_, track_len_);
    if (!std::isfinite(gap) || gap > track_len_ * 0.5) {
        return kNoCap;
    }
    // 목표 속도에는 관측 창 평균을 쓴다 — 단일 프레임 vs 는 정지가 증명된
    // 트랙에서도 std 0.43~0.45 로 흔들린다 (run_0815). 그 값을 캡에 그대로
    // 넣으면 명령이 매 사이클 요동친다.
    const double v_opp = std::max(
        trailing_vs_filter_ ? ObsVsMean(opp.id, opp.vs) : opp.vs, 0.0);
    const double v_ego = std::max(std::abs(cur_vs_), 0.0);
    const double a = std::max(plant_decel_mps2_, 0.1);
    // 반응지연: 인지-계획 지연 + 명령->실제 감속 사이의 구동계 지연.
    const double t_react =
        std::max(planning_delay_sec_, 0.0) + std::max(trailing_lag_sec_, 0.0);

    // ── 개입 게이트 ────────────────────────────────────────────────────
    // "gap_target 밖은 신경 쓰지 않는다"를 물리적으로 성립하게 만든 형태.
    // 정속 추종(dv≈0)이면 engage ≈ gap_target + v·T 라 사실상 목표 간격에서
    // 열리고, 급접근일 때만 미리 열린다.
    const double dv = std::max(v_ego - v_opp, 0.0);
    const double engage =
        dyn_follow_gap_m_ + dv * dv / (2.0 * a) + v_ego * t_react;
    if (!std::isfinite(engage) || gap > engage) {
        return kNoCap;
    }

    // ── (1) 비례 추종 ──────────────────────────────────────────────────
    double cap = v_opp + dyn_follow_kp_ * (gap - dyn_follow_gap_m_);
    // ── (2) 안전 상한 ──────────────────────────────────────────────────
    // 반응지연 동안 굴러갈 거리를 먼저 빼고 남는 것만 제동 거리로 친다
    // (BuildTrailing 의 reachable_v 와 같은 형태).
    const double room =
        std::max(gap - dyn_follow_gap_min_m_ - v_ego * t_react, 0.0);
    const double v_safe = std::sqrt(v_opp * v_opp + 2.0 * a * room);
    if (std::isfinite(v_safe)) {
        cap = std::min(cap, v_safe);
    }
    return std::isfinite(cap) ? std::max(cap, 0.0) : kNoCap;
}

double SplinerNode::ObsVsMean(int32_t id, double fallback) const {
    auto it = cls_tracks_.find(id);
    if (it == cls_tracks_.end() || it->second.vs_hist.empty()) {
        return fallback;
    }
    double sum = 0.0;
    for (double v : it->second.vs_hist) {
        if (!std::isfinite(v)) {
            return fallback;   // 이력이 오염됐으면 창 평균을 믿지 않는다
        }
        sum += v;
    }
    return sum / static_cast<double>(it->second.vs_hist.size());
}

std::vector<f1tenth_planning_msgs::msg::Obstacle> SplinerNode::StableObstacles(
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> &obstacles) const {
    std::vector<f1tenth_planning_msgs::msg::Obstacle> out;
    out.reserve(obstacles.size());
    for (const auto &o : obstacles) {
        // 외삽(coasting)만으로는 이미 성립한 회피를 못 깬다. 관측 이력이
        // 충분해도 **지금 안 보이는** 물체는 위치가 KF 예측이라, 커밋 경로와의
        // 충돌 판정이 근거 없이 회피를 정지로 되돌린다 (rosbag_0811 실측).
        if (commit_break_requires_visible_ && !o.is_visible) {
            continue;
        }
        if (ObsSeen(o.id) >= commit_break_min_obs_) {
            out.push_back(o);
        }
    }
    return out;
}

bool SplinerNode::CheckPathCollision(
    const std::vector<f1tenth_planning_msgs::msg::Wpnt> &path,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> &obstacles,
    double *blocking_s,
    const f1tenth_planning_msgs::msg::Obstacle **blocking_obs,
    double *contact_slack) const {
    if (blocking_obs != nullptr) {
        *blocking_obs = nullptr;
    }
    if (contact_slack != nullptr) {
        *contact_slack = std::numeric_limits<double>::infinity();
    }
    if (path.empty() || !(track_len_ > 0.0)) {
        return false;
    }

    // 각 경로 점의 도착시각. 첫 점이 ego 앞에서 시작하는 free spline은 그
    // 거리도 포함하며, 계획/제어 지연 동안 장애물이 계속 움직인다고 본다.
    std::vector<double> arrival(path.size(), planning_delay_sec_);
    double first_dist = Mod(path.front().s_m - cur_s_, track_len_);
    if (first_dist > track_len_ / 2.0) {
        first_dist = 0.0;  // 현재 pose anchor의 수치 wrap
    }
    double speed0 = std::max(std::abs(cur_vs_), 0.25);
    arrival.front() += first_dist / speed0;
    const double speed_scale = std::max(cmd_speed_scale_, 0.05);
    for (size_t i = 1; i < path.size(); ++i) {
        const double ds = std::hypot(
            path[i].x_m - path[i - 1].x_m,
            path[i].y_m - path[i - 1].y_m);
        const double v0 = std::max(path[i - 1].vx_mps * speed_scale, 0.25);
        const double v1 = std::max(path[i].vx_mps * speed_scale, 0.25);
        arrival[i] = arrival[i - 1] + 2.0 * ds / (v0 + v1);
    }

    // ── blocking_s 는 **최근접** 차단점이어야 한다 (2026-08-17) ─────────────
    // 예전에는 첫 충돌에서 곧바로 return 했다. 그런데 바깥 루프는 obstacles
    // 벡터 순서로 도는데 그 순서는 ClusterObstacles 출력이지 s 정렬이 아니다
    // — 즉 blocking_s 가 더 **먼** 장애물로 잡힐 수 있었다. 호출부
    // (BuildTrailing 의 트레일링 램프 충돌검사)는 이 값으로
    //   stop_dist = Mod(blocking_s - cur_s_) - trailing_gap_m_
    // 를 만들어 제동 곡선을 씌우므로, 먼 값이 잡히면 **제동이 모자란다**.
    // 최근접을 고르는 쪽이 항상 더 이른 제동이라 방향이 안전하다.
    // blocking_s 를 안 받는 호출부(순수 가/부 판정)는 종전처럼 즉시 반환한다.
    bool collided = false;
    double nearest_ahead = std::numeric_limits<double>::infinity();

    // 경로 각 점과 그 도착시각의 동적 장애물 확장 박스를 비교한다.
    for (const auto &obs : obstacles) {
        if (!std::isfinite(obs.s_start) || !std::isfinite(obs.s_end) ||
            !std::isfinite(obs.d_right) || !std::isfinite(obs.d_left) ||
            !std::isfinite(obs.vs) || !std::isfinite(obs.vd)) {
            // 비정상 장애물을 "없음"으로 취급하면 fail-open이므로 후보를 거부.
            return false;
        }
        double base_span = Mod(obs.s_end - obs.s_start, track_len_);
        if (base_span < 1e-3 && obs.size > 0.0) {
            base_span = obs.size;
        }
        for (size_t i = 0; i < path.size(); ++i) {
            const double t = arrival[i];
            const double motion_s = obs.is_static ? 0.0 : obs.vs * t;
            const double motion_d = obs.is_static ? 0.0 : obs.vd * t;
            const double s_var =
                (std::isfinite(obs.s_var) && obs.s_var > 0.0) ? obs.s_var : 0.0;
            const double d_var =
                (std::isfinite(obs.d_var) && obs.d_var > 0.0) ? obs.d_var : 0.0;
            const double vs_var =
                (std::isfinite(obs.vs_var) && obs.vs_var > 0.0) ? obs.vs_var : 0.0;
            const double vd_var =
                (std::isfinite(obs.vd_var) && obs.vd_var > 0.0) ? obs.vd_var : 0.0;
            // 정적 장애물의 시간 성장 불확실성 캡 (FilterObstacles와 동일 근거):
            // vd_var 0.25 상수 x t^2가 3σ_d≈1.5t를 만들어, 정지 출발처럼
            // 도착시각이 긴 경로는 요구 여유가 트랙 폭을 넘어 전부 거부됐다.
            const double uncertainty_t =
                obs.is_static ? std::min(t, static_time_uncertainty_cap_) : t;
            const double sigma_s = UncertaintyMargin(s_var, vs_var, uncertainty_t);
            const double sigma_d = UncertaintyMargin(d_var, vd_var, uncertainty_t);
            const double longitudinal_margin = vehicle_half_length_m_ + sigma_s;
            const double expanded_span =
                base_span + 2.0 * longitudinal_margin;
            const double expanded_start =
                Mod(obs.s_start + motion_s - longitudinal_margin, track_len_);
            const bool longitudinal_overlap =
                expanded_span >= track_len_ ||
                Mod(path[i].s_m - expanded_start, track_len_) <= expanded_span;
            if (!longitudinal_overlap) {
                continue;
            }
            const double lateral_margin =
                std::max(path_check_margin_, vehicle_half_width_m_) + sigma_d;
            const double d_right =
                obs.d_right + motion_d - lateral_margin;
            const double d_left =
                obs.d_left + motion_d + lateral_margin;
            if (path[i].d_m >= d_right && path[i].d_m <= d_left) {
                collided = true;
                if (blocking_s == nullptr) {
                    return false;  // 충돌 — 지점이 필요 없으면 즉시
                }
                // 이 장애물에 대해 처음 맞은 점이 도착시각이 가장 이르므로
                // motion_s 도 이 점 기준이 맞다. 다음 장애물로 넘어간다.
                const double s_blocking = Mod(obs.s_start + motion_s, track_len_);
                const double ahead = Mod(s_blocking - cur_s_, track_len_);
                if (ahead < nearest_ahead) {
                    nearest_ahead = ahead;
                    *blocking_s = s_blocking;
                    if (blocking_obs != nullptr) {
                        *blocking_obs = &obs;
                    }
                    if (contact_slack != nullptr) {
                        // 확장 박스가 아니라 **관측된 장애물 자체**를 기준으로
                        // 잰다. inside > 0 이면 경로가 장애물 안이고, 차체는
                        // 그보다 반폭만큼 먼저 닿는다.
                        const double inside = std::min(
                            path[i].d_m - (obs.d_right + motion_d),
                            (obs.d_left + motion_d) - path[i].d_m);
                        *contact_slack = -inside - vehicle_half_width_m_;
                    }
                }
                break;
            }
        }
    }
    return !collided;
}

bool SplinerNode::PlanSpanAllowed(double span) const {
    if (!std::isfinite(span) || !(span > 0.0) || !(track_len_ > wpnt_dist_)) {
        return false;
    }
    // front/back modulo 두 점으로 span을 복원하는 downstream 계약 때문에
    // 정확히 한 바퀴인 경로도 0m로 보인다. 최소 한 sample 여유를 둔다.
    return span < track_len_ - std::max(wpnt_dist_, 0.05);
}

double SplinerNode::TrackingReserve(double speed) const {
    const double v = std::isfinite(speed) ? std::abs(speed) : 0.0;
    const double extra = std::max(boundary_tracking_speed_gain_, 0.0) *
                         std::max(v - boundary_tracking_speed_ref_, 0.0);
    // 상한은 생성자가 이미 max(margin_max, margin_m) 로 올려 맞춰 두므로
    // 여기서 다시 max 를 취할 필요가 없다 (불변식: margin_max >= margin_m).
    return std::min(boundary_tracking_margin_m_ + extra,
                    boundary_tracking_margin_max_);
}

double SplinerNode::ArrivalSpeed(double dist_ahead) const {
    // dist_ahead 앞 지점에 **도달했을 때** 차가 낼 속도 [m/s].
    //
    // 감속 크레딧이 0 이면 PlanSpeed() 를 그대로 돌려준다 = 예전 동작.
    // 크레딧이 있으면 그 감속으로 낼 수 있는 가장 낮은 속도를 쓴다. 상한은
    // 파라미터 검증이 plant_decel_mps2_ 로 묶어 두므로, "브레이크가 실제보다
    // 잘 듣는다"고 주장할 수는 없다.
    const double v0 = PlanSpeed();
    const double a = boundary_arrival_decel_mps2_;
    if (!(a > 0.0) || !std::isfinite(dist_ahead) || dist_ahead <= 0.0) {
        return v0;
    }
    const double v2 = v0 * v0 - 2.0 * a * dist_ahead;
    return std::max(std::sqrt(std::max(v2, 0.0)), plan_speed_floor_);
}

double SplinerNode::RequiredBoundaryMarginAt(
    double requested_margin, double dist_ahead) const {
    // ── 벽 여유는 "지금 속도"가 아니라 "그 자리를 지날 때 속도"로 재야 한다 ──
    //
    // TrackingReserve 는 추종오차를 덮는 예비인데, 덮어야 할 오차는 차가
    // **그 지점 옆을 지나는 동안**의 오차다. 그런데 여기 들어가던 값은 늘
    // PlanSpeed() = 계획 시점의 현재 속도였다. 감속하며 접근하는 상황에서는
    // 도착 속도가 훨씬 낮은데도 지금 속도로 예비를 잡으므로, 예비가 과대해져
    // 통로를 스스로 닫는다.
    //   run_0819_180345 t=267.05 실측: 좌벽 1.145, 장애물 왼쪽 끝 0.51.
    //   필요 d = 0.51 + 0.27 = 0.781. 현재 v=5.53 -> 예비 0.22(상한) ->
    //   허용 상한 0.775 = **6 mm 부족**으로 좌측 통로가 닫혔다. 그 6 mm 때문에
    //   목표가 반대쪽(-0.07)으로 뒤집혔고, 0.65 초 뒤 장애물 정중앙에 4.0 m/s
    //   로 부딪혔다. 같은 구간에서 차는 이미 2.29 m/s^2 로 감속 중이었고,
    //   도착 속도는 4.1~4.4 m/s 였다 — 그 값으로 재면 상한이 0.846 이라
    //   65 mm 여유로 열린다.
    // 같은 결함을 BuildLaneSwitch 는 이미 BlendPlanSpeed 로 고쳤다("비교 대상은
    // '지금 속도'가 아니라 '진입 구간을 지나는 동안 낼 속도'다"). 여기만 옛
    // 형태로 남아 있었다.
    //
    // ⚠️ 이것은 **완화다** — 한 방향으로만 움직인다. run_0819_180345 재구성
    //    (lookahead 내 6094 검사, 크레딧 2.0): 통로 열림 60.5% -> 62.6%,
    //    막힘->열림 128 건(2.1%), 열림->막힘 0 건. 벽 여유 감소는 중앙 2 mm /
    //    p90 96 mm / 최대 120 mm 이고 프레임의 49% 는 정확히 0 이다.
    //    (최대 120 mm = 예비 상한 0.22 와 기본값 0.10 의 차 전폭.)
    //
    // 이 완화가 안전한 이유는 **크레딧의 모양**에 있다. sqrt(v^2 - 2*a*d) 는
    // 거리가 짧을수록 지금 속도에 수렴하므로, 여유를 실제로 지켜야 하는
    // 근접 구간에서는 크레딧이 사라진다. 같은 재구성의 거리별 감소 최대치:
    //     0~1 m  42 mm |  1~2 m  79 mm |  2~3 m 111 mm |  4 m 이상 120 mm
    // 즉 전폭 크레딧은 4 m 이상 남았을 때(= 5.5 m/s 에서 0.7 초 이상 제동
    // 시간이 있을 때)만 나오고, 그 판정은 아직 예비 결정이다.
    //
    // ⚠️ 남는 가정: "접근 중 실제로 이 감속률로 줄인다". 강제되지는 않는다
    //    (여기서는 속도 프로파일을 아직 모른다). 근거는 실측이다 —
    //    run_0819_180345 t=267.00~267.70 트레일링 접근의 실측 감속이
    //    2.29 m/s^2 였다. 가정이 깨지면 벽 예비가 위 표만큼 얇아진다.
    //    되돌리려면 boundary_arrival_decel_mps2 를 0 으로 두면 된다 —
    //    그때 이 함수는 종전과 비트 단위로 같다.
    return std::max(
        std::max(requested_margin, 0.0),
        vehicle_half_width_m_ + TrackingReserve(ArrivalSpeed(dist_ahead)));
}

double SplinerNode::RequiredBoundaryMargin(double requested_margin) const {
    // 거리 정보가 없는 호출부는 종전대로 현재 속도로 잰다 (보수적).
    return RequiredBoundaryMarginAt(requested_margin, 0.0);
}

bool SplinerNode::WithinTrackBoundary(
    int gb_idx, double d, double requested_margin) const {
    if (gb_idx < 0 || gb_idx >= static_cast<int>(gb_wpnts_.size()) ||
        !std::isfinite(d)) {
        return false;
    }
    const double margin = RequiredBoundaryMargin(requested_margin);
    const double d_min = -std::abs(gb_wpnts_[gb_idx].d_right) + margin;
    const double d_max = std::abs(gb_wpnts_[gb_idx].d_left) - margin;
    return d_min <= d_max && d >= d_min && d <= d_max;
}

double SplinerNode::SteerKappaMax() const {
    // kinematic 자전거모델의 기하 한계에 실차 보정 배율을 곱한다.
    // 배율 1.0(기본)이면 예전 식과 완전히 동일하다.
    return steer_kappa_scale_ *
           std::tan(std::abs(vehicle_max_steering_rad_)) /
           std::max(vehicle_wheelbase_m_, 1e-3);
}

bool SplinerNode::BlendGeomFeasible(double amplitude, double blend_len,
                                    double s_from, bool one_sided) const {
    const int gb_size = static_cast<int>(gb_wpnts_.size());
    if (gb_size < 1 || !(track_len_ > 0.0) || !(blend_len > 1e-3) ||
        !std::isfinite(blend_len) || !std::isfinite(s_from) || s_from < 0.0) {
        return false;
    }
    const double kappa_steer_max = SteerKappaMax();
    if (kappa_steer_max <= 1e-3) {
        return false;
    }
    // 근사 여유는 기동 항에만 (헤더 주석). 트랙 곡률은 CSV 값 그대로 쓴다.
    const double inv_margin = 1.0 / std::max(geom_floor_margin_, 1e-3);
    // 상쇄를 얼마나 믿을 것인가 — 헤더 주석 참조.
    const double credit_cap =
        std::max(1.0 - geom_floor_kappa_ratio_, 0.0) * kappa_steer_max;
    const int steps = std::max(
        1, static_cast<int>(blend_len / std::max(wpnt_dist_, 0.01)));
    const double inv_len2 = 1.0 / (blend_len * blend_len);
    for (int i = 0; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const double s = Mod(s_from + t * blend_len, track_len_);
        const int gi = static_cast<int>(s / std::max(wpnt_dist_, 1e-3)) % gb_size;
        double kt = gb_wpnts_[gi].kappa_radpm;
        if (!std::isfinite(kt)) {
            kt = 0.0;
        }
        // 기동 곡률: smoothstep d'' = A(6-12t)/L², 한쪽 원호 d'' = 2A/L².
        // 부호를 그대로 살린다 — 이게 스칼라 식과의 유일한 실질 차이다.
        const double km = one_sided
            ? (2.0 * amplitude * inv_len2)
            : (amplitude * (6.0 - 12.0 * t) * inv_len2);
        // 부호가 반대(= 코너가 기동을 상쇄)일 때만 크레딧을 제한한다.
        // 같은 부호면 트랙 곡률을 그대로 더해야 보수적이다.
        double kt_eff = kt;
        if (kt * km < 0.0) {
            const double credit = std::min(std::abs(kt), credit_cap);
            kt_eff = (kt >= 0.0) ? credit : -credit;
        }
        if (std::abs(kt_eff + km * inv_margin) > kappa_steer_max) {
            return false;
        }
    }
    return true;
}

double SplinerNode::BlendGeomFloor(double amplitude, double kappa_track,
                                   bool one_sided, double s_from) const {
    // 그립 예산은 저속에서 사실상 무한대가 된다(a_lat/v²) — 하지만 조향각
    // 한계라는 순수 기하 제약은 속도와 무관하게 남는다. 피크 기하곡률
    // 6A/L²이 조향 한계를 넘으면 ApplySpeedProfile이 경로를 통째로 거부하고,
    // 저속/정지에서는 감속으로 살릴 수도 없다 (run_0806_155621 정지 라치).
    // geom_floor_margin_ 여유는 실제 샘플 곡률이 smoothstep 근사보다 큰 것을
    // 흡수한다.
    //
    // 조향은 트랙 곡률과 기동 곡률을 합쳐서 낸다:
    //   6A/L² + |kappa_track| <= geom_floor_margin_*kappa_steer_max
    // 예전 식은 kappa_track=0을 가정해 헤어핀에서 하한을 과소평가했다
    // (0807 run_164216: kappa 0.74, A 0.49 -> 참 하한 2.23~2.45m를 1.80m로).
    const double a = std::abs(amplitude);
    if (a < 1e-3) {
        return 0.0;
    }
    const double kappa_steer_max = SteerKappaMax();
    if (kappa_steer_max <= 1e-3) {
        return 0.0;
    }
    // ── s 문맥이 있으면 지점별로 최소 길이를 찾는다 ──────────────────────
    // 실현 가능성은 L 에 대해 단조다: 기동 항 |d''| 는 1/L² 로 줄고, 트랙
    // 곡률만으로는 한계를 못 넘는다(레이스라인이 validation_max_steer_rad 로
    // 이미 검증돼 있다). 따라서 이분법이 성립한다.
    if (s_from >= 0.0 && std::isfinite(s_from) && !gb_wpnts_.empty() &&
        track_len_ > 0.0) {
        // ── 호출부가 넘긴 kappa_track 을 버리지 않는다 ──────────────────────
        // BlendGeomFeasible 은 [s_from, s_from+L] (= 진입 블렌드)만 훑고
        // 레이스라인 **원본** 곡률만 본다. 그런데 호출부(BuildLaneSwitch 의
        // kappa_in / DoSpline 의 kappa_sp)가 넘기는 값은 진입 구간 원본과
        // **hold 구간의 오프셋 유효 곡률 OffsetKappa(k, d_hold)** 의 max 다.
        // 그대로 두면 "그 오프셋을 그 구간에서 유지할 수 있는가"라는 별개
        // 가부 조건이 s 문맥 경로에서 통째로 사라진다 (hold 구간은 스캔 범위
        // 밖이라 좌표상으로도 안 보인다).
        // 오프셋 유지는 블렌드 길이로 해결되는 문제가 아니므로 여기서 먼저
        // 자른다. 문턱은 ApplySpeedProfile 의 곡률 기각 문턱과 **같은 값**이라,
        // 이 게이트가 다운스트림 검증기보다 엄격해지지는 않는다 — 즉 지금
        // 통과하던 경로를 새로 죽이지 않고, 죽을 경로를 더 일찍 죽인다.
        if (std::isfinite(kappa_track) &&
            std::abs(kappa_track) >
                kappa_steer_max * std::max(steer_kappa_reject_ratio_, 1.0)) {
            return blend_dist_max_m_;
        }
        if (!BlendGeomFeasible(amplitude, blend_dist_max_m_, s_from,
                               one_sided)) {
            // 아무 길이로도 안 되면 스칼라 경로와 같은 sentinel 을 돌려
            // 상위가 "가용보다 큼"으로 읽게 한다.
            return blend_dist_max_m_;
        }
        double lo = 0.0, hi = blend_dist_max_m_;
        for (int it = 0; it < 20 && (hi - lo) > 0.02; ++it) {
            const double mid = 0.5 * (lo + hi);
            if (BlendGeomFeasible(amplitude, mid, s_from, one_sided)) {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        return hi;
    }
    // geom_floor_margin_ 여유는 "smoothstep 근사 6A/L²보다 실제 샘플 피크가
    // 크다"를 흡수하는 것이므로 **기동 항에만** 적용한다. 트랙 곡률은 CSV에서
    // 온 정확한 값이라 그대로 빼면 된다:
    //   (1/m)*6A/L² + |kappa_track| <= kappa_steer_max
    //   -> L >= sqrt(6A / (m*(kappa_steer_max - |kappa_track|)))
    // kappa_track=0이면 예전 식과 정확히 같다 (직선 구간 호환).
    // 트랙 곡률이 조향 능력을 거의 다 먹으면 하한이 발산한다 — 그 구간은
    // 회피가 성립하지 않는 게 맞지만, inf를 돌려주면 상위 로그가 읽을 수
    // 없어지므로 blend_dist_max_m_로 잘라 "가용보다 큼"으로 만든다.
    const double spare = geom_floor_margin_ * std::max(
        kappa_steer_max - std::abs(kappa_track),
        geom_floor_kappa_ratio_ * kappa_steer_max);
    // 형상 계수: smoothstep(평행 복귀) 피크 곡률 6A/L², 한쪽 원호 2A/L².
    const double shape = one_sided ? 2.0 : 6.0;
    return std::min(std::sqrt(shape * a / spare), blend_dist_max_m_);
}

void SplinerNode::CommitOtSide(
    const std::string &side,
    const f1tenth_planning_msgs::msg::Obstacle &obs) {
    last_ot_side_ = side;
    // 스탬프는 s_center 유효성과 무관하게 **항상** 갱신한다 — 이 시각이
    // OtSideCommitFresh 의 기준(= 마지막으로 이 사이드로 회피 경로를 만든
    // 시각)이므로, s_center 가 NaN 인 프레임에서 갱신을 건너뛰면 진행 중인
    // 회피가 만료된 것으로 보인다. 같은-장애물 잠금(has_ot_side_obs_)만
    // s_center 에 의존한다.
    ot_side_stamp_ = this->now();
    if (std::isfinite(obs.s_center)) {
        ot_side_obs_s_ = Mod(obs.s_center, std::max(track_len_, 1e-3));
        has_ot_side_obs_ = true;
    } else {
        has_ot_side_obs_ = false;
    }
}

bool SplinerNode::OtSideCommitFresh() const {
    if (last_ot_side_.empty()) {
        return false;
    }
    const double age = (this->now() - ot_side_stamp_).seconds();
    return age >= 0.0 && age <= kOtSideCommitValidSec;
}

bool SplinerNode::SideKeepActive(
    const f1tenth_planning_msgs::msg::Obstacle &obs) const {
    // 커밋이 만료됐으면 유지할 사이드가 없다 (헤더 OtSideCommitFresh 주석).
    if (!OtSideCommitFresh()) {
        return false;
    }
    // (1) 같은 장애물을 계속 회피하는 중이면 |cur_d_|와 무관하게 유지한다.
    //     예전 조건(|cur_d_| > 문턱)은 **접근 구간**에 히스테리시스를 전혀
    //     주지 않았다 — 차가 아직 라인 위(|d| ~ 0.05)일 때가 정확히 좌우가
    //     뒤집히는 구간이다. 0808 run_185049에서 같은 장애물의 관측 박스
    //     폭이 0.28~1.25m로 흔들리며 left/right 판정이 프레임마다 바뀌었고,
    //     그 결과 통과 직후 "사이드 전환 (left -> right)"이 4랩에서 발생해
    //     로컬 경로가 반대편으로 점프했다(control 추종 백스톱 경로오차
    //     0.37~0.63m -> 속도 1.5 m/s 캡, 통과 후 0.7~3.1초 크롤).
    //     detector가 장애물 id를 초당 ~1.7개꼴로 갈아치우므로 동일성은
    //     id가 아니라 s 근접으로 본다.
    if (has_ot_side_obs_ && std::isfinite(obs.s_center) && track_len_ > 0.0) {
        double ds = Mod(obs.s_center - ot_side_obs_s_, track_len_);
        if (ds > track_len_ / 2.0) {
            ds -= track_len_;
        }
        const double age = (this->now() - ot_side_stamp_).seconds();
        if (std::abs(ds) <= kOtSideLockDistM &&
            age >= 0.0 && age <= kOtSideLockSec) {
            return true;
        }
    }
    // (2) 기존 조건: 이미 그 사이드로 충분히 나가 있으면 유지.
    //     d 부호가 유지 사이드와 같아야 "그 사이드로 회피 중"이다.
    //     (d>0 = 왼쪽, d<0 = 오른쪽 — Wpnt.d_left/d_right와 같은 부호 규약)
    if (std::abs(cur_d_) <= side_switch_d_tresh_) {
        return false;
    }
    return (cur_d_ > 0.0) ? (last_ot_side_ == "left")
                          : (last_ot_side_ == "right");
}

f1tenth_planning_msgs::msg::Wpnt SplinerNode::MakeBaseWpnt(
    int idx, double s, double d, double x, double y, int gb_idx) const {
    f1tenth_planning_msgs::msg::Wpnt w;
    w.id = static_cast<int32_t>(idx);
    w.s_m = s;
    w.d_m = d;
    w.x_m = x;
    w.y_m = y;
    // 부호 규약: d > 0 = 좌측. 좌측 여유는 줄고 우측 여유는 늘어난다.
    // (WithinTrackBoundary 의 d_min/d_max 와 같은 규약이어야 한다.)
    w.d_left = gb_wpnts_[gb_idx].d_left - d;
    w.d_right = gb_wpnts_[gb_idx].d_right + d;
    // 레이스라인 헤딩/곡률을 그대로 싣는다. 오프셋 경로의 실제 헤딩이
    // 필요한 호출부는 인접 점으로 psi_rad 를 다시 계산한다.
    w.psi_rad = gb_wpnts_[gb_idx].psi_rad;
    w.kappa_radpm = gb_wpnts_[gb_idx].kappa_radpm;
    w.ax_mps2 = gb_wpnts_[gb_idx].ax_mps2;
    return w;
}

double SplinerNode::BlendKappaMax(double s_from, double s_to,
                                  double d_offset) const {
    const int gb_size = static_cast<int>(gb_wpnts_.size());
    if (gb_size < 1 || !(track_len_ > 0.0) ||
        !std::isfinite(s_from) || !std::isfinite(s_to)) {
        return 0.0;
    }
    double span = Mod(s_to - s_from, track_len_);
    span = std::min(span, track_len_ - wpnt_dist_);
    const int steps = std::max(
        1, static_cast<int>(span / std::max(wpnt_dist_, 0.01)));
    const int start_idx =
        static_cast<int>(Mod(s_from, track_len_) / wpnt_dist_) % gb_size;
    const double d = std::isfinite(d_offset) ? d_offset : 0.0;
    double worst = 0.0;
    for (int i = 0; i <= steps; ++i) {
        const auto &w = gb_wpnts_[(start_idx + i) % gb_size];
        if (std::isfinite(w.kappa_radpm)) {
            // d_offset=0 이면 레이스라인 곡률 그대로(기존 동작).
            // 오프셋을 주면 그 오프셋 경로가 **실제로** 그리는 곡률을 본다:
            // 코너 바깥으로 비키면 반경이 커져 곡률이 줄고, 안쪽으로 비키면
            // 커진다 (OffsetKappa = k / (1 - d*k), offset_grip_cap과 같은 물리).
            worst = std::max(worst,
                             std::abs(OffsetKappa(w.kappa_radpm, d)));
        }
    }
    return worst;
}

double SplinerNode::BlendDist(
    double amplitude, double speed, double lat_acc, double kappa_track,
    double s_from) const {
    // smoothstep 횡이동의 피크 횡가속 = 6*A*v^2/L^2 이므로
    // L = v*sqrt(6A/a_max). 거리를 상수로 두면 v^2로 커진다.
    const double a = std::abs(amplitude);
    const double v = std::max(std::abs(speed), 0.0);
    const double a_lat = (lat_acc > 0.0) ? lat_acc : blend_lat_acc_max_;
    if (a < 1e-3 || a_lat < 1e-3) {
        return 0.0;
    }
    double dist = v * std::sqrt(6.0 * a / a_lat);
    // 진폭은 **부호를 살려** 넘긴다 — 지점별 경로가 방향을 봐야 한다.
    // (예전에는 여기서 std::abs 로 눌러 버려 부호가 도달하지 못했다.)
    dist = std::max(dist, BlendGeomFloor(amplitude, kappa_track, false, s_from));
    return std::min(dist, blend_dist_max_m_);
}

double SplinerNode::BlendSpeedCap(
    double amplitude, double available, double lat_acc,
    double kappa_track, double s_from) const {
    const double a = std::abs(amplitude);
    // 옮길 횡거리가 사실상 0이면 속도 제약이 없다.
    if (a < 1e-3) {
        return std::numeric_limits<double>::infinity();
    }
    const double a_lat = (lat_acc > 0.0) ? lat_acc : blend_lat_acc_max_;
    if (!(available > 0.0) || a_lat < 1e-3) {
        return 0.0;
    }
    // 조향 기하 하한을 못 채우면 감속으로도 살릴 수 없다 — 여기서만 정지가
    // 정당하다 (rosbag_0806 주행중 검출 32건 중 8건, 그나마 4건은 벽 유령,
    // 4건은 앞선 오작동 정지로 이미 코앞에서 검출된 경우였다).
    // s 문맥이 있으면 **실제로 쓸 길이(available)** 에서 바로 판정한다 —
    // 블렌드는 available 을 그대로 쓰므로 최소 하한을 따로 구해 비교하는 것보다
    // 정확하다 (하한 계산의 이분법 오차 0.02m 도 타지 않는다).
    if (s_from >= 0.0 && std::isfinite(s_from) && !gb_wpnts_.empty() &&
        track_len_ > 0.0) {
        // BlendGeomFloor 의 s 문맥 경로와 같은 이유로 kappa_track 을 먼저 본다
        // (hold 구간 오프셋 유지 가부는 available 로 해결되지 않는다).
        if (std::isfinite(kappa_track) &&
            std::abs(kappa_track) >
                SteerKappaMax() * std::max(steer_kappa_reject_ratio_, 1.0)) {
            return 0.0;
        }
        if (!BlendGeomFeasible(amplitude, available, s_from, false)) {
            return 0.0;
        }
    } else if (available < BlendGeomFloor(amplitude, kappa_track)) {
        return 0.0;
    }
    return available / std::sqrt(6.0 * a / a_lat);
}

double SplinerNode::BlendLatAcc(double s_from, double s_to) const {
    if (!blend_use_spare_grip_) {
        return blend_lat_acc_max_;
    }
    const int gb_size = static_cast<int>(gb_wpnts_.size());
    if (gb_size < 1 || !(track_len_ > 0.0) ||
        !std::isfinite(s_from) || !std::isfinite(s_to)) {
        return blend_lat_acc_max_;
    }
    // 레이스라인이 그 구간에서 이미 쓰고 있는 횡가속(kappa·v²)을 빼고 남는
    // 그립만 횡 전환에 쓴다. 고정 blend_lat_acc_max는 직선에서 과보수였다 —
    // rosbag_0806 글로벌 궤적 실측: 레이스라인 a_lat p10=0.08 / p50=2.99 /
    // p90=5.49, 마찰원 잔여 p50=6.73이고 트랙의 89%에서 잔여가 5.0 이상이다.
    double span = Mod(s_to - s_from, track_len_);
    span = std::min(span, track_len_ - wpnt_dist_);
    const int steps = std::max(
        1, static_cast<int>(span / std::max(wpnt_dist_, 0.01)));
    const int start_idx =
        static_cast<int>(Mod(s_from, track_len_) / wpnt_dist_) % gb_size;
    // 이번 사이클 예산(active_lat_budget_). 이 값이 레이스라인
    // 생성 마찰보다 낮으면 아래 spare 가 코너에서 0 이 되어 하한 2.0 에
    // 고착되고, 그 결과 BlendDist 가 발산해 모든 회피가 기각된다.
    const double budget = active_lat_budget_;
    double worst = budget;
    for (int i = 0; i <= steps; ++i) {
        const auto &w = gb_wpnts_[(start_idx + i) % gb_size];
        const double v_ref = w.vx_mps * std::max(cmd_speed_scale_, 0.05);
        const double lat_ref = std::abs(w.kappa_radpm) * v_ref * v_ref;
        const double spare = std::sqrt(
            std::max(budget * budget - lat_ref * lat_ref, 0.0));
        worst = std::min(worst, spare);
    }
    // 하한 2.0: 잔여가 0에 수렴하면 BlendDist가 발산해 어떤 회피도 못 만든다.
    // 그 구간은 어차피 아래 ApplySpeedProfile의 곡률 캡이 다시 잡는다.
    return std::clamp(worst, 2.0, 15.0);
}

double SplinerNode::UncertaintyMargin(
    double pos_var, double vel_var, double t) const {
    const double p = (std::isfinite(pos_var) && pos_var > 0.0) ? pos_var : 0.0;
    const double v = (std::isfinite(vel_var) && vel_var > 0.0) ? vel_var : 0.0;
    const double tt = (std::isfinite(t) && t > 0.0) ? t : 0.0;
    const double sigma = collision_sigma_ * std::sqrt(p + tt * tt * v);
    // max_uncertainty_m_로 자르는 이유는 헤더 주석 참조 — 요약하면 브릿지가
    // 첫 프레임에 붙이는 "속도 클립 한계의 제곱"(vs_var 100 / vd_var 16)이
    // 3σ 9.0m / 3.6m를 만들어, 폭 1.1~2.1m 트랙에서 모든 회피를 자기 검증기가
    // 거부하게 만들었다 (rosbag_0806 4런 68회 = 장애물 획득당 1회).
    return std::min(sigma, max_uncertainty_m_);
}

double SplinerNode::RequiredObstacleClearance(
    const f1tenth_planning_msgs::msg::Obstacle &obs) const {
    // CheckPathCollision의 lateral_margin과 같은 식으로 계산한다:
    //   max(path_check_margin, 차반폭) + collision_sigma * σ_d
    // σ_d는 관측 분산 + (도착시각 x 속도분산). 도착시각은 여기서 알 수 없으므로
    // 정적 장애물은 시간 성장 캡(static_time_uncertainty_cap_), 동적은
    // fixed_pred_time_ 을 대표 시간으로 쓴다 (등속 전파는 4/8단계에 없앴고
    // 이 값은 이제 그 대표 시간으로만 남았다) — 검증기와 완전히 같지는 않지만
    // 목표 간격이 요구치보다 작아 매번 자기 거부하는 상황은 없어진다.
    const double d_var =
        (std::isfinite(obs.d_var) && obs.d_var > 0.0) ? obs.d_var : 0.0;
    const double vd_var =
        (std::isfinite(obs.vd_var) && obs.vd_var > 0.0) ? obs.vd_var : 0.0;
    const double t = obs.is_static ? static_time_uncertainty_cap_
                                   : std::max(fixed_pred_time_, 0.0);
    return std::max(path_check_margin_, vehicle_half_width_m_) +
           UncertaintyMargin(d_var, vd_var, t);
}

double SplinerNode::PlanSpeed() const {
    const double v = std::abs(cur_vs_);
    if (!std::isfinite(v)) {
        return plan_speed_floor_;
    }
    return std::max(v, plan_speed_floor_);
}

double SplinerNode::BlendPlanSpeed(double s_from, double span) const {
    const double v0 = PlanSpeed();
    const int gb_size = static_cast<int>(gb_wpnts_.size());
    if (gb_size < 2 || !(track_len_ > 0.0) || !(span > 0.0) ||
        !std::isfinite(s_from) || !std::isfinite(span)) {
        return v0;
    }
    const double step = std::max(wpnt_dist_, 0.01);
    const double walk = std::min(span, track_len_ - step);
    const int steps = std::max(1, static_cast<int>(walk / step));
    const int start_idx =
        static_cast<int>(Mod(s_from, track_len_) / step) % gb_size;
    double v_line = v0;
    for (int i = 0; i <= steps; ++i) {
        const double vx = gb_wpnts_[(start_idx + i) % gb_size].vx_mps;
        if (std::isfinite(vx)) {
            v_line = std::max(v_line, std::min(vx, gb_vmax_));
        }
    }
    // 구동계가 그 거리 안에 실제로 낼 수 있는 속도. 트레일링 등으로 지금
    // 느리면 레이스라인 목표까지 못 올라가므로 여기서 잘린다 — 이게 없으면
    // 정지 직전에도 직선 목표속도로 블렌드를 잡아 회피가 통째로 죽는다.
    const double v_reach =
        std::sqrt(v0 * v0 + 2.0 * std::max(path_accel_max_, 0.0) * walk);
    return std::max(v0, std::min(v_line, v_reach));
}

void SplinerNode::MinWallInSpan(
    double s_from, double s_to, double &min_d_left, double &min_d_right) const {
    const int gb_size = static_cast<int>(gb_wpnts_.size());
    min_d_left = 0.0;
    min_d_right = 0.0;
    if (gb_size < 1 || !(track_len_ > 0.0) ||
        !std::isfinite(s_from) || !std::isfinite(s_to)) {
        return;
    }
    // 스팬 길이는 wrap 후 계산 — 뒤집힌 입력(s_to < s_from)도 한 바퀴로
    // 해석되지 않게 한 바퀴에서 자른다.
    double span = Mod(s_to - s_from, track_len_);
    span = std::min(span, track_len_ - wpnt_dist_);
    const int steps = std::max(
        1, static_cast<int>(span / std::max(wpnt_dist_, 0.01)));
    const int start_idx =
        static_cast<int>(Mod(s_from, track_len_) / wpnt_dist_) % gb_size;
    min_d_left = std::abs(gb_wpnts_[start_idx].d_left);
    min_d_right = std::abs(gb_wpnts_[start_idx].d_right);
    for (int i = 1; i <= steps; ++i) {
        const auto &w = gb_wpnts_[(start_idx + i) % gb_size];
        min_d_left = std::min(min_d_left, std::abs(w.d_left));
        min_d_right = std::min(min_d_right, std::abs(w.d_right));
    }
}

double SplinerNode::OffsetKappa(double kappa, double d) {
    // 기준선에서 d만큼 옮긴 경로의 곡률. d>0(왼쪽)이고 kappa>0(좌선회)이면
    // 반경이 d만큼 줄어 곡률이 커진다. 분모가 0에 가까우면(중심을 지나감)
    // 물리적으로 무의미하므로 큰 곡률로 포화시킨다.
    double denom = 1.0 - d * kappa;
    if (std::abs(denom) < 0.05) {
        return (kappa >= 0.0) ? 20.0 : -20.0;
    }
    return kappa / denom;
}

bool SplinerNode::ApplySpeedProfile(
    std::vector<f1tenth_planning_msgs::msg::Wpnt> & wpnts,
    bool require_current_feasible,
    std::vector<double> *out_geom_ceiling,
    const std::vector<double> *in_geom_ceiling,
    int *steer_reject_idx_out) const {
    speed_reject_idx_ = -1;      // 호출마다 초기화 (직전 호출 값이 새지 않게)
    if (steer_reject_idx_out != nullptr) {
        *steer_reject_idx_out = -1;
    }
    if (wpnts.size() < 2) {
        return !wpnts.empty();
    }
    const double steer_reject_ratio = steer_kappa_reject_ratio_;

    const size_t n = wpnts.size();
    const double command_scale = std::max(cmd_speed_scale_, 0.05);
    std::vector<double> curvature(n, 0.0);
    std::vector<double> speed(n, 0.0);  // 실제 차량 속도 단위
    std::vector<double> ds(n - 1, std::max(wpnt_dist_, 0.01));
    // 기준선(레이스라인) 곡률과 그때 실제로 쓰이던 횡가속. 아래 "그립 일관성
    // 캡"에서 쓴다. curvature[]/speed[]는 뒤에서 덮어써지므로 따로 보관한다.
    std::vector<double> kappa_ref(n, 0.0);
    std::vector<double> lat_ref(n, -1.0);
    // 이보다 곧으면 kappa*v^2가 그 지점의 그립을 대표하지 못한다(직선에서는
    // 0에 수렴). 직선 위 lane-change의 S자는 아래 기하곡률 + path_lat_acc_max가
    // 담당하므로 여기서는 건드리지 않는다.
    constexpr double kKappaRefMin = 0.02;  // 반경 50m

    // 커밋 재생: 기하 단계를 건너뛰고 보관해 둔 상한에서 시작한다.
    const bool reuse_ceiling =
        (in_geom_ceiling != nullptr && in_geom_ceiling->size() == n);
    for (size_t i = 0; i < n; ++i) {
        if (!std::isfinite(wpnts[i].vx_mps) || wpnts[i].vx_mps < 0.0 ||
            !std::isfinite(wpnts[i].x_m) || !std::isfinite(wpnts[i].y_m)) {
            return false;
        }
        if (reuse_ceiling) {
            // kappa_radpm 은 직전 실행이 최종 곡률로 덮어써 둔 값이다 —
            // OffsetKappa 를 다시 걸면 이중 적용이 된다.
            curvature[i] = wpnts[i].kappa_radpm;
            speed[i] = (*in_geom_ceiling)[i];
        } else {
            curvature[i] = OffsetKappa(wpnts[i].kappa_radpm, wpnts[i].d_m);
            speed[i] = wpnts[i].vx_mps * command_scale;
            kappa_ref[i] = std::abs(wpnts[i].kappa_radpm);
            if (kappa_ref[i] > kKappaRefMin) {
                lat_ref[i] = kappa_ref[i] * speed[i] * speed[i];
            }
        }
        if (i + 1 < n) {
            ds[i] = std::max(std::hypot(
                wpnts[i + 1].x_m - wpnts[i].x_m,
                wpnts[i + 1].y_m - wpnts[i].y_m), 1e-3);
        }
    }

    if (!reuse_ceiling) {
    // IQP 평행 오프셋 공식만으로는 직선 위 lane-change의 S자 곡률을 못 본다.
    // 0.3m baseline의 실제 xy 헤딩 변화가 더 크면 그 값을 사용한다.
    if (n >= 7) {
        constexpr size_t kWindow = 3;
        // 반폭 w의 대칭 윈도로 잰 기하곡률. w를 줄이면 국소적이지만 거칠어진다.
        auto k_geom_at = [&](size_t i, size_t w) {
            const double h0 = std::atan2(
                wpnts[i].y_m - wpnts[i - w].y_m,
                wpnts[i].x_m - wpnts[i - w].x_m);
            const double h1 = std::atan2(
                wpnts[i + w].y_m - wpnts[i].y_m,
                wpnts[i + w].x_m - wpnts[i].x_m);
            const double baseline = 0.5 * std::hypot(
                wpnts[i + w].x_m - wpnts[i - w].x_m,
                wpnts[i + w].y_m - wpnts[i - w].y_m);
            return WrapToPi(h1 - h0) / std::max(baseline, 1e-6);
        };
        for (size_t i = kWindow; i + kWindow < n; ++i) {
            const double k_geom = k_geom_at(i, kWindow);
            if (std::abs(k_geom) > std::abs(curvature[i])) {
                curvature[i] = k_geom;
            }
        }
        // 머리/꼬리 3점은 대칭 윈도를 못 만들어 안쪽 값을 그대로 복사한다.
        // ⚠️ 2026-08-15: 이 복사를 "들어가는 만큼 좁힌 윈도(w=1,2)"로 바꿔
        //    보려다 되돌렸다. wpnt 간격이 0.1 m 라 w=1 은 baseline 0.2 m 인데
        //    k = Δheading/baseline 의 분모가 그만큼 작아져 헤딩 노이즈가
        //    그대로 증폭된다. run_0814_191443 의 실제 경로 9,920 개로 A/B:
        //      머리 |kappa| p99  0.802 -> 5.013,  max 1.311 -> 30.6 rad/m
        //      머리가 거부 문턱(1.312)을 넘어 경로 전체를 버리는 경우 0 -> 523회
        //      머리 속도캡 중앙값도 5.90 -> 5.80 으로 오히려 하락
        //    즉 이 복사는 편의가 아니라 **노이즈 게이트**다. 머리 속도 낙폭을
        //    줄이려면 곡률 추정이 아니라 그 낙폭을 실제로 만드는 항
        //    (전방 가속 envelope의 시작값 = cur_vs_)을 봐야 한다.
        for (size_t i = 0; i < kWindow; ++i) {
            if (std::abs(curvature[kWindow]) > std::abs(curvature[i])) {
                curvature[i] = curvature[kWindow];
            }
            const size_t j = n - 1 - i;
            const size_t src = n - 1 - kWindow;
            if (std::abs(curvature[src]) > std::abs(curvature[j])) {
                curvature[j] = curvature[src];
            }
        }
    }

    // ── 레이스라인 그립 일관성 캡 ────────────────────────────────────────
    // 코너 안쪽으로 피하면 반경이 줄어 곡률이 커진다(OffsetKappa). 그 물리를
    // 예전에는 inside_speed_scale이라는 **고정 배율**(실차 0.7 = 30% 감속)로
    // 대신했는데, 이건 두 가지가 잘못돼 있었다:
    //   1) 곡률이 얼마나 커졌는지와 무관하게 항상 같은 비율로 깎는다 —
    //      완만한 코너에서 30%는 순손실이다.
    //   2) OffsetKappa + path_lat_acc_max 캡이 이미 같은 물리를 반영하므로
    //      이중 계산이다.
    // 대신 "그 지점에서 레이스라인이 실제로 쓰던 횡가속(kappa*v^2)을 오프셋
    // 경로에서도 넘지 않는다"로 바꾼다. 결과는 정확히
    //     v_offset = v_raceline * sqrt(kappa_ref / kappa_offset)
    // 즉 안쪽으로 d만큼 피하면 sqrt(1 - d*kappa_ref) 만큼만 감속한다.
    //   R=2m, d=0.4 -> 0.894 (10.6% 감속. 기존 고정 0.7 대비 28% 빠름)
    //   R=10m, d=0.4 -> 0.980 (2% 감속)  <- 완만한 코너에서 손실이 사라진다
    // 곡률이 줄어드는 바깥쪽 회피는 캡하지 않는다(min이라 속도가 오르지도
    // 않는다 — 아웃라인 여유 그립을 쓰려면 별도 결정이 필요, 지금은 보류).
    if (offset_grip_cap_) {
        for (size_t i = 0; i < n; ++i) {
            if (lat_ref[i] <= 0.0) {
                continue;             // 직선 구간: 기하곡률 캡이 담당
            }
            const double k_off = std::abs(curvature[i]);
            if (!std::isfinite(k_off) || k_off <= kappa_ref[i]) {
                continue;             // 바깥쪽(완만해짐) — 감속 근거 없음
            }
            speed[i] = std::min(speed[i], std::sqrt(lat_ref[i] / k_off));
        }
    }

    // 횡가속 상한은 실제 차량 속도(cmd scale 적용)에서 계산한다.
    const double steering_curvature_limit = SteerKappaMax();
    // ── 단일 샘플 스파이크로 경로 전체를 버리지 않는다 ────────────────────
    // 예전에는 한 점이라도 조향 한계를 1e-3 넘으면 즉시 return false 였다.
    // curvature[]는 자연 3차 스플라인을 wpnt_dist 간격으로 이산 샘플링한
    // 값이라 절점 근처에서 한두 점이 튀는데, 그 한 점 때문에 나머지 전부가
    // 성립하는 회피가 통째로 죽고 lane_speed_profile / spline_speed_profile
    // 거부로 기록됐다.
    // reject_ratio 밴드 안의 점은 살리되, 바로 아래 횡가속 캡이 sqrt(a/k)로
    // 속도를 강하게 눌러 실제로 그 점을 천천히 통과하게 만든다. 밴드를
    // 넘으면 예전과 같이 경로를 버린다 — 그건 스파이크가 아니라 기하 자체가
    // 못 도는 것이다.
    const double steering_curvature_reject =
        steering_curvature_limit * steer_reject_ratio;
    if (active_lat_budget_ > 1e-3) {
        // ── 검사를 전수로 먼저 돌고, 통과한 뒤에만 wpnts 를 건드린다 ────────
        // 예전에는 한 루프에서 검사와 `wpnts[i].kappa_radpm = curvature[i]` 를
        // 같이 해서, 인덱스 j 에서 기각되면 0..j-1 만 오프셋 곡률로 덮이고
        // 나머지는 원본이 남는 **반쯤 갱신된 입력**이 됐다. 대부분의 호출부는
        // 실패 시 경로를 버리지만 BuildTrailing 은 반환값을 쓰지 않으므로
        // 그 뒤섞인 곡률이 그대로 발행됐다.
        // ── 소프트 모드는 버리지 않고 캡만 건다 (2026-08-20) ───────────────
        // steer_reject_idx_out 이 주어지면 위반 인덱스만 기록하고 아래 캡
        // 루프를 그대로 태운다. 근거는 선언부 주석(run_0820_201714 4:45).
        // 비유한 곡률은 소프트 모드에서도 하드 기각이다 — sqrt(a/k) 가 NaN 이
        // 되어 속도와 kappa_radpm 을 동시에 오염시킨다. 그래서 첫 위반에서
        // 멈추지 않고 **전수로** 훑어 비유한 값을 끝까지 찾는다.
        int soft_reject_idx = -1;
        for (size_t i = 0; i < n; ++i) {
            const double k = std::abs(curvature[i]);
            if (std::isfinite(k) && k <= steering_curvature_reject + 1e-3) {
                continue;
            }
            if (soft_reject_idx < 0) {
                // 어느 점이 걸렸는지 남긴다 — 호출자의 거부 로그가 이 값을
                // 붙여 찍는다. 머리(idx 0~2)가 반복해서 걸리면 위 좁힌-윈도
                // 수정이 아직 부족하다는 뜻이다.
                speed_reject_idx_ = static_cast<int>(i);
                speed_reject_n_ = static_cast<int>(n);
                speed_reject_kappa_ = k;
                speed_reject_limit_ = steering_curvature_reject;
                soft_reject_idx = static_cast<int>(i);
            }
            if (steer_reject_idx_out == nullptr || !std::isfinite(k)) {
                NoteReject("speed_steer_curv");
                return false;
            }
        }
        if (soft_reject_idx >= 0) {
            *steer_reject_idx_out = soft_reject_idx;
            NoteReject("speed_steer_curv_clamped");
        }
        for (size_t i = 0; i < n; ++i) {
            const double k = std::abs(curvature[i]);
            if (k > 1e-4) {
                speed[i] = std::min(
                    speed[i], std::sqrt(active_lat_budget_ / k));
            }
            wpnts[i].kappa_radpm = curvature[i];
        }
    }

    // 조향 액추에이터 rate를 경로 시간축에 반영한다. 인접 곡률 변화가 크면
    // 필요한 steering slew가 2 rad/s를 넘지 않도록 그 구간 속도를 낮춘다.
    for (size_t i = 0; i + 1 < n; ++i) {
        const double steer0 =
            std::atan(vehicle_wheelbase_m_ * curvature[i]);
        const double steer1 =
            std::atan(vehicle_wheelbase_m_ * curvature[i + 1]);
        const double steer_delta = std::abs(steer1 - steer0);
        if (steer_delta > 1e-5) {
            const double steer_rate_speed =
                ds[i] * vehicle_max_steering_rate_radps_ / steer_delta;
            speed[i] = std::min(speed[i], steer_rate_speed);
            speed[i + 1] = std::min(speed[i + 1], steer_rate_speed);
        }
    }
    }   // !reuse_ceiling

    auto longitudinal_budget =
        [this](double base, double v, double kappa) {
            if (active_lat_budget_ <= 1e-3) {
                return base;
            }
            const double lat = std::abs(kappa) * v * v;
            const double ratio =
                std::clamp(lat / active_lat_budget_, 0.0, 1.0);
            return base * std::sqrt(std::max(1.0 - ratio * ratio, 0.0));
        };

    // 미래 곡률 cap을 만족하도록 역방향 감속. 횡그립을 쓰는 만큼 종그립을
    // friction-circle에서 차감해 두 한계를 동시에 최대로 쓰지 못하게 한다.
    if (!reuse_ceiling) {
    for (int i = static_cast<int>(n) - 2; i >= 0; --i) {
        // 이 segment에서 실제로 쓰는 횡그립은 시작점 곡률 기준이다. 다음
        // 점의 코너 곡률을 미리 적용하면 코너 "진입 전" 제동 여유가 0이 된다.
        const double k = std::abs(curvature[static_cast<size_t>(i)]);
        const double decel = longitudinal_budget(
            path_decel_max_, speed[static_cast<size_t>(i)], k);
        const double next = speed[static_cast<size_t>(i) + 1];
        const double allow = std::sqrt(
            std::max(next * next + 2.0 * decel * ds[static_cast<size_t>(i)], 0.0));
        speed[static_cast<size_t>(i)] =
            std::min(speed[static_cast<size_t>(i)], allow);
    }
    }   // !reuse_ceiling

    // ── 여기까지가 기하 전용 상한이다 (cur_vs_ 와 무관) ──────────────────
    // 아래부터는 현재 속도에 의존한다. 커밋 경로는 이 경계의 값을 보관해
    // 두었다가 재생 때 아래만 다시 푼다 — 그래야 커밋 생성 당시의 속도가
    // 경로에 얼어붙지 않는다 (run_0814_191443: 동결 머리 속도와 생성 시점
    // 차속의 상관 0.99, 지금 차속과는 0.56, 최장 16.8초).
    last_geom_ceiling_.assign(speed.begin(), speed.end());
    if (out_geom_ceiling != nullptr) {
        *out_geom_ceiling = last_geom_ceiling_;
    }

    // 현재 속도부터 순방향 가속 가능 envelope. 동적 v_floor/sector boost는
    // 여기서 물리 상한보다 높으면 자연히 잘린다.
    double previous = std::max(std::abs(cur_vs_), 0.0);
    double first_distance = Mod(wpnts.front().s_m - cur_s_, track_len_);
    if (first_distance > track_len_ / 2.0) {
        first_distance = 0.0;
    }
    double accel = longitudinal_budget(
        path_accel_max_, previous, curvature.front());
    speed.front() = std::min(
        speed.front(), std::sqrt(previous * previous + 2.0 * accel * first_distance));
    previous = speed.front();
    for (size_t i = 0; i + 1 < n; ++i) {
        accel = longitudinal_budget(
            path_accel_max_, previous,
            std::max(std::abs(curvature[i]), std::abs(curvature[i + 1])));
        const double allow = std::sqrt(
            previous * previous + 2.0 * accel * ds[i]);
        speed[i + 1] = std::min(speed[i + 1], allow);
        previous = speed[i + 1];
    }

    // jerk-limited 속도 shaping. 역방향 pass는 급격한 제동 onset을 앞쪽
    // 속도를 낮춰 완화하고, 순방향 pass는 급격한 가속을 낮춘다.
    for (int iteration = 0; iteration < 2; ++iteration) {
        double a_next = 0.0;
        for (int i = static_cast<int>(n) - 2; i >= 0; --i) {
            const size_t ui = static_cast<size_t>(i);
            const double dt = 2.0 * ds[ui] /
                std::max(speed[ui] + speed[ui + 1], 0.5);
            const double desired =
                (speed[ui + 1] * speed[ui + 1] - speed[ui] * speed[ui]) /
                (2.0 * ds[ui]);
            const double decel = longitudinal_budget(
                path_decel_max_, speed[ui],
                std::abs(curvature[ui]));
            const double lower = std::max(
                -decel, a_next - path_jerk_max_ * dt);
            if (desired < lower) {
                const double corrected = std::sqrt(std::max(
                    speed[ui + 1] * speed[ui + 1] -
                    2.0 * lower * ds[ui], 0.0));
                speed[ui] = std::min(speed[ui], corrected);
            }
            a_next =
                (speed[ui + 1] * speed[ui + 1] - speed[ui] * speed[ui]) /
                (2.0 * ds[ui]);
        }

        double a_previous = 0.0;
        for (size_t i = 0; i + 1 < n; ++i) {
            const double dt = 2.0 * ds[i] /
                std::max(speed[i] + speed[i + 1], 0.5);
            double desired =
                (speed[i + 1] * speed[i + 1] - speed[i] * speed[i]) /
                (2.0 * ds[i]);
            const double accel_cap = longitudinal_budget(
                path_accel_max_, speed[i],
                std::max(std::abs(curvature[i]), std::abs(curvature[i + 1])));
            const double upper = std::min(
                accel_cap, a_previous + path_jerk_max_ * dt);
            if (desired > upper) {
                speed[i + 1] = std::min(speed[i + 1], std::sqrt(std::max(
                    speed[i] * speed[i] + 2.0 * upper * ds[i], 0.0)));
                desired =
                    (speed[i + 1] * speed[i + 1] - speed[i] * speed[i]) /
                    (2.0 * ds[i]);
            }
            a_previous = desired;
        }
    }

    // ── 현재 상태 성립성: "계획 속도 도달"이 아니라 "그립 초과"로 판정한다 ──
    // 현재 속도에서 계획 지연 후 jerk/decel 한계로 최대 제동했을 때 **실제로
    // 낼 속도**(reachable)를 구한다.
    //
    // 2026-08-11 이전에는 reachable 이 계획 속도를 tolerance 이상 넘으면 그
    // 자리에서 경로를 통째로 버렸다(return false). 그런데 버리면 남는 건
    // Tier3 트레일링(감속/정지)뿐이고, 감속하고 나면 다음 사이클에 같은 회피가
    // 성립한다 — rosbag0810 두 런에서 이것이 "먼저 서고 나중에 피한다"의
    // 직접 원인이었다 (첫 검출 4.4~5.5 m / 3.0 m/s 에서 거부 -> 2.2~2.9 m/s 로
    // 떨어진 뒤에야 커밋).
    //
    // 실제로 위험한 것은 "계획 속도를 못 맞추는 것"이 아니라 "그 속도로 그
    // 곡률을 돌 때 마찰원을 넘는 것"이다. 거부된 케이스들을 재구성하면
    // 계획보다 최대 +2.33 m/s 빨리 지나가도 요구 횡가속은 6.48~7.60 으로
    // 예산 7.85 안이었다 (물리한계 0.90g = 8.83). 즉 거부는 순손실이었다.
    //
    // 그래서 (1) 판정을 그립 초과로 바꾸고, (2) 통과하면 진입 구간의 계획
    // 속도를 reachable 로 **올려서** 발행한다.
    //
    // (2)가 필수인 이유: 컨트롤러가 조향을 waypoint 의 vx_mps 로 계산한다
    // (control_node.cpp calc_steering_angle 의 speed_for_lookup ->
    //  lateral_acceleration = 2*v^2/L1*sin(eta)). 계획 속도가 실제보다 낮으면
    // 조향 요구가 (계획/실제)^2 배로 줄어 차가 덜 꺾는다 — 위 케이스에서
    // 0.08~0.56 배였다. vx=0 프로파일이 조향을 0으로 만들어 0807 코스아웃
    // 4건을 낸 것과 같은 메커니즘이다. 못 지킬 숫자를 적어 보내면 안 된다.
    //
    // reachable 은 최대 제동 곡선이라 앞쪽에서 급격히 작아지므로, max() 는
    // 진입 구간만 올리고 그 뒤 계획 프로파일은 건드리지 않는다.
    //
    // 2026-08-11(2): 이 시뮬레이션의 제동 한계를 path_decel_max_ 에서
    // plant_decel_mps2_ 로 분리했다. path_decel_max_(6.0)는 경로 정형용 상한이라
    // 실차 제동 능력(A_DEC_PLANNED 2.38)의 2.5배이고, 그 값으로 reachable 을
    // 풀면 **차가 실제로는 훨씬 빨리 지나가는데 시뮬은 이미 섰다**고 본다.
    // 두 방향 모두 틀렸다:
    //   · 게이트: k*v²>예산 판정의 v 가 과소 -> 실제로는 마찰원을 넘는 회피를
    //     통과시킨다. v0=5.0 에서 4 m 앞 시뮬 2.50 vs 실제 3.58 m/s 이고,
    //     kappa 0.783(이 트랙 최대)에서 실제 a_lat 10.0 > 물리한계 8.83.
    //   · 발행: 아래 max() 로 올려 쓰는 값도 실제보다 낮아, 컨트롤러
    //     speed_for_lookup 이 여전히 조향을 (계획/실제)² 배로 줄인다 —
    //     이 블록이 애초에 고치려던 항목이 절반만 고쳐진다.
    if (require_current_feasible) {
        std::vector<double> reachable(n, 0.0);
        double simulated_v = std::max(std::abs(cur_vs_), 0.0);
        double simulated_a = 0.0;
        double delay_distance = simulated_v * planning_delay_sec_;
        for (size_t i = 0; i < n; ++i) {
            double segment = (i == 0) ? first_distance : ds[i - 1];
            const double delayed = std::min(segment, delay_distance);
            delay_distance -= delayed;
            segment -= delayed;
            if (segment > 1e-6) {
                const double dt =
                    segment / std::max(simulated_v, 0.25);
                const double decel = longitudinal_budget(
                    plant_decel_mps2_, simulated_v,
                    std::abs(curvature[(i == 0) ? 0 : i - 1]));
                simulated_a = std::max(
                    -decel, simulated_a - path_jerk_max_ * dt);
                simulated_v = std::sqrt(std::max(
                    simulated_v * simulated_v +
                    2.0 * simulated_a * segment, 0.0));
            }
            reachable[i] = simulated_v;
            // 최대 제동으로도 이 곡률을 마찰원 안에서 못 도는 지점이 있으면
            // 감속으로 살릴 수 없다 — 그때만 상위 폴백(트레일링)으로 넘긴다.
            const double k = std::abs(curvature[i]);
            if (active_lat_budget_ > 1e-3 && k > 1e-4 &&
                k * simulated_v * simulated_v > active_lat_budget_) {
                // 이 함수는 const 라 THROTTLE 로그(clock 필요)를 못 쓴다.
                // 사유는 카운터로 남긴다 — 호출자가 찍는 "속도 프로파일
                // 불성립"만으로는 곡률 초과와 구분이 안 됐다.
                NoteReject("speed_grip_infeasible");
                return false;
            }
        }
        // 통과 — 못 지킬 낮은 숫자 대신 실제 낼 속도를 적는다.
        for (size_t i = 0; i < n; ++i) {
            speed[i] = std::max(speed[i], reachable[i]);
        }
    }

    for (size_t i = 0; i < n; ++i) {
        wpnts[i].vx_mps = speed[i] / command_scale;
    }
    // 속도 변경 후 stale global ax를 남기지 않는다. Wpnt 속도 좌표계에서
    // v^2 공간미분으로 일관되게 다시 계산한다.
    for (size_t i = 0; i + 1 < n; ++i) {
        wpnts[i].ax_mps2 =
            (wpnts[i + 1].vx_mps * wpnts[i + 1].vx_mps -
             wpnts[i].vx_mps * wpnts[i].vx_mps) /
            (2.0 * ds[i]);
    }
    wpnts.back().ax_mps2 = wpnts[n - 2].ax_mps2;
    return true;
}

std::pair<double, double> SplinerNode::FrenetToCartesian(double s, double d) const {
    int gb_size = static_cast<int>(gb_wpnts_.size());
    double idx_f = s / wpnt_dist_;
    // track_len_ = (마지막 s - 첫 s) + 폐합 길이 이므로, 폐합 구간이
    // wpnt_dist_ 보다 길면 s -> track_len_ 근처에서 idx_f 가 gb_size 를 넘는다.
    // 예전처럼 `% gb_size` 로 접으면 i0 가 0 이 되어 **랩 끝 지점이 랩 시작
    // 좌표로** 매핑됐다 (오차 ≈ 한 바퀴). 마지막 구간으로 클램프하면 오차가
    // 폐합 초과분으로 묶인다. 정상 범위에서는 종전과 완전히 같다.
    int i0 = static_cast<int>(std::floor(idx_f));
    i0 = std::clamp(i0, 0, gb_size - 1);
    int i1 = (i0 + 1) % gb_size;
    double t = std::clamp(idx_f - static_cast<double>(i0), 0.0, 1.0);

    const auto &w0 = gb_wpnts_[i0];
    const auto &w1 = gb_wpnts_[i1];
    double x_c = w0.x_m + t * (w1.x_m - w0.x_m);
    double y_c = w0.y_m + t * (w1.y_m - w0.y_m);
    double psi = w0.psi_rad + t * WrapToPi(w1.psi_rad - w0.psi_rad);

    // frenet_frame_node와 동일 규약: d>0 = 진행방향 왼쪽
    return {x_c - d * std::sin(psi), y_c + d * std::cos(psi)};
}

std::vector<double> SplinerNode::NaturalCubicSpline(
    const std::vector<double> &xs, const std::vector<double> &ys,
    const std::vector<double> &query) {
    // 절점의 2차 도함수 M을 자연 경계조건(M0=Mn=0)으로 풀고 (Thomas algorithm)
    // 구간별 3차 다항식으로 평가
    size_t n = xs.size();
    // h[i]는 모든 계수의 분모다. 절점이 단조증가가 아니면 h<=0 -> NaN 궤적이
    // 그대로 발행되므로 여기서 막고 빈 결과로 실패를 알린다.
    if (n < 3 || ys.size() != n) {
        return {};
    }
    std::vector<double> h(n - 1);
    for (size_t i = 0; i < n - 1; ++i) {
        h[i] = xs[i + 1] - xs[i];
        if (!(h[i] > 1e-9)) {
            return {};
        }
    }

    std::vector<double> diag(n, 2.0), sub(n, 0.0), sup(n, 0.0), rhs(n, 0.0);
    for (size_t i = 1; i < n - 1; ++i) {
        sub[i] = h[i - 1] / (h[i - 1] + h[i]);
        sup[i] = h[i] / (h[i - 1] + h[i]);
        rhs[i] = 6.0 / (h[i - 1] + h[i]) *
                 ((ys[i + 1] - ys[i]) / h[i] - (ys[i] - ys[i - 1]) / h[i - 1]);
    }

    std::vector<double> m(n, 0.0), cp(n, 0.0), dp(n, 0.0);
    cp[0] = 0.0;
    dp[0] = 0.0;
    for (size_t i = 1; i < n; ++i) {
        double denom = diag[i] - sub[i] * cp[i - 1];
        cp[i] = sup[i] / denom;
        dp[i] = (rhs[i] - sub[i] * dp[i - 1]) / denom;
    }
    m[n - 1] = dp[n - 1];
    for (int i = static_cast<int>(n) - 2; i >= 0; --i) {
        m[i] = dp[i] - cp[i] * m[i + 1];
    }

    std::vector<double> out;
    out.reserve(query.size());
    size_t seg = 0;
    for (double q : query) {
        while (seg + 2 < n && q > xs[seg + 1]) {
            ++seg;
        }
        double a = (xs[seg + 1] - q) / h[seg];
        double b = (q - xs[seg]) / h[seg];
        double val = a * ys[seg] + b * ys[seg + 1] +
                     ((a * a * a - a) * m[seg] + (b * b * b - b) * m[seg + 1]) *
                     (h[seg] * h[seg]) / 6.0;
        out.push_back(val);
    }
    return out;
}

visualization_msgs::msg::Marker SplinerNode::MakeWpntMarker(
    double x, double y, double v, int id) const {
    visualization_msgs::msg::Marker mrk;
    mrk.header.frame_id = "map";
    mrk.header.stamp = this->now();
    mrk.type = visualization_msgs::msg::Marker::CYLINDER;
    mrk.id = id;
    mrk.scale.x = 0.1;
    mrk.scale.y = 0.1;
    mrk.scale.z = std::max(v / std::max(gb_vmax_, 0.1), 0.01);
    mrk.color.a = 1.0;
    mrk.color.r = 0.75;
    mrk.color.b = 0.75;
    mrk.pose.position.x = x;
    mrk.pose.position.y = y;
    mrk.pose.position.z = mrk.scale.z / 2.0;
    mrk.pose.orientation.w = 1.0;
    return mrk;
}

int main(int argc, char **argv) {
    std::string node_name = "spliner_node";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SplinerNode>(node_name));
    rclcpp::shutdown();
    return 0;
}
