#include "planning_pkg/spliner_node.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>

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

// opponent predictor가 내보내는 catch-zone은 이미 미래 충돌시점으로 공간
// 투영된 "튜브"다. 메시지에 provenance 필드가 없으므로 예약 ID 대역으로
// source를 보존한다. 전략상 동적(is_static=false)이어도 이 기하는 재전파하지 않는다.
constexpr std::int32_t kPredictionTubeIdBase = -1000000000;
constexpr std::int64_t kPredictionTubeIdRange = 1000000000;

std::int32_t EncodePredictionTubeId(std::int32_t raw_id) {
    const std::int64_t wide_id = raw_id;
    const std::int64_t magnitude = wide_id < 0 ? -wide_id : wide_id;
    return kPredictionTubeIdBase -
        static_cast<std::int32_t>(magnitude % kPredictionTubeIdRange);
}

bool IsPredictionTube(
    const f1tenth_planning_msgs::msg::Obstacle &obstacle) {
    return obstacle.id <= kPredictionTubeIdBase;
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
    this->declare_parameter("spliner/use_prediction", true);
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
    this->declare_parameter("spliner/offset_grip_cap", true);
    this->declare_parameter("spliner/path_decel_max", 6.0);
    this->declare_parameter("spliner/plant_decel_mps2", 2.38);
    this->declare_parameter("spliner/path_accel_max", 3.0);
    this->declare_parameter("spliner/path_jerk_max", 8.0);
    this->declare_parameter("spliner/planning_delay_sec", 0.10);
    this->declare_parameter("spliner/vehicle_half_length_m", 0.275);
    this->declare_parameter("spliner/vehicle_half_width_m", 0.15);
    this->declare_parameter("spliner/boundary_tracking_margin_m", 0.05);
    this->declare_parameter("spliner/collision_sigma", 2.0);
    // 정적 장애물의 속도분산 시간 성장 상한 [s] / 회피 기하 계획 속도 하한 [m/s]
    this->declare_parameter("spliner/static_time_uncertainty_cap", 0.3);
    this->declare_parameter("spliner/max_uncertainty_m", 0.25);
    this->declare_parameter("spliner/plan_speed_floor", 1.5);
    this->declare_parameter("spliner/path_check_margin", 0.15);
    this->declare_parameter("spliner/cluster_gap_s_m", 8.0);
    this->declare_parameter("spliner/trailing_engage_dist_m", 8.0);
    this->declare_parameter("spliner/trailing_lag_sec", 0.3);
    this->declare_parameter("spliner/trailing_ramp_decel", 2.0);
    this->declare_parameter("spliner/trailing_retry_speed", 1.5);
    this->declare_parameter("spliner/apex_clearance_eps_m", 0.05);
    this->declare_parameter("spliner/commit_break_min_obs", 3);
    this->declare_parameter("spliner/commit_break_requires_visible", true);
    this->declare_parameter("spliner/creep_escape_speed", 0.6);
    // 원호 이탈 (BuildEscapeArc) — 근거는 spliner_node.hpp 선언부 주석.
    this->declare_parameter("spliner/escape_arc_enabled", true);
    this->declare_parameter("spliner/escape_arc_lat_acc", 5.5);
    this->declare_parameter("spliner/escape_arc_min_speed", 1.2);
    this->declare_parameter("spliner/escape_arc_max_heading_rad", 0.70);
    this->declare_parameter("spliner/dyn_pass_speed_ratio", 1.4);
    this->declare_parameter("spliner/cluster_gap_d_m", 0.8);
    // 정적/동적 재분류 (unicorn perception/multi_tracking.py 이식)
    this->declare_parameter("spliner/cls_min_nb_meas", 5);
    this->declare_parameter("spliner/cls_min_std", 0.16);
    this->declare_parameter("spliner/cls_max_std", 0.2);
    this->declare_parameter("spliner/cls_vs_reset", 0.1);
    this->declare_parameter("spliner/sector_gating", true);
    this->declare_parameter("spliner/ot_min_clearance", 0.95);
    this->declare_parameter("spliner/ot_max_kappa", 0.1);
    // 라인 스위치 (기본 maneuver): true면 오프셋 레인 스위치를 1순위로 시도하고
    // 모든 레인이 막힐 때만 DoSpline을 폴백으로 호출한다. false면 기존 동작(DoSpline 우선).
    this->declare_parameter("spliner/use_lane_switch", true);
    this->declare_parameter("spliner/lane_blend_out_m", 5.0);
    this->declare_parameter("spliner/early_switch_kappa_max", 0.10);
    this->declare_parameter("spliner/sp_lane_change", true);
    this->declare_parameter("spliner/sp_mapping_max_residual_m", 0.5);
    this->declare_parameter("spliner/vehicle_wheelbase_m", 0.33);
    this->declare_parameter("spliner/vehicle_max_steering_rad", 0.3578);
    this->declare_parameter("spliner/vehicle_max_steering_rate_radps", 2.0);
    // 조향 기하 게이트 완화 노브 3종. 기본값은 예전 하드코딩과 동일하므로
    // yaml에 없으면 동작이 바뀌지 않는다. 의미는 헤더 멤버 주석 참조.
    this->declare_parameter("spliner/steer_kappa_scale", 1.0);
    this->declare_parameter("spliner/geom_floor_margin", 0.8);
    this->declare_parameter("spliner/geom_floor_kappa_ratio", 0.15);
    this->declare_parameter("spliner/steer_kappa_reject_ratio", 1.0);
    // 추월 완료 판정 s 간격: 상대 앞머리가 ego 뒤로 이만큼 벌어져야
    // 라인 복귀 시작 (그 전 복귀는 반 차신도 안 앞선 상대 앞 끼어들기)
    this->declare_parameter("spliner/lc_pass_clear_m", 4.0);
    this->declare_parameter("spliner/ot_sector_min_len_m", 8.0);
    this->declare_parameter("spliner/ot_speed_scale", 1.0);
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
    use_prediction_ = this->get_parameter("spliner/use_prediction").as_bool();
    trailing_gap_m_ = this->get_parameter("spliner/trailing_gap_m").as_double();
    trailing_decel_ = this->get_parameter("spliner/trailing_decel").as_double();
    cmd_speed_scale_ = this->get_parameter("spliner/cmd_speed_scale").as_double();
    blend_lat_acc_max_ = this->get_parameter("spliner/blend_lat_acc_max").as_double();
    blend_dist_max_m_ = this->get_parameter("spliner/blend_dist_max_m").as_double();
    blend_use_spare_grip_ =
        this->get_parameter("spliner/blend_use_spare_grip").as_bool();
    path_lat_acc_max_ = this->get_parameter("spliner/path_lat_acc_max").as_double();
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
    collision_sigma_ = this->get_parameter("spliner/collision_sigma").as_double();
    static_time_uncertainty_cap_ =
        this->get_parameter("spliner/static_time_uncertainty_cap").as_double();
    max_uncertainty_m_ =
        this->get_parameter("spliner/max_uncertainty_m").as_double();
    plan_speed_floor_ = this->get_parameter("spliner/plan_speed_floor").as_double();
    path_check_margin_ = this->get_parameter("spliner/path_check_margin").as_double();
    cluster_gap_s_m_ = this->get_parameter("spliner/cluster_gap_s_m").as_double();
    trailing_engage_dist_m_ =
        this->get_parameter("spliner/trailing_engage_dist_m").as_double();
    trailing_lag_sec_ =
        this->get_parameter("spliner/trailing_lag_sec").as_double();
    trailing_ramp_decel_ =
        this->get_parameter("spliner/trailing_ramp_decel").as_double();
    trailing_retry_speed_ =
        this->get_parameter("spliner/trailing_retry_speed").as_double();
    apex_clearance_eps_m_ =
        this->get_parameter("spliner/apex_clearance_eps_m").as_double();
    commit_break_min_obs_ = static_cast<int>(
        this->get_parameter("spliner/commit_break_min_obs").as_int());
    commit_break_requires_visible_ =
        this->get_parameter("spliner/commit_break_requires_visible").as_bool();
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
    dyn_pass_speed_ratio_ = this->get_parameter("spliner/dyn_pass_speed_ratio").as_double();
    cluster_gap_d_m_ = this->get_parameter("spliner/cluster_gap_d_m").as_double();
    cls_min_nb_meas_ = static_cast<int>(this->get_parameter("spliner/cls_min_nb_meas").as_int());
    cls_min_std_ = this->get_parameter("spliner/cls_min_std").as_double();
    cls_max_std_ = this->get_parameter("spliner/cls_max_std").as_double();
    cls_vs_reset_ = this->get_parameter("spliner/cls_vs_reset").as_double();
    sector_gating_ = this->get_parameter("spliner/sector_gating").as_bool();
    ot_min_clearance_ = this->get_parameter("spliner/ot_min_clearance").as_double();
    ot_max_kappa_ = this->get_parameter("spliner/ot_max_kappa").as_double();
    use_lane_switch_ = this->get_parameter("spliner/use_lane_switch").as_bool();
    lane_blend_out_m_ = this->get_parameter("spliner/lane_blend_out_m").as_double();
    early_switch_kappa_max_ =
        this->get_parameter("spliner/early_switch_kappa_max").as_double();
    sp_lane_change_ = this->get_parameter("spliner/sp_lane_change").as_bool();
    sp_mapping_max_residual_m_ =
        this->get_parameter("spliner/sp_mapping_max_residual_m").as_double();
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
    lc_pass_clear_m_ = this->get_parameter("spliner/lc_pass_clear_m").as_double();
    ot_sector_min_len_m_ = this->get_parameter("spliner/ot_sector_min_len_m").as_double();
    ot_speed_scale_ = this->get_parameter("spliner/ot_speed_scale").as_double();
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
        boundary_tracking_margin_m_, collision_sigma_, path_check_margin_,
        static_time_uncertainty_cap_, max_uncertainty_m_, plan_speed_floor_,
        cluster_gap_s_m_, cluster_gap_d_m_,
        trailing_engage_dist_m_,
        dyn_pass_speed_ratio_, cls_min_std_, cls_max_std_, cls_vs_reset_,
        ot_min_clearance_, ot_max_kappa_, lane_blend_out_m_,
        sp_mapping_max_residual_m_, vehicle_wheelbase_m_,
        vehicle_max_steering_rad_, vehicle_max_steering_rate_radps_,
        steer_kappa_scale_, geom_floor_margin_, geom_floor_kappa_ratio_,
        steer_kappa_reject_ratio_,
        lc_pass_clear_m_, ot_sector_min_len_m_, ot_speed_scale_,
        // 아래 5개는 범위 비교(x < lo || x > hi)로만 검사되던 것들이다. NaN은
        // 두 비교 모두 false라 그 검사를 그냥 통과하므로 isfinite 목록에 반드시
        // 있어야 위 주석의 계약("잘못된 설정은 기동 거부")이 실제로 성립한다.
        trailing_lag_sec_, trailing_ramp_decel_, trailing_retry_speed_,
        apex_clearance_eps_m_, creep_escape_speed_,
        escape_arc_lat_acc_, escape_arc_min_speed_, escape_arc_max_heading_};
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
        path_accel_max_ <= 0.0 || path_accel_max_ > 20.0 ||
        path_decel_max_ <= 0.0 || path_decel_max_ > 20.0 ||
        plant_decel_mps2_ <= 0.0 || plant_decel_mps2_ > 20.0 ||
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
        trailing_lag_sec_ < 0.0 || trailing_lag_sec_ > 2.0 ||
        trailing_ramp_decel_ <= 0.0 || trailing_ramp_decel_ > 20.0 ||
        trailing_retry_speed_ < 0.0 || trailing_retry_speed_ > 10.0 ||
        apex_clearance_eps_m_ < 0.0 || apex_clearance_eps_m_ > 0.5 ||
        commit_break_min_obs_ < 1 || commit_break_min_obs_ > 20 ||
        creep_escape_speed_ < 0.0 || creep_escape_speed_ > 3.0 ||
        dyn_pass_speed_ratio_ < 1.0 || dyn_pass_speed_ratio_ > 2.0 ||
        cls_min_nb_meas_ < 2 || cls_min_nb_meas_ > 100 ||
        cls_min_std_ <= 0.0 || cls_max_std_ < cls_min_std_ ||
        cls_vs_reset_ < 0.0 || cls_vs_reset_ > 5.0 ||
        ot_min_clearance_ <= 0.0 || ot_min_clearance_ > 10.0 ||
        ot_max_kappa_ <= 0.0 || ot_max_kappa_ > 5.0 ||
        lane_blend_out_m_ <= 0.0 || lane_blend_out_m_ > lookahead_m_ ||
        early_switch_kappa_max_ < 0.0 || early_switch_kappa_max_ > 5.0 ||
        sp_mapping_max_residual_m_ < 0.05 || sp_mapping_max_residual_m_ > 1.0 ||
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
        lc_pass_clear_m_ <= 0.0 || lc_pass_clear_m_ > lookahead_m_ ||
        ot_sector_min_len_m_ <= 0.0 || ot_sector_min_len_m_ > lookahead_m_ ||
        // 1.0 초과 = 섹터 내 부스트. 예전 상한 1.0은 헤더/params 주석이
        // 약속한 "섹터 안 공격 모드"를 원리적으로 불가능하게 만들었다.
        // 실제 상한은 gb_vmax_(레이스라인 최고속)이므로 여기서는 2.0까지 허용.
        ot_speed_scale_ <= 0.0 || ot_speed_scale_ > 2.0 ||
        // 원호는 마지막 회피 수단이라 통상 전환보다 큰 횡가속을 쓰지만,
        // 경로 곡률 검증(path_lat_acc_max_)이 쓰는 그립 예산을 넘으면
        // 자기가 만든 경로를 자기 검증기가 거부한다.
        escape_arc_lat_acc_ <= 0.0 || escape_arc_lat_acc_ > path_lat_acc_max_ ||
        escape_arc_min_speed_ < 0.0 || escape_arc_min_speed_ > 5.0 ||
        // 상한 1.0 rad(57도). 그 이상 남기면 복귀가 트랙 폭을 넘는다.
        escape_arc_max_heading_ <= 0.0 || escape_arc_max_heading_ > 1.0) {
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

    if (ot_speed_scale_ > 1.0) {
        RCLCPP_WARN(this->get_logger(),
            "spliner/ot_speed_scale=%.2f (>1) — 추월 섹터 내 회피 속도를 "
            "레이스라인보다 높입니다. 상한은 gb_vmax_이고 물리 성립성은 "
            "ApplySpeedProfile이 다시 검증합니다.", ot_speed_scale_);
    }

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

    // Predictive Spliner: opponent_predictor가 계산한 "따라잡는 시점의
    // 충돌 구간" 장애물. 같은 id의 원시 관측을 대체한다.
    s_predicted_obstacles_ = this->create_subscription<f1tenth_planning_msgs::msg::ObstacleArray>(
        "/opponent_prediction/obstacles", qos_profile,
        std::bind(&SplinerNode::CallbackPredictedObstacles, this, std::placeholders::_1));

    s_frenet_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/car_state/frenet/odom", qos_profile,
        std::bind(&SplinerNode::CallbackFrenetOdom, this, std::placeholders::_1));

    s_global_waypoints_ = this->create_subscription<f1tenth_planning_msgs::msg::WpntArray>(
        "/global_waypoints", qos_profile,
        std::bind(&SplinerNode::CallbackGlobalWaypoints, this, std::placeholders::_1));

    // SP(최단거리) 추월 라인 — 동적 상대 레인체인지 추월용 (없으면 기존 방식)
    s_sp_waypoints_ = this->create_subscription<f1tenth_planning_msgs::msg::WpntArray>(
        "/global_waypoints/overtaking", qos_profile,
        std::bind(&SplinerNode::CallbackSpWaypoints, this, std::placeholders::_1));

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

void SplinerNode::CallbackPredictedObstacles(const f1tenth_planning_msgs::msg::ObstacleArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_obstacles_);
    predicted_obstacles_ = *msg;
    predicted_stamp_ = this->now();
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
        bool first = !has_gb_wpnts_;
        has_gb_wpnts_ = true;
        if (first) {
            ComputeOtSectors();
            PublishSectorMarkers();
            if (!sp_raw_.empty() && !has_sp_ && !sp_mapping_attempted_) {
                BuildSpMapping();
            }
        }
    }
}

void SplinerNode::CallbackSpWaypoints(const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_gb_wpnts_);
    if (msg->wpnts.empty() || has_sp_ || sp_mapping_attempted_) {
        return;  // 첫 수신만 매핑 (라인은 정적)
    }
    sp_raw_ = msg->wpnts;
    if (has_gb_wpnts_) {
        BuildSpMapping();
    }
}

void SplinerNode::BuildSpMapping() {
    // SP는 별도 CSV이므로 "같은 맵/방향/차량 한계로 생성됐다"는 가정을
    // 런타임에서 검증한 뒤에만 사용한다. 한 항목이라도 실패하면 이 실행에서
    // SP를 완전히 비활성화한다(불완전 매핑으로 재시도하지 않음).
    sp_mapping_attempted_ = true;
    has_sp_ = false;
    sp_d_.clear();
    sp_vx_.clear();

    auto reject = [this](const std::string &reason) {
        RCLCPP_ERROR(this->get_logger(), "SP overtaking line disabled: %s", reason.c_str());
        pitwall::Event("SP 추월 라인 비활성: " + reason);
        sp_d_.clear();
        sp_vx_.clear();
        has_sp_ = false;
    };

    const int n = static_cast<int>(gb_wpnts_.size());
    const int m = static_cast<int>(sp_raw_.size());
    if (n < 3 || m < 3 || !(track_len_ > 0.0)) {
        reject("waypoint 수/트랙 길이 부족");
        return;
    }

    auto closed_length = [](const std::vector<f1tenth_planning_msgs::msg::Wpnt> &line) {
        double length = 0.0;
        for (size_t i = 0; i < line.size(); ++i) {
            const auto &a = line[i];
            const auto &b = line[(i + 1) % line.size()];
            if (!std::isfinite(a.x_m) || !std::isfinite(a.y_m) ||
                !std::isfinite(a.psi_rad) || !std::isfinite(a.vx_mps)) {
                return std::numeric_limits<double>::quiet_NaN();
            }
            length += std::hypot(b.x_m - a.x_m, b.y_m - a.y_m);
        }
        return length;
    };
    const double iq_length = closed_length(gb_wpnts_);
    const double sp_length = closed_length(sp_raw_);
    if (!std::isfinite(iq_length) || !std::isfinite(sp_length) ||
        iq_length < 1.0 || sp_length < 1.0) {
        reject("비정상 좌표/라인 길이");
        return;
    }
    const double length_ratio = sp_length / iq_length;
    if (length_ratio < 0.80 || length_ratio > 1.20) {
        reject("IQP/SP lap length 불일치");
        return;
    }

    // 회피 게이트와 같은 한계를 쓴다 — 여기만 엄격하면 "회피는 되는데 SP
    // 라인은 통째로 거부"라는 모순이 생긴다.
    const double max_curvature = SteerKappaMax();
    auto triangle_curvature = [](const auto &a, const auto &b, const auto &c) {
        const double ab = std::hypot(b.x_m - a.x_m, b.y_m - a.y_m);
        const double bc = std::hypot(c.x_m - b.x_m, c.y_m - b.y_m);
        const double ca = std::hypot(a.x_m - c.x_m, a.y_m - c.y_m);
        if (ab < 1e-5 || bc < 1e-5 || ca < 1e-5) {
            return 0.0;
        }
        const double cross =
            (b.x_m - a.x_m) * (c.y_m - a.y_m) -
            (b.y_m - a.y_m) * (c.x_m - a.x_m);
        return 2.0 * cross / (ab * bc * ca);
    };
    for (int j = 0; j < m; ++j) {
        const double k_geom = std::abs(triangle_curvature(
            sp_raw_[(j + m - 1) % m], sp_raw_[j], sp_raw_[(j + 1) % m]));
        const double k_csv = std::abs(sp_raw_[j].kappa_radpm);
        if (!std::isfinite(k_csv) || std::max(k_geom, k_csv) > max_curvature + 1e-3) {
            reject("SP 곡률이 차량 조향 한계 초과");
            return;
        }
    }

    sp_d_.assign(n, 0.0);
    sp_vx_.assign(n, 0.0);
    int previous_j = -1;
    long mapped_progress = 0;
    double max_residual = 0.0;
    for (int i = 0; i < n; ++i) {
        int best_j = -1;
        double best_sq = std::numeric_limits<double>::infinity();
        for (int j = 0; j < m; ++j) {
            // 반대 방향으로 가까이 붙은 헤어핀 branch는 후보에서 제외한다.
            if (std::cos(WrapToPi(sp_raw_[j].psi_rad - gb_wpnts_[i].psi_rad)) < 0.25) {
                continue;
            }
            const double dx = sp_raw_[j].x_m - gb_wpnts_[i].x_m;
            const double dy = sp_raw_[j].y_m - gb_wpnts_[i].y_m;
            const double sq = dx * dx + dy * dy;
            if (sq < best_sq) {
                best_sq = sq;
                best_j = j;
            }
        }
        if (best_j < 0 || !std::isfinite(best_sq)) {
            reject("동일 진행방향 최근접점 없음");
            return;
        }
        const double residual = std::sqrt(best_sq);
        max_residual = std::max(max_residual, residual);
        if (residual > sp_mapping_max_residual_m_) {
            reject("IQP-SP 최근접 residual 초과");
            return;
        }
        if (previous_j >= 0) {
            const int forward_step = (best_j - previous_j + m) % m;
            const int max_step = std::max(12, m / 10);
            if (forward_step > max_step) {
                reject("SP 최근접 매핑 순서 역행/branch jump");
                return;
            }
            mapped_progress += forward_step;
        }
        previous_j = best_j;

        const double dx = sp_raw_[best_j].x_m - gb_wpnts_[i].x_m;
        const double dy = sp_raw_[best_j].y_m - gb_wpnts_[i].y_m;
        const double psi = gb_wpnts_[i].psi_rad;
        const double along = dx * std::cos(psi) + dy * std::sin(psi);
        if (std::abs(along) > std::max(0.50, 5.0 * wpnt_dist_)) {
            reject("SP 최근접점 종방향 residual 초과");
            return;
        }
        const double d = -dx * std::sin(psi) + dy * std::cos(psi);
        if (!WithinTrackBoundary(i, d, spline_bound_mindist_)) {
            reject("SP 라인이 차량 폭/추적여유 포함 경계 밖");
            return;
        }
        if (!std::isfinite(sp_raw_[best_j].vx_mps) || sp_raw_[best_j].vx_mps < 0.0) {
            reject("SP 속도 프로파일 비정상");
            return;
        }
        sp_d_[i] = d;
        sp_vx_[i] = sp_raw_[best_j].vx_mps;
    }
    if (mapped_progress < m / 2 || mapped_progress > (3L * m) / 2L) {
        reject("SP 매핑이 한 바퀴를 단조롭게 덮지 못함");
        return;
    }

    // 실제 런타임은 raw SP 좌표가 아니라 IQP+d_sp로 재구성하므로 그 경로의
    // 곡률도 별도로 검증한다.
    std::vector<f1tenth_planning_msgs::msg::Wpnt> mapped(n);
    for (int i = 0; i < n; ++i) {
        const auto xy = FrenetToCartesian(gb_wpnts_[i].s_m, sp_d_[i]);
        mapped[i].x_m = xy.first;
        mapped[i].y_m = xy.second;
    }
    for (int i = 0; i < n; ++i) {
        const double k = std::abs(triangle_curvature(
            mapped[(i + n - 1) % n], mapped[i], mapped[(i + 1) % n]));
        if (k > max_curvature + 1e-3) {
            reject("재구성 SP 곡률이 차량 조향 한계 초과");
            return;
        }
    }

    has_sp_ = true;
    RCLCPP_INFO(this->get_logger(),
        "SP overtaking line validated: %d pts, lap ratio %.3f, residual max %.2fm, "
        "|d_sp| max %.2fm", n, length_ratio, max_residual,
        std::abs(*std::max_element(sp_d_.begin(), sp_d_.end(),
            [](double a, double b) { return std::abs(a) < std::abs(b); })));
    pitwall::Event("SP 추월 라인 검증/매핑 완료");
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
    cycle_rejects_.clear();

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
        // Predictive Spliner: 예측 "따라잡는 시점의 충돌 구간"을 원시 관측과
        // 함께 유지한다 (별도 id). 클러스터링이 상대의 현재 위치~따라잡는
        // 지점을 한 박스로 합쳐 그 전체를 크게 회피하게 된다.
        // 과거처럼 원시를 대체(erase)하면 경로를 상대의 '현재' 위치와 충돌검사
        // 하지 않아 복귀 지점에서 상대와 접촉한다 (실측 0.01m).
        if (!perception_stale && use_prediction_ &&
            (this->now() - predicted_stamp_).seconds() < obs_timeout_sec_) {
            for (const auto &pred : predicted_obstacles_.obstacles) {
                if (!std::isfinite(pred.s_start) || !std::isfinite(pred.s_end) ||
                    !std::isfinite(pred.d_right) || !std::isfinite(pred.d_left) ||
                    !std::isfinite(pred.vs) || !std::isfinite(pred.vd) ||
                    pred.d_right > pred.d_left) {
                    perception_stale = true;
                    obs_copy.obstacles.clear();
                    break;
                }
                // 원본 상대가 정적으로 재분류됐으면(파킹) 예측 구간은 유령 —
                // GP가 과거 주행 속도로 만든 수십 m짜리 가짜 충돌구간이라 폐기
                auto it = cls_tracks_.find(pred.id);
                if (it != cls_tracks_.end() && it->second.has_flag &&
                    it->second.static_flag) {
                    continue;
                }
                auto p = pred;
                p.id = EncodePredictionTubeId(pred.id);
                // predictor는 "이미 전파됨" 의미로 is_static=true를 주지만,
                // 여기선 동적 상대의 미래 구간이다 — false로 되돌려야
                // SP 레인체인지/추월 부스트가 동작하고 경로 커밋(정적 전용)이
                // 움직이는 상대에 걸리지 않는다. 대신 예약 ID가 provenance를
                // 보존해 Predict/ETA 충돌검사에서 두 번 이동시키지 않게 한다.
                p.is_static = false;
                obs_copy.obstacles.push_back(p);
            }
        }
    }

    f1tenth_planning_msgs::msg::WpntArray evasion_wpnts;
    evasion_wpnts.header.stamp = this->now();
    evasion_wpnts.header.frame_id = "map";
    visualization_msgs::msg::MarkerArray mrks;

    bool evasion_valid = false;
    bool used_trailing = false;
    // 이번 사이클에 회피를 시도했는가. 장애물이 사라진 사이클에도 반드시
    // 갱신돼야 한다 — 안 그러면 in_ot_sector_가 true로 굳어 다음 장애물의
    // "회피 시도 진입" 상승엣지를 영원히 놓친다.
    bool try_evade_now = false;
    // 같은 이유로 아래 두 엣지 플래그도 사이클마다 갱신한다. 예전에는
    // if (!close_obs.empty()) 블록 안에서만 대입해서, 장애물이 사라지면
    // true로 굳어 두 번째 발생부터 pitwall 이벤트가 통째로 사라졌다.
    bool spline_fallback_now = false;
    bool static_stop_now = false;
    bool escape_arc_now = false;
    {
        std::lock_guard<std::mutex> lock(mutex_gb_wpnts_);
        // 붙어 있는 장애물들을 먼저 하나의 박스로 병합 — 이후의 최근접 선택,
        // 회피, 경로 검증이 모두 클러스터 단위로 일관되게 동작한다
        obs_copy.obstacles = ClusterObstacles(obs_copy.obstacles);
        auto all_ahead_now = ObstaclesInLookahead(obs_copy.obstacles);

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
            lc_active_ = false;
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
            if (pos < span &&
                CheckPathCollision(committed_wpnts_.wpnts, stable_ahead, nullptr)) {
                evasion_wpnts.wpnts = committed_wpnts_.wpnts;
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
                    "[trace] committed_break pos=%.2f/%.2f stable_obs=%zu",
                    pos, span, stable_ahead.size());
                committed_until_s_ = -1.0;
            }
        }

        // 디버그: lookahead 내 장애물 구성 (동적 상대 상호작용 분석용)
        for (const auto &o : all_ahead_now) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "[obs] id=%d s=[%.1f,%.1f] d=[%.2f,%.2f] vs=%.1f static=%d",
                o.id, o.s_start, o.s_end, o.d_right, o.d_left, o.vs,
                o.is_static ? 1 : 0);
        }

        auto close_obs = evasion_valid
            ? std::vector<f1tenth_planning_msgs::msg::Obstacle>{}
            : FilterObstacles(obs_copy.obstacles);
        if (!close_obs.empty()) {
            // wrap-around을 고려한 전방 최근접 장애물 선택
            auto closest = *std::min_element(
                close_obs.begin(), close_obs.end(),
                [this](const auto &a, const auto &b) {
                    return Mod(a.s_end - cur_s_, track_len_) <
                           Mod(b.s_end - cur_s_, track_len_);
                });
            // SP 레인체인지 검증용: lookahead 내 전체 장애물.
            // obs_copy.obstacles 는 ClusterObstacles 이후 수정되지 않으므로
            // 위에서 만든 all_ahead_now 를 그대로 쓴다 (장애물마다
            // PredictObstacleMovement 를 다시 도는 중복 계산 제거).
            const auto &all_ahead = all_ahead_now;
            // 섹터 게이팅: 장애물(예측 장애물이면 따라잡는 지점)이 추월 섹터
            // 안일 때만 회피 시도. 섹터 안은 공격 모드(경계 마진 완화 + 속도
            // 부스트), 밖은 TRAILING으로 따라가다 섹터에서 승부한다.
            // 동적 상대를 이미 추월 중(|d| 큼)이면 섹터 이탈로 공격을 끊지
            // 않는다 — 끊기면 trailing d램프가 차를 상대 위치(raceline)로
            // 도로 끌어당겨 접촉한다 (섹터 조각 경계에서 진동 실측).
            // ⚠️ 이 mid_pass 유지가 동적 상대에 대한 유일한 방어다.
            //    BuildTrailing 의 retry_d 유지(d_goal = retry_d_target_)는
            //    allow_stop=false 일 때만 걸리는데 동적은 allow_stop 이 항상
            //    true 라 적용되지 않는다 — 의도적으로 정적 전용으로 둔 것이다
            //    (2026-08-10 결정). 동적까지 확장하려면 상대가 계속 움직이는
            //    동안 목표 d 를 어떻게 갱신할지부터 정해야 한다.
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
            bool try_evade = in_sector || closest.is_static;
            try_evade_now = try_evade;
            active_speed_scale_ = in_sector ? ot_speed_scale_ : 1.0;
            // 회피 시도 진입 알림 (try_evade 상승엣지)
            if (try_evade && !evade_attempt_active_) {
                pitwall::Event(in_sector ? "추월 섹터 — 라인 스위치 시도"
                                         : "정적 장애물 — 라인 스위치 시도");
            }

            // ── Tier 1: 라인 스위치 (기본 maneuver) ─────────────────────────
            // 오프셋 레인(±apex)으로 ease-on→hold→ease-back(rule-based). 정적
            // 장애물은 항상, 동적 상대는 추월 섹터 안에서만(try_evade) 시도한다
            // — 코너의 긴 오프셋 추월은 페이스 우위를 상쇄하므로 코너 동적은
            // SP(Tier 1b)/트레일링 몫.
            if (use_lane_switch_ && try_evade) {
                evasion_valid = TryLaneSwitch(closest, all_ahead,
                                              evasion_wpnts, mrks);
            }

            // ── Tier 1b: SP 레인체인지 (옵션, 동적 상대 전용) ───────────────
            // 생성기와 동일하게 추월 섹터에서만 SP를 시작한다. 이미 추월 중이면
            // mid_pass가 try_evade를 유지하므로 코너/섹터 경계에서 끊기지 않는다.
            bool lc_done = false;
            if (!evasion_valid && sp_lane_change_ && has_sp_ &&
                !closest.is_static && try_evade) {
                lc_done = BuildSpLaneChange(closest, all_ahead,
                                            evasion_wpnts, mrks);
                if (lc_done && !lc_active_) {
                    pitwall::Event("SP 레인체인지 추월");
                }
                evasion_valid = lc_done;
            }
            lc_active_ = lc_done;

            // ── Tier 2: free-form 스플라인 폴백 (모든 레인 막힘) ────────────
            // 고정 오프셋/SP 레인이 전부 막혔을 때만 DoSpline을 "호출"한다 —
            // 자연 3차 스플라인이 고정 레인이 못 덮는 틈으로 d(s)를 최적화해
            // 마지막으로 회피 해를 찾는다.
            bool used_spline = false;
            if (!evasion_valid && try_evade) {
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
                //  - 정적: Tier1/2가 조향 기하 하한 초과로 거부했을 때만
                //  - 동적: 트레일링은 원래 "따라가기"이므로 그대로 (v_target=상대속도)
                const bool geom_blocked = lane_switch_geom_infeasible_ ||
                                          free_spline_geom_infeasible_;
                const bool allow_stop = geom_blocked || !closest.is_static;
                evasion_wpnts.wpnts.clear();
                mrks.markers.clear();
                evasion_valid = BuildTrailing(closest, evasion_wpnts, mrks,
                                              allow_stop);
                used_trailing = evasion_valid;
                if (used_trailing && closest.is_static &&
                    !static_stop_active_) {
                    pitwall::Event(geom_blocked
                        ? "정적 장애물 정지 (회피 기하 불성립)"
                        : "정적 장애물 감속 (회피 일시 실패 — 재시도)");
                }
                static_stop_now = used_trailing && closest.is_static;
            }

            TraceCycle(&closest, false, try_evade, in_sector,
                       !evasion_valid   ? "none"
                       : used_trailing  ? "trailing"
                       : used_escape    ? "escape_arc"
                       : used_spline    ? "free_spline"
                       : lc_done        ? "sp_lane_change"
                                        : "lane_switch");

            // 정적 장애물 회피 성공 -> 경로 끝까지 커밋 (동적/예측 상대는
            // 움직이므로 커밋하지 않고 기존처럼 매 사이클 갱신)
            if (evasion_valid && !used_trailing &&
                closest.is_static && !evasion_wpnts.wpnts.empty()) {
                committed_wpnts_ = evasion_wpnts;
                committed_mrks_ = mrks;
                committed_start_s_ = evasion_wpnts.wpnts.front().s_m;
                committed_until_s_ = evasion_wpnts.wpnts.back().s_m;
            }
        } else if (lc_active_ && !evasion_valid &&
                   sp_lane_change_ && has_sp_) {
            // ego 노즈가 상대 앞머리(s_end)를 넘으면 상대는 회피 대상
            // 리스트에서 빠지지만, 속도차가 작으면 아직 "옆에 나란히"다 —
            // 이때 raceline으로 복귀하면 반 차신도 안 앞선 상대 앞을
            // 가로지르거나 도로 재추월당한다. 상대 앞머리가 ego 뒤로
            // lc_pass_clear_m 벌어질 때까지 SP 라인을 유지하며 추월
            // 페이스(v_floor)를 강제하고, 그 뒤에야 코너 밖에서 복귀한다.
            bool opp_beside = false;
            double v_floor = 0.0;
            for (const auto &o : obs_copy.obstacles) {
                if (o.is_static) {
                    continue;
                }
                double gap_back = Mod(cur_s_ - o.s_end, track_len_);
                if (gap_back < lc_pass_clear_m_) {
                    opp_beside = true;
                    v_floor = std::max(v_floor, std::max(o.vs, 0.0) /
                        std::max(cmd_speed_scale_, 0.05) * dyn_pass_speed_ratio_);
                }
            }
            evasion_valid = BuildSpExit(opp_beside, v_floor, all_ahead_now,
                                        evasion_wpnts, mrks);
            if (!evasion_valid) {
                pitwall::Event("추월 완료 — 아웃코스 복귀");
            }
            lc_active_ = evasion_valid;
        }
    }
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
        auto obs = PredictObstacleMovement(raw_obs);
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
        const bool projected_tube = IsPredictionTube(obs);
        // 정적 장애물은 시간이 지나도 안 움직이므로 vd_var의 시간 성장분을
        // 자른다 (static_time_uncertainty_cap_ 주석 참조). 위치 분산 d_var는
        // 그대로 — 관측 오차는 정적이든 아니든 존재한다.
        const double uncertainty_eta = projected_tube ? 0.0
            : (obs.is_static ? std::min(eta, static_time_uncertainty_cap_) : eta);
        const double lateral_uncertainty =
            UncertaintyMargin(d_var, vd_var, uncertainty_eta);
        const double lateral_motion =
            (obs.is_static || projected_tube) ? 0.0 : obs.vd * eta;
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

f1tenth_planning_msgs::msg::Obstacle SplinerNode::PredictObstacleMovement(
    f1tenth_planning_msgs::msg::Obstacle obs) const {
    // 동적 장애물을 고정 시간만큼 등속 전파 (ForzaETH "constant" 모드)
    // prediction tube는 이미 미래시점 좌표이므로 전략상 dynamic이어도 고정한다.
    if (obs.is_static || IsPredictionTube(obs) ||
        Mod(obs.s_center - cur_s_, track_len_) >= lookahead_m_) {
        return obs;
    }
    double delta_s = fixed_pred_time_ * obs.vs;
    double delta_d = fixed_pred_time_ * obs.vd;

    obs.s_start = Mod(obs.s_start + delta_s, track_len_);
    obs.s_center = Mod(obs.s_center + delta_s, track_len_);
    obs.s_end = Mod(obs.s_end + delta_s, track_len_);
    obs.d_left += delta_d;
    obs.d_center += delta_d;
    obs.d_right += delta_d;
    obs.s_var = std::max(obs.s_var, 0.0) +
        fixed_pred_time_ * fixed_pred_time_ * std::max(obs.vs_var, 0.0);
    obs.d_var = std::max(obs.d_var, 0.0) +
        fixed_pred_time_ * fixed_pred_time_ * std::max(obs.vd_var, 0.0);
    return obs;
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
    const double passable_width =
        RequiredObstacleClearance(obs) +
        RequiredBoundaryMargin(spline_bound_mindist_);
    // (예전엔 여기서 spliner/evasion_side_space_min(0.40)과 max를 취했다.
    //  cc22ca9 도입 시점엔 passable_width 와 같은 값이었지만, ff8c934 가 첫 항을
    //  RequiredObstacleClearance(= 같은 항 + UncertaintyMargin >= 0)로 바꾼 뒤로
    //  passable_width 가 항상 0.40 이상이라 그 max 는 항등이었다.)
    const double min_space = passable_width;

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
        return (left_gap >= right_gap)
            ? std::pair<std::string, double>{"left", apex_left}
            : std::pair<std::string, double>{"right", apex_right};
    }
    // 둘 다 통과 가능 — 이때만 "raceline 에서 덜 벗어나는 쪽"이 의미가 있다.
    if (std::abs(apex_left) <= std::abs(apex_right)) {
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
    bool changed = true;
    while (changed && group.size() < kEvasionGroupMax) {
        changed = false;
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
                continue;          // 아직 사정권 밖 (나중 반복에서 다시 본다)
            }
            taken[i] = 1;
            group.push_back(o);
            reach = std::min(std::max(reach, rel(o.s_end) + lane_blend_out_m_),
                             lookahead_m_);
            changed = true;
        }
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
    double d = (side == "left") ? -std::numeric_limits<double>::max()
                                : std::numeric_limits<double>::max();
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

double SplinerNode::ApexFor(
    const std::string &side,
    const f1tenth_planning_msgs::msg::Obstacle &obs) const {
    const double boundary_margin =
        RequiredBoundaryMargin(spline_bound_mindist_);
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
        double corridor = std::abs(wall_left - obs.d_left);
        double surplus =
            std::max(corridor - dist_min - boundary_margin, 0.0);
        // corridor 여유를 장애물/벽에 반씩 배분한 지점 (넓으면 목표값이 더 작음)
        double balanced = obs.d_left + dist_min + surplus / 2.0;
        double target = obs.d_left + dist_target;
        return std::clamp(std::min(target, balanced), 0.0,
                          std::max(wall - boundary_margin, 0.0));
    }
    double wall = wall_right;
    double corridor = std::abs(wall_right + obs.d_right);
    double surplus =
        std::max(corridor - dist_min - boundary_margin, 0.0);
    double balanced = obs.d_right - dist_min - surplus / 2.0;
    double target = obs.d_right - dist_target;
    return std::clamp(std::max(target, balanced),
                      -std::max(wall - boundary_margin, 0.0), 0.0);
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
    const bool committed_side = std::abs(cur_d_) > side_switch_d_tresh_ &&
                                !last_ot_side_.empty() &&
                                ((cur_d_ > 0.0) ? (last_ot_side_ == "left")
                                                : (last_ot_side_ == "right"));

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
        } else if (!group_clears) {
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
        // 이 레인은 장애물 여유를 만족한다 — 뒤이은 속도/충돌 검증에서
        // 떨어지더라도 "여기로 가려던 중"이라는 사실은 남긴다. 트레일링이
        // raceline으로 되돌리는 대신 이 목표를 유지하는 근거가 된다
        // (retry_d_target_ 주석 참조).
        NoteRetryTarget(d_hold);
        if (BuildLaneSwitch(d_hold, closest_obs, hold_through_s, all_ahead,
                            out_wpnts, out_mrks)) {
            CommitOtSide(side, closest_obs);
            NoteReject("ok_lane_switch");
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
    const double entry_avail = std::max(dist_front - vehicle_half_length_m_, 0.0);

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
        double available = entry_avail + wpnt_dist_;
        double cap = BlendSpeedCap(d_hold - cur_d_, available, a_lat, kappa_in);
        if (!(cap > 0.0) && relax_possible) {
            // 이상 레인까지는 못 가더라도 "충돌만 벗어나는" 최소 오프셋으로는
            // 성립할 수 있다. 성립하면 목표를 그쪽으로 낮춰 회피를 살린다.
            const double relaxed_cap =
                BlendSpeedCap(d_hold_min - cur_d_, available, a_lat, kappa_in);
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
                BlendSpeedCap(d_hold - cur_d_, creep_available, a_lat, kappa_in);
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
                d_hold - cur_d_, BlendGeomFloor(d_hold - cur_d_, kappa_in),
                available, cur_vs_, kappa_in);
            NoteReject("lane_geom_floor");
            lane_switch_geom_infeasible_ = true;
            return false;
        }
        if (cap < plan_v) {
            entry_speed_cap = std::max(cap, plan_speed_floor_);
            plan_v = std::min(plan_v, entry_speed_cap);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "LaneSwitch 속도 캡 %.2f m/s (횡 %.2fm, 가용 %.2fm, a_lat %.1f)",
                entry_speed_cap, d_hold - cur_d_, available, a_lat);
        }
    }
    // 진입/복귀 거리는 상수가 아니라 그립 예산으로 정한다.
    //  - 진입 진폭은 d_hold 전체가 아니라 "현재 d에서 남은 양" — 매 사이클
    //    재생성되므로 이미 옮겨간 만큼은 다시 옮길 필요가 없다.
    //  - 복귀 진폭은 d_hold 전체 (raceline까지 되돌아와야 함).
    const double need_in = BlendDist(d_hold - cur_d_, plan_v, a_lat, kappa_in);
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
    // 아래에서 벽에 맞춰 줄일 수 있다 (복귀 블렌드 단축)
    double blend_out = std::max(
        lane_blend_out_m_,
        BlendDist(d_hold, PlanSpeed(), a_lat_out, kappa_out));
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
        const double hold_overrun_max =
            std::max(0.5 - vehicle_half_length_m_, 0.0);
        const double blend_in_cap = creep_relaxed
            ? std::max(hold_end, wpnt_dist_)
            : std::min(hold_start + hold_overrun_max, blend_dist_max_m_);
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
        if (early_switch_kappa_max_ > 0.0) {
            const double early_span =
                std::max(std::min(hold_start, blend_dist_max_m_), 0.0);
            if (early_span > blend_in) {
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
                        "LaneSwitch 조기 전환: 진입 %.2f -> %.2fm "
                        "(구간 벽여유 %.2f >= |d_hold| %.2f, kappa %.3f)",
                        blend_in, early_span, room, std::abs(d_hold),
                        kappa_early);
                    NoteReject("ok_early_switch");
                    blend_in = early_span;
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

    // 동적 상대면 추월 페이스 하한(v_floor) 강제 — 나란히 달리다 추월이 영원히
    // 안 끝나는 것 방지. 정적이면 0 (raceline 프로파일 그대로).
    double v_floor = closest_obs.is_static ? 0.0 :
        std::max(closest_obs.vs, 0.0) /
        std::max(cmd_speed_scale_, 0.05) * dyn_pass_speed_ratio_;

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
    const double need_out = BlendDist(d_hold, PlanSpeed(), a_lat_out, kappa_out);
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

        f1tenth_planning_msgs::msg::Wpnt wpnt;
        wpnt.id = static_cast<int32_t>(i);
        wpnt.s_m = s;
        wpnt.d_m = d;
        wpnt.x_m = xy[i].first;
        wpnt.y_m = xy[i].second;
        wpnt.d_left = gb_wpnts_[gi].d_left - d;
        wpnt.d_right = gb_wpnts_[gi].d_right + d;
        wpnt.psi_rad = gb_wpnts_[gi].psi_rad;   // 아래에서 실제 헤딩으로 갱신
        wpnt.kappa_radpm = gb_wpnts_[gi].kappa_radpm;
        double vx = std::min(gb_wpnts_[gi].vx_mps * active_speed_scale_, gb_vmax_);
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
        wpnt.vx_mps = vx;
        wpnt.ax_mps2 = gb_wpnts_[gi].ax_mps2;
        out_wpnts.wpnts.push_back(wpnt);
    }

    // 오프셋 경로의 실제 곡률로 속도 상한을 다시 잡고 역방향 감속 패스를
    // 건다. 직선(kappa~0)에서는 raceline 속도가 그대로 유지되고, 코너 진입
    // 전에서만 크게 감속된다.
    if (!ApplySpeedProfile(out_wpnts.wpnts)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "LaneSwitch 거부: 속도 프로파일 불성립 (기하곡률>조향한계 또는 감속 불가, v=%.1f)",
            cur_vs_);
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
            // 여기서 false를 돌려도 lane_switch/free_spline_geom_infeasible_
            // 플래그를 세우지 않으므로 Tier3는 정지가 아니라
            // trailing_retry_speed 감속으로 받는다 — 가로지르기보다 안전하다.
            const bool committed_to_desired =
                std::abs(cur_d_) > side_switch_d_tresh_ &&
                ((cur_d_ > 0.0) ? (desired == "left") : (desired == "right"));
            if (committed_to_desired) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                    "OT side 전환 거부: 이미 %s로 회피 중 (cur_d=%.2f) — "
                    "가로지르지 않고 감속",
                    desired.c_str(), cur_d_);
                NoteReject("side_switch_blocked_mid_evasion");
                return false;
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
    const double cur_unwrapped_pre = s_apex - Mod(s_apex - cur_s_, track_len_);
    // BuildLaneSwitch와 같은 기준: s_front는 장애물 앞 0.5m hold 시작점이지만
    // 횡전환을 끝내야 하는 진짜 경계는 충돌 박스(vehicle_half_length)다.
    const double entry_avail_pre =
        (s_apex - obs_span / 2.0 - vehicle_half_length_m_) - cur_unwrapped_pre;
    double entry_speed_cap = std::numeric_limits<double>::infinity();
    double plan_v = PlanSpeed();
    if (entry_avail_pre >= 0.0) {
        // 충돌만 벗어나는 최소 오프셋 (BuildLaneSwitch의 d_hold_min과 동일 논리)
        const double clear_margin = RequiredObstacleClearance(closest_obs);
        const bool apex_left = (d_apex >= closest_obs.d_left);
        const double d_apex_min = apex_left
            ? closest_obs.d_left + clear_margin
            : closest_obs.d_right - clear_margin;
        double cap = BlendSpeedCap(
            d_apex - cur_d_, entry_avail_pre + wpnt_dist_, a_lat, kappa_sp);
        const bool relax_possible =
            apex_left ? (d_apex_min < d_apex) : (d_apex_min > d_apex);
        if (!(cap > 0.0) && relax_possible) {
            const double relaxed = BlendSpeedCap(
                d_apex_min - cur_d_, entry_avail_pre + wpnt_dist_, a_lat,
                kappa_sp);
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
                d_apex - cur_d_, BlendGeomFloor(d_apex - cur_d_, kappa_sp),
                entry_avail_pre, kappa_sp);
            NoteReject("spline_geom_floor");
            free_spline_geom_infeasible_ = true;
            return false;
        }
        if (cap < plan_v) {
            entry_speed_cap = std::max(cap, plan_speed_floor_);
            plan_v = std::min(plan_v, entry_speed_cap);
        }
        // 기하 하한은 통과했다 — 이후 속도/경계/충돌 검증에서 떨어져도
        // 이 apex가 "가려던 목표"다 (TryLaneSwitch와 같은 이유).
        NoteRetryTarget(d_apex);
    }
    double knot_base = std::abs(pre_apex_dists_.front());
    double need_scale = (knot_base > 0.1)
        ? BlendDist(d_apex, plan_v, a_lat, kappa_sp) / knot_base : 1.0;
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
    const double cur_unwrapped = s_apex - Mod(s_apex - cur_s_, track_len_);
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
        if (std::abs(evasion_d[i]) > spline_resolution_) {
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

        f1tenth_planning_msgs::msg::Wpnt wpnt;
        wpnt.id = static_cast<int32_t>(i);
        wpnt.s_m = s_wrapped;
        wpnt.d_m = evasion_d[i];
        wpnt.x_m = xy[i].first;
        wpnt.y_m = xy[i].second;
        // 회피 지점 기준 남은 경계 거리 (d>0: 왼쪽으로 이동)
        wpnt.d_left = gb_wpnts_[gb_idx].d_left - evasion_d[i];
        wpnt.d_right = gb_wpnts_[gb_idx].d_right + evasion_d[i];
        wpnt.psi_rad = gb_wpnts_[gb_idx].psi_rad;  // 아래에서 실제 헤딩으로 갱신
        wpnt.kappa_radpm = gb_wpnts_[gb_idx].kappa_radpm;
        // 추월 섹터 내 공격 부스트 (raceline 최고속 캡).
        {
            // (안쪽 회피의 고정 배율 inside_speed_scale 제거: cc22ca9 가
            //  offset_grip_cap(OffsetKappa + 레이스라인 그립 일관성 캡)으로
            //  대체한다고 명시했는데 이 곱셈만 남아 있었다. 배포 yaml 3개가
            //  모두 1.0 이라 궤적 변화는 없고, 코드 기본값 0.9 와의 불일치만
            //  사라진다 — yaml 없이 띄우면 offset_grip_cap 과 이중 감속이었다.)
            wpnt.vx_mps = std::min(
                gb_wpnts_[gb_idx].vx_mps * active_speed_scale_, gb_vmax_);
            // 동적 상대 추월 중에는 상대보다 확실히 빠르게 (감속 요소가 상대
            // 속도 밑으로 내려가면 옆에서 나란히 달리다 추월이 영원히 안 끝난다)
            if (!closest_obs.is_static) {
                double v_floor = std::max(closest_obs.vs, 0.0) /
                    std::max(cmd_speed_scale_, 0.05) * dyn_pass_speed_ratio_;
                wpnt.vx_mps = std::clamp(
                    wpnt.vx_mps, std::min(v_floor, gb_vmax_), gb_vmax_);
            }
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
        wpnt.ax_mps2 = gb_wpnts_[gb_idx].ax_mps2;
        out_wpnts.wpnts.push_back(wpnt);
    }

    // 오프셋 곡률 기반 속도 상한 + 역방향 감속 패스 (레인스위치와 동일 기준)
    if (!ApplySpeedProfile(out_wpnts.wpnts)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "FreeSpline 거부: 속도 프로파일 불성립 (기하곡률>조향한계 또는 감속 불가, v=%.1f)",
            cur_vs_);
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
    NoteReject("ok_free_spline");

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
    // (1) 정적 + 지금 실제로 보이는 것만. 외삽 유령에 급조향하면 벽으로 간다.
    if (!closest_obs.is_static || !closest_obs.is_visible) {
        NoteReject("escape_not_visible_static");
        return false;
    }
    // 이미 느리면 정상 회피가 성립한다 (그립 곡률이 1/v² 로 커진다).
    // 굳이 헤딩을 남기는 기동을 할 이유가 없다.
    if (cur_vs_ < escape_arc_min_speed_) {
        return false;
    }
    // 장애물 앞머리까지 남은 거리. 이미 옆에 들어왔으면 원호로도 못 피한다.
    const double dist_front =
        Mod(closest_obs.s_start - cur_s_, track_len_) - vehicle_half_length_m_;
    if (!(dist_front > 0.2) || dist_front > lookahead_m_) {
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
    const bool left_ok = (d_left_target <= wall_left - boundary);
    const bool right_ok = (d_right_target >= wall_right + boundary);
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
        return false;
    }
    // 원호 하한: A = kappa*L²/2  ->  L = sqrt(2A/kappa).
    const double arc_floor = std::sqrt(2.0 * a_amp / kappa_use);
    if (arc_floor > dist_front) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "EscapeArc 불가: %.2fm 횡이동에 원호 하한 %.2fm 필요, 가용 %.2fm "
            "(v=%.1f, kappa_use=%.2f)",
            a_amp, arc_floor, dist_front, cur_vs_, kappa_use);
        NoteReject("escape_geom_floor");
        return false;
    }
    // 실제로 쓸 길이는 가용 전체 (완만할수록 좋다). 그때의 원호 곡률로
    // 헤딩 잔여를 검사한다 — 이게 (3)번 전제다.
    const double arc_len = dist_front;
    const double kappa_arc = 2.0 * a_amp / (arc_len * arc_len);
    const double heading_left = kappa_arc * arc_len;
    if (heading_left > escape_arc_max_heading_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "EscapeArc 불가: 기동 끝 헤딩 %.0f도 (상한 %.0f도)",
            heading_left * 180.0 / M_PI,
            escape_arc_max_heading_ * 180.0 / M_PI);
        NoteReject("escape_heading_over");
        return false;
    }
    // 이 원호를 실제로 낼 수 있는 속도 상한 (그립).
    const double v_cap = std::sqrt(escape_arc_lat_acc_ / kappa_arc);

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
        if (t < arc_len) {
            const double w = (arc_len > 1e-6) ? (t / arc_len) : 1.0;
            arc_d[i] = cur_d_ + amplitude * w * w;   // 포물선 = 등곡률 근사
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

        f1tenth_planning_msgs::msg::Wpnt wpnt;
        wpnt.id = static_cast<int32_t>(i);
        wpnt.s_m = s;
        wpnt.d_m = arc_d[i];
        wpnt.x_m = xy.first;
        wpnt.y_m = xy.second;
        wpnt.d_left = gb_wpnts_[gi].d_left - arc_d[i];
        wpnt.d_right = gb_wpnts_[gi].d_right + arc_d[i];
        wpnt.psi_rad = gb_wpnts_[gi].psi_rad;
        wpnt.kappa_radpm = gb_wpnts_[gi].kappa_radpm;
        double vx = std::min(gb_wpnts_[gi].vx_mps, gb_vmax_);
        // 원호 구간에는 그립 캡을 건다. 그 뒤는 ApplySpeedProfile 몫.
        if (t <= arc_len) {
            vx = std::min(vx, v_cap / std::max(cmd_speed_scale_, 0.05));
        }
        wpnt.vx_mps = std::max(vx, plan_speed_floor_);
        wpnt.ax_mps2 = gb_wpnts_[gi].ax_mps2;
        out_wpnts.wpnts.push_back(wpnt);
    }

    // 통상 회피와 똑같은 검증기를 통과해야 한다 — 이 기동만 예외로 두면
    // "마지막 수단"이 아니라 "검증 안 된 경로"가 된다.
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
        "EscapeArc: %.2fm 횡이동을 %.2fm 원호로 이탈 (하한 %.2f, S자였다면 %.2f), "
        "kappa %.2f, 잔여헤딩 %.0f도, v캡 %.1f",
        a_amp, arc_len, arc_floor, BlendGeomFloor(a_amp, kappa_track),
        kappa_arc, heading_left * 180.0 / M_PI, v_cap);
    NoteReject("ok_escape_arc");
    return true;
}

bool SplinerNode::BuildTrailing(
    const f1tenth_planning_msgs::msg::Obstacle &obs,
    f1tenth_planning_msgs::msg::WpntArray &out_wpnts,
    visualization_msgs::msg::MarkerArray &out_mrks,
    bool allow_stop) {

    int gb_size = static_cast<int>(gb_wpnts_.size());
    if (gb_size < 2) {
        return false;
    }
    const bool emergency_stop =
        obs.id == std::numeric_limits<int32_t>::min();
    // 목표 속도: 동적(또는 예측) 장애물이면 그 진행 속도, 정적이면 0.
    // 제동 방정식은 실제 m/s 단위에서 계산하고 마지막에 wpnt 단위로 환산한다.
    const double command_scale = std::max(cmd_speed_scale_, 0.05);
    double v_target_actual = std::max(obs.vs, 0.0);

    // 정지 목표점: 장애물 앞 가장자리에서 trailing_gap 만큼 뒤
    double span = Mod(obs.s_start - trailing_gap_m_ - cur_s_, track_len_);
    if (span > lookahead_m_ + 5.0) {
        span = 0.0;      // 이미 gap 안쪽까지 파고든 상태
        v_target_actual *= 0.85;  // 살짝 느리게 가서 간격을 다시 벌린다
    }

    // 상대는 코너에서 감속한다 — 지금 측정 속도로만 프로파일을 짜면 상대가
    // 헤어핀 진입 중 감속할 때 gap이 무너져 상대를 관통한다 (실측).
    // 상대도 코너 한계속도(raceline 속도 프로파일) 이상으로는 못 돈다고
    // 가정하고, 정지점~장애물 구간 raceline 최저 속도를 목표 속도 상한으로.
    // (raceline 대비 비율 유지 모델은 정속 주행 상대를 과잉 감속 예측해
    //  ego가 상대 절반 속도로 기어가는 부작용 — 실측 폐기)
    double v_target_prof_actual = v_target_actual;
    if (obs.vs > 0.1) {
        int stop_gi = static_cast<int>(
            Mod(obs.s_start - trailing_gap_m_, track_len_) / wpnt_dist_) % gb_size;
        int n_ahead = static_cast<int>(
            (trailing_gap_m_ + 5.0) / std::max(wpnt_dist_, 0.05));
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
    if (!emergency_stop && !allow_stop && has_retry_d_) {
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
    // 램프 전 구간이 트랙 경계 안인가.
    auto ramp_ok = [&](double goal, double len) {
        for (int i = 0; i < n; ++i) {
            const double dist = i * wpnt_dist_;
            const double s = Mod(cur_s_ + dist, track_len_);
            const int gi = static_cast<int>(s / wpnt_dist_) % gb_size;
            const double t = std::min(dist / std::max(len, 1e-3), 1.0);
            const double d = cur_d_ + (goal - cur_d_) * t;
            if (!WithinTrackBoundary(gi, d, spline_bound_mindist_)) {
                return false;
            }
        }
        return true;
    };
    double d_ramp_len = ramp_len_for(d_goal);
    bool hold_current_d = emergency_stop;
    if (!hold_current_d && !ramp_ok(d_goal, d_ramp_len)) {
        // 회피 목표 유지가 경계를 침범하면 예전 동작(raceline 복귀)으로 내려간다.
        d_goal = 0.0;
        d_ramp_len = ramp_len_for(0.0);
        if (std::abs(cur_d_) > 0.02 && !ramp_ok(0.0, d_ramp_len)) {
            // 안전한 raceline 복귀 기하도 없으면 빈 경로 대신 현재 d에서
            // 정지한다. 빈 경로는 곧 전속 global path이므로 금지.
            hold_current_d = true;
            v_target_prof_actual = 0.0;
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
        } else if (std::abs(d_goal - cur_d_) > 0.05) {
            const double t = std::min(dist / std::max(d_ramp_len, 1e-3), 1.0);
            d = cur_d_ + (d_goal - cur_d_) * t;
        } else {
            d = d_goal;
        }
        w.d_m = d;
        if (std::abs(d) > 0.02) {
            if (!hold_current_d &&
                !WithinTrackBoundary(gi, d, spline_bound_mindist_)) {
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
        double rem = std::max(span - dist, 0.0);
        if (hold_current_d) {
            // 최대 제동 램프. 예전에는 여기서 전 점 0을 썼는데, 장애물이
            // trailing_gap 안쪽이면 span이 modulo로 감겨 0이 되고(그러면
            // rem도 0) 정상 경로에서도 같은 계단이 나왔다.
            w.vx_mps = std::min(w.vx_mps, reachable_v(dist) / command_scale);
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
            const double aT = trailing_decel_ * trailing_lag_sec_;
            const double v_allow = std::max(
                -aT + std::sqrt(
                    aT * aT + v_target_prof_actual * v_target_prof_actual +
                    2.0 * trailing_decel_ * rem),
                0.0);
            // 도달 가능 곡선 아래로는 내리지 않는다 (계단 -> 램프).
            w.vx_mps = std::min(
                w.vx_mps,
                std::max(v_allow, reachable_v(dist)) / command_scale);
        }
        out_wpnts.wpnts.push_back(w);
    }

    // 횡복귀 램프로 오프셋된 구간의 곡률/그립도 반영 (위 trailing 프로파일보다
    // 낮은 값만 채택되므로 제동 프로파일을 약화시키지 않는다)
    // 이미 위험 구간이면 최대 제동 경로를 반드시 발행해야 하므로 현재 상태
    // feasibility 실패를 이유로 trailing 자체를 버리지는 않는다.
    ApplySpeedProfile(out_wpnts.wpnts, false);

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
    NoteReject("trailing");
    return true;
}

bool SplinerNode::BuildSpLaneChange(
    const f1tenth_planning_msgs::msg::Obstacle &closest_obs,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> &all_ahead,
    f1tenth_planning_msgs::msg::WpntArray &out_wpnts,
    visualization_msgs::msg::MarkerArray &out_mrks) {

    int gb_size = static_cast<int>(gb_wpnts_.size());
    if (gb_size < 2 || sp_d_.empty()) {
        return false;
    }

    // 상대(클러스터) 구간을 ego 기준 상대 거리로. 추월 중(ego가 상대 s 구간
    // 옆)이면 dist_front가 wrap으로 거대해진다 — 진행 중인 레인체인지는
    // 끊지 않고 SP 유지 경로를 계속 생성해야 코너 안에서 인코스를 지킨다
    // (끊기면 raceline 최소곡률 라인으로 복귀하며 인코스를 뺏김).
    double span = Mod(closest_obs.s_end - closest_obs.s_start, track_len_);
    double behind = Mod(cur_s_ - closest_obs.s_start, track_len_);
    bool alongside = behind < span + 2.0;   // ego가 상대 몸통 옆까지 진입
    double dist_front, dist_back;
    if (alongside) {
        if (!lc_active_) {
            return false;   // 옆에서 새로 레인체인지 시작은 기하가 안 나옴
        }
        dist_front = 0.0;
        dist_back = Mod(closest_obs.s_end - cur_s_, track_len_);
    } else {
        dist_front = Mod(closest_obs.s_start - cur_s_, track_len_);
        dist_back = dist_front + span;
        if (dist_front > lookahead_m_ + 10.0) {
            return false;  // 너무 멀다
        }
        if (dist_front < 3.0 && !lc_active_) {
            return false;  // 블렌드가 상대 앞에서 못 끝날 만큼 늦은 시작
        }
    }

    // 진입/복귀 역시 현재 속도에서 횡가속 예산을 만족하는 BlendDist로 정한다.
    // 장애물 전에 필요한 거리가 없으면 짧은 고정 blend를 억지로 넣지 않는다.
    const int cur_gi =
        static_cast<int>(Mod(cur_s_, track_len_) / wpnt_dist_) % gb_size;
    const double need_in = BlendDist(sp_d_[cur_gi] - cur_d_, cur_vs_);
    const double available_in = std::max(
        dist_front - 1.0 - vehicle_half_length_m_, 0.0);
    if (!lc_active_ && need_in > available_in + wpnt_dist_) {
        return false;
    }
    double blend_in = lc_active_
        ? std::max(1.0, need_in)
        : std::clamp(available_in, std::max(need_in, 2.0),
                     std::max(blend_dist_max_m_, std::max(need_in, 2.0)));
    const double hold_exit = 5.0;
    double max_sp_offset = 0.0;
    for (double d : sp_d_) {
        max_sp_offset = std::max(max_sp_offset, std::abs(d));
    }
    const double blend_out = std::max(6.0, BlendDist(max_sp_offset, cur_vs_));
    // 인코스 진입 - 아웃코스 탈출: 복귀 블렌드 창이 코너(|kappa| >
    // ot_max_kappa)와 겹치면 코너가 끝날 때까지 SP 유지 구간을 연장한다.
    // raceline은 코너 출구에서 바깥으로 벌어지므로 출구에서의 복귀가
    // 그대로 아웃코스 라인이 된다.
    double hold_end = dist_back + hold_exit +
        SpHoldExtension(cur_s_ + dist_back + hold_exit, blend_out);
    double total = hold_end + blend_out;
    if (!PlanSpanAllowed(total)) {
        return false;
    }

    int n = static_cast<int>(total / wpnt_dist_);
    if (n < 10) {
        return false;
    }
    out_wpnts.wpnts.reserve(n);
    std::vector<std::pair<double, double>> xy(n);
    for (int i = 0; i < n; ++i) {
        double t = i * wpnt_dist_;
        double s = Mod(cur_s_ + t, track_len_);
        int gi = static_cast<int>(s / wpnt_dist_) % gb_size;
        double d_target = sp_d_[gi];
        double d;
        if (t < blend_in) {
            double w = t / blend_in;
            w = w * w * (3.0 - 2.0 * w);  // smoothstep
            d = cur_d_ * (1.0 - w) + d_target * w;
        } else if (t < hold_end) {
            d = d_target;
        } else {
            double w = std::clamp((t - hold_end) / blend_out, 0.0, 1.0);
            w = w * w * (3.0 - 2.0 * w);
            d = d_target * (1.0 - w);
        }
        if (!WithinTrackBoundary(gi, d, spline_bound_mindist_)) {
            out_wpnts.wpnts.clear();
            return false;
        }
        xy[i] = FrenetToCartesian(s, d);

        f1tenth_planning_msgs::msg::Wpnt wpnt;
        wpnt.id = static_cast<int32_t>(i);
        wpnt.s_m = s;
        wpnt.d_m = d;
        wpnt.x_m = xy[i].first;
        wpnt.y_m = xy[i].second;
        wpnt.d_left = gb_wpnts_[gi].d_left - d;
        wpnt.d_right = gb_wpnts_[gi].d_right + d;
        wpnt.psi_rad = gb_wpnts_[gi].psi_rad;  // 아래에서 실제 헤딩으로 갱신
        wpnt.kappa_radpm = gb_wpnts_[gi].kappa_radpm;
        // 속도: SP 라인 프로파일 + 섹터 부스트, 상대보다 확실히 빠르게
        double vx = std::min(sp_vx_[gi] * active_speed_scale_, gb_vmax_);
        double v_floor = std::max(closest_obs.vs, 0.0) /
            std::max(cmd_speed_scale_, 0.05) * dyn_pass_speed_ratio_;
        wpnt.vx_mps = std::max(vx, std::min(v_floor, gb_vmax_));
        wpnt.ax_mps2 = gb_wpnts_[gi].ax_mps2;
        out_wpnts.wpnts.push_back(wpnt);
    }

    if (!ApplySpeedProfile(out_wpnts.wpnts)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "SpLaneChange 거부: 속도 프로파일 불성립 (기하곡률>조향한계 또는 감속 불가, v=%.1f)",
            cur_vs_);
        out_wpnts.wpnts.clear();
        return false;
    }

    // 상대 "자신 포함" 전체 장애물과 충돌검사 — SP 라인이 그 구간에서
    // 상대 주행 라인과 충분히 벌어지지 않으면 레인체인지 불가
    double blocking_s = -1.0;
    if (!CheckPathCollision(out_wpnts.wpnts, all_ahead, &blocking_s)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "SpLaneChange 거부: 경로가 장애물과 충돌 (blocking s=%.2f)", blocking_s);
        out_wpnts.wpnts.clear();
        return false;
    }

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
    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
        "SP lane change: obs s=%.1f blend_in=%.1fm hold_end=%.1fm (%zu wpnts)",
        closest_obs.s_center, blend_in, hold_end, out_wpnts.wpnts.size());
    return true;
}

double SplinerNode::SpHoldExtension(double s_from, double blend_len) const {
    // s_from부터 시작해, raceline 복귀 블렌드 창(blend_len)이 코너
    // (|kappa| > ot_max_kappa)와 겹치지 않게 되는 최소 추가 유지 거리.
    // 코너 안에서 복귀가 시작되면 인코스를 잃는다 — 복귀는 코너 출구에서만.
    int gb_size = static_cast<int>(gb_wpnts_.size());
    const double cap = 25.0;   // 연속 코너 무한 유지 방지
    for (double extra = 0.0; extra <= cap; extra += 1.0) {
        bool clear = true;
        for (double ds = 0.0; ds <= blend_len; ds += 1.0) {
            int gi = static_cast<int>(
                Mod(s_from + extra + ds, track_len_) / wpnt_dist_) % gb_size;
            if (std::abs(gb_wpnts_[gi].kappa_radpm) > ot_max_kappa_) {
                clear = false;
                break;
            }
        }
        if (clear) {
            return extra;
        }
    }
    return cap;
}

bool SplinerNode::BuildSpExit(
    bool opp_beside, double v_floor,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> &all_ahead,
    f1tenth_planning_msgs::msg::WpntArray &out_wpnts,
    visualization_msgs::msg::MarkerArray &out_mrks) {
    // 추월 마무리 경로: 상대가 회피 대상에서 사라져도 (1) 아직 옆에
    // 나란하면(opp_beside) 간격이 벌어질 때까지, (2) 코너 안이면 코너
    // 출구까지 SP 인코스를 유지한 뒤 raceline(아웃코스)으로 복귀한다.
    // 매 사이클 ego 위치에서 재생성되며, 상대 클리어 + 직선 도달 +
    // raceline 복귀 완료 시 false를 반환해 종료된다.
    int gb_size = static_cast<int>(gb_wpnts_.size());
    if (gb_size < 2 || sp_d_.empty()) {
        return false;
    }
    const int cur_gi =
        static_cast<int>(Mod(cur_s_, track_len_) / wpnt_dist_) % gb_size;
    const double blend_in =
        std::max(1.0, BlendDist(sp_d_[cur_gi] - cur_d_, cur_vs_));
    double max_sp_offset = 0.0;
    for (double d : sp_d_) {
        max_sp_offset = std::max(max_sp_offset, std::abs(d));
    }
    const double blend_out =
        std::max(6.0, BlendDist(max_sp_offset, cur_vs_));
    double hold = SpHoldExtension(cur_s_ + blend_in, blend_out);
    if (opp_beside) {
        // 상대가 아직 옆/바로 뒤 — 복귀 블렌드가 경로에 아예 안 들어오게
        // lookahead 전체를 유지 구간으로 잡는다 (간격이 벌어지면 자연 해제)
        hold = std::max(hold, lookahead_m_);
    } else if (hold <= 0.0 && std::abs(cur_d_) < 0.2) {
        return false;   // 상대 클리어 + 코너 끝 + raceline 복귀 완료
    }
    double hold_end = blend_in + hold;
    double total = hold_end + blend_out;
    bool defer_return = false;
    if (!PlanSpanAllowed(total)) {
        // 한 바퀴보다 긴 modulo 경로는 local splice에서 붕괴한다. 코너가 너무
        // 길면 이번 cycle은 SP를 유지만 하고 다음 cycle에 복귀를 재평가한다.
        total = std::max(track_len_ - wpnt_dist_, wpnt_dist_);
        hold_end = total;
        defer_return = true;
    }
    int n = static_cast<int>(total / wpnt_dist_);
    if (n < 5) {
        return false;
    }

    out_wpnts.wpnts.reserve(n);
    std::vector<std::pair<double, double>> xy(n);
    for (int i = 0; i < n; ++i) {
        double t = i * wpnt_dist_;
        double s = Mod(cur_s_ + t, track_len_);
        int gi = static_cast<int>(s / wpnt_dist_) % gb_size;
        double d_target = sp_d_[gi];
        double d;
        if (t < blend_in) {
            double w = t / blend_in;
            w = w * w * (3.0 - 2.0 * w);
            d = cur_d_ * (1.0 - w) + d_target * w;
        } else if (t < hold_end) {
            d = d_target;
        } else if (!defer_return) {
            double w = std::clamp((t - hold_end) / blend_out, 0.0, 1.0);
            w = w * w * (3.0 - 2.0 * w);
            d = d_target * (1.0 - w);
        } else {
            d = d_target;
        }
        if (!WithinTrackBoundary(gi, d, spline_bound_mindist_)) {
            out_wpnts.wpnts.clear();
            return false;
        }
        xy[i] = FrenetToCartesian(s, d);

        f1tenth_planning_msgs::msg::Wpnt wpnt;
        wpnt.id = static_cast<int32_t>(i);
        wpnt.s_m = s;
        wpnt.d_m = d;
        wpnt.x_m = xy[i].first;
        wpnt.y_m = xy[i].second;
        wpnt.d_left = gb_wpnts_[gi].d_left - d;
        wpnt.d_right = gb_wpnts_[gi].d_right + d;
        wpnt.psi_rad = gb_wpnts_[gi].psi_rad;  // 아래에서 실제 헤딩으로 갱신
        wpnt.kappa_radpm = gb_wpnts_[gi].kappa_radpm;
        // 속도: SP/raceline 프로파일 중 낮은 쪽 (블렌드 구간은 두 라인 사이).
        // 상대가 옆에 있으면 추월 페이스 하한(v_floor)을 강제 — 여기서
        // 느려지면 나란히 달리기만 하고 간격이 영원히 안 벌어진다
        wpnt.vx_mps = std::min({sp_vx_[gi], gb_wpnts_[gi].vx_mps, gb_vmax_});
        if (v_floor > 0.0) {
            wpnt.vx_mps = std::max(
                wpnt.vx_mps, std::min(v_floor, gb_vmax_));
        }
        wpnt.ax_mps2 = gb_wpnts_[gi].ax_mps2;
        out_wpnts.wpnts.push_back(wpnt);
    }

    if (!ApplySpeedProfile(out_wpnts.wpnts)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "SpExit 거부: 속도 프로파일 불성립 (기하곡률>조향한계 또는 감속 불가, v=%.1f)",
            cur_vs_);
        out_wpnts.wpnts.clear();
        return false;
    }
    double blocking_s = -1.0;
    if (!CheckPathCollision(out_wpnts.wpnts, all_ahead, &blocking_s)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "SpExit 거부: 경로가 장애물과 충돌 (blocking s=%.2f)", blocking_s);
        out_wpnts.wpnts.clear();
        return false;
    }

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
    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
        "SP exit: hold=%.1fm cur_d=%.2f (%zu wpnts)",
        hold, cur_d_, out_wpnts.wpnts.size());
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
        auto &tr = cls_tracks_[obs.id];
        tr.last_seen = now;
        // s는 첫 관측 기준으로 unwrap해 원형 좌표 std 왜곡 방지
        if (tr.s_hist.empty()) {
            tr.s_ref = obs.s_center;
        }
        tr.s_hist.push_back(
            Mod(obs.s_center - tr.s_ref + track_len_ / 2.0, track_len_) -
            track_len_ / 2.0);
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
        bool by_vs = tr.vs_hist.size() >= kClsVsWindow && vs_avg < cls_vs_reset_;

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
        return false;   // 판단 불가 — 기존 동작(병합) 유지 = 보수적
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
                        NoteReject("cluster_merge_blocks_track");
                        mergeable = false;
                    }
                }
                if (mergeable) {
                    auto &a = obs[i];
                    const auto &b = obs[j];
                    const bool projected_tube =
                        IsPredictionTube(a) || IsPredictionTube(b);
                    const auto merged_id = IsPredictionTube(a) ? a.id :
                        (IsPredictionTube(b) ? b.id : std::min(a.id, b.id));
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
                    // 현재 관측과 catch-zone이 합쳐지면 이미 swept tube이므로
                    // 예약 ID를 유지해 병합 후 전체 박스를 재전파하지 않는다.
                    a.id = projected_tube ? merged_id : std::min(a.id, b.id);
                    obs.erase(obs.begin() + static_cast<long>(j));
                    merged_any = true;
                }
            }
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
        auto obs = PredictObstacleMovement(raw);
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

std::vector<f1tenth_planning_msgs::msg::Obstacle> SplinerNode::StableObstacles(
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> &obstacles) const {
    std::vector<f1tenth_planning_msgs::msg::Obstacle> out;
    out.reserve(obstacles.size());
    for (const auto &o : obstacles) {
        // 예측 튜브는 관측 이력이 없다 — 항상 유효한 것으로 본다.
        if (IsPredictionTube(o)) {
            out.push_back(o);
            continue;
        }
        // 외삽(coasting)만으로는 이미 성립한 회피를 못 깬다. 관측 이력이
        // 충분해도 **지금 안 보이는** 물체는 위치가 KF 예측이라, 커밋 경로와의
        // 충돌 판정이 근거 없이 회피를 정지로 되돌린다 (rosbag_0811 실측).
        if (commit_break_requires_visible_ && !o.is_visible) {
            continue;
        }
        auto it = cls_tracks_.find(o.id);
        const int seen = (it == cls_tracks_.end())
            ? 0 : static_cast<int>(it->second.s_hist.size());
        if (seen >= commit_break_min_obs_) {
            out.push_back(o);
        }
    }
    return out;
}

bool SplinerNode::CheckPathCollision(
    const std::vector<f1tenth_planning_msgs::msg::Wpnt> &path,
    const std::vector<f1tenth_planning_msgs::msg::Obstacle> &obstacles,
    double *blocking_s) const {
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
            const bool projected_tube = IsPredictionTube(obs);
            const double motion_s =
                (obs.is_static || projected_tube) ? 0.0 : obs.vs * t;
            const double motion_d =
                (obs.is_static || projected_tube) ? 0.0 : obs.vd * t;
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
            const double uncertainty_t = projected_tube ? 0.0
                : (obs.is_static ? std::min(t, static_time_uncertainty_cap_) : t);
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
                if (blocking_s) {
                    *blocking_s = Mod(obs.s_start + motion_s, track_len_);
                }
                return false;  // 충돌
            }
        }
    }
    return true;  // 충돌 없음
}

bool SplinerNode::PlanSpanAllowed(double span) const {
    if (!std::isfinite(span) || !(span > 0.0) || !(track_len_ > wpnt_dist_)) {
        return false;
    }
    // front/back modulo 두 점으로 span을 복원하는 downstream 계약 때문에
    // 정확히 한 바퀴인 경로도 0m로 보인다. 최소 한 sample 여유를 둔다.
    return span < track_len_ - std::max(wpnt_dist_, 0.05);
}

double SplinerNode::RequiredBoundaryMargin(double requested_margin) const {
    return std::max(
        std::max(requested_margin, 0.0),
        vehicle_half_width_m_ + boundary_tracking_margin_m_);
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

double SplinerNode::BlendGeomFloor(double amplitude, double kappa_track,
                                   bool one_sided) const {
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
    if (std::isfinite(obs.s_center)) {
        ot_side_obs_s_ = Mod(obs.s_center, std::max(track_len_, 1e-3));
        ot_side_stamp_ = this->now();
        has_ot_side_obs_ = true;
    } else {
        has_ot_side_obs_ = false;
    }
}

bool SplinerNode::SideKeepActive(
    const f1tenth_planning_msgs::msg::Obstacle &obs) const {
    if (last_ot_side_.empty()) {
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
    double amplitude, double speed, double lat_acc, double kappa_track) const {
    // smoothstep 횡이동의 피크 횡가속 = 6*A*v^2/L^2 이므로
    // L = v*sqrt(6A/a_max). 거리를 상수로 두면 v^2로 커진다.
    const double a = std::abs(amplitude);
    const double v = std::max(std::abs(speed), 0.0);
    const double a_lat = (lat_acc > 0.0) ? lat_acc : blend_lat_acc_max_;
    if (a < 1e-3 || a_lat < 1e-3) {
        return 0.0;
    }
    double dist = v * std::sqrt(6.0 * a / a_lat);
    dist = std::max(dist, BlendGeomFloor(a, kappa_track));
    return std::min(dist, blend_dist_max_m_);
}

double SplinerNode::BlendSpeedCap(
    double amplitude, double available, double lat_acc,
    double kappa_track) const {
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
    if (available < BlendGeomFloor(a, kappa_track)) {
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
    const double budget = path_lat_acc_max_;
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
    // 정적 장애물은 시간 성장 캡(static_time_uncertainty_cap_), 동적은 예측
    // 시간(fixed_pred_time_)을 대표값으로 쓴다 — 검증기와 완전히 같지는 않지만
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
    bool require_current_feasible) const {
    if (wpnts.size() < 2) {
        return !wpnts.empty();
    }

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

    for (size_t i = 0; i < n; ++i) {
        if (!std::isfinite(wpnts[i].vx_mps) || wpnts[i].vx_mps < 0.0 ||
            !std::isfinite(wpnts[i].x_m) || !std::isfinite(wpnts[i].y_m)) {
            return false;
        }
        curvature[i] = OffsetKappa(wpnts[i].kappa_radpm, wpnts[i].d_m);
        speed[i] = wpnts[i].vx_mps * command_scale;
        kappa_ref[i] = std::abs(wpnts[i].kappa_radpm);
        if (kappa_ref[i] > kKappaRefMin) {
            lat_ref[i] = kappa_ref[i] * speed[i] * speed[i];
        }
        if (i + 1 < n) {
            ds[i] = std::max(std::hypot(
                wpnts[i + 1].x_m - wpnts[i].x_m,
                wpnts[i + 1].y_m - wpnts[i].y_m), 1e-3);
        }
    }

    // IQP 평행 오프셋 공식만으로는 직선 위 lane-change의 S자 곡률을 못 본다.
    // 0.3m baseline의 실제 xy 헤딩 변화가 더 크면 그 값을 사용한다.
    if (n >= 7) {
        constexpr size_t kWindow = 3;
        for (size_t i = kWindow; i + kWindow < n; ++i) {
            const double h0 = std::atan2(
                wpnts[i].y_m - wpnts[i - kWindow].y_m,
                wpnts[i].x_m - wpnts[i - kWindow].x_m);
            const double h1 = std::atan2(
                wpnts[i + kWindow].y_m - wpnts[i].y_m,
                wpnts[i + kWindow].x_m - wpnts[i].x_m);
            const double baseline = 0.5 * std::hypot(
                wpnts[i + kWindow].x_m - wpnts[i - kWindow].x_m,
                wpnts[i + kWindow].y_m - wpnts[i - kWindow].y_m);
            const double k_geom =
                WrapToPi(h1 - h0) / std::max(baseline, 1e-6);
            if (std::abs(k_geom) > std::abs(curvature[i])) {
                curvature[i] = k_geom;
            }
        }
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
        steering_curvature_limit * steer_kappa_reject_ratio_;
    if (path_lat_acc_max_ > 1e-3) {
        for (size_t i = 0; i < n; ++i) {
            const double k = std::abs(curvature[i]);
            if (!std::isfinite(k) || k > steering_curvature_reject + 1e-3) {
                NoteReject("speed_steer_curv");
                return false;
            }
            if (k > 1e-4) {
                speed[i] = std::min(
                    speed[i], std::sqrt(path_lat_acc_max_ / k));
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

    auto longitudinal_budget =
        [this](double base, double v, double kappa) {
            if (path_lat_acc_max_ <= 1e-3) {
                return base;
            }
            const double lat = std::abs(kappa) * v * v;
            const double ratio =
                std::clamp(lat / path_lat_acc_max_, 0.0, 1.0);
            return base * std::sqrt(std::max(1.0 - ratio * ratio, 0.0));
        };

    // 미래 곡률 cap을 만족하도록 역방향 감속. 횡그립을 쓰는 만큼 종그립을
    // friction-circle에서 차감해 두 한계를 동시에 최대로 쓰지 못하게 한다.
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
            if (path_lat_acc_max_ > 1e-3 && k > 1e-4 &&
                k * simulated_v * simulated_v > path_lat_acc_max_) {
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
    int i0 = static_cast<int>(idx_f) % gb_size;
    int i1 = (i0 + 1) % gb_size;
    double t = idx_f - std::floor(idx_f);

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
