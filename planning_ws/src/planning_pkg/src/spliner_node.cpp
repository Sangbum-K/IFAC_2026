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

// BlendGeomFloor에서 트랙 곡률이 조향 예산을 거의 다 먹었을 때 남겨두는
// 최소 비율. 이 아래로 내려가면 하한이 발산해 로그가 inf로 찍히고 상위
// 로직이 "가용 대비 얼마나 모자란가"를 표현할 수 없다.
constexpr double kBlendGeomKappaFloorRatio = 0.15;

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
    this->declare_parameter("spliner/inside_speed_scale", 0.9);
    // 임시 테스트: 회피 속도 프로파일(감속·캡·feasibility) 전면 우회.
    this->declare_parameter("spliner/bypass_speed_profile", false);
    this->declare_parameter("spliner/side_switch_d_tresh", 0.25);
    this->declare_parameter("spliner/obs_timeout_sec", 0.5);
    this->declare_parameter("spliner/use_prediction", true);
    this->declare_parameter("spliner/use_trailing", true);
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
    this->declare_parameter("spliner/evasion_dist_min", 0.35);
    this->declare_parameter("spliner/evasion_side_space_min", 0.40);
    this->declare_parameter("spliner/squeeze_speed_scale", 0.35);
    this->declare_parameter("spliner/squeeze_brake_dist_m", 6.0);
    this->declare_parameter("spliner/trailing_engage_dist_m", 8.0);
    this->declare_parameter("spliner/trailing_lag_sec", 0.3);
    this->declare_parameter("spliner/trailing_ramp_decel", 2.0);
    this->declare_parameter("spliner/trailing_retry_speed", 1.5);
    this->declare_parameter("spliner/apex_clearance_eps_m", 0.05);
    this->declare_parameter("spliner/commit_break_min_obs", 3);
    this->declare_parameter("spliner/creep_escape_speed", 0.6);
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
    this->declare_parameter("spliner/sp_lane_change", true);
    this->declare_parameter("spliner/sp_mapping_max_residual_m", 0.5);
    this->declare_parameter("spliner/vehicle_wheelbase_m", 0.33);
    this->declare_parameter("spliner/vehicle_max_steering_rad", 0.36);
    this->declare_parameter("spliner/vehicle_max_steering_rate_radps", 2.0);
    // 추월 완료 판정 s 간격: 상대 앞머리가 ego 뒤로 이만큼 벌어져야
    // 라인 복귀 시작 (그 전 복귀는 반 차신도 안 앞선 상대 앞 끼어들기)
    this->declare_parameter("spliner/lc_pass_clear_m", 4.0);
    this->declare_parameter("spliner/ot_sector_min_len_m", 8.0);
    this->declare_parameter("spliner/ot_bound_mindist", 0.25);
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
    inside_speed_scale_ = this->get_parameter("spliner/inside_speed_scale").as_double();
    bypass_speed_profile_ =
        this->get_parameter("spliner/bypass_speed_profile").as_bool();
    side_switch_d_tresh_ = this->get_parameter("spliner/side_switch_d_tresh").as_double();
    obs_timeout_sec_ = this->get_parameter("spliner/obs_timeout_sec").as_double();
    use_prediction_ = this->get_parameter("spliner/use_prediction").as_bool();
    use_trailing_ = this->get_parameter("spliner/use_trailing").as_bool();
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
    evasion_dist_min_ = this->get_parameter("spliner/evasion_dist_min").as_double();
    evasion_side_space_min_ =
        this->get_parameter("spliner/evasion_side_space_min").as_double();
    squeeze_speed_scale_ = this->get_parameter("spliner/squeeze_speed_scale").as_double();
    squeeze_brake_dist_m_ = this->get_parameter("spliner/squeeze_brake_dist_m").as_double();
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
    creep_escape_speed_ =
        this->get_parameter("spliner/creep_escape_speed").as_double();
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
    sp_lane_change_ = this->get_parameter("spliner/sp_lane_change").as_bool();
    sp_mapping_max_residual_m_ =
        this->get_parameter("spliner/sp_mapping_max_residual_m").as_double();
    vehicle_wheelbase_m_ =
        this->get_parameter("spliner/vehicle_wheelbase_m").as_double();
    vehicle_max_steering_rad_ =
        this->get_parameter("spliner/vehicle_max_steering_rad").as_double();
    vehicle_max_steering_rate_radps_ =
        this->get_parameter("spliner/vehicle_max_steering_rate_radps").as_double();
    lc_pass_clear_m_ = this->get_parameter("spliner/lc_pass_clear_m").as_double();
    ot_sector_min_len_m_ = this->get_parameter("spliner/ot_sector_min_len_m").as_double();
    ot_bound_mindist_ = this->get_parameter("spliner/ot_bound_mindist").as_double();
    ot_speed_scale_ = this->get_parameter("spliner/ot_speed_scale").as_double();
    ot_sectors_param_ = this->get_parameter("spliner/ot_sectors").as_double_array();
    active_bound_mindist_ = spline_bound_mindist_;

    // NaN에 std::max를 적용하면 그대로 남아 모든 비교를 우회할 수 있다.
    // local_trajectory가 이 노드의 heartbeat를 필수 lease로 사용하므로,
    // 잘못된 설정은 명확히 기동 거부 -> downstream timeout 정지로 처리한다.
    const std::vector<double> safety_values = {
        loop_rate_hz_, lookahead_m_, evasion_dist_, obs_traj_tresh_,
        spline_bound_mindist_, fixed_pred_time_, outside_spline_scale_,
        inside_speed_scale_, side_switch_d_tresh_, obs_timeout_sec_,
        trailing_gap_m_, trailing_decel_, cmd_speed_scale_,
        blend_lat_acc_max_, blend_dist_max_m_, path_lat_acc_max_,
        path_decel_max_, path_accel_max_, path_jerk_max_, planning_delay_sec_,
        vehicle_half_length_m_, vehicle_half_width_m_,
        boundary_tracking_margin_m_, collision_sigma_, path_check_margin_,
        static_time_uncertainty_cap_, max_uncertainty_m_, plan_speed_floor_,
        cluster_gap_s_m_, cluster_gap_d_m_, evasion_dist_min_,
        squeeze_speed_scale_, squeeze_brake_dist_m_, trailing_engage_dist_m_,
        dyn_pass_speed_ratio_, cls_min_std_, cls_max_std_, cls_vs_reset_,
        ot_min_clearance_, ot_max_kappa_, lane_blend_out_m_,
        sp_mapping_max_residual_m_, vehicle_wheelbase_m_,
        vehicle_max_steering_rad_, vehicle_max_steering_rate_radps_,
        lc_pass_clear_m_, ot_sector_min_len_m_, ot_bound_mindist_,
        ot_speed_scale_};
    if (!std::all_of(
            safety_values.begin(), safety_values.end(),
            [](double value) { return std::isfinite(value); }) ||
        loop_rate_hz_ < 1.0 || loop_rate_hz_ > 100.0 ||
        lookahead_m_ < 0.5 || lookahead_m_ > 100.0 ||
        evasion_dist_ <= 0.0 || evasion_dist_ > 5.0 ||
        evasion_dist_min_ <= 0.0 || evasion_dist_min_ > evasion_dist_ ||
        evasion_side_space_min_ <= 0.0 || evasion_side_space_min_ > 2.0 ||
        obs_traj_tresh_ < 0.0 || obs_traj_tresh_ > 5.0 ||
        spline_bound_mindist_ < 0.0 || spline_bound_mindist_ > 2.0 ||
        fixed_pred_time_ < 0.0 || fixed_pred_time_ > 2.0 ||
        outside_spline_scale_ <= 0.0 || outside_spline_scale_ > 5.0 ||
        inside_speed_scale_ <= 0.0 || inside_speed_scale_ > 1.0 ||
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
        squeeze_speed_scale_ <= 0.0 || squeeze_speed_scale_ > 1.0 ||
        squeeze_brake_dist_m_ <= 0.0 || squeeze_brake_dist_m_ > lookahead_m_ ||
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
        sp_mapping_max_residual_m_ < 0.05 || sp_mapping_max_residual_m_ > 1.0 ||
        vehicle_wheelbase_m_ < 0.1 || vehicle_wheelbase_m_ > 1.0 ||
        vehicle_max_steering_rad_ <= 0.0 || vehicle_max_steering_rad_ > 0.8 ||
        vehicle_max_steering_rate_radps_ <= 0.0 ||
        vehicle_max_steering_rate_radps_ > 10.0 ||
        lc_pass_clear_m_ <= 0.0 || lc_pass_clear_m_ > lookahead_m_ ||
        ot_sector_min_len_m_ <= 0.0 || ot_sector_min_len_m_ > lookahead_m_ ||
        ot_bound_mindist_ < 0.0 || ot_bound_mindist_ > 2.0 ||
        ot_speed_scale_ <= 0.0 || ot_speed_scale_ > 1.0) {
        throw std::runtime_error(
            "Invalid/non-finite spliner safety parameter; planner heartbeat disabled");
    }
    if (!use_trailing_) {
        // 회피 실패 시 빈 경로는 downstream에서 전속 raceline 복귀를 뜻한다.
        // 고속 운용에서 fail-open 모드는 허용하지 않는다.
        RCLCPP_WARN(this->get_logger(),
            "spliner/use_trailing=false is unsafe and is overridden to true");
        use_trailing_ = true;
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

    if (bypass_speed_profile_) {
        RCLCPP_WARN(this->get_logger(),
            "spliner/bypass_speed_profile=true — 회피 경로 감속/속도캡을 "
            "전부 끄고 글로벌 레이스라인 속도를 그대로 씁니다 (임시 테스트 전용, "
            "물리 성립성 미검증)");
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
    std::lock_guard<std::mutex> lock(mutex_frenet_odom_);
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
        // 트랙 길이 = 마지막 s + 간격 (마지막 점과 첫 점 사이 구간 포함).
        // 마지막 s만 쓰면 모듈러스가 한 칸 짧아져 local_trajectory_node·
        // perception 브릿지의 s와 wrap 근처에서 어긋난다.
        track_len_ = gb_wpnts_.back().s_m + wpnt_dist_;
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

    const double max_curvature =
        std::tan(std::abs(vehicle_max_steering_rad_)) /
        std::max(vehicle_wheelbase_m_, 1e-3);
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
            } else {
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
            // SP 레인체인지 검증용: lookahead 내 전체 장애물
            auto all_ahead = ObstaclesInLookahead(obs_copy.obstacles);
            // 섹터 게이팅: 장애물(예측 장애물이면 따라잡는 지점)이 추월 섹터
            // 안일 때만 회피 시도. 섹터 안은 공격 모드(경계 마진 완화 + 속도
            // 부스트), 밖은 TRAILING으로 따라가다 섹터에서 승부한다.
            // 동적 상대를 이미 추월 중(|d| 큼)이면 섹터 이탈로 공격을 끊지
            // 않는다 — 끊기면 trailing d램프가 차를 상대 위치(raceline)로
            // 도로 끌어당겨 접촉한다 (섹터 조각 경계에서 진동 실측)
            bool mid_pass = std::abs(cur_d_) > side_switch_d_tresh_ &&
                            !closest.is_static;
            bool in_sector = !sector_gating_ || mid_pass ||
                             InOtSector(Mod(closest.s_center, track_len_));
            // 정적 장애물은 섹터(직선) 밖이어도 회피해야 한다 — trailing으로
            // 빠지면 v_target=0이라 그 뒤에 영원히 정지 (헤어핀 정적 장애물
            // 실측). 속도 부스트 없이 회피만 허용하고, 경계 마진은 차
            // 반폭(ot_bound)까지 완화 — 좁은 corridor 감속은 squeeze 로직 몫.
            bool try_evade = in_sector || closest.is_static;
            try_evade_now = try_evade;
            active_bound_mindist_ = try_evade ? ot_bound_mindist_ : spline_bound_mindist_;
            active_speed_scale_ = in_sector ? ot_speed_scale_ : 1.0;
            // 회피 시도 진입 알림 (try_evade 상승엣지)
            if (try_evade && !in_ot_sector_) {
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
            spline_fallback_active_ = used_spline;

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
            // 예전에는 `if (!evasion_valid && use_trailing_)` 뿐이라 충돌검사
            // 거부·벽 축소·"이미 장애물 구간 내부"·스팬 초과 같은 일시적
            // 사유에도 v_target=0 정지 궤적을 발행했다. 0807 실측으로 그
            // 정지 궤적(전 점 vx=0)이 MAP 컨트롤러의 조향을 0으로 만들어
            // (control_node.cpp:1057) 코스아웃의 직접 원인이 됐다.
            if (!evasion_valid && use_trailing_) {
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
                static_stop_active_ = used_trailing && closest.is_static;
            } else {
                static_stop_active_ = false;
            }

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
    in_ot_sector_ = try_evade_now;

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
    // evasion_dist_min 이상 확보되면 회피를 시도한다 — 고정 간격을 못
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
        RequiredBoundaryMargin(active_bound_mindist_);
    // 설정값이 물리적 하한보다 작으면 검증기가 어차피 거부하므로 하한으로 올린다
    // (사전 필터만 통과시키고 매번 Tier3로 떨어지는 것을 막는다).
    double min_space = std::max(evasion_side_space_min_, passable_width);

    double apex_left = ApexFor("left", obs);
    double apex_right = ApexFor("right", obs);

    if (right_gap > min_space && left_gap < min_space) {
        return {"right", apex_right};
    }
    if (left_gap > min_space && right_gap < min_space) {
        return {"left", apex_left};
    }
    // 양쪽 다 가능(또는 불가능)하면 raceline에서 덜 벗어나는 쪽 선택
    if (std::abs(apex_left) <= std::abs(apex_right)) {
        return {"left", apex_left};
    }
    return {"right", apex_right};
}

double SplinerNode::ApexFor(
    const std::string &side,
    const f1tenth_planning_msgs::msg::Obstacle &obs) const {
    const double boundary_margin =
        RequiredBoundaryMargin(active_bound_mindist_);
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
    const double dist_min = std::max(evasion_dist_min_, need);
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

    // 사이드: MoreSpace(빈 공간)가 우선. 단 이미 회피 중(|d|>thresh)이면
    // 히스테리시스로 현재 사이드를 유지한다 — mid-evasion 사이드 뒤집힘은
    // 장애물 사이로 끌려드는 충돌 원인 (DoSpline과 동일 정책).
    std::string more_side = MoreSpace(closest_obs).first;
    std::string first = more_side;
    if (SideKeepActive()) {
        first = last_ot_side_;
    }
    std::string second = (first == "left") ? "right" : "left";

    // 후보 레인: 우선 사이드 → 반대 사이드. 목표 오프셋 d_hold = ApexFor
    // (장애물 가장자리 + evasion_dist, 벽 클램프). 벽 클램프로 evasion_dist_min
    // 조차 못 챙기면 그 레인은 막힌 것 → 스킵. 첫 성공 레인을 반환한다.
    for (const std::string &side : {first, second}) {
        double d_hold = ApexFor(side, closest_obs);
        const double need_clear = std::max(
            evasion_dist_min_, RequiredObstacleClearance(closest_obs));
        bool clears = (side == "left")
            ? d_hold >= closest_obs.d_left + need_clear
            : d_hold <= closest_obs.d_right - need_clear;
        if (!clears) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "LaneSwitch %s 레인 불가: d_hold=%.2f (필요 %.2f, 장애물 d[%.2f,%.2f])",
                side.c_str(), d_hold,
                (side == "left") ? closest_obs.d_left + need_clear
                                 : closest_obs.d_right - need_clear,
                closest_obs.d_right, closest_obs.d_left);
            continue;
        }
        if (BuildLaneSwitch(d_hold, closest_obs, all_ahead,
                            out_wpnts, out_mrks)) {
            last_ot_side_ = side;
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
    if (alongside && std::abs(cur_d_) < evasion_dist_min_) {
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
    // 이 구간 레이스라인 곡률 — 조향 기하 하한에 더해진다 (BlendGeomFloor).
    const double kappa_in = BlendKappaMax(
        Mod(cur_s_, track_len_), Mod(cur_s_ + hold_end, track_len_));
    const double kappa_out = BlendKappaMax(
        Mod(cur_s_ + hold_end, track_len_),
        Mod(cur_s_ + hold_end + lane_blend_out_m_, track_len_));

    // 회피에 **실제로 필요한 최소 오프셋**. 목표 레인 d_hold까지 다 옮기는
    // 것과 "충돌을 벗어나는 것"은 다르다 — 현재 d가 이미 장애물 밖이면
    // 필요 횡이동은 0이다. 예전에는 d_hold - cur_d_ 를 그대로 진폭으로 써서
    // 추종 잔차 13cm를 못 지운다는 이유로 성립하는 회피를 통째로 버렸다
    // (0807 run_164819 t=4.815: 필요 0.13m / 하한 0.92m / 가용 0.78m -> 거부.
    //  실제로는 자차가 이미 d=+0.13 이라 횡이동 0으로 8cm 여유 통과 가능).
    const double clear_margin = std::max(
        evasion_dist_min_, RequiredObstacleClearance(closest_obs));
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
            }
        }
        if (!(cap > 0.0)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "LaneSwitch infeasible: %.2fm 횡이동에 기하 하한 %.2fm 필요, "
                "가용 %.2fm (v=%.1f, kappa=%.2f) — 어떤 속도로도 불가",
                d_hold - cur_d_, BlendGeomFloor(d_hold - cur_d_, kappa_in),
                available, cur_vs_, kappa_in);
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
    // 아래에서 벽에 맞춰 줄일 수 있다 (복귀 블렌드 단축)
    double blend_out = std::max(
        lane_blend_out_m_,
        BlendDist(d_hold, PlanSpeed(), a_lat, kappa_out));
    double blend_in;
    if (alongside) {
        // 이미 장애물 옆 — 남은 거리가 없으므로 잔차 보정에 필요한 만큼만.
        blend_in = std::max(1.0, need_in);
    } else {
        // 속도 캡을 씌웠으므로 need_in <= hold_start + wpnt_dist_가 보장된다.
        // 필요한 만큼만 쓰고(여유 20%) 시작을 늦춘다 — 남은 거리 전부를
        // 진입에 쓰면 상류의 좁은 구간을 오프셋 상태로 통과하게 된다.
        blend_in = std::clamp(need_in * 1.2, need_in,
                              std::max(std::min(hold_start, blend_dist_max_m_),
                                       need_in));
    }
    // 진입 블렌드가 hold 구간을 넘어가면 d 프로파일이 끊긴다.
    blend_in = std::min(blend_in, std::max(hold_end, wpnt_dist_));
    double total = hold_end + blend_out;
    if (!PlanSpanAllowed(total)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "LaneSwitch 거부: 계획 스팬 %.1fm (한 바퀴 초과)", total);
        return false;
    }

    int n = static_cast<int>(total / wpnt_dist_);
    if (n < 8) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "LaneSwitch 거부: 경로 %d점 (최소 8)", n);
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
        const double margin = RequiredBoundaryMargin(active_bound_mindist_);
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
    // 조향 기하 하한(geom_out)이다 — 그보다 길게 필요한 부분은 그 구간에서
    // 감속하면 만족시킬 수 있고, 실제로 아래에서 squeeze 감속을 건다.
    // 복귀를 짧게 못 하면 좁아지는 코너에 걸려 회피 자체가 사라진다.
    const double need_out = BlendDist(d_hold, PlanSpeed(), -1.0, kappa_out);
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
        return false;
    }
    const bool lane_squeezed = fit < 1.0 - 1e-6;
    if (lane_squeezed) {
        for (auto &d : lane_d) {
            d *= fit;
        }
        // 축소된 오프셋으로도 장애물을 비켜 가는가 — 아니면 이 레인은 무의미
        const double d_fit = d_hold * fit;
        const double need_clear2 = std::max(
            evasion_dist_min_, RequiredObstacleClearance(closest_obs));
        const bool still_clears = (d_hold > 0.0)
            ? d_fit >= closest_obs.d_left + need_clear2
            : d_fit <= closest_obs.d_right - need_clear2;
        if (!still_clears) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "LaneSwitch 거부: 벽 때문에 오프셋 %.2f -> %.2f로 축소되어 "
                "장애물 간격 부족 (장애물 d[%.2f,%.2f])",
                d_hold, d_fit, closest_obs.d_right, closest_obs.d_left);
            return false;
        }
    }

    out_wpnts.wpnts.reserve(n);
    std::vector<std::pair<double, double>> xy(n);
    // 복귀를 그립 예산보다 짧게 줄였으면 그 구간 속도를 낮춰 물리를 맞춘다
    // (6*A*v²/L² <= blend_lat_acc_max  ->  v <= L*sqrt(a/(6A))).
    double return_speed_cap = -1.0;
    if (return_shortened && d_hold > 1e-6) {
        return_speed_cap = blend_out *
            std::sqrt(blend_lat_acc_max_ / (6.0 * std::abs(d_hold)));
    }
    const bool wpnt_d_clamped = lane_squeezed;
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
        double vx = std::min(gb_wpnts_[gi].vx_mps, gb_vmax_);
        if (!bypass_speed_profile_) {
            vx = std::min(gb_wpnts_[gi].vx_mps * active_speed_scale_, gb_vmax_);
            if (v_floor > 0.0) {
                vx = std::clamp(vx, std::min(v_floor, gb_vmax_), gb_vmax_);
            }
            if (return_speed_cap > 0.0 && t >= hold_end - blend_in) {
                // 복귀(및 그 직전) 구간만 캡 — 접근 구간 속도는 유지한다
                vx = std::min(vx, return_speed_cap / std::max(cmd_speed_scale_, 0.05));
            }
            // 진입 속도 캡: 자차 위치부터 진입 블렌드가 끝나는 지점까지.
            // 여기서 미리 낮춰야 차가 블렌드 시작 시점에 그 속도에 도달해 있다.
            // (이 캡이 물리적으로 도달 불가능하면 아래 ApplySpeedProfile의
            //  require_current_feasible이 잡아 상위 폴백으로 넘긴다.)
            if (std::isfinite(entry_speed_cap) && t <= blend_start + blend_in) {
                vx = std::min(vx, entry_speed_cap / std::max(cmd_speed_scale_, 0.05));
            }
        }
        wpnt.vx_mps = vx;
        wpnt.ax_mps2 = gb_wpnts_[gi].ax_mps2;
        out_wpnts.wpnts.push_back(wpnt);
    }

    // 벽에 눌린(클램프된) 구간이 있으면 squeeze와 같은 근거로 감속한다 —
    // 좁아진 corridor를 전속으로 지나면 추적오차가 벽에 닿는다.
    if (wpnt_d_clamped && !bypass_speed_profile_) {
        for (auto &w : out_wpnts.wpnts) {
            w.vx_mps *= squeeze_speed_scale_;
        }
    }

    // 오프셋 경로의 실제 곡률로 속도 상한을 다시 잡고 역방향 감속 패스를
    // 건다. 직선(kappa~0)에서는 raceline 속도가 그대로 유지되고, 코너 진입
    // 전에서만 크게 감속된다.
    if (!ApplySpeedProfile(out_wpnts.wpnts)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "LaneSwitch 거부: 속도 프로파일 불성립 (기하곡률>조향한계 또는 감속 불가, v=%.1f)",
            cur_vs_);
        out_wpnts.wpnts.clear();
        return false;
    }

    // 목표 장애물도 포함해 실제 sampled path와 시간축 footprint를 검증한다.
    double blocking_s = -1.0;
    if (!CheckPathCollision(out_wpnts.wpnts, all_ahead, &blocking_s)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "LaneSwitch 거부: 경로가 장애물과 충돌 (blocking s=%.2f)", blocking_s);
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

    // apex 이후 ~2m 곡률 합으로 코너 바깥쪽 판단
    double kappa_sum = 0.0;
    int apex_idx = static_cast<int>(s_apex / wpnt_dist_) % gb_size;
    for (int i = 0; i < 20; ++i) {
        kappa_sum += gb_wpnts_[(apex_idx + i) % gb_size].kappa_radpm;
    }
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
    if (SideKeepActive() && more_space != last_ot_side_) {
        desired = last_ot_side_;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "Keep OT side %s (proposed %s, mid-evasion d=%.2f)",
            desired.c_str(), more_space.c_str(), cur_d_);
    }
    if (desired != more_space) {
        const double kept_apex = ApexFor(desired, closest_obs);
        const bool kept_ok = (desired == "left")
            ? kept_apex >= closest_obs.d_left + evasion_dist_min_
            : kept_apex <= closest_obs.d_right - evasion_dist_min_;
        if (kept_ok) {
            more_space = desired;
            d_apex = kept_apex;
        } else {
            // 유지 사이드가 막혔다 — 회피를 버리지 말고 원래 제안(넓은 쪽)으로
            // 되돌린다. 제안 쪽도 최소 간격이 안 나오면 그때만 포기한다.
            const bool proposed_ok = (more_space == "left")
                ? d_apex >= closest_obs.d_left + evasion_dist_min_
                : d_apex <= closest_obs.d_right - evasion_dist_min_;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "OT side %s infeasible (d_apex=%.2f) -> %s 폴백 (%s)",
                desired.c_str(), kept_apex, more_space.c_str(),
                proposed_ok ? "성립" : "양쪽 불가");
            if (!proposed_ok) {
                pitwall::Event("사이드 유지 불가 (" + desired + ")");
                return false;
            }
            pitwall::Event("사이드 전환 (" + desired + " -> " + more_space + ")");
            last_ot_side_ = more_space;
        }
    }

    // squeeze 감지/감속 배율은 d_apex가 최종 확정된 뒤(최소오프셋 완화 포함)에
    // 계산한다 — 완화로 간격이 더 깎이면 그만큼 더 감속해야 한다. 아래
    // 진입 속도 캡 블록이 d_apex를 낮출 수 있으므로 여기서는 선언만 한다.
    bool squeezed = false;
    double squeeze_scale_eff = 1.0;
    auto update_squeeze = [&]() {
        const double target_apex = (more_space == "left")
            ? std::max(closest_obs.d_left + evasion_dist_, 0.0)
            : std::min(closest_obs.d_right - evasion_dist_, 0.0);
        squeezed = std::abs(d_apex) < std::abs(target_apex) - 1e-3;
        // 감속은 실제로 깎인 간격에 비례 (이분법이면 균등배분 apex 특성상 거의
        // 모든 정적 회피가 squeeze로 분류돼 과도하게 감속한다):
        // 간격이 evasion_dist면 감속 없음, evasion_dist_min까지 깎이면 최대 감속
        const double clearance = (more_space == "left")
            ? d_apex - closest_obs.d_left
            : closest_obs.d_right - d_apex;
        const double t_sq = std::clamp(
            (clearance - evasion_dist_min_) /
                std::max(evasion_dist_ - evasion_dist_min_, 0.01), 0.0, 1.0);
        squeeze_scale_eff =
            squeeze_speed_scale_ + t_sq * (1.0 - squeeze_speed_scale_);
    };
    update_squeeze();

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
    // 이 구간 레이스라인 곡률 — 조향 기하 하한에 더해진다 (BlendGeomFloor).
    const double kappa_sp = BlendKappaMax(Mod(cur_s_, track_len_),
                                          Mod(s_back, track_len_));
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
        const double clear_margin = std::max(
            evasion_dist_min_, RequiredObstacleClearance(closest_obs));
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
                update_squeeze();   // 간격이 깎였으니 감속 배율 재계산
            }
        }
        if (!(cap > 0.0)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "Free spline infeasible: %.2fm 횡이동에 기하 하한 %.2fm 필요, "
                "가용 %.2fm (kappa=%.2f) — 어떤 속도로도 불가",
                d_apex - cur_d_, BlendGeomFloor(d_apex - cur_d_, kappa_sp),
                entry_avail_pre, kappa_sp);
            free_spline_geom_infeasible_ = true;
            return false;
        }
        if (cap < plan_v) {
            entry_speed_cap = std::max(cap, plan_speed_floor_);
            plan_v = std::min(plan_v, entry_speed_cap);
        }
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
    // 0.8 여유는 자연 3차 스플라인의 실제 피크가 smoothstep 근사보다
    // 큰 것을 흡수한다.
    const double knot_gap = std::abs(pre_apex_dists_.back());
    double geom_scale = 1.0;
    if (knot_gap > 0.05) {
        // BlendGeomFloor와 같은 식(트랙 곡률 포함)을 쓴다 — 예전에는 여기만
        // kappa_track=0 가정이라 헤어핀에서 절점 스케일이 과소평가됐다.
        geom_scale = BlendGeomFloor(d_apex, kappa_sp) / knot_gap;
    }
    double speed_scale = std::clamp(
        std::max({1.0 + plan_v / std::max(gb_vmax_, 0.1), need_scale,
                  geom_scale}), 1.0, 3.0);
    // squeeze 회피는 진입도 2배 앞에서 시작 — 감속 구간(gym 제동 ~ v/1s)을
    // 확보해 corridor 진입 전에 squeeze 속도까지 떨어지게 한다.
    // 진입이 짧으면 전속으로 들이닥쳐 추적오차로 장애물을 스친다.
    double squeeze_stretch = squeezed ? 2.0 : 1.0;
    std::vector<double> knot_s, knot_d;
    for (double dst : pre_apex_dists_) {
        double scaled = dst * speed_scale * squeeze_stretch;
        // 코너 바깥쪽으로 추월할 때는 스플라인을 더 완만하게
        if (outside == more_space) {
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
        double scaled = dst * speed_scale * squeeze_stretch;
        if (outside == more_space) {
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
        const bool already_clear = (more_space == "left")
            ? cur_d_ >= closest_obs.d_left + evasion_dist_min_
            : cur_d_ <= closest_obs.d_right - evasion_dist_min_;
        if (!already_clear) {
            // 장애물 구간 안에서 새 횡전환을 시작하면 sampled path가 대상
            // 내부를 통과한다. 감속 명령으로 이 기하 오류를 숨기지 않는다.
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "FreeSpline 거부: 이미 장애물 구간 내부 (cur_d=%.2f, %s 필요)",
                cur_d_, more_space.c_str());
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
    bool squeezed_now = squeezed;
    for (size_t i = 0; i < evasion_s.size(); ++i) {
        double s_wrapped = Mod(evasion_s[i], track_len_);
        int gb_idx = static_cast<int>(s_wrapped / wpnt_dist_) % gb_size;

        // raceline에서 실제로 벗어난 구간만 경계 거리 검사.
        // 허용치를 넘쳐도 바로 포기하지 않고 그 지점만 벽 허용치로 눌러본다 —
        // 눌렀을 때 장애물 구간에서 evasion_dist_min을 침범할 때만 포기.
        // (벽 폭은 s 따라 변하므로 한 점 넘쳤다고 전체를 버리면 회피포기 남발)
        if (std::abs(evasion_d[i]) > spline_resolution_) {
            bool in_obs_span = (evasion_s[i] > s_front - 0.3) &&
                               (evasion_s[i] < s_back + 0.3);
            // 공격 마진(0.1)은 장애물 corridor 구간에서만 — 진입/복귀 구간이
            // 벽 허용치에 계속 붙어 다니면 추적오차로 벽을 스친다 (실측)
            double bound = RequiredBoundaryMargin(
                in_obs_span ? active_bound_mindist_ : spline_bound_mindist_);
            double d_min = -std::abs(gb_wpnts_[gb_idx].d_right) + bound;
            double d_max = std::abs(gb_wpnts_[gb_idx].d_left) - bound;
            if (d_min > d_max) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                    "FreeSpline 거부: s=%.2f 통과폭 음수 (d_min %.2f > d_max %.2f)",
                    s_wrapped, d_min, d_max);
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
                const double need_clear = std::max(
                    evasion_dist_min_, RequiredObstacleClearance(closest_obs));
                bool cuts_obstacle = in_obs_body &&
                    ((more_space == "left")
                         ? clamped < closest_obs.d_left + need_clear
                         : clamped > closest_obs.d_right - need_clear);
                if (cuts_obstacle) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                        "Evasion trajectory too close to TRACKBOUNDS, aborting evasion");
                    boundary_abort_now_ = true;
                    out_wpnts.wpnts.clear();
                    return false;
                }
                evasion_d[i] = clamped;
                squeezed_now = true;  // 벽에 눌린 구간부터는 감속
            }
        }
        if (!WithinTrackBoundary(
                gb_idx, evasion_d[i],
                ((evasion_s[i] > s_front - 0.3) &&
                 (evasion_s[i] < s_back + 0.3))
                    ? active_bound_mindist_ : spline_bound_mindist_)) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "FreeSpline 거부: 경계 이탈 s=%.2f d=%.2f (벽 L%.2f/R%.2f)",
                s_wrapped, evasion_d[i], gb_wpnts_[gb_idx].d_left,
                gb_wpnts_[gb_idx].d_right);
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
        // 코너 안쪽 통과 시 감속 + 추월 섹터 내 공격 부스트 (raceline 최고속 캡).
        // squeeze(벽 마진으로 축소된 회피) 감속은 횡이동 직전(brake_dist)부터만 —
        // 스플라인이 장애물 한참 앞에서 시작하므로 전 구간 감속하면 접근이
        // 느려져 상대 꽁무니만 따라간다. 접근 구간은 기존 속도 유지.
        if (bypass_speed_profile_) {
            // 임시 테스트: 위 감속 근거를 전부 무시하고 글로벌 속도 그대로
            wpnt.vx_mps = std::min(gb_wpnts_[gb_idx].vx_mps, gb_vmax_);
        } else {
            wpnt.vx_mps = (outside == more_space) ?
                gb_wpnts_[gb_idx].vx_mps
                : gb_wpnts_[gb_idx].vx_mps * inside_speed_scale_;
            bool in_brake_zone = evasion_s[i] > s_front - squeeze_brake_dist_m_;
            double sscale = (squeezed_now && in_brake_zone)
                ? std::min(active_speed_scale_, squeeze_scale_eff)
                : active_speed_scale_;
            wpnt.vx_mps = std::min(wpnt.vx_mps * sscale, gb_vmax_);
            // 동적 상대 추월 중에는 상대보다 확실히 빠르게 (squeeze 감속이 상대
            // 속도 밑으로 내려가면 옆에서 나란히 달리다 추월이 영원히 안 끝난다)
            if (!closest_obs.is_static) {
                double v_floor = std::max(closest_obs.vs, 0.0) /
                    std::max(cmd_speed_scale_, 0.05) * dyn_pass_speed_ratio_;
                wpnt.vx_mps = std::clamp(
                    wpnt.vx_mps, std::min(v_floor, gb_vmax_), gb_vmax_);
            }
            // 진입 속도 캡 — apex 도달 전까지. 정적 장애물에서 "지금 속도로는
            // 안 되니 포기"(-> 정지) 대신 "되는 속도로 간다"를 실현하는 부분.
            // 동적 상대는 위 v_floor가 뒤에 적용되지 않도록 순서상 여기가 마지막.
            if (std::isfinite(entry_speed_cap) && closest_obs.is_static &&
                evasion_s[i] <= s_front) {
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
        out_wpnts.wpnts.clear();
        return false;
    }

    // free-form도 목표/2차 장애물을 모두 검사한다. 이 검사가 없으면 고정
    // 레인을 막아 Tier 2로 보낸 두 번째 장애물을 그대로 관통했다.
    double blocking_s = -1.0;
    if (!CheckPathCollision(out_wpnts.wpnts, all_ahead, &blocking_s)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "FreeSpline 거부: 경로가 장애물과 충돌 (blocking s=%.2f)", blocking_s);
        out_wpnts.wpnts.clear();
        return false;
    }

    // 인접 점으로 회피 궤적의 실제 헤딩 계산
    for (size_t i = 0; i + 1 < out_wpnts.wpnts.size(); ++i) {
        out_wpnts.wpnts[i].psi_rad = std::atan2(
            xy[i + 1].second - xy[i].second, xy[i + 1].first - xy[i].first);
    }

    last_ot_side_ = more_space;

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
    // 장애물 쪽으로 끌려간다 — 현재 d에서 raceline으로 되돌아오는 램프.
    // 길이는 고정 3m이 아니라 그립 예산으로: 6 m/s에서 d=0.4를 3m에 되돌리면
    // 1.0g를 요구한다.
    const double d_ramp_len = std::max(1.0, BlendDist(cur_d_, cur_vs_));
    const double planned_span = std::min(
        span + extend, std::max(track_len_ - wpnt_dist_, wpnt_dist_));
    int n = std::max(static_cast<int>(planned_span / wpnt_dist_), 2);
    bool hold_current_d = emergency_stop;
    if (!hold_current_d && std::abs(cur_d_) > 0.02) {
        for (int i = 0; i < n; ++i) {
            const double dist = i * wpnt_dist_;
            const double s = Mod(cur_s_ + dist, track_len_);
            const int gi = static_cast<int>(s / wpnt_dist_) % gb_size;
            const double d = (dist < d_ramp_len)
                ? cur_d_ * (1.0 - dist / d_ramp_len) : 0.0;
            if (!WithinTrackBoundary(gi, d, spline_bound_mindist_)) {
                // 안전한 raceline 복귀 기하가 없으면 빈 경로 대신 현재 d에서
                // 정지한다. 빈 경로는 곧 전속 global path이므로 금지.
                hold_current_d = true;
                v_target_prof_actual = 0.0;
                break;
            }
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
        double d = hold_current_d
            ? cur_d_
            : ((std::abs(cur_d_) > 0.05 && dist < d_ramp_len)
                ? cur_d_ * (1.0 - dist / d_ramp_len) : 0.0);
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
        "Trailing fallback: obs s=%.1f gap=%.1fm v_target=%.1f d_ramp=%.1fm (%zu wpnts)",
        obs.s_center, span, v_target_actual, d_ramp_len, out_wpnts.wpnts.size());
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
        if (!WithinTrackBoundary(gi, d, active_bound_mindist_)) {
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
        if (bypass_speed_profile_) {
            // 임시 테스트: SP 라인 프로파일/부스트 대신 글로벌 속도 그대로
            wpnt.vx_mps = std::min(gb_wpnts_[gi].vx_mps, gb_vmax_);
        }
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
        if (bypass_speed_profile_) {
            // 임시 테스트: SP 라인이 더 느려도 글로벌 속도 그대로
            wpnt.vx_mps = std::min(gb_wpnts_[gi].vx_mps, gb_vmax_);
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

std::vector<f1tenth_planning_msgs::msg::Obstacle> SplinerNode::ClusterObstacles(
    std::vector<f1tenth_planning_msgs::msg::Obstacle> obs) const {
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
                    const double a_lat = BlendLatAcc(obs[i].s_end, obs[j].s_start);
                    const double kap = BlendKappaMax(obs[i].s_end, obs[j].s_start);
                    if (BlendSpeedCap(need_lat, s_gap, a_lat, kap) > 0.0) {
                        // 사이로 지나갈 기하가 성립한다 — 병합하지 않는다.
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

double SplinerNode::BlendGeomFloor(double amplitude, double kappa_track) const {
    // 그립 예산은 저속에서 사실상 무한대가 된다(a_lat/v²) — 하지만 조향각
    // 한계라는 순수 기하 제약은 속도와 무관하게 남는다. 피크 기하곡률
    // 6A/L²이 조향 한계를 넘으면 ApplySpeedProfile이 경로를 통째로 거부하고,
    // 저속/정지에서는 감속으로 살릴 수도 없다 (run_0806_155621 정지 라치).
    // 0.8 여유는 실제 샘플 곡률이 smoothstep 근사보다 큰 것을 흡수한다.
    //
    // 조향은 트랙 곡률과 기동 곡률을 합쳐서 낸다:
    //   6A/L² + |kappa_track| <= 0.8*kappa_steer_max
    // 예전 식은 kappa_track=0을 가정해 헤어핀에서 하한을 과소평가했다
    // (0807 run_164216: kappa 0.74, A 0.49 -> 참 하한 2.23~2.45m를 1.80m로).
    const double a = std::abs(amplitude);
    if (a < 1e-3) {
        return 0.0;
    }
    const double kappa_steer_max =
        std::tan(std::abs(vehicle_max_steering_rad_)) /
        std::max(vehicle_wheelbase_m_, 1e-3);
    if (kappa_steer_max <= 1e-3) {
        return 0.0;
    }
    // 0.8 여유는 "smoothstep 근사 6A/L²보다 실제 샘플 피크가 크다"를 흡수하는
    // 것이므로 **기동 항에만** 적용한다. 트랙 곡률은 CSV에서 온 정확한 값이라
    // 그대로 빼면 된다:
    //   (1/0.8)*6A/L² + |kappa_track| <= kappa_steer_max
    //   -> L >= sqrt(6A / (0.8*(kappa_steer_max - |kappa_track|)))
    // kappa_track=0이면 예전 식과 정확히 같다 (직선 구간 호환).
    // 트랙 곡률이 조향 능력을 거의 다 먹으면 하한이 발산한다 — 그 구간은
    // 회피가 성립하지 않는 게 맞지만, inf를 돌려주면 상위 로그가 읽을 수
    // 없어지므로 blend_dist_max_m_로 잘라 "가용보다 큼"으로 만든다.
    const double spare = 0.8 * std::max(
        kappa_steer_max - std::abs(kappa_track),
        kBlendGeomKappaFloorRatio * kappa_steer_max);
    return std::min(std::sqrt(6.0 * a / spare), blend_dist_max_m_);
}

bool SplinerNode::SideKeepActive() const {
    if (last_ot_side_.empty() || std::abs(cur_d_) <= side_switch_d_tresh_) {
        return false;
    }
    // d 부호가 유지 사이드와 같아야 "그 사이드로 회피 중"이다.
    // (d>0 = 왼쪽, d<0 = 오른쪽 — Wpnt.d_left/d_right와 같은 부호 규약)
    return (cur_d_ > 0.0) ? (last_ot_side_ == "left")
                          : (last_ot_side_ == "right");
}

double SplinerNode::BlendKappaMax(double s_from, double s_to) const {
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
    double worst = 0.0;
    for (int i = 0; i <= steps; ++i) {
        const auto &w = gb_wpnts_[(start_idx + i) % gb_size];
        if (std::isfinite(w.kappa_radpm)) {
            worst = std::max(worst, std::abs(w.kappa_radpm));
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

    // 임시 테스트: 속도 프로파일 전면 우회. 곡률 캡·조향률 캡·역방향 감속·
    // 순방향 가속 envelope·jerk shaping·현재상태 feasibility 거부를 전부
    // 건너뛰고 들어온 속도를 그대로 둔다. trailing이 앞에서 계산한 제동
    // 프로파일은 여기서 더 깎이지 않을 뿐 그대로 유지된다.
    if (bypass_speed_profile_) {
        (void)require_current_feasible;
        return true;
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
    const double steering_curvature_limit =
        std::tan(std::abs(vehicle_max_steering_rad_)) /
        std::max(vehicle_wheelbase_m_, 1e-3);
    if (path_lat_acc_max_ > 1e-3) {
        for (size_t i = 0; i < n; ++i) {
            const double k = std::abs(curvature[i]);
            if (!std::isfinite(k) || k > steering_curvature_limit + 1e-3) {
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

    // 현재 속도에서 계획 지연 후 jerk/decel 한계로 최대 제동했을 때도
    // 곡률 cap에 도달하지 못하면 회피 기하 자체를 거부한다.
    if (require_current_feasible) {
        double simulated_v = std::max(std::abs(cur_vs_), 0.0);
        double simulated_a = 0.0;
        double delay_distance = simulated_v * planning_delay_sec_;
        const double tolerance = 0.25 + 0.03 * simulated_v;
        for (size_t i = 0; i < n; ++i) {
            double segment = (i == 0) ? first_distance : ds[i - 1];
            const double delayed = std::min(segment, delay_distance);
            delay_distance -= delayed;
            segment -= delayed;
            if (segment > 1e-6) {
                const double dt =
                    segment / std::max(simulated_v, 0.25);
                const double decel = longitudinal_budget(
                    path_decel_max_, simulated_v,
                    std::abs(curvature[(i == 0) ? 0 : i - 1]));
                simulated_a = std::max(
                    -decel, simulated_a - path_jerk_max_ * dt);
                simulated_v = std::sqrt(std::max(
                    simulated_v * simulated_v +
                    2.0 * simulated_a * segment, 0.0));
            }
            if (simulated_v > speed[i] + tolerance) {
                return false;
            }
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
