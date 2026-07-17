#include "planning_pkg/spliner_node.hpp"

#include <algorithm>
#include <limits>

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
    this->declare_parameter("spliner/inside_speed_scale", 0.9);
    this->declare_parameter("spliner/side_switch_d_tresh", 0.25);
    this->declare_parameter("spliner/obs_timeout_sec", 0.5);
    this->declare_parameter("spliner/use_prediction", true);

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
    side_switch_d_tresh_ = this->get_parameter("spliner/side_switch_d_tresh").as_double();
    obs_timeout_sec_ = this->get_parameter("spliner/obs_timeout_sec").as_double();
    use_prediction_ = this->get_parameter("spliner/use_prediction").as_bool();

    RCLCPP_INFO(this->get_logger(),
        "spliner params: lookahead=%.1fm evasion_dist=%.2fm obs_traj_tresh=%.2fm",
        lookahead_m_, evasion_dist_, obs_traj_tresh_);

    // Subscriber init
    s_obstacles_ = this->create_subscription<ae_hyu_msgs::msg::ObstacleArray>(
        "/perception/obstacles", qos_profile,
        std::bind(&SplinerNode::CallbackObstacles, this, std::placeholders::_1));

    // Predictive Spliner: opponent_predictor가 계산한 "따라잡는 시점의
    // 충돌 구간" 장애물. 같은 id의 원시 관측을 대체한다.
    s_predicted_obstacles_ = this->create_subscription<ae_hyu_msgs::msg::ObstacleArray>(
        "/opponent_prediction/obstacles", qos_profile,
        std::bind(&SplinerNode::CallbackPredictedObstacles, this, std::placeholders::_1));

    s_frenet_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/car_state/frenet/odom", qos_profile,
        std::bind(&SplinerNode::CallbackFrenetOdom, this, std::placeholders::_1));

    s_global_waypoints_ = this->create_subscription<ae_hyu_msgs::msg::WpntArray>(
        "/global_waypoints", qos_profile,
        std::bind(&SplinerNode::CallbackGlobalWaypoints, this, std::placeholders::_1));

    // Publisher init
    p_evasion_wpnts_ = this->create_publisher<ae_hyu_msgs::msg::WpntArray>(
        "/planner/avoidance/otwpnts", qos_profile);
    p_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/planner/avoidance/markers", qos_profile);

    // Timer init
    t_run_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / loop_rate_hz_)),
        [this]() { this->Run(); });

    RCLCPP_INFO(this->get_logger(), "SplinerNode initialized successfully");
}

void SplinerNode::CallbackObstacles(const ae_hyu_msgs::msg::ObstacleArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_obstacles_);
    obstacles_ = *msg;
    obstacles_stamp_ = this->now();
    has_obstacles_ = true;
}

void SplinerNode::CallbackPredictedObstacles(const ae_hyu_msgs::msg::ObstacleArray::SharedPtr msg) {
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

void SplinerNode::CallbackGlobalWaypoints(const ae_hyu_msgs::msg::WpntArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_gb_wpnts_);
    gb_wpnts_ = msg->wpnts;
    if (gb_wpnts_.size() >= 2) {
        gb_max_s_ = gb_wpnts_.back().s_m;
        wpnt_dist_ = gb_wpnts_[1].s_m - gb_wpnts_[0].s_m;
        gb_vmax_ = 0.0;
        for (const auto &w : gb_wpnts_) {
            gb_vmax_ = std::max(gb_vmax_, w.vx_mps);
        }
        has_gb_wpnts_ = true;
    }
}

void SplinerNode::Run() {
    if (!has_gb_wpnts_ || !has_frenet_odom_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
                             "Waiting for global waypoints / frenet odom...");
        return;
    }

    ae_hyu_msgs::msg::ObstacleArray obs_copy;
    {
        std::lock_guard<std::mutex> lock(mutex_obstacles_);
        // perception이 멈추면 유령 장애물을 계속 피하지 않도록 stale 메시지 폐기
        if (has_obstacles_ &&
            (this->now() - obstacles_stamp_).seconds() < obs_timeout_sec_) {
            obs_copy = obstacles_;
        }
        // Predictive Spliner: 예측 장애물이 신선하면 같은 id의 원시 관측을
        // "따라잡는 시점의 충돌 구간"으로 대체 (등속 전파보다 정확)
        if (use_prediction_ &&
            (this->now() - predicted_stamp_).seconds() < obs_timeout_sec_) {
            for (const auto &pred : predicted_obstacles_.obstacles) {
                obs_copy.obstacles.erase(
                    std::remove_if(obs_copy.obstacles.begin(), obs_copy.obstacles.end(),
                                   [&pred](const auto &o) { return o.id == pred.id; }),
                    obs_copy.obstacles.end());
                obs_copy.obstacles.push_back(pred);
            }
        }
    }

    ae_hyu_msgs::msg::WpntArray evasion_wpnts;
    evasion_wpnts.header.stamp = this->now();
    evasion_wpnts.header.frame_id = "map";
    visualization_msgs::msg::MarkerArray mrks;

    bool evasion_valid = false;
    if (!obs_copy.obstacles.empty()) {
        std::lock_guard<std::mutex> lock(mutex_gb_wpnts_);
        auto close_obs = FilterObstacles(obs_copy.obstacles);
        if (!close_obs.empty()) {
            // wrap-around을 고려한 전방 최근접 장애물 선택
            auto closest = *std::min_element(
                close_obs.begin(), close_obs.end(),
                [this](const auto &a, const auto &b) {
                    return Mod(a.s_center - cur_s_, gb_max_s_) <
                           Mod(b.s_center - cur_s_, gb_max_s_);
                });
            evasion_valid = DoSpline(closest, evasion_wpnts, mrks);
        }
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
}

std::vector<ae_hyu_msgs::msg::Obstacle> SplinerNode::FilterObstacles(
    const std::vector<ae_hyu_msgs::msg::Obstacle> &obstacles) const {
    std::vector<ae_hyu_msgs::msg::Obstacle> close_obs;
    for (const auto &raw_obs : obstacles) {
        // raceline 근처의 장애물만 회피 대상
        if (std::abs(raw_obs.d_center) >= obs_traj_tresh_) {
            continue;
        }
        auto obs = PredictObstacleMovement(raw_obs);
        double dist_in_front = Mod(obs.s_center - cur_s_, gb_max_s_);
        if (dist_in_front < lookahead_m_) {
            close_obs.push_back(obs);
        }
    }
    return close_obs;
}

ae_hyu_msgs::msg::Obstacle SplinerNode::PredictObstacleMovement(
    ae_hyu_msgs::msg::Obstacle obs) const {
    // 동적 장애물을 고정 시간만큼 등속 전파 (ForzaETH "constant" 모드)
    if (obs.is_static || Mod(obs.s_center - cur_s_, gb_max_s_) >= lookahead_m_) {
        return obs;
    }
    double delta_s = fixed_pred_time_ * obs.vs;
    double delta_d = fixed_pred_time_ * obs.vd;

    obs.s_start = Mod(obs.s_start + delta_s, gb_max_s_);
    obs.s_center = Mod(obs.s_center + delta_s, gb_max_s_);
    obs.s_end = Mod(obs.s_end + delta_s, gb_max_s_);
    obs.d_left += delta_d;
    obs.d_center += delta_d;
    obs.d_right += delta_d;
    return obs;
}

std::pair<std::string, double> SplinerNode::MoreSpace(
    const ae_hyu_msgs::msg::Obstacle &obs, int gb_idx) const {
    // 트랙 좌/우 빈 공간과 회피에 필요한 최소 공간 비교
    double left_gap = std::abs(gb_wpnts_[gb_idx].d_left - obs.d_left);
    double right_gap = std::abs(gb_wpnts_[gb_idx].d_right + obs.d_right);
    double min_space = evasion_dist_ + spline_bound_mindist_;

    if (right_gap > min_space && left_gap < min_space) {
        // 오른쪽 회피인데 apex가 raceline 왼쪽이면 0으로 클램프
        double d_apex = std::min(obs.d_right - evasion_dist_, 0.0);
        return {"right", d_apex};
    }
    if (left_gap > min_space && right_gap < min_space) {
        double d_apex = std::max(obs.d_left + evasion_dist_, 0.0);
        return {"left", d_apex};
    }
    // 양쪽 다 가능(또는 불가능)하면 raceline에서 덜 벗어나는 쪽 선택
    double cand_left = obs.d_left + evasion_dist_;
    double cand_right = obs.d_right - evasion_dist_;
    if (std::abs(cand_left) <= std::abs(cand_right)) {
        return {"left", std::max(cand_left, 0.0)};
    }
    return {"right", std::min(cand_right, 0.0)};
}

bool SplinerNode::DoSpline(
    const ae_hyu_msgs::msg::Obstacle &closest_obs,
    ae_hyu_msgs::msg::WpntArray &out_wpnts,
    visualization_msgs::msg::MarkerArray &out_mrks) {

    int gb_size = static_cast<int>(gb_wpnts_.size());

    // apex: 장애물 중심 (s wrap 처리)
    double s_apex;
    if (closest_obs.s_end < closest_obs.s_start) {
        s_apex = (closest_obs.s_end + gb_max_s_ + closest_obs.s_start) / 2.0;
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

    auto [more_space, d_apex] = MoreSpace(closest_obs, apex_idx);

    // raceline에서 벗어나 있으면 사이드 스위치 금지 (ForzaETH danger_flag)
    if (std::abs(cur_d_) > side_switch_d_tresh_ &&
        !last_ot_side_.empty() && more_space != last_ot_side_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "Can't switch OT side (%s -> %s): not on raceline (d=%.2f)",
            last_ot_side_.c_str(), more_space.c_str(), cur_d_);
        pitwall::Event("사이드 스위치 거부 (" + last_ot_side_ + " -> " + more_space + ")");
        return false;
    }

    // 스플라인 절점: pre-apex 3점 + apex + post-apex 3점
    std::vector<double> spline_dists;
    spline_dists.insert(spline_dists.end(), pre_apex_dists_.begin(), pre_apex_dists_.end());
    spline_dists.push_back(0.0);
    spline_dists.insert(spline_dists.end(), post_apex_dists_.begin(), post_apex_dists_.end());

    double speed_scale = std::clamp(1.0 + cur_vs_ / std::max(gb_vmax_, 0.1), 1.0, 1.5);
    std::vector<double> knot_s, knot_d;
    knot_s.reserve(spline_dists.size());
    knot_d.reserve(spline_dists.size());
    for (double dst : spline_dists) {
        double scaled = dst * speed_scale;
        // 코너 바깥쪽으로 추월할 때는 스플라인을 더 완만하게
        if (outside == more_space) {
            scaled *= outside_spline_scale_;
        }
        knot_s.push_back(s_apex + scaled);
        knot_d.push_back(dst == 0.0 ? d_apex : 0.0);
    }

    // d(s) 자연 3차 스플라인을 0.1m 간격으로 샘플
    std::vector<double> evasion_s;
    for (double s = knot_s.front(); s < knot_s.back(); s += spline_resolution_) {
        evasion_s.push_back(s);
    }
    std::vector<double> evasion_d = NaturalCubicSpline(knot_s, knot_d, evasion_s);
    for (auto &d : evasion_d) {
        d = (d_apex < 0.0) ? std::clamp(d, d_apex, 0.0) : std::clamp(d, 0.0, d_apex);
    }

    // 궤적 생성 + 트랙 경계 안전 검사
    out_wpnts.wpnts.reserve(evasion_s.size());
    std::vector<std::pair<double, double>> xy(evasion_s.size());
    for (size_t i = 0; i < evasion_s.size(); ++i) {
        double s_wrapped = Mod(evasion_s[i], gb_max_s_);
        int gb_idx = static_cast<int>(s_wrapped / wpnt_dist_) % gb_size;

        // raceline에서 실제로 벗어난 구간만 경계 거리 검사
        if (std::abs(evasion_d[i]) > spline_resolution_) {
            double tb_dist = (more_space == "left") ?
                gb_wpnts_[gb_idx].d_left : gb_wpnts_[gb_idx].d_right;
            if (std::abs(evasion_d[i]) > std::abs(tb_dist) - spline_bound_mindist_) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                    "Evasion trajectory too close to TRACKBOUNDS, aborting evasion");
                pitwall::Event("회피 포기(트랙경계)");
                out_wpnts.wpnts.clear();
                return false;
            }
        }

        xy[i] = FrenetToCartesian(s_wrapped, evasion_d[i]);

        ae_hyu_msgs::msg::Wpnt wpnt;
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
        // 코너 안쪽 통과 시 감속
        wpnt.vx_mps = (outside == more_space) ?
            gb_wpnts_[gb_idx].vx_mps : gb_wpnts_[gb_idx].vx_mps * inside_speed_scale_;
        wpnt.ax_mps2 = gb_wpnts_[gb_idx].ax_mps2;
        out_wpnts.wpnts.push_back(wpnt);
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
        more_space.c_str(), d_apex, Mod(s_apex, gb_max_s_), out_wpnts.wpnts.size());
    pitwall::Log("evasion_d_apex", d_apex);
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
    std::vector<double> h(n - 1);
    for (size_t i = 0; i < n - 1; ++i) {
        h[i] = xs[i + 1] - xs[i];
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
