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
    this->declare_parameter("spliner/trailing_gap_m", 2.5);
    this->declare_parameter("spliner/trailing_decel", 2.5);
    // 컨트롤러가 wpnt 속도에 곱하는 스케일 (sim tracker 0.5, 실차 1.0).
    // 장애물 실측 속도를 wpnt 속도 단위로 환산할 때 필요 — 안 맞추면
    // 동적 상대 trailing 시 상대보다 느리게 따라가 격차가 벌어진다.
    this->declare_parameter("spliner/cmd_speed_scale", 1.0);
    // 이중장애물 재계획 / 안전 정지 (mina Task 2)
    this->declare_parameter("spliner/path_check_margin", 0.15);
    this->declare_parameter("spliner/cluster_gap_s_m", 8.0);
    this->declare_parameter("spliner/evasion_dist_min", 0.35);
    this->declare_parameter("spliner/squeeze_speed_scale", 0.35);
    this->declare_parameter("spliner/squeeze_brake_dist_m", 6.0);
    this->declare_parameter("spliner/trailing_engage_dist_m", 12.0);
    this->declare_parameter("spliner/dyn_pass_speed_ratio", 1.4);
    this->declare_parameter("spliner/cluster_gap_d_m", 0.8);
    this->declare_parameter("spliner/stop_margin_m", 1.0);
    this->declare_parameter("spliner/stop_ramp_dist_m", 2.5);
    this->declare_parameter("spliner/sector_gating", true);
    this->declare_parameter("spliner/ot_min_clearance", 0.95);
    this->declare_parameter("spliner/ot_max_kappa", 0.1);
    this->declare_parameter("spliner/sp_lane_change", true);
    this->declare_parameter("spliner/ot_sector_min_len_m", 8.0);
    this->declare_parameter("spliner/ot_bound_mindist", 0.25);
    this->declare_parameter("spliner/ot_speed_scale", 1.1);
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
    side_switch_d_tresh_ = this->get_parameter("spliner/side_switch_d_tresh").as_double();
    obs_timeout_sec_ = this->get_parameter("spliner/obs_timeout_sec").as_double();
    use_prediction_ = this->get_parameter("spliner/use_prediction").as_bool();
    trailing_gap_m_ = this->get_parameter("spliner/trailing_gap_m").as_double();
    trailing_decel_ = this->get_parameter("spliner/trailing_decel").as_double();
    cmd_speed_scale_ = this->get_parameter("spliner/cmd_speed_scale").as_double();
    path_check_margin_ = this->get_parameter("spliner/path_check_margin").as_double();
    cluster_gap_s_m_ = this->get_parameter("spliner/cluster_gap_s_m").as_double();
    evasion_dist_min_ = this->get_parameter("spliner/evasion_dist_min").as_double();
    squeeze_speed_scale_ = this->get_parameter("spliner/squeeze_speed_scale").as_double();
    squeeze_brake_dist_m_ = this->get_parameter("spliner/squeeze_brake_dist_m").as_double();
    trailing_engage_dist_m_ =
        this->get_parameter("spliner/trailing_engage_dist_m").as_double();
    dyn_pass_speed_ratio_ = this->get_parameter("spliner/dyn_pass_speed_ratio").as_double();
    cluster_gap_d_m_ = this->get_parameter("spliner/cluster_gap_d_m").as_double();
    stop_margin_m_ = this->get_parameter("spliner/stop_margin_m").as_double();
    stop_ramp_dist_m_ = this->get_parameter("spliner/stop_ramp_dist_m").as_double();
    sector_gating_ = this->get_parameter("spliner/sector_gating").as_bool();
    ot_min_clearance_ = this->get_parameter("spliner/ot_min_clearance").as_double();
    ot_max_kappa_ = this->get_parameter("spliner/ot_max_kappa").as_double();
    sp_lane_change_ = this->get_parameter("spliner/sp_lane_change").as_bool();
    ot_sector_min_len_m_ = this->get_parameter("spliner/ot_sector_min_len_m").as_double();
    ot_bound_mindist_ = this->get_parameter("spliner/ot_bound_mindist").as_double();
    ot_speed_scale_ = this->get_parameter("spliner/ot_speed_scale").as_double();
    ot_sectors_param_ = this->get_parameter("spliner/ot_sectors").as_double_array();
    active_bound_mindist_ = spline_bound_mindist_;

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

    // SP(최단거리) 추월 라인 — 동적 상대 레인체인지 추월용 (없으면 기존 방식)
    s_sp_waypoints_ = this->create_subscription<ae_hyu_msgs::msg::WpntArray>(
        "/global_waypoints/overtaking", qos_profile,
        std::bind(&SplinerNode::CallbackSpWaypoints, this, std::placeholders::_1));

    // Publisher init
    p_evasion_wpnts_ = this->create_publisher<ae_hyu_msgs::msg::WpntArray>(
        "/planner/avoidance/otwpnts", qos_profile);
    p_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/planner/avoidance/markers", qos_profile);
    p_sector_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/planner/ot_sectors/markers",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

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
        bool first = !has_gb_wpnts_;
        has_gb_wpnts_ = true;
        if (first) {
            ComputeOtSectors();
            PublishSectorMarkers();
            if (!sp_raw_.empty() && !has_sp_) {
                BuildSpMapping();
            }
        }
    }
}

void SplinerNode::CallbackSpWaypoints(const ae_hyu_msgs::msg::WpntArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_gb_wpnts_);
    if (msg->wpnts.empty() || has_sp_) {
        return;  // 첫 수신만 매핑 (라인은 정적)
    }
    sp_raw_ = msg->wpnts;
    if (has_gb_wpnts_) {
        BuildSpMapping();
    }
}

void SplinerNode::BuildSpMapping() {
    // SP 라인을 IQP frenet으로: gb 인덱스별 최근접 SP 점의 부호 있는 횡
    // 오프셋 d_sp와 속도. 두 라인은 같은 도구로 같은 방향으로 생성되므로
    // 인덱스가 함께 진행한다 — 롤링 윈도우 최근접 탐색.
    int n = static_cast<int>(gb_wpnts_.size());
    int m = static_cast<int>(sp_raw_.size());
    if (n < 2 || m < 2) {
        return;
    }
    sp_d_.assign(n, 0.0);
    sp_vx_.assign(n, 0.0);
    int j_hint = 0;
    {   // 첫 점은 전역 탐색
        double best = 1e18;
        for (int j = 0; j < m; ++j) {
            double dx = sp_raw_[j].x_m - gb_wpnts_[0].x_m;
            double dy = sp_raw_[j].y_m - gb_wpnts_[0].y_m;
            double dd = dx * dx + dy * dy;
            if (dd < best) { best = dd; j_hint = j; }
        }
    }
    for (int i = 0; i < n; ++i) {
        int best_j = j_hint;
        double best = 1e18;
        for (int k = -40; k <= 40; ++k) {   // ±4m 윈도우
            int j = ((j_hint + k) % m + m) % m;
            double dx = sp_raw_[j].x_m - gb_wpnts_[i].x_m;
            double dy = sp_raw_[j].y_m - gb_wpnts_[i].y_m;
            double dd = dx * dx + dy * dy;
            if (dd < best) { best = dd; best_j = j; }
        }
        j_hint = best_j;
        double dx = sp_raw_[best_j].x_m - gb_wpnts_[i].x_m;
        double dy = sp_raw_[best_j].y_m - gb_wpnts_[i].y_m;
        double psi = gb_wpnts_[i].psi_rad;
        sp_d_[i] = -dx * std::sin(psi) + dy * std::cos(psi);  // d>0 = 왼쪽
        sp_vx_[i] = sp_raw_[best_j].vx_mps;
    }
    has_sp_ = true;
    RCLCPP_INFO(this->get_logger(),
        "SP overtaking line mapped: %d pts, |d_sp| max %.2fm", n,
        std::abs(*std::max_element(sp_d_.begin(), sp_d_.end(),
            [](double a, double b) { return std::abs(a) < std::abs(b); })));
    pitwall::Event("SP 추월 라인 매핑 완료");
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
        ot_sectors_.emplace_back(0.0, gb_max_s_);
    } else {
        int run_begin = -1;
        for (int k = 0; k <= n; ++k) {
            int idx = (start + k) % n;
            bool f = (k < n) ? feasible[idx] : false;
            if (f && run_begin < 0) {
                run_begin = idx;
            } else if (!f && run_begin >= 0) {
                int run_end = (start + k - 1) % n;
                double len = Mod((run_end - run_begin) * wpnt_dist_, gb_max_s_);
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
        total += Mod(s1 - s0, gb_max_s_);
    }
    double pct = 100.0 * total / std::max(gb_max_s_, 0.1);
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
        double len = Mod(s1 - s0, gb_max_s_);
        if (len <= 0.0) len = gb_max_s_;
        for (double ds = 0.0; ds <= len; ds += 0.5) {
            auto [x, y] = FrenetToCartesian(Mod(s0 + ds, gb_max_s_), 0.0);
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

    ae_hyu_msgs::msg::ObstacleArray obs_copy;
    {
        std::lock_guard<std::mutex> lock(mutex_obstacles_);
        // perception이 멈추면 유령 장애물을 계속 피하지 않도록 stale 메시지 폐기
        if (has_obstacles_ &&
            (this->now() - obstacles_stamp_).seconds() < obs_timeout_sec_) {
            obs_copy = obstacles_;
        }
        // Predictive Spliner: 예측 "따라잡는 시점의 충돌 구간"을 원시 관측과
        // 함께 유지한다 (별도 id). 클러스터링이 상대의 현재 위치~따라잡는
        // 지점을 한 박스로 합쳐 그 전체를 크게 회피하게 된다.
        // 과거처럼 원시를 대체(erase)하면 경로를 상대의 '현재' 위치와 충돌검사
        // 하지 않아 복귀 지점에서 상대와 접촉한다 (실측 0.01m).
        if (use_prediction_ &&
            (this->now() - predicted_stamp_).seconds() < obs_timeout_sec_) {
            for (const auto &pred : predicted_obstacles_.obstacles) {
                auto p = pred;
                p.id = pred.id + 10000;
                // predictor는 "이미 전파됨" 의미로 is_static=true를 주지만,
                // 여기선 동적 상대의 미래 구간이다 — false로 되돌려야
                // SP 레인체인지/추월 부스트가 동작하고 경로 커밋(정적 전용)이
                // 움직이는 상대에 걸리지 않는다
                p.is_static = false;
                obs_copy.obstacles.push_back(p);
            }
        }
    }

    ae_hyu_msgs::msg::WpntArray evasion_wpnts;
    evasion_wpnts.header.stamp = this->now();
    evasion_wpnts.header.frame_id = "map";
    visualization_msgs::msg::MarkerArray mrks;

    bool evasion_valid = false;
    bool used_trailing = false;
    bool used_stop = false;
    {
        std::lock_guard<std::mutex> lock(mutex_gb_wpnts_);
        // 붙어 있는 장애물들을 먼저 하나의 박스로 병합 — 이후의 최근접 선택,
        // 회피, 경로 검증이 모두 클러스터 단위로 일관되게 동작한다
        obs_copy.obstacles = ClusterObstacles(obs_copy.obstacles);
        auto all_ahead_now = ObstaclesInLookahead(obs_copy.obstacles);

        // 진행 중인 정적 회피는 경로 끝까지 커밋 — 매 사이클 재계획(ego-anchor
        // 재생성)과 '장애물 지남' 판정으로 회피가 도중에 사라지면, 복귀 중인
        // 차(d 큼)가 raceline 전속 경로로 순간 전환돼 반대편 벽을 친다 (실측).
        // 새 장애물이 커밋 경로를 막으면 즉시 폐기하고 재계획한다.
        if (committed_until_s_ >= 0.0) {
            double span = Mod(committed_until_s_ - committed_start_s_, gb_max_s_);
            double pos = Mod(cur_s_ - committed_start_s_, gb_max_s_);
            if (pos < span &&
                CheckPathCollision(committed_wpnts_.wpnts, all_ahead_now,
                                   committed_id_, nullptr)) {
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
            ? std::vector<ae_hyu_msgs::msg::Obstacle>{}
            : FilterObstacles(obs_copy.obstacles);
        if (!close_obs.empty()) {
            // wrap-around을 고려한 전방 최근접 장애물 선택
            auto closest = *std::min_element(
                close_obs.begin(), close_obs.end(),
                [this](const auto &a, const auto &b) {
                    return Mod(a.s_end - cur_s_, gb_max_s_) <
                           Mod(b.s_end - cur_s_, gb_max_s_);
                });
            // 경로 검증은 raceline 근접 여부와 무관하게 lookahead 내 모든
            // 장애물 대상 (mina Task 2: 회피로 옆 차선에 있는 장애물도
            // 새 경로에 걸릴 수 있음)
            auto all_ahead = ObstaclesInLookahead(obs_copy.obstacles);
            double blocking_s = -1.0;
            // 섹터 게이팅: 장애물(예측 장애물이면 따라잡는 지점)이 추월 섹터
            // 안일 때만 회피 시도. 섹터 안은 공격 모드(경계 마진 완화 + 속도
            // 부스트), 밖은 TRAILING으로 따라가다 섹터에서 승부한다.
            // 동적 상대를 이미 추월 중(|d| 큼)이면 섹터 이탈로 공격을 끊지
            // 않는다 — 끊기면 trailing d램프가 차를 상대 위치(raceline)로
            // 도로 끌어당겨 접촉한다 (섹터 조각 경계에서 진동 실측)
            bool mid_pass = std::abs(cur_d_) > side_switch_d_tresh_ &&
                            !closest.is_static;
            bool in_sector = !sector_gating_ || mid_pass ||
                             InOtSector(Mod(closest.s_center, gb_max_s_));
            // 정적 장애물은 섹터(직선) 밖이어도 회피해야 한다 — trailing으로
            // 빠지면 v_target=0이라 그 뒤에 영원히 정지 (헤어핀 정적 장애물
            // 실측). 속도 부스트 없이 회피만 허용하고, 경계 마진은 차
            // 반폭(ot_bound)까지 완화 — 좁은 corridor 감속은 squeeze 로직 몫.
            bool try_evade = in_sector || closest.is_static;
            active_bound_mindist_ = try_evade ? ot_bound_mindist_ : spline_bound_mindist_;
            active_speed_scale_ = in_sector ? ot_speed_scale_ : 1.0;
            // SP 레인체인지: 동적 상대 전용, 섹터(직선) 여부와 무관하게 시도.
            // 직선에선 SP≈IQP라 상대와 겹쳐 검증에서 자연 탈락하고, 코너에서
            // 두 라인이 벌어질 때 인사이드 찌르기가 성립 — 섹터 게이팅의
            // 예외지만 SP 라인 자체가 검증된 레이싱 라인이라 안전하다.
            bool lc_done = false;
            if (sp_lane_change_ && has_sp_ && !closest.is_static) {
                lc_done = BuildSpLaneChange(closest, all_ahead,
                                            evasion_wpnts, mrks);
                if (lc_done && !lc_active_) {
                    pitwall::Event("SP 레인체인지 추월");
                }
            }
            lc_active_ = lc_done;
            if (lc_done) {
                evasion_valid = true;
            } else if (try_evade) {
                if (!in_ot_sector_) {
                    pitwall::Event(in_sector ? "추월 섹터 — 공격 회피 시도"
                                             : "코너 정적 장애물 — 회피 시도");
                }
                evasion_valid = DoSpline(closest, evasion_wpnts, mrks);
                // 이중장애물 재검증 (mina Task 2): 후보 경로가 두 번째 장애물에
                // 막히면 반대 사이드로 재계획, 그것도 막히면 회피 포기
                if (evasion_valid &&
                    !CheckPathCollision(evasion_wpnts.wpnts, all_ahead,
                                        closest.id, &blocking_s)) {
                    std::string other = (last_ot_side_ == "left") ? "right" : "left";
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000,
                        "Avoidance path blocked by 2nd obstacle (s=%.1f), "
                        "replanning on %s side", blocking_s, other.c_str());
                    pitwall::Event("2차 장애물 — 반대 사이드 재계획");
                    evasion_wpnts.wpnts.clear();
                    mrks.markers.clear();
                    evasion_valid = DoSpline(closest, evasion_wpnts, mrks, other);
                    if (evasion_valid &&
                        !CheckPathCollision(evasion_wpnts.wpnts, all_ahead,
                                            closest.id, &blocking_s)) {
                        evasion_valid = false;  // 좌·우 모두 불가
                    }
                }
            }
            in_ot_sector_ = try_evade;
            if (!evasion_valid) {
                // 회피 불가(트랙경계 abort / 사이드 스위치 거부 / 양쪽 차단) 폴백:
                // raceline을 유지한 채 장애물 뒤 trailing_gap에서 장애물
                // 속도(정적이면 0)까지 감속하는 궤적 발행 — ForzaETH 상태머신의
                // TRAILING 대응. 없으면 ego가 감속 없이 정면으로 돌진한다.
                evasion_wpnts.wpnts.clear();
                mrks.markers.clear();
                evasion_valid = BuildTrailing(closest, evasion_wpnts, mrks);
                used_trailing = evasion_valid;
                // trailing 경로(d=0)마저 "다른" 장애물에 막히면 그 앞
                // stop_margin에서 정지하는 안전 경로로 대체 (mina Task 2)
                if (evasion_valid &&
                    !CheckPathCollision(evasion_wpnts.wpnts, all_ahead,
                                        closest.id, &blocking_s)) {
                    evasion_wpnts.wpnts.clear();
                    mrks.markers.clear();
                    BuildStopWpnts(blocking_s, evasion_wpnts, mrks);
                    evasion_valid = !evasion_wpnts.wpnts.empty();
                    used_trailing = false;
                    used_stop = evasion_valid;
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000,
                        "Trailing path blocked by 2nd obstacle -> "
                        "STOP wpnts before s=%.1f", blocking_s);
                }
            }
            // 정적 장애물 회피 성공 -> 경로 끝까지 커밋 (동적/예측 상대는
            // 움직이므로 커밋하지 않고 기존처럼 매 사이클 갱신)
            if (evasion_valid && !used_trailing && !used_stop &&
                closest.is_static && !evasion_wpnts.wpnts.empty()) {
                committed_wpnts_ = evasion_wpnts;
                committed_mrks_ = mrks;
                committed_id_ = closest.id;
                committed_start_s_ = evasion_wpnts.wpnts.front().s_m;
                committed_until_s_ = evasion_wpnts.wpnts.back().s_m;
            }
        }
    }
    if (used_trailing && !trailing_active_) {
        pitwall::Event("TRAILING 폴백 (회피 불가 — 감속 추종)");
    }
    trailing_active_ = used_trailing;
    if (used_stop && !stop_active_) {
        pitwall::Event("이중 장애물 — 안전 정지");
    }
    stop_active_ = used_stop;

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
        // 장애물의 (d) 구간이 raceline corridor(±obs_traj_tresh)와 겹칠 때만
        // 회피 대상 (d_center 기준은 폭이 넓은 클러스터의 가장자리 침범을 놓침)
        if (raw_obs.d_right >= obs_traj_tresh_ ||
            raw_obs.d_left <= -obs_traj_tresh_) {
            continue;
        }
        auto obs = PredictObstacleMovement(raw_obs);
        // 판정 기준은 s_end: 몸통이 긴 클러스터는 중심이 ego 뒤로 가도
        // 꼬리를 다 지날 때까지 회피 대상 유지 (중심 기준이면 옆에 있는데
        // 회피가 풀려 raceline 복귀 경로가 장애물을 관통)
        double dist_in_front = Mod(obs.s_end - cur_s_, gb_max_s_);
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
    // 트랙 좌/우 빈 공간과 회피에 필요한 최소 공간 비교.
    // 목표 간격은 evasion_dist지만, 벽이 허용하는 만큼 apex를 줄여서라도
    // evasion_dist_min 이상 확보되면 회피를 시도한다 — 고정 간격을 못
    // 맞추면 바로 포기하던 것이 "회피포기 남발"의 원인 (특히 클러스터).
    double left_gap = std::abs(gb_wpnts_[gb_idx].d_left - obs.d_left);
    double right_gap = std::abs(gb_wpnts_[gb_idx].d_right + obs.d_right);
    double min_space = evasion_dist_min_ + active_bound_mindist_;

    double apex_left = ApexFor("left", obs, gb_idx);
    double apex_right = ApexFor("right", obs, gb_idx);

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
    const ae_hyu_msgs::msg::Obstacle &obs, int gb_idx) const {
    if (side == "left") {
        double wall = std::abs(gb_wpnts_[gb_idx].d_left);
        double corridor = std::abs(gb_wpnts_[gb_idx].d_left - obs.d_left);
        double surplus =
            std::max(corridor - evasion_dist_min_ - active_bound_mindist_, 0.0);
        // corridor 여유를 장애물/벽에 반씩 배분한 지점 (넓으면 목표값이 더 작음)
        double balanced = obs.d_left + evasion_dist_min_ + surplus / 2.0;
        double target = obs.d_left + evasion_dist_;
        return std::clamp(std::min(target, balanced), 0.0,
                          std::max(wall - active_bound_mindist_, 0.0));
    }
    double wall = std::abs(gb_wpnts_[gb_idx].d_right);
    double corridor = std::abs(gb_wpnts_[gb_idx].d_right + obs.d_right);
    double surplus =
        std::max(corridor - evasion_dist_min_ - active_bound_mindist_, 0.0);
    double balanced = obs.d_right - evasion_dist_min_ - surplus / 2.0;
    double target = obs.d_right - evasion_dist_;
    return std::clamp(std::max(target, balanced),
                      -std::max(wall - active_bound_mindist_, 0.0), 0.0);
}

bool SplinerNode::DoSpline(
    const ae_hyu_msgs::msg::Obstacle &closest_obs,
    ae_hyu_msgs::msg::WpntArray &out_wpnts,
    visualization_msgs::msg::MarkerArray &out_mrks,
    const std::string &forced_side) {

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

    // 사이드 결정 우선순위:
    //  1) forced_side (이중장애물 재계획) — 무조건 그 사이드
    //  2) 이미 회피 중(|d| > side_switch_d_tresh)인데 MoreSpace가 반대를
    //     제안하면 "현재 사이드 유지"로 재계산 (ForzaETH danger_flag는
    //     거부->trailing 폴백이었는데, 회피 중 d=0 trailing 경로로의 붕괴가
    //     장애물 사이로 끌려 들어가는 충돌 원인이라 유지-재계산으로 변경)
    std::string desired = more_space;
    if (!forced_side.empty()) {
        desired = forced_side;
    } else if (std::abs(cur_d_) > side_switch_d_tresh_ &&
               !last_ot_side_.empty() && more_space != last_ot_side_) {
        desired = last_ot_side_;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
            "Keep OT side %s (proposed %s, mid-evasion d=%.2f)",
            desired.c_str(), more_space.c_str(), cur_d_);
    }
    if (desired != more_space) {
        more_space = desired;
        d_apex = ApexFor(desired, closest_obs, apex_idx);
        // 그 사이드로는 최소 간격이 안 나올 때만 진짜 포기
        bool ok = (desired == "left")
            ? d_apex >= closest_obs.d_left + evasion_dist_min_
            : d_apex <= closest_obs.d_right - evasion_dist_min_;
        if (!ok) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                "OT side %s infeasible (d_apex=%.2f)", desired.c_str(), d_apex);
            pitwall::Event("사이드 유지 불가 (" + desired + ")");
            return false;
        }
    }

    // squeeze 감지: 벽 클램프 때문에 목표 evasion_dist를 다 못 챙긴 회피.
    // 이 경우 좁은 corridor 통과이므로 속도 부스트 대신 감속한다
    // (추적오차는 속도에 비례 — 전속 squeeze가 벽 접촉 원인이었음)
    double target_apex = (more_space == "left")
        ? std::max(closest_obs.d_left + evasion_dist_, 0.0)
        : std::min(closest_obs.d_right - evasion_dist_, 0.0);
    bool squeezed = std::abs(d_apex) < std::abs(target_apex) - 1e-3;
    // 감속은 실제로 깎인 간격에 비례 (이분법이면 균등배분 apex 특성상 거의
    // 모든 정적 회피가 squeeze로 분류돼 과도하게 감속한다):
    // 간격이 evasion_dist면 감속 없음, evasion_dist_min까지 깎이면 최대 감속
    double clearance = (more_space == "left")
        ? d_apex - closest_obs.d_left
        : closest_obs.d_right - d_apex;
    double t_sq = std::clamp(
        (clearance - evasion_dist_min_) /
            std::max(evasion_dist_ - evasion_dist_min_, 0.01), 0.0, 1.0);
    double squeeze_scale_eff =
        squeeze_speed_scale_ + t_sq * (1.0 - squeeze_speed_scale_);

    // 스플라인 절점: pre-apex 3점 + apex 구간 + post-apex 3점.
    // apex는 점이 아니라 장애물(클러스터)의 s 구간 [s_front, s_back] — 점
    // 장애물이면 기존 단일 apex와 사실상 동일하고, 병합된 긴 클러스터면
    // 구간 전체에서 d_apex를 유지한다 (단일 apex는 꼬리에서 d=0으로
    // 되돌아와 클러스터 뒷부분을 관통).
    // apex 유지 구간을 장애물 앞뒤로 0.5m 연장 — pure pursuit lookahead가
    // 진입/이탈 대각선을 안쪽(장애물 쪽)으로 커팅해 스치는 것을 방지
    double obs_span = Mod(closest_obs.s_end - closest_obs.s_start, gb_max_s_);
    double s_front = s_apex - obs_span / 2.0 - 0.5;
    double s_back = s_apex + obs_span / 2.0 + 0.5;

    double speed_scale = std::clamp(1.0 + cur_vs_ / std::max(gb_vmax_, 0.1), 1.0, 1.5);
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

    // path jump 방지 (mina Task 2): 차량이 이미 스플라인 구간 안에 있으면
    // (회피 중 재계획) 차량 뒤쪽 절점을 버리고 현재 위치 (cur_s, cur_d)를
    // 첫 절점으로 삼아 새 경로가 차량 현재 상태에서 연속적으로 시작하게 한다.
    {
        double cur_unwrapped = s_apex - Mod(s_apex - cur_s_, gb_max_s_);
        if (cur_unwrapped > knot_s.front() &&
            cur_unwrapped < s_front - wpnt_dist_) {
            size_t keep = 0;
            while (keep < knot_s.size() && knot_s[keep] <= cur_unwrapped + wpnt_dist_) {
                ++keep;
            }
            knot_s.erase(knot_s.begin(), knot_s.begin() + keep);
            knot_d.erase(knot_d.begin(), knot_d.begin() + keep);
            knot_s.insert(knot_s.begin(), cur_unwrapped);
            knot_d.insert(knot_d.begin(), cur_d_);
        }
    }

    // d(s) 자연 3차 스플라인을 0.1m 간격으로 샘플
    std::vector<double> evasion_s;
    for (double s = knot_s.front(); s < knot_s.back(); s += spline_resolution_) {
        evasion_s.push_back(s);
    }
    std::vector<double> evasion_d = NaturalCubicSpline(knot_s, knot_d, evasion_s);
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
        double s_wrapped = Mod(evasion_s[i], gb_max_s_);
        int gb_idx = static_cast<int>(s_wrapped / wpnt_dist_) % gb_size;

        // raceline에서 실제로 벗어난 구간만 경계 거리 검사.
        // 허용치를 넘쳐도 바로 포기하지 않고 그 지점만 벽 허용치로 눌러본다 —
        // 눌렀을 때 장애물 구간에서 evasion_dist_min을 침범할 때만 포기.
        // (벽 폭은 s 따라 변하므로 한 점 넘쳤다고 전체를 버리면 회피포기 남발)
        if (std::abs(evasion_d[i]) > spline_resolution_) {
            double tb_dist = (more_space == "left") ?
                gb_wpnts_[gb_idx].d_left : gb_wpnts_[gb_idx].d_right;
            bool in_obs_span = (evasion_s[i] > s_front - 0.3) &&
                               (evasion_s[i] < s_back + 0.3);
            // 공격 마진(0.1)은 장애물 corridor 구간에서만 — 진입/복귀 구간이
            // 벽 허용치에 계속 붙어 다니면 추적오차로 벽을 스친다 (실측)
            double bound = in_obs_span ? active_bound_mindist_
                                       : spline_bound_mindist_;
            double limit = std::max(std::abs(tb_dist) - bound, 0.0);
            if (std::abs(evasion_d[i]) > limit) {
                double clamped = (evasion_d[i] > 0.0) ? limit : -limit;
                bool cuts_obstacle = in_obs_span &&
                    ((more_space == "left")
                         ? clamped < closest_obs.d_left + evasion_dist_min_
                         : clamped > closest_obs.d_right - evasion_dist_min_);
                if (cuts_obstacle) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
                        "Evasion trajectory too close to TRACKBOUNDS, aborting evasion");
                    pitwall::Event("회피 포기(트랙경계)");
                    out_wpnts.wpnts.clear();
                    return false;
                }
                evasion_d[i] = clamped;
                squeezed_now = true;  // 벽에 눌린 구간부터는 감속
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
        // 코너 안쪽 통과 시 감속 + 추월 섹터 내 공격 부스트 (raceline 최고속 캡).
        // squeeze(벽 마진으로 축소된 회피) 감속은 횡이동 직전(brake_dist)부터만 —
        // 스플라인이 장애물 한참 앞에서 시작하므로 전 구간 감속하면 접근이
        // 느려져 상대 꽁무니만 따라간다. 접근 구간은 기존 속도 유지.
        wpnt.vx_mps = (outside == more_space) ?
            gb_wpnts_[gb_idx].vx_mps : gb_wpnts_[gb_idx].vx_mps * inside_speed_scale_;
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
            wpnt.vx_mps = std::clamp(wpnt.vx_mps, std::min(v_floor, gb_vmax_), gb_vmax_);
        }
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

bool SplinerNode::BuildTrailing(
    const ae_hyu_msgs::msg::Obstacle &obs,
    ae_hyu_msgs::msg::WpntArray &out_wpnts,
    visualization_msgs::msg::MarkerArray &out_mrks) {

    int gb_size = static_cast<int>(gb_wpnts_.size());
    if (gb_size < 2) {
        return false;
    }
    // 목표 속도: 동적(또는 예측) 장애물이면 그 진행 속도, 정적이면 0.
    // 실측 속도(m/s)를 wpnt 속도 단위로 환산 (컨트롤러가 다시 scale을 곱함)
    double v_target = std::max(obs.vs, 0.0) / std::max(cmd_speed_scale_, 0.05);

    // 정지 목표점: 장애물 앞 가장자리에서 trailing_gap 만큼 뒤
    double span = Mod(obs.s_start - trailing_gap_m_ - cur_s_, gb_max_s_);
    if (span > lookahead_m_ + 5.0) {
        span = 0.0;      // 이미 gap 안쪽까지 파고든 상태
        v_target *= 0.85;  // 살짝 느리게 가서 간격을 다시 벌린다 (범퍼 접촉 방지)
    }

    // 상대는 코너에서 감속한다 — 지금 측정 속도로만 프로파일을 짜면 상대가
    // 헤어핀 진입 중 감속할 때 gap이 무너져 상대를 관통한다 (실측).
    // 상대도 코너 한계속도(raceline 속도 프로파일) 이상으로는 못 돈다고
    // 가정하고, 정지점~장애물 구간 raceline 최저 속도를 목표 속도 상한으로.
    // (raceline 대비 비율 유지 모델은 정속 주행 상대를 과잉 감속 예측해
    //  ego가 상대 절반 속도로 기어가는 부작용 — 실측 폐기)
    double v_target_prof = v_target;
    if (obs.vs > 0.1) {
        int stop_gi = static_cast<int>(
            Mod(obs.s_start - trailing_gap_m_, gb_max_s_) / wpnt_dist_) % gb_size;
        int n_ahead = static_cast<int>(
            (trailing_gap_m_ + 5.0) / std::max(wpnt_dist_, 0.05));
        double v_corner = gb_wpnts_[stop_gi].vx_mps;
        for (int i = 1; i <= n_ahead; ++i) {
            v_corner = std::min(
                v_corner, gb_wpnts_[(stop_gi + i) % gb_size].vx_mps);
        }
        v_target_prof = std::min(v_target, v_corner);
    }

    // 정지점 뒤로도 v_target 구간을 연장한다 — 컨트롤러 lookahead(수 m)가
    // 궤적 밖의 원본 raceline 속도(전속)를 집어 재가속하는 것을 방지
    const double extend = 5.0;
    // 회피 중(|cur_d| 큼) 폴백이 걸리면 경로가 d=0으로 순간이동해 차가
    // 장애물 쪽으로 끌려간다 — 현재 d에서 3m에 걸쳐 raceline으로 복귀
    const double d_ramp_len = 3.0;
    int n = std::max(static_cast<int>((span + extend) / wpnt_dist_), 2);
    out_wpnts.wpnts.reserve(n);
    for (int i = 0; i < n; ++i) {
        double dist = i * wpnt_dist_;
        double s = Mod(cur_s_ + dist, gb_max_s_);
        int gi = static_cast<int>(s / wpnt_dist_) % gb_size;
        ae_hyu_msgs::msg::Wpnt w = gb_wpnts_[gi];
        w.id = i;
        double d = (std::abs(cur_d_) > 0.05 && dist < d_ramp_len)
            ? cur_d_ * (1.0 - dist / d_ramp_len) : 0.0;
        w.d_m = d;
        if (std::abs(d) > 0.02) {
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
        if (rem <= engage) {
            double v_allow = std::sqrt(
                v_target_prof * v_target_prof + 2.0 * trailing_decel_ * rem);
            w.vx_mps = std::min(w.vx_mps, v_allow);
        }
        out_wpnts.wpnts.push_back(w);
        out_mrks.markers.push_back(MakeWpntMarker(w.x_m, w.y_m, w.vx_mps, i));
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
        "Trailing fallback: obs s=%.1f gap=%.1fm v_target=%.1f (%zu wpnts)",
        obs.s_center, span, v_target, out_wpnts.wpnts.size());
    return true;
}

bool SplinerNode::BuildSpLaneChange(
    const ae_hyu_msgs::msg::Obstacle &closest_obs,
    const std::vector<ae_hyu_msgs::msg::Obstacle> &all_ahead,
    ae_hyu_msgs::msg::WpntArray &out_wpnts,
    visualization_msgs::msg::MarkerArray &out_mrks) {

    int gb_size = static_cast<int>(gb_wpnts_.size());
    if (gb_size < 2 || sp_d_.empty()) {
        return false;
    }

    // 상대(클러스터) 구간을 ego 기준 상대 거리로
    double dist_front = Mod(closest_obs.s_start - cur_s_, gb_max_s_);
    double dist_back =
        dist_front + Mod(closest_obs.s_end - closest_obs.s_start, gb_max_s_);
    if (dist_front > lookahead_m_ + 10.0 || dist_front < 3.0) {
        return false;  // 너무 멀거나, 블렌드가 상대 앞에서 못 끝날 만큼 가까움
    }

    // 진입 블렌드는 상대 1m 앞에서 끝나게 (가까우면 빠르게 갈아탄다),
    // 상대 뒤로 hold_exit 만큼 SP 유지 후 blend_out으로 raceline 복귀
    double blend_in = std::clamp(dist_front - 1.0, 2.0, 10.0);
    const double hold_exit = 5.0;
    const double blend_out = 6.0;
    double total = dist_back + hold_exit + blend_out;

    int n = static_cast<int>(total / wpnt_dist_);
    if (n < 10) {
        return false;
    }
    out_wpnts.wpnts.reserve(n);
    std::vector<std::pair<double, double>> xy(n);
    for (int i = 0; i < n; ++i) {
        double t = i * wpnt_dist_;
        double s = Mod(cur_s_ + t, gb_max_s_);
        int gi = static_cast<int>(s / wpnt_dist_) % gb_size;
        double d_target = sp_d_[gi];
        double d;
        if (t < blend_in) {
            double w = t / blend_in;
            w = w * w * (3.0 - 2.0 * w);  // smoothstep
            d = cur_d_ * (1.0 - w) + d_target * w;
        } else if (t < dist_back + hold_exit) {
            d = d_target;
        } else {
            double w = std::clamp((t - dist_back - hold_exit) / blend_out, 0.0, 1.0);
            w = w * w * (3.0 - 2.0 * w);
            d = d_target * (1.0 - w);
        }
        // 경계 검사는 블렌드(보간) 구간만 — SP 라인 위(d==d_sp)는 자체
        // safe-dist로 생성된 검증된 라인이라 벽 마진 검사를 하면 벽에 붙는
        // 최단거리 라인 특성상 코너에서 항상 탈락해 버린다
        bool on_line = (t >= blend_in) && (t < dist_back + hold_exit);
        if (!on_line) {
            double wall = (d >= 0.0) ? std::abs(gb_wpnts_[gi].d_left)
                                     : std::abs(gb_wpnts_[gi].d_right);
            if (std::abs(d) > wall - spline_bound_mindist_) {
                out_wpnts.wpnts.clear();
                return false;
            }
        }
        xy[i] = FrenetToCartesian(s, d);

        ae_hyu_msgs::msg::Wpnt wpnt;
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
        wpnt.vx_mps = std::clamp(vx, std::min(v_floor, gb_vmax_), gb_vmax_);
        wpnt.ax_mps2 = gb_wpnts_[gi].ax_mps2;
        out_wpnts.wpnts.push_back(wpnt);
    }

    // 상대 "자신 포함" 전체 장애물과 충돌검사 — SP 라인이 그 구간에서
    // 상대 주행 라인과 충분히 벌어지지 않으면 레인체인지 불가
    if (!CheckPathCollision(out_wpnts.wpnts, all_ahead, -99999, nullptr)) {
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
        "SP lane change: obs s=%.1f blend_in=%.1fm (%zu wpnts)",
        closest_obs.s_center, blend_in, out_wpnts.wpnts.size());
    return true;
}

std::vector<ae_hyu_msgs::msg::Obstacle> SplinerNode::ClusterObstacles(
    std::vector<ae_hyu_msgs::msg::Obstacle> obs) const {
    // 사이 corridor가 cluster_gap_d보다 좁고 s 간격이 cluster_gap_s 이내인
    // 쌍을 하나의 (s,d) 박스로 반복 병합 (합쳐진 박스가 또 다른 장애물과
    // 붙으면 다시 병합 — 수렴할 때까지)
    // ego "뒤 15m"를 기준점으로 unwrap — ego가 클러스터 안(두 장애물 사이)에
    // 진입해 일부가 살짝 뒤로 가도 클러스터가 wrap 때문에 쪼개지지 않게 한다
    // (쪼개지면 남은 장애물 기준으로 사이드가 바뀌며 회피가 중간에 붕괴)
    double ref = Mod(cur_s_ - 15.0, gb_max_s_);
    bool merged_any = true;
    while (merged_any && obs.size() > 1) {
        merged_any = false;
        for (size_t i = 0; i < obs.size() && !merged_any; ++i) {
            for (size_t j = i + 1; j < obs.size() && !merged_any; ++j) {
                double ai = Mod(obs[i].s_start - ref, gb_max_s_);
                double bi = ai + Mod(obs[i].s_end - obs[i].s_start, gb_max_s_);
                double aj = Mod(obs[j].s_start - ref, gb_max_s_);
                double bj = aj + Mod(obs[j].s_end - obs[j].s_start, gb_max_s_);
                double s_gap = std::max(ai, aj) - std::min(bi, bj);
                double d_gap = std::max(obs[i].d_right, obs[j].d_right) -
                               std::min(obs[i].d_left, obs[j].d_left);
                if (s_gap < cluster_gap_s_m_ && d_gap < cluster_gap_d_m_) {
                    auto &a = obs[i];
                    const auto &b = obs[j];
                    double na = std::min(ai, aj);
                    double nb = std::max(bi, bj);
                    a.s_start = Mod(ref + na, gb_max_s_);
                    a.s_end = Mod(ref + nb, gb_max_s_);
                    a.s_center = Mod(ref + (na + nb) / 2.0, gb_max_s_);
                    a.d_left = std::max(a.d_left, b.d_left);
                    a.d_right = std::min(a.d_right, b.d_right);
                    a.d_center = (a.d_left + a.d_right) / 2.0;
                    a.vs = std::min(a.vs, b.vs);       // trailing 목표속도 보수적
                    // 구성원 중 하나라도 움직이면 클러스터는 동적 (OR였을 때
                    // 예측 구간과 합쳐진 상대가 정적으로 분류돼 커밋/부스트 오동작)
                    a.is_static = a.is_static && b.is_static;
                    a.id = std::min(a.id, b.id);
                    obs.erase(obs.begin() + static_cast<long>(j));
                    merged_any = true;
                }
            }
        }
    }
    return obs;
}

std::vector<ae_hyu_msgs::msg::Obstacle> SplinerNode::ObstaclesInLookahead(
    const std::vector<ae_hyu_msgs::msg::Obstacle> &obstacles) const {
    std::vector<ae_hyu_msgs::msg::Obstacle> out;
    for (const auto &raw : obstacles) {
        auto obs = PredictObstacleMovement(raw);
        // 장애물 몸통 일부라도 lookahead 안이면 검증 대상
        if (Mod(obs.s_start - cur_s_, gb_max_s_) < lookahead_m_ ||
            Mod(obs.s_end - cur_s_, gb_max_s_) < lookahead_m_) {
            out.push_back(obs);
        }
    }
    return out;
}

bool SplinerNode::CheckPathCollision(
    const std::vector<ae_hyu_msgs::msg::Wpnt> &path,
    const std::vector<ae_hyu_msgs::msg::Obstacle> &obstacles,
    int32_t exclude_id, double *blocking_s) const {
    // 경로 각 점의 (s, d)가 어떤 장애물의 확장 (s,d) 박스 안이면 충돌.
    // 횡방향은 path_check_margin만큼 확장 (차폭/오차 여유).
    // exclude_id 장애물은 제외 — 회피 스플라인은 그 옆을, trailing은 그
    // 뒤(속도 프로파일로 정지)를 의도적으로 지나므로 기하 검사 대상이 아니다.
    for (const auto &obs : obstacles) {
        if (obs.id == exclude_id) continue;
        double span = Mod(obs.s_end - obs.s_start, gb_max_s_);
        for (const auto &wp : path) {
            // 차량 뒤쪽/lookahead 밖 경로 점은 검사 대상 아님
            if (Mod(wp.s_m - cur_s_, gb_max_s_) > lookahead_m_) continue;
            double off = Mod(wp.s_m - obs.s_start, gb_max_s_);
            if (off > span) continue;  // s 구간 밖
            if (wp.d_m > obs.d_right - path_check_margin_ &&
                wp.d_m < obs.d_left + path_check_margin_) {
                if (blocking_s) {
                    *blocking_s = Mod(obs.s_start, gb_max_s_);
                }
                return false;  // 충돌
            }
        }
    }
    return true;  // 충돌 없음
}

void SplinerNode::BuildStopWpnts(
    double blocking_s, ae_hyu_msgs::msg::WpntArray &out_wpnts,
    visualization_msgs::msg::MarkerArray &out_mrks) {
    // 최후 폴백: raceline을 따라가되 차단 지점 앞 stop_margin_m에서 vx=0.
    // 감속은 trailing_decel 물리 한계(sqrt)와 정지점 앞 stop_ramp 선형 램프 중
    // 낮은 쪽 — 원거리에선 물리적으로 제때 감속을 시작하고(관통 방지),
    // 근거리에선 선형으로 부드럽게 0에 붙는다.
    int gb_size = static_cast<int>(gb_wpnts_.size());
    if (gb_size < 2) {
        return;
    }
    double dist_to_block = Mod(blocking_s - cur_s_, gb_max_s_);
    double stop_dist = std::max(dist_to_block - stop_margin_m_, 0.0);

    // 정지점 뒤로도 vx=0 구간을 연장 — 컨트롤러 lookahead가 궤적 밖의 원본
    // raceline 속도(전속)를 집어 재가속하는 것을 방지 (BuildTrailing과 동일)
    const double extend = 5.0;
    const double d_ramp_len = 3.0;  // 회피 중 폴백 시 cur_d -> 0 복귀 구간
    int n = std::max(static_cast<int>((stop_dist + extend) / wpnt_dist_), 2);
    out_wpnts.wpnts.reserve(n);
    for (int i = 0; i < n; ++i) {
        double dist = i * wpnt_dist_;
        double s = Mod(cur_s_ + dist, gb_max_s_);
        int gi = static_cast<int>(s / wpnt_dist_) % gb_size;
        ae_hyu_msgs::msg::Wpnt w = gb_wpnts_[gi];
        w.id = i;
        double d = (std::abs(cur_d_) > 0.05 && dist < d_ramp_len)
            ? cur_d_ * (1.0 - dist / d_ramp_len) : 0.0;
        w.d_m = d;
        if (std::abs(d) > 0.02) {
            auto [x, y] = FrenetToCartesian(s, d);
            w.x_m = x;
            w.y_m = y;
            w.d_left -= d;
            w.d_right += d;
        }
        double remain = std::max(stop_dist - dist, 0.0);
        double v_phys = std::sqrt(2.0 * trailing_decel_ * remain);
        double v_ramp = (remain < stop_ramp_dist_m_)
            ? w.vx_mps * (remain / std::max(stop_ramp_dist_m_, 0.01))
            : w.vx_mps;
        w.vx_mps = std::min({w.vx_mps, v_phys, v_ramp});
        w.ax_mps2 = 0.0;
        out_wpnts.wpnts.push_back(w);
        out_mrks.markers.push_back(MakeWpntMarker(w.x_m, w.y_m, w.vx_mps, i));
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000,
        "Stop fallback: blocking s=%.1f stop_dist=%.1fm (%zu wpnts)",
        blocking_s, stop_dist, out_wpnts.wpnts.size());
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
