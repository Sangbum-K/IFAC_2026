#include "planning_pkg/global_trajectory_node.hpp"

#include <cctype>
#include <filesystem>
#include <limits>
#include <stdexcept>

namespace
{
constexpr char kGenerationPrefix[] = "# generation_id=";

bool IsGenerationId(const std::string & value)
{
    return value.size() == 32U &&
           std::all_of(
        value.begin(), value.end(),
        [](unsigned char character) {return std::isxdigit(character) != 0;});
}
}  // namespace

GlobalTrajectoryNode::GlobalTrajectoryNode(const std::string &node_name, const rclcpp::NodeOptions &options)
    : Node(node_name, options) {

    // QoS init
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Parameters - declare and get values
    this->declare_parameter(
        "global_trajectory/trajectory_csv_file",
        "my_track0807_iqp_max7.00ms_margin0.08m_mu0.68.csv");
    this->declare_parameter("global_trajectory/overtaking_csv_file", "");
    this->declare_parameter("global_trajectory/publish_rate_hz", 2.0);
    this->declare_parameter("global_trajectory/velocity_scale", 1.0);
    this->declare_parameter("global_trajectory/min_velocity_scale", 0.1);
    this->declare_parameter("global_trajectory/max_velocity_scale", 1.0);
    this->declare_parameter("global_trajectory/velocity_scale_slew_per_sec", 0.1);
    this->declare_parameter("global_trajectory/validation_mu", 0.75);
    this->declare_parameter("global_trajectory/validation_wheelbase_m", 0.33);
    this->declare_parameter("global_trajectory/validation_max_steer_rad", 0.3578);
    this->declare_parameter("global_trajectory/validation_tolerance", 0.03);
    this->declare_parameter("global_trajectory/validation_ax_tolerance", 1.0);
    // "strict"(기본) | "advisory". advisory 는 마찰원/조향 한계 위반을 경고로만
    // 남기고 노드를 띄운다 — 사람이 직감으로 손댄 <맵>_tuned_ 라인을 실차에서
    // 시험하기 위한 것이다. 형식 검사는 어느 모드에서도 통과해야 한다.
    this->declare_parameter("global_trajectory/validation_mode", "strict");
    // ── 조향 능력 표 (2026-08-23) ────────────────────────────────────────
    // control 이 쓰는 조향 LUT 을 뒤집어 얻은 "그 속도에서 최대 조향으로 낼 수
    // 있는 횡가속". 근거·되돌리는 법은 steer_grip_model.hpp 머리주석.
    // 빈 배열 = 비활성 = 종전 동작(CSV vx 를 그대로 발행).
    this->declare_parameter(
        "global_trajectory/steer_lat_acc_speed_brk", std::vector<double>{});
    this->declare_parameter(
        "global_trajectory/steer_lat_acc_max_at", std::vector<double>{});
    this->declare_parameter("global_trajectory/steer_lat_acc_margin", 1.0);

    trajectory_csv_file_ = this->get_parameter("global_trajectory/trajectory_csv_file").as_string();
    overtaking_csv_file_ = this->get_parameter("global_trajectory/overtaking_csv_file").as_string();
    publish_rate_hz_ = this->get_parameter("global_trajectory/publish_rate_hz").as_double();
    velocity_scale_ = this->get_parameter("global_trajectory/velocity_scale").as_double();
    min_velocity_scale_ =
        this->get_parameter("global_trajectory/min_velocity_scale").as_double();
    max_velocity_scale_ =
        this->get_parameter("global_trajectory/max_velocity_scale").as_double();
    velocity_scale_slew_per_sec_ =
        this->get_parameter("global_trajectory/velocity_scale_slew_per_sec").as_double();
    validation_mu_ = this->get_parameter("global_trajectory/validation_mu").as_double();
    validation_wheelbase_m_ =
        this->get_parameter("global_trajectory/validation_wheelbase_m").as_double();
    validation_max_steer_rad_ =
        this->get_parameter("global_trajectory/validation_max_steer_rad").as_double();
    validation_tolerance_ =
        this->get_parameter("global_trajectory/validation_tolerance").as_double();
    validation_ax_tolerance_ =
        this->get_parameter("global_trajectory/validation_ax_tolerance").as_double();
    const std::string validation_mode =
        this->get_parameter("global_trajectory/validation_mode").as_string();
    if (validation_mode == "advisory") {
        validation_advisory_ = true;
    } else if (validation_mode != "strict") {
        // 오타를 조용히 strict 로 떨어뜨리지 않는다. 반대 방향(advisory 를 원했는데
        // strict 가 되는 것)은 노드가 안 뜨는 것으로 드러나지만, 그 반대는
        // 검증이 꺼진 채로 주행하는 것이라 반드시 여기서 죽어야 한다.
        throw std::runtime_error(
            "global_trajectory/validation_mode must be \"strict\" or \"advisory\", got \"" +
            validation_mode + "\"");
    }

    if (!(publish_rate_hz_ > 0.0) || !std::isfinite(publish_rate_hz_) ||
        !std::isfinite(velocity_scale_) ||
        !std::isfinite(min_velocity_scale_) ||
        !std::isfinite(max_velocity_scale_) ||
        !std::isfinite(velocity_scale_slew_per_sec_) ||
        !std::isfinite(validation_mu_) ||
        !std::isfinite(validation_wheelbase_m_) ||
        !std::isfinite(validation_max_steer_rad_) ||
        !std::isfinite(validation_tolerance_) ||
        !std::isfinite(validation_ax_tolerance_) ||
        !(validation_mu_ > 0.0 && validation_mu_ <= 1.5) ||
        !(validation_wheelbase_m_ >= 0.1 && validation_wheelbase_m_ <= 1.0) ||
        !(validation_max_steer_rad_ > 0.0 &&
          validation_max_steer_rad_ <= 0.8) ||
        validation_tolerance_ < 0.0 || validation_tolerance_ > 0.10 ||
        validation_ax_tolerance_ < 0.0 || validation_ax_tolerance_ > 1.0 ||
        velocity_scale_slew_per_sec_ < 0.0) {
        throw std::runtime_error("Invalid/non-finite global trajectory safety parameters");
    }
    min_velocity_scale_ = std::clamp(min_velocity_scale_, 0.05, 1.0);
    max_velocity_scale_ = std::clamp(max_velocity_scale_, min_velocity_scale_, 1.0);
    target_velocity_scale_ =
        std::clamp(velocity_scale_, min_velocity_scale_, max_velocity_scale_);
    if (target_velocity_scale_ != velocity_scale_) {
        RCLCPP_WARN(
            this->get_logger(),
            "velocity_scale %.3f is outside the certified range [%.3f, %.3f]; clamped",
            velocity_scale_, min_velocity_scale_, max_velocity_scale_);
    }
    velocity_scale_ = target_velocity_scale_;

    RCLCPP_INFO(this->get_logger(), "trajectory_csv_file: %s", trajectory_csv_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "publish_rate_hz: %f", publish_rate_hz_);
    RCLCPP_INFO(this->get_logger(), "velocity_scale: %f", velocity_scale_);
    if (validation_advisory_) {
        RCLCPP_WARN(
            this->get_logger(),
            "======================= validation_mode=advisory =======================\n"
            "  마찰원/조향 한계 위반을 거부하지 않고 경고로만 남깁니다.\n"
            "  사람이 손으로 튜닝한 라인(<맵>_tuned_)을 실차에서 시험하는 경로입니다.\n"
            "  이 CSV 는 차가 물리적으로 따라갈 수 없는 구간을 포함할 수 있습니다 —\n"
            "  속도를 낮춰 시작하고 사람이 붙어 있을 때만 쓰십시오.\n"
            "  원본 라인으로 돌아가려면 map:= 에서 _tuned 접미사를 떼십시오.\n"
            "========================================================================");
    }

    if (!steer_grip_.Set(
            this->get_parameter("global_trajectory/steer_lat_acc_speed_brk")
                .as_double_array(),
            this->get_parameter("global_trajectory/steer_lat_acc_max_at")
                .as_double_array(),
            this->get_parameter("global_trajectory/steer_lat_acc_margin")
                .as_double())) {
        throw std::runtime_error(
            "global_trajectory/steer_lat_acc_* 표가 유효하지 않습니다 "
            "(길이 불일치 / 속도 비단조 / 비양수 / margin 범위 밖)");
    }
    RCLCPP_INFO(
        this->get_logger(), "steering-capability speed cap: %s",
        steer_grip_.enabled() ? "ON" : "off (표 비어 있음 — 종전 동작)");

    // Load global trajectory from CSV
    if (LoadTrajectoryFromCSV(
            trajectory_csv_file_, global_trajectory_, main_generation_id_)) {
        BuildSteerSpeedCap(global_trajectory_, global_steer_cap_);
        RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu waypoints from CSV", global_trajectory_.size());
    } else {
        throw std::runtime_error(
            "Failed to load or validate main trajectory CSV: " + trajectory_csv_file_);
    }

    // SP 추월 라인 (H2H lane-change용, unicorn의 IQP+SP 이중 라인 구조).
    // 파일이 없으면 경고만 남기고 메인 라인만 발행한다.
    if (!overtaking_csv_file_.empty()) {
        if (LoadTrajectoryFromCSV(
                overtaking_csv_file_, overtaking_trajectory_,
                overtaking_generation_id_) &&
            !main_generation_id_.empty() &&
            main_generation_id_ == overtaking_generation_id_ &&
            ValidateTrajectoryPair(global_trajectory_, overtaking_trajectory_)) {
            BuildSteerSpeedCap(overtaking_trajectory_, overtaking_steer_cap_);
            RCLCPP_INFO(this->get_logger(), "Loaded overtaking (SP) line: %zu waypoints (%s)",
                        overtaking_trajectory_.size(), overtaking_csv_file_.c_str());
        } else {
            RCLCPP_ERROR(
                this->get_logger(),
                "Overtaking CSV is unsafe, incompatible, or not from the same verified "
                "generation as the main line (%s, IQP generation='%s', SP generation='%s') "
                "— SP disabled, main line only",
                overtaking_csv_file_.c_str(), main_generation_id_.c_str(),
                overtaking_generation_id_.c_str());
            overtaking_trajectory_.clear();
            overtaking_steer_cap_.clear();
            overtaking_generation_id_.clear();
        }
    }

    // Publisher init
    p_global_waypoints_ = this->create_publisher<f1tenth_planning_msgs::msg::WpntArray>(
        "/global_waypoints", qos_profile);
    if (!overtaking_trajectory_.empty()) {
        p_overtaking_waypoints_ = this->create_publisher<f1tenth_planning_msgs::msg::WpntArray>(
            "/global_waypoints/overtaking", qos_profile);
    }

    // Timer init - publish global waypoints periodically
    t_publish_waypoints_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_hz_),
        [this]() { this->Run(); });

    RCLCPP_INFO(this->get_logger(), "GlobalTrajectoryNode node initialized successfully");
}

GlobalTrajectoryNode::~GlobalTrajectoryNode() {}

void GlobalTrajectoryNode::ProcessParams() {
    double requested = target_velocity_scale_;
    this->get_parameter("global_trajectory/velocity_scale", requested);
    if (!std::isfinite(requested)) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *get_clock(), 2000,
            "Ignoring non-finite global_trajectory/velocity_scale");
        requested = target_velocity_scale_;
    }
    target_velocity_scale_ =
        std::clamp(requested, min_velocity_scale_, max_velocity_scale_);
    if (requested > max_velocity_scale_) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *get_clock(), 2000,
            "velocity_scale %.3f exceeds certified maximum %.3f; regenerate the velocity "
            "profile instead of scaling above 1.0",
            requested, max_velocity_scale_);
    }
    const double max_step =
        std::max(velocity_scale_slew_per_sec_, 0.0) / publish_rate_hz_;
    if (target_velocity_scale_ <= velocity_scale_) {
        // 감속 요청은 즉시 적용한다. slew는 가속 재개에만 걸어 안전 요청을
        // 늦추지 않는다.
        velocity_scale_ = target_velocity_scale_;
    } else if (max_step <= 0.0) {
        velocity_scale_ = target_velocity_scale_;
    } else {
        velocity_scale_ +=
            std::min(target_velocity_scale_ - velocity_scale_, max_step);
    }
}

void GlobalTrajectoryNode::Run() {

    // 동적으로 velocity 스케일링 파라미터 적용
    this->ProcessParams();

    // Publish global waypoints
    this->PublishGlobalWaypoints();
}

bool GlobalTrajectoryNode::LoadTrajectoryFromCSV(const std::string& csv_filename,
                                                 std::vector<std::vector<double>>& out_trajectory,
                                                 std::string& out_generation_id) {
    try {
        const std::filesystem::path filename_path(csv_filename);
        if (csv_filename.empty() || filename_path.is_absolute() ||
            filename_path.has_parent_path() || filename_path.extension() != ".csv") {
            RCLCPP_ERROR(
                this->get_logger(),
                "Trajectory filename must be a plain .csv basename: '%s'",
                csv_filename.c_str());
            return false;
        }
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("planning_pkg");
        std::string csv_path = package_share_directory + "/config/global_trajectory/" + csv_filename;

        RCLCPP_INFO(this->get_logger(), "Loading trajectory from: %s", csv_path.c_str());

        std::ifstream file(csv_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open CSV file: %s", csv_path.c_str());
            return false;
        }

        out_trajectory.clear();
        out_generation_id.clear();
        std::string line;
        bool saw_data = false;

        while (std::getline(file, line)) {
            // Skip empty lines and comment lines (starting with '#')
            if (line.empty()) {
                continue;
            }
            if (line[0] == '#') {
                if (line.rfind(kGenerationPrefix, 0) == 0) {
                    std::string generation_id =
                        line.substr(sizeof(kGenerationPrefix) - 1U);
                    if (!generation_id.empty() && generation_id.back() == '\r') {
                        generation_id.pop_back();
                    }
                    if (saw_data || !out_generation_id.empty() ||
                        !IsGenerationId(generation_id)) {
                        RCLCPP_ERROR(
                            this->get_logger(),
                            "Malformed, duplicate, or late generation_id in %s",
                            csv_filename.c_str());
                        return false;
                    }
                    out_generation_id = generation_id;
                }
                continue;
            }
            saw_data = true;

            std::stringstream ss(line);
            std::string cell;
            std::vector<double> row;

            // Parse CSV line with SEMICOLON delimiter (not comma!)
            while (std::getline(ss, cell, ';')) {
                try {
                    // Trim whitespace
                    cell.erase(0, cell.find_first_not_of(" \t"));
                    cell.erase(cell.find_last_not_of(" \t") + 1);

                    // Skip empty cells
                    if (cell.empty()) {
                        continue;
                    }

                    std::size_t parsed = 0;
                    double value = std::stod(cell, &parsed);
                    if (parsed != cell.size()) {
                        RCLCPP_ERROR(
                            this->get_logger(), "Trailing characters in CSV value: '%s'",
                            cell.c_str());
                        return false;
                    }
                    row.push_back(value);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error parsing CSV value: '%s' - %s", cell.c_str(), e.what());
                    return false;
                }
            }

            // Expect exactly 9 columns: s, x, y, right, left, psi, kappa, vx, ax.
            if (row.size() == 9) {
                out_trajectory.push_back(row);
            } else if (row.size() > 0) {
                RCLCPP_ERROR(this->get_logger(),
                    "Rejecting CSV line with %zu columns (expected exactly 9): %s",
                    row.size(), line.c_str());
                return false;
            }
        }

        file.close();

        if (out_trajectory.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No valid waypoints found in CSV file");
            return false;
        }

        // 파일명/라인 종류와 무관하게 차량이 따라갈 수 없는 곡률은 거부한다.
        if (!ValidateTrajectory(csv_filename, out_trajectory)) {
            out_trajectory.clear();
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu validated waypoints",
                    out_trajectory.size());
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while loading CSV: %s", e.what());
        return false;
    }
}

bool GlobalTrajectoryNode::ValidateTrajectory(
    const std::string &name,
    const std::vector<std::vector<double>> &trajectory) const {
    if (trajectory.size() < 8) {
        RCLCPP_ERROR(this->get_logger(), "%s has fewer than 8 waypoints", name.c_str());
        return false;
    }
    const double g_limit = validation_mu_ * 9.81;
    const double kappa_limit =
        std::tan(validation_max_steer_rad_) /
        std::max(validation_wheelbase_m_, 1e-6);
    std::vector<double> spacings;
    spacings.reserve(trajectory.size() - 1);
    // advisory 모드 물리 위반 집계. 행마다 경고를 뿜으면 314행 CSV 에서 로그가
    // 쓸려 내려가므로 최악값만 모아 마지막에 한 번 보고한다. strict 모드에서는
    // 예전과 똑같이 첫 위반 행에서 즉시 거부하므로 이 변수들은 쓰이지 않는다.
    size_t steer_violations = 0;
    size_t friction_violations = 0;
    double worst_kappa = 0.0;
    size_t worst_kappa_row = 0;
    double worst_combined = 0.0;
    size_t worst_combined_row = 0;
    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto &r = trajectory[i];
        if (r.size() != 9 ||
            !std::all_of(r.begin(), r.end(), [](double v) { return std::isfinite(v); })) {
            RCLCPP_ERROR(this->get_logger(), "%s row %zu contains invalid/non-finite data",
                         name.c_str(), i);
            return false;
        }
        if (r[0] < 0.0 || r[3] < 0.0 || r[4] < 0.0 || r[7] < 0.0) {
            RCLCPP_ERROR(this->get_logger(), "%s row %zu has a negative s/bound/speed",
                         name.c_str(), i);
            return false;
        }
        if (i > 0) {
            const double ds = r[0] - trajectory[i - 1][0];
            if (!(ds > 1e-5)) {
                RCLCPP_ERROR(this->get_logger(), "%s s is not strictly increasing at row %zu",
                             name.c_str(), i);
                return false;
            }
            spacings.push_back(ds);
            const double geometric_ds = std::hypot(
                r[1] - trajectory[i - 1][1],
                r[2] - trajectory[i - 1][2]);
            if (!(geometric_ds > 1e-5) ||
                std::abs(geometric_ds - ds) > std::max(0.02, 0.20 * ds)) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "%s row %zu has inconsistent s/geometry spacing %.4f vs %.4f",
                    name.c_str(), i, ds, geometric_ds);
                return false;
            }
            const double numeric_ax =
                (r[7] * r[7] - trajectory[i - 1][7] * trajectory[i - 1][7]) /
                (2.0 * ds);
            if (std::abs(numeric_ax - trajectory[i - 1][8]) >
                validation_ax_tolerance_) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "%s ax profile mismatch at row %zu: csv %.3f, numeric %.3f",
                    name.c_str(), i - 1, trajectory[i - 1][8], numeric_ax);
                return false;
            }
        }
        const double abs_kappa = std::abs(r[6]);
        if (abs_kappa > kappa_limit * (1.0 + validation_tolerance_)) {
            if (!validation_advisory_) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "%s row %zu requires |kappa| %.3f > steering limit %.3f",
                    name.c_str(), i, abs_kappa, kappa_limit);
                return false;
            }
            ++steer_violations;
            if (abs_kappa > worst_kappa) {
                worst_kappa = abs_kappa;
                worst_kappa_row = i;
            }
        }
        const double lat_acc = r[7] * r[7] * abs_kappa;
        const double combined_acc = std::hypot(lat_acc, r[8]);
        if (combined_acc > g_limit * (1.0 + validation_tolerance_)) {
            if (!validation_advisory_) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "%s row %zu requires combined tire acceleration %.3f > %.3f "
                    "(lat %.3f, long %.3f)",
                    name.c_str(), i, combined_acc, g_limit, lat_acc, r[8]);
                return false;
            }
            ++friction_violations;
            if (combined_acc > worst_combined) {
                worst_combined = combined_acc;
                worst_combined_row = i;
            }
        }
    }
    const auto [min_it, max_it] =
        std::minmax_element(spacings.begin(), spacings.end());
    if (*max_it > *min_it * 1.25) {
        RCLCPP_ERROR(
            this->get_logger(),
            "%s waypoint spacing is not uniform (min %.4f, max %.4f)",
            name.c_str(), *min_it, *max_it);
        return false;
    }
    const double closure = std::hypot(
        trajectory.front()[1] - trajectory.back()[1],
        trajectory.front()[2] - trajectory.back()[2]);
    if (closure > *max_it * 2.5) {
        RCLCPP_ERROR(
            this->get_logger(), "%s is not a closed loop (closure %.3f m)",
            name.c_str(), closure);
        return false;
    }
    // 형식 검사를 모두 통과한 뒤에만 보고한다. 형식 위반으로 어차피 죽는
    // CSV 에 대해 물리 요약까지 찍으면 진짜 원인이 묻힌다.
    // 위반이 없는 항목은 아예 찍지 않는다. 0행짜리 항목을 "최악 0.000 @ row 0"
    // 으로 같이 출력하면 사람이 그걸 실제 위반으로 읽는다.
    if (friction_violations > 0) {
        RCLCPP_WARN(
            this->get_logger(),
            "%s: [advisory] 마찰원 초과 %zu행을 통과시켰습니다 — 최악 합성 가속도 "
            "%.3f > mu*g %.3f m/s^2 (row %zu). 그 구간에서 타이어가 요구량을 못 내면 "
            "라인 바깥으로 밀려납니다.",
            name.c_str(), friction_violations,
            worst_combined, g_limit, worst_combined_row);
    }
    if (steer_violations > 0) {
        RCLCPP_WARN(
            this->get_logger(),
            "%s: [advisory] 조향 한계 초과 %zu행을 통과시켰습니다 — 최악 |kappa| "
            "%.3f > %.3f 1/m (row %zu, 요구 R %.3f m < 실차 최소 %.3f m). "
            "핸들을 끝까지 꺾어도 못 도는 구간이라 컨트롤러가 포화하고 "
            "라인을 이탈합니다.",
            name.c_str(), steer_violations,
            worst_kappa, kappa_limit, worst_kappa_row,
            1.0 / worst_kappa, 1.0 / kappa_limit);
    }
    return true;
}

bool GlobalTrajectoryNode::ValidateTrajectoryPair(
    const std::vector<std::vector<double>> &main_trajectory,
    const std::vector<std::vector<double>> &overtaking_trajectory) const {
    if (main_trajectory.size() < 2 || overtaking_trajectory.size() < 2) {
        return false;
    }
    const double main_len =
        main_trajectory.back()[0] +
        (main_trajectory[1][0] - main_trajectory[0][0]);
    const double sp_len =
        overtaking_trajectory.back()[0] +
        (overtaking_trajectory[1][0] - overtaking_trajectory[0][0]);
    if (std::abs(main_len - sp_len) > std::max(1.5, 0.08 * main_len)) {
        RCLCPP_ERROR(
            this->get_logger(),
            "IQP/SP track length mismatch: %.3f vs %.3f m", main_len, sp_len);
        return false;
    }
    double max_nearest = 0.0;
    double max_heading_error = 0.0;
    for (const auto &main_row : main_trajectory) {
        double best_sq = std::numeric_limits<double>::max();
        const std::vector<double> *best_sp = nullptr;
        for (const auto &sp_row : overtaking_trajectory) {
            const double dx = sp_row[1] - main_row[1];
            const double dy = sp_row[2] - main_row[2];
            const double dist_sq = dx * dx + dy * dy;
            if (dist_sq < best_sq) {
                best_sq = dist_sq;
                best_sp = &sp_row;
            }
        }
        max_nearest = std::max(max_nearest, std::sqrt(best_sq));
        if (best_sp != nullptr) {
            const double angle_error = std::abs(std::atan2(
                std::sin((*best_sp)[5] - main_row[5]),
                std::cos((*best_sp)[5] - main_row[5])));
            max_heading_error = std::max(max_heading_error, angle_error);
        }
    }
    if (max_nearest > 1.5) {
        RCLCPP_ERROR(
            this->get_logger(),
            "IQP/SP geometry mismatch: max nearest distance %.3f m", max_nearest);
        return false;
    }
    if (max_heading_error > 0.5 * M_PI) {
        RCLCPP_ERROR(
            this->get_logger(),
            "IQP/SP direction mismatch: max nearest heading error %.1f deg",
            max_heading_error * 180.0 / M_PI);
        return false;
    }
    return true;
}

void GlobalTrajectoryNode::BuildSteerSpeedCap(
    const std::vector<std::vector<double>>& trajectory,
    std::vector<double>& out_cap) const {
    out_cap.clear();
    if (!steer_grip_.enabled() || trajectory.size() < 2) {
        return;                     // 비활성 — BuildWpntArray 가 CSV 를 그대로 쓴다
    }
    const size_t n = trajectory.size();
    const double kappa_limit =
        std::tan(validation_max_steer_rad_) /
        std::max(validation_wheelbase_m_, 1e-6);

    // (1) 점별 정적 상한. "그 곡률을 실제로 낼 수 있는 최대 속도".
    // ⚠️ 곡률이 **기하 한계 자체**를 넘는 점은 캡하지 않는다. 그런 점은 어떤
    //    속도로도 못 도는 것이라 감속이 답이 아니고, SpeedCapForKappa 가
    //    0 을 돌려주므로 그대로 쓰면 발행 속도가 0 이 되어 차가 선다.
    //    그건 ValidateTrajectory 가 잡아야 할 CSV 문제다 (strict 모드는 이미
    //    거부하고, advisory 모드는 경고를 남기고 통과시킨다) — 여기서 조용히
    //    정지 궤적으로 바꿔 버리면 그 경고가 무의미해진다.
    out_cap.assign(n, std::numeric_limits<double>::infinity());
    size_t over_geom = 0;
    for (size_t i = 0; i < n; ++i) {
        if (std::abs(trajectory[i][6]) > kappa_limit) {
            ++over_geom;
            continue;                       // 캡 없음 (= 종전 동작)
        }
        out_cap[i] =
            steer_grip_.SpeedCapForKappa(trajectory[i][6], kappa_limit);
    }
    if (over_geom > 0) {
        RCLCPP_WARN(
            this->get_logger(),
            "조향 능력 속도캡: %zu 점이 기하 한계(|kappa| > %.3f)를 넘어 "
            "캡을 적용하지 않았다 — 감속으로 살릴 수 없는 구간이다",
            over_geom, kappa_limit);
    }

    // (2) 가감속 연속성. 캡은 점별로 계단이 지므로 그대로 쓰면 차가 도달할 수
    //     없는 프로파일이 된다. 전/후진 패스로 이어 붙이되, **가감속 한계는
    //     이 CSV 가 이미 요구하고 있던 값**을 쓴다 — 새 물리 가정을 들이지
    //     않기 위해서다 (원본 프로파일이 쓰던 것보다 더 세게 밟지 않는다).
    double a_acc = 0.0, a_dec = 0.0;
    for (size_t i = 0; i < n; ++i) {
        a_acc = std::max(a_acc, trajectory[i][8]);
        a_dec = std::max(a_dec, -trajectory[i][8]);
    }
    a_acc = std::max(a_acc, 0.5);
    a_dec = std::max(a_dec, 0.5);
    // 닫힌 회로라 한 바퀴로는 시작점이 수렴하지 않는다 — 두 바퀴 돈다.
    for (int pass = 0; pass < 2; ++pass) {
        for (size_t k = 1; k <= n; ++k) {          // 전진(가속 한계)
            const size_t i = k % n, p = (k - 1) % n;
            const double ds = std::abs(trajectory[i][0] - trajectory[p][0]);
            if (!(ds > 1e-6) || ds > 1.0) {
                continue;                          // s wrap 구간은 건너뛴다
            }
            out_cap[i] = std::min(
                out_cap[i],
                std::sqrt(out_cap[p] * out_cap[p] + 2.0 * a_acc * ds));
        }
        for (size_t k = n; k >= 1; --k) {          // 후진(제동 한계)
            const size_t i = (k - 1) % n, nx = k % n;
            const double ds = std::abs(trajectory[nx][0] - trajectory[i][0]);
            if (!(ds > 1e-6) || ds > 1.0) {
                continue;
            }
            out_cap[i] = std::min(
                out_cap[i],
                std::sqrt(out_cap[nx] * out_cap[nx] + 2.0 * a_dec * ds));
        }
    }
    // 보고: 이 캡이 실제로 몇 %를 깎는지 로그로 남긴다 (튜닝 근거).
    size_t cut = 0;
    double worst_ratio = 1.0;
    size_t worst_row = 0;
    for (size_t i = 0; i < n; ++i) {
        if (out_cap[i] < trajectory[i][7] - 1e-6) {
            ++cut;
            const double ratio = out_cap[i] / std::max(trajectory[i][7], 1e-6);
            if (ratio < worst_ratio) {
                worst_ratio = ratio;
                worst_row = i;
            }
        }
    }
    RCLCPP_INFO(
        this->get_logger(),
        "조향 능력 속도캡: %zu/%zu 점 감속 (%.1f%%), 최악 s=%.2f "
        "kappa=%.2f  %.2f -> %.2f m/s",
        cut, n, 100.0 * static_cast<double>(cut) / static_cast<double>(n),
        trajectory[worst_row][0], trajectory[worst_row][6],
        trajectory[worst_row][7], out_cap[worst_row]);
}

f1tenth_planning_msgs::msg::WpntArray GlobalTrajectoryNode::BuildWpntArray(
    const std::vector<std::vector<double>>& trajectory) const {
    f1tenth_planning_msgs::msg::WpntArray waypoint_array;
    waypoint_array.header.frame_id = "map";

    // 이 라인에 대응하는 조향 캡 (없으면 캡 없음 = 종전 동작).
    const std::vector<double>* cap = nullptr;
    if (global_steer_cap_.size() == trajectory.size() &&
        &trajectory == &global_trajectory_) {
        cap = &global_steer_cap_;
    } else if (overtaking_steer_cap_.size() == trajectory.size() &&
               &trajectory == &overtaking_trajectory_) {
        cap = &overtaking_steer_cap_;
    }

    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto& row = trajectory[i];

        f1tenth_planning_msgs::msg::Wpnt waypoint;
        waypoint.id = static_cast<int32_t>(i);  // Sequential ID starting from 0
        waypoint.s_m = row[0];     // s_m
        // 이 스택의 Frenet 기준선은 센터라인이 아니라 레이스라인 자신이므로
        // 전역 라인 위의 점은 정의상 d=0 이다. 하류가 이 규약에 의존한다:
        // spliner 는 d_m 을 raceline 기준 오프셋으로 채우고, recovery 는
        // (cur_d_ - w.d_m) 으로 경로 대비 횡이탈을 계산한다.
        waypoint.d_m = 0.0;
        waypoint.x_m = row[1];     // x_m
        waypoint.y_m = row[2];     // y_m
        waypoint.d_right = row[3]; // d_right
        waypoint.d_left = row[4];  // d_left
        waypoint.psi_rad = row[5]; // psi_rad
        waypoint.kappa_radpm = row[6]; // kappa_radpm
        waypoint.vx_mps = row[7] * velocity_scale_;  // vx_mps
        // Spatial speed scaling requires acceleration to scale by q^2.
        waypoint.ax_mps2 = row[8] * velocity_scale_ * velocity_scale_;
        // 조향 능력 캡 (2026-08-23). velocity_scale_ 이 이미 낮으면 캡이 안
        // 걸리므로 저속 시험 프로파일의 거동은 그대로다.
        if (cap != nullptr && (*cap)[i] < waypoint.vx_mps) {
            waypoint.vx_mps = (*cap)[i];
        }

        waypoint_array.wpnts.push_back(waypoint);
    }
    // ax 는 "이 속도 프로파일의 공간 미분"이라는 규약이다 (ValidateTrajectory
    // 가 CSV 에 대해 강제하는 것과 같은 식). 캡으로 vx 를 건드렸으면 여기서
    // 다시 맞춰야 하류(컨트롤러 속도 룩어헤드, recovery)가 어긋나지 않는다.
    const size_t n = waypoint_array.wpnts.size();
    if (cap != nullptr && n >= 2) {
        for (size_t i = 0; i < n; ++i) {
            const size_t j = (i + 1) % n;
            const double ds = trajectory[j][0] - trajectory[i][0];
            if (!(ds > 1e-6)) {
                continue;              // 마지막→처음 wrap 은 원본 ax 유지
            }
            const double v0 = waypoint_array.wpnts[i].vx_mps;
            const double v1 = waypoint_array.wpnts[j].vx_mps;
            waypoint_array.wpnts[i].ax_mps2 = (v1 * v1 - v0 * v0) / (2.0 * ds);
        }
    }
    return waypoint_array;
}

void GlobalTrajectoryNode::PublishGlobalWaypoints() {
    if (global_trajectory_.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 5000,
                            "Global trajectory is empty, cannot publish waypoints");
        return;
    }

    auto waypoint_array = BuildWpntArray(global_trajectory_);
    waypoint_array.header.stamp = this->now();
    p_global_waypoints_->publish(waypoint_array);

    // SP 추월 라인 (로드된 경우에만)
    if (p_overtaking_waypoints_ && !overtaking_trajectory_.empty()) {
        auto ot_array = BuildWpntArray(overtaking_trajectory_);
        ot_array.header.stamp = this->now();
        p_overtaking_waypoints_->publish(ot_array);
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 5000,
                        "Published %zu global waypoints", global_trajectory_.size());
}

int main(int argc, char **argv) {
    std::string node_name = "global_trajectory_node";

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalTrajectoryNode>(node_name));
    rclcpp::shutdown();
    return 0;
}
