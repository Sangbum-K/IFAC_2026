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
        "my_track_iqp_max6.00ms_margin0.08m_mu0.75.csv");
    this->declare_parameter("global_trajectory/overtaking_csv_file", "");
    this->declare_parameter("global_trajectory/publish_rate_hz", 2.0);
    this->declare_parameter("global_trajectory/velocity_scale", 1.0);
    this->declare_parameter("global_trajectory/min_velocity_scale", 0.1);
    this->declare_parameter("global_trajectory/max_velocity_scale", 1.0);
    this->declare_parameter("global_trajectory/velocity_scale_slew_per_sec", 0.1);
    this->declare_parameter("global_trajectory/validation_mu", 0.75);
    this->declare_parameter("global_trajectory/validation_wheelbase_m", 0.33);
    this->declare_parameter("global_trajectory/validation_max_steer_rad", 0.36);
    this->declare_parameter("global_trajectory/validation_tolerance", 0.03);
    this->declare_parameter("global_trajectory/validation_ax_tolerance", 1.0);

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

    // Load global trajectory from CSV
    if (LoadTrajectoryFromCSV(
            trajectory_csv_file_, global_trajectory_, main_generation_id_)) {
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
        if (!ValidateTrajectory(csv_filename, out_trajectory, true)) {
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
    const std::vector<std::vector<double>> &trajectory,
    bool strict_curvature) const {
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
        if (strict_curvature &&
            abs_kappa > kappa_limit * (1.0 + validation_tolerance_)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "%s row %zu requires |kappa| %.3f > steering limit %.3f",
                name.c_str(), i, abs_kappa, kappa_limit);
            return false;
        }
        const double lat_acc = r[7] * r[7] * abs_kappa;
        const double combined_acc = std::hypot(lat_acc, r[8]);
        if (combined_acc > g_limit * (1.0 + validation_tolerance_)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "%s row %zu requires combined tire acceleration %.3f > %.3f "
                "(lat %.3f, long %.3f)",
                name.c_str(), i, combined_acc, g_limit, lat_acc, r[8]);
            return false;
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

f1tenth_planning_msgs::msg::WpntArray GlobalTrajectoryNode::BuildWpntArray(
    const std::vector<std::vector<double>>& trajectory) const {
    f1tenth_planning_msgs::msg::WpntArray waypoint_array;
    waypoint_array.header.frame_id = "map";

    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto& row = trajectory[i];

        f1tenth_planning_msgs::msg::Wpnt waypoint;
        waypoint.id = static_cast<int32_t>(i);  // Sequential ID starting from 0
        waypoint.s_m = row[0];     // s_m
        waypoint.d_m = 0.0;        // d_m (set to 0 as requested)
        waypoint.x_m = row[1];     // x_m
        waypoint.y_m = row[2];     // y_m
        waypoint.d_right = row[3]; // d_right
        waypoint.d_left = row[4];  // d_left
        waypoint.psi_rad = row[5]; // psi_rad
        waypoint.kappa_radpm = row[6]; // kappa_radpm
        waypoint.vx_mps = row[7] * velocity_scale_;  // vx_mps
        // Spatial speed scaling requires acceleration to scale by q^2.
        waypoint.ax_mps2 = row[8] * velocity_scale_ * velocity_scale_;

        waypoint_array.wpnts.push_back(waypoint);
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
