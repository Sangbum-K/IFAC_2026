#include "planning_pkg/global_trajectory_node.hpp"

GlobalTrajectoryNode::GlobalTrajectoryNode(const std::string &node_name, const rclcpp::NodeOptions &options)
    : Node(node_name, options) {

    // QoS init
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Parameters - declare and get values
    this->declare_parameter("global_trajectory/trajectory_csv_file", "Spielberg_map_iqp_max7.00ms_margin0.15m_mu0.85.csv");
    this->declare_parameter("global_trajectory/overtaking_csv_file", "");
    this->declare_parameter("global_trajectory/publish_rate_hz", 2.0);
    this->declare_parameter("global_trajectory/velocity_scale", 1.0);

    trajectory_csv_file_ = this->get_parameter("global_trajectory/trajectory_csv_file").as_string();
    overtaking_csv_file_ = this->get_parameter("global_trajectory/overtaking_csv_file").as_string();
    publish_rate_hz_ = this->get_parameter("global_trajectory/publish_rate_hz").as_double();
    velocity_scale_ = this->get_parameter("global_trajectory/velocity_scale").as_double();
    velocity_scale_ = std::clamp(velocity_scale_, 0.5, 1.4);

    RCLCPP_INFO(this->get_logger(), "trajectory_csv_file: %s", trajectory_csv_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "publish_rate_hz: %f", publish_rate_hz_);
    RCLCPP_INFO(this->get_logger(), "velocity_scale: %f", velocity_scale_);

    // Load global trajectory from CSV
    if (LoadTrajectoryFromCSV(trajectory_csv_file_, global_trajectory_)) {
        RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu waypoints from CSV", global_trajectory_.size());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to load trajectory from CSV file");
        return;
    }

    // SP 추월 라인 (H2H lane-change용, unicorn의 IQP+SP 이중 라인 구조).
    // 파일이 없으면 경고만 남기고 메인 라인만 발행한다.
    if (!overtaking_csv_file_.empty()) {
        if (LoadTrajectoryFromCSV(overtaking_csv_file_, overtaking_trajectory_)) {
            RCLCPP_INFO(this->get_logger(), "Loaded overtaking (SP) line: %zu waypoints (%s)",
                        overtaking_trajectory_.size(), overtaking_csv_file_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Overtaking CSV not loadable (%s) — 메인 라인만 발행",
                        overtaking_csv_file_.c_str());
            overtaking_trajectory_.clear();
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
        std::chrono::milliseconds((int64_t)(1000 / publish_rate_hz_)),
        [this]() { this->Run(); });

    RCLCPP_INFO(this->get_logger(), "GlobalTrajectoryNode node initialized successfully");
}

GlobalTrajectoryNode::~GlobalTrajectoryNode() {}

void GlobalTrajectoryNode::ProcessParams() {
    this->get_parameter("global_trajectory/velocity_scale", velocity_scale_);

    // 안전을 위한 클리핑 수행
    velocity_scale_ = std::clamp(velocity_scale_, 0.5, 1.4);
}

void GlobalTrajectoryNode::Run() {

    // 동적으로 velocity 스케일링 파라미터 적용
    this->ProcessParams();

    // Publish global waypoints
    this->PublishGlobalWaypoints();
}

bool GlobalTrajectoryNode::LoadTrajectoryFromCSV(const std::string& csv_filename,
                                                 std::vector<std::vector<double>>& out_trajectory) {
    try {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("planning_pkg");
        std::string csv_path = package_share_directory + "/config/global_trajectory/" + csv_filename;

        RCLCPP_INFO(this->get_logger(), "Loading trajectory from: %s", csv_path.c_str());

        std::ifstream file(csv_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open CSV file: %s", csv_path.c_str());
            return false;
        }

        out_trajectory.clear();
        std::string line;

        while (std::getline(file, line)) {
            // Skip empty lines and comment lines (starting with '#')
            if (line.empty() || line[0] == '#') {
                continue;
            }

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

                    double value = std::stod(cell);
                    row.push_back(value);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error parsing CSV value: '%s' - %s", cell.c_str(), e.what());
                    return false;
                }
            }

            // Expect 9 columns: s_m, x_m, y_m, d_right, d_left, psi_rad, kappa_radpm, vx_mps, ax_mps2
            if (row.size() == 9) {
                out_trajectory.push_back(row);
            } else if (row.size() > 0) {
                RCLCPP_WARN(this->get_logger(), "Skipping CSV line with %zu columns (expected 9): %s",
                           row.size(), line.c_str());
            }
        }

        file.close();

        if (out_trajectory.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No valid waypoints found in CSV file");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu waypoints", out_trajectory.size());
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while loading CSV: %s", e.what());
        return false;
    }
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
        waypoint.ax_mps2 = row[8]; // ax_mps2

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
