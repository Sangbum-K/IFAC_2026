// 원본 planner_pkg에 헤더가 누락되어 있어 cpp 구현부로부터 복원 작성함.
// raceline CSV(9열 세미콜론, step4_main.py 산출물)를 로드해
// /global_waypoints (ae_hyu_msgs/WpntArray)로 주기 발행하는 노드.
#ifndef PLANNING_PKG__GLOBAL_TRAJECTORY_NODE_HPP_
#define PLANNING_PKG__GLOBAL_TRAJECTORY_NODE_HPP_

#include <algorithm>
#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ae_hyu_msgs/msg/wpnt.hpp"
#include "ae_hyu_msgs/msg/wpnt_array.hpp"

class GlobalTrajectoryNode : public rclcpp::Node
{
public:
  explicit GlobalTrajectoryNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GlobalTrajectoryNode() override;

private:
  void ProcessParams();
  void Run();
  bool LoadTrajectoryFromCSV(
    const std::string & csv_filename,
    std::vector<std::vector<double>> & out_trajectory);
  void PublishGlobalWaypoints();
  ae_hyu_msgs::msg::WpntArray BuildWpntArray(
    const std::vector<std::vector<double>> & trajectory) const;

  // Parameters
  std::string trajectory_csv_file_;
  std::string overtaking_csv_file_;   // SP 추월 라인 (빈 문자열이면 발행 안 함)
  double publish_rate_hz_{2.0};
  double velocity_scale_{1.0};   // 주행 중 ros2 param set으로 조정 (0.5~1.4 클램프)

  // CSV rows: [s_m, x_m, y_m, d_right, d_left, psi_rad, kappa_radpm, vx_mps, ax_mps2]
  std::vector<std::vector<double>> global_trajectory_;
  std::vector<std::vector<double>> overtaking_trajectory_;

  rclcpp::Publisher<ae_hyu_msgs::msg::WpntArray>::SharedPtr p_global_waypoints_;
  rclcpp::Publisher<ae_hyu_msgs::msg::WpntArray>::SharedPtr p_overtaking_waypoints_;
  rclcpp::TimerBase::SharedPtr t_publish_waypoints_;
};

#endif  // PLANNING_PKG__GLOBAL_TRAJECTORY_NODE_HPP_
