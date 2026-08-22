// 원본 planner_pkg에 헤더가 누락되어 있어 cpp 구현부로부터 복원 작성함.
// raceline CSV(9열 세미콜론, build_map.py 산출물)를 로드해
// /global_waypoints (f1tenth_planning_msgs/WpntArray)로 주기 발행하는 노드.
#ifndef PLANNING_PKG__GLOBAL_TRAJECTORY_NODE_HPP_
#define PLANNING_PKG__GLOBAL_TRAJECTORY_NODE_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"

#include "f1tenth_planning_msgs/msg/wpnt.hpp"
#include "f1tenth_planning_msgs/msg/wpnt_array.hpp"

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
    std::vector<std::vector<double>> & out_trajectory,
    std::string & out_generation_id);
  // (strict_curvature 인자 제거: 호출처가 하나뿐이고 항상 리터럴 true 라
  //  "곡률 검사를 건너뛰는 라인이 있다"는 잘못된 인상만 줬다.)
  bool ValidateTrajectory(
    const std::string & name,
    const std::vector<std::vector<double>> & trajectory) const;
  bool ValidateTrajectoryPair(
    const std::vector<std::vector<double>> & main_trajectory,
    const std::vector<std::vector<double>> & overtaking_trajectory) const;
  void PublishGlobalWaypoints();
  f1tenth_planning_msgs::msg::WpntArray BuildWpntArray(
    const std::vector<std::vector<double>> & trajectory) const;

  // Parameters
  std::string trajectory_csv_file_;
  std::string overtaking_csv_file_;   // SP 추월 라인 (빈 문자열이면 발행 안 함)
  double publish_rate_hz_{2.0};
  double velocity_scale_{1.0};
  double target_velocity_scale_{1.0};
  double min_velocity_scale_{0.1};
  double max_velocity_scale_{1.0};
  double velocity_scale_slew_per_sec_{0.1};
  double validation_mu_{0.75};
  double validation_wheelbase_m_{0.33};
  double validation_max_steer_rad_{0.3220};
  double validation_tolerance_{0.03};
  double validation_ax_tolerance_{1.0};
  // validation_mode:=advisory 일 때만 true. 형식 검사(NaN/단조/간격/폐루프/ax↔v)는
  // 그대로 fail 이고, 물리 검사(마찰원·조향 한계)만 경고로 내린다. 손으로 튜닝한
  // <맵>_tuned_ 라인을 실차에서 시험하기 위한 경로이며 기본값은 strict 다.
  bool validation_advisory_{false};

  // CSV rows: [s_m, x_m, y_m, d_right, d_left, psi_rad, kappa_radpm, vx_mps, ax_mps2]
  std::vector<std::vector<double>> global_trajectory_;
  std::vector<std::vector<double>> overtaking_trajectory_;
  std::string main_generation_id_;
  std::string overtaking_generation_id_;

  rclcpp::Publisher<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr p_global_waypoints_;
  rclcpp::Publisher<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr p_overtaking_waypoints_;
  rclcpp::TimerBase::SharedPtr t_publish_waypoints_;
};

#endif  // PLANNING_PKG__GLOBAL_TRAJECTORY_NODE_HPP_
