// 원본 planner_pkg에 헤더가 누락되어 있어 cpp 구현부로부터 복원 작성함.
// /global_waypoints(빨강), /local_waypoints(초록), centerline CSV(파랑)를
// RViz MarkerArray로 발행. (IQP/속도/3D벽 시각화는 rclpy 노드
// raceline_viz_node.py가 별도로 담당 — 둘을 같이 띄워도 됨)
#ifndef PLANNING_PKG__PLANNER_VISUALIZER_NODE_HPP_
#define PLANNING_PKG__PLANNER_VISUALIZER_NODE_HPP_

#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "f1tenth_planning_msgs/msg/wpnt_array.hpp"

class PlannerVisualizerNode : public rclcpp::Node
{
public:
  explicit PlannerVisualizerNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PlannerVisualizerNode() override;

private:
  void CallbackGlobalWaypoints(const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg);
  void CallbackLocalWaypoints(const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg);
  void Run();
  bool LoadCenterlineFromCSV(const std::string & csv_filename);
  void PublishCenterlineMarkers();
  void PublishGlobalWaypointsMarkers();
  void PublishLocalWaypointsMarkers();

  // Parameters
  double loop_rate_hz_{10.0};
  std::string centerline_csv_file_;

  // State
  std::vector<std::vector<double>> centerline_data_;  // rows: [x_m, y_m, ...]
  f1tenth_planning_msgs::msg::WpntArray global_waypoints_;
  bool b_is_global_waypoints_{false};
  f1tenth_planning_msgs::msg::WpntArray local_waypoints_;
  bool b_is_local_waypoints_{false};
  std::mutex mutex_global_waypoints_;
  std::mutex mutex_local_waypoints_;

  rclcpp::Subscription<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr s_global_waypoints_;
  rclcpp::Subscription<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr s_local_waypoints_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr p_centerline_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr p_global_waypoints_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr p_local_waypoints_markers_;
  rclcpp::TimerBase::SharedPtr t_run_node_;
};

#endif  // PLANNING_PKG__PLANNER_VISUALIZER_NODE_HPP_
