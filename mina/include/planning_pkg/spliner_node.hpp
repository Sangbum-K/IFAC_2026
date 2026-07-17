// ForzaETH(unicorn-racing-stack) planner/spliner/spliner_node.py의 C++ 포팅.
// 장애물(ObstacleArray) 주변에 pre/post-apex 7점을 잡아 자연 3차 스플라인으로
// d(s) 회피 궤적을 만들고 /planner/avoidance/otwpnts로 발행한다.
// 회피 불가(트랙 경계 근접, 사이드 스위치 불가) 시 빈 배열을 발행하며,
// local_trajectory_node가 이를 로컬 윈도우에 병합(splice)한다.
#ifndef PLANNING_PKG__SPLINER_NODE_HPP_
#define PLANNING_PKG__SPLINER_NODE_HPP_

#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "ae_hyu_msgs/msg/obstacle.hpp"
#include "ae_hyu_msgs/msg/obstacle_array.hpp"
#include "ae_hyu_msgs/msg/wpnt.hpp"
#include "ae_hyu_msgs/msg/wpnt_array.hpp"

class SplinerNode : public rclcpp::Node
{
public:
  explicit SplinerNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void CallbackObstacles(const ae_hyu_msgs::msg::ObstacleArray::SharedPtr msg);
  void CallbackPredictedObstacles(const ae_hyu_msgs::msg::ObstacleArray::SharedPtr msg);
  void CallbackFrenetOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void CallbackGlobalWaypoints(const ae_hyu_msgs::msg::WpntArray::SharedPtr msg);
  void Run();

  // ForzaETH spliner 핵심 로직
  std::vector<ae_hyu_msgs::msg::Obstacle> FilterObstacles(
    const std::vector<ae_hyu_msgs::msg::Obstacle> & obstacles) const;
  ae_hyu_msgs::msg::Obstacle PredictObstacleMovement(
    ae_hyu_msgs::msg::Obstacle obs) const;
  // 반환: {side("left"/"right"), d_apex}
  std::pair<std::string, double> MoreSpace(
    const ae_hyu_msgs::msg::Obstacle & obs, int gb_idx) const;
  // forced_side: ""이면 MoreSpace가 결정, "left"/"right"면 해당 사이드 강제
  bool DoSpline(
    const ae_hyu_msgs::msg::Obstacle & closest_obs,
    ae_hyu_msgs::msg::WpntArray & out_wpnts,
    visualization_msgs::msg::MarkerArray & out_mrks,
    const std::string & forced_side = "");

  // --- Task 2: 이중장애물 재계획 및 안전 정지 ---
  // 후보 회피 경로를 lookahead 내 "모든" 장애물과 (s,d) 충돌 검사.
  // 충돌하는 첫 장애물의 s_start를 blocking_s로 반환.
  bool CheckPathCollision(
    const std::vector<ae_hyu_msgs::msg::Wpnt> & path,
    const std::vector<ae_hyu_msgs::msg::Obstacle> & obstacles,
    double * blocking_s) const;
  // lookahead 내 전체 장애물 (raceline 근접 여부 무관 - 회피 경로 검증용)
  std::vector<ae_hyu_msgs::msg::Obstacle> ObstaclesInLookahead(
    const std::vector<ae_hyu_msgs::msg::Obstacle> & obstacles) const;
  // 안전 후보가 없을 때: 장애물 앞 stop_margin에서 vx=0이 되는 정지 경로 생성
  void BuildStopWpnts(
    double blocking_s,
    ae_hyu_msgs::msg::WpntArray & out_wpnts) const;

  // (s, d) -> (x, y). 균일 간격 global waypoints 기반 선형 보간.
  std::pair<double, double> FrenetToCartesian(double s, double d) const;

  // 자연 3차 스플라인: 절점 (xs, ys)를 지나는 곡선을 query 지점들에서 평가
  static std::vector<double> NaturalCubicSpline(
    const std::vector<double> & xs, const std::vector<double> & ys,
    const std::vector<double> & query);

  visualization_msgs::msg::Marker MakeWpntMarker(
    double x, double y, double v, int id) const;

  // Parameters (ForzaETH dyn_spliner_tuner.cfg 기본값)
  double loop_rate_hz_{20.0};
  double lookahead_m_{10.0};
  double evasion_dist_{0.65};
  double obs_traj_tresh_{0.3};
  double spline_bound_mindist_{0.2};
  std::vector<double> pre_apex_dists_{-4.0, -3.0, -1.5};
  std::vector<double> post_apex_dists_{2.0, 3.0, 4.0};
  double fixed_pred_time_{0.15};
  double outside_spline_scale_{1.75};
  double inside_speed_scale_{0.9};
  double spline_resolution_{0.1};
  double side_switch_d_tresh_{0.25};
  double obs_timeout_sec_{0.5};
  bool use_prediction_{true};   // Predictive Spliner: 예측 장애물이 원시 관측을 대체
  // Task 2 params
  double path_check_margin_{0.15};   // 경로-장애물 충돌 판정 횡방향 여유 [m]
  double stop_margin_m_{1.0};        // 정지 시 장애물 앞 안전거리 [m]
  double stop_ramp_dist_m_{2.5};     // vx를 0까지 줄이는 감속 구간 길이 [m]

  // State
  ae_hyu_msgs::msg::ObstacleArray obstacles_;
  rclcpp::Time obstacles_stamp_{0, 0, RCL_ROS_TIME};
  ae_hyu_msgs::msg::ObstacleArray predicted_obstacles_;
  rclcpp::Time predicted_stamp_{0, 0, RCL_ROS_TIME};
  std::vector<ae_hyu_msgs::msg::Wpnt> gb_wpnts_;
  double gb_vmax_{0.0};
  double gb_max_s_{0.0};
  double wpnt_dist_{0.1};
  double cur_s_{0.0};
  double cur_d_{0.0};
  double cur_vs_{0.0};
  bool has_obstacles_{false};
  bool has_gb_wpnts_{false};
  bool has_frenet_odom_{false};
  std::string last_ot_side_{""};
  std::mutex mutex_obstacles_;
  std::mutex mutex_gb_wpnts_;
  std::mutex mutex_frenet_odom_;

  rclcpp::Subscription<ae_hyu_msgs::msg::ObstacleArray>::SharedPtr s_obstacles_;
  rclcpp::Subscription<ae_hyu_msgs::msg::ObstacleArray>::SharedPtr s_predicted_obstacles_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr s_frenet_odom_;
  rclcpp::Subscription<ae_hyu_msgs::msg::WpntArray>::SharedPtr s_global_waypoints_;
  rclcpp::Publisher<ae_hyu_msgs::msg::WpntArray>::SharedPtr p_evasion_wpnts_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr p_markers_;
  rclcpp::TimerBase::SharedPtr t_run_;
};

#endif  // PLANNING_PKG__SPLINER_NODE_HPP_
