// RViz Pitwall 패널 (unicorn/HMCL pitwall-panel 대응).
//
// 표시: 주행 STATE 배너 (GB_TRACK/TRAILING/OVERTAKE/RECOVERY/MANUAL),
//       /pitwall/events 이벤트 로그, ego/상대 상태 라벨.
// 제어: ego Autonomous/Manual, 상대 모드(Path/FTG/Manual)·속도±0.5·제거,
//       정적 장애물 전체 삭제, ego/상대 LiDAR 토글, perception 주입 모드.
//
// ROS 콜백은 mutex로 보호된 상태에 쓰고, QTimer(4 Hz)가 UI에 반영한다
// (Qt 위젯은 GUI 스레드에서만 만질 수 있음).
#ifndef PITWALL_PANEL__PITWALL_PANEL_HPP_
#define PITWALL_PANEL__PITWALL_PANEL_HPP_

#include <deque>
#include <mutex>
#include <string>

#include <QLabel>
#include <QListWidget>
#include <QComboBox>
#include <QCheckBox>
#include <QTimer>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "f1tenth_planning_msgs/msg/wpnt_array.hpp"
#include "f1tenth_planning_msgs/msg/obstacle_array.hpp"

namespace pitwall_panel
{

class PitwallPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit PitwallPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void RefreshUi();

private:
  void PublishString(
    const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & pub,
    const std::string & data);

  // --- UI ---
  QLabel * state_banner_{nullptr};
  QLabel * ego_status_label_{nullptr};
  QLabel * opp_status_label_{nullptr};
  QComboBox * opp_mode_combo_{nullptr};
  QComboBox * perception_combo_{nullptr};
  QCheckBox * ego_lidar_check_{nullptr};
  QCheckBox * opp_lidar_check_{nullptr};
  QListWidget * event_list_{nullptr};
  QTimer * refresh_timer_{nullptr};

  // --- ROS ---
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_ego_mode_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_opp_mode_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_opp_delta_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_remove_opp_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_clear_obs_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ego_lidar_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_opp_lidar_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_perception_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_events_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_ego_status_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_opp_status_;
  rclcpp::Subscription<f1tenth_planning_msgs::msg::WpntArray>::SharedPtr sub_otwpnts_;
  rclcpp::Subscription<f1tenth_planning_msgs::msg::ObstacleArray>::SharedPtr sub_pred_;

  // --- ROS 콜백 <-> UI 공유 상태 ---
  std::mutex mutex_;
  std::deque<std::string> pending_events_;
  std::string ego_status_;
  std::string opp_status_;
  rclcpp::Time last_otwpnts_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pred_{0, 0, RCL_ROS_TIME};
  bool in_recovery_{false};
};

}  // namespace pitwall_panel

#endif  // PITWALL_PANEL__PITWALL_PANEL_HPP_
