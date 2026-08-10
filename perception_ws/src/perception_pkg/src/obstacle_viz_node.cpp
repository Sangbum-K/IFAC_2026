/*
 * 장애물 인지 결과 시각화 — ObstacleArray -> MarkerArray.
 *
 *   /perception/obstacles (f1tenth_perception_msgs/ObstacleArray)
 *     -> /perception/obstacle_markers (visualization_msgs/MarkerArray)
 *
 * monitor_pkg의 monitor.rviz "Obstacles" 디스플레이가 이 토픽을 본다. 실차에서
 * obstacle_detector_node는 잘 돌면서 마커 발행자가 없어 "회피 라인은 휘는데
 * 무엇을 피하는지 안 보이는" 상태였는데, 그걸 메꾸는 노드다.
 *
 * 표시:
 *   - 정적(is_dynamic=false): 초록 원기둥. 트랙 ID 텍스트 `#3`.
 *   - 동적(is_dynamic=true):  빨강 원기둥 + 노랑 속도 화살표. 텍스트 `#3 D 1.2m/s`.
 *   - confidence가 낮으면(코스팅 중) 반투명해진다.
 *   - 사라진 트랙은 DELETE 마커로 지운다 (안 지우면 RViz에 유령이 남는다).
 *
 * 원기둥 지름은 Obstacle.radius(클러스터 반경 추정치)의 2배라 화면상 크기가
 * 실제 인지 크기다 — 회피 라인이 왜 그만큼 벌어졌는지 눈으로 대조할 수 있다.
 *
 * 프레임은 입력 메시지의 header를 그대로 쓴다. detector의 target_frame이 map이면
 * 마커도 map에 고정되므로, 정적 장애물은 차가 지나가도 제자리에 남는다.
 */
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <set>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "f1tenth_perception_msgs/msg/obstacle_array.hpp"

namespace perception_pkg
{
namespace
{
// 마커 네임스페이스. DELETE도 같은 (ns, id)로 보내야 지워진다.
constexpr const char * kNsBody = "obstacle";
constexpr const char * kNsVel = "velocity";
constexpr const char * kNsText = "id";
const std::array<const char *, 3> kAllNs{kNsBody, kNsVel, kNsText};
}  // namespace

class ObstacleViz : public rclcpp::Node
{
public:
  ObstacleViz()
  : Node("obstacle_viz_node")
  {
    this->declare_parameter<std::string>("viz/input_topic", "/perception/obstacles");
    this->declare_parameter<std::string>("viz/output_topic", "/perception/obstacle_markers");
    // 마커 수명. 0이면 무한 — 발행이 끊겨도 화면에 남는다. 인지가 죽은 걸
    // 알아채야 하므로 기본은 유한값으로 두고 매 프레임 갱신한다.
    this->declare_parameter<double>("viz/marker_lifetime_sec", 0.5);
    this->declare_parameter<double>("viz/body_height_m", 0.3);
    this->declare_parameter<double>("viz/text_size_m", 0.25);
    this->declare_parameter<double>("viz/min_diameter_m", 0.05);

    const auto input_topic = this->get_parameter("viz/input_topic").as_string();
    const auto output_topic = this->get_parameter("viz/output_topic").as_string();
    lifetime_sec_ = this->get_parameter("viz/marker_lifetime_sec").as_double();
    body_height_ = this->get_parameter("viz/body_height_m").as_double();
    text_size_ = this->get_parameter("viz/text_size_m").as_double();
    min_diameter_ = this->get_parameter("viz/min_diameter_m").as_double();

    sub_ = this->create_subscription<f1tenth_perception_msgs::msg::ObstacleArray>(
      input_topic, 10, std::bind(&ObstacleViz::on_obstacles, this, std::placeholders::_1));
    pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(output_topic, 10);

    RCLCPP_INFO(
      this->get_logger(), "장애물 시각화 시작: %s -> %s",
      input_topic.c_str(), output_topic.c_str());
  }

private:
  void set_lifetime(visualization_msgs::msg::Marker & marker) const
  {
    const int sec = static_cast<int>(lifetime_sec_);
    marker.lifetime.sec = sec;
    marker.lifetime.nanosec =
      static_cast<uint32_t>((lifetime_sec_ - static_cast<double>(sec)) * 1e9);
  }

  void on_obstacles(f1tenth_perception_msgs::msg::ObstacleArray::ConstSharedPtr msg)
  {
    visualization_msgs::msg::MarkerArray markers;
    std::set<int> cur_ids;

    for (const auto & obs : msg->obstacles) {
      cur_ids.insert(obs.id);
      // 동적=빨강, 정적=초록. 코스팅(외삽) 중이면 confidence가 떨어지므로
      // 알파에 그대로 반영해 "지금 보고 있는 것"과 구분한다.
      const float r = obs.is_dynamic ? 1.0f : 0.1f;
      const float g = obs.is_dynamic ? 0.1f : 0.9f;
      const float b = obs.is_dynamic ? 0.1f : 0.2f;
      const float alpha =
        0.25f + 0.55f * std::max(0.0f, std::min(1.0f, obs.confidence));

      visualization_msgs::msg::Marker body;
      body.header = msg->header;
      body.ns = kNsBody;
      body.id = obs.id;
      body.type = visualization_msgs::msg::Marker::CYLINDER;
      body.action = visualization_msgs::msg::Marker::ADD;
      body.pose.position = obs.position;
      body.pose.position.z = body_height_ / 2.0;
      body.pose.orientation.w = 1.0;
      const double diameter = std::max(min_diameter_, 2.0 * static_cast<double>(obs.radius));
      body.scale.x = diameter;
      body.scale.y = diameter;
      body.scale.z = body_height_;
      body.color.r = r;
      body.color.g = g;
      body.color.b = b;
      body.color.a = alpha;
      set_lifetime(body);
      markers.markers.push_back(body);

      const double speed = std::hypot(obs.velocity.x, obs.velocity.y);

      // 속도 화살표는 동적일 때만. 정적 장애물의 잔여 속도(수 cm/s)까지 그리면
      // 화면이 지저분하고 "정적인데 왜 화살표?"로 오해를 부른다.
      if (obs.is_dynamic) {
        visualization_msgs::msg::Marker arrow;
        arrow.header = msg->header;
        arrow.ns = kNsVel;
        arrow.id = obs.id;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.pose.orientation.w = 1.0;
        arrow.scale.x = 0.05;   // shaft 지름
        arrow.scale.y = 0.1;    // head 지름
        arrow.scale.z = 0.0;
        arrow.color.r = 1.0f;
        arrow.color.g = 1.0f;
        arrow.color.b = 0.0f;
        arrow.color.a = 0.9f;
        geometry_msgs::msg::Point tail;
        tail.x = obs.position.x;
        tail.y = obs.position.y;
        tail.z = 0.05;
        geometry_msgs::msg::Point head;
        head.x = obs.position.x + obs.velocity.x;
        head.y = obs.position.y + obs.velocity.y;
        head.z = 0.05;
        arrow.points.push_back(tail);
        arrow.points.push_back(head);
        set_lifetime(arrow);
        markers.markers.push_back(arrow);
      }

      visualization_msgs::msg::Marker text;
      text.header = msg->header;
      text.ns = kNsText;
      text.id = obs.id;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = obs.position.x;
      text.pose.position.y = obs.position.y;
      text.pose.position.z = body_height_ + 0.15;
      text.pose.orientation.w = 1.0;
      text.scale.z = text_size_;
      text.color.r = 1.0f;
      text.color.g = 1.0f;
      text.color.b = 1.0f;
      text.color.a = 1.0f;
      char buf[64];
      if (obs.is_dynamic) {
        std::snprintf(buf, sizeof(buf), "#%d D %.1fm/s", obs.id, speed);
      } else {
        std::snprintf(buf, sizeof(buf), "#%d", obs.id);
      }
      text.text = buf;
      set_lifetime(text);
      markers.markers.push_back(text);
    }

    // 사라진 트랙 마커 삭제 (lifetime만 믿으면 ID 재사용 시 유령이 남는다)
    for (const int old_id : prev_ids_) {
      if (cur_ids.count(old_id)) {
        continue;
      }
      for (const char * ns : kAllNs) {
        visualization_msgs::msg::Marker gone;
        gone.header = msg->header;
        gone.ns = ns;
        gone.id = old_id;
        gone.action = visualization_msgs::msg::Marker::DELETE;
        markers.markers.push_back(gone);
      }
    }
    prev_ids_ = std::move(cur_ids);

    pub_->publish(markers);
  }

  double lifetime_sec_{0.5};
  double body_height_{0.3};
  double text_size_{0.25};
  double min_diameter_{0.05};
  std::set<int> prev_ids_;

  rclcpp::Subscription<f1tenth_perception_msgs::msg::ObstacleArray>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
};

}  // namespace perception_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_pkg::ObstacleViz>());
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
