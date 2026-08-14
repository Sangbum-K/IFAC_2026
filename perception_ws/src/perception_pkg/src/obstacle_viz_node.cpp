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
 *   - 미관측(is_visible=false): 청록 `KF COAST` 고스트. 실측과 외삽을 분리한다.
 *   - 동적 트랙은 현재 KF 상태에서 prediction_horizon_sec 뒤의 예상점/선을 표시한다.
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
#include <stdexcept>
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
constexpr const char * kNsCoast = "kf_coast";
constexpr const char * kNsFuture = "kf_future";
constexpr const char * kNsFuturePath = "kf_future_path";
const std::array<const char *, 6> kAllNs{
  kNsBody, kNsVel, kNsText, kNsCoast, kNsFuture, kNsFuturePath};
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
    // 현재 KF posterior에서 앞으로 얼마나 외삽해 예상점을 그릴지. 0이면 비활성.
    this->declare_parameter<double>("viz/prediction_horizon_sec", 0.30);
    this->declare_parameter<double>("viz/prediction_marker_diameter_m", 0.16);

    const auto input_topic = this->get_parameter("viz/input_topic").as_string();
    const auto output_topic = this->get_parameter("viz/output_topic").as_string();
    lifetime_sec_ = this->get_parameter("viz/marker_lifetime_sec").as_double();
    body_height_ = this->get_parameter("viz/body_height_m").as_double();
    text_size_ = this->get_parameter("viz/text_size_m").as_double();
    min_diameter_ = this->get_parameter("viz/min_diameter_m").as_double();
    prediction_horizon_sec_ = this->get_parameter("viz/prediction_horizon_sec").as_double();
    prediction_marker_diameter_ =
      this->get_parameter("viz/prediction_marker_diameter_m").as_double();
    if (!std::isfinite(prediction_horizon_sec_) || prediction_horizon_sec_ < 0.0 ||
      prediction_horizon_sec_ > 5.0 || !std::isfinite(prediction_marker_diameter_) ||
      prediction_marker_diameter_ <= 0.0 || prediction_marker_diameter_ > 5.0)
    {
      throw std::invalid_argument(
              "invalid viz/prediction_horizon_sec or viz/prediction_marker_diameter_m");
    }

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
    const auto delete_marker =
      [&](const char * ns, int id) {
        visualization_msgs::msg::Marker gone;
        gone.header = msg->header;
        gone.ns = ns;
        gone.id = id;
        gone.action = visualization_msgs::msg::Marker::DELETE;
        markers.markers.push_back(gone);
      };

    for (const auto & obs : msg->obstacles) {
      cur_ids.insert(obs.id);
      const double diameter = std::max(min_diameter_, 2.0 * static_cast<double>(obs.radius));
      const double speed = std::hypot(obs.velocity.x, obs.velocity.y);

      if (obs.is_visible) {
        // 이번 프레임 실측이 들어온 트랙: 기존 실체 마커를 선명하게 그린다.
        const float r = obs.is_dynamic ? 1.0f : 0.1f;
        const float g = obs.is_dynamic ? 0.1f : 0.9f;
        const float b = obs.is_dynamic ? 0.1f : 0.2f;
        const float alpha =
          0.35f + 0.55f * std::max(0.0f, std::min(1.0f, obs.confidence));

        visualization_msgs::msg::Marker body;
        body.header = msg->header;
        body.ns = kNsBody;
        body.id = obs.id;
        body.type = visualization_msgs::msg::Marker::CYLINDER;
        body.action = visualization_msgs::msg::Marker::ADD;
        body.pose.position = obs.position;
        body.pose.position.z = body_height_ / 2.0;
        body.pose.orientation.w = 1.0;
        body.scale.x = diameter;
        body.scale.y = diameter;
        body.scale.z = body_height_;
        body.color.r = r;
        body.color.g = g;
        body.color.b = b;
        body.color.a = alpha;
        set_lifetime(body);
        markers.markers.push_back(body);
        delete_marker(kNsCoast, obs.id);
      } else {
        // 실제 관측이 끊긴 경우에는 실체 원기둥을 유지하지 않는다. KF posterior
        // 위치에만 청록 고스트를 그려 실측과 외삽을 즉시 구분하게 한다.
        delete_marker(kNsBody, obs.id);
        visualization_msgs::msg::Marker coast;
        coast.header = msg->header;
        coast.ns = kNsCoast;
        coast.id = obs.id;
        coast.type = visualization_msgs::msg::Marker::SPHERE;
        coast.action = visualization_msgs::msg::Marker::ADD;
        coast.pose.position = obs.position;
        coast.pose.position.z = body_height_ / 2.0;
        coast.pose.orientation.w = 1.0;
        coast.scale.x = 1.15 * diameter;
        coast.scale.y = 1.15 * diameter;
        coast.scale.z = body_height_;
        coast.color.r = 0.0f;
        coast.color.g = 0.85f;
        coast.color.b = 1.0f;
        coast.color.a = 0.20f + 0.35f * std::max(
          0.0f, std::min(1.0f, obs.confidence));
        set_lifetime(coast);
        markers.markers.push_back(coast);
      }

      // 속도 화살표는 실측 동적 트랙에만 그린다. 코스팅 때는 아래의 미래 예상
      // 선이 대신하므로, 오래된 속도가 "실제 화살표"처럼 보이지 않는다.
      if (obs.is_dynamic && obs.is_visible) {
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
      } else {
        delete_marker(kNsVel, obs.id);
      }

      // 현재 KF posterior에서 정해진 horizon만큼 CV 외삽한 예상점과 연결선.
      // is_visible=false이면 시작점 자체가 KF coast 위치다.
      if (obs.is_dynamic && prediction_horizon_sec_ > 0.0) {
        geometry_msgs::msg::Point now;
        now.x = obs.position.x;
        now.y = obs.position.y;
        now.z = 0.08;
        geometry_msgs::msg::Point future = now;
        future.x += obs.velocity.x * prediction_horizon_sec_;
        future.y += obs.velocity.y * prediction_horizon_sec_;

        visualization_msgs::msg::Marker future_path;
        future_path.header = msg->header;
        future_path.ns = kNsFuturePath;
        future_path.id = obs.id;
        future_path.type = visualization_msgs::msg::Marker::LINE_STRIP;
        future_path.action = visualization_msgs::msg::Marker::ADD;
        future_path.pose.orientation.w = 1.0;
        future_path.scale.x = 0.035;
        future_path.color.r = 1.0f;
        future_path.color.g = 0.45f;
        future_path.color.b = 0.0f;
        future_path.color.a = obs.is_visible ? 0.85f : 0.50f;
        future_path.points.push_back(now);
        future_path.points.push_back(future);
        set_lifetime(future_path);
        markers.markers.push_back(future_path);

        visualization_msgs::msg::Marker future_marker;
        future_marker.header = msg->header;
        future_marker.ns = kNsFuture;
        future_marker.id = obs.id;
        future_marker.type = visualization_msgs::msg::Marker::SPHERE;
        future_marker.action = visualization_msgs::msg::Marker::ADD;
        future_marker.pose.position = future;
        future_marker.pose.orientation.w = 1.0;
        future_marker.scale.x = prediction_marker_diameter_;
        future_marker.scale.y = prediction_marker_diameter_;
        future_marker.scale.z = prediction_marker_diameter_;
        future_marker.color.r = 1.0f;
        future_marker.color.g = 0.45f;
        future_marker.color.b = 0.0f;
        future_marker.color.a = obs.is_visible ? 0.85f : 0.50f;
        set_lifetime(future_marker);
        markers.markers.push_back(future_marker);
      } else {
        delete_marker(kNsFuturePath, obs.id);
        delete_marker(kNsFuture, obs.id);
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
      if (!obs.is_visible) {
        std::snprintf(buf, sizeof(buf), "#%d KF COAST %.1fm/s", obs.id, speed);
      } else if (obs.is_dynamic) {
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
  double prediction_horizon_sec_{0.30};
  double prediction_marker_diameter_{0.16};
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
