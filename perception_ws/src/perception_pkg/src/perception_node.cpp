#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode()
  : Node("perception")
  {
    detection_threshold_ =
      this->declare_parameter<double>("detection_threshold", 0.5);
    const auto publish_topic =
      this->declare_parameter<std::string>("publish_topic", "/perception/objects");
    const auto publish_period =
      this->declare_parameter<double>("publish_period", 1.0);

    pub_ = this->create_publisher<std_msgs::msg::String>(publish_topic, 10);
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(publish_period),
      std::bind(&PerceptionNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Perception node started");
  }

private:
  void timer_callback()
  {
    std_msgs::msg::String msg;
    msg.data = "object_detected:confidence=" + std::to_string(detection_threshold_);
    pub_->publish(msg);
    RCLCPP_DEBUG(this->get_logger(), "Published: %s", msg.data.c_str());
  }

  double detection_threshold_{0.5};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
