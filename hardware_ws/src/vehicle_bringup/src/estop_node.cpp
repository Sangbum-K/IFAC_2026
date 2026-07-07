#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

class EStopNode : public rclcpp::Node
{
public:
  EStopNode()
  : Node("estop_node")
  {
    const auto joy_topic =
      this->declare_parameter<std::string>("joy_topic", "/joy");
    const auto estop_topic =
      this->declare_parameter<std::string>("estop_topic", "/safety/emergency_stop");
    button_index_ = this->declare_parameter<int>("button_index", 4);
    latch_estop_ = this->declare_parameter<bool>("latch_estop", true);

    publisher_ = this->create_publisher<std_msgs::msg::Bool>(estop_topic, 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic,
      10,
      std::bind(&EStopNode::joy_cb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Emergency stop node started");
  }

private:
  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (button_index_ < 0 || static_cast<size_t>(button_index_) >= msg->buttons.size()) {
      RCLCPP_WARN(this->get_logger(), "Configured e-stop button index is out of range");
      return;
    }

    const bool button_pressed = msg->buttons[button_index_] == 1;
    if (!button_pressed) {
      return;
    }

    if (latch_estop_) {
      estop_active_ = true;
    } else {
      estop_active_ = !estop_active_;
    }

    std_msgs::msg::Bool estop_msg;
    estop_msg.data = estop_active_;
    publisher_->publish(estop_msg);
    RCLCPP_WARN(this->get_logger(), "E-stop state changed: %s", estop_active_ ? "true" : "false");
  }

  int button_index_{4};
  bool latch_estop_{true};
  bool estop_active_{false};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EStopNode>());
  rclcpp::shutdown();
  return 0;
}
