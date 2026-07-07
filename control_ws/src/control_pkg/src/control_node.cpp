#include <memory>
#include <string>
#include <utility>

#include "example_interfaces/srv/trigger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ControlNode : public rclcpp::Node
{
public:
  ControlNode()
  : Node("control"), mode_("idle")
  {
    const int control_rate =
      this->declare_parameter<int>("control_rate", 20);
    const auto input_topic =
      this->declare_parameter<std::string>("input_topic", "/planning/plan");
    const auto output_topic =
      this->declare_parameter<std::string>("output_topic", "/control/cmd");
    const auto service_name =
      this->declare_parameter<std::string>("service_name", "/control/set_mode");

    (void)control_rate;

    sub_ = this->create_subscription<std_msgs::msg::String>(
      input_topic,
      10,
      std::bind(&ControlNode::plan_cb, this, std::placeholders::_1));

    pub_ = this->create_publisher<std_msgs::msg::String>(output_topic, 10);

    srv_ = this->create_service<example_interfaces::srv::Trigger>(
      service_name,
      std::bind(
        &ControlNode::set_mode_cb,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Control node started");
  }

private:
  void plan_cb(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received plan: %s", msg->data.c_str());

    std_msgs::msg::String cmd;
    cmd.data = "cmd_from_plan:" + msg->data;
    pub_->publish(cmd);
  }

  void set_mode_cb(
    const std::shared_ptr<example_interfaces::srv::Trigger::Request> request,
    std::shared_ptr<example_interfaces::srv::Trigger::Response> response)
  {
    (void)request;

    mode_ = (mode_ == "idle") ? "active" : "idle";
    response->success = true;
    response->message = "mode_set:" + mode_;

    RCLCPP_INFO(this->get_logger(), "Control mode changed to %s", mode_.c_str());
  }

  std::string mode_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
