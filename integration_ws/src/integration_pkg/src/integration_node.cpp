#include <chrono>
#include <memory>
#include <string>

#include "example_interfaces/srv/trigger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class IntegrationNode : public rclcpp::Node
{
public:
  IntegrationNode()
  : Node("integration")
  {
    this->declare_parameter<bool>("use_sim_time", false);
    const auto planning_service =
      this->declare_parameter<std::string>("planning_service", "/planning/compute_plan");
    const auto control_service =
      this->declare_parameter<std::string>("control_service", "/control/set_mode");
    const auto perception_topic =
      this->declare_parameter<std::string>("perception_topic", "/perception/objects");
    service_check_timeout_ =
      this->declare_parameter<double>("service_check_timeout", 1.0);
    const auto timer_period =
      this->declare_parameter<double>("timer_period", 5.0);

    this->declare_parameter<std::string>("system_mode", "sim");
    this->declare_parameter<bool>("slam_enabled", false);
    this->declare_parameter<bool>("map_mode", false);
    this->declare_parameter<std::string>("localization_mode", "none");
    this->declare_parameter<std::string>("localization_map_file", "");
    this->declare_parameter<std::string>("localization_scan_topic", "/scan");
    this->declare_parameter<std::string>("localization_odom_topic", "/odom");

    cli_plan_ = this->create_client<example_interfaces::srv::Trigger>(planning_service);
    cli_mode_ = this->create_client<example_interfaces::srv::Trigger>(control_service);
    sub_ = this->create_subscription<std_msgs::msg::String>(
      perception_topic,
      10,
      std::bind(&IntegrationNode::objects_cb, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(timer_period),
      std::bind(&IntegrationNode::timer_cb, this));

    RCLCPP_INFO(this->get_logger(), "Integration node started");
  }

private:
  void objects_cb(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Integration received: %s", msg->data.c_str());
  }

  void timer_cb()
  {
    if (cli_plan_->wait_for_service(std::chrono::duration<double>(service_check_timeout_))) {
      auto req = std::make_shared<example_interfaces::srv::Trigger::Request>();
      cli_plan_->async_send_request(
        req,
        std::bind(&IntegrationNode::plan_response, this, std::placeholders::_1));
    } else {
      RCLCPP_WARN(this->get_logger(), "Planning service not available");
    }

    if (cli_mode_->wait_for_service(std::chrono::duration<double>(service_check_timeout_))) {
      auto req = std::make_shared<example_interfaces::srv::Trigger::Request>();
      cli_mode_->async_send_request(
        req,
        std::bind(&IntegrationNode::mode_response, this, std::placeholders::_1));
    } else {
      RCLCPP_WARN(this->get_logger(), "Control service not available");
    }
  }

  void plan_response(rclcpp::Client<example_interfaces::srv::Trigger>::SharedFuture future)
  {
    try {
      const auto res = future.get();
      RCLCPP_INFO(
        this->get_logger(),
        "Planning service replied: success=%s, msg=%s",
        res->success ? "true" : "false",
        res->message.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Planning service call failed: %s", e.what());
    }
  }

  void mode_response(rclcpp::Client<example_interfaces::srv::Trigger>::SharedFuture future)
  {
    try {
      const auto res = future.get();
      RCLCPP_INFO(
        this->get_logger(),
        "Control service replied: success=%s, msg=%s",
        res->success ? "true" : "false",
        res->message.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Control service call failed: %s", e.what());
    }
  }

  double service_check_timeout_{1.0};
  rclcpp::Client<example_interfaces::srv::Trigger>::SharedPtr cli_plan_;
  rclcpp::Client<example_interfaces::srv::Trigger>::SharedPtr cli_mode_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IntegrationNode>());
  rclcpp::shutdown();
  return 0;
}
