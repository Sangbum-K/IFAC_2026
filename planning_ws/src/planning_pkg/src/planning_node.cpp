#include <memory>
#include <string>

#include "example_interfaces/srv/trigger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PlanningNode : public rclcpp::Node
{
public:
  PlanningNode()
  : Node("planning")
  {
    planning_horizon_ =
      this->declare_parameter<double>("planning_horizon", 5.0);
    const auto input_topic =
      this->declare_parameter<std::string>("input_topic", "/perception/objects");
    const auto output_topic =
      this->declare_parameter<std::string>("output_topic", "/planning/plan");
    const auto service_name =
      this->declare_parameter<std::string>("service_name", "/planning/compute_plan");

    sub_ = this->create_subscription<std_msgs::msg::String>(
      input_topic,
      10,
      std::bind(&PlanningNode::objects_cb, this, std::placeholders::_1));
    pub_ = this->create_publisher<std_msgs::msg::String>(output_topic, 10);
    srv_ = this->create_service<example_interfaces::srv::Trigger>(
      service_name,
      std::bind(
        &PlanningNode::compute_plan_cb,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Planning node started");
  }

private:
  void objects_cb(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received objects: %s", msg->data.c_str());
    std_msgs::msg::String plan;
    plan.data = "plan_based_on:" + msg->data;
    pub_->publish(plan);
  }

  void compute_plan_cb(
    const std::shared_ptr<example_interfaces::srv::Trigger::Request> request,
    std::shared_ptr<example_interfaces::srv::Trigger::Response> response)
  {
    (void)request;
    response->success = true;
    response->message = "Computed plan (horizon=" + std::to_string(planning_horizon_) + ")";
    RCLCPP_INFO(this->get_logger(), "compute_plan service called");
  }

  double planning_horizon_{5.0};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningNode>());
  rclcpp::shutdown();
  return 0;
}
