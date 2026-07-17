// pitwall — 초경량 텔레메트리 (unicorn/HMCL pitwall-telemetry 대응, C++판).
//
//   #include "planning_pkg/pitwall.hpp"
//   pitwall::Init(this);                 // 노드 생성자에서 1회
//   pitwall::Log("evasion_d_apex", d);   // -> /pitwall/evasion_d_apex (Float32)
//   pitwall::Event("회피 포기(트랙경계)");  // -> /pitwall/events (String)
//
// Init을 안 하면 모든 호출이 조용히 no-op (계측이 주행 코드를 죽이면 안 된다).
// header-only: planning_pkg 외 패키지(control_ws 등)에서도 include만 하면 된다.
#ifndef PLANNING_PKG__PITWALL_HPP_
#define PLANNING_PKG__PITWALL_HPP_

#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

namespace pitwall
{

namespace detail
{
inline rclcpp::Node *& NodePtr()
{
  static rclcpp::Node * node = nullptr;
  return node;
}
inline std::unordered_map<std::string,
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> & FloatPubs()
{
  static std::unordered_map<std::string,
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> pubs;
  return pubs;
}
inline rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & EventPub()
{
  static rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
  return pub;
}
}  // namespace detail

inline void Init(rclcpp::Node * node)
{
  try {
    detail::NodePtr() = node;
    detail::FloatPubs().clear();
    detail::EventPub() =
      node->create_publisher<std_msgs::msg::String>("/pitwall/events", 10);
  } catch (...) {
    detail::NodePtr() = nullptr;
  }
}

inline void Log(const std::string & name, double value)
{
  auto * node = detail::NodePtr();
  if (node == nullptr) {return;}
  try {
    auto & pubs = detail::FloatPubs();
    auto it = pubs.find(name);
    if (it == pubs.end()) {
      it = pubs.emplace(name, node->create_publisher<std_msgs::msg::Float32>(
        "/pitwall/" + name, 10)).first;
    }
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(value);
    it->second->publish(msg);
  } catch (...) {
  }
}

inline void Event(const std::string & text)
{
  auto * node = detail::NodePtr();
  if (node == nullptr || detail::EventPub() == nullptr) {return;}
  try {
    const auto t = node->get_clock()->now();
    const int64_t ns = t.nanoseconds();
    std_msgs::msg::String msg;
    msg.data = "[" + std::to_string(ns / 1000000000) + "." +
      std::to_string(ns % 1000000000) + "][" + node->get_name() + "] " + text;
    detail::EventPub()->publish(msg);
  } catch (...) {
  }
}

}  // namespace pitwall

#endif  // PLANNING_PKG__PITWALL_HPP_
