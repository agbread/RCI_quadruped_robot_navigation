#pragma once
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <memory>

namespace floor_patrol {

class IsDoorProgress : public BT::ConditionNode {
public:
  IsDoorProgress(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr getOrMakeNode_();
  std::string topicFrom(const std::string& lift_id, const std::string& key) const;
  bool ensureSubTo(const std::string& topic);

  struct Sample { rclcpp::Time t; double m; };
  static double measure(const std::vector<double>& v); // min(right,left)
  static double slope(const Sample& a, const Sample& b); // dm/dt

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  std::string current_topic_;
  std::mutex mtx_;
  std::deque<Sample> buf_; // 최근 샘플 5개 정도
};

} // namespace floor_patrol
