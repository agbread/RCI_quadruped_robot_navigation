#pragma once
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <memory>
#include <mutex>
#include <optional>

namespace floor_patrol {

class IsDoorOpen : public BT::ConditionNode {
public:
  IsDoorOpen(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr getOrMakeNode_();
  std::string topicFrom(const std::string& lift_id, const std::string& key) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  std::string current_topic_;

  std::mutex mtx_;
  std::optional<std::array<double,2>> last_lr_;

  // helper
  bool ensureSubTo(const std::string& topic);
  static double doorMeasure(const std::array<double,2>& lr); // min(right,left)
};

} // namespace floor_patrol
