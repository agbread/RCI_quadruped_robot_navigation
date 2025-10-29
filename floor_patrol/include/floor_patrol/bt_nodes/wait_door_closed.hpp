#pragma once
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <mutex>
#include <optional>
#include <array>
#include <string>
#include <memory>

namespace floor_patrol {

class WaitDoorClosed : public BT::SyncActionNode {
public:
  WaitDoorClosed(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr getOrMakeNode_();
  std::string topicFrom(const std::string& lift_id, const std::string& key) const;
  bool ensureSubTo(const std::string& topic);
  static double measure(const std::array<double,2>& lr);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  std::string current_topic_;
  std::mutex mtx_;
  std::optional<std::array<double,2>> last_lr_;
};

} // namespace floor_patrol
