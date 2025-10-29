#pragma once
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include <memory>

namespace floor_patrol {

class WaitElevatorArrived : public BT::SyncActionNode {
public:
  WaitElevatorArrived(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr getOrMakeNode_();
  std::string topicFrom(const std::string& lift_id, const std::string& key) const;
  bool ensureSubTo(const std::string& topic);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
  std::string current_topic_;
  std::mutex mtx_;
  std::optional<double> last_z_;
};

} // namespace floor_patrol
