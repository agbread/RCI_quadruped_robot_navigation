#pragma once
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <memory>

namespace floor_patrol {

class PublishTargetFloor : public BT::SyncActionNode {
public:
  PublishTargetFloor(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr getOrMakeNode_();
  std::string topicFrom(const std::string& lift_id, const std::string& key) const;

  rclcpp::Node::SharedPtr node_;
};

} // namespace floor_patrol
