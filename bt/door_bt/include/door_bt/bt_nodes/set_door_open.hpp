#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class SetDoorOpen : public BT::SyncActionNode
{
public:
  SetDoorOpen(const std::string& name,
              const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  std::string last_topic_;
};
