#pragma once

#include <behaviortree_cpp_v3/action_node.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>

namespace floor_change_bt
{

class PublishDoorYaml : public BT::SyncActionNode
{
public:
  PublishDoorYaml(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  // Nav2 BT(혹은 자체 runner)에서 blackboard로 node를 주면 그걸 쓰고,
  // 없으면 내부 node를 하나 만든다.
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  std::string current_topic_;

  // 같은 yaml이면 재발행 안 하도록 캐시
  std::string last_yaml_;

  void ensurePublisher(const std::string& topic);
  rclcpp::Node::SharedPtr getNodeFromBlackboardOrCreate();
};

}  // namespace door_bt
