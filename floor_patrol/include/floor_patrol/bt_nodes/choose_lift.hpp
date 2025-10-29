#pragma once
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <map>
#include <string>
#include <vector>
#include <memory>

namespace floor_patrol {

// elevator_front: lift_id -> floor(int) -> PoseStamped(map frame)
class ChooseLift : public BT::SyncActionNode {
public:
  using FrontMap = std::map<std::string, std::map<int, geometry_msgs::msg::PoseStamped>>;

  ChooseLift(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg)
  {}

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr getOrMakeNode_();
  bool lookupBaseInMap_(double& x, double& y);

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace floor_patrol
