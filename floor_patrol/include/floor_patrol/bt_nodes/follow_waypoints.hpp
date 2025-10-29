#pragma once
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <vector>
#include <string>
#include <memory>

namespace floor_patrol {

class FollowWaypointsNode : public BT::SyncActionNode {
public:
  using FW = nav2_msgs::action::FollowWaypoints;
  using FWGoalHandle = rclcpp_action::ClientGoalHandle<FW>;

  FollowWaypointsNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  // 외부에서 ros_node를 안 주면 내부에서 임시 생성해 씀
  rclcpp::Node::SharedPtr getOrMakeNode_();

  rclcpp::Node::SharedPtr node_;
};

} // namespace floor_patrol
