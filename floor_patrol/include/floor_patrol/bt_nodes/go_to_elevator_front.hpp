#pragma once
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <map>
#include <string>
#include <memory>

namespace floor_patrol {

// elevator_front: lift_id -> floor(int) -> PoseStamped(map frame)
class GoToElevatorFront : public BT::SyncActionNode {
public:
  using FW = nav2_msgs::action::FollowWaypoints;

  using FrontMap = std::map<std::string, std::map<int, geometry_msgs::msg::PoseStamped>>;

  GoToElevatorFront(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr getOrMakeNode_();

  rclcpp::Node::SharedPtr node_;
};

} // namespace floor_patrol
