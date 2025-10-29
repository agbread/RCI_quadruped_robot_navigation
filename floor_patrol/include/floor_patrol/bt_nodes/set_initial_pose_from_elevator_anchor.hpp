#pragma once
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <map>
#include <string>
#include <memory>

namespace floor_patrol {

class SetInitialPoseFromElevatorAnchor : public BT::SyncActionNode {
public:
  struct Pose2D { double x, y, yaw_deg; };

  SetInitialPoseFromElevatorAnchor(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr getOrMakeNode_();
  static geometry_msgs::msg::PoseWithCovarianceStamped makeMsg(const Pose2D& p,
                                                               const std::string& frame);

  rclcpp::Node::SharedPtr node_;
};

} // namespace floor_patrol
