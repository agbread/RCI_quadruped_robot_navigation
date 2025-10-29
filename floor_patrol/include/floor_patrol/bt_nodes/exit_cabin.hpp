#pragma once
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <map>
#include <string>
#include <memory>

namespace floor_patrol {

class ExitCabin : public BT::SyncActionNode {
public:
  using FW = nav2_msgs::action::FollowWaypoints;

  ExitCabin(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  struct Pose2D { double x, y, yaw_deg; };

  rclcpp::Node::SharedPtr getOrMakeNode_();
  rclcpp_action::Client<FW>::SharedPtr getClient_(const std::string& action_name);
  static geometry_msgs::msg::PoseStamped makePose(const Pose2D& p,
                                                  const std::string& frame,
                                                  rclcpp::Clock& clock);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<FW>::SharedPtr client_;
  std::string client_action_name_;   // 현재 바인딩된 액션 이름 캐시
};

} // namespace floor_patrol
