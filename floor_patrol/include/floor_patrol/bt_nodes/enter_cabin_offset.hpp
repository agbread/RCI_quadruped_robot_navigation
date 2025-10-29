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

// ... 기존 include/namespace 동일

class EnterCabinOffset : public BT::SyncActionNode {
public:
  using FW = nav2_msgs::action::FollowWaypoints;

  EnterCabinOffset(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  struct Pose2D { double x, y, yaw_deg; };
  struct Offset { double forward_m, lateral_m, yaw_delta_deg; };

  rclcpp::Node::SharedPtr getOrMakeNode_();
  rclcpp_action::Client<FW>::SharedPtr getClient_(const std::string& action_name);

  static double deg2rad(double d);
  static geometry_msgs::msg::PoseStamped makePose(const Pose2D& p, const std::string& frame,
                                                  rclcpp::Clock& clock);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<FW>::SharedPtr client_;
  std::string client_action_name_;   // ★ 추가: 현재 클라이언트가 바인딩된 액션 이름 캐시
};


} // namespace floor_patrol
