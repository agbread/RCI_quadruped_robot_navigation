#pragma once

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <behaviortree_cpp_v3/action_node.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace elevator_bt
{

class Nav2NavigateToPose : public BT::StatefulActionNode
{
public:
  using NavigateToPose           = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  Nav2NavigateToPose(const std::string & name,
                     const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")
    };
  }

private:
  // BT::StatefulActionNode 인터페이스
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  // 기본 ROS 객체들
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Logger logger_;

  // Goal 진행 상태
  bool            goal_sent_{false};
  rclcpp::Time    goal_start_time_;
  rclcpp::Duration timeout_;

  // 현재 진행 중인 goal handle
  GoalHandleNavigateToPose::SharedPtr current_goal_handle_;

  // 목표 포즈 / 성공 반경
  geometry_msgs::msg::PoseStamped goal_pose_;
  double success_radius_;   // 생성자에서 파라미터로 설정

  // TF
  std::shared_ptr<tf2_ros::Buffer>           tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace elevator_bt
