#pragma once

#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class IsRobotPastDoor : public BT::ConditionNode
{
public:
  IsRobotPastDoor(const std::string& name,
                  const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  bool getRobotPosition(double& x, double& y);

  // ROS2
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // frame 이름 (필요하면 나중에 포트로 뺄 수 있음)
  std::string world_frame_{"world"};
  std::string robot_frame_{"base_link"};
};
