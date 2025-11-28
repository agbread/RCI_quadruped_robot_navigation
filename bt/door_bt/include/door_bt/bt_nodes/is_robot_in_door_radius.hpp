#pragma once

#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mutex>

class IsRobotInDoorRadius : public BT::ConditionNode
{
public:
  IsRobotInDoorRadius(const std::string& name,
                      const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  // TF에서 robot pose 가져오기
  bool getRobotPosition(double& x, double& y);

  // ROS
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // frame 이름
  std::string world_frame_{"world"};
  std::string robot_frame_{"base_link"};
};
