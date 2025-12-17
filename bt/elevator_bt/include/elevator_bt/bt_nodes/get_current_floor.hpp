#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace elevator_bt
{

// 현재 world->base_link TF의 z 값을 읽어서 층 index를 계산하는 노드
//  - z ≈ 0   -> 0층
//  - z ≈ 5   -> 1층
//  - z ≈ 10  -> 2층
// (임계값은 .cpp에서 설명)
class GetCurrentFloor : public BT::SyncActionNode
{
public:
  GetCurrentFloor(const std::string & name,
                  const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace elevator_bt
