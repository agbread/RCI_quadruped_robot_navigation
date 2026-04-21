#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace stair_bt
{

class GetCurrentFloor : public BT::StatefulActionNode
{
public:
  GetCurrentFloor(const std::string & name,
                  const BT::NodeConfiguration & config);


  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("map_frame", "기준 frame (예: map/world)"),
      BT::InputPort<std::string>("base_frame", "로봇 base frame (예: base_link)"),
      BT::InputPort<double>("floor0_z", 0.0, "0층 z"),
      BT::InputPort<double>("floor1_z", 5.0, "1층 z"),
      BT::InputPort<double>("floor2_z", 10.0, "2층 z"),
      BT::OutputPort<int>("current_floor", "현재 층 인덱스")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace stair_bt
