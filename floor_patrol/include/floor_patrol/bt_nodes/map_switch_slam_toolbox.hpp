#pragma once
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <map>
#include <string>
#include <memory>

#include <slam_toolbox/srv/deserialize_pose_graph.hpp>  // SLAM Toolbox

namespace floor_patrol {

class MapSwitchSlamToolbox : public BT::SyncActionNode {
public:
  using Deserialize = slam_toolbox::srv::DeserializePoseGraph;

  MapSwitchSlamToolbox(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr getOrMakeNode_();
  std::string serviceName_(const std::string& slam_ns) const;

  rclcpp::Node::SharedPtr node_;
};

} // namespace floor_patrol
