#pragma once
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map>
#include <vector>
#include <string>

namespace floor_patrol {

/// waypoints:  floor(int) -> vector<PoseStamped>
/// subset:     floor(int) -> vector<int> (선택된 인덱스)  (partial일 때 사용)
class ComputePlan : public BT::SyncActionNode {
public:
  ComputePlan(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  using Plan = std::vector<geometry_msgs::msg::PoseStamped>;
  using WaypointMap = std::map<int, Plan>;
  using SubsetMap   = std::map<int, std::vector<int>>;

  static Plan pickAll(const Plan& src);
  static Plan pickSubset(const Plan& src, const std::vector<int>& idx);
};

} // namespace floor_patrol
