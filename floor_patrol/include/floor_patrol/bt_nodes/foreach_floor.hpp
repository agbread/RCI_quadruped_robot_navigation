#pragma once
#include <behaviortree_cpp_v3/bt_factory.h>
#include <vector>
#include <string>

namespace floor_patrol {

class ForEachFloor : public BT::ControlNode {
public:
  ForEachFloor(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
  void halt() override;

private:
  std::vector<int> floors_;
  size_t idx_;
  bool child_running_;
};

} // namespace floor_patrol
