#include "floor_patrol/bt_nodes/foreach_floor.hpp"
#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/blackboard.h>


namespace floor_patrol {


ForEachFloor::ForEachFloor(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ControlNode(name, config), idx_(0), child_running_(false) {}

BT::PortsList ForEachFloor::providedPorts() {
  return {
    BT::InputPort<std::vector<int>>("floor_list"),
    BT::OutputPort<int>("floor"),
    BT::OutputPort<int>("next_floor")
  };
}

BT::NodeStatus ForEachFloor::tick() {
  // floor_list 읽기
  if (floors_.empty()) {
    auto res = getInput<std::vector<int>>("floor_list");
    if (!res) {
      throw BT::RuntimeError("ForEachFloor: missing required input [floor_list]: ", res.error());
    }
    floors_ = res.value();
    if (floors_.empty()) {
      throw BT::RuntimeError("ForEachFloor: floor_list is empty");
    }
  }

  // 현재/다음 층 설정
  const int cur = floors_[idx_ % floors_.size()];
  const int nxt = floors_[(idx_ + 1) % floors_.size()];
  setOutput("floor", cur);
  setOutput("next_floor", nxt);

  // 자식 한 개만 있다고 가정(네 XML에서 Sequence 1개)
  if (childrenCount() != 1) {
    throw BT::RuntimeError("ForEachFloor expects exactly 1 child (the Sequence). Found: ",
                           std::to_string(childrenCount()));
  }

  BT::TreeNode* child = children_nodes_[0];
  BT::NodeStatus child_status = child->executeTick();

  switch (child_status) {
    case BT::NodeStatus::SUCCESS:
      // 다음 층으로 진행
      idx_ = (idx_ + 1) % floors_.size();
      child_running_ = false;
      return BT::NodeStatus::RUNNING; // 무한 루프 상위 Repeat가 있으므로 RUNNING 반환
    case BT::NodeStatus::FAILURE:
      child_running_ = false;
      return BT::NodeStatus::FAILURE;
    case BT::NodeStatus::RUNNING:
      child_running_ = true;
      return BT::NodeStatus::RUNNING;
    default:
      return BT::NodeStatus::FAILURE;
  }
}

void ForEachFloor::halt() {
  if (child_running_ && childrenCount() == 1) {
    children_nodes_[0]->halt();
  }
  child_running_ = false;
  ControlNode::halt();
}

} // namespace floor_patrol
