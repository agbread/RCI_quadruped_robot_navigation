#include "floor_patrol/bt_nodes/compute_plan.hpp"
#include <rclcpp/rclcpp.hpp>

namespace floor_patrol {

BT::PortsList ComputePlan::providedPorts() {
  return {
    BT::InputPort<std::string>("patrol_mode"),              // "full" | "partial"
    BT::InputPort<WaypointMap>("waypoints"),                // floor -> poses
    BT::InputPort<SubsetMap>("subset"),                     // floor -> indices (optional)
    BT::InputPort<int>("floor"),
    BT::OutputPort<Plan>("out_plan")
  };
}

ComputePlan::Plan ComputePlan::pickAll(const Plan& src) {
  return src; // 복사
}

ComputePlan::Plan ComputePlan::pickSubset(const Plan& src, const std::vector<int>& idx) {
  Plan out;
  out.reserve(idx.size());
  for (int i : idx) {
    if (i < 0 || static_cast<size_t>(i) >= src.size()) {
      // 인덱스가 잘못되면 그냥 스킵(필요시 throw 로 바꿔도 됨)
      continue;
    }
    out.push_back(src[static_cast<size_t>(i)]);
  }
  return out;
}

BT::NodeStatus ComputePlan::tick() {
  // 필수 입력 읽기
  const auto mode_res = getInput<std::string>("patrol_mode");
  const auto wp_res   = getInput<WaypointMap>("waypoints");
  const auto floor_res= getInput<int>("floor");

  if (!mode_res)  throw BT::RuntimeError("ComputePlan: missing patrol_mode: ", mode_res.error());
  if (!wp_res)    throw BT::RuntimeError("ComputePlan: missing waypoints: ", wp_res.error());
  if (!floor_res) throw BT::RuntimeError("ComputePlan: missing floor: ", floor_res.error());

  const std::string mode = *mode_res;
  const WaypointMap& wmap = *wp_res;
  const int floor = *floor_res;

  auto it = wmap.find(floor);
  if (it == wmap.end()) {
    throw BT::RuntimeError("ComputePlan: no waypoints for floor=", std::to_string(floor));
  }
  const Plan& src = it->second;

  Plan plan;
  if (mode == "partial") {
    // subset은 optional
    if (auto sub_res = getInput<SubsetMap>("subset"); sub_res && !sub_res->empty()) {
      const auto& smap = *sub_res;
      if (auto sit = smap.find(floor); sit != smap.end()) {
        plan = pickSubset(src, sit->second);
      } else {
        // 해당 층에 subset이 없으면 전체 사용(혹은 빈 계획으로 하고 싶으면 빈 벡터 할당)
        plan = pickAll(src);
      }
    } else {
      plan = pickAll(src);
    }
  } else {
    // 기본은 full
    plan = pickAll(src);
  }

  setOutput("out_plan", plan);
  return BT::NodeStatus::SUCCESS;
}

} // namespace floor_patrol
