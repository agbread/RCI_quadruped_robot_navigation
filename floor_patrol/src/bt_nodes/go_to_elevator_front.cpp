#include "floor_patrol/bt_nodes/go_to_elevator_front.hpp"
#include <chrono>
using namespace std::chrono_literals;

namespace floor_patrol {

BT::PortsList GoToElevatorFront::providedPorts() {
  return {
    BT::InputPort<std::string>("lift_id"),
    BT::InputPort<int>("floor"),
    BT::InputPort<FrontMap>("elevator_front"),
    BT::InputPort<std::string>("action_name"),                    // ex) "/follow_waypoints"
    BT::InputPort<rclcpp::Node::SharedPtr>("ros_node")            // optional
  };
}

rclcpp::Node::SharedPtr GoToElevatorFront::getOrMakeNode_() {
  if (node_) return node_;
  if (auto nres = getInput<rclcpp::Node::SharedPtr>("ros_node"); nres && nres.value()) {
    node_ = nres.value();
  } else {
    node_ = rclcpp::Node::make_shared("bt_go_to_elevator_front_node");
  }
  return node_;
}

BT::NodeStatus GoToElevatorFront::tick() {
  auto node = getOrMakeNode_();

  // 입력 체크
  const auto id_res    = getInput<std::string>("lift_id");
  const auto fl_res    = getInput<int>("floor");
  const auto map_res   = getInput<FrontMap>("elevator_front");
  const auto aname_res = getInput<std::string>("action_name");

  if (!id_res)    throw BT::RuntimeError("GoToElevatorFront: missing lift_id: ", id_res.error());
  if (!fl_res)    throw BT::RuntimeError("GoToElevatorFront: missing floor: ", fl_res.error());
  if (!map_res)   throw BT::RuntimeError("GoToElevatorFront: missing elevator_front: ", map_res.error());
  if (!aname_res) throw BT::RuntimeError("GoToElevatorFront: missing action_name: ", aname_res.error());

  const std::string lift_id = id_res.value();
  const int floor = fl_res.value();
  const auto& front_map = map_res.value();
  const std::string action_name = aname_res.value();

  // 목표 포즈 조회
  auto itLift = front_map.find(lift_id);
  if (itLift == front_map.end()) {
    RCLCPP_ERROR(node->get_logger(), "[GoToElevatorFront] elevator_front has no entry for %s", lift_id.c_str());
    return BT::NodeStatus::FAILURE;
  }
  auto itFloor = itLift->second.find(floor);
  if (itFloor == itLift->second.end()) {
    RCLCPP_ERROR(node->get_logger(), "[GoToElevatorFront] elevator_front[%s] missing floor=%d", lift_id.c_str(), floor);
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped target = itFloor->second;
  // stamp 보정(선택)
  target.header.stamp = node->get_clock()->now();

  // FollowWaypoints(1점) 호출
  auto client = rclcpp_action::create_client<FW>(node, action_name);
  if (!client->wait_for_action_server(5s)) {
    RCLCPP_ERROR(node->get_logger(), "[GoToElevatorFront] Action server %s not available", action_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  FW::Goal goal;
  goal.poses.clear();
  goal.poses.emplace_back(target);

  auto send_opts = rclcpp_action::Client<FW>::SendGoalOptions{};
  auto fut_handle = client->async_send_goal(goal, send_opts);
  if (rclcpp::spin_until_future_complete(node, fut_handle, 30s) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "[GoToElevatorFront] Failed to send goal");
    return BT::NodeStatus::FAILURE;
  }
  auto handle = fut_handle.get();
  if (!handle) {
    RCLCPP_ERROR(node->get_logger(), "[GoToElevatorFront] Goal rejected");
    return BT::NodeStatus::FAILURE;
  }

  auto fut_result = client->async_get_result(handle);
  if (rclcpp::spin_until_future_complete(node, fut_result, 10min) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "[GoToElevatorFront] Result wait timeout/failure");
    return BT::NodeStatus::FAILURE;
  }

  auto result = fut_result.get();
  if (result.result && result.result->missed_waypoints.empty()) {
    RCLCPP_INFO(node->get_logger(), "[GoToElevatorFront] Reached elevator front (%s, floor=%d)",
                lift_id.c_str(), floor);
    return BT::NodeStatus::SUCCESS;
  } else {
    if (result.result) {
      std::string missed;
      for (auto i : result.result->missed_waypoints) missed += std::to_string(i) + " ";
      RCLCPP_WARN(node->get_logger(), "[GoToElevatorFront] Missed waypoints: %s", missed.c_str());
    }
    return BT::NodeStatus::FAILURE;
  }
}

} // namespace floor_patrol
