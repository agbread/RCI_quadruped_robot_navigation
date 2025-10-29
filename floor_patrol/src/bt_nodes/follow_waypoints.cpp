#include "floor_patrol/bt_nodes/follow_waypoints.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace floor_patrol {

BT::PortsList FollowWaypointsNode::providedPorts() {
  return {
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("input_plan"),
    BT::InputPort<std::string>("action_name"),                   // ex) "/follow_waypoints"
    BT::InputPort<rclcpp::Node::SharedPtr>("ros_node")           // 선택: 외부 노드 주입
  };
}

rclcpp::Node::SharedPtr FollowWaypointsNode::getOrMakeNode_() {
  if (node_) return node_;
  if (auto nres = getInput<rclcpp::Node::SharedPtr>("ros_node"); nres && nres.value()) {
    node_ = nres.value();
    return node_;
  }
  node_ = rclcpp::Node::make_shared("bt_follow_waypoints_node");
  return node_;
}

BT::NodeStatus FollowWaypointsNode::tick() {
  auto node = getOrMakeNode_();

  const auto plan_res  = getInput<std::vector<geometry_msgs::msg::PoseStamped>>("input_plan");
  const auto aname_res = getInput<std::string>("action_name");
  if (!plan_res)  throw BT::RuntimeError("FollowWaypoints: missing input_plan: ", plan_res.error());
  if (!aname_res) throw BT::RuntimeError("FollowWaypoints: missing action_name: ", aname_res.error());

  const auto& plan = plan_res.value();
  const std::string action_name = aname_res.value();

  if (plan.empty()) {
    RCLCPP_WARN(node->get_logger(), "[FollowWaypoints] input_plan is empty. Returning SUCCESS.");
    return BT::NodeStatus::SUCCESS;
  }

  auto client = rclcpp_action::create_client<FW>(node, action_name);

  if (!client->wait_for_action_server(5s)) {
    RCLCPP_ERROR(node->get_logger(), "[FollowWaypoints] Action server %s not available", action_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  FW::Goal goal;
  goal.poses = plan;

  auto send_goal_options = rclcpp_action::Client<FW>::SendGoalOptions{};
  auto future_handle = client->async_send_goal(goal, send_goal_options);
  if (rclcpp::spin_until_future_complete(node, future_handle, 30s) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "[FollowWaypoints] Failed to send goal");
    return BT::NodeStatus::FAILURE;
  }
  auto goal_handle = future_handle.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "[FollowWaypoints] Goal was rejected");
    return BT::NodeStatus::FAILURE;
  }

  auto future_result = client->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node, future_result, 10min) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "[FollowWaypoints] Result wait timeout/failure");
    return BT::NodeStatus::FAILURE;
  }

  auto result = future_result.get();
  if (result.result && result.result->missed_waypoints.empty()) {
    RCLCPP_INFO(node->get_logger(), "[FollowWaypoints] Succeeded. visited=%zu", plan.size());
    return BT::NodeStatus::SUCCESS;
  } else {
    if (result.result) {
      std::string missed;
      for (auto i : result.result->missed_waypoints) { missed += std::to_string(i) + " "; }
      RCLCPP_WARN(node->get_logger(), "[FollowWaypoints] Missed waypoints: %s", missed.c_str());
    }
    return BT::NodeStatus::FAILURE;
  }
}

} // namespace floor_patrol
