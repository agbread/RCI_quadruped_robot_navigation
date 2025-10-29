#include "floor_patrol/bt_nodes/enter_cabin_offset.hpp"
#include <chrono>
#include <cmath>
using namespace std::chrono_literals;

namespace floor_patrol {

BT::PortsList EnterCabinOffset::providedPorts() {
  return {
    BT::InputPort<std::string>("lift_id"),
    BT::InputPort<int>("floor"),
    BT::InputPort<std::map<std::string, std::map<int, Pose2D>>>("elevator_front"),
    BT::InputPort<std::map<std::string, Offset>>("cabin_offset"),
    BT::InputPort<std::string>("action_name", std::string("/follow_waypoints"),
                               "nav2 FollowWaypoints action name"),
    BT::InputPort<std::string>("global_frame", std::string("map"), "frame id")
  };
}

rclcpp::Node::SharedPtr EnterCabinOffset::getOrMakeNode_(){
  if (node_) return node_;
  node_ = rclcpp::Node::make_shared("bt_enter_cabin_offset");
  return node_;
}

rclcpp_action::Client<EnterCabinOffset::FW>::SharedPtr
EnterCabinOffset::getClient_(const std::string& action_name) {
  auto node = getOrMakeNode_();
  if (client_ && client_action_name_ == action_name) {
    return client_;
  }
  client_.reset();
  client_ = rclcpp_action::create_client<FW>(node, action_name);
  if (!client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(node->get_logger(), "[EnterCabinOffset] action server not available: %s",
                 action_name.c_str());
    return nullptr;
  }
  client_action_name_ = action_name;  // ★ 캐시 갱신
  return client_;
}

double EnterCabinOffset::deg2rad(double d){ return d*M_PI/180.0; }

geometry_msgs::msg::PoseStamped
EnterCabinOffset::makePose(const Pose2D& p, const std::string& frame, rclcpp::Clock& clock){
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = frame;
  msg.header.stamp = clock.now();  
  msg.pose.position.x = p.x;
  msg.pose.position.y = p.y;
  msg.pose.position.z = 0.0;
  const double yaw = deg2rad(p.yaw_deg);
  const double c = std::cos(yaw*0.5), s = std::sin(yaw*0.5);
  msg.pose.orientation.z = s;
  msg.pose.orientation.w = c;
  return msg;
}

BT::NodeStatus EnterCabinOffset::tick() {
  auto node = getOrMakeNode_();

  const auto lid  = getInput<std::string>("lift_id");
  const auto fl   = getInput<int>("floor");
  const auto fr   = getInput<std::map<std::string, std::map<int, Pose2D>>>("elevator_front");
  const auto off  = getInput<std::map<std::string, Offset>>("cabin_offset");
  const auto act  = getInput<std::string>("action_name").value();
  const auto frame= getInput<std::string>("global_frame").value();

  if(!lid)  throw BT::RuntimeError("EnterCabinOffset: missing lift_id: ", lid.error());
  if(!fl)   throw BT::RuntimeError("EnterCabinOffset: missing floor: ", fl.error());
  if(!fr)   throw BT::RuntimeError("EnterCabinOffset: missing elevator_front: ", fr.error());
  if(!off)  throw BT::RuntimeError("EnterCabinOffset: missing cabin_offset: ", off.error());

  const auto& ef_by_lift = fr.value();
  auto itL = ef_by_lift.find(lid.value());
  if (itL == ef_by_lift.end()) {
    RCLCPP_ERROR(node->get_logger(), "[EnterCabinOffset] no elevator_front for lift_id=%s", lid.value().c_str());
    return BT::NodeStatus::FAILURE;
  }
  auto itF = itL->second.find(fl.value());
  if (itF == itL->second.end()) {
    RCLCPP_ERROR(node->get_logger(), "[EnterCabinOffset] no elevator_front for floor=%d", fl.value());
    return BT::NodeStatus::FAILURE;
  }
  Pose2D base = itF->second;

  const auto& off_map = off.value();
  auto itO = off_map.find(lid.value());
  if (itO == off_map.end()) {
    RCLCPP_ERROR(node->get_logger(), "[EnterCabinOffset] no cabin_offset for lift_id=%s", lid.value().c_str());
    return BT::NodeStatus::FAILURE;
  }
  const Offset o = itO->second;

  const double yaw = deg2rad(base.yaw_deg);
  const double dx =  o.forward_m*std::cos(yaw) - o.lateral_m*std::sin(yaw);
  const double dy =  o.forward_m*std::sin(yaw) + o.lateral_m*std::cos(yaw);
  Pose2D goal { base.x + dx, base.y + dy, base.yaw_deg + o.yaw_delta_deg };

  auto client = getClient_(act);
  if (!client) return BT::NodeStatus::FAILURE;

  FW::Goal goal_msg;
  goal_msg.poses.emplace_back(makePose(goal, frame, *node->get_clock()));  // ★ clock 전달


  auto fut = client->async_send_goal(goal_msg);
  if (rclcpp::spin_until_future_complete(node_, fut, 30s) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "[EnterCabinOffset] send_goal timeout");
    return BT::NodeStatus::FAILURE;
  }
  auto handle = fut.get();
  if (!handle) {
    RCLCPP_ERROR(node->get_logger(), "[EnterCabinOffset] goal rejected");
    return BT::NodeStatus::FAILURE;
  }

  auto fut_res = client->async_get_result(handle);
  if (rclcpp::spin_until_future_complete(node_, fut_res, 180s) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "[EnterCabinOffset] result timeout");
    return BT::NodeStatus::FAILURE;
  }
  auto res = fut_res.get();
  if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(node->get_logger(), "[EnterCabinOffset] reached cabin offset");
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_ERROR(node->get_logger(), "[EnterCabinOffset] action failed");
  return BT::NodeStatus::FAILURE;
}

} // namespace floor_patrol
