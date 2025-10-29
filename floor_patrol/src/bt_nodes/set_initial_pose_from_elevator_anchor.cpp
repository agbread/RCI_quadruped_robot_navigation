#include "floor_patrol/bt_nodes/set_initial_pose_from_elevator_anchor.hpp"
#include <chrono>
#include <cmath>
using namespace std::chrono_literals;

namespace floor_patrol {

BT::PortsList SetInitialPoseFromElevatorAnchor::providedPorts() {
  return {
    BT::InputPort<std::string>("lift_id"),
    BT::InputPort<int>("floor"),
    BT::InputPort<std::map<std::string, std::map<int, Pose2D>>>("door_anchor"),
    BT::InputPort<std::string>("initialpose_topic", std::string("/initialpose")),
    BT::InputPort<std::string>("global_frame", std::string("map")),
    BT::InputPort<double>("relocalize_wait_sec", 1.0, "sleep secs after publish")
  };
}

rclcpp::Node::SharedPtr SetInitialPoseFromElevatorAnchor::getOrMakeNode_(){
  if (node_) return node_;
  node_ = rclcpp::Node::make_shared("bt_set_initial_pose_from_elevator_anchor");
  return node_;
}

geometry_msgs::msg::PoseWithCovarianceStamped
SetInitialPoseFromElevatorAnchor::makeMsg(const Pose2D& p, const std::string& frame){
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = frame;
  msg.header.stamp = rclcpp::Clock().now();
  msg.pose.pose.position.x = p.x;
  msg.pose.pose.position.y = p.y;
  msg.pose.pose.position.z = 0.0;
  const double yaw = p.yaw_deg * M_PI / 180.0;
  const double c = std::cos(yaw*0.5), s = std::sin(yaw*0.5);
  msg.pose.pose.orientation.z = s;
  msg.pose.pose.orientation.w = c;

  // 적당한 초기 공분산 (Nav2 예제와 유사)
  auto& cov = msg.pose.covariance;
  cov.fill(0.0);
  cov[0]  = 0.25; // x var
  cov[7]  = 0.25; // y var
  cov[35] = 0.30; // yaw var
  return msg;
}

BT::NodeStatus SetInitialPoseFromElevatorAnchor::tick() {
  auto node = getOrMakeNode_();

  const auto lid   = getInput<std::string>("lift_id");
  const auto fl    = getInput<int>("floor");
  const auto anc   = getInput<std::map<std::string, std::map<int, Pose2D>>>("door_anchor");
  const auto topic = getInput<std::string>("initialpose_topic").value();
  const auto frame = getInput<std::string>("global_frame").value();
  const auto waitS = getInput<double>("relocalize_wait_sec").value();

  if (!lid)  throw BT::RuntimeError("SetInitialPoseFromElevatorAnchor: missing lift_id: ", lid.error());
  if (!fl)   throw BT::RuntimeError("SetInitialPoseFromElevatorAnchor: missing floor: ", fl.error());
  if (!anc)  throw BT::RuntimeError("SetInitialPoseFromElevatorAnchor: missing door_anchor: ", anc.error());

  const auto& by_lift = anc.value();
  auto itL = by_lift.find(lid.value());
  if (itL == by_lift.end()) {
    RCLCPP_ERROR(node->get_logger(), "[SetInitialPoseFromElevatorAnchor] no door_anchor for lift_id=%s", lid.value().c_str());
    return BT::NodeStatus::FAILURE;
  }
  auto itF = itL->second.find(fl.value());
  if (itF == itL->second.end()) {
    RCLCPP_ERROR(node->get_logger(), "[SetInitialPoseFromElevatorAnchor] no door_anchor for floor=%d", fl.value());
    return BT::NodeStatus::FAILURE;
  }
  const Pose2D pose = itF->second;

  auto pub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(topic, rclcpp::QoS(1).transient_local());
  auto msg = makeMsg(pose, frame);
  for (int i=0;i<3;i++){
    pub->publish(msg);
    rclcpp::sleep_for(50ms);
    rclcpp::spin_some(node);
  }

  if (waitS > 0.0) rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(waitS*1000)));
  RCLCPP_INFO(node->get_logger(), "[SetInitialPoseFromElevatorAnchor] set initial pose at floor=%d", fl.value());
  return BT::NodeStatus::SUCCESS;
}

} // namespace floor_patrol
