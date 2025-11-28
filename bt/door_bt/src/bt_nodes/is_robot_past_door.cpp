#include "door_bt/bt_nodes/is_robot_past_door.hpp"

#include <tf2/exceptions.h>
#include <cmath>

IsRobotPastDoor::IsRobotPastDoor(
  const std::string& name,
  const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("bt_is_robot_past_door");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList IsRobotPastDoor::providedPorts()
{
  return {
    BT::InputPort<double>("door_c_x"),
    BT::InputPort<double>("door_c_y"),
    BT::InputPort<double>("n_x"),   // 법선 벡터
    BT::InputPort<double>("n_y")
  };
}

bool IsRobotPastDoor::getRobotPosition(double& x, double& y)
{
  // TF 콜백 처리
  rclcpp::spin_some(node_);

  try
  {
    auto tf = tf_buffer_->lookupTransform(
      world_frame_, robot_frame_, tf2::TimePointZero);

    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "IsRobotPastDoor: TF lookup failed (%s)", ex.what());
    return false;
  }
}

BT::NodeStatus IsRobotPastDoor::tick()
{
  double cx, cy, nx, ny;
  if (!getInput("door_c_x", cx) ||
      !getInput("door_c_y", cy) ||
      !getInput("n_x", nx) ||
      !getInput("n_y", ny))
  {
    RCLCPP_ERROR(node_->get_logger(),
      "IsRobotPastDoor: missing input ports");
    return BT::NodeStatus::FAILURE;
  }

  double rx, ry;
  if (!getRobotPosition(rx, ry))
  {
    return BT::NodeStatus::FAILURE;
  }

  // (P - C) · n 의 부호로 앞/뒤 판별
  const double dx = rx - cx;
  const double dy = ry - cy;
  const double s  = dx * nx + dy * ny;

  if (s > 0.0)
  {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "IsRobotPastDoor[%s]: robot is past the door (s=%.3f)",
      this->name().c_str(), s);
    return BT::NodeStatus::SUCCESS;   // 이미 문 '뒤쪽'
  }
  else
  {
    return BT::NodeStatus::FAILURE;   // 아직 문 '앞쪽' 또는 정확히 선 위
  }
}
