#include "door_bt/bt_nodes/is_robot_in_door_radius.hpp"

#include <tf2/exceptions.h>
#include <cmath>

IsRobotInDoorRadius::IsRobotInDoorRadius(
  const std::string& name,
  const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("bt_is_robot_in_door_radius");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList IsRobotInDoorRadius::providedPorts()
{
  return {
    BT::InputPort<double>("door_c_x"),
    BT::InputPort<double>("door_c_y"),
    BT::InputPort<double>("radius")
    // 필요하면 나중에 world_frame, robot_frame 포트도 추가 가능
  };
}

bool IsRobotInDoorRadius::getRobotPosition(double& x, double& y)
{
  // 이 노드도 콜백 처리를 위해 spin_some
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
      "IsRobotInDoorRadius: TF lookup failed (%s)", ex.what());
    return false;
  }
}

BT::NodeStatus IsRobotInDoorRadius::tick()
{
  double cx, cy, radius;
  if (!getInput("door_c_x", cx) ||
      !getInput("door_c_y", cy) ||
      !getInput("radius", radius))
  {
    RCLCPP_ERROR(node_->get_logger(),
      "IsRobotInDoorRadius: missing input ports");
    return BT::NodeStatus::FAILURE;
  }

  double rx, ry;
  if (!getRobotPosition(rx, ry))
  {
    // 로봇 위치 못 얻으면 일단 FAILURE
    return BT::NodeStatus::FAILURE;
  }

  const double dx = rx - cx;
  const double dy = ry - cy;
  const double dist2 = dx*dx + dy*dy;
  const double r2 = radius * radius;

  if (dist2 <= r2)
  {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "IsRobotInDoorRadius[%s]: inside radius (d=%.2f, r=%.2f)",
      this->name().c_str(), std::sqrt(dist2), radius);
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    // 필요하면 DEBUG용으로 찍고, 지금은 조용히 FAILURE
    // RCLCPP_DEBUG(node_->get_logger(), ...);
    return BT::NodeStatus::FAILURE;
  }
}
