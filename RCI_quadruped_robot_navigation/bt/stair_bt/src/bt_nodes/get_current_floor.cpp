#include "stair_bt/bt_nodes/get_current_floor.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace stair_bt
{

GetCurrentFloor::GetCurrentFloor(
    const std::string & name,
    const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  node_ = std::make_shared<rclcpp::Node>("get_current_floor");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::NodeStatus GetCurrentFloor::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "[GetCurrentFloor] onStart");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetCurrentFloor::onRunning()
{
  std::string map_frame = "map";
  std::string base_frame = "base_link";
  getInput<std::string>("map_frame", map_frame);
  getInput<std::string>("base_frame", base_frame);

  double f0 = 0.0, f1 = 5.0, f2 = 10.0;
  getInput<double>("floor0_z", f0);
  getInput<double>("floor1_z", f1);
  getInput<double>("floor2_z", f2);

  geometry_msgs::msg::TransformStamped tf;

  try {
    tf = tf_buffer_->lookupTransform(
      map_frame, base_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(),
      1000,
      "[GetCurrentFloor] TF %s->%s 아직 없음: %s",
      map_frame.c_str(), base_frame.c_str(), ex.what());

    return BT::NodeStatus::RUNNING;
  }

  double z = tf.transform.translation.z;

  double b01 = 0.5 * (f0 + f1);
  double b12 = 0.5 * (f1 + f2);

  int floor = 0;
  if (z < f1) {
    floor = 0;
  } else if (z < f2) {
    floor = 1;
  } else {
    floor = 2;
  }

  RCLCPP_INFO(node_->get_logger(),
              "[GetCurrentFloor] z=%.3f -> floor=%d", z, floor);

  setOutput<int>("current_floor", floor);
  return BT::NodeStatus::SUCCESS;
}

void GetCurrentFloor::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "[GetCurrentFloor] halted");
}

}  // namespace stair_bt
