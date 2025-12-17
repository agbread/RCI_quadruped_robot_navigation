#include "elevator_bt/bt_nodes/get_current_floor.hpp"

#include <tf2/time.h>
#include <tf2_ros/transform_listener.h>

namespace elevator_bt
{

GetCurrentFloor::GetCurrentFloor(const std::string & name,
                                 const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = std::make_shared<rclcpp::Node>("get_current_floor");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, true);
}

BT::PortsList GetCurrentFloor::providedPorts()
{
  return {
    BT::InputPort<std::string>("world_frame", "world"),
    BT::InputPort<std::string>("base_frame", "base_link"),
    BT::OutputPort<int>("start_floor")
  };
}

BT::NodeStatus GetCurrentFloor::tick()
{
  std::string world_frame = "world";
  std::string base_frame  = "base_link";
  getInput("world_frame", world_frame);
  getInput("base_frame", base_frame);

  std::string err;
  if (!tf_buffer_->canTransform(
        world_frame, base_frame, tf2::TimePointZero,
        tf2::durationFromSec(0.0), &err))
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *(node_->get_clock()), 2000,
      "[GetCurrentFloor] waiting for TF (%s -> %s): %s",
      world_frame.c_str(), base_frame.c_str(), err.c_str());

    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(world_frame, base_frame, tf2::TimePointZero);
  }
  catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[GetCurrentFloor] unexpected TF error (%s -> %s): %s",
      world_frame.c_str(), base_frame.c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }

  const double z = tf.transform.translation.z;

  int floor_idx = 0;
  if (z < 2.5) {
    floor_idx = 0;
  } else if (z < 7.5) {
    floor_idx = 1;
  } else {
    floor_idx = 2;
  }

  setOutput("start_floor", floor_idx);

  RCLCPP_INFO(node_->get_logger(),
    "[GetCurrentFloor] z=%.3f -> floor %d", z, floor_idx);

  return BT::NodeStatus::SUCCESS;
}


}  // namespace elevator_bt
