#include "floor_patrol/bt_nodes/choose_lift.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <limits>

using namespace std::chrono_literals;

namespace floor_patrol {

BT::PortsList ChooseLift::providedPorts() {
  return {
    BT::InputPort<std::vector<std::string>>("lifts"),      // ["lift1","lift2",...]
    BT::InputPort<FrontMap>("elevator_front"),             // lift->floor->PoseStamped(map)
    BT::InputPort<int>("floor"),
    BT::OutputPort<std::string>("out_lift_id"),
    // 선택 포트: TF frame 이름 (기본 map/base_link)
    BT::InputPort<std::string>("map_frame",  std::string("map")),
    BT::InputPort<std::string>("base_frame", std::string("base_link")),
    // 선택 포트: 외부 ROS 노드 주입
    BT::InputPort<rclcpp::Node::SharedPtr>("ros_node")
  };
}

rclcpp::Node::SharedPtr ChooseLift::getOrMakeNode_(){
  if (node_) return node_;
  if (auto nres = getInput<rclcpp::Node::SharedPtr>("ros_node"); nres && nres.value()){
    node_ = nres.value();
  } else {
    node_ = rclcpp::Node::make_shared("bt_choose_lift_node");
  }
  // TF 준비(1회)
  if (!tf_buffer_){
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  }
  return node_;
}

bool ChooseLift::lookupBaseInMap_(double& x, double& y){
  auto node = getOrMakeNode_();
  const auto map_frame  = getInput<std::string>("map_frame").value();
  const auto base_frame = getInput<std::string>("base_frame").value();

  try{
    // 최신 transform (timeout 0.3s)
    geometry_msgs::msg::TransformStamped tf =
        tf_buffer_->lookupTransform(map_frame, base_frame, tf2::TimePointZero, 300ms);
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    return true;
  }catch(const tf2::TransformException& ex){
    RCLCPP_WARN(node->get_logger(), "[ChooseLift] TF lookup failed: %s", ex.what());
    return false;
  }
}

BT::NodeStatus ChooseLift::tick() {
  auto node = getOrMakeNode_();

  // 필수 입력
  const auto lifts_res  = getInput<std::vector<std::string>>("lifts");
  const auto front_res  = getInput<FrontMap>("elevator_front");
  const auto floor_res  = getInput<int>("floor");
  if (!lifts_res)  throw BT::RuntimeError("ChooseLift: missing lifts: ", lifts_res.error());
  if (!front_res)  throw BT::RuntimeError("ChooseLift: missing elevator_front: ", front_res.error());
  if (!floor_res)  throw BT::RuntimeError("ChooseLift: missing floor: ", floor_res.error());

  const auto& lifts = lifts_res.value();
  const auto& front = front_res.value();
  const int floor = floor_res.value();

  if (lifts.empty()){
    RCLCPP_ERROR(node->get_logger(), "[ChooseLift] lifts is empty");
    return BT::NodeStatus::FAILURE;
  }

  double rx=0.0, ry=0.0;
  if (!lookupBaseInMap_(rx, ry)){
    RCLCPP_ERROR(node->get_logger(), "[ChooseLift] cannot get robot pose in map");
    return BT::NodeStatus::FAILURE;
  }

  // 각 lift의 해당 층 포즈를 찾아 거리 계산
  double best_d2 = std::numeric_limits<double>::infinity();
  std::string best_id;

  for (const auto& id : lifts){
    auto itL = front.find(id);
    if (itL == front.end()){
      RCLCPP_WARN(node->get_logger(), "[ChooseLift] elevator_front has no entry for %s", id.c_str());
      continue;
    }
    const auto& perFloor = itL->second;
    auto itF = perFloor.find(floor);
    if (itF == perFloor.end()){
      RCLCPP_WARN(node->get_logger(), "[ChooseLift] elevator_front[%s] missing floor=%d", id.c_str(), floor);
      continue;
    }
    const auto& pose = itF->second.pose; // PoseStamped
    const double dx = pose.position.x - rx;
    const double dy = pose.position.y - ry;
    const double d2 = dx*dx + dy*dy;
    if (d2 < best_d2){
      best_d2 = d2;
      best_id = id;
    }
  }

  if (best_id.empty()){
    RCLCPP_ERROR(node->get_logger(), "[ChooseLift] no valid elevator_front entry for current floor=%d", floor);
    return BT::NodeStatus::FAILURE;
  }

  setOutput("out_lift_id", best_id);
  RCLCPP_INFO(node->get_logger(), "[ChooseLift] selected lift: %s (d=%.2f m)",
              best_id.c_str(), std::sqrt(best_d2));
  return BT::NodeStatus::SUCCESS;
}

} // namespace floor_patrol
