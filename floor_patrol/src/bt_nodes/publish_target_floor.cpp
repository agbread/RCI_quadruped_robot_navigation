#include "floor_patrol/bt_nodes/publish_target_floor.hpp"
#include <chrono>
using namespace std::chrono_literals;

namespace floor_patrol {

BT::PortsList PublishTargetFloor::providedPorts() {
  return {
    BT::InputPort<std::string>("cmd_floor_topic_key"),     // 예: "cmd_floor"
    BT::InputPort<std::string>("lift_id"),                 // "lift1"
    BT::InputPort<int>("next_floor"),
    BT::InputPort<rclcpp::Node::SharedPtr>("ros_node")     // optional
  };
}

rclcpp::Node::SharedPtr PublishTargetFloor::getOrMakeNode_(){
  if (node_) return node_;
  if (auto n = getInput<rclcpp::Node::SharedPtr>("ros_node"); n && n.value()){
    node_ = n.value();
  } else {
    node_ = rclcpp::Node::make_shared("bt_publish_target_floor");
  }
  return node_;
}

std::string PublishTargetFloor::topicFrom(const std::string& lift_id, const std::string& key) const {
  return "/"+lift_id+"/"+key;
}

BT::NodeStatus PublishTargetFloor::tick() {
  auto node = getOrMakeNode_();

  const auto key = getInput<std::string>("cmd_floor_topic_key");
  const auto lid = getInput<std::string>("lift_id");
  const auto nf  = getInput<int>("next_floor");
  if (!key) throw BT::RuntimeError("PublishTargetFloor: missing cmd_floor_topic_key: ", key.error());
  if (!lid) throw BT::RuntimeError("PublishTargetFloor: missing lift_id: ", lid.error());
  if (!nf)  throw BT::RuntimeError("PublishTargetFloor: missing next_floor: ", nf.error());

  const std::string topic = topicFrom(lid.value(), key.value());
  auto pub = node->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(1).transient_local());

  std_msgs::msg::Int32 msg; msg.data = nf.value();
  for (int i=0;i<3;i++){
    pub->publish(msg);
    rclcpp::sleep_for(30ms);
    rclcpp::spin_some(node);
  }
  RCLCPP_INFO(node->get_logger(), "[PublishTargetFloor] %s -> next_floor=%d",
              topic.c_str(), msg.data);
  return BT::NodeStatus::SUCCESS;
}

} // namespace floor_patrol
