#include "floor_patrol/bt_nodes/open_door_if_possible.hpp"
#include <chrono>
using namespace std::chrono_literals;

namespace floor_patrol {

BT::PortsList OpenDoorIfPossible::providedPorts() {
  return {
    BT::InputPort<std::string>("door_cmd_topic_key"),                 // 예: "door_cmd"
    BT::InputPort<std::string>("lift_id"),                            // 예: "lift1"
    BT::InputPort<std::string>("value", std::string("true"),          // "true"|"false"
                               "desired door state: true=open false=close"),
    BT::InputPort<rclcpp::Node::SharedPtr>("ros_node")                // optional
  };
}

rclcpp::Node::SharedPtr OpenDoorIfPossible::getOrMakeNode_() {
  if (node_) return node_;
  if (auto n = getInput<rclcpp::Node::SharedPtr>("ros_node"); n && n.value()) {
    node_ = n.value();
  } else {
    node_ = rclcpp::Node::make_shared("bt_open_door_if_possible");
  }
  return node_;
}

std::string OpenDoorIfPossible::topicFrom(const std::string& lift_id, const std::string& key) const {
  return "/"+ lift_id +"/"+ key;
}

BT::NodeStatus OpenDoorIfPossible::tick() {
  auto node = getOrMakeNode_();

  const auto key = getInput<std::string>("door_cmd_topic_key");
  const auto lid = getInput<std::string>("lift_id");
  const auto v   = getInput<std::string>("value");

  if (!key) throw BT::RuntimeError("OpenDoorIfPossible: missing door_cmd_topic_key: ", key.error());
  if (!lid) throw BT::RuntimeError("OpenDoorIfPossible: missing lift_id: ", lid.error());
  if (!v)   throw BT::RuntimeError("OpenDoorIfPossible: missing value: ", v.error());

  const bool open = (v.value() == "true" || v.value() == "1");
  const std::string topic = topicFrom(lid.value(), key.value());

  auto pub = node->create_publisher<std_msgs::msg::Bool>(topic, rclcpp::QoS(1).transient_local());
  std_msgs::msg::Bool msg; msg.data = open;
  // 짧게 여러 번 퍼블리시(라치/게이트웨이 대응)
  for (int i=0;i<3;i++){
    pub->publish(msg);
    rclcpp::sleep_for(50ms);
    rclcpp::spin_some(node);
  }
  RCLCPP_INFO(node->get_logger(), "[OpenDoorIfPossible] %s -> %s",
              topic.c_str(), open ? "OPEN(true)" : "CLOSE(false)");
  return BT::NodeStatus::SUCCESS;
}

} // namespace floor_patrol
