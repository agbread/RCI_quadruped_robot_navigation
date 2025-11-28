#include "door_bt/bt_nodes/set_door_open.hpp"

SetDoorOpen::SetDoorOpen(
  const std::string& name,
  const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("bt_set_door_open");
}

BT::PortsList SetDoorOpen::providedPorts()
{
  return {
    BT::InputPort<std::string>("door_ns"),   // 예: "/doors/L1_door1"
    BT::InputPort<bool>("open")             // true = 열기, false = 닫기
  };
}

BT::NodeStatus SetDoorOpen::tick()
{
  // 콜백 처리
  rclcpp::spin_some(node_);

  std::string door_ns;
  bool open = false;

  if (!getInput("door_ns", door_ns) ||
      !getInput("open", open))
  {
    RCLCPP_ERROR(node_->get_logger(),
      "SetDoorOpen: missing input ports [door_ns / open]");
    return BT::NodeStatus::FAILURE;
  }

  // 토픽 이름 구성
  const std::string topic = door_ns + "/door_open";

  // 토픽이 바뀌면 새 publisher 생성
  if (!pub_ || topic != last_topic_)
  {
    RCLCPP_INFO(node_->get_logger(),
      "SetDoorOpen: creating publisher on [%s]", topic.c_str());

    pub_ = node_->create_publisher<std_msgs::msg::Bool>(topic, 10);
    last_topic_ = topic;
    // publisher 생성 직후 잠깐 기다릴 수도 있지만,
    // Gazebo 플러그인 쪽이 QoS 기본이면 바로 써도 보통 문제 없음.
  }

  std_msgs::msg::Bool msg;
  msg.data = open;
  pub_->publish(msg);

  RCLCPP_INFO(node_->get_logger(),
    "SetDoorOpen[%s]: %s -> %s",
    this->name().c_str(),
    topic.c_str(),
    open ? "OPEN" : "CLOSE");

  return BT::NodeStatus::SUCCESS;
}
