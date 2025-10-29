#include "floor_patrol/bt_nodes/is_door_open.hpp"

namespace floor_patrol {

IsDoorOpen::IsDoorOpen(const std::string& name, const BT::NodeConfiguration& cfg)
: BT::ConditionNode(name, cfg) {}

BT::PortsList IsDoorOpen::providedPorts() {
  return {
    BT::InputPort<std::string>("door_pos_topic_key"),
    BT::InputPort<std::string>("lift_id"),
    BT::InputPort<double>("open_th"),
    BT::InputPort<double>("close_th"),
    BT::InputPort<double>("eps_equal", 0.01, "tolerance for equality checks")
  };
}
rclcpp::Node::SharedPtr IsDoorOpen::getOrMakeNode_(){
  if (node_) return node_;
  node_ = rclcpp::Node::make_shared("bt_is_door_open");
  return node_;
}

std::string IsDoorOpen::topicFrom(const std::string& lift_id, const std::string& key) const {
  // "/lift1/door_pos"
  return "/"+ lift_id +"/"+ key;
}

bool IsDoorOpen::ensureSubTo(const std::string& topic){
  auto node = getOrMakeNode_();
  if (current_topic_ == topic && sub_) return true;

  current_topic_.clear();
  sub_.reset();

  auto cb = [this](const std_msgs::msg::Float64MultiArray& msg){
    if (msg.data.size() >= 2){
      std::lock_guard<std::mutex> lk(mtx_);
      last_lr_ = std::array<double,2>{ msg.data[0], msg.data[1] };
    }
  };
  sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      topic, rclcpp::QoS(10), cb);
  current_topic_ = topic;
  return true;
}

double IsDoorOpen::doorMeasure(const std::array<double,2>& lr){
  return std::min(lr[0], lr[1]);
}

BT::NodeStatus IsDoorOpen::tick() {
  auto node = getOrMakeNode_();

  const auto key = getInput<std::string>("door_pos_topic_key");
  const auto lid = getInput<std::string>("lift_id");
  const auto open_th  = getInput<double>("open_th");
  const auto close_th = getInput<double>("close_th");
  const auto eps      = getInput<double>("eps_equal").value();

  if (!key)  throw BT::RuntimeError("IsDoorOpen: missing door_pos_topic_key: ", key.error());
  if (!lid)  throw BT::RuntimeError("IsDoorOpen: missing lift_id: ", lid.error());
  if (!open_th)  throw BT::RuntimeError("IsDoorOpen: missing open_th: ", open_th.error());
  if (!close_th) throw BT::RuntimeError("IsDoorOpen: missing close_th: ", close_th.error());

  const std::string topic = topicFrom(lid.value(), key.value());
  ensureSubTo(topic);

  std::optional<std::array<double,2>> lr;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    lr = last_lr_;
  }
  if (!lr){
    // 아직 데이터 없음 → FAILURE로 처리 (열림 보장 못함)
    return BT::NodeStatus::FAILURE;
  }

  const double m = doorMeasure(*lr);
  // “열림” 판정: m >= open_th - eps
  if (m >= open_th.value() - eps) return BT::NodeStatus::SUCCESS;

  // “닫힘”에 가깝다면 확실히 실패
  if (m <= close_th.value() + eps) return BT::NodeStatus::FAILURE;

  // 중간값(애매) → 열림 보장 아님 → FAILURE
  return BT::NodeStatus::FAILURE;
}

} // namespace floor_patrol
