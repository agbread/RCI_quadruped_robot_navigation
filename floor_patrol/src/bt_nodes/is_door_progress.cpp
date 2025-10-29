#include "floor_patrol/bt_nodes/is_door_progress.hpp"

namespace floor_patrol {

IsDoorProgress::IsDoorProgress(const std::string& name, const BT::NodeConfiguration& cfg)
: BT::ConditionNode(name, cfg) {}

BT::PortsList IsDoorProgress::providedPorts() {
  return {
    BT::InputPort<std::string>("door_pos_topic_key"),
    BT::InputPort<std::string>("lift_id"),
    BT::InputPort<std::string>("mode"),  // "opening" | "closing"
    BT::InputPort<double>("vel_eps", 0.001, "velocity epsilon"),
    BT::InputPort<double>("open_th", 0.5,  "optional open threshold"),
    BT::InputPort<double>("close_th", 0.05, "optional close threshold")
  };
}

rclcpp::Node::SharedPtr IsDoorProgress::getOrMakeNode_(){
  if (node_) return node_;
  node_ = rclcpp::Node::make_shared("bt_is_door_progress");
  return node_;
}

std::string IsDoorProgress::topicFrom(const std::string& lift_id, const std::string& key) const {
  return "/"+lift_id+"/"+key;
}

bool IsDoorProgress::ensureSubTo(const std::string& topic){
  auto node = getOrMakeNode_();
  if (current_topic_ == topic && sub_) return true;

  current_topic_.clear();
  sub_.reset();
  buf_.clear();

  auto cb = [this, node](const std_msgs::msg::Float64MultiArray& msg){
    if (msg.data.size() >= 2){
      const double m = measure(msg.data);
      std::lock_guard<std::mutex> lk(mtx_);
      buf_.push_back({node->get_clock()->now(), m});
      while (buf_.size() > 6) buf_.pop_front();
    }
  };
  sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      topic, rclcpp::QoS(10), cb);
  current_topic_ = topic;
  return true;
}

double IsDoorProgress::measure(const std::vector<double>& v){
  return std::min(v[0], v[1]);
}

double IsDoorProgress::slope(const Sample& a, const Sample& b){
  const double dt = (b.t - a.t).seconds();
  if (dt <= 1e-6) return 0.0;
  return (b.m - a.m)/dt;
}

BT::NodeStatus IsDoorProgress::tick() {
  auto node = getOrMakeNode_();

  const auto key  = getInput<std::string>("door_pos_topic_key");
  const auto lid  = getInput<std::string>("lift_id");
  const auto mode = getInput<std::string>("mode");
  const auto velE = getInput<double>("vel_eps").value();

  if (!key)  throw BT::RuntimeError("IsDoorProgress: missing door_pos_topic_key: ", key.error());
  if (!lid)  throw BT::RuntimeError("IsDoorProgress: missing lift_id: ", lid.error());
  if (!mode) throw BT::RuntimeError("IsDoorProgress: missing mode: ", mode.error());

  const std::string topic = topicFrom(lid.value(), key.value());
  ensureSubTo(topic);

  std::deque<Sample> buf;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    buf = buf_;
  }
  if (buf.size() < 2) return BT::NodeStatus::FAILURE;

  // 최근 두 점으로 속도 추정 (필요시 평균화 가능)
  const double v = slope(buf[buf.size()-2], buf[buf.size()-1]);

  if (mode.value() == "opening") {
    return (v >  velE) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  } else if (mode.value() == "closing") {
    return (v < -velE) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  } else {
    throw BT::RuntimeError("IsDoorProgress: mode must be 'opening' or 'closing'");
  }
}

} // namespace floor_patrol
