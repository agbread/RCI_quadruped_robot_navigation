#include "floor_patrol/bt_nodes/wait_door_open.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace floor_patrol {

BT::PortsList WaitDoorOpen::providedPorts() {
  return {
    BT::InputPort<std::string>("door_pos_topic_key"),
    BT::InputPort<std::string>("lift_id"),
    BT::InputPort<double>("open_th"),
    BT::InputPort<double>("close_th"),
    BT::InputPort<double>("vel_eps", 0.001, "velocity epsilon for progress guard"),
    BT::InputPort<double>("eps_equal", 0.01, "tolerance for equality checks"),
    BT::InputPort<double>("timeout")  // seconds
  };
}

rclcpp::Node::SharedPtr WaitDoorOpen::getOrMakeNode_(){
  if (node_) return node_;
  node_ = rclcpp::Node::make_shared("bt_wait_door_open");
  return node_;
}

std::string WaitDoorOpen::topicFrom(const std::string& lift_id, const std::string& key) const {
  return "/"+lift_id+"/"+key;
}

bool WaitDoorOpen::ensureSubTo(const std::string& topic){
  auto node = getOrMakeNode_();
  if (current_topic_ == topic && sub_) return true;

  current_topic_.clear();
  sub_.reset();
  last_lr_.reset();

  auto cb = [this](const std_msgs::msg::Float64MultiArray& msg){
    if (msg.data.size() >= 2){
      std::lock_guard<std::mutex> lk(mtx_);
      last_lr_ = std::array<double,2>{ msg.data[0], msg.data[1] };
    }
  };
  sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(topic, rclcpp::QoS(10), cb);
  current_topic_ = topic;
  return true;
}

double WaitDoorOpen::measure(const std::array<double,2>& lr){
  return std::min(lr[0], lr[1]);
}

BT::NodeStatus WaitDoorOpen::tick() {
  auto node = getOrMakeNode_();

  const auto key  = getInput<std::string>("door_pos_topic_key");
  const auto lid  = getInput<std::string>("lift_id");
  const auto open = getInput<double>("open_th");
  const auto eps  = getInput<double>("eps_equal");
  const auto tout = getInput<double>("timeout");

  if (!key)  throw BT::RuntimeError("WaitDoorOpen: missing door_pos_topic_key: ", key.error());
  if (!lid)  throw BT::RuntimeError("WaitDoorOpen: missing lift_id: ", lid.error());
  if (!open) throw BT::RuntimeError("WaitDoorOpen: missing open_th: ", open.error());
  if (!eps)  throw BT::RuntimeError("WaitDoorOpen: missing eps_equal: ", eps.error());
  if (!tout) throw BT::RuntimeError("WaitDoorOpen: missing timeout: ", tout.error());

  const std::string topic = topicFrom(lid.value(), key.value());
  ensureSubTo(topic);

  const rclcpp::Time t0 = node->get_clock()->now();
  rclcpp::Rate rate(20.0);

  while (rclcpp::ok()) {
    // 타임아웃
    if ((node->get_clock()->now() - t0).seconds() > tout.value()) {
      RCLCPP_ERROR(node->get_logger(), "[WaitDoorOpen] timeout %.2fs", tout.value());
      return BT::NodeStatus::FAILURE;
    }

    // 최신 측정
    std::optional<std::array<double,2>> lr;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      lr = last_lr_;
    }
    if (lr) {
      const double m = measure(*lr);
      if (m >= open.value() - eps.value()) {
        RCLCPP_INFO(node->get_logger(), "[WaitDoorOpen] door open m=%.3f", m);
        return BT::NodeStatus::SUCCESS;
      }
    }

    rclcpp::spin_some(node);
    rate.sleep();
  }
  return BT::NodeStatus::FAILURE;
}

} // namespace floor_patrol
