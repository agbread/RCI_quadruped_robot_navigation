#include "floor_patrol/bt_nodes/wait_elevator_arrived.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace floor_patrol {

BT::PortsList WaitElevatorArrived::providedPorts() {
  return {
    BT::InputPort<std::string>("lift_id"),
    BT::InputPort<std::string>("cabin_z_topic_key"),           // 예: "cabin_z"
    BT::InputPort<std::vector<double>>("floor_heights"),       // 예: [0.0, 8.0, 16.0]
    BT::InputPort<double>("z_tol"),                            // 예: 0.03
    BT::InputPort<double>("timeout"),                          // 예: 60.0 (sec)
    BT::InputPort<int>("next_floor")                           // 블랙보드/속성 어느 쪽이든 OK
  };
}

rclcpp::Node::SharedPtr WaitElevatorArrived::getOrMakeNode_() {
  if (node_) return node_;
  node_ = rclcpp::Node::make_shared("bt_wait_elevator_arrived");
  return node_;
}

std::string WaitElevatorArrived::topicFrom(const std::string& lift_id, const std::string& key) const {
  return "/"+lift_id+"/"+key;
}

bool WaitElevatorArrived::ensureSubTo(const std::string& topic) {
  auto node = getOrMakeNode_();
  if (current_topic_ == topic && sub_) return true;

  current_topic_.clear();
  sub_.reset();
  last_z_.reset();

  auto cb = [this](const std_msgs::msg::Float64& msg){
    std::lock_guard<std::mutex> lk(mtx_);
    last_z_ = msg.data;
  };
  sub_ = node->create_subscription<std_msgs::msg::Float64>(topic, rclcpp::QoS(10), cb);
  current_topic_ = topic;
  return true;
}

BT::NodeStatus WaitElevatorArrived::tick() {
  auto node = getOrMakeNode_();

  const auto lid   = getInput<std::string>("lift_id");
  const auto key   = getInput<std::string>("cabin_z_topic_key");
  const auto fhs   = getInput<std::vector<double>>("floor_heights");
  const auto ztol  = getInput<double>("z_tol");
  const auto tout  = getInput<double>("timeout");
  const auto nf    = getInput<int>("next_floor");  // XML에 명시가 없어도 블랙보드에 있으면 읽힘

  if (!lid)  throw BT::RuntimeError("WaitElevatorArrived: missing lift_id: ", lid.error());
  if (!key)  throw BT::RuntimeError("WaitElevatorArrived: missing cabin_z_topic_key: ", key.error());
  if (!fhs)  throw BT::RuntimeError("WaitElevatorArrived: missing floor_heights: ", fhs.error());
  if (!ztol) throw BT::RuntimeError("WaitElevatorArrived: missing z_tol: ", ztol.error());
  if (!tout) throw BT::RuntimeError("WaitElevatorArrived: missing timeout: ", tout.error());
  if (!nf)   throw BT::RuntimeError("WaitElevatorArrived: missing next_floor: ", nf.error());

  const int next_floor = nf.value();
  const auto& heights = fhs.value();
  if (next_floor < 0 || next_floor >= static_cast<int>(heights.size())) {
    RCLCPP_ERROR(node->get_logger(), "[WaitElevatorArrived] invalid next_floor=%d", next_floor);
    return BT::NodeStatus::FAILURE;
  }

  const double target_z = heights[next_floor];
  const std::string topic = topicFrom(lid.value(), key.value());
  ensureSubTo(topic);

  const rclcpp::Time t0 = node->get_clock()->now();
  rclcpp::Rate rate(30.0);

  while (rclcpp::ok()) {
    if ((node->get_clock()->now() - t0).seconds() > tout.value()) {
      RCLCPP_ERROR(node->get_logger(), "[WaitElevatorArrived] timeout %.2fs (target_z=%.3f)",
                   tout.value(), target_z);
      return BT::NodeStatus::FAILURE;
    }

    std::optional<double> z;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      z = last_z_;
    }
    if (z) {
      const double err = std::fabs(z.value() - target_z);
      if (err <= ztol.value()) {
        RCLCPP_INFO(node->get_logger(), "[WaitElevatorArrived] arrived: z=%.3f ≈ %.3f (tol=%.3f)",
                    z.value(), target_z, ztol.value());
        return BT::NodeStatus::SUCCESS;
      }
    }

    rclcpp::spin_some(node);
    rate.sleep();
  }
  return BT::NodeStatus::FAILURE;
}

} // namespace floor_patrol
