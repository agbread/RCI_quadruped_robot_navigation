#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>                 // 확실하지 않음: cmd_floor가 Int32라 가정
#include <std_msgs/msg/float64.hpp>              // 확실하지 않음: cmd_z, cabin_z
#include <std_msgs/msg/bool.hpp>                 // 확실하지 않음: door_open
#include <std_msgs/msg/float64_multi_array.hpp>  // 확실: door_pos는 Float64MultiArray로 수신됨
#include <atomic>
#include <mutex>
#include <string>
#include <vector>
#include <optional>

namespace elevator_bt {

class ElevatorClient {
public:
  explicit ElevatorClient(const rclcpp::Node::SharedPtr& node);

  // 네임스페이스 설정 및 모든 Pub/Sub 초기화
  void setNamespace(const std::string& ns);

  // 명령(확실하지 않음: 메시지 타입 추정)
  void cmdFloor(int floor_idx);     // /<ns>/cmd_floor : std_msgs/Int32
  void cmdZ(double z_m);            // /<ns>/cmd_z     : std_msgs/Float64
  void setDoorOpen(bool open);      // /<ns>/door_open : std_msgs/Bool

  // 상태 조회
  double cabinZ() const;                                // NaN이면 아직 미수신
  std::vector<double> doorPos() const;                  // 비어있으면 아직 미수신
  double doorOpenExtent() const;                        // |L|,|R| 평균. NaN이면 미수신
  double doorA() const;                                 // 좌문(가정)
  double doorB() const;                                 // 우문(가정)

private:
  rclcpp::Node::SharedPtr node_;
  std::string ns_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr            pub_cmd_floor_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr          pub_cmd_z_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr             pub_door_open_;

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr             sub_cabin_z_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   sub_door_pos_;

  // Latest states
  std::atomic<double> cabin_z_{std::numeric_limits<double>::quiet_NaN()};
  std::atomic<double> door_a_{std::numeric_limits<double>::quiet_NaN()};
  std::atomic<double> door_b_{std::numeric_limits<double>::quiet_NaN()};

  mutable std::mutex mtx_door_vec_;
  std::vector<double> door_vec_; // 원본 벡터도 보관(디버깅/필요 시)
};

} // namespace elevator_bt
