#pragma once

#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <mutex>
#include <string>

// 경로가 특정 문 하단 선분을 실제로 교차하면 SUCCESS
class CheckPathUsesDoor : public BT::ConditionNode
{
public:
  CheckPathUsesDoor(const std::string& name,
                    const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

  // 선분 교차 여부 (2D)
  static bool segmentsIntersect(double p1x, double p1y,
                                double p2x, double p2y,
                                double q1x, double q1y,
                                double q2x, double q2y);

  // 내부 유틸
  static int orientation(double ax, double ay,
                         double bx, double by,
                         double cx, double cy);

  static bool onSegment(double ax, double ay,
                        double bx, double by,
                        double cx, double cy);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;

  nav_msgs::msg::Path::SharedPtr last_path_;
  mutable std::mutex path_mutex_;

  std::string path_topic_{"/planned_path"};  // nav2 쓸 때는 global/local path로 바꿀 예정
};
