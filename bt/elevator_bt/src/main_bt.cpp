#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/blackboard.h>   // <-- 추가
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

#include "elevator_bt/bt_nodes/ok_node.hpp"
#include "elevator_bt/bt_nodes/call_elevator_to_floor.hpp"
#include <behaviortree_cpp_v3/blackboard.h>
#include "elevator_bt/bt_nodes/set_elevator_door.hpp"
#include "elevator_bt/bt_nodes/wait_door_open.hpp"
#include "elevator_bt/bt_nodes/wait_cabin_at_target.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("elevator_bt_runner");

  // 트리 파일 경로 결정
  std::string tree_path;
  for (int i = 1; i < argc - 1; ++i)
    if (std::string(argv[i]) == "--tree") { tree_path = argv[i + 1]; break; }
  if (tree_path.empty()) {
    const auto share = ament_index_cpp::get_package_share_directory("elevator_bt");
    tree_path = share + "/bt_trees/tests/t_call_elevator_pick.xml";
  }

  const auto share = ament_index_cpp::get_package_share_directory("elevator_bt");
  node->declare_parameter<std::string>("elev_yaml", share + "/config/elevator.yaml");
  node->declare_parameter<std::string>("elevator_ns", "/lift1");  // 기본값: lift1
  node->declare_parameter<int>("target_floor", 1);

  // ★ 추가: 실수 파라미터 선언 (timeout, door open threshold)
  node->declare_parameter<double>("call_timeout", 120.0);
  node->declare_parameter<double>("open_threshold", 0.5);

  std::string elev_yaml = node->get_parameter("elev_yaml").as_string();
  std::string elevator_ns = node->get_parameter("elevator_ns").as_string();
  int target_floor = node->get_parameter("target_floor").as_int();

  // ★ 추가: 값 읽기
  double call_timeout = node->get_parameter("call_timeout").as_double();
  double open_threshold = node->get_parameter("open_threshold").as_double();

  // 팩토리/등록 (그대로)
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<elevator_bt::Ok>("Ok");
  factory.registerNodeType<elevator_bt::CallElevatorToFloor>("CallElevatorToFloor");
  factory.registerNodeType<elevator_bt::SetElevatorDoor>("SetElevatorDoor");
  factory.registerNodeType<elevator_bt::WaitDoorOpen>("WaitDoorOpen");
  factory.registerNodeType<elevator_bt::WaitCabinAtTarget>("WaitCabinAtTarget");
  BT::Tree tree = factory.createTreeFromFile(tree_path);

  // 블랙보드 주입
  tree.rootBlackboard()->set("elev_yaml", elev_yaml);
  tree.rootBlackboard()->set("elevator_ns", elevator_ns); // "/lift1" 또는 "/lift2"
  tree.rootBlackboard()->set("target_floor", target_floor);

  // ★ 추가: 실수 키들도 동일 방식으로 주입
  tree.rootBlackboard()->set("call_timeout", call_timeout);
  tree.rootBlackboard()->set("open_threshold", open_threshold);

  // 루프
  rclcpp::WallRate rate(20.0);
  while (rclcpp::ok())
  {
    auto status = tree.tickRoot();
    rclcpp::spin_some(node);
    if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) break;
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}