#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>

// 우리가 만든 BT 노드 헤더
#include "door_bt/bt_nodes/check_path_uses_door.hpp"
#include "door_bt/bt_nodes/is_robot_in_door_radius.hpp"
#include "door_bt/bt_nodes/is_robot_past_door.hpp"
#include "door_bt/bt_nodes/set_door_open.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("door_bt_runner");

  // 1) BT 팩토리 생성
  BT::BehaviorTreeFactory factory;

  // 2) 플러그인(dynamic load) 말고, 그냥 직접 노드 타입 등록
  factory.registerNodeType<CheckPathUsesDoor>("CheckPathUsesDoor");
  factory.registerNodeType<IsRobotInDoorRadius>("IsRobotInDoorRadius");
  factory.registerNodeType<IsRobotPastDoor>("IsRobotPastDoor");     
  factory.registerNodeType<SetDoorOpen>("SetDoorOpen");   

  std::string tree_file =
    "/home/home/ros2_ws/src/RCI_quadruped_robot_navigation/bt/door_bt/bt_trees/tests/door_open_close_3doors.xml";

  RCLCPP_INFO(node->get_logger(), "Using BT XML: %s", tree_file.c_str());

  // 4) 트리 생성
  auto tree = factory.createTreeFromFile(tree_file);

  // 5) 루프 돌면서 tick + ROS 콜백 처리
  rclcpp::Rate rate(10.0);  // 10 Hz
  while (rclcpp::ok())
  {
    tree.tickRoot();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
