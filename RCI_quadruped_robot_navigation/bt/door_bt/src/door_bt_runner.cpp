#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>


#include "door_bt/bt_nodes/door_plan_controller.hpp"  

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("door_bt_runner");

  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<DoorPlanController>("DoorPlanController");

  std::string tree_file =
    "/home/home/ros2_ws/src/RCI_quadruped_robot_navigation/bt/door_bt/bt_trees/tests/door_plan_controller_L1.xml";

  RCLCPP_INFO(node->get_logger(), "Using BT XML: %s", tree_file.c_str());

  auto tree = factory.createTreeFromFile(tree_file);

  rclcpp::Rate rate(10.0);
  while (rclcpp::ok())
  {
    tree.tickRoot();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
