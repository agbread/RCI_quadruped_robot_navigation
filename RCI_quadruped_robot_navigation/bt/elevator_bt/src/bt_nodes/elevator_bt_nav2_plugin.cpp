#include <behaviortree_cpp_v3/bt_factory.h>
#include "elevator_bt/bt_nodes/ok_node.hpp"
#include "elevator_bt/bt_nodes/call_elevator_to_floor.hpp"
#include "elevator_bt/bt_nodes/set_elevator_door.hpp"
#include "elevator_bt/bt_nodes/wait_door_open.hpp"
#include "elevator_bt/bt_nodes/wait_cabin_at_target.hpp"
#include "elevator_bt/bt_nodes/wait_robot_near_elevator.hpp"
#include "elevator_bt/bt_nodes/wait_robot_inside_cabin.hpp"
#include "elevator_bt/bt_nodes/wait_robot_outside_elevator.hpp"
#include "elevator_bt/bt_nodes/get_current_floor.hpp"
#include "elevator_bt/bt_nodes/select_nearest_elevator.hpp"
#include "elevator_bt/bt_nodes/get_cabin_goal.hpp"
#include "elevator_bt/bt_nodes/wait_floor_command.hpp"

using namespace elevator_bt;

BT_REGISTER_NODES(factory)
{
  // XML에서 쓰는 태그 이름이랑 **완전히 동일**해야 함
  factory.registerNodeType<Ok>("Ok");

  factory.registerNodeType<CallElevatorToFloor>("CallElevatorToFloor");
  factory.registerNodeType<SetElevatorDoor>("SetElevatorDoor");
  factory.registerNodeType<WaitDoorOpen>("WaitDoorOpen");
  factory.registerNodeType<WaitCabinAtTarget>("WaitCabinAtTarget");

  factory.registerNodeType<WaitRobotNearElevator>("WaitRobotNearElevator");
  factory.registerNodeType<WaitRobotInsideCabin>("WaitRobotInsideCabin");
  factory.registerNodeType<WaitRobotOutsideElevator>("WaitRobotOutsideElevator");

  factory.registerNodeType<GetCurrentFloor>("GetCurrentFloor");
  factory.registerNodeType<SelectNearestElevator>("SelectNearestElevator");
  factory.registerNodeType<GetCabinGoal>("GetCabinGoal");
  factory.registerNodeType<WaitFloorCommand>("WaitFloorCommand");

}
