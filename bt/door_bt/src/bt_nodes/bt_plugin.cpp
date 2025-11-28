#include <behaviortree_cpp_v3/bt_factory.h>
#include "door_bt/bt_nodes/check_path_uses_door.hpp"
#include "door_bt/bt_nodes/is_robot_in_door_radius.hpp" 
#include "door_bt/bt_nodes/is_robot_past_door.hpp"
#include "door_bt/bt_nodes/set_door_open.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<CheckPathUsesDoor>("CheckPathUsesDoor");
  factory.registerNodeType<IsRobotInDoorRadius>("IsRobotInDoorRadius");
  factory.registerNodeType<IsRobotPastDoor>("IsRobotPastDoor");     
  factory.registerNodeType<SetDoorOpen>("SetDoorOpen");        
}