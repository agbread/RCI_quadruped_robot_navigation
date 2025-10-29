#include <behaviortree_cpp_v3/bt_factory.h>
#include "floor_patrol/bt_nodes/foreach_floor.hpp"
#include "floor_patrol/bt_nodes/compute_plan.hpp"   
#include "floor_patrol/bt_nodes/follow_waypoints.hpp" 
#include "floor_patrol/bt_nodes/choose_lift.hpp"
#include "floor_patrol/bt_nodes/go_to_elevator_front.hpp"
#include "floor_patrol/bt_nodes/is_door_open.hpp"
#include "floor_patrol/bt_nodes/is_door_progress.hpp"
#include "floor_patrol/bt_nodes/wait_door_open.hpp"
#include "floor_patrol/bt_nodes/open_door_if_possible.hpp"
#include "floor_patrol/bt_nodes/wait_door_closed.hpp"
#include "floor_patrol/bt_nodes/publish_target_floor.hpp"
#include "floor_patrol/bt_nodes/wait_elevator_arrived.hpp"
#include "floor_patrol/bt_nodes/map_switch_slam_toolbox.hpp"
#include "floor_patrol/bt_nodes/enter_cabin_offset.hpp"
#include "floor_patrol/bt_nodes/exit_cabin.hpp"
#include "floor_patrol/bt_nodes/set_initial_pose_from_elevator_anchor.hpp"

using namespace floor_patrol;

// 이 파일에서 앞으로 다른 노드들도 registerNodeType으로 계속 추가할 예정
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ForEachFloor>("ForEachFloor");
  factory.registerNodeType<ComputePlan>("ComputePlan");  
  factory.registerNodeType<FollowWaypointsNode>("FollowWaypoints");  
  factory.registerNodeType<ChooseLift>("ChooseLift"); 
  factory.registerNodeType<GoToElevatorFront>("GoToElevatorFront");
  factory.registerNodeType<IsDoorOpen>("IsDoorOpen");
  factory.registerNodeType<IsDoorProgress>("IsDoorProgress");
  factory.registerNodeType<WaitDoorOpen>("WaitDoorOpen");
  factory.registerNodeType<OpenDoorIfPossible>("OpenDoorIfPossible");
  factory.registerNodeType<WaitDoorClosed>("WaitDoorClosed");
  factory.registerNodeType<PublishTargetFloor>("PublishTargetFloor");
  factory.registerNodeType<WaitElevatorArrived>("WaitElevatorArrived");
  factory.registerNodeType<MapSwitchSlamToolbox>("MapSwitchSlamToolbox");
  factory.registerNodeType<EnterCabinOffset>("EnterCabinOffset");
  factory.registerNodeType<ExitCabin>("ExitCabin");
  factory.registerNodeType<SetInitialPoseFromElevatorAnchor>("SetInitialPoseFromElevatorAnchor");


}
