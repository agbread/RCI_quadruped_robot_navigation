cmake_minimum_required(VERSION 3.8)
project(floor_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)

include_directories(
  include
)

add_library(floor_patrol_bt SHARED
  src/bt_nodes/foreach_floor.cpp
  src/bt_nodes/compute_plan.cpp
  src/bt_nodes/follow_waypoints.cpp   
  src/bt_nodes/choose_lift.cpp
  src/bt_nodes/go_to_elevator_front.cpp     
  src/bt_nodes/is_door_open.cpp          
  src/bt_nodes/is_door_progress.cpp      
  src/bt_nodes/wait_door_open.cpp  
  src/bt_nodes/open_door_if_possible.cpp      
  src/bt_nodes/wait_door_closed.cpp           
  src/bt_nodes/publish_target_floor.cpp        
  src/bt_nodes/wait_elevator_arrived.cpp
  src/bt_nodes/map_switch_slam_toolbox.cpp
  src/bt_nodes/enter_cabin_offset.cpp
  src/bt_nodes/exit_cabin.cpp
  src/bt_nodes/set_initial_pose_from_elevator_anchor.cpp

  src/bt_nodes/bt_plugin.cpp

  )

ament_target_dependencies(floor_patrol_bt
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  geometry_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  slam_toolbox
)

# plugin 이름을 런타임에서 찾을 수 있게 export
ament_export_libraries(floor_patrol_bt)

install(TARGETS floor_patrol_bt
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
