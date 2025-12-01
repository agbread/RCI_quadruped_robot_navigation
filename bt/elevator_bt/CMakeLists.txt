cmake_minimum_required(VERSION 3.8)
project(elevator_bt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)    # ← 중요: find_package

add_executable(elevator_bt_runner
  src/main_bt.cpp
  src/bt_nodes/ok_node.cpp
  src/bt_nodes/elevator_client.cpp
  src/bt_nodes/call_elevator_to_floor.cpp
  src/bt_nodes/set_elevator_door.cpp          # ← 추가
  src/bt_nodes/wait_door_open.cpp             # ← 추가
  src/bt_nodes/wait_cabin_at_target.cpp  
)

target_include_directories(elevator_bt_runner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(elevator_bt_runner
  rclcpp
  behaviortree_cpp_v3
  std_msgs
  # yaml-cpp 는 여기 쓰지 마세요 (링크 안 걸림)
)

# ✅ yaml-cpp는 target_link_libraries 로 연결
if(TARGET yaml-cpp)
  target_link_libraries(elevator_bt_runner yaml-cpp)
else()  # 드물게 구형 패키지환경일 때의 대비책 (확실하지 않음)
  find_package(YAML-CPP REQUIRED)
  target_link_libraries(elevator_bt_runner ${YAML_CPP_LIBRARIES})
endif()

install(TARGETS elevator_bt_runner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config/
  DESTINATION config)

ament_package()
