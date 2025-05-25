cmake_minimum_required(VERSION 3.8)
project(cr_bt_orchestrator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(cr_interfaces REQUIRED)
find_package(cr_motion_core REQUIRED)
find_package(cr_bt_pick_place REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cr_bt_common REQUIRED)


include_directories(
  include
)

add_executable(orchestrator
  src/orchestrator_node.cpp
  src/nodes.cpp
  src/orchestrator_node_main.cpp)

ament_target_dependencies(orchestrator
  rclcpp
  behaviortree_cpp
  behaviortree_ros2
  cr_interfaces
  ament_index_cpp
  rclcpp_action
  cr_motion_core
  cr_bt_pick_place
  cr_bt_common
)

# --- Installazione ---
install(TARGETS
  orchestrator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  bt_xml/
  DESTINATION share/${PROJECT_NAME}/bt_xml
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
