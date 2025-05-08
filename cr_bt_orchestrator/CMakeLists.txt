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
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
)

add_executable(task_planner src/task_planner.cpp src/custom_bt_nodes.cpp src/task_planner_main.cpp)
ament_target_dependencies(task_planner
  rclcpp
  behaviortree_cpp
  cr_interfaces
  ament_index_cpp
  rclcpp_action
  behaviortree_ros2
)

# --- Installazione ---
install(TARGETS
  task_planner
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
