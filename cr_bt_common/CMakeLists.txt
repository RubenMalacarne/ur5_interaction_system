cmake_minimum_required(VERSION 3.8)
project(cr_bt_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(cr_bt_common_nodes SHARED
  src/check_blackboard_key_node.cpp
  src/log_message_node.cpp
  src/gui_log_node.cpp
)

target_include_directories(cr_bt_common_nodes
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# <<< qui: usa ament_target_dependencies invece di linkare 'std_msgs' a mano >>>
ament_target_dependencies(cr_bt_common_nodes
  rclcpp
  behaviortree_cpp      # il nome del pacchetto BehaviorTree.CPP
  std_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)

install(
  TARGETS cr_bt_common_nodes
  EXPORT cr_bt_common_nodes_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  EXPORT cr_bt_common_nodes_targets
  NAMESPACE cr_bt_common::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(include)
ament_export_libraries(cr_bt_common_nodes)
ament_export_dependencies(std_msgs)  # esporti anche la dipendenza
ament_package()
