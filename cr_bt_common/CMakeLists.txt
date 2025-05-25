cmake_minimum_required(VERSION 3.8)
project(cr_bt_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)

add_library(cr_bt_common_nodes SHARED
  src/check_blackboard_key_node.cpp
  src/log_message_node.cpp
)

target_include_directories(cr_bt_common_nodes
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(cr_bt_common_nodes
  PUBLIC
    rclcpp::rclcpp
    behaviortree_cpp::behaviortree_cpp
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
ament_package()
