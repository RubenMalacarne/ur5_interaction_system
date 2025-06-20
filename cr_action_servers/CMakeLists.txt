cmake_minimum_required(VERSION 3.8)
project(cr_action_servers)

# -------------------------------------------------------------------
# Compiler flags
# -------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------------------------------------------
# Find dependencies
# -------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cr_interfaces REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

# -------------------------------------------------------------------
# Executable
# -------------------------------------------------------------------
add_executable(pick_action_server
  src/pick_action_server_main.cpp
  src/pick_action_server.cpp
)

target_include_directories(pick_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


ament_target_dependencies(pick_action_server
  ament_cmake
  tf2_geometry_msgs
  cr_interfaces
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
)


add_executable(place_action_server
  src/place_action_server_main.cpp
  src/place_action_server.cpp
)

target_include_directories(place_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


ament_target_dependencies(place_action_server
  ament_cmake
  tf2_geometry_msgs
  cr_interfaces
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
)

# -------------------------------------------------------------------
# Install
# -------------------------------------------------------------------
install(TARGETS 
  pick_action_server
  place_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# -------------------------------------------------------------------
# Final export
# -------------------------------------------------------------------
ament_package()