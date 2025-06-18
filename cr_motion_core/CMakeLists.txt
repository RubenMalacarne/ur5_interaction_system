cmake_minimum_required(VERSION 3.8)
project(cr_motion_core LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)


add_library(motion_commander
  src/motion_commander.cpp
)

set_target_properties(motion_commander PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

ament_target_dependencies(motion_commander
  rclcpp
  moveit_ros_planning_interface
)

target_include_directories(motion_commander
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


install(
  TARGETS motion_commander
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_libraries(motion_commander)


ament_export_dependencies(
  rclcpp
  moveit_ros_planning_interface
)

ament_export_include_directories(include)

ament_package()
