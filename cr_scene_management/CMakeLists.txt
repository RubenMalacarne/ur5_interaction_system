cmake_minimum_required(VERSION 3.8)
project(cr_scene_management)

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
find_package(ament_index_cpp REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(cr_interfaces REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(shape_msgs REQUIRED)

# -------------------------------------------------------------------
# Include directories
# -------------------------------------------------------------------
include_directories(
  include
)

# -------------------------------------------------------------------
# Sources for component nodes
# -------------------------------------------------------------------
set(component_sources
    src/scene_manager.cpp
    src/static_scene_publisher.cpp
    src/planning_scene_modifier.cpp
)

# -------------------------------------------------------------------
# Create library for the nodes (components)
# -------------------------------------------------------------------
add_library(${PROJECT_NAME}_components SHARED ${component_sources})
ament_target_dependencies(${PROJECT_NAME}_components
  rclcpp
  rclcpp_components
  ament_index_cpp
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometric_shapes
  shape_msgs
  cr_interfaces
)

# Registrazione dei nodi componenti (assicurarsi che i namespace e nomi dei plugin coincidano con quelli definiti nei file sorgenti)
rclcpp_components_register_nodes(${PROJECT_NAME}_components
  "cr::scene_management::StaticScenePublisher"
  "cr::scene_management::PlanningSceneModifier"
)

# -------------------------------------------------------------------
# Installation
# -------------------------------------------------------------------
install(TARGETS
  ${PROJECT_NAME}_components
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export the package configuration
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
