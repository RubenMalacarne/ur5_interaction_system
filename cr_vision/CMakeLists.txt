cmake_minimum_required(VERSION 3.8)
project(cr_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(cr_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)

# -------------------------------------------------------------------
# Executable
# -------------------------------------------------------------------
add_executable(object_state_manager
  src/object_state_manager_main.cpp
  src/object_state_manager.cpp
)

target_include_directories(object_state_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


ament_target_dependencies(object_state_manager
  ament_cmake
  rclcpp
  cr_interfaces
)

add_executable(human_proximity_monitor
  src/human_proximity_monitor_main.cpp
  src/human_proximity_monitor.cpp
)

target_include_directories(human_proximity_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


ament_target_dependencies(human_proximity_monitor
  ament_cmake
  rclcpp
  geometry_msgs
  std_msgs
  tf2_ros
)

# -------------------------------------------------------------------
# Install
# -------------------------------------------------------------------
install(
  PROGRAMS
    cr_vision/mirror_scena.py 
    cr_vision/obj_detection.py
    cr_vision/pose_estimation_person.py
    cr_vision/obj_selection.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS 
  object_state_manager
  human_proximity_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
