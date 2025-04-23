cmake_minimum_required(VERSION 3.8)
project(cr_hw_configuration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/coppelia_sim_hardware_interface.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
)


# add_executable(scene_publisher src/scene_publisher.cpp)
# ament_target_dependencies(scene_publisher
#   rclcpp
#   moveit_core
#   moveit_ros_planning
#   moveit_msgs
#   geometry_msgs
#   shape_msgs
#   geometric_shapes
# )

pluginlib_export_plugin_description_file(hardware_interface coppelia_description.xml)


install(TARGETS ${PROJECT_NAME}
  # scene_publisher
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Installa i file header
install(
  DIRECTORY include/
  DESTINATION include
)

# Installa il file XML del plugin
install(FILES coppelia_description.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(test_load_coppelia_sim_hardware_interface
    test/test_load_coppelia_sim_hardware_interface.cpp
  )

  target_link_libraries(test_load_coppelia_sim_hardware_interface
    ${PROJECT_NAME}
  )
endif()

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
