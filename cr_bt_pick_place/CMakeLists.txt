cmake_minimum_required(VERSION 3.8)
project(cr_bt_pick_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(cr_motion_core REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(cr_interfaces REQUIRED)

# ------------------------------------------------------------
# 1) Creazione libreria di plugin
# ------------------------------------------------------------
add_library(cr_bt_pick_place_nodes SHARED
  src/arm_horizontal_move_node.cpp
  src/arm_vertical_move_node.cpp
  src/set_gripper_node.cpp
  src/set_collision_allowed_node.cpp
  src/set_object_attached_node.cpp
)

ament_target_dependencies(cr_bt_pick_place_nodes
  behaviortree_cpp
  cr_motion_core
  behaviortree_ros2
  cr_interfaces
)

# ------------------------------------------------------------
# 2) Header include
# ------------------------------------------------------------
target_include_directories(cr_bt_pick_place_nodes
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ------------------------------------------------------------
# 3) Installazione
# ------------------------------------------------------------
install(
  TARGETS cr_bt_pick_place_nodes
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY bt_xml/
  DESTINATION share/${PROJECT_NAME}/bt_xml
  FILES_MATCHING PATTERN "*.xml"      # ← correct pattern
)


# ------------------------------------------------------------
# 4) Esportazione
# ------------------------------------------------------------
ament_export_include_directories(include)
ament_export_libraries(cr_bt_pick_place_nodes)
ament_export_dependencies(behaviortree_cpp cr_motion_core cr_interfaces behaviortree_ros2)

ament_package()
