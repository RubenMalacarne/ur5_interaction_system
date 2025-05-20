cmake_minimum_required(VERSION 3.8)
project(cr_bt_pick_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(cr_motion_core REQUIRED)

# ------------------------------------------------------------
# 1) Creazione libreria di plugin
# ------------------------------------------------------------
add_library(cr_bt_pick_place_nodes SHARED
  src/set_gripper_node.cpp
)

ament_target_dependencies(cr_bt_pick_place_nodes
  behaviortree_cpp
  cr_motion_core
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
# 3) Install: libreria e header
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

# ------------------------------------------------------------
# 4) Esportazione e package
# ------------------------------------------------------------
ament_export_include_directories(include)
ament_export_libraries(cr_bt_pick_place_nodes)
ament_export_dependencies(behaviortree_cpp cr_motion_core)
ament_package()


## E' importante che questa libreria sia compilata come libreria condivisa