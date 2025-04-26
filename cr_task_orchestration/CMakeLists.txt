cmake_minimum_required(VERSION 3.8)
project(cr_task_orchestration)

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
find_package(cr_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

# -------------------------------------------------------------------
# Executable
# -------------------------------------------------------------------
add_executable(task_orchestrator
  src/task_orchestrator_main.cpp
  src/task_orchestrator.cpp
)

target_include_directories(task_orchestrator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


ament_target_dependencies(task_orchestrator
  cr_interfaces
  rclcpp
  rclcpp_action
)

# -------------------------------------------------------------------
# Install
# -------------------------------------------------------------------
install(TARGETS 
  task_orchestrator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# -------------------------------------------------------------------
# Final export
# -------------------------------------------------------------------
ament_package()
