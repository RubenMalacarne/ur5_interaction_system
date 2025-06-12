cmake_minimum_required(VERSION 3.8)
project(cr_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    cr_bringup/emergency_stop_node.py
    cr_bringup/socket_listener.py 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
