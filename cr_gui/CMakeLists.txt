cmake_minimum_required(VERSION 3.8)
project(cr_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

add_executable(gui_viewer
  src/main.cpp
  src/gui_node.cpp
  src/image_widget.cpp
  # Aggiungi qui i tuoi file header che contengono Q_OBJECT
  include/cr_gui/gui_node.hpp
  include/cr_gui/image_widget.hpp
)

ament_target_dependencies(gui_viewer
  rclcpp sensor_msgs cv_bridge
)

target_include_directories(gui_viewer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(gui_viewer
  Qt5::Widgets
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS gui_viewer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
