cmake_minimum_required(VERSION 3.8)
project(cr_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CoppeliaTask.action"
  "action/Pick.action"
  "action/Place.action"

  "msgs/DetectedSurfaces.msg"
  "msgs/DetectedObjects.msg"
  "msgs/InferenceResult.msg"
  "msgs/Yolov8Inference.msg"
  "msgs/PoseKeypoint.msg"
  "msgs/PoseResult.msg"
  "msgs/SegmentationResult.msg"
  "msgs/Yolov8Segmentation.msg"
  "msgs/ObjectDetectionBox.msg"
  "msgs/ObjectDetectionResult.msg"
  "msgs/ObjectCount.msg"
  "msgs/ObjectInfo.msg"

  "srv/AllowCollision.srv"
  "srv/AttachObject.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs

)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
