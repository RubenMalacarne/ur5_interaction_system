# Scene Management Module

## Overview
The `scene_management` module handles interaction with the MoveIt planning scene.  
It allows dynamic updates such as adding detected objects, attaching/detaching them to the robot, and setting collision permissions.

## Components
- **PlanningSceneModifier**  
  Publishes objects, attaches them to the robot, and manages collision rules via services.
- **StaticScenePublisher**  
  Publishes static elements (e.g., tables) once or periodically.
- **SceneManager**  
  Class that provides access to a shared `PlanningSceneMonitor`.

## Interfaces
- **Topic (published)**: `/planning_scene`  
- **Topic (subscribed)**: `/cr/scene_objects`  
- **Services**:  
  - `/allow_collision`  
  - `/attach_object`

## Dependencies
- ROS 2  
- MoveIt 2  
- `cr_interfaces`  
- `geometric_shapes`  
- `cr_hw_configuration` (for meshes)

## Author
Sabrina Vinco and Ruben Malacarne
