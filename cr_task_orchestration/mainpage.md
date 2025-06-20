# Task Orchestration Module

## Overview
The `task_orchestration` module manages high-level robotic workflows by coordinating multiple task-level actions.  
It enables execution of complex sequences such as object picking and placing, integrating feedback and service calls  
to ensure reliable and flexible task execution.

## Components
- **TaskOrchestrator**  
  Core class that implements the workflow logic. Acts as an action server and coordinates sub-tasks like pick and place  
  by sending goals to other action servers and interacting with services.

## Interfaces

- **Action Server**  
  - `/cr/execute_workflow` — Receives high-level workflow execution requests.

- **Action Clients**  
  - `/cr/pick_action` — Sends a pick goal to grasp the target object.  
  - `/cr/place_action` — Sends a place goal to move and release the object.

- **Service Clients**  
  - `/cr/get_object_info` — Requests metadata and pose for the specified object.

- **Topic (published)**  
  - `/cr/freeze_scene` — Signals to freeze/unfreeze the planning scene during critical workflow stages.

## Behavior
When a workflow goal is accepted:
1. The orchestrator requests object information from the service.
2. It sends a pick action goal to retrieve the object.
3. If the pick succeeds, it sends a place action goal to move the object to a target location.
4. It publishes scene freeze control messages at the beginning and end of the workflow.
5. Upon completion, the workflow result is reported back to the client.

## Dependencies
- ROS 2  
- `cr_interfaces`  
- Action and service servers for:
  - Pick  
  - Place  
  - Object Info

## Authors
Ruben Malacarne and Sabrina Vinco
