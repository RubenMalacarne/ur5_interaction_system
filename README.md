# Course Project: Software Development for Collaborative Robotics - AY 2024/2025

## Project Overview
Designed for dynamic environments involving close human-robot interaction, this project integrates natural voice commands, visual perception, and intelligent motion planning to enable intuitive and safe collaboration.

Users can select one of several tall, square-based colored blocks, specifically designed for robotic manipulation, by simply interacting with Amazon Alexa. Upon receiving the voice command, the robot detects the specified object and performs a controlled horizontal displacement.

To ensure safety, the system continuously monitors a designated area around the robot. If a person enters this zone, the robot halts its execution immediately and resumes only when the area is clear.

The result is a cohesive robotic workflow where voice interaction, perception, and safety are tightly integrated to support collaborative tasks.

## Technologies Used
- **Robot**: UR5e manipulator
- **Gripper**: Robotiq 2F-85
- **Programming Languages**: C++ (core ROS 2 nodes), Python (image stream processing, voice interaction)
- **Robotic Framework**: ROS 2
- **Simulation Environment**: CoppeliaSim
- **Motion Planning**: MoveIt2
- **Task Planning**: Behavior Trees (https://www.behaviortree.dev and https://github.com/BehaviorTree/BehaviorTree.ROS2)
- **Object Detection**: Ultralytics YOLO
- **Voice Interface**: Amazon Alexa


## Simulation Scene
The simulation scene includes the following key components:

- A robotic workbench equipped with a UR5 manipulator and a Robotiq 2F-85 gripper  
- Three colored blocks (green, red and blue), each available for pick-and-place operations  
- A randomly moving human to simulate unpredictable behavior in shared spaces  
- A second human following a predefined path, repeatedly approaching and leaving the workspace to demonstrate the system's responsiveness to human presence  
- An ideal RGB-D camera for object detection  

![alt text](Images/Simulation%20Scene.png)


## Control and Interaction Components

### Voice Command Integration
The system integrates Amazon Alexa as a voice interface for initiating and controlling pick-and-place operations.

To issue a command, the user must first activate the assistant by saying:  
**“Alexa, attiva esecuzione”**  
followed by one of the supported intents.

Alexa can be used to:
- Select a colored block to manipulate by specifying its color (e.g., “pick the red block”)
- Pause the execution, temporarily halting the workflow until resumed
- Cancel the current task, stopping the ongoing operation, returning the robot to its home position and resetting the system to an idle state.

This natural language interface simplifies coordination in the collaborative environment.

### GUI
The GUI is designed to assist the operator in monitoring system execution and status, offering the following features:

![alt text](Images/GUI%20Features.png)


### Turtle Pet
This device is designed to allow quick system initialization and emergency shutdown.

![alt text](Images/Turtle%20Pet.png)



### RGB-D Camera and YOLO
Leveraging an RGB-D camera, the system performs both object detection and 3D localization. The RGB stream is processed by the YOLO algorithm to identify and classify objects in the scene, while the depth data is used to estimate the 3D position of the object's top surface center,critical information for planning an accurate and stable grasp.

![alt text](Images/Block%20Center.png)


## Installation

_Tutorial Video: https://drive.google.com/file/d/1UDiQOxL4sO0ssUpsXL1aLrQIR0Jx1Poy/view?usp=sharing_

1. Install Ngrok seguendo step by step le istruzioni sul sito ufficiale: https://ngrok.com/

2. Clone the repository:
git clone https://github.com/SwDev4Cobots/2024-25-Final-Project-Group-1.git

3. Run the file install.sh on the termial to install all the necessary dependencies for the project and if you don't already dowloaded CoppeliaSim, add "-d".

When you will asked "Do you want to copy the YOLO model for object detection" presse:
- yes -> copy the model already tested
- no -> to re-train your of pre-set model

4. Build docker-compose:
docker compose -f 'src/docker-compose.yml' up -d --build 'ros2'

5. Run Docker with the following commands:
docker exec -it cr_project_container.bash

6. Add information inside 'sim_ros2_interface'

7. Execute colcon build of ros2

8. source install/setup.bash

## TODO: mettere questo: 

sensor_msgs/msg/JointState
rosgraph_msgs/msg/Clock
std_msgs/msg/MultiArrayDimension
std_msgs/msg/Float64MultiArray
std_msgs/msg/MultiArrayLayout
tf2_msgs/msg/TFMessage
geometry_msgs/msg/PoseArray


## Running the System

After completing the steps in the **Installation** section, open:

- In a terminal, navigate to the directory where you installed CoppeliaSim and run `./coppeliaSim.sh` to launch CoppeliaSim and open the provided scene.

#### If you do **not** have the Turtle Pet joystick:
- In another terminal: 
```bash
ros2 launch cr_bringup system_bringup.launch.py
```

This will start the full system.

#### If you do have the Turtle Pet joystick:
- In another terminal: 
```bash
ros2 launch cr_bringup emergency_stop_controller.launch.py
```
Then, move the joystick to activate the system.

In both cases, RViz should launch automatically with the MoveIt scene and the corresponding GUI.


## How to use

_Demo Video: https://drive.google.com/file/d/1OCiY79zKw5pEY-AEaQqU6TcGRgGThvL9/view?usp=sharing_

Important Note for Alexa Users:
Before issuing any command, you must first activate the custom skill by saying:
**Alexa, attiva simulazione**

The system supports the following interactions, which can be triggered either via voice commands with Alexa or through the ROS2 command-line interface.

### Execute Workflow
This command initiates the primary workflow, instructing the robot to pick and place a specified object.

- With Alexa:
  Use the intent to request picking up a cube, specifying its color. For example:
  "Alexa, ask simulation to get the green cube."

- Via ROS2 CLI:
  Send a goal to the /cr/execute_workflow action server. Specify the desired object_label which must match one of the labels displayed in the GUI.
  ```bash
  ros2 action send_goal /cr/execute_workflow cr_interfaces/action/ExecuteWorkflow "{object_label: 'green_cube'}"
  ```
  _(Available labels: green_cube, red_cube, blue_cube)_

### Pause Workflow
This command temporarily halts the execution of the current workflow. The robot will stop its motion and wait for a resume command.
- With Alexa: "Alexa, ask simulation to pause."
- Via ROS2 CLI: Publish a true message to the /cr/pause_command topic.
  ```bash
  ros2 topic pub /cr/pause_command std_msgs/msg/Bool "{data: true}"
  ```

#### Resume Workflow 
This command resumes a workflow that has been previously paused.
- With Alexa: "Alexa, ask simulation to resume."
- Via ROS2 CLI: Publish a false message to the /cr/pause_command topic.
  ```bash
  ros2 topic pub /cr/pause_command std_msgs/msg/Bool "{data: false}"
  ```

### Cancel Worflow
This command completely stops and cancels the current workflow. This action is final and cannot be undone for the current task.
- With Alexa: "Alexa, ask simulation to stop."
- Via ROS2 CLI: Publish a true message to the /cr/stop_command topic.
  ```bash
  ros2 topic pub /cr/stop_command std_msgs/msg/Bool "{data: true}"
  ```

### Emergency Stop Worflow
TODO


## Project Structure
```bash
.
└── workspace/
    ├── cr_bringup            --> Launch files for system startup and emergency stop handling  
    ├── cr_bt_common          --> Reusable Behavior Tree nodes and utilities  
    ├── cr_bt_orchestrator    --> Behavior Trees and nodes to manage the full system workflow
    ├── cr_bt_pick_place      --> Behavior Trees and nodes for pick and place operations
    ├── cr_controller         --> Controller configurations and parameters  
    ├── cr_gui                --> GUI for workflow monitoring 
    ├── cr_hw_configuration   --> URDF, hardware interfaces, and simulation assets
    ├── cr_interfaces         --> Custom message, service, and action definitions
    ├── cr_motion_core        --> Abstraction layer for asynchronous MoveIt motion commands
    ├── cr_moveit             --> MoveIt configuration package  
    ├── cr_remote_assistant   --> Integration with Alexa for voice-based assistance 
    ├── cr_scene_management   --> Scene management utilities to keep MoveIt Planning Scene updated 
    └── cr_vision             --> Camera data processing for scene understanding 
```

## Documentation

To generate the complete project documentation, make sure all required dependencies (`doxygen`, `python3`) are installed, then follow these steps:

1. **Run the documentation generation script**:
   ```bash
   ./generate_doc.sh
   ```
This will create a `docs/` folder inside each package, containing the Doxygen-generated documentation files.

2. **Open the full documentation in the browser**:
   ```bash
   python3 html_conversion.py
   ```
When prompted, select option `2`.
This will start a local server accessible at `http://localhost:6001/index_documentation.html`.

## Contributors
This project was developed by:
- Ruben Malacarne - ruben.malacarne@studenti.unitn.it
- Sabrina Vinco - sabrina.vinco@studenti.unitn.it