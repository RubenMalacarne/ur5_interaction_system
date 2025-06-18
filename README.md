# Course Project: Software Development for Collaborative Robotics - AY 2024/2025

## Project Overview
Designed for dynamic environments involving human-robot interaction, this project combines natural voice commands and visual perception within a robotic system to ensure easy and safe collaboration.

Users can select one of several colored blocks, specifically designed for robotic manipulation, by simply interacting with Amazon Alexa. Upon receiving the voice command, the robot detects the specified object and performs a controlled horizontal displacement.

To ensure safety, the system continuously monitors a designated area around the robot. If a person enters this zone, the robot halts its execution immediately and resumes only when the area is clear.

This [Demo Video](https://drive.google.com/file/d/1UDiQOxL4sO0ssUpsXL1aLrQIR0Jx1Poy/view?usp=drive_link) shows a complete example of the project.

## Technologies Used
- **Robot**: [UR5e manipulator](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
- **Gripper**: [Robotiq 2F-85](https://github.com/PickNikRobotics/ros2_robotiq_gripper)
- **Programming Languages**: C++ (core ROS 2 nodes), Python (image stream processing, voice interaction)
- **Robotic Framework**: [ROS 2](https://docs.ros.org/en/humble/index.html)
- **Simulation Environment**: [CoppeliaSim](https://www.coppeliarobotics.com/)
- **Control Algorithms**: [Standard ROS2 Controllers](https://github.com/ros-controls/ros2_control)
- **Motion Planning**: [MoveIt2](https://moveit.picknik.ai/main/index.html#)
- **Task Planning**: [Behavior Trees](https://www.behaviortree.dev) and [Behavior Trees ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2)
- **Object Detection**: [Ultralytics YOLO](https://www.ultralytics.com/it)
- **Voice Interface**: [Amazon Alexa](https://developer.amazon.com/it-IT/alexa/alexa-skills-kit)


## Simulation Scene
The simulation scene includes the following key components:

- A robotic workbench equipped with a UR5e manipulator and a Robotiq 2F-85 gripper  
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
- Select a colored block to manipulate by specifying its color
- Pause the execution, temporarily halting the workflow until resumed
- Cancel the current task, stopping the ongoing operation, returning the robot to its home position and resetting the system to an idle state.

This natural language interface simplifies coordination in our collaborative environment.

### GUI
The GUI is designed to assist the operator in monitoring system execution and status, offering the following features:

![alt text](Images/GUI%20Features.png)


### Turtle Pet
This device is designed to allow quick system initialization and emergency shutdown.

![alt text](Images/Turtle%20Pet.png)



### RGB-D Camera and YOLO
Leveraging an RGB-D camera, the system performs both object detection and 3D localization. The RGB stream is processed by the YOLO algorithm to identify and classify objects in the scene, while the depth data is used to estimate the 3D position of the object's top surface center,critical information for planning an accurate and stable grasp.

![alt text](Images/Block%20Center.png)


## Local Installation Guide
This guide explains how to set up the project on your local machine:

### System Requirements
- Ubuntu 22.04 or WSL2 with Ubuntu 22.04
- ROS 2 Humble: [Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### Dependencies
#### CoppeliaSim
Download and install CoppeliaSim for Ubuntu: 
- [CoppeliaSim Download](https://www.coppeliarobotics.com/) 
- Follow the official [ROS 2 integration guide](https://manual.coppeliarobotics.com/en/ros2Tutorial.htm)

> Note: Make sure to copy the package _sim_ros2_interface_ correctly in your ROS2 workspace.

#### Additional ROS Packages
- ros2_control: [ros2_control Setup](https://control.ros.org/humble/doc/getting_started/getting_started.html)
- MoveIt 2: [MoveIt Humble Guide](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html)
> Note: Always source the MoveIt workspace as needed.

#### ZMQ support
```bash
sudo apt update
sudo apt install libzmq3-dev
```

#### Other required tools:
```bash
pip3 install xmlschema notebook ultralytics pyyaml
sudo apt install xsltproc
```

#### Ngrok
If you use Alexa install Ngrok, following step by step the instructions on the [web site](https://ngrok.com/).

### Project Setup
#### Clone the Repository
Clone this repository inside `src` folder of your workspace:
```bash
git clone https://github.com/SwDev4Cobots/2024-25-Final-Project-Group-1.git
```

#### Run Installation Script
Navigate into the cloned repository and run:

```bash
./install.sh
```

When prompted _Do you want to copy the YOLO model for object detection?_, respond with:
- `yes` → to copy the default model
- `no` → to use your own model or select one of our pre-configured options

#### Register Custom Messages
Add the following custom custom messages inside _sim_ros2_interface/meta/interfaces.txt_:
```bash
sensor_msgs/msg/JointState
rosgraph_msgs/msg/Clock
std_msgs/msg/MultiArrayDimension
std_msgs/msg/Float64MultiArray
std_msgs/msg/MultiArrayLayout
tf2_msgs/msg/TFMessage
geometry_msgs/msg/PoseArray
```

#### Build the Workspace
Navigate back to the root of your workspace and build:

```bash
colcon build
source install/setup.bash
```


## Installing with Docker

>Note: docker use 16 GB

To simplify the setup process and ensure all ROS 2 and project dependencies are correctly installed, we recommend using Docker.
You can also follow this step-by-step [tutorial video](https://drive.google.com/file/d/1OCiY79zKw5pEY-AEaQqU6TcGRgGThvL9/view?usp=drive_link).

#### (Optional) Alexa Users Only
If you're using Alexa, install Ngrok on your local machine by following the official guide: [Ngrok Installation](https://ngrok.com/)

#### Clone the Repository
Clone our repository inside your workspace (also before create src file):
  ```bash
  git clone https://github.com/SwDev4Cobots/2024-25-Final-Project-Group-1.git src
  ```
  
#### Run Installation Script
Navigate into the cloned repository and run:

```bash
./install.sh # If you haven’t already downloaded CoppeliaSim, add the -d flag.
```

When prompted _Do you want to copy the YOLO model for object detection?_, respond with:
- `yes` → to copy the default model
- `no` → to use your own model or select one of our pre-configured options


#### Register Custom Messages
Add the following custom custom messages inside _sim_ros2_interface/meta/interfaces.txt_:
```bash
sensor_msgs/msg/JointState
rosgraph_msgs/msg/Clock
std_msgs/msg/MultiArrayDimension
std_msgs/msg/Float64MultiArray
std_msgs/msg/MultiArrayLayout
tf2_msgs/msg/TFMessage
geometry_msgs/msg/PoseArray
```

#### Build the Docker container
From the workspace root:
```bash
docker compose -f 'src/docker-compose.yml' up -d --build 'ros2'
```

#### Access the Docker Container
To enter the container:
```bash
docker exec -it cr_project_container bash
```
If your system restarts, you can relaunch the container with:
```bash
docker start -ai cr_project_container
```
#### Build the ROS 2 Workspace
Inside the container:
```bash
colcon build
```
Still inside the container:
```bash
source install/setup.bash 
```

## Running the System

> Note: When using Docker, all terminal commands should be executed inside the running container.

After completing the steps in the **Installation** section:

- In a terminal, navigate to the directory where you installed CoppeliaSim, run `./coppeliaSim` to launch CoppeliaSim and open the provided scene.
  
>Note: if you are using Docker, you find the CoppeliaSim here: 
```bash
cd /opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04/
#run with this: 
./coppeliaSim
```
open the scene `simulation_scene.ttt` located in the `src/cr_hw_configuration/scene/` directory.

#### If you do **not** have the Turtle Pet, you can start the system without it:
- open new terminal: 
```bash
cd ros2_ws
source install/setup.bash
ros2 launch cr_bringup system_bringup.launch.py
```
This will start the full system.

#### If you do have the Turtle Pet:
- In another terminal: 
```bash
cd ros2_ws
source install/setup.bash
ros2 launch cr_bringup emergency_stop_controller.launch.py
```
Then, move the switch to activate/disactivate(kill) the system.

In both cases, RViz should launch automatically with the MoveIt scene and the corresponding GUI.


## How to use

[Demo Video](https://drive.google.com/file/d/1UDiQOxL4sO0ssUpsXL1aLrQIR0Jx1Poy/view?usp=drive_link)

The system supports the following interactions, which can be triggered either via voice commands with Alexa or through the ROS2 command-line interface.

> Note: If you use Alexa before using any commands, you must first activate the custom skill by saying _Alexa, attiva simulazione_

> Note: When using Docker, all terminal commands should be executed inside the running container.

### Execute Workflow
This command initiates the primary workflow, instructing the robot to pick and place a specified object.

- With Alexa:
  Use the intent to request picking up a cube, specifying its color. For example:
  `Alexa, prendi cubo verde.`

- Via ROS2 CLI:
  Send a goal to the `/cr/execute_workflow` action server. Specify the desired object_label which must match one of the labels displayed in the GUI.
  ```bash
  ros2 action send_goal /cr/execute_workflow cr_interfaces/action/ExecuteWorkflow "{object_label: 'green_cube'}"
  ```
  _(Available labels: green_cube, red_cube, blue_cube)_

### Pause Workflow
This command temporarily halts the execution of the current workflow. The robot will stop its motion and wait for a resume command.
- With Alexa: "Alexa, metti in pausa."
- Via ROS2 CLI: Publish a true message to the `/cr/pause_command topic`.

  ```bash
  ros2 topic pub /cr/pause_command std_msgs/msg/Bool "{data: true}"
  ```

### Resume Workflow 
This command resumes a workflow that has been previously paused.
- With Alexa: "Alexa, riprendi."
- Via ROS2 CLI: Publish a false message to the /cr/pause_command topic.

  ```bash
  ros2 topic pub /cr/pause_command std_msgs/msg/Bool "{data: false}"
  ```

### Cancel Worflow
This command completely stops and cancels the current workflow. This action is final and cannot be undone for the current task.
- With Alexa: "Alexa, ferma l'esecuzione."
- Via ROS2 CLI: Publish a true message to the /cr/stop_command topic.

  ```bash
  ros2 topic pub /cr/stop_command std_msgs/msg/Bool "{data: true}"
  ```

### Emergency Stop Worflow
If you are using the Turtle Pet, you can immediately **stop** the robot by **switch it off** the emergency switching (led green turn **off** and red turn **on**). This action will halt all robot movements and kill the system to a safe state.
If you **switch it on** (the green LED will **turn on**), the system will restart and you can continue your session.


## Project Structure
```
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

To generate the complete project documentation, make sure all required dependencies (`doxygen`, `python3`,`markdown`) are installed, then follow these steps:

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

Enjoy the project! :D
