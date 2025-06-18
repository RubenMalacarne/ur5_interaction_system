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

> Note: don’t forget to source your ROS environment after installation: 
  ```bash
  source /opt/ros/humble/setup.bash
  ```

### Dependencies
CoppeliaSim
Download and install CoppeliaSim Edu for Ubuntu:
CoppeliaSim Download

Set the environment variable:

bash
Copia
Modifica
export COPPELIASIM_ROOT_DIR=~/CoppeliaSim_Edu_V4_9_0_rev2_Ubuntu22_04/
Follow the official ROS 2 integration guide

⚠️ Make sure to include the sim_ros2_interface plugin correctly in your workspace.

Additional ROS Packages
ros2_control (required):
ros2_control Setup

MoveIt 2 (if used):
MoveIt Humble Guide

⚠️ Always source the MoveIt workspace as needed.

ZMQ support:

bash
Copia
Modifica
sudo apt update
sudo apt install libzmq3-dev
Other required tools:

bash
Copia
Modifica
pip3 install xmlschema
sudo apt install xsltproc
3. Project Setup
Clone the Repository
Use SSH to clone the repository into your ROS 2 workspace:

bash
Copia
Modifica
git clone git@github.com:your_org/your_repo.git
Run Installation Script
Navigate into the cloned repository and run:

bash
Copia
Modifica
./install.sh
Register Custom Messages
Ensure that any custom message packages are properly listed in your meta file or package.xml.

Build the Workspace
Navigate back to the root of your workspace and build:

bash
Copia
Modifica
colcon build
source install/setup.bash


## Installation

You can follow the [tutorial video](https://drive.google.com/file/d/1OCiY79zKw5pEY-AEaQqU6TcGRgGThvL9/view?usp=drive_link) to see exactly what needs to be done. Alternatively, follow these steps:

0. If you use Alexa install Ngrok on your local machine, following step by step the instructions on the web site: [link_website](https://ngrok.com/)

1. Clone our repository inside your workspace (also before create src file):
  ```bash
  git clone https://github.com/SwDev4Cobots/2024-25-Final-Project-Group-1.git src
  ```
2. Run the ./install.sh script in a terminal to install all necessary project dependencies. If you haven’t already downloaded CoppeliaSim, add the -d flag.

When prompted _Do you want to copy the YOLO model for object detection?_, respond with:
- `yes` → to copy the pre-tested model
- `no` → to use your own model or select one of our pre-configured options

3. Add the following lines information inside _sim_ros2_interface/meta/interfaces.txt_:
```bash
sensor_msgs/msg/JointState
rosgraph_msgs/msg/Clock
std_msgs/msg/MultiArrayDimension
std_msgs/msg/Float64MultiArray
std_msgs/msg/MultiArrayLayout
tf2_msgs/msg/TFMessage
geometry_msgs/msg/PoseArray
```

We highly recommend using Docker for the following steps to ensure a fully prepared environment. If you prefer not to use Docker, make sure all dependencies listed in the Dockerfile are installed on your system, and then proceed directly to step 6.
>Note: docker use 16 GB
4. Build the Docker container:
```bash
docker compose -f 'src/docker-compose.yml' up -d --build 'ros2'
```
5. Access the running Docker container:
```bash
docker exec -it cr_project_container bash
```
>Note: If the system is restarted, you can relaunch Docker with:
```bash
docker start -ai cr_project_container
```
>Note: If you are using Docker, all terminal commands from this point onward must be executed inside the container.

6. In a terminal, build the ROS 2 workspace:
```bash
colcon build
```

7. Source the workspace setup script:
```bash
source install/setup.bash 
```

## Running the System

**_NOTE:_** If you are using Docker, all terminal commands must be executed inside the container. 

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

You can watch the [Demo Video](https://drive.google.com/file/d/1UDiQOxL4sO0ssUpsXL1aLrQIR0Jx1Poy/view?usp=drive_link)

The system supports the following interactions, which can be triggered either via voice commands with Alexa or through the ROS2 command-line interface.

 **_NOTE:_** If you use Alexa before using any commands, you must first activate the custom skill by saying _Alexa, attiva simulazione_

**_NOTE:_** If you are using Docker, all terminal commands must be executed inside the container. 

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
