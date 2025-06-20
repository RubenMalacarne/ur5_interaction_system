# 🤖 Projects SDCR

Welcome to the **UR5 Pick and Place** project!  
This project involves controlling a **UR5 robotic arm** equipped with a **2F-85 Robotiq gripper** to autonomously pick, place and scanning objects based on visual detection.

## 📋 Project Overview

- **Robot:** UR5 manipulator
- **Gripper:** Robotiq 2F-85
- **Perception:** RGB-D Camera
- **Object Detection:** YOLO 
- **Pose estimation:** YOLO
- **Objective:** Detect objects in the environment, classify them, and pick up only the desired ones.

Using an RGB-D camera, the system identifies objects in 3D space. The YOLO algorithm is used to recognize and classify objects, helping the robot decide which objects to pick and which ones to ignore.

---

## 🗺️ ROS 2 Package Graph

You can view the full ROS 2 package architecture by clicking the link below:  
🔗 [View ROS 2 System Map](https://app.diagrams.net/#G1d35rVPIo2uMd6x9BIGAX8NVfY-hrPb3q#%7B%22pageId%22%3A%22MBEQydd_YOxhgCBZd9wA%22%7D)

---
---

## 🔗 Step to prepare NGROK

Ngrok is used to expose a local server to the internet. This is particularly useful for enabling Alexa or other external services to communicate with your local application.

setp sono : 

- [download Ngrok](https://dashboard.ngrok.com/get-started/setup/linux)
- copy your toke inside your pc: token about Ngrok is [here](https://dashboard.ngrok.com/get-started/your-authtoken)
- put your token in the following command: 
```bash
ngrok config add-authtoken <il_tuo_token>
```
- in the end run the followint code: 
```bash
ngrok http 6000
```
---



## 🚀 Technologies Used

- ROS 2 (Robot Operating System 2)
- MoveIt 2
- YOLOv8 for Object Detection
- Gazebo / Ignition for Simulation
- C++ and Python



---

## 📂 Project Structure

```bash
WS_CR/
├── cr_bringup/
├── cr_vision/
├── cr_hw_controller/
├── cr_interfaces/
├── cr_moveit/
├── ...  
└── cr_task_orchestration/
---
<p align="center">
  <img src="./images_logo/coppeliasim.png" alt="CoppeliaSim" height="100"/>  
  <img src="./images_logo/moveit2.png" alt="Moveit2" height="70"/>  
  <img src="./images_logo/ros2.png" alt="Ros2" height="70"/>  
  <img src="./images_logo/ultralytics.png" alt="Ultralytics" height="70"/>  
</p>

sensor_msgs/msg/JointState
rosgraph_msgs/msg/Clock
std_msgs/msg/MultiArrayDimension
std_msgs/msg/Float64MultiArray
std_msgs/msg/MultiArrayLayout
tf2_msgs/msg/TFMessage
