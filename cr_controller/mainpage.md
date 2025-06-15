<!-- mainpage.md -->

# cr_controller

`cr_controller` is a ROS 2 package that contains all abouts controller and controller manager for the robot of this project.
Use a ur5e_control.yaml file to set the controllers for gripper and arm (we use a unique file to avoid path error).

There is a launch file to start the controller manager and load the controllers, it is called `controller_demo.launch.py`, key points of this file are:

    - Loads the URDF model of the robot and publishes it using the robot_state_publisher.
    - Start a node for controller
    - Spawns the necessary controllers, including:
        - joint_state_broadcaster for publishing joint states
        - arm_trajectory_controller to control the arm’s movement
        - gripper_controller to operate the end-effector



## Author
Sabrina Vinco and Ruben Malacarne