<!-- mainpage.md -->

# cr_bringup

`cr_bringup` is a ROS 2 package that manages the full system bring-up of a complex robot, with integrated support for an emergency stop mechanism via socket and ROS topics.

The communciation use a communication with Hardware part ("Turtle_pet")

It includes a modular launch system to bring up subsystems in sequence, and a dedicated emergency stop node to safely control their execution.

---

## Launch the complete robotic system:

```bash
ros2 launch cr_bringup system_bringup.launch.py
```

Is recommended build a turtle_pet and run the following command, to have a safety environment.

```bash
ros2 launch cr_bringup emergency_stop_node.launch.py
```

## port use for each connection: 
`6000` --> alexa assistant
`6001` --> documentation index
`6002` --> turtle pet socket connection

## Author
Sabrina Vinco and Ruben Malacarne