##
# @file visualiza_robot.launch.py
# @brief ROS 2 launch file for visualizing a robot model with RViz.
#   it's used for only visualizzation purposes.
# @return A LaunchDescription object containing all the nodes for visualization.
#
# @author Ruben Malacarne and Sabrina vinco
# @date 2025-05-28
##
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
    
    ##
    # @brief Generates the launch description for robot visualization.
    #
    # This function sets up the necessary ROS 2 nodes to visualize a robot model using RViz.
    # It includes:
    # - Loading the URDF/XACRO file from the `cr_hw_configuration` package.
    # - Starting `robot_state_publisher` with the robot description.
    # - Launching the `joint_state_publisher_gui` for manual joint control.
    # - Launching RViz with a predefined configuration.
    #
    # @return launch.LaunchDescription object containing the nodes to launch.
    ##
def generate_launch_description():
    urdf_file = PathJoinSubstitution([
        FindPackageShare('cr_hw_configuration'),
        'urdf',
        'ur5e_with_robotiq85.urdf'  # or arctos.xacro if using xacro
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('cr_hw_configuration'),
        'rviz',
        'default.rviz'
    ])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file, ' is_ignition:=false'])
            }]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file]
        )
    ])