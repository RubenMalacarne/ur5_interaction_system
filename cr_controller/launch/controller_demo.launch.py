import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

##
# @file robot_setup.launch.py
# @brief Launch file for setting up the UR5e robot with ROS 2 Control.
#
# This launch file loads the URDF, starts the robot_state_publisher, and launches the ROS 2 controller manager 
# along with the necessary controllers (joint state broadcaster, trajectory controller, and gripper controller).
#
# @package cr_hw_configuration
# @dependencies
# - robot_state_publisher
# - controller_manager
# - cr_controller (for controller configuration)
#
# @param is_sim (bool) Whether to use simulation time (default: True)
#
# @author Sabrina Vinco and Ruben Malacarne
##

def generate_launch_description():
    pkg_share = get_package_share_directory('cr_hw_configuration')
    urdf_file = os.path.join(pkg_share, 'urdf', 'ur5e.urdf')

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    is_sim = LaunchConfiguration("is_sim")

    # Legge il file URDF
    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("cr_controller"),
            "config",
            "ur5e_control.yaml",
        ]
    )


    # Nodo robot_state_publisher per pubblicare il robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": is_sim}
        ]
    )

    # Avvia il nodo controller manager (ros2_control_node) e passa il parametro robot_description
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_controllers,
            {"use_sim_time": is_sim}
        ],
        output="screen",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # Spawn del joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]
    )

    # Spawn del joint_trajectory_position_controller
    arm_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["arm_trajectory_controller", "--controller-manager", "/controller_manager"]
    )
    
    # Spawn del controller del gripper
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"]
    )


    return LaunchDescription([
        is_sim_arg,
        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        arm_trajectory_controller_spawner,
        gripper_controller_spawner,
    ])
