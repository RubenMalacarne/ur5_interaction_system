from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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