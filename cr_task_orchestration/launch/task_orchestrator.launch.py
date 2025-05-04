from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cr_task_orchestration',
            executable='task_orchestrator',
            output='screen',
            parameters=[{'auto_start': True}]
        )
    ])
