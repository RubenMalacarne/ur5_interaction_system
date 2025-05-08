from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cr_bt_orchestrator',
            executable='task_planner',
            name='task_planner',
            output='screen',
            emulate_tty=True
        ),
    ])
