from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cr_bt_orchestrator',
            executable='bt_runner_node',
            name='bt_orchestrator',
            output='screen',
            emulate_tty=True
        ),
    ])
