from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def include(pkg, launch_file, delay=0.0, **kwargs):
    """Utility: include <pkg>/launch/<launch_file> con eventuale ritardo."""
    action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(pkg), 'launch', launch_file])
        ),
        launch_arguments=kwargs.items()
    )
    return GroupAction([TimerAction(period=delay, actions=[action])]) if delay else action


def generate_launch_description():
    return LaunchDescription([
        # 1) MoveIt e controller ROS 2 Control
        include('cr_moveit', 'controllers_and_moveit.launch.py'),

        # 2) MoveIt Scene management
        include('cr_scene_management', 'scene_management.launch.py', delay=2.0),

        # 3) Vision
        include('cr_vision', 'cr_vision.launch.py', delay=4.0),

        # 4) Pick / Place / Scan action-servers
        include('cr_action_servers', 'action_servers.launch.py', delay=6.0),

        # 5) Task-orchestrator
        include('cr_task_orchestration', 'task_orchestrator.launch.py', delay=8.0),
    ])
