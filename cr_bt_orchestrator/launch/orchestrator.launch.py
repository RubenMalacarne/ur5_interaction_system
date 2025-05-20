from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(robot_name="arm_manipulator", package_name="cr_moveit")
        .robot_description()
        .robot_description_semantic()
        .trajectory_execution()
        .planning_pipelines()
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package='cr_bt_orchestrator',
            executable='task_planner',
            name='task_planner',
            parameters=[
                moveit_config.to_dict(),
                {"use_sim_time": True}
            ],
            output='screen',
            emulate_tty=True
        ),
    ])
