from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(robot_name="arm_manipulator", package_name="cr_moveit")
        .robot_description()
        .robot_description_semantic()
        .trajectory_execution()
        .planning_pipelines()
        .to_moveit_configs()
    )

    config_file = PathJoinSubstitution([
        FindPackageShare("cr_bt_orchestrator"),
        "config",
        "orchestrator_config.yaml"
    ])

    return LaunchDescription([
        Node(
            package='cr_bt_orchestrator',
            executable='orchestrator',
            name='orchestrator',
            parameters=[
                moveit_config.to_dict(),
                {"use_sim_time": True},
                config_file
            ],
            output='screen',
            emulate_tty=True
        ),
    ])
