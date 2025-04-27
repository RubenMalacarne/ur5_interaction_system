from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    is_sim = LaunchConfiguration("is_sim")

    config_path = PathJoinSubstitution([
        FindPackageShare("cr_action_servers"),
        "config",
        "pick_params.yaml"
    ])

    config_path_place = PathJoinSubstitution([
        FindPackageShare("cr_action_servers"),
        "config",
        "place_params.yaml"
    ])

    moveit_config = (
        MoveItConfigsBuilder(robot_name="arm_manipulator", package_name="cr_moveit")
        .robot_description()
        .robot_description_semantic()
        .trajectory_execution()
        .planning_pipelines()
        .to_moveit_configs()
    )
    
    pick_action_server = Node(
        package='cr_action_servers',
        executable='pick_action_server',
        name='pick_action_server',
        parameters=[
            moveit_config.to_dict(),
            config_path,
            {"use_sim_time": is_sim}
        ],
        output='screen'
    )

    place_action_server = Node(
        package='cr_action_servers',
        executable='place_action_server',
        name='place_action_server',
        parameters=[
            moveit_config.to_dict(),
            config_path_place,
            {"use_sim_time": is_sim}
        ],
        output='screen'
    )

    return LaunchDescription([
        is_sim_arg,
        pick_action_server,
        place_action_server
    ])
