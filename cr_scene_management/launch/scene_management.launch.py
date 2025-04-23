import launch
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Crea la configurazione MoveIt (usando MoveItConfigsBuilder, come nel tuo codice)
    moveit_config = (
        MoveItConfigsBuilder("ur5e", package_name="cr_moveit")
        .robot_description(file_path="config/ur5e_with_robotiq85.urdf.xacro")
        .robot_description_semantic(file_path="config/ur5e_with_robotiq85.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl","chomp","pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    ### CORREZIONE CHIAVI - MoveItConfigsBuilder.to_moveit_configs(), sembra avre dei problemi, genera delle chiavi sbagliate

    # A) URDF -> "robot_description"
    old_robot_description = moveit_config.robot_description
    if "moveit_config" in old_robot_description:
        corrected_robot_description = {
            "robot_description": old_robot_description["moveit_config"]
        }
    else:
        corrected_robot_description = old_robot_description

    # B) SRDF -> "robot_description_semantic"
    old_semantic = moveit_config.robot_description_semantic
    if "moveit_config_semantic" in old_semantic:
        corrected_robot_description_semantic = {
            "robot_description_semantic": old_semantic["moveit_config_semantic"]
        }
    else:
        corrected_robot_description_semantic = old_semantic

    # C) Kinematics -> "robot_description_kinematics"
    old_kinematics = moveit_config.robot_description_kinematics
    # Se la chiave è "moveit_config_kinematics", la rinominiamo
    if "moveit_config_kinematics" in old_kinematics:
        corrected_kinematics = {
            "robot_description_kinematics": old_kinematics["moveit_config_kinematics"]
        }
    elif "moveit_config" in old_kinematics:
        # Se invece troviamo "moveit_config", la rinominiamo
        corrected_kinematics = {
            "robot_description_kinematics": old_kinematics["moveit_config"]
        }
    else:
        corrected_kinematics = old_kinematics

    # D) Joint Limits -> "robot_description_planning"
    old_joint_limits = moveit_config.joint_limits
    # Se la chiave è "moveit_config_planning", la rinominiamo
    if "moveit_config_planning" in old_joint_limits:
        corrected_joint_limits = {
            "robot_description_planning": old_joint_limits["moveit_config_planning"]
        }
    elif "moveit_config" in old_joint_limits:
        # Se invece troviamo "moveit_config", la rinominiamo
        corrected_joint_limits = {
            "robot_description_planning": old_joint_limits["moveit_config"]
        }
    else:
        corrected_joint_limits = old_joint_limits

    # E) Pianificazione
    old_planning_pipelines = moveit_config.planning_pipelines
    if "moveit_config" in old_planning_pipelines:
        corrected_planning_pipelines = old_planning_pipelines["moveit_config"]
    else:
        corrected_planning_pipelines = old_planning_pipelines

    # F) Trajectory Execution
    corrected_trajectory_execution = moveit_config.trajectory_execution

    #######################################################

    container = ComposableNodeContainer(
        name='scene_management_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # usa il container multithreaded
        composable_node_descriptions=[
            ComposableNode(
                package='cr_scene_management',
                plugin='cr::scene_management::StaticScenePublisher',
                name='static_scene_publisher',
                    parameters=[
                        corrected_robot_description,
                        corrected_robot_description_semantic,
                        corrected_kinematics,
                        corrected_joint_limits,
                        corrected_planning_pipelines,
                        corrected_trajectory_execution
                    ]
            ),
            ComposableNode(
                package='cr_scene_management',
                plugin='cr::scene_management::PlanningSceneModifier',
                name='planning_scene_modifier',
                    parameters=[
                        corrected_robot_description,
                        corrected_robot_description_semantic,
                        corrected_kinematics,
                        corrected_joint_limits,
                        corrected_planning_pipelines,
                        corrected_trajectory_execution
                    ]
            )
        ],
        output='screen'
    )

    return LaunchDescription([container])
