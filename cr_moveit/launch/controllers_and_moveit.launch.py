import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    is_sim = LaunchConfiguration("is_sim")

    # Argomento per il livello di log
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="INFO",
        description="Livello di log (DEBUG, INFO, WARN, ERROR)"
    )

    log_level = LaunchConfiguration("log_level")

    # Includi il launch file della simulazione (coppelia_controller)
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('cr_controller'),
                'launch',
                'controller_demo.launch.py'
            )
        )
    )

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

    # move_group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            {"use_sim_time": is_sim},
            corrected_robot_description,
            corrected_robot_description_semantic,
            corrected_kinematics,
            corrected_joint_limits,
            corrected_planning_pipelines,
            corrected_trajectory_execution,
        ],
    )

    # RViz + plugin MoveIt
    rviz_config_path = os.path.join(
        get_package_share_directory('cr_moveit'),
        "config",
        "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            {"use_sim_time": is_sim},
            corrected_robot_description,
            corrected_robot_description_semantic,
            corrected_planning_pipelines,
            corrected_kinematics,
        ]
    )
    
    scene_node = Node(
        package="cr_hw_configuration",
        executable="scene_publisher",
        name="scene_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": is_sim},
            corrected_robot_description,
            corrected_robot_description_semantic,
        ]
        
    )
    
    # move_group_interface_node = Node(
    #     package="coppelia_pick_and_place",
    #     executable="test_mini",
    #     name="move_group_interface_tutorial",
    #     output="screen",
    #     parameters=[
    #         {"use_sim_time": True},
    #         corrected_robot_description,
    #         corrected_robot_description_semantic,
    #         corrected_planning_pipelines,
    #         corrected_kinematics,
    #         corrected_joint_limits,
    #         corrected_trajectory_execution,
    #     ],
    # )

    return LaunchDescription([
        is_sim_arg,
        log_level_arg,
        # Avvia PRIMA la simulazione
        simulation_launch,
        # Poi avvia move_group e RViz
        move_group_node,
        #scene_node,
        rviz_node,
        # move_group_interface_node
    ])
