import launch
import launch_ros
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    obj_detector_config = PathJoinSubstitution([
        FindPackageShare("cr_vision"),
        "config",
        "obj_detector_config.yaml"
    ])

    cr_mirror_node = launch_ros.actions.Node(
        package='cr_vision',
        executable='mirror_scena.py', 
        name='mirror_camera_node',
        output='screen',
    )

    cr_obj_detector_node = launch_ros.actions.Node(
        package='cr_vision',
        executable='obj_detector.py',  
        name='object_detector_node',
        output='screen',
        parameters=[obj_detector_config]
    )

    cr_obj_state_manager_node = launch_ros.actions.Node(
        package='cr_vision',
        executable='object_state_manager',  
        name='object_state_manager',
        output='screen'
    )
    cr_human_proximity_monitor_node = launch_ros.actions.Node(
        package='cr_vision',
        executable='human_proximity_monitor',  
        name='human_proximity_monitor',
        output='screen'
    )

    nodes = [
        cr_mirror_node,
        cr_obj_detector_node,
        cr_obj_state_manager_node,
        cr_human_proximity_monitor_node
    ]

    return launch.LaunchDescription(nodes)