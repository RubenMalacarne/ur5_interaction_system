import launch
import launch_ros
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Generate a launch description for the cr_vision perception pipeline.

    This launch file starts the following nodes:
    1. mirror_scena.py: Mirrors the camera scene from simulation or input
    2. obj_detection.py: Performs object detection
    3. obj_selection.py: Selects relevant objects based on criteria
    4. object_state_manager: Maintains object state and serves query requests
    5. human_proximity_monitor: Detects if a human is near the robot end-effector
    """
    # Path to the configuration file for the object selection node
    obj_selection_config = PathJoinSubstitution([
        FindPackageShare("cr_vision"),
        "config",
        "obj_selection.yaml"
    ])
    # Node for mirroring the scene (camera input or simulation view)
    cr_mirror_node = launch_ros.actions.Node(
        package='cr_vision',
        executable='mirror_scena.py', 
        name='mirror_camera_node',
        output='screen',
        # parameters=[{'param_name': 'param_value'}]
    )
    # Node for detecting objects in the scene
    cr_obj_detection_node = launch_ros.actions.Node(
        package='cr_vision',
        executable='obj_detection.py',  
        name='object_detection_node',
        output='screen',
        # parameters=[{'param_name': 'param_value'}]
    )
    # Node for selecting objects using specific rules defined in a YAML config
    cr_obj_selection_node = launch_ros.actions.Node(
        package='cr_vision',
        executable='obj_selection.py',  
        name='obj_selection_node',
        output='screen',
        parameters=[obj_selection_config]
    )
    # Node for managing the state of the detected objects
    cr_obj_state_manager_node = launch_ros.actions.Node(
        package='cr_vision',
        executable='object_state_manager',  
        name='object_state_manager',
        output='screen'
    )
    # Node to detect proximity of humans to the robot
    cr_human_proximity_monitor_node = launch_ros.actions.Node(
        package='cr_vision',
        executable='human_proximity_monitor',  
        name='human_proximity_monitor',
        output='screen'
    )
    # List of nodes to launch
    nodes = [
        cr_mirror_node,
        cr_obj_detection_node,
        cr_obj_selection_node,
        cr_obj_state_manager_node,
        cr_human_proximity_monitor_node
    ]

    return launch.LaunchDescription(nodes)