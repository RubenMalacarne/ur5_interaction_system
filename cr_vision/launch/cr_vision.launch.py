import launch
import launch_ros

def generate_launch_description():
    cr_mirror_node = launch_ros.actions.Node(
        package='cr_vision',
        executable='mirror_scena.py', 
        name='mirror_camera_node',
        output='screen',
        # parameters=[{'param_name': 'param_value'}]
    )
    cr_obj_detection_node = launch_ros.actions.Node(
        package='cr_vision',
        executable='obj_detection.py',  
        name='object_detection_node',
        output='screen',
        # parameters=[{'param_name': 'param_value'}]
    )
    cr_obj_selection_node = launch_ros.actions.Node(
        package='cr_vision',
        executable='obj_selection.py',  
        name='obj_selection_node',
        output='screen',
        # parameters=[{'param_name': 'param_value'}]
    )

    nodes = [
        cr_mirror_node,
        cr_obj_detection_node,
        cr_obj_selection_node
    ]

    return launch.LaunchDescription(nodes)