import launch
import launch_ros
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    emergency_stop_node = launch_ros.actions.Node(
        package='cr_bringup',
        executable='emergency_stop_node.py', 
        name='emergency_stop_node',
        output='screen',
    )
    
    nodes = [
        emergency_stop_node
    ]


    return launch.LaunchDescription(nodes)