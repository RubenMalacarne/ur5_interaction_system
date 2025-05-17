from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    this_dir = os.path.dirname(os.path.realpath(__file__))
    script_path = os.path.join(this_dir, '..', 'cr_remote_assistant', 'alexa_skill_interface.py')
    script_path = os.path.realpath(script_path)

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'python3',
                script_path
            ],
            output='screen'
        )
    ])


# ricorda di ativare ngrok con:    ngrok http 6000