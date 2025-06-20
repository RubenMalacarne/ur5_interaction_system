from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    """
    Generate the launch description to run the Flask server for Alexa skill.

    This function finds the absolute path to the `alexa_skill_interface.py` 

    Returns:
        LaunchDescription: A launch description object with the Flask process.
    """
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