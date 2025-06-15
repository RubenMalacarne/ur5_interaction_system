<!-- mainpage.md -->

# cr_remote_assistant

`cr_remote_assistant` is a basic ROS 2 package for alexa integration using Flask and Ngrok to connnect with alexa skill.

It is designed to integrate advanced object recognition and human-awareness into a robotic system, enabling safe and context-aware task execution.


## execute program with: ros2 launch cr_remote_assistant launch_alexa_interface.py

    ```bash
    ros2 launch cr_remote_assistant launch_alexa_interface.py
    ```

## Expose the local server using **ngrok**:

    ```bash
    ngrok http 6000
    ```

## Node list
- `alexa_skill_interface.py` – Flask server handling Alexa voice commands



## Example Alexa Voice Commands

- "Alexa, chiedi al robot di prendere il cubetto verde"
- "Alexa, chiedi al robot di fermarsi"
- "Alexa, chiedi al robot di riprendere"

## Author
Sabrina Vinco and Ruben Malacarne