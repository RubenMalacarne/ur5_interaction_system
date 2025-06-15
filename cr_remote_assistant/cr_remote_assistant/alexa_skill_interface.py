"""
alexa_skill_interface.py

Flask web server that connects an Alexa skill with a ROS 2 robotic system.

This script bridges an Alexa voice skill with a ROS 2-based robotic platform.
It uses the Flask web framework and the Alexa SDK to handle voice commands,
then communicates with ROS 2 to control a robot—for example, to pick up objects
or to pause/resume a workflow.

To enable communication with the Alexa service, expose the Flask server to the internet
using a tunneling service like ngrok:

    ngrok http 6000

Main Features:
-------------
- Embeds a ROS 2 node inside a Flask application
- Integrates Alexa skills via the Flask-Ask-SDK
- Sends ROS 2 action goals to trigger robot workflows
- Provides asynchronous execution using threading for responsiveness

ROS 2 Communication:
--------------------
- Subscribes to: `/cr/scene_objects` (cr_interfaces.msg.ObjectInfoArray)
- Publishes to: `/cr/pause_command` (std_msgs.msg.Bool)
- Publishes to: `/cr/stop_command` (std_msgs.msg.Bool)
- Sends goals to: `/cr/execute_workflow` (cr_interfaces.action.ExecuteWorkflow)
"""
from flask import Flask
from ask_sdk_core.skill_builder import SkillBuilder
from flask_ask_sdk.skill_adapter import SkillAdapter
from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model import Response
from ask_sdk_model.ui import SimpleCard
from ask_sdk_core.dispatch_components import AbstractExceptionHandler
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
import threading
from std_msgs.msg import Bool
from cr_interfaces.msg import ObjectInfoArray
from cr_interfaces.action import ExecuteWorkflow
from rclpy.action.client import GoalStatus

threading.Thread(target=lambda: rclpy.init()).start()

class AlexaNode(Node):
    """
    @class AlexaNode
    @brief A ROS 2 node that manages interaction with scene object information and robot commands.

    @details
    - Subscribes to `/cr/scene_objects` to get visible objects
    - Publishes pause/resume commands
    - Publishes stop command
    @description: you can use this node to interact with the robot's scene objects and control its actions.
    @note: This node is designed to be run within a Flask application to handle Alexa skill requests.
    """
    def __init__(self):
        super().__init__('alexa_interface')
        self.latest_objects = []

        self.subscription_id_obj = self.create_subscription(
            ObjectInfoArray,
            '/cr/scene_objects',
            self.object_callback,
            10
        )
        self.publishers_cr_command = self.create_publisher(
            Bool,
            '/cr/pause_command',
            10
        )
        self.publishers_cr_stop_command = self.create_publisher(
            Bool,
            '/cr/stop_command',
            10
        )

    def object_callback(self, msg):
        self.latest_objects = msg.objects

alexa_node = AlexaNode()
action_client = ActionClient(alexa_node, ExecuteWorkflow, '/cr/execute_workflow')

def exists_label(objects, target_label): 
    """
    @brief Checks whether a specific object label exists in the given list of objects.
    @return True if at least one object with the specified label is found; otherwise, False.
    """
    return any(obj.label == target_label for obj in objects)

# Funzione helper per gestire l'invio del goal e controllare se viene rifiutato
def send_goal_and_check_reject(object_label, color_name):
    """
    @brief Sends a workflow execution goal and checks if it's accepted.
    @param object_label The label of the object to manipulate (e.g., "green_cube").
    @param color_name Spoken name of the object (for response).
    @return Tuple (bool accepted, string response_text)
    """
    goal = ExecuteWorkflow.Goal()
    goal.object_label = object_label
    
    future = action_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(alexa_node, future, timeout_sec=2.0)
    
    if future.result() is not None:
        goal_handle = future.result()
        if goal_handle.accepted:
            alexa_node.get_logger().info(f"Goal accettato per {color_name}")
            return True, f"Il robot andrà a prendere il cubetto {color_name}"
        else:
            alexa_node.get_logger().warn(f"Goal rifiutato per {color_name}")
            return False, "Non posso, sto già prendendo un altro cubo!"
    else:
        alexa_node.get_logger().error("Timeout nell'invio del goal")
        return False, "Errore di comunicazione con il robot."

app = Flask(__name__)


class LaunchRequestHandler(AbstractRequestHandler):
    """
    @brief Handles the initial Alexa skill launch. "alexa attiva simulazione"
    """
    def can_handle(self, handler_input):
        return is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        speech_text = "cosa devo prendere?"
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Hello World", speech_text)).set_should_end_session(
            False)
        
        return handler_input.response_builder.response

class PickGreenIntentHandler(AbstractRequestHandler):
    """
    @brief Handles requests to pick up the green cube.
    """
    def can_handle(self, handler_input):
        return is_intent_name("PrendiCuboVerdeIntent")(handler_input)

    def handle(self, handler_input):
        rclpy.spin_once(alexa_node, timeout_sec=1.0)
        object_exists = exists_label(alexa_node.latest_objects, "green_cube")
        
        if object_exists is False:
            speech_text = "Nessun cubetto verde trovato."
            alexa_node.get_logger().error("Nessun cubetto verde trovato.")
        else:
            accepted, response_msg = send_goal_and_check_reject("green_cube", "verde")
            speech_text = response_msg

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pick", speech_text)).set_should_end_session(True)

        return handler_input.response_builder.response
    
class PickRedIntentHandler(AbstractRequestHandler):
    """
    @brief Handles requests to pick up the red cube.
    """
    def can_handle(self, handler_input):
        return is_intent_name("PrendiCuboRossoIntent")(handler_input)

    def handle(self, handler_input):
        rclpy.spin_once(alexa_node, timeout_sec=1.0)
        object_exists = exists_label(alexa_node.latest_objects, "red_cube")
        
        if object_exists is False:
            speech_text = "Nessun cubetto rosso trovato."
            alexa_node.get_logger().error("Nessun cubetto rosso trovato.")
        else:
            accepted, response_msg = send_goal_and_check_reject("red_cube", "rosso")
            speech_text = response_msg

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pick", speech_text)).set_should_end_session(True)

        return handler_input.response_builder.response

class PickBluIntentHandler(AbstractRequestHandler):
    
    """
    @brief Handles requests to pick up the blue cube.
    """
    def can_handle(self, handler_input):
        return is_intent_name("PrendiCuboBluIntent")(handler_input)

    def handle(self, handler_input):
        rclpy.spin_once(alexa_node, timeout_sec=1.0)
        object_exists = exists_label(alexa_node.latest_objects, "blue_cube")
        
        if object_exists is False:
            speech_text = "Nessun cubetto blu trovato."
            alexa_node.get_logger().error("Nessun cubetto blu trovato.")
        else:
            accepted, response_msg = send_goal_and_check_reject("blue_cube", "blu")
            speech_text = response_msg

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pick", speech_text)).set_should_end_session(True)

        return handler_input.response_builder.response

class StopIntentHandler(AbstractRequestHandler):
    """
    @brief Handles requests to stop current robot action.
    """
    def can_handle(self, handler_input):
        return is_intent_name("StopIntent")(handler_input)

    def handle(self, handler_input):
        speech_text = "ok, annullamento esecuzione"
        
        def ros_action():
            msg= Bool()
            msg.data = True
            alexa_node.publishers_cr_stop_command.publish(msg)
            alexa_node.get_logger().info('Publishing: "%s"' % msg.data)
            
        threading.Thread(target=ros_action).start()
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Stop", speech_text)).set_should_end_session(True)
        return handler_input.response_builder.response

class PauseIntentHandler(AbstractRequestHandler):
    """
    @brief Handles requests to pause the robot.
    """
    def can_handle(self, handler_input):
        return is_intent_name("PauseIntent")(handler_input)

    def handle(self, handler_input):
        speech_text = "ok, pausa dell'esecuzione in corso"
        
        def ros_action():
            msg= Bool()
            msg.data = True
            alexa_node.publishers_cr_command.publish(msg)
            alexa_node.get_logger().info('Publishing: "%s"' % msg.data)
            
        threading.Thread(target=ros_action).start()
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pause", speech_text)).set_should_end_session(True)
        return handler_input.response_builder.response

class ResumeIntentHandler(AbstractRequestHandler):
    """
    @brief Handles requests to resume the robot's workflow.
    """
    def can_handle(self, handler_input):
        return is_intent_name("ResumeIntent")(handler_input)

    def handle(self, handler_input):
        speech_text = "il robot si muove, riparte dall'ultima esecuzione"
        
        def ros_action():
            
            msg= Bool()
            msg.data = False
            alexa_node.publishers_cr_command.publish(msg)
            alexa_node.get_logger().info('Publishing: "%s"' % msg.data)

        threading.Thread(target=ros_action).start()
        
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Resume", speech_text)).set_should_end_session(True)
        return handler_input.response_builder.response

class AllExceptionHandler(AbstractExceptionHandler):
    """
    @brief Catches all unhandled exceptions during intent processing.
    """
    def can_handle(self, handler_input, exception):
        return True

    def handle(self, handler_input, exception):
        print(exception)

        speech = "accipicchia, non ho capito bene!!"
        handler_input.response_builder.speak(speech).ask(speech)
        return handler_input.response_builder.response


# ==================== Alexa Skill Setup ====================

skill_builder = SkillBuilder()
skill_builder.add_request_handler(LaunchRequestHandler())
skill_builder.add_request_handler(PickGreenIntentHandler())
skill_builder.add_request_handler(PickRedIntentHandler())
skill_builder.add_request_handler(PickBluIntentHandler())
skill_builder.add_request_handler(StopIntentHandler())
skill_builder.add_request_handler(ResumeIntentHandler())
skill_builder.add_request_handler(PauseIntentHandler())
skill_builder.add_exception_handler(AllExceptionHandler())

# Register your intent handlers to the skill_builder object
SKILL_ID = "amzn1.ask.skill.3ad3dd7c-03cb-4a11-94af-0ac3b027efe2"

skill_adapter = SkillAdapter(
    skill=skill_builder.create(), 
    skill_id=SKILL_ID, 
    app=app
)

@app.route("/")
def invoke_skill():
    """
    @brief Flask route to dispatch Alexa requests.
    @return Alexa response
    """
    return skill_adapter.dispatch_request()

skill_adapter.register(app=app, route="/")

if __name__ == "__main__":
    app.run(port=6000)