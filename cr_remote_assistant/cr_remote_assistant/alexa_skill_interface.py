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
- Publishes to: `/cr/pause_command` (std_msgs.msg.String)
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

from cr_interfaces.msg import ObjectInfoArray
from cr_interfaces.action import ExecuteWorkflow

threading.Thread(target=lambda: rclpy.init()).start()



class AlexaNode(Node):
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
            String,
            '/cr/pause_command',
            10
        )

    def object_callback(self, msg):
        self.latest_objects = msg.objects

alexa_node = AlexaNode()
action_client = ActionClient(alexa_node, ExecuteWorkflow, '/cr/execute_workflow')

#function to take the lowest id of an object with a specific label
def get_lowest_id_by_label(objects, target_label):
    filtered = [obj for obj in objects if obj.label == target_label]
    if not filtered:
        return None
    return min(filtered, key=lambda x: x.id).id

app = Flask(__name__)

# first function --> used when the skill is launched
class LaunchRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        speech_text = "cosa devo prendere?"
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Hello World", speech_text)).set_should_end_session(
            False)
        
        return handler_input.response_builder.response

# second function --> used when the user asks to pick a green cube
class PickGreenIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_intent_name("PrendiCuboVerdeIntent")(handler_input)

    def handle(self, handler_input):
        # Rispondi subito ad Alexa
        speech_text = "Ok, prendo il cubetto verde..."

        # Esegui il resto in background (thread)
        def ros_action():
            nonlocal speech_text #usato per aggiornare quello che deve dire alexa
            rclpy.spin_once(alexa_node, timeout_sec=1.0)
            object_id = get_lowest_id_by_label(alexa_node.latest_objects, "green_cube")
            if object_id is None:
                speech_text = "Nessun cubetto verde trovato."
                alexa_node.get_logger().error("Nessun cubetto verde trovato.")
                return

            speech_text = f"Il robot andrà a prendere il cubetto verde con id {object_id}."
            alexa_node.get_logger().info(speech_text)

            goal = ExecuteWorkflow.Goal()
            goal.object_id = object_id
            action_client.send_goal_async(goal)

        threading.Thread(target=ros_action).start()

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pick", speech_text)).set_should_end_session(True)

        return handler_input.response_builder.response
    
# second function --> used when the user asks to pick a red cube    
class PickRedIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_intent_name("PrendiCuboRossoIntent")(handler_input)

    def handle(self, handler_input):
        # Rispondi subito ad Alexa
        speech_text = "Ok, ora cerco il cubetto rosso..."

        # Esegui il resto in background
        def ros_action():
            nonlocal speech_text #usato per aggiornare quello che deve dire alexa
            rclpy.spin_once(alexa_node, timeout_sec=1.0)
            object_id = get_lowest_id_by_label(alexa_node.latest_objects, "red_cube")
            if object_id is None:
                speech_text = "Nessun cubetto rosso trovato."
                alexa_node.get_logger().error("Nessun cubetto rosso trovato.")
                return

            speech_text = f"Il robot andrà a prendere il cubetto rosso con id {object_id}."
            alexa_node.get_logger().info(speech_text)

            goal = ExecuteWorkflow.Goal()
            goal.object_id = object_id
            action_client.send_goal_async(goal)

        threading.Thread(target=ros_action).start()

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pick", speech_text)).set_should_end_session(True)

        return handler_input.response_builder.response
    
# second function --> used when the user asks to pick a color cube    
class PickColorIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_intent_name("PrendiCuboColoratoIntent")(handler_input)

    def handle(self, handler_input):
        # Rispondi subito ad Alexa
        speech_text = "Ok, prendo il cubetto colorato..."

        # Esegui il resto in background
        def ros_action():
            nonlocal speech_text
            rclpy.spin_once(alexa_node, timeout_sec=1.0)
            object_id = get_lowest_id_by_label(alexa_node.latest_objects, "red_cube")
            if object_id is None:
                speech_text = "Nessun cubetto colorato trovato."
                alexa_node.get_logger().error("Nessun cubetto rosso trovato.")
                return

            speech_text = f"Il robot andrà a prendere il cubetto colorato con id {object_id}."
            alexa_node.get_logger().info(speech_text)

            goal = ExecuteWorkflow.Goal()
            goal.object_id = object_id
            action_client.send_goal_async(goal)

        threading.Thread(target=ros_action).start()

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pick", speech_text)).set_should_end_session(True)

        return handler_input.response_builder.response
# stop the robot (pause)
class StopIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_intent_name("StopIntent")(handler_input)

    def handle(self, handler_input):
        speech_text = "il robot si sta per fermare"
        
        def ros_action():
        
            msg = String()
            msg.data = 'pause'
            alexa_node.publishers_cr_command.publish(msg)
            alexa_node.get_logger().info('Publishing: "%s"' % msg.data)

        threading.Thread(target=ros_action).start()
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Stop", speech_text)).set_should_end_session(True)
        return handler_input.response_builder.response
# resume the robot (with the last action)
class ResumeIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_intent_name("ResumeIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "il robot si muove, riparte dall'ultima esecuzione"
        
        def ros_action():
            
            msg = String()
            msg.data = 'resume'
            alexa_node.publishers_cr_command.publish(msg)
            alexa_node.get_logger().info('Publishing: "%s"' % msg.data)

        threading.Thread(target=ros_action).start()
        
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Resume", speech_text)).set_should_end_session(True)
        return handler_input.response_builder.response
#if the command is wrong
class AllExceptionHandler(AbstractExceptionHandler):
    def can_handle(self, handler_input, exception):
        return True

    def handle(self, handler_input, exception):
        print(exception)

        speech = "accipicchia, non ho capito bene!!"
        handler_input.response_builder.speak(speech).ask(speech)
        return handler_input.response_builder.response

skill_builder = SkillBuilder()
skill_builder.add_request_handler(LaunchRequestHandler())
skill_builder.add_request_handler(PickGreenIntentHandler())
skill_builder.add_request_handler(PickRedIntentHandler())
skill_builder.add_request_handler(PickColorIntentHandler())
skill_builder.add_request_handler(StopIntentHandler())
skill_builder.add_request_handler(ResumeIntentHandler())
skill_builder.add_exception_handler(AllExceptionHandler())


SKILL_ID = "amzn1.ask.skill.3ad3dd7c-03cb-4a11-94af-0ac3b027efe2"

skill_adapter = SkillAdapter(
    skill=skill_builder.create(), 
    skill_id=SKILL_ID, 
    app=app
)

@app.route("/")
def invoke_skill():
    return skill_adapter.dispatch_request()


skill_adapter.register(app=app, route="/")

if __name__ == "__main__":
    app.run(port=6000)
