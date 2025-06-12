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

#function to take the lowest id of an object with a specific label
def exists_label(objects, target_label):
    return any(obj.label == target_label for obj in objects)

# Funzione helper per gestire l'invio del goal e controllare se viene rifiutato
def send_goal_and_check_reject(object_label, color_name):
    goal = ExecuteWorkflow.Goal()
    goal.object_label = object_label
    
    # Invia il goal e aspetta la risposta
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
        # Esegui tutto prima di rispondere ad Alexa
        rclpy.spin_once(alexa_node, timeout_sec=1.0)
        object_exists = exists_label(alexa_node.latest_objects, "green_cube")
        
        if object_exists is False:
            speech_text = "Nessun cubetto verde trovato."
            alexa_node.get_logger().error("Nessun cubetto verde trovato.")
        else:
            # Usa la nuova funzione per controllare il reject
            accepted, response_msg = send_goal_and_check_reject("green_cube", "verde")
            speech_text = response_msg

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pick", speech_text)).set_should_end_session(True)

        return handler_input.response_builder.response
    
# second function --> used when the user asks to pick a red cube    
class PickRedIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_intent_name("PrendiCuboRossoIntent")(handler_input)

    def handle(self, handler_input):
        # Esegui tutto prima di rispondere ad Alexa
        rclpy.spin_once(alexa_node, timeout_sec=1.0)
        object_exists = exists_label(alexa_node.latest_objects, "red_cube")
        
        if object_exists is False:
            speech_text = "Nessun cubetto rosso trovato."
            alexa_node.get_logger().error("Nessun cubetto rosso trovato.")
        else:
            # Usa la nuova funzione per controllare il reject
            accepted, response_msg = send_goal_and_check_reject("red_cube", "rosso")
            speech_text = response_msg

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pick", speech_text)).set_should_end_session(True)

        return handler_input.response_builder.response

class PickBlueIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_intent_name("PrendiCuboBlueIntetnt")(handler_input)

    def handle(self, handler_input):
        # Esegui tutto prima di rispondere ad Alexa
        rclpy.spin_once(alexa_node, timeout_sec=1.0)
        object_exists = exists_label(alexa_node.latest_objects, "blue_cube")
        
        if object_exists is False:
            speech_text = "Nessun cubetto blu trovato."
            alexa_node.get_logger().error("Nessun cubetto blu trovato.")
        else:
            # Usa la nuova funzione per controllare il reject
            accepted, response_msg = send_goal_and_check_reject("blue_cube", "blu")
            speech_text = response_msg

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pick", speech_text)).set_should_end_session(True)

        return handler_input.response_builder.response

class StopIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_intent_name("StopIntent")(handler_input)

    def handle(self, handler_input):
        speech_text = "ok, annullamento esecuzione"
        
        def ros_action():
            #ritorna un bool
            msg= Bool()
            msg.data = True
            alexa_node.publishers_cr_stop_command.publish(msg)
            alexa_node.get_logger().info('Publishing: "%s"' % msg.data)
            
        threading.Thread(target=ros_action).start()
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Stop", speech_text)).set_should_end_session(True)
        return handler_input.response_builder.response

class PauseIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_intent_name("PauseIntent")(handler_input)

    def handle(self, handler_input):
        speech_text = "ok, pausa dell'esecuzione in corso"
        
        def ros_action():
            #ritorna un bool
            msg= Bool()
            msg.data = True
            alexa_node.publishers_cr_command.publish(msg)
            alexa_node.get_logger().info('Publishing: "%s"' % msg.data)
            
        threading.Thread(target=ros_action).start()
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pause", speech_text)).set_should_end_session(True)
        return handler_input.response_builder.response

class ResumeIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_intent_name("ResumeIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "il robot si muove, riparte dall'ultima esecuzione"
        
        def ros_action():
            
            #ritorna un bool
            msg= Bool()
            msg.data = False
            alexa_node.publishers_cr_command.publish(msg)
            alexa_node.get_logger().info('Publishing: "%s"' % msg.data)

        threading.Thread(target=ros_action).start()
        
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Resume", speech_text)).set_should_end_session(True)
        return handler_input.response_builder.response

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
skill_builder.add_request_handler(PickBlueIntentHandler())
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
    return skill_adapter.dispatch_request()

skill_adapter.register(app=app, route="/")

if __name__ == "__main__":
    app.run(port=6000)