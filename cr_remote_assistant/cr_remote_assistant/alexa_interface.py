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
import threading

threading.Thread(target=lambda: rclpy.init()).start()
# action_client = ActionClient(Node("alexa interface"))
 

app = Flask(__name__)

class LaunchRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "ciao! come posso esserti utile ruby?"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Hello World", speech_text)).set_should_end_session(
            False)
            
        #inserisci il goal da eseguire in ros2!!!!
        
        return handler_input.response_builder.response



class PickIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("PickIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Ok, ora prendo il cubetto verde!"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pick", speech_text)).set_should_end_session(
            True)
        return handler_input.response_builder.response


class SleepIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("SleepIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "ora sono in standby"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Sleep", speech_text)).set_should_end_session(
            True)
        return handler_input.response_builder.response

class WeakIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("SleepIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "ora sono pronto"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Weak", speech_text)).set_should_end_session(
            True)
        return handler_input.response_builder.response

class AllExceptionHandler(AbstractExceptionHandler):

    def can_handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> bool
        return True

    def handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> Response
        # Log the exception in CloudWatch Logs
        print(exception)

        speech = "Sorry, non ho capito, me lo ripeti !!"
        handler_input.response_builder.speak(speech).ask(speech)
        return handler_input.response_builder.response

skill_builder = SkillBuilder()
skill_builder.add_request_handler(LaunchRequestHandler())
skill_builder.add_request_handler(PickIntentHandler())
skill_builder.add_request_handler(SleepIntentHandler())
skill_builder.add_request_handler(WeakIntentHandler())
skill_builder.add_exception_handler(AllExceptionHandler())
# Register your intent handlers to the skill_builder object

skill_adapter = SkillAdapter(
    skill=skill_builder.create(), skill_id="amzn1.ask.skill.e02fbacc-58bc-4310-9120-f61f508599f8", app=app)

@app.route("/")
def invoke_skill():
    return skill_adapter.dispatch_request()


skill_adapter.register(app=app, route="/")

if __name__ == "__main__":
    app.run(port=6000)
