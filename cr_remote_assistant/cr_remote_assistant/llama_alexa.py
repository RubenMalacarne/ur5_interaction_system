#!/usr/bin/env python3
import os
from flask import Flask
from ask_sdk_core.skill_builder import SkillBuilder
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.dispatch_components import (AbstractRequestHandler, AbstractExceptionHandler)
from ask_sdk_model.ui import SimpleCard
from flask_ask_sdk.skill_adapter import SkillAdapter
from llama_cpp import Llama

app = Flask(__name__)

LLAMA_MODEL_PATH = os.getenv(
    "LLAMA_MODEL_PATH",
    "./llama_model/llama-2-7b-chat.Q4_K_M.gguf"
)
llm = Llama(model_path=LLAMA_MODEL_PATH, n_ctx=512)

class LaunchRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        speech_text = "Ciao! ora stai usando llama io faro da ponte, dimmi pure cosa posso fare per te."
        handler_input.response_builder.speak(speech_text) \
            .set_card(SimpleCard("Hello World", speech_text)) \
            .set_should_end_session(False)
        return handler_input.response_builder.response

#il comando di alexa deve avere una AMAZON.SearchQuery
class CommandIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_intent_name("CommandIntent")(handler_input)

    def handle(self, handler_input):
        # testo dettato dall’utente
        user_input = handler_input.request_envelope.request.intent.slots["comando"].value
        print(f"🗣️ Comando da Alexa: {user_input}")
        prompt = f"""
                        now extract the action and the object from JSON.

                        Command: "{user_input}"
                        Output:
                        """
        response = llm(prompt, max_tokens=200, stop=["\n\n"])
        parsed_intent = response["choices"][0]["text"].strip()
        print("✅ Intent find:", parsed_intent)

        speech_text = "ora elaboro."
        handler_input.response_builder.speak(speech_text) \
            .set_card(SimpleCard("Intent", parsed_intent)) \
            .set_should_end_session(True)

        return handler_input.response_builder.response

class SessionEndedRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return is_request_type("SessionEndedRequest")(handler_input)

    def handle(self, handler_input):
        return handler_input.response_builder.response


class AllExceptionHandler(AbstractExceptionHandler):
    def can_handle(self, handler_input, exception):
        return True

    def handle(self, handler_input, exception):
        print(exception)

        speech = "accipicchia, non ho capito bene!!"
        handler_input.response_builder.speak(speech).ask(speech)
        return handler_input.response_builder.response


sb = SkillBuilder()
sb.add_request_handler(LaunchRequestHandler())
sb.add_request_handler(CommandIntentHandler())
sb.add_request_handler(SessionEndedRequestHandler())
sb.add_exception_handler(AllExceptionHandler())
# Register your intent handlers to the skill_builder object

SKILL_ID = "amzn1.ask.skill.10748845-77c3-4195-82e4-b2ab915a5d1a"

skill_adapter = SkillAdapter(
    skill=sb.create(),
    skill_id=SKILL_ID,
    app=app
)

skill_adapter.register(app=app, route="/")

if __name__ == "__main__":
    app.run(port=6000)
