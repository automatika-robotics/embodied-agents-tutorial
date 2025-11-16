from agents.components import VLM, SpeechToText, TextToSpeech
from agents.clients import OllamaClient, RoboMLWSClient
from agents.models import Whisper, SpeechT5, OllamaModel
from agents.ros import Topic, Launcher

HOST = "localhost"

vlm_query = Topic(name="vlm_query", msg_type="String")
answer = Topic(name="answer", msg_type="String")

# Image topic
image0 = Topic(name="image_raw", msg_type="Image")

# Audio input topic
audio_in = Topic(name="audio0", msg_type="Audio")

whisper = Whisper(name="whisper")  # Custom model init params can be provided here
roboml_whisper = RoboMLWSClient(whisper, host=HOST)

speech_to_text = SpeechToText(
    inputs=[audio_in],
    outputs=[vlm_query],
    model_client=roboml_whisper,
    trigger=audio_in,
    component_name="speech_to_text",
)

qwen = OllamaModel(name="qwen-vl", checkpoint="qwen2.5vl:7b")
qwen_client = OllamaClient(qwen, host=HOST)

vlm = VLM(
    inputs=[vlm_query, image0],
    outputs=[answer],
    model_client=qwen_client,
    trigger=vlm_query,
    component_name="vqa",
)

# Additional prompt settings
vlm.set_system_prompt(
    prompt="""You are a robot who can see using the image provided in this query.
    Answer the questions you are asked consicely."""
)

# Audio output topic
audio_out = Topic(name="audio_out", msg_type="Audio")

speecht5 = SpeechT5(name="speecht5")
roboml_speecht5 = RoboMLWSClient(speecht5, host=HOST)
text_to_speech = TextToSpeech(
    inputs=[answer],
    outputs=[audio_out],
    trigger=answer,
    model_client=roboml_speecht5,
    component_name="text_to_speech",
)

launcher = Launcher()
launcher.enable_ui(inputs=[vlm_query, audio_in], outputs=[image0, audio_out])  # specify topics
launcher.add_pkg(components=[speech_to_text, vlm, text_to_speech])
launcher.bringup()
