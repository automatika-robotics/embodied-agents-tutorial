import numpy as np
from typing import Optional
from agents.components import (
    VLM,
    SpeechToText,
    TextToSpeech,
    LLM,
    Vision,
    MapEncoding,
    SemanticRouter,
)
from agents.clients import RoboMLWSClient
from agents.clients import ChromaClient
from agents.clients import OllamaClient
from agents.models import Whisper, SpeechT5, OllamaModel
from agents.vectordbs import ChromaDB
from agents.config import VisionConfig, LLMConfig, MapConfig, SemanticRouterConfig
from agents.ros import Topic, Launcher, FixedInput, MapLayer, Route

HOST = "localhost"

### Setup our models and vectordb ###
whisper = Whisper(name="whisper")
whisper_client = RoboMLWSClient(whisper, host=HOST)
speecht5 = SpeechT5(name="speecht5")
speecht5_client = RoboMLWSClient(speecht5, host=HOST)
qwen = OllamaModel(name="qwen-vl", checkpoint="qwen2.5vl:7b")
qwen_client = OllamaClient(qwen, host=HOST)
llama = OllamaModel(name="llama", checkpoint="llama3.2:3b")
llama_client = OllamaClient(llama, host=HOST)
chroma = ChromaDB(ollama_host=HOST)
chroma_client = ChromaClient(db=chroma, host=HOST, port=8080)

### Setup our components ###
# Setup a speech to text component
audio_in = Topic(name="audio0", msg_type="Audio")
audio_out = Topic(name="audio1", msg_type="Audio")
main_query = Topic(name="question", msg_type="String")

speech_to_text = SpeechToText(
    inputs=[audio_in],
    outputs=[main_query],
    model_client=whisper_client,
    trigger=audio_in,
    component_name="speech_to_text",
)

# Setup a text to speech component
query_answer = Topic(name="answer", msg_type="String")


text_to_speech = TextToSpeech(
    inputs=[query_answer],
    outputs=[audio_out],
    trigger=query_answer,
    model_client=speecht5_client,
    component_name="text_to_speech",
)

# Setup a vision component for object detection
image0 = Topic(name="image_raw", msg_type="Image")
detections_topic = Topic(name="detections", msg_type="Detections")

detection_config = VisionConfig(threshold=0.5, enable_local_classifier=True)
vision = Vision(
    inputs=[image0],
    outputs=[detections_topic],
    trigger=image0,
    config=detection_config,
    component_name="object_detection",
)

# Define a generic mllm component for vqa
vlm_query = Topic(name="mllm_query", msg_type="String")

vlm = VLM(
    inputs=[vlm_query, image0, detections_topic],
    outputs=[query_answer],
    model_client=qwen_client,
    trigger=vlm_query,
    component_name="visual_q_and_a",
)

vlm.set_component_prompt(
    template="""Imagine you are a robot.
    This image has following items: {{ detections }}.
    Answer the following about this image: {{ text0 }}"""
)

# Define a fixed input mllm component that does introspection
introspection_query = FixedInput(
    name="introspection_query",
    msg_type="String",
    fixed="What kind of a room is this? Is it an office, a bedroom or a kitchen? Give a one word answer, out of the given choices",
)
introspection_answer = Topic(name="introspection_answer", msg_type="String")

introspector = VLM(
    inputs=[introspection_query, image0],
    outputs=[introspection_answer],
    model_client=qwen_client,
    trigger=15.0,
    component_name="introspector",
)


def introspection_validation(output: str) -> Optional[str]:
    for option in ["office", "bedroom", "kitchen"]:
        if option in output.lower():
            return option


introspector.add_publisher_preprocessor(introspection_answer, introspection_validation)

# Define a semantic map using MapEncoding component
layer1 = MapLayer(subscribes_to=detections_topic, temporal_change=True)
layer2 = MapLayer(
    subscribes_to=introspection_answer,
    resolution_multiple=3,
    pre_defined=[(np.array([4.0, 5.0, 0.0]), "The door is here. DOOR.")],
)

position = Topic(name="odom", msg_type="Odometry")
map_topic = Topic(name="map", msg_type="OccupancyGrid")

map_conf = MapConfig(map_name="map")
map = MapEncoding(
    layers=[layer1, layer2],
    position=position,
    map_topic=map_topic,
    config=map_conf,
    db_client=chroma_client,
    trigger=15.0,
    component_name="map_encoder",
)

# Define a generic LLM component
llm_query = Topic(name="llm_query", msg_type="String")

llm = LLM(
    inputs=[llm_query],
    outputs=[query_answer],
    model_client=llama_client,
    trigger=[llm_query],
    component_name="general_q_and_a",
)

# Define a Go-to-X component using LLM
goto_query = Topic(name="goto_query", msg_type="String")
goal_point = Topic(name="goal_point", msg_type="PoseStamped")

goto_config = LLMConfig(
    enable_rag=True,
    collection_name="map",
    distance_func="l2",
    n_results=1,
    add_metadata=True,
)

goto = LLM(
    inputs=[goto_query],
    outputs=[goal_point],
    model_client=llama_client,
    config=goto_config,
    db_client=chroma_client,
    trigger=goto_query,
    component_name="go_to_x",
)

# set a component prompt
goto.set_component_prompt(
    template="""What are the position coordinates in the given metadata?"""
)

# add a tool call for retreiving a numpy array that can be converted to PoseStamped
def get_coordinates(position: list[float]) -> np.ndarray:
    """Get position coordinates"""
    return np.array(position, dtype=float)

function_description = {
    "type": "function",
    "function": {
        "name": "get_coordinates",
        "description": "Get position coordinates",
        "parameters": {
            "type": "object",
            "properties": {
                "position": {
                    "type": "list[float]",
                    "description": "The position coordinates in x, y and z",
                }
            },
        },
        "required": ["position"],
    },
}

# add the pre-processing function to the goal_point output topic
goto.register_tool(
    tool=get_coordinates,
    tool_description=function_description,
    send_tool_response_to_model=False,
)

# Define a semantic router between a generic LLM component, VQA MLLM component and Go-to-X component
goto_route = Route(
    routes_to=goto_query,
    samples=[
        "Go to the door",
        "Go to the kitchen",
        "Get me a glass",
        "Fetch a ball",
        "Go to hallway",
    ],
)

llm_route = Route(
    routes_to=llm_query,
    samples=[
        "What is the capital of France?",
        "Is there life on Mars?",
        "How many tablespoons in a cup?",
        "How are you today?",
        "Whats up?",
    ],
)

mllm_route = Route(
    routes_to=vlm_query,
    samples=[
        "Are we indoors or outdoors",
        "What do you see?",
        "Whats in front of you?",
        "Where are we",
        "Do you see any people?",
        "How many things are infront of you?",
        "Is this room occupied?",
    ],
)

router_config = SemanticRouterConfig(router_name="go-to-router", distance_func="l2")
# Initialize the router component
router = SemanticRouter(
    inputs=[main_query],
    routes=[llm_route, goto_route, mllm_route],
    default_route=llm_route,
    config=router_config,
    db_client=chroma_client,
    component_name="router",
)

# Launch the components
launcher = Launcher()
launcher.enable_ui(inputs=[main_query], outputs=[query_answer, detections_topic])
launcher.add_pkg(
    components=[
        vlm,
        llm,
        goto,
        introspector,
        map,
        router,
        speech_to_text,
        text_to_speech,
        vision,
    ],
    package_name="automatika_embodied_agents",
    multiprocessing=True,
)
launcher.on_fail(action_name="restart")
launcher.fallback_rate = 1 / 10  # 0.1 Hz or 10 seconds
launcher.bringup()

