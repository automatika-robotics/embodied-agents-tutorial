import numpy as np
from typing import Optional
from agents.components import VLM, MapEncoding, Vision, LLM
from agents.clients import OllamaClient, ChromaClient
from agents.models import OllamaModel
from agents.vectordbs import ChromaDB
from agents.config import MapConfig, VisionConfig, LLMConfig
from agents.ros import Topic, Launcher, MapLayer, FixedInput

HOST = "localhost"

image0 = Topic(name="image_raw", msg_type="Image")

# Create a detection topic
detections_topic = Topic(name="detections", msg_type="Detections")

# Initialize the Vision component
detection_config = VisionConfig(threshold=0.5, enable_local_classifier=True)
vision = Vision(
    inputs=[image0],
    outputs=[detections_topic],
    trigger=image0,
    config=detection_config,
    component_name="detection_component",
)

# Define a model client (working with Ollama in this case)
qwen = OllamaModel(name="qwen-vl", checkpoint="qwen2.5vl:7b")
qwen_client = OllamaClient(qwen, host=HOST)

# Define a fixed input for the component
introspection_query = FixedInput(
    name="introspection_query",
    msg_type="String",
    fixed="What kind of a room is this? Is it an office, a bedroom or a kitchen? Give a one word answer, out of the given choices",
)
# Define output of the component
introspection_answer = Topic(name="introspection_answer", msg_type="String")

# Start a timed (periodic) component using the mllm model defined earlier
# This component answers the same question after every 15 seconds
introspector = VLM(
    inputs=[introspection_query, image0],  # we use the image0 topic defined earlier
    outputs=[introspection_answer],
    model_client=qwen_client,
    trigger=10.0,  # we provide the time interval as a float value to the trigger parameter
    component_name="introspector",
)

# Define an arbitrary function to validate the output of the introspective component
# before publication.
def introspection_validation(output: str) -> Optional[str]:
    for option in ["office", "bedroom", "kitchen"]:
        if option in output.lower():
            return option


introspector.add_publisher_preprocessor(introspection_answer, introspection_validation)

### Add a semantic memory ###
# Object detection output from vision component
layer1 = MapLayer(subscribes_to=detections_topic, temporal_change=True, pre_defined=[(np.array([4.0, 5.0, 0.0]), "Door")])
# Introspection output from vlm component
layer2 = MapLayer(subscribes_to=introspection_answer, resolution_multiple=3)

# Initialize mandatory topics defining the robots localization in space
position = Topic(name="odom", msg_type="Odometry")
map_topic = Topic(name="map", msg_type="OccupancyGrid")

# Initialize a vector DB that will store our semantic map
chroma = ChromaDB(ollama_host=HOST)
chroma_client = ChromaClient(db=chroma, host=HOST, port=8080)

# Create the map component
map_conf = MapConfig(map_name="map")  # We give our map a name
map = MapEncoding(
    layers=[layer1, layer2],
    position=position,
    map_topic=map_topic,
    config=map_conf,
    db_client=chroma_client,
    trigger=15.0,
    component_name="map_encoding",
)

### Add a Go-to-X component that uses the map ###
# Define LLM input and output topics including goal_point topic of type PoseStamped
goto_in = Topic(name="goto_in", msg_type="String")
goal_point = Topic(name="goal_point", msg_type="PoseStamped")

# Start a Llama3.2 based llm component using ollama client
llama = OllamaModel(name="llama", checkpoint="llama3.2:3b")
llama_client = OllamaClient(llama, host=HOST)

config = LLMConfig(
    enable_rag=True,
    collection_name="map",
    distance_func="l2",
    n_results=1,
    add_metadata=True,
)

# initialize the component
goto = LLM(
    inputs=[goto_in],
    outputs=[goal_point],
    model_client=llama_client,
    db_client=chroma_client,
    trigger=goto_in,
    config=config,
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

# Launch the component
launcher = Launcher()
launcher.enable_ui(inputs=[goto_in])
launcher.add_pkg(
    components=[introspector, vision, map, goto],
    package_name="automatika_embodied_agents",
    multiprocessing=True,
)
launcher.bringup()

