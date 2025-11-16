from typing import Optional
from agents.components import VLM, MapEncoding, Vision
from agents.clients import OllamaClient, ChromaClient
from agents.models import OllamaModel
from agents.vectordbs import ChromaDB
from agents.config import MapConfig, VisionConfig
from agents.ros import Topic, Launcher, MapLayer, FixedInput

HOST = "localhost"

image0 = Topic(name="image_raw", msg_type="Image")

# Create a detection topic
detections_topic = Topic(name="detections", msg_type="Detections")

# # Add an object detection model
# object_detection = VisionModel(
#     name="object_detection", checkpoint="dino-4scale_r50_8xb2-12e_coco"
# )
# roboml_detection = RoboMLRESPClient(object_detection, host=HOST)

# Initialize the Vision component
detection_config = VisionConfig(threshold=0.5, enable_local_classifier=True)  # Use a local small classifier
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
layer1 = MapLayer(subscribes_to=detections_topic, temporal_change=True)
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

# Launch the component
launcher = Launcher()
launcher.enable_ui(outputs=[introspection_answer, detections_topic])
launcher.add_pkg(
    components=[introspector, vision, map],
    package_name="automatika_embodied_agents",
    multiprocessing=True,
)
launcher.bringup()

