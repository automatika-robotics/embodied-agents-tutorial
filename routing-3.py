from agents.components import VLM, LLM, SemanticRouter
from agents.clients import OllamaClient, ChromaClient
from agents.models import OllamaModel
from agents.vectordbs import ChromaDB
from agents.config import SemanticRouterConfig
from agents.ros import Topic, Launcher, Route

HOST = "localhost"

# Define input and output topics (pay attention to msg_type)
vlm_query = Topic(name="vlm_query", msg_type="String")
answer = Topic(name="answer", msg_type="String")

image0 = Topic(name="image_raw", msg_type="Image")

# Define a model client (working with Ollama in this case)
# OllamaModel is a generic wrapper for all Ollama models
qwen = OllamaModel(name="qwen-vl", checkpoint="qwen2.5vl:7b")
qwen_client = OllamaClient(qwen, host=HOST)

# Define a VLM component
vlm = VLM(
    inputs=[vlm_query, image0],
    outputs=[answer],
    model_client=qwen_client,
    trigger=[vlm_query],
    component_name="vqa",
)
# Additional prompt settings
vlm.set_system_prompt(
    prompt="""You are a robot who can see using the image provided in this query.
    Answer the questions you are asked consicely."""
)

# LLM query topic
llm_query = Topic(name="llm_query", msg_type="String")

llama = OllamaModel(name="llama", checkpoint="llama3.2:3b")
llama_client = OllamaClient(llama, host=HOST)

# Define a LLM component
llm = LLM(
    inputs=[llm_query],
    outputs=[answer],
    model_client=llama_client,
    trigger=[llm_query],
    component_name="general_qa",
)

# Additional prompt settings
llm.set_topic_prompt(
    llm_query,
    template="""You are an amazing and funny robot.
    Answer the following question: {{ text0 }}""",
)

### Add a router ###
# Initialize a vector DB that will store our routes
chroma = ChromaDB(ollama_host=HOST)
chroma_client = ChromaClient(db=chroma, host=HOST, port=8080)

# Create the input topic for the router
main_query = Topic(name="question", msg_type="String")

# Define a route to a topic that processes general queries
vlm_route = Route(
    routes_to=vlm_query,
    samples=[
        "Are we indoors or outdoors",
        "What do you see?",
        "What kind of a room is this?"
        "Whats in front of you?",
        "Where are we",
        "Do you see any people?",
        "How many things are infront of you?",
        "Is this room occupied?",
    ],
)

# Define a route to a topic that is input to an LLM component
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

router_config = SemanticRouterConfig(router_name="question-router", distance_func="l2")
# Initialize the router component
router = SemanticRouter(
    inputs=[main_query],
    routes=[llm_route, vlm_route],
    default_route=llm_route,  # If none of the routes fall within a distance threshold
    config=router_config,
    db_client=chroma_client,
    component_name="router",
)

# Launch the component
launcher = Launcher()
launcher.enable_ui(inputs=[main_query], outputs=[answer, image0])
launcher.add_pkg(
    components=[llm, vlm, router],
    package_name="automatika_embodied_agents",
    multiprocessing=True,
)
launcher.bringup()

