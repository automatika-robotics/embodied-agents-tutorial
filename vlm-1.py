from agents.components import VLM
from agents.clients.ollama import OllamaClient
from agents.models import OllamaModel
from agents.ros import Topic, Launcher

HOST = "localhost"

# Define input and output topics (pay attention to msg_type)
vlm_query = Topic(name="vlm_query", msg_type="String")
answer = Topic(name="answer", msg_type="String")

image0 = Topic(name="image_raw", msg_type="Image")

# Define a model client (working with Ollama in this case)
# OllamaModel is a generic wrapper for all Ollama models
qwen = OllamaModel(name="qwen-vl", checkpoint="qwen2.5vl:7b")
qwen_client = OllamaClient(qwen, host=HOST)

# Define a VLM component (A component represents a node with a particular functionality)
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

# Launch the component
launcher = Launcher()
launcher.enable_ui(inputs=[vlm_query], outputs=[answer, image0])
launcher.add_pkg(
    components=[vlm],
    package_name="automatika_embodied_agents",
    multiprocessing=True,
)
launcher.bringup()

