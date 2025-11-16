from agents.components import LLM
from agents.clients.ollama import OllamaClient
from agents.models import OllamaModel
from agents.ros import Topic, Launcher

HOST = "localhost"

# Define input and output topics (pay attention to msg_type)
llm_query = Topic(name="llm_query", msg_type="String")
answer = Topic(name="answer", msg_type="String")

# Define a model client (working with Ollama in this case)
# OllamaModel is a generic wrapper for all Ollama models
llama = OllamaModel(name="llama", checkpoint="llama3.2:3b")
llama_client = OllamaClient(llama, host=HOST)

# Define an LLM component (A component represents a node with a particular functionality)
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
    Answer the following question: {{ llm_query }}""",
)
# Launch the component
launcher = Launcher()
# launcher.enable_ui(inputs=[llm_query], outputs=[answer])
launcher.add_pkg(
    components=[llm],
    package_name="automatika_embodied_agents",
    multiprocessing=True,
)
launcher.bringup()
