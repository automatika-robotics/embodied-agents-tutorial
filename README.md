# Embodied Agents Tutorial

This tutorial will familiarize you with the basic workings of [EmbodiedAgents](https://github.com/automatika-robotics/embodied-agents), a fully-loaded ROS2 based framework for creating interactive physical agents that can understand, remember, and act upon contextual information from their environment.

Join our 👾[Discord](https://discord.gg/B9ZU6qjzND) for any questions you might have about the tutorial. 

## Getting Started

To get started you will need docker and ollama installed on your system.

1. Find docker installation instructions on the [official site](https://docs.docker.com/engine/install/).

2. Install Ollama on your system by running the following in your terminal:

```shell
curl -fsSL https://ollama.com/install.sh | sh
```

Once Ollama is installed, pull a couple of useful models that we will use in the tutorial, so that we don't have wait for them to be pulled by the recipes:

```shell
ollama pull llama3.2:3b
ollama pull qwen2.5vl:7b
```

3. Pull this repository and cd into it.

```shell
git clone https://github.com/automatika-robotics/embodied-agents-tutorial.git && cd embodied-agents-tutorial
```

4. Pull the appropriate tutorial container.

- For machines without a GPU, pull the CPU container. Even though the container was made with x86, it should work fine on ARM devices.

```shell
docker run -it --privileged --network host --name=ea_tutorial -v .:/embodied-agents-tutorial ghcr.io/automatika-robotics/embodied-agents-tutorial:cpu_x86
```

- For machines with an NVIDIA GPU, pull the GPU container as follows:

```shell
docker run -it --privileged --network host --gpus all --name=ea_tutorial -v .:/embodied-agents-tutorial ghcr.io/automatika-robotics/embodied-agents-tutorial:gpu_nvidia
```

> [!NOTE]
> The container has ROS2 and EmbodiedAgents installed, along with all the basic dependencies. It also contains Ollama, however we don't need to pull models inside the container, as setting network to `host` would ensure that we use the Ollama server we installed on our system earlier. Naming the container with `--name=ea_tutorial` makes sure we can attach to the same container from multiple terminals easily. We have also mounted our tutorial files with `-v .:/embodied-agents-tutorial`, so that we can change and play with the recipes on our machine.

You will run recipes from the terminal from which you ran the `docker run` command. Now we can start having fun!

## Recipe 0 - An agent that can answer questions:

Once in the container run the first recipe as follows:

`python3 start-0.py`

Now that the recipe is running, we can publish to the `llm_query` topic and listen for an answer on the `answer` topic.

From another terminal do the following to start a listener:

```shell
docker exec -it ea_tutorial bash
source /wrapper_entrypoint.sh
ros2 topic echo /answer
```

From yet another terminal we can do the following for publishing a question:

```shell
docker exec -it ea_tutorial bash
source /wrapper_entrypoint.sh
ros2 topic pub /llm_query std_msgs/msg/String "{data : 'How are you doing ?'}" --once
```

We should be able to see the answer from our agent in the terminal running the listener. However we don't need to use the terminals as we can use the dynamic UI created by the recipe for talking to the agent. You can stop the recipe by pressing `Ctrl+c` and uncomment line 33 to enable the UI, then run the recipe again.

Go to `http://localhost:5001` on your browser to access the UI. Thats it for the first recipe.

## Recipe 1 - An agent that can see:

This recipe requires the webcam as we want to make our agent see. So from a new terminal we will run the following to publish images from our webcam:

```shell
docker exec -it ea_tutorial bash
source /wrapper_entrypoint.sh
ros2 run usb_cam usb_cam_node_exe
```

Now in the terminal where we started the container, run the recipe as follows:

`python3 vlm-1.py`

Go to `http://localhost:5001` on your browser to access the UI. We can see the webcam stream and ask the agent questions about what it sees. _Tip: Sometimes, refreshing the UI page helps._

## Recipe 2 - An agent that can listen and talk:

> [!CAUTION]
> This recipe only works on the GPU container that has roboml installed. You can skip this one if you are using the CPU container.

First we will turn on the webcam similar to the previous recipe.

```shell
docker exec -it ea_tutorial bash
source /wrapper_entrypoint.sh
ros2 run usb_cam usb_cam_node_exe
```

Then in a new terminal we will run roboml server which exists inside the container.

```shell
docker exec -it ea_tutorial bash
source /.venv/bin/activate
roboml
```

Now in the terminal where we started the container, we will run the recipe as follows:

`python3 speech-2.py`

Go to `http://localhost:5001` on your browser to access the UI.

## Recipe 3 - Making the agent smarter with routing:

In this recipe we will route the input between two components. First we will turn on the webcam similar to the previous recipes.

```shell
docker exec -it ea_tutorial bash
source /wrapper_entrypoint.sh
ros2 run usb_cam usb_cam_node_exe
```

Now in the terminal where we started the container, we will run the recipe as follows:

`python3 routing-3.py`

Go to `http://localhost:5001` on your browser to access the UI. Ask the agent a general question like **Whats the capital of France** and a contextual question like **What do you see?** and you should receive answers from different components.

## Recipe 4 - Giving the agent a memory:

First we will turn on the webcam similar to the previous recipes.

```shell
docker exec -it ea_tutorial bash
source /wrapper_entrypoint.sh
ros2 run usb_cam usb_cam_node_exe
```

Now in the terminal where we started the container, we will run the recipe as follows:

`python3 map-4.py`

Go to `http://localhost:5001` on your browser to access the UI. This recipe illustrates the MapEncoding (memory) component. So there is no input and output. However we can see the answers appearing in the log. These answers will be stored in the Layer taking input from the VLM component (when we have position and map information being published on the robot).

## Recipe 5 - Making the agent use tools:

In this recipe we will illustrate the use of LLM component with RAG and tool calling capabilities to create a component that can process commands of the format **Go to X** . First we will turn on the webcam similar to the previous recipes.

```shell
docker exec -it ea_tutorial bash
source /wrapper_entrypoint.sh
ros2 run usb_cam usb_cam_node_exe
```

Now in the terminal where we started the container, we will run the recipe as follows:

`python3 tool_calling-5.py`

We gave a fixed coordinate for the door in our recipe and we expect the LLM component to retreive it from memory and publish it as a `PoseStamped` message. So before sending a `Go to X` command to the agent we will open another terminal and listen on the `/goal_point` topic. We can do this in a new terminal as follows:

```shell
docker exec -it ea_tutorial bash
source /wrapper_entrypoint.sh
ros2 topic echo /goal_point
```

Now we can access the UI at `http://localhost:5001` on our browser and say something like **Go to the door**. We should see a PoseStamped message published on the second terminal.

## Recipe 6 - An Arbitrarily Complex Agent:

> [!CAUTION]
> This recipe only works on the GPU container that has roboml installed. However, you can comment out the SpeechToText and TextToSpeech components (which use RoboML) to run it with the CPU container.

First we will turn on the webcam similar to the previous recipe.

```shell
docker exec -it ea_tutorial bash
source /wrapper_entrypoint.sh
ros2 run usb_cam usb_cam_node_exe
```

Then in a new terminal we will run roboml server which exists inside the container. _If on the CPU container, skip this step._

```shell
docker exec -it ea_tutorial bash
cd / && source .venv/bin/activate
roboml
```

Now in the terminal where we started the container, we will run the recipe as follows:

`python3 embodied_agent-6.py`

Go to `http://localhost:5001` on your browser to access the UI and talk to the agent.

## Next Steps

- You can learn more by following more elaborate recipe examples provided in the [Docs](https://automatika-robotics.github.io/embodied-agents/).
  
- Join the 👾[Discord community](https://discord.gg/B9ZU6qjzND), to get immediate answers to your questions and sharing your creations.
  
- Contribute to [EmbodiedAgents](https://github.com/automatika-robotics/embodied-agents) on Github by opening bug reports and feature requests.
