FROM nvidia/cuda:12.1.0-cudnn8-devel-ubuntu22.04

# Set Environment Variables
ENV ROS_WS=/tmp/ros2_ws
ENV ROS_VERSION=ros2
ENV ROS_DISTRO=iron
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

# Set environment variables for non-interactive installs and locale
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# --- STAGE 0: Install ROS2 IRON ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        locales \
        git \
        curl \
        jq \
        python3-pip \
        software-properties-common \
        tzdata \
    # Generate locale
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    # Add universe repository
    && add-apt-repository universe \
    # Get latest ros-apt-source version from GitHub API
    && ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | jq -r '.tag_name') \
    \
    # Determine Ubuntu codename
    && UBUNTU_CODENAME=$( . /etc/os-release && echo ${UBUNTU_CODENAME:-$VERSION_CODENAME} ) \
    \
    # Download new ros2-apt-source deb
    && curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME}_all.deb" \
    # Install the new keyring + source list
    && dpkg -i /tmp/ros2-apt-source.deb \
    \
    && apt-get update && apt-get install -y ros-dev-tools ros-${ROS_DISTRO}-ros-base \
    # Cleanup
    && rm -f /tmp/ros2-apt-source.deb \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- STAGE 1: Install usb_cam ---
RUN  apt-get update && apt-get install -y ros-${ROS_DISTRO}-usb-cam \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- STAGE 2: Install Ollama --- #
RUN curl -fsSL https://ollama.com/download/ollama-linux-amd64.tgz \
    | sudo tar zx -C /usr \
    && rm -rf /var/lib/apt/lists/*

# Create ollama home and log directory
RUN mkdir -p /root/.ollama /var/log/ollama \
    && chown -R root:root /root/.ollama /var/log/ollama

# --- STAGE 3: Install ChromaDb and RoboML in their own virtual environment ---
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    python3-dev ffmpeg libsm6 libxext6 \
    && apt-get clean && rm -rf /var/lib/apt/lists/* \
    # Install uv
    && python3 -m pip install uv --no-cache-dir \
    && git clone https://github.com/automatika-robotics/roboml.git \
    # Create uv environment
    && uv venv && . .venv/bin/activate && uv pip install -v ./roboml/ \
    # Test PyTorch
    && python -c "import torch; print('PyTorch OK:', torch.cuda.is_available())" \
    # Install OpenMMLab deps
    && uv pip install --no-cache-dir mmcv==2.1.0 -f https://download.openmmlab.com/mmcv/dist/cu121/torch2.1/index.html \
    \
    # Install mmdetection (editable)
    && git clone https://github.com/open-mmlab/mmdetection.git \
    && uv pip install --no-build-isolation --no-cache-dir -e ./mmdetection/ \
    \
    # Cleanup
    && rm -rf /roboml \
    && uv pip install chromadb --no-cache-dir && deactivate

# --- STAGE 4: Get EmbodiedAgents and its mandatory and optional dependencies ---
WORKDIR ${ROS_WS}/src
RUN apt update && apt install -y portaudio19-dev && \
    # Install python based dependencies
    python3 -m pip install --no-cache-dir numpy opencv-python-headless jinja2 msgpack setproctitle platformdirs tqdm toml httpx 'attrs>=23.2.0' msgpack-numpy pyyaml && \
    # Install optional dependencies for EmbodiedAgents
    python3 -m pip install --no-cache-dir ollama redis[hiredis] pyaudio soundfile python-fasthtml monsterui onnxruntime && \
    rm -rf /root/.cache/pip && \
    set -eux; \
    for repo in \
    "automatika-robotics/sugarcoat" \
    "automatika-robotics/embodied-agents"; \
    do \
    name=$(basename "$repo"); \
    git clone --depth 1 "https://github.com/$repo.git" "$name"; \
    done && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# --- STAGE 5: Build EMOS Packages ---
WORKDIR ${ROS_WS}
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --merge-install --install-base ${ROS_ROOT} \
    --cmake-args -DCMAKE_BUILD_TYPE=Release" && \
    apt-get clean && rm -rf /var/lib/apt/lists/* && rm -rf ${ROS_WS} /root/.ros /tmp/*

# --- STAGE 6: Get Tutorial and Final Configuration ---
# Get the tutorial files
WORKDIR /embodied-agents-tutorial

# Unset ROS_WS
ENV ROS_WS=

# Add wrapper entrypoint
COPY wrapper_entrypoint.sh /wrapper_entrypoint.sh
RUN chmod +x /wrapper_entrypoint.sh

ENTRYPOINT ["/wrapper_entrypoint.sh"]
CMD ["bash"]


