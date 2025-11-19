FROM ros:jazzy-ros-base

# Set Environment Variables
ENV ROS_WS=/tmp/ros2_ws
ENV ROS_VERSION=ros2
ENV ROS_DISTRO=${ROS_DISTRO}
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

# --- STAGE 0: HOT FIX ROS2 apt keyring from base container ---
# Remove old ROS/ROS2 keys if they exist
RUN rm -f /usr/share/keyrings/ros2-latest-archive-keyring.gpg \
    && rm -f /etc/apt/sources.list.d/ros2-latest.list \
    \
    && apt-get update && apt-get install -y git curl jq python3-pip \
    # Get latest ros-apt-source version from GitHub API
    && ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | jq -r '.tag_name') \
    \
    # Determine Ubuntu codename
    && UBUNTU_CODENAME=$( . /etc/os-release && echo ${UBUNTU_CODENAME:-$VERSION_CODENAME} ) \
    \
    # Download new ros2-apt-source deb
    && curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME}_all.deb" \
    \
    # Install the new keyring + source list
    && dpkg -i /tmp/ros2-apt-source.deb \
    \
    # Cleanup
    && rm -f /tmp/ros2-apt-source.deb \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- STAGE 1: Install usb_cam ---
RUN  apt-get update && apt-get install -y ros-jazzy-usb-cam \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- STAGE 2: Install Ollama --- #
RUN curl -fsSL https://ollama.com/download/ollama-linux-amd64.tgz \
    | sudo tar zx -C /usr \
    && rm -rf /var/lib/apt/lists/*

# Create ollama home and log directory
RUN mkdir -p /root/.ollama /var/log/ollama \
    && chown -R root:root /root/.ollama /var/log/ollama

# --- STAGE 3: Install ChromaDb in its own virtual environment ---
RUN python3 -m pip install --no-cache-dir --break-system-packages uv \
    && uv venv && uv pip install chromadb

# --- STAGE 4: Get EmbodiedAgents and its mandatory and optional dependencies ---
WORKDIR ${ROS_WS}/src
RUN apt update && apt install -y python3-numpy python3-opencv python3-jinja2 python3-msgpack python3-setproctitle python3-platformdirs python3-tqdm python3-toml portaudio19-dev && \
    # Install python based dependencies
    python3 -m pip install --no-cache-dir --break-system-packages httpx 'attrs>=23.2.0' msgpack-numpy pyyaml && \
    # Install optional dependencies for EmbodiedAgents
    python3 -m pip install --no-cache-dir --break-system-packages ollama redis[hiredis] pyaudio soundfile python-fasthtml monsterui onnxruntime && \
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
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_DEFAULT_CMP0148=OLD" && \
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


