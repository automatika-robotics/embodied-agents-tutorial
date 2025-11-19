#!/usr/bin/env bash
# wrapper_entrypoint.sh
# Wrapper that only launches ChromaDB when executed (not when sourced).
#
# Usage:
#  - Executed as ENTRYPOINT: starts chroma (unless ENABLE_CHROMA=0) and then calls /ros_entrypoint.sh "$@"
#  - Sourced into a shell: simply sources /ros_entrypoint.sh and returns (no chroma startup)

# Detect if script is being sourced:
# If ${BASH_SOURCE[0]} != "$0" then the script is sourced into the current shell.
if [ "${BASH_SOURCE[0]}" != "$0" ]; then
  # Being sourced: Source ROS,
  # but DO NOT start ChromaDB (sourcing should be side-effect-free wrt services).
  if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
   echo "Sourcing /opt/ros/${ROS_DISTRO}/setup.bash"
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
  else
   echo "Warning: ROS2 ${ROS_DISTRO} installation not found while sourcing wrapper_entrypoint.sh"
  fi
  return 0
fi

# If we reach here, script is being executed as PID 1 (or by exec).
set -euo pipefail

# Configurable environment variables:
# ENABLE_CHROMA=0 to disable starting chroma
# CHROMA_PORT=#### to override port (default 8080)
ENABLE_CHROMA="${ENABLE_CHROMA:-1}"
CHROMA_PORT="${CHROMA_PORT:-8080}"

start_chroma() {
  # Ensure log/var directories exist and are writable
  mkdir -p /var/log /var/run || true

  echo "Starting ChromaDB on port ${CHROMA_PORT}..."
  # start in background, redirect stdout/stderr to log
  #source /.venv/bin/activate
  uv run chroma run --port "${CHROMA_PORT}" > /var/log/chroma.log 2>&1 &
  #deactivate

  CHROMA_PID=$!
  echo "${CHROMA_PID}" > /var/run/chroma.pid
  echo "ChromaDB started with PID ${CHROMA_PID}; logs: /var/log/chroma.log"
}

if [ "${ENABLE_CHROMA}" != "0" ]; then
  # Start chroma in background
  start_chroma
else
  echo "ENABLE_CHROMA=0; not starting ChromaDB."
fi

# Source ros
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "Sourcing /opt/ros/${ROS_DISTRO}/setup.bash"
    set +u
    source "/opt/ros/iron/setup.bash"
fi

exec "$@"
