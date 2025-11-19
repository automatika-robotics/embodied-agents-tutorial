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
  # Being sourced: delegate to original ros entrypoint to preserve ROS env setup,
  # but DO NOT start ChromaDB (sourcing should be side-effect-free wrt services).
  if [ -f "/ros_entrypoint.sh" ]; then
    # Use `source` so it affects the current shell as intended by callers who source this wrapper.
    source /ros_entrypoint.sh "$@"
  else
    echo "Warning: /ros_entrypoint.sh not found while sourcing custom_entrypoint.sh"
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
  uv run chroma run --port "${CHROMA_PORT}" > /var/log/chroma.log 2>&1 &

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

# Hand off to the original ros entrypoint
if [ -f "/ros_entrypoint.sh" ]; then
  exec /ros_entrypoint.sh "$@"
else
  echo "Error: /ros_entrypoint.sh not found. Exiting."
  exit 1
fi

