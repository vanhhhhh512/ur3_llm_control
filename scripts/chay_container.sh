#!/usr/bin/env bash
# Mo mot shell trong container da co ROS 2 Humble + MoveIt + Gazebo.
# Dung tu may co man hinh (co DISPLAY) de mo Gazebo va RViz that.
set -e
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$(dirname "$REPO")/ur3_llm_ws"
mkdir -p "$WS/src"

docker run --rm -it --network host \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e NINEROUTER_API_KEY="${NINEROUTER_API_KEY:-}" \
  -e NINEROUTER_BASE_URL="${NINEROUTER_BASE_URL:-http://127.0.0.1:20128/v1}" \
  -e NINEROUTER_MODEL="${NINEROUTER_MODEL:-}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$WS:/ws" \
  -v "$REPO:/ws/src/ur3_llm_control" \
  -w /ws ur3-llm:humble "$@"
