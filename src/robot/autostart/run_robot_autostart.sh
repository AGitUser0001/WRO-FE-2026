#!/usr/bin/env bash
set -eo pipefail

source "${HOME}/robot_ws/robot/scripts/setup_ros_env.sh"

if [[ -z "${JETSON_MODEL_NAME:-}" && -r /proc/device-tree/model ]]; then
  jetson_model="$(tr -d '\0' < /proc/device-tree/model)"
  case "${jetson_model}" in
    *"Jetson Orin Nano"*) export JETSON_MODEL_NAME=JETSON_ORIN_NANO ;;
    *"Jetson Orin NX"*) export JETSON_MODEL_NAME=JETSON_ORIN_NX ;;
    *"Jetson AGX Orin"*) export JETSON_MODEL_NAME=JETSON_ORIN ;;
  esac
fi

exec /usr/bin/python3 /usr/local/lib/wro-robot-autostart/robot_autostart.py
