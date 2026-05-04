#!/usr/bin/env bash
set -euo pipefail

echo "[INFO] Sourcing workspace: /home/hanmo/code/catkin_ws/devel/setup.bash"
source /home/hanmo/code/catkin_ws/devel/setup.bash

PKG_NAME="threshold_listener_jaka"
PKG_DIR="$(rospack find ${PKG_NAME})"
URDF_FILE="${PKG_DIR}/urdf/jaka_zu3.urdf"

echo "[INFO] Package: ${PKG_NAME}"
echo "[INFO] Package path: ${PKG_DIR}"
echo "[INFO] URDF path: ${URDF_FILE}"
echo "[INFO] Target arm: jaka4"

if [[ ! -f "${URDF_FILE}" ]]; then
  echo "[ERROR] URDF file not found: ${URDF_FILE}" >&2
  exit 1
fi

echo "[INFO] Launching openloop stack for jaka4 ..."
roslaunch threshold_listener_jaka multi_jaka_openloop.launch \
  enable_jaka1:=false enable_jaka2:=false enable_jaka3:=false enable_jaka4:=true \
  jaka4_ip:=192.168.1.103 \
  start_jaka4_demo:=true \
  urdf_file:="${URDF_FILE}"
