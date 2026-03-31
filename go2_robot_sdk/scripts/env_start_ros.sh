#!/bin/bash
# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

# Manual environment setup for Jetson ROS 2 + CycloneDDS network discovery.
# Usage:
#   source /root/Projects/UnitreeGo2/ros2_ws/src/go2_robot_sdk/scripts/source_go2_network_env.sh
#
# Optional overrides:
#   ROBOT_IP=192.168.123.161 PC_IP=192.168.123.10 ROBOT_ETH_IFACE=enP8p1s0 \
#   source /root/Projects/UnitreeGo2/ros2_ws/src/go2_robot_sdk/scripts/source_go2_network_env.sh

set -e

WORKSPACE_ROOT="${WORKSPACE_ROOT:-/root/Projects/UnitreeGo2/ros2_ws}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WS_SETUP="${WS_SETUP:-${WORKSPACE_ROOT}/install/setup.bash}"
DDS_SETUP="${DDS_SETUP:-${WORKSPACE_ROOT}/src/go2_robot_sdk/scripts/setup_cyclonedds.sh}"
ROBOT_IP="${ROBOT_IP:-192.168.123.161}"
PC_IP="${PC_IP:-192.168.123.10}"
ROBOT_ETH_IFACE="${ROBOT_ETH_IFACE:-enP8p1s0}"

if command -v conda >/dev/null 2>&1; then
    conda deactivate || true
fi

if [ ! -f "${ROS_SETUP}" ]; then
    echo "ROS setup file not found: ${ROS_SETUP}"
    return 1 2>/dev/null || exit 1
fi

if [ ! -f "${WS_SETUP}" ]; then
    echo "Workspace setup file not found: ${WS_SETUP}"
    return 1 2>/dev/null || exit 1
fi

if [ ! -f "${DDS_SETUP}" ]; then
    echo "CycloneDDS setup script not found: ${DDS_SETUP}"
    return 1 2>/dev/null || exit 1
fi

source "${ROS_SETUP}"
source "${WS_SETUP}"
source "${DDS_SETUP}" "${ROBOT_IP}" "${PC_IP}" "${ROBOT_ETH_IFACE}"

echo ""
echo "Go2 ROS network discovery environment is active."
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
echo "  ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-<unset>}"
echo "  CYCLONEDDS_URI=${CYCLONEDDS_URI:-<unset>}"
echo ""
echo "Next step: run your ROS 2 nodes or launch file manually."
