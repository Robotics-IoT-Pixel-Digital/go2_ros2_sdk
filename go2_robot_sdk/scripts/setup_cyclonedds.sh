#!/bin/bash
# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

#
# Go2 Pro CycloneDDS Setup Script
# 
# This script configures the ROS2 environment for communicating with
# the Go2 Pro robot over Ethernet using CycloneDDS.
# 
# Automatically detects if running on native Linux or WSL and configures accordingly.
#
# Usage:
#   source setup_cyclonedds.sh [ROBOT_IP] [YOUR_PC_IP] [INTERFACE]
#
# Examples:
#   source setup_cyclonedds.sh                           # Use defaults
#   source setup_cyclonedds.sh 192.168.123.161          # Specify robot IP
#   source setup_cyclonedds.sh 192.168.123.161 192.168.123.100 eth0  # Full config
#

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Detect environment (WSL vs Native Linux)
detect_environment() {
    IS_WSL=false
    WSL_VERSION=""
    
    if grep -qi microsoft /proc/version 2>/dev/null; then
        IS_WSL=true
        if grep -qi "WSL2" /proc/version 2>/dev/null; then
            WSL_VERSION="2"
        else
            WSL_VERSION="1"
        fi
    elif [ -f /proc/sys/fs/binfmt_misc/WSLInterop ]; then
        IS_WSL=true
        WSL_VERSION="2"
    elif [ -n "$WSL_DISTRO_NAME" ]; then
        IS_WSL=true
        WSL_VERSION="2"
    fi
    
    if [ "$IS_WSL" = true ]; then
        echo -e "${CYAN}🐧 Environment: WSL${WSL_VERSION} (Windows Subsystem for Linux)${NC}"
    else
        echo -e "${CYAN}🐧 Environment: Native Linux${NC}"
    fi
}

# Check WSL networking mode
check_wsl_networking() {
    if [ "$IS_WSL" != true ]; then
        return 0
    fi
    
    echo ""
    echo -e "${BLUE}Checking WSL Networking Configuration...${NC}"
    
    # Check if mirrored networking is enabled (Windows 11 22H2+)
    local wsl_ip=$(hostname -I 2>/dev/null | awk '{print $1}')
    local windows_ip=$(cat /etc/resolv.conf 2>/dev/null | grep nameserver | awk '{print $2}')
    
    # In mirrored mode, WSL shares the Windows network stack
    # In NAT mode (default), WSL has its own virtual network
    
    if [ -n "$wsl_ip" ]; then
        echo -e "  WSL IP Address: ${wsl_ip}"
    fi
    
    # Check if we can see the 192.168.123.x subnet
    local go2_subnet_ip=$(ip addr show 2>/dev/null | grep -oP '192\.168\.123\.\d+' | head -1)
    
    if [ -n "$go2_subnet_ip" ]; then
        echo -e "${GREEN}✓${NC} Found IP in Go2 subnet: ${go2_subnet_ip}"
        echo -e "${GREEN}✓${NC} WSL networking appears correctly configured for Go2"
        return 0
    fi
    
    echo ""
    echo -e "${YELLOW}⚠ WSL Network Configuration Required${NC}"
    echo ""
    echo -e "WSL${WSL_VERSION} requires additional setup to communicate with the Go2 robot."
    echo ""
    
    if [ "$WSL_VERSION" = "2" ]; then
        echo -e "${CYAN}Option 1: Enable Mirrored Networking (Recommended for Windows 11)${NC}"
        echo "  1. Create/edit %USERPROFILE%\\.wslconfig in Windows:"
        echo ""
        echo "     [wsl2]"
        echo "     networkingMode=mirrored"
        echo ""
        echo "  2. Restart WSL: wsl --shutdown (in PowerShell)"
        echo "  3. Re-run this script"
        echo ""
        echo -e "${CYAN}Option 2: USB/IP Passthrough for Ethernet Adapter${NC}"
        echo "  1. Install usbipd-win on Windows"
        echo "  2. Attach your USB Ethernet adapter to WSL"
        echo "  3. Configure the interface in WSL"
        echo ""
        echo -e "${CYAN}Option 3: Bridge Network (Advanced)${NC}"
        echo "  1. Create a network bridge in Windows including your Ethernet adapter"
        echo "  2. Configure WSL to use the bridge"
        echo ""
    else
        echo -e "${YELLOW}WSL1 shares the Windows network stack directly.${NC}"
        echo "  Ensure your Windows Ethernet adapter has an IP in 192.168.123.x subnet."
    fi
    
    return 1
}

echo -e "${BLUE}=========================================${NC}"
echo -e "${BLUE}  Go2 Pro CycloneDDS Setup${NC}"
echo -e "${BLUE}=========================================${NC}"

# Detect environment first
detect_environment

# Default values
DEFAULT_ROBOT_IP="192.168.123.161"
DEFAULT_PC_IP="192.168.123.100"
DEFAULT_INTERFACE=""
DEFAULT_ROS_DOMAIN_ID="0"
DEFAULT_ROS_LOCALHOST_ONLY="0"

# Parse arguments
ROBOT_IP=${1:-$DEFAULT_ROBOT_IP}
PC_IP=${2:-$DEFAULT_PC_IP}
INTERFACE=${3:-$DEFAULT_INTERFACE}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-$DEFAULT_ROS_DOMAIN_ID}
ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-$DEFAULT_ROS_LOCALHOST_ONLY}

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Try to find the cyclonedds.xml config file
if [ -f "${SCRIPT_DIR}/../config/cyclonedds.xml" ]; then
    CYCLONEDDS_CONFIG="${SCRIPT_DIR}/../config/cyclonedds.xml"
elif [ -f "${SCRIPT_DIR}/config/cyclonedds.xml" ]; then
    CYCLONEDDS_CONFIG="${SCRIPT_DIR}/config/cyclonedds.xml"
else
    # Try to find it via ROS2 package
    CYCLONEDDS_CONFIG=$(ros2 pkg prefix go2_robot_sdk 2>/dev/null)/share/go2_robot_sdk/config/cyclonedds.xml
fi

# Export environment variables
export ROBOT_IP="${ROBOT_IP}"
export CONN_TYPE="cyclonedds"
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}"

if [ -f "${CYCLONEDDS_CONFIG}" ]; then
    export CYCLONEDDS_URI="file://${CYCLONEDDS_CONFIG}"
    echo -e "${GREEN}✓${NC} CycloneDDS config: ${CYCLONEDDS_CONFIG}"
else
    echo -e "${YELLOW}⚠${NC} CycloneDDS config file not found, using defaults"
fi

echo ""
echo -e "${GREEN}Environment Variables Set:${NC}"
echo -e "  ROBOT_IP           = ${ROBOT_IP}"
echo -e "  CONN_TYPE          = ${CONN_TYPE}"
echo -e "  RMW_IMPLEMENTATION = ${RMW_IMPLEMENTATION}"
echo -e "  ROS_DOMAIN_ID      = ${ROS_DOMAIN_ID}"
echo -e "  ROS_LOCALHOST_ONLY = ${ROS_LOCALHOST_ONLY}"
echo -e "  CYCLONEDDS_URI     = ${CYCLONEDDS_URI:-<not set>}"
echo ""

# Check WSL networking if applicable
if [ "$IS_WSL" = true ]; then
    check_wsl_networking
fi

# Function to check network configuration
check_network() {
    echo -e "${BLUE}Checking Network Configuration...${NC}"
    echo ""
    
    # Check if we have an IP in the 192.168.123.x subnet
    local subnet_ip=$(ip addr show 2>/dev/null | grep -oP '192\.168\.123\.\d+' | head -1)
    
    if [ -n "$subnet_ip" ]; then
        echo -e "${GREEN}✓${NC} Found IP in Go2 subnet: ${subnet_ip}"
        
        # Find the interface
        local iface=$(ip addr show 2>/dev/null | grep -B2 "${subnet_ip}" | grep -oP '^\d+: \K[^:]+' | head -1)
        if [ -n "$iface" ]; then
            echo -e "${GREEN}✓${NC} Interface: ${iface}"
        fi
    else
        echo -e "${RED}✗${NC} No IP found in 192.168.123.x subnet"
        echo ""
        
        if [ "$IS_WSL" = true ]; then
            echo -e "${YELLOW}For WSL, see the networking options above.${NC}"
            echo ""
            echo "If using mirrored networking, configure the IP on Windows:"
            echo "  1. Open Network Connections in Windows"
            echo "  2. Configure your Ethernet adapter with:"
            echo "     IP: ${PC_IP}"
            echo "     Subnet: 255.255.255.0"
            echo ""
        else
            echo -e "${YELLOW}To configure your network interface:${NC}"
            echo ""
            
            # List available interfaces
            echo "  Available interfaces:"
            ip link show | grep -E "^[0-9]+" | awk '{print "    " $2}' | sed 's/://'
            echo ""
            
            if [ -n "$INTERFACE" ]; then
                echo -e "  Run: ${GREEN}sudo ip addr add ${PC_IP}/24 dev ${INTERFACE}${NC}"
            else
                echo -e "  Run: ${GREEN}sudo ip addr add ${PC_IP}/24 dev <interface>${NC}"
                echo "  Replace <interface> with your Ethernet interface name (e.g., eth0, enp0s3)"
            fi
            echo ""
        fi
    fi
    
    # Try to ping the robot
    echo -e "${BLUE}Testing connection to Go2 Pro (${ROBOT_IP})...${NC}"
    if ping -c 1 -W 2 "${ROBOT_IP}" &>/dev/null; then
        echo -e "${GREEN}✓${NC} Robot is reachable!"
    else
        echo -e "${RED}✗${NC} Cannot reach robot at ${ROBOT_IP}"
        echo ""
        echo -e "${YELLOW}Troubleshooting:${NC}"
        echo "  1. Ensure Ethernet cable is connected to Go2 Pro"
        echo "  2. Verify your PC has an IP in 192.168.123.x subnet"
        echo "  3. Check that Go2 Pro is powered on"
        echo "  4. Verify Go2 Pro IP (default: 192.168.123.161)"
        
        if [ "$IS_WSL" = true ]; then
            echo ""
            echo -e "${YELLOW}WSL-specific checks:${NC}"
            echo "  5. Ensure WSL networking is configured (see above)"
            echo "  6. Try pinging from Windows PowerShell first"
            echo "  7. Check Windows Firewall settings"
        fi
    fi
    echo ""
}

# Function to show DDS topics from the robot
show_dds_topics() {
    echo -e "${BLUE}Checking for Go2 DDS Topics...${NC}"
    echo ""
    
    # Set timeout for ros2 topic list
    timeout 5 ros2 topic list 2>/dev/null | grep -E "^/rt/" | head -20 || {
        echo -e "${YELLOW}⚠${NC} No Go2 topics detected yet"
        echo "  This is normal if the robot is not publishing or discovery is still in progress."
        echo "  Wait a few seconds and try: ros2 topic list | grep rt/"
    }
    echo ""
}

# Run checks
check_network

echo -e "${BLUE}=========================================${NC}"
echo -e "${GREEN}Setup Complete!${NC}"
echo -e "${BLUE}=========================================${NC}"
echo ""

if [ "$IS_WSL" = true ]; then
    echo -e "${CYAN}Running in WSL${WSL_VERSION}${NC}"
    echo ""
fi

echo "To launch the Go2 driver with CycloneDDS:"
echo ""
echo -e "  ${GREEN}ros2 launch go2_robot_sdk cyclonedds.launch.py${NC}"
echo ""
echo "Or run the driver node directly:"
echo ""
echo -e "  ${GREEN}ros2 run go2_robot_sdk go2_driver_node${NC}"
echo ""
echo "To check available topics from the robot:"
echo ""
echo -e "  ${GREEN}ros2 topic list | grep rt/${NC}"
echo ""
