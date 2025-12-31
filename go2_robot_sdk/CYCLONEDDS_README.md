# CycloneDDS Setup Guide for Go2 Pro Robot

This guide explains how to connect to your Unitree Go2 Pro robot using CycloneDDS over Ethernet.

## Prerequisites

1. **Hardware Setup**
   - Unitree Go2 Pro robot
   - Ethernet cable connected between your PC and the Go2 Pro
   - PC running Ubuntu 22.04 (or WSL2 with Ubuntu 22.04)

2. **Software Requirements**
   - ROS2 Humble installed
   - CycloneDDS RMW implementation:
     ```bash
     sudo apt install ros-humble-rmw-cyclonedds-cpp
     ```

## Network Configuration

The Go2 Pro robot uses the `192.168.123.x` subnet for Ethernet communication.

- **Default Go2 Pro IP**: `192.168.123.161`
- **Your PC should have an IP in the same subnet**, e.g., `192.168.123.100`

### Configure Your PC's Network Interface

1. **Find your Ethernet interface name**:
   ```bash
   ip link show
   ```
   Look for an interface like `eth0`, `enp0s3`, `eno1`, etc.

2. **Add an IP address to your interface**:
   ```bash
   sudo ip addr add 192.168.123.100/24 dev <interface_name>
   ```
   
   For example:
   ```bash
   sudo ip addr add 192.168.123.100/24 dev eth0
   ```

3. **Verify the configuration**:
   ```bash
   ip addr show <interface_name>
   ```

4. **Test connectivity to the robot**:
   ```bash
   ping 192.168.123.161
   ```

### Persistent Network Configuration (Optional)

For a permanent configuration, you can use netplan (Ubuntu) or NetworkManager.

**Using Netplan** (create/edit `/etc/netplan/99-go2-ethernet.yaml`):
```yaml
network:
  version: 2
  ethernets:
    eth0:  # Replace with your interface name
      addresses:
        - 192.168.123.100/24
      routes:
        - to: 192.168.123.0/24
          via: 192.168.123.100
```

Apply with: `sudo netplan apply`

## Building the Workspace

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select go2_interfaces go2_robot_sdk
source install/setup.bash
```

## Running with CycloneDDS

### Method 1: Using the Setup Script (Recommended)

```bash
# Source the setup script
source $(ros2 pkg prefix go2_robot_sdk)/share/go2_robot_sdk/scripts/setup_cyclonedds.sh

# Or if running from source
cd ~/ros2_ws/src/go2_robot_sdk
source scripts/setup_cyclonedds.sh

# Then launch the driver
ros2 launch go2_robot_sdk cyclonedds.launch.py
```

### Method 2: Manual Environment Setup

```bash
# Set environment variables
export ROBOT_IP="192.168.123.161"
export CONN_TYPE="cyclonedds"
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
export CYCLONEDDS_URI="file://$(ros2 pkg prefix go2_robot_sdk)/share/go2_robot_sdk/config/cyclonedds.xml"

# Launch the driver
ros2 launch go2_robot_sdk driver.launch.py
```

### Method 3: Using the Dedicated CycloneDDS Launch File

```bash
export ROBOT_IP="192.168.123.161"
ros2 launch go2_robot_sdk cyclonedds.launch.py
```

## Verifying the Connection

Once launched, check if topics from the robot are visible:

```bash
# List all topics
ros2 topic list

# Look for Go2 specific topics
ros2 topic list | grep rt/

# Expected topics from Go2 Pro:
# /rt/lf/lowstate
# /rt/sportmodestate
# /rt/utlidar/cloud
# /rt/utlidar/robot_pose
# /rt/utlidar/robot_odom
```

## Troubleshooting

### No Topics Visible

1. **Check network connectivity**:
   ```bash
   ping 192.168.123.161
   ```

2. **Verify CycloneDDS is being used**:
   ```bash
   echo $RMW_IMPLEMENTATION
   # Should output: rmw_cyclonedds_cpp
   ```

3. **Check CycloneDDS config is loaded**:
   ```bash
   echo $CYCLONEDDS_URI
   # Should show path to cyclonedds.xml
   ```

4. **Enable CycloneDDS debug logging**:
   Edit `cyclonedds.xml` and change:
   ```xml
   <Verbosity>fine</Verbosity>
   ```
   This will show detailed discovery and connection logs.

### Multiple Network Interfaces Issue

If your PC has multiple network interfaces, CycloneDDS might use the wrong one.

Edit `config/cyclonedds.xml` and specify your interface explicitly:

```xml
<Interfaces>
    <!-- Option 1: By interface name -->
    <NetworkInterface name="eth0" priority="default" />
    
    <!-- Option 2: By IP address -->
    <NetworkInterface address="192.168.123.100" priority="default" />
</Interfaces>
```

### QoS Incompatibility

The Go2 Pro uses Best Effort QoS for high-frequency data. If you're having issues receiving data, ensure your subscribers also use Best Effort QoS.

### Firewall Issues

Ensure your firewall allows UDP traffic on the DDS ports:

```bash
# Temporarily disable firewall for testing
sudo ufw disable

# Or add rules for DDS
sudo ufw allow 7400:7500/udp
```

## WSL2 Specific Notes

If running in WSL2, you need to ensure the WSL2 network can reach the robot:

1. **Use mirrored networking mode** (Windows 11 22H2+):
   Add to `%USERPROFILE%\.wslconfig`:
   ```ini
   [wsl2]
   networkingMode=mirrored
   ```

2. **Or use port forwarding** from Windows to WSL2.

3. **Check WSL2 IP**:
   ```bash
   ip addr show eth0
   ```

## Topics Published by Go2 Pro over Ethernet

| Topic | Type | Description |
|-------|------|-------------|
| `/rt/lf/lowstate` | `go2_interfaces/LowState` | Motor states, IMU, battery |
| `/rt/sportmodestate` | `go2_interfaces/SportModeState` | Robot mode, gait, position |
| `/rt/utlidar/cloud` | `sensor_msgs/PointCloud2` | LiDAR point cloud |
| `/rt/utlidar/robot_pose` | `geometry_msgs/PoseStamped` | Robot pose from SLAM |
| `/rt/utlidar/robot_odom` | `nav_msgs/Odometry` | Odometry data |

## Sending Commands to the Robot

```bash
# Move the robot using cmd_vel
ros2 topic pub /cmd_vel_out geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# Stop the robot
ros2 topic pub /cmd_vel_out geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

## Advanced: Customizing the CycloneDDS Configuration

The CycloneDDS configuration file is located at:
```
config/cyclonedds.xml
```

Key settings you may want to adjust:

- **Network Interface**: Specify which interface to use
- **Peer Discovery**: Add additional peer addresses
- **Buffer Sizes**: Increase for large point cloud data
- **Logging**: Enable for debugging
