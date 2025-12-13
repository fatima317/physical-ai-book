# ROS 2 Humble Examples for Jetson Platform

This directory contains ROS 2 Humble code examples specifically designed for the NVIDIA Jetson platform. These examples demonstrate how to interface with Jetson hardware, perform navigation, and utilize perception capabilities.

## Examples Included

### 1. Jetson Hardware Interface (`jetson-ros-hardware-interface.py`)
Demonstrates how to interface with Jetson hardware components like GPIO, I2C sensors, and camera modules.

**Features:**
- GPIO control via ROS service
- I2C sensor reading (IMU, etc.)
- Camera frame publishing
- Sensor data publishing

**Usage:**
```bash
# Terminal 1
ros2 run your_package jetson-ros-hardware-interface.py

# Terminal 2 - Control GPIO
ros2 service call /jetson/gpio_control std_srvs/srv/SetBool "{data: true}"
```

### 2. Jetson Navigation (`jetson-ros-navigation.py`)
Basic navigation example using ROS 2 Navigation Stack with simulated differential drive robot.

**Features:**
- Target-based navigation
- Obstacle avoidance using laser scan
- IMU integration
- Odometry-based control

**Usage:**
```bash
ros2 run your_package jetson-ros-navigation.py
```

### 3. Jetson Perception (`jetson-ros-perception.py`)
Computer vision perception example leveraging Jetson's GPU capabilities.

**Features:**
- Object detection using YOLOv8
- Real-time inference on GPU
- ArUco marker detection
- Bounding box annotation

**Usage:**
```bash
ros2 run your_package jetson-ros-perception.py
```

## Jetson-Specific Considerations

### Hardware Setup
- Ensure proper power supply for Jetson NX (recommended 19V/4A)
- Connect sensors to appropriate GPIO/I2C interfaces
- Use CSI camera when possible for lower latency

### Performance Optimization
- Use lightweight models (YOLOv8n) for edge inference
- Process every Nth frame to reduce computational load
- Optimize image resolution for real-time processing

### Dependencies
```bash
pip3 install rclpy cv2 numpy ultralytics torch
```

## ROS 2 Topics and Services

### Topics Published
- `/jetson/sensor_data` - Sensor readings (Float32MultiArray)
- `/jetson/camera/image_raw` - Raw camera frames (Image)
- `/jetson/camera/image_result` - Processed camera frames (Image)
- `/jetson/detections` - Object detection results (String)
- `/cmd_vel` - Velocity commands for navigation (Twist)

### Services
- `/jetson/gpio_control` - GPIO control (SetBool)

## Running on Jetson

1. Set up ROS 2 Humble on Jetson NX:
```bash
sudo apt update
sudo apt install ros-humble-desktop
```

2. Source ROS environment:
```bash
source /opt/ros/humble/setup.bash
```

3. Install Python dependencies:
```bash
pip3 install rclpy cv-bridge ultralytics torch
```

4. Run the examples as shown above.

## Validation

All examples are validated for Jetson NX compatibility using the Jetson validation service. The examples consider:
- Memory usage constraints
- ARM64 architecture compatibility
- GPU acceleration where appropriate
- Power consumption optimization