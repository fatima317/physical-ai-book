# Gazebo Simulation Examples for JetBot

This directory contains Gazebo simulation examples that match the hardware behavior of a JetBot robot running on the NVIDIA Jetson platform. These examples demonstrate how to create realistic simulations that can be used for testing and development before deploying on real hardware.

## Files Included

### 1. Robot Model (`jetbot.urdf.xacro`)
A URDF/Xacro model of a JetBot robot with:
- Differential drive base with realistic wheel parameters
- RGB camera sensor
- IMU sensor
- Hokuyo laser range finder
- Proper inertial properties
- Gazebo plugins for all sensors

### 2. Simulation World (`jetbot_world.world`)
A Gazebo world file that includes:
- Ground plane
- Lighting (sun)
- Physics configuration
- Obstacles and targets for navigation testing
- Walls and objects to simulate real-world environments

### 3. Launch File (`launch_jetbot_gazebo.launch.py`)
A ROS 2 launch file that:
- Starts Gazebo server with the custom world
- Launches Gazebo client (GUI)
- Spawns the JetBot robot in the simulation
- Sets up robot state publishing
- Configures joint state publishing

### 4. Controller Configuration (`jetbot_controllers.yaml`)
Configuration file for ROS 2 controllers:
- Joint state broadcaster
- Differential drive controller
- Proper wheel separation and radius matching hardware
- Velocity and acceleration limits matching real hardware

## Usage

### Prerequisites
- ROS 2 Humble installed
- Gazebo Garden (or compatible version)
- robot_state_publisher package
- joint_state_publisher package
- gazebo_ros packages

### Running the Simulation

1. Make the launch file executable:
```bash
chmod +x launch_jetbot_gazebo.launch.py
```

2. Source ROS 2 environment:
```bash
source /opt/ros/humble/setup.bash
```

3. Launch the simulation:
```bash
python3 launch_jetbot_gazebo.launch.py
```

4. In another terminal, send velocity commands to test the robot:
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}'

# Rotate
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.5}}'
```

## Hardware Matching Features

### Physical Dimensions
- Wheel radius: 0.033m (matches real JetBot wheels)
- Wheel separation: 0.16m (matches real JetBot)
- Base dimensions: 0.15m x 0.15m x 0.1m (approximate JetBot size)

### Sensors
- RGB Camera: 640x480 resolution at 30Hz (matches typical JetBot camera)
- IMU: 100Hz update rate
- Laser Range Finder: 270° field of view with 720 samples

### Physics Properties
- Realistic mass and inertia values
- Appropriate friction coefficients
- Proper collision detection

## Validation

The simulation has been validated to match hardware behavior:
- Wheel dynamics match real robot acceleration limits
- Sensor data rates match hardware capabilities
- Control response times are realistic
- Physical interactions are properly simulated

## Integration with ROS 2

The simulation publishes and subscribes to standard ROS 2 topics:
- `/cmd_vel` - Velocity commands (geometry_msgs/Twist)
- `/odom` - Odometry data (nav_msgs/Odometry)
- `/scan` - Laser scan data (sensor_msgs/LaserScan)
- `/jetson/camera/image_raw` - Camera images (sensor_msgs/Image)
- `/imu/data` - IMU data (sensor_msgs/Imu)

## Troubleshooting

### Common Issues
1. **Robot not spawning**: Check that the URDF file is valid and all dependencies are installed
2. **Gazebo not starting**: Ensure proper display settings and GPU drivers
3. **Sensors not publishing**: Verify that the Gazebo plugins are properly loaded

### Performance Tips
- Reduce physics update rate if experiencing performance issues
- Use simpler collision meshes during development
- Limit the number of active sensors during testing

## Extending the Simulation

The simulation can be extended to include:
- Additional sensors (depth cameras, GPS, etc.)
- More complex environments
- Dynamic objects and obstacles
- Custom Gazebo plugins for specific hardware

For more information on Gazebo simulation, refer to the official documentation: http://gazebosim.org/tutorials