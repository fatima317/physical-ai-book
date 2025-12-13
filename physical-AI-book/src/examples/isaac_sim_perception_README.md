# Isaac Sim Perception Examples

This directory contains Isaac Sim perception examples that demonstrate how to use synthetic data generation and sensor simulation for robotics perception tasks. These examples are designed to match real hardware sensors and can be used to generate training data for real-world deployment.

## Examples Included

### 1. Basic Perception Example (`isaac_sim_perception_example.py`)
Demonstrates how to set up and use perception sensors in Isaac Sim:
- RGB camera with realistic parameters
- Depth camera matching real sensors
- LiDAR simulation (Hokuyo equivalent)
- Synthetic data generation with domain randomization

**Features:**
- Camera setup with realistic intrinsics
- LiDAR configuration matching hardware specs
- Synchronized sensor data collection
- Domain randomization for robust training

### 2. VSLAM Example (`isaac_sim_vslam_example.py`)
Demonstrates Visual SLAM using Isaac Sim sensor data:
- RGB-D processing for pose estimation
- Feature detection and matching
- Trajectory tracking
- Map building capabilities

**Features:**
- ORB feature detection
- Essential matrix-based pose estimation
- Trajectory visualization
- Path following for data collection

## Sensor Configuration

### RGB Camera
- Resolution: 640x480
- Frame rate: 30 Hz
- Focal length: 24.0 mm
- Aperture: 20.955 mm (horizontal), 15.2908 mm (vertical)
- Matches: Typical RGB cameras on JetBot or similar platforms

### Depth Camera
- Resolution: 640x480
- Frame rate: 30 Hz
- Field of view: Matched to RGB camera
- Matches: Intel Realsense or similar depth sensors

### LiDAR
- Field of view: 270° horizontal
- Samples: 720 per rotation
- Range: 0.06m to 5.6m
- Rate: 20 Hz rotation
- Matches: Hokuyo URG-04LX-UG01 or similar

## Usage

### Prerequisites
- Isaac Sim installed (version 2023.1 or later)
- NVIDIA GPU with RTX capabilities
- CUDA 11.8+ installed
- Python 3.8-3.10 environment
- Isaac ROS packages

### Running the Examples

1. Set up Isaac Sim environment:
```bash
# If using Omniverse Launcher
export ISAACSIM_PATH=/path/to/isaac-sim
source $ISAACSIM_PATH/setup_conda_env.sh
```

2. Run the basic perception example:
```bash
python3 isaac_sim_perception_example.py
```

3. Run the VSLAM example:
```bash
python3 isaac_sim_vslam_example.py
```

## Real Hardware Equivalents

The simulated sensors are configured to match real hardware:

### Cameras
- **Simulated**: RGB and depth cameras with 640x480 resolution at 30Hz
- **Real Equivalent**: Raspberry Pi Camera, Intel Realsense, or similar
- **Parameters**: Matched focal length, aperture, and field of view

### LiDAR
- **Simulated**: 270° field of view with 720 samples at 20Hz
- **Real Equivalent**: Hokuyo URG-04LX-UG01, SICK TIM, or similar
- **Parameters**: Range (0.06m-5.6m), resolution, and update rate matched

## Domain Randomization

The examples include domain randomization to improve real-world transfer:
- Lighting condition variations
- Texture randomization
- Background changes
- Color variations

This helps models trained on synthetic data perform better on real data.

## Synthetic Data Generation

The examples demonstrate how to generate synthetic training data:
- Synchronized RGB-D data
- LiDAR point clouds
- Ground truth annotations
- Multiple sensor modalities

## Integration with ROS 2

The examples can be extended to work with ROS 2:
- Publish sensor data to ROS topics
- Subscribe to control commands
- Integrate with Navigation Stack
- Use with perception pipelines

## Performance Considerations

- Use appropriate scene complexity for training
- Balance visual quality with simulation speed
- Consider using lower resolution during development
- Optimize rendering settings for faster simulation

## Troubleshooting

### Common Issues
1. **Performance**: Reduce scene complexity or rendering quality
2. **Memory**: Close unnecessary applications, increase swap space
3. **Sensors not working**: Check sensor placement and permissions
4. **Physics instability**: Adjust physics parameters and step size

### Rendering Settings for Performance
```python
import carb.settings
carb.settings.get_settings().set("/rtx/rendermode", "RaytracedLightmapped")
carb.settings.get_settings().set("/rtx/ambientOcclusion/enabled", False)
carb.settings.get_settings().set("/rtx/dlss/enable", True)  # If supported
```

## Validation

The perception examples have been validated to:
- Match real hardware sensor specifications
- Generate realistic synthetic data
- Support domain randomization techniques
- Work with standard computer vision algorithms

For more information on Isaac Sim, refer to the official documentation: https://docs.omniverse.nvidia.com/isaacsim/latest/