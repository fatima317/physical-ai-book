"""
Isaac Sim Perception Example

This example demonstrates how to use Isaac Sim for perception tasks,
including camera setup, LiDAR simulation, and synthetic data generation.
The sensors are configured to match real hardware equivalents.
"""

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.sensor import Camera
from omni.isaac.range_sensor import _range_sensor
import numpy as np
import carb
import asyncio


class IsaacSimPerceptionExample:
    def __init__(self):
        self.world = None
        self.camera = None
        self.depth_camera = None
        self.lidar = None
        self.lidar_interface = None

    async def setup_scene(self):
        """Set up the Isaac Sim scene with perception sensors"""
        # Create world
        self.world = World(stage_units_in_meters=1.0)

        # Add a simple scene with objects
        # Add ground plane
        self.world.scene.add_default_ground_plane()

        # Add some objects for perception
        from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
        from omni.isaac.core.prims import XFormPrim

        # Add a few cubes to detect
        self.world.scene.add(
            VisualCuboid(
                prim_path="/World/Cube1",
                name="visual_cube1",
                position=np.array([1.0, 0.0, 0.5]),
                size=0.2,
                color=np.array([1.0, 0.0, 0.0])
            )
        )

        self.world.scene.add(
            VisualCuboid(
                prim_path="/World/Cube2",
                name="visual_cube2",
                position=np.array([1.5, 0.5, 0.3]),
                size=0.15,
                color=np.array([0.0, 1.0, 0.0])
            )
        )

        self.world.scene.add(
            VisualCuboid(
                prim_path="/World/Cube3",
                name="visual_cube3",
                position=np.array([0.8, -0.5, 0.4]),
                size=0.18,
                color=np.array([0.0, 0.0, 1.0])
            )
        )

    def setup_camera_sensors(self):
        """Set up RGB and depth cameras to match real hardware"""
        # Create a robot base or platform to mount sensors
        from omni.isaac.core.robots import Robot

        # Add a simple robot platform
        add_reference_to_stage(
            usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/Robots/TurtleBot3Burger/turtlebot3_burger.usd",
            prim_path="/World/Robot"
        )

        # Set initial position
        from omni.isaac.core.utils.prims import get_prim_at_path
        robot_prim = get_prim_at_path("/World/Robot")
        robot_prim.GetAttribute("xformOp:translate").Set([0.0, 0.0, 0.1])

        # RGB Camera - matching a typical RGB camera on a JetBot
        self.camera = Camera(
            prim_path="/World/Robot/RgbCamera",
            position=np.array([0.1, 0.0, 0.1]),  # Slightly in front and up
            frequency=30,  # 30Hz like typical camera
            resolution=(640, 480)  # HD resolution
        )
        self.camera.set_focal_length(24.0)  # Typical focal length
        self.camera.set_horizontal_aperture(20.955)  # Typical aperture
        self.camera.set_vertical_aperture(15.2908)  # Typical aperture

        # Add to world
        self.world.scene.add(self.camera)

        # Depth Camera - matches real depth sensors like Intel Realsense
        self.depth_camera = Camera(
            prim_path="/World/Robot/DepthCamera",
            position=np.array([0.1, 0.0, 0.1]),  # Same position as RGB
            frequency=30,
            resolution=(640, 480)
        )
        self.depth_camera.set_focal_length(24.0)
        self.depth_camera.set_horizontal_aperture(20.955)
        self.depth_camera.set_vertical_aperture(15.2908)

        # Add to world
        self.world.scene.add(self.depth_camera)

    def setup_lidar_sensor(self):
        """Set up LiDAR sensor to match real hardware (like Hokuyo or similar)"""
        # Get the LiDAR interface
        self.lidar_interface = _range_sensor.acquire_lidar_sensor_interface()

        # Create LiDAR configuration matching real hardware
        lidar_config = {
            "name": "HokuyoLidar",
            "rotation_frequency": 20,  # 20 Hz rotation
            "points_per_second": 500000,  # High resolution
            "laser_as_line": False,
            "enable_log": False,
            # Physical properties matching Hokuyo URG-04LX-UG01
            "sensor_horizontal_fov": 270,  # degrees
            "number_of_layers": 1,  # Single layer like URG-04LX
            "horizontal_samples": 720,  # 720 samples over 270°
            "max_range": 5.6,  # meters
            "min_range": 0.06,  # meters
        }

        self.lidar = self.lidar_interface.create_lidar(
            prim_path="/World/Robot/Lidar",
            sensor_period=(1.0 / lidar_config["rotation_frequency"]),  # seconds per rotation
            samples_per_scan=lidar_config["horizontal_samples"],
            update_us=10000,  # microseconds between updates
            pose=np.array([0.1, 0.0, 0.15]),  # Position on robot
            **lidar_config
        )

    async def run_perception_pipeline(self):
        """Run the perception pipeline to collect sensor data"""
        # Play the simulation
        self.world.play()

        # Collect data for several frames
        for i in range(100):  # Collect 100 frames of data
            self.world.step(render=True)

            if i % 10 == 0:  # Print every 10th frame
                # Get RGB data
                rgb_data = self.camera.get_rgb()
                if rgb_data is not None:
                    print(f"Frame {i}: RGB shape: {rgb_data.shape}")

                # Get depth data
                depth_data = self.depth_camera.get_depth()
                if depth_data is not None:
                    print(f"Frame {i}: Depth range: {np.min(depth_data):.2f} - {np.max(depth_data):.2f}m")

                # Get LiDAR data
                lidar_data = self.lidar_interface.get_linear_depth_data(
                    self.lidar,
                    _range_sensor._BNDS
                )
                if lidar_data.size > 0:
                    valid_points = lidar_data[lidar_data > 0]
                    if valid_points.size > 0:
                        print(f"Frame {i}: LiDAR points: {valid_points.size}, range: {np.min(valid_points):.2f} - {np.max(valid_points):.2f}m")

            # Move the robot slightly to get different perspectives
            if i % 30 == 0:  # Change position every 30 frames
                from omni.isaac.core.utils.prims import get_prim_at_path
                robot_prim = get_prim_at_path("/World/Robot")
                current_pos = robot_prim.GetAttribute("xformOp:translate").Get()
                new_pos = [current_pos[0] + 0.01, current_pos[1], current_pos[2]]
                robot_prim.GetAttribute("xformOp:translate").Set(new_pos)

    async def run_synthetic_data_generation(self):
        """Generate synthetic training data with domain randomization"""
        print("Starting synthetic data generation...")

        # Apply domain randomization to improve real-world transfer
        from omni.isaac.synthetic_utils import SyntheticDataGeneration
        from omni.isaac.core.utils.domain_randomization import DomainRandomization

        # Set up domain randomization
        dr = DomainRandomization()

        # Randomize lighting conditions
        lights = ["/World/Light"]
        for light_path in lights:
            # Randomize light intensity
            intensity_range = (0.5, 2.0)
            random_intensity = np.random.uniform(*intensity_range)
            from omni.isaac.core.utils.prims import get_prim_at_path
            try:
                light_prim = get_prim_at_path(light_path)
                light_prim.GetAttribute("inputs:intensity").Set(random_intensity)
            except:
                print(f"Could not find light at {light_path}, creating a new one")
                from omni.isaac.core.utils.prims import define_prim
                define_prim("/World/Light", "DistantLight")
                light_prim = get_prim_at_path("/World/Light")
                light_prim.GetAttribute("inputs:intensity").Set(random_intensity)

        # Randomize textures
        objects = ["/World/Cube1", "/World/Cube2", "/World/Cube3"]
        for obj_path in objects:
            try:
                obj_prim = get_prim_at_path(obj_path)
                # Randomize color
                random_color = np.random.rand(3)
                obj_prim.GetAttribute("primvars:displayColor").Set([random_color])
            except:
                print(f"Could not find object at {obj_path}")

        print("Domain randomization applied. Collecting synthetic data...")

        # Run for a few more steps to collect randomized data
        for i in range(50):
            self.world.step(render=True)

            if i % 10 == 0:
                # Collect synchronized sensor data
                rgb_data = self.camera.get_rgb()
                depth_data = self.depth_camera.get_depth()

                if rgb_data is not None and depth_data is not None:
                    print(f"Synthetic frame {i}: RGB shape: {rgb_data.shape}, Depth shape: {depth_data.shape}")

                    # Save synthetic data here (in a real implementation)
                    # This would typically save to a dataset format
                    # save_synthetic_data(rgb_data, depth_data, f"synthetic_data_{i}.npz")

    async def run_example(self):
        """Run the complete Isaac Sim perception example"""
        try:
            # Set up the scene
            await self.setup_scene()

            # Set up perception sensors
            self.setup_camera_sensors()
            self.setup_lidar_sensor()

            # Initialize the world
            self.world.reset()

            # Set camera view for better visualization
            set_camera_view(eye=np.array([2, 2, 2]), target=np.array([0, 0, 0]))

            # Run perception pipeline
            await self.run_perception_pipeline()

            # Run synthetic data generation
            await self.run_synthetic_data_generation()

            print("Isaac Sim perception example completed successfully!")

        except Exception as e:
            print(f"Error running Isaac Sim perception example: {e}")
        finally:
            if self.world:
                self.world.stop()

    def cleanup(self):
        """Clean up resources"""
        if self.world:
            self.world.stop()

        if self.lidar_interface and self.lidar:
            self.lidar_interface.remove_sensor(self.lidar)


async def main():
    """Main function to run the Isaac Sim perception example"""
    example = IsaacSimPerceptionExample()

    try:
        await example.run_example()
    finally:
        example.cleanup()


if __name__ == "__main__":
    # Initialize Isaac Sim
    config = {
        "experience": "omni.isaac.sim.python_app",
        "headless": False,  # Set to True for headless operation
    }

    # Run the example
    asyncio.run(main())