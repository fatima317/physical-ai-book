"""
Isaac Sim VSLAM Example

This example demonstrates Visual SLAM in Isaac Sim using synthetic data
from RGB and depth cameras. It shows how to create maps and track pose
using simulated sensor data that matches real hardware.
"""

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.sensor import Camera
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import asyncio
import math


class VSLAMProcessor:
    """VSLAM processor that works with Isaac Sim sensor data"""
    def __init__(self):
        # Feature detector (using ORB which is lightweight and good for real-time)
        self.feature_detector = cv2.ORB_create(nfeatures=1000)

        # Feature matcher
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        # Storage for keyframes
        self.keyframes = []
        self.current_pose = np.eye(4)  # 4x4 identity matrix
        self.frame_count = 0

        # Map points (3D points in the world)
        self.map_points = []
        self.point_descriptors = []

        # Camera parameters (from Isaac Sim camera)
        self.fx = 300.0  # Focal length x
        self.fy = 300.0  # Focal length y
        self.cx = 320.0  # Principal point x
        self.cy = 240.0  # Principal point y

        # Previous frame data
        self.prev_keypoints = None
        self.prev_descriptors = None

        # Trajectory storage
        self.trajectory = []

    def process_frame(self, rgb_image, depth_image, timestamp):
        """Process a single RGB-D frame for VSLAM"""
        # Convert to grayscale for feature detection
        gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)

        # Detect features
        keypoints = self.feature_detector.detect(gray)
        if keypoints:
            keypoints, descriptors = self.feature_detector.compute(gray, keypoints)
        else:
            return self.current_pose, False  # No features detected

        if descriptors is None:
            return self.current_pose, False

        # Store first frame as reference
        if self.prev_descriptors is None:
            self.prev_keypoints = keypoints
            self.prev_descriptors = descriptors
            self.frame_count += 1
            return self.current_pose, True

        # Match features with previous frame
        matches = self.matcher.knnMatch(self.prev_descriptors, descriptors, k=2)

        # Apply Lowe's ratio test for good matches
        good_matches = []
        if len(matches) > 0:
            for m, n in matches:
                if len(matches) > 0 and m.distance < 0.75 * n.distance:
                    good_matches.append(m)

        # Need at least 10 good matches for pose estimation
        if len(good_matches) < 10:
            # Update for next iteration but don't estimate pose
            self.prev_keypoints = keypoints
            self.prev_descriptors = descriptors
            return self.current_pose, False

        # Get matched points
        prev_pts = np.float32([self.prev_keypoints[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        curr_pts = np.float32([keypoints[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

        # Estimate pose using Essential Matrix
        E, mask = cv2.findEssentialMat(
            curr_pts, prev_pts,
            cameraMatrix=np.array([[self.fx, 0, self.cx],
                                 [0, self.fy, self.cy],
                                 [0, 0, 1]]),
            method=cv2.RANSAC,
            prob=0.999,
            threshold=1.0
        )

        if E is not None:
            # Recover pose from Essential Matrix
            _, R, t, mask_pose = cv2.recoverPose(E, curr_pts, prev_pts,
                                               cameraMatrix=np.array([[self.fx, 0, self.cx],
                                                                     [0, self.fy, self.cy],
                                                                     [0, 0, 1]]))

            # Create transformation matrix
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t.flatten()

            # Update current pose
            self.current_pose = self.current_pose @ np.linalg.inv(T)

        # Store current frame for next iteration
        self.prev_keypoints = keypoints
        self.prev_descriptors = descriptors

        # Store trajectory point
        position = self.current_pose[:3, 3]
        self.trajectory.append(position.copy())

        self.frame_count += 1
        return self.current_pose, True


class IsaacSimVSLAMExample:
    def __init__(self):
        self.world = None
        self.rgb_camera = None
        self.depth_camera = None
        self.vslam_processor = VSLAMProcessor()
        self.robot_path = []

    async def setup_scene(self):
        """Set up the Isaac Sim scene with objects for VSLAM"""
        # Create world
        self.world = World(stage_units_in_meters=1.0)

        # Add ground plane
        self.world.scene.add_default_ground_plane()

        # Add a complex scene with many features for VSLAM
        from omni.isaac.core.objects import VisualCuboid, VisualSphere
        from omni.isaac.core.prims import XFormPrim

        # Add various objects to create features
        positions = [
            [2.0, 0.0, 0.5], [2.5, 1.0, 0.7], [2.0, 2.0, 0.5],
            [-1.0, 1.5, 0.6], [-2.0, 0.0, 0.4], [-1.5, -1.5, 0.8],
            [0.5, -2.0, 0.5], [1.5, -1.5, 0.6], [0.0, 1.0, 0.9]
        ]

        colors = [
            [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0],
            [1.0, 1.0, 0.0], [1.0, 0.0, 1.0], [0.0, 1.0, 1.0],
            [1.0, 0.5, 0.0], [0.5, 0.0, 1.0], [0.0, 0.5, 1.0]
        ]

        for i, (pos, color) in enumerate(zip(positions, colors)):
            self.world.scene.add(
                VisualCuboid(
                    prim_path=f"/World/Object{i}",
                    name=f"object_{i}",
                    position=np.array(pos),
                    size=0.3,
                    color=np.array(color)
                )
            )

        # Add some structures
        self.world.scene.add(
            VisualCuboid(
                prim_path="/World/Wall1",
                name="wall_1",
                position=np.array([0.0, 3.0, 1.0]),
                size=np.array([6.0, 0.2, 2.0]),
                color=np.array([0.5, 0.5, 0.5])
            )
        )

        self.world.scene.add(
            VisualCuboid(
                prim_path="/World/Wall2",
                name="wall_2",
                position=np.array([3.0, 0.0, 1.0]),
                size=np.array([0.2, 6.0, 2.0]),
                color=np.array([0.5, 0.5, 0.5])
            )
        )

    def setup_cameras(self):
        """Set up RGB and depth cameras for VSLAM"""
        # Add robot platform
        add_reference_to_stage(
            usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/Robots/Carter/carter_navigation.usd",
            prim_path="/World/Robot"
        )

        # Position robot
        from omni.isaac.core.utils.prims import get_prim_at_path
        robot_prim = get_prim_at_path("/World/Robot")
        robot_prim.GetAttribute("xformOp:translate").Set([0.0, 0.0, 0.2])

        # RGB Camera
        self.rgb_camera = Camera(
            prim_path="/World/Robot/RgbCamera",
            position=np.array([0.2, 0.0, 0.1]),  # Forward and up
            frequency=30,  # 30Hz
            resolution=(640, 480)
        )
        self.rgb_camera.set_focal_length(24.0)
        self.rgb_camera.set_horizontal_aperture(20.955)
        self.rgb_camera.set_vertical_aperture(15.2908)

        # Depth Camera
        self.depth_camera = Camera(
            prim_path="/World/Robot/DepthCamera",
            position=np.array([0.2, 0.0, 0.1]),  # Same position as RGB
            frequency=30,
            resolution=(640, 480)
        )
        self.depth_camera.set_focal_length(24.0)
        self.depth_camera.set_horizontal_aperture(20.955)
        self.depth_camera.set_vertical_aperture(15.2908)

        # Add to world
        self.world.scene.add(self.rgb_camera)
        self.world.scene.add(self.depth_camera)

    async def move_robot_path(self):
        """Move robot along a predefined path for VSLAM testing"""
        from omni.isaac.core.utils.prims import get_prim_at_path

        robot_prim = get_prim_at_path("/World/Robot")

        # Define a path for the robot to follow
        path = [
            [0.0, 0.0, 0.2],
            [1.0, 0.0, 0.2],
            [1.5, 0.5, 0.2],
            [2.0, 1.0, 0.2],
            [1.5, 1.5, 0.2],
            [1.0, 2.0, 0.2],
            [0.0, 2.0, 0.2],
            [-1.0, 1.5, 0.2],
            [-1.5, 1.0, 0.2],
            [-2.0, 0.0, 0.2],
            [-1.0, -1.0, 0.2],
            [0.0, -1.0, 0.2],
            [0.0, 0.0, 0.2]  # Return to start
        ]

        self.robot_path = path

    async def run_vslam_pipeline(self):
        """Run the VSLAM pipeline with Isaac Sim sensors"""
        # Play the simulation
        self.world.play()

        # Move robot along path
        await self.move_robot_path()

        print("Starting VSLAM pipeline...")

        for i in range(len(self.robot_path) * 10):  # 10 steps per path point
            # Move robot along the path
            path_idx = min(i // 10, len(self.robot_path) - 1)
            if path_idx < len(self.robot_path):
                from omni.isaac.core.utils.prims import get_prim_at_path
                robot_prim = get_prim_at_path("/World/Robot")

                # Interpolate between path points for smooth movement
                if path_idx < len(self.robot_path) - 1:
                    start_pos = np.array(self.robot_path[path_idx])
                    end_pos = np.array(self.robot_path[path_idx + 1])
                    progress = (i % 10) / 10.0
                    current_pos = start_pos + (end_pos - start_pos) * progress
                else:
                    current_pos = np.array(self.robot_path[path_idx])

                robot_prim.GetAttribute("xformOp:translate").Set(current_pos)

            # Step the simulation
            self.world.step(render=True)

            # Get sensor data
            rgb_data = self.rgb_camera.get_rgb()
            depth_data = self.depth_camera.get_depth()

            if rgb_data is not None and depth_data is not None:
                # Process frame with VSLAM
                pose, success = self.vslam_processor.process_frame(
                    rgb_data, depth_data, i
                )

                if success and i % 30 == 0:  # Print every 30 frames
                    pos = pose[:3, 3]
                    rot = R.from_matrix(pose[:3, :3]).as_euler('xyz')
                    print(f"Frame {i}: Position=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}), "
                          f"Rotation=({rot[0]:.2f}, {rot[1]:.2f}, {rot[2]:.2f})")

                # Print trajectory length occasionally
                if len(self.vslam_processor.trajectory) > 0 and i % 50 == 0:
                    print(f"Trajectory length: {len(self.vslam_processor.trajectory)} points")

    async def run_example(self):
        """Run the complete Isaac Sim VSLAM example"""
        try:
            # Set up the scene
            await self.setup_scene()

            # Set up cameras
            self.setup_cameras()

            # Initialize the world
            self.world.reset()

            # Set camera view
            set_camera_view(eye=np.array([4, 4, 4]), target=np.array([0, 0, 0]))

            # Run VSLAM pipeline
            await self.run_vslam_pipeline()

            print("Isaac Sim VSLAM example completed successfully!")
            print(f"Final trajectory has {len(self.vslam_processor.trajectory)} points")

        except Exception as e:
            print(f"Error running Isaac Sim VSLAM example: {e}")
        finally:
            if self.world:
                self.world.stop()

    def cleanup(self):
        """Clean up resources"""
        if self.world:
            self.world.stop()


async def main():
    """Main function to run the Isaac Sim VSLAM example"""
    example = IsaacSimVSLAMExample()

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