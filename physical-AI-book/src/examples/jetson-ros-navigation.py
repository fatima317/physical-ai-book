#!/usr/bin/env python3

"""
ROS 2 Humble Example: Jetson Navigation Node

This example demonstrates basic navigation using ROS 2 Navigation Stack
with Jetson hardware components. This would typically run on a robot
with differential drive and sensors.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan, Imu
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
import numpy as np
import math


class JetsonNavigationNode(Node):
    def __init__(self):
        super().__init__('jetson_navigation')

        # Create QoS profile
        qos_profile = QoSProfile(depth=10)

        # Create publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            qos_profile
        )

        # Create subscribers
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile
        )

        self.imu_subscriber = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            qos_profile
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos_profile
        )

        # Create timer for navigation control
        self.timer = self.create_timer(0.1, self.navigation_callback)  # 10Hz

        # Navigation state
        self.laser_data = None
        self.imu_data = None
        self.odom_data = None
        self.target_x = 1.0  # Target position
        self.target_y = 1.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Robot parameters (for Jetbot or similar differential drive)
        self.wheel_radius = 0.033  # meters
        self.wheel_separation = 0.16  # meters
        self.max_linear_speed = 0.5  # m/s
        self.max_angular_speed = 1.0  # rad/s

        self.get_logger().info('Jetson Navigation Node initialized')
        self.get_logger().info(f'Target: ({self.target_x}, {self.target_y})')

    def scan_callback(self, msg):
        """Handle laser scan data"""
        self.laser_data = msg.ranges

    def imu_callback(self, msg):
        """Handle IMU data"""
        # Convert quaternion to yaw angle
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        """Handle odometry data"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def navigation_callback(self):
        """Main navigation control loop"""
        if self.odom_data is None:
            # Publish zero velocity until we have odometry
            cmd_vel = Twist()
            cmd_vel.linear = Vector3(x=0.0, y=0.0, z=0.0)
            cmd_vel.angular = Vector3(x=0.0, y=0.0, z=0.0)
            self.cmd_vel_publisher.publish(cmd_vel)
            return

        # Calculate distance to target
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)

        # Calculate angle difference
        angle_diff = target_angle - self.current_yaw
        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        cmd_vel = Twist()

        # Simple proportional controller
        linear_kp = 0.5
        angular_kp = 1.0

        # Only move forward if not too close to obstacles
        min_obstacle_distance = 0.5  # meters
        safe_to_move = True

        if self.laser_data:
            # Check if there are obstacles in the forward direction
            front_scan = self.laser_data[len(self.laser_data)//2-30:len(self.laser_data)//2+30]
            if min(front_scan) < min_obstacle_distance:
                safe_to_move = False
                self.get_logger().info('Obstacle detected, stopping...')

        if safe_to_move and distance > 0.1:  # If we're not close to target
            # Adjust linear speed based on distance
            cmd_vel.linear.x = min(linear_kp * distance, self.max_linear_speed)
            # Adjust angular speed based on angle error
            cmd_vel.angular.z = angular_kp * angle_diff
        else:
            # Stop when close to target or obstacle detected
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        # Publish velocity command
        self.cmd_vel_publisher.publish(cmd_vel)

        self.get_logger().info(
            f'Position: ({self.current_x:.2f}, {self.current_y:.2f}), '
            f'Distance to target: {distance:.2f}, '
            f'Command: ({cmd_vel.linear.x:.2f}, {cmd_vel.angular.z:.2f})'
        )


def main(args=None):
    rclpy.init(args=args)

    navigation_node = JetsonNavigationNode()

    try:
        rclpy.spin(navigation_node)
    except KeyboardInterrupt:
        navigation_node.get_logger().info('Interrupted, shutting down...')
    finally:
        navigation_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()