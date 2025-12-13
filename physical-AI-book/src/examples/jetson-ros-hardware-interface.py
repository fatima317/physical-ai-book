#!/usr/bin/env python3

"""
ROS 2 Humble Example: Jetson Hardware Interface Node

This example demonstrates how to interface with Jetson hardware components
like GPIO, I2C sensors, and camera modules using ROS 2 Humble.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray, Header
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import numpy as np
import cv2

# Mock implementations for Jetson hardware interfaces
# In a real implementation, these would interface with actual Jetson hardware
class JetsonHardwareInterface:
    def __init__(self):
        # GPIO simulation
        self.gpio_pins = {}

        # I2C simulation
        self.i2c_devices = {
            0x68: "MPU6050 IMU",  # Common IMU sensor
            0x77: "BME280",       # Temperature/pressure/humidity sensor
        }

        # Camera interface
        self.camera = cv2.VideoCapture(0)  # Use default camera or CSI camera
        self.cv_bridge = CvBridge()

    def set_gpio_mode(self, pin, mode):
        """Set GPIO pin mode"""
        print(f"Setting GPIO pin {pin} to {mode}")
        self.gpio_pins[pin] = {'mode': mode, 'value': 0}

    def write_gpio(self, pin, value):
        """Write to GPIO pin"""
        if pin in self.gpio_pins:
            self.gpio_pins[pin]['value'] = value
            print(f"Writing {value} to GPIO pin {pin}")
        else:
            print(f"GPIO pin {pin} not initialized")

    def read_gpio(self, pin):
        """Read from GPIO pin"""
        if pin in self.gpio_pins:
            # Simulate reading with some randomness
            return np.random.choice([0, 1])
        else:
            print(f"GPIO pin {pin} not initialized")
            return 0

    def read_i2c_byte(self, address, register):
        """Read byte from I2C device"""
        print(f"Reading from I2C device at 0x{address:02x}, register 0x{register:02x}")
        # Simulate sensor reading with some randomness
        return np.random.randint(0, 255)

    def write_i2c_byte(self, address, register, value):
        """Write byte to I2C device"""
        print(f"Writing 0x{value:02x} to I2C device at 0x{address:02x}, register 0x{register:02x}")

    def capture_camera_frame(self):
        """Capture frame from camera"""
        ret, frame = self.camera.read()
        if ret:
            # Resize frame to reduce processing load on Jetson
            frame = cv2.resize(frame, (640, 480))
            return frame
        else:
            # Return a blank frame if capture fails
            return np.zeros((480, 640, 3), dtype=np.uint8)

    def cleanup(self):
        """Clean up hardware resources"""
        self.camera.release()


class JetsonHardwareInterfaceNode(Node):
    def __init__(self):
        super().__init__('jetson_hardware_interface')

        # Initialize Jetson hardware interface
        self.hw_interface = JetsonHardwareInterface()

        # Create QoS profile
        qos_profile = QoSProfile(depth=10)

        # Create publishers
        self.sensor_publisher = self.create_publisher(
            Float32MultiArray,
            'jetson/sensor_data',
            qos_profile
        )

        self.image_publisher = self.create_publisher(
            Image,
            'jetson/camera/image_raw',
            qos_profile
        )

        # Create service server for GPIO control
        self.gpio_service = self.create_service(
            SetBool,
            'jetson/gpio_control',
            self.gpio_control_callback
        )

        # Create timer for periodic sensor publishing
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

        # Initialize GPIO pin 7 for LED control
        self.hw_interface.set_gpio_mode(7, 'OUTPUT')

        self.get_logger().info('Jetson Hardware Interface Node initialized')
        self.get_logger().info('Publishing to: jetson/sensor_data, jetson/camera/image_raw')
        self.get_logger().info('Service available: jetson/gpio_control')

    def gpio_control_callback(self, request, response):
        """Handle GPIO control service requests"""
        self.get_logger().info(f'GPIO control request: {request.data}')
        self.hw_interface.write_gpio(7, 1 if request.data else 0)  # Use GPIO pin 7
        response.success = True
        response.message = f'GPIO set to {"HIGH" if request.data else "LOW"}'
        return response

    def timer_callback(self):
        """Timer callback to publish sensor data"""
        # Simulate reading from various sensors connected via I2C
        imu_data = [
            float(self.hw_interface.read_i2c_byte(0x68, 0x3B)),  # Accel X
            float(self.hw_interface.read_i2c_byte(0x68, 0x3D)),  # Accel Y
            float(self.hw_interface.read_i2c_byte(0x68, 0x3F)),  # Accel Z
            float(self.hw_interface.read_i2c_byte(0x68, 0x43)),  # Gyro X
            float(self.hw_interface.read_i2c_byte(0x68, 0x45)),  # Gyro Y
            float(self.hw_interface.read_i2c_byte(0x68, 0x47))   # Gyro Z
        ]

        # Create and publish sensor message
        sensor_msg = Float32MultiArray()
        sensor_msg.data = imu_data
        self.sensor_publisher.publish(sensor_msg)
        self.get_logger().info(f'Published sensor data: {imu_data}')

        # Capture and publish camera frame
        frame = self.hw_interface.capture_camera_frame()
        image_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = "camera"
        self.image_publisher.publish(image_msg)
        self.get_logger().info(f'Published camera frame: {frame.shape[1]}x{frame.shape[0]}')

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)

    jetson_node = JetsonHardwareInterfaceNode()

    try:
        rclpy.spin(jetson_node)
    except KeyboardInterrupt:
        jetson_node.get_logger().info('Interrupted, shutting down...')
    finally:
        jetson_node.hw_interface.cleanup()
        jetson_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()