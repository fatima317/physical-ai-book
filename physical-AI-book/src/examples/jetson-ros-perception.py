#!/usr/bin/env python3

"""
ROS 2 Humble Example: Jetson Perception Node

This example demonstrates computer vision perception using ROS 2
on Jetson platform, leveraging GPU acceleration where possible.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO  # YOLOv8 for object detection
import torch


class JetsonPerceptionNode(Node):
    def __init__(self):
        super().__init__('jetson_perception')

        # Create QoS profile
        qos_profile = QoSProfile(depth=10)

        # Create publishers
        self.detection_publisher = self.create_publisher(
            String,
            'jetson/detections',
            qos_profile
        )

        self.result_publisher = self.create_publisher(
            Image,
            'jetson/camera/image_result',
            qos_profile
        )

        # Create subscribers
        self.image_subscriber = self.create_subscription(
            Image,
            'jetson/camera/image_raw',
            self.image_callback,
            qos_profile
        )

        # Initialize OpenCV bridge
        self.cv_bridge = CvBridge()

        # Initialize YOLO model (will use GPU if available)
        try:
            # Use a smaller model for Jetson's constraints
            self.model = YOLO('yolov8n.pt')  # Nano model for efficiency

            # Set device to GPU if available
            if torch.cuda.is_available():
                self.model.to('cuda')
                self.get_logger().info('Using GPU for inference')
            else:
                self.get_logger().info('Using CPU for inference')

            self.get_logger().info('YOLO model loaded successfully')
        except Exception as e:
            self.get_logger().warn(f'Failed to load YOLO model: {e}')
            self.model = None

        # Processing parameters
        self.process_every_n_frames = 5  # Process every 5th frame to reduce load
        self.frame_count = 0

        self.get_logger().info('Jetson Perception Node initialized')
        self.get_logger().info('Subscribing to: jetson/camera/image_raw')
        self.get_logger().info('Publishing to: jetson/detections, jetson/camera/image_result')

    def image_callback(self, msg):
        """Process incoming image for perception"""
        self.frame_count += 1

        # Only process every Nth frame to reduce computational load
        if self.frame_count % self.process_every_n_frames != 0:
            return

        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if self.model is not None:
                # Run inference
                results = self.model(cv_image)

                # Process results
                detections = []
                annotated_image = cv_image.copy()

                # Get the first result (assuming single image input)
                result = results[0]

                # Draw bounding boxes and labels
                for box in result.boxes:
                    # Extract bounding box coordinates
                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    # Extract confidence and class
                    conf = box.conf[0]
                    cls = int(box.cls[0])

                    # Only include detections with confidence > 0.5
                    if conf > 0.5:
                        label = f"{self.model.names[cls]} {conf:.2f}"
                        detections.append({
                            'class': self.model.names[cls],
                            'confidence': float(conf),
                            'bbox': [x1, y1, x2, y2]
                        })

                        # Draw bounding box
                        cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                        # Draw label
                        cv2.putText(
                            annotated_image,
                            label,
                            (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            2
                        )

                # Publish detections as JSON string
                if detections:
                    detections_msg = String()
                    detections_msg.data = str(detections)
                    self.detection_publisher.publish(detections_msg)
                    self.get_logger().info(f'Detected: {[d["class"] for d in detections]}')

                # Publish annotated image
                result_msg = self.cv_bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
                result_msg.header = msg.header
                self.result_publisher.publish(result_msg)
            else:
                # If no model, just publish the original image
                self.result_publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def detect_aruco_markers(self, cv_image):
        """Detect ArUco markers in the image"""
        # Define ArUco dictionary
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters_create()

        # Detect markers
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(
            cv_image, aruco_dict, parameters=parameters
        )

        if ids is not None:
            # Draw detected markers
            cv_image = cv2.aruco.drawDetectedMarkers(cv_image, corners)

            # Calculate poses if camera matrix is available
            # For this example, we'll just return the corners
            return [{'id': int(id[0]), 'corners': corner[0].tolist()} for id, corner in zip(ids, corners)]

        return []


def main(args=None):
    rclpy.init(args=args)

    perception_node = JetsonPerceptionNode()

    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        perception_node.get_logger().info('Interrupted, shutting down...')
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()