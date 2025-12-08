#!/usr/bin/env python3
"""
Sensor Data Publisher for Module 2: Digital Twin Simulation
Reference: Chapter 2.3 Sensor Simulation

This node demonstrates handling sensor data from Gazebo simulation:
- Subscribes to IMU, camera, and lidar topics
- Processes sensor data
- Republishes filtered/processed data

Requirements:
- rclpy >= 3.3.0
- sensor_msgs
- OpenCV (cv_bridge) for camera processing

Usage:
    ros2 run my_package sensor_publisher

Expected Topics:
    Subscribe: /robot/imu, /robot/camera/image_raw, /robot/scan
    Publish: /robot/imu_filtered, /robot/camera_processed
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image, LaserScan
from std_msgs.msg import Float64
import math


class SensorPublisher(Node):
    """
    Processes sensor data from Gazebo simulation.

    Demonstrates:
    - Subscribing to multiple sensor topics
    - Basic sensor data filtering (IMU low-pass filter)
    - Publishing processed data
    """

    def __init__(self):
        super().__init__('sensor_publisher')

        # IMU Subscriber and Publisher
        self.imu_sub = self.create_subscription(
            Imu,
            '/robot/imu',
            self.imu_callback,
            10
        )
        self.imu_pub = self.create_publisher(
            Float64,
            '/robot/imu_filtered',
            10
        )

        # Camera Subscriber
        self.camera_sub = self.create_subscription(
            Image,
            '/robot/camera/image_raw',
            self.camera_callback,
            10
        )

        # Lidar Subscriber
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/robot/scan',
            self.lidar_callback,
            10
        )

        # State variables
        self.imu_pitch_filtered = 0.0
        self.alpha = 0.1  # Low-pass filter coefficient
        self.message_count = 0

        # Statistics timer (1 Hz)
        self.timer = self.create_timer(1.0, self.print_statistics)

        self.get_logger().info('Sensor Publisher Node started')
        self.get_logger().info('Subscribing to: /robot/imu, /robot/camera/image_raw, /robot/scan')

    def imu_callback(self, msg):
        """
        Process IMU data with low-pass filter.

        Extracts pitch angle from orientation quaternion and applies
        exponential moving average filter to reduce noise.

        Args:
            msg (Imu): IMU message with orientation, angular velocity, linear acceleration
        """
        # Extract pitch from quaternion
        # Simplified: assumes robot is roughly level (small roll/yaw)
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # Convert quaternion to pitch (rotation about y-axis)
        pitch = math.atan2(
            2.0 * (qw * qy - qz * qx),
            1.0 - 2.0 * (qx * qx + qy * qy)
        )

        # Apply low-pass filter (exponential moving average)
        self.imu_pitch_filtered = (
            self.alpha * pitch + (1.0 - self.alpha) * self.imu_pitch_filtered
        )

        # Publish filtered pitch
        filtered_msg = Float64()
        filtered_msg.data = self.imu_pitch_filtered
        self.imu_pub.publish(filtered_msg)

        self.message_count += 1

    def camera_callback(self, msg):
        """
        Process camera images.

        In real implementation, you would:
        - Convert ROS Image to OpenCV format using cv_bridge
        - Apply image processing (edge detection, filtering, etc.)
        - Run object detection/tracking

        Args:
            msg (Image): Camera image (640x480 RGB8)
        """
        # Log image metadata (size, encoding)
        if self.message_count % 30 == 0:  # Log every 30 frames (~1 sec at 30 FPS)
            self.get_logger().debug(
                f'Camera frame: {msg.width}x{msg.height}, encoding: {msg.encoding}'
            )

        # TODO: Add OpenCV processing here
        # Example:
        # cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # processed = cv2.Canny(cv_image, 50, 150)

    def lidar_callback(self, msg):
        """
        Process lidar scan data for obstacle detection.

        Finds minimum distance in scan and logs warning if obstacle < 0.5m.

        Args:
            msg (LaserScan): 360-degree lidar scan with range measurements
        """
        # Find minimum range in scan
        min_range = min(msg.ranges) if msg.ranges else float('inf')

        # Check for close obstacles
        if min_range < 0.5:
            self.get_logger().warn(
                f'Obstacle detected at {min_range:.2f}m! '
                f'Collision risk within 0.5m safety zone.'
            )

        # Count valid range measurements
        valid_ranges = sum(
            1 for r in msg.ranges
            if msg.range_min < r < msg.range_max
        )

        if self.message_count % 10 == 0:  # Log every 10 scans (~1 sec at 10 Hz)
            self.get_logger().debug(
                f'Lidar: {valid_ranges}/{len(msg.ranges)} valid points, '
                f'min_range={min_range:.2f}m'
            )

    def print_statistics(self):
        """
        Print sensor processing statistics every second.
        """
        self.get_logger().info(
            f'Processed {self.message_count} sensor messages | '
            f'IMU pitch (filtered): {self.imu_pitch_filtered:.3f} rad '
            f'({math.degrees(self.imu_pitch_filtered):.1f}°)'
        )


def main(args=None):
    rclpy.init(args=args)

    node = SensorPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
