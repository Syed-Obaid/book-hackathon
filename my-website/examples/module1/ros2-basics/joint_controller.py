#!/usr/bin/env python3
"""
Joint Controller Subscriber Node
Reference: Chapter 1.4 Nodes and Topics

Subscribes to joint state messages and performs safety monitoring.
This node demonstrates:
- Creating a ROS 2 subscriber
- Processing sensor_msgs/JointState messages
- Implementing safety checks (joint limit validation)
- Logging and error handling

Requirements:
- rclpy >= 3.3.0
- sensor_msgs

Usage:
    python3 joint_controller.py

Topics Subscribed:
    /joint_states (sensor_msgs/JointState): Joint positions, velocities, efforts
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointController(Node):
    """
    Monitors joint states and validates against safety limits.

    This node subscribes to /joint_states and checks that joint positions
    remain within acceptable ranges. It demonstrates basic control loop
    structure for robot safety monitoring.
    """

    def __init__(self):
        super().__init__('joint_controller')

        # Joint limits from simple_arm.urdf (in radians)
        self.joint_limits = {
            'shoulder_joint': {'lower': -1.5708, 'upper': 1.5708},  # ±90°
            'elbow_joint': {'lower': 0.0, 'upper': 2.618},  # 0° to 150°
            'wrist_joint': {'lower': None, 'upper': None}  # Continuous (no limits)
        }

        # Create subscription: topic name, message type, callback, queue size
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )

        # State tracking
        self.message_count = 0
        self.warning_count = 0

        self.get_logger().info('Joint Controller started')
        self.get_logger().info('Subscribing to /joint_states')
        self.get_logger().info(f'Monitoring {len(self.joint_limits)} joints for safety')

    def listener_callback(self, msg):
        """
        Callback function executed when JointState message arrives.

        Args:
            msg (sensor_msgs.msg.JointState): Joint state message containing
                positions, velocities, and efforts for all joints.
        """
        self.message_count += 1

        # Validate message has required fields
        if not msg.name or not msg.position:
            self.get_logger().warn('Received incomplete JointState message')
            return

        # Check that number of positions matches number of joints
        if len(msg.name) != len(msg.position):
            self.get_logger().error(
                f'Mismatch: {len(msg.name)} joint names but '
                f'{len(msg.position)} positions'
            )
            return

        # Process each joint
        for i, joint_name in enumerate(msg.name):
            position = msg.position[i]

            # Check if joint is known
            if joint_name not in self.joint_limits:
                if self.message_count == 1:  # Only warn once
                    self.get_logger().warn(f'Unknown joint: {joint_name}')
                continue

            # Validate joint limits
            limits = self.joint_limits[joint_name]

            # Skip continuous joints (no limits)
            if limits['lower'] is None or limits['upper'] is None:
                continue

            # Check lower limit
            if position < limits['lower']:
                self.warning_count += 1
                self.get_logger().warn(
                    f'{joint_name} below lower limit: '
                    f'{position:.3f} < {limits["lower"]:.3f} rad'
                )

            # Check upper limit
            if position > limits['upper']:
                self.warning_count += 1
                self.get_logger().warn(
                    f'{joint_name} exceeds upper limit: '
                    f'{position:.3f} > {limits["upper"]:.3f} rad'
                )

        # Log summary every 100 messages (2 seconds at 50 Hz)
        if self.message_count % 100 == 0:
            self.get_logger().info(
                f'Processed {self.message_count} messages, '
                f'{self.warning_count} warnings'
            )

            # Log current joint positions
            if len(msg.position) >= 3:
                self.get_logger().info(
                    f'Current positions: '
                    f'shoulder={msg.position[0]:.2f} rad, '
                    f'elbow={msg.position[1]:.2f} rad, '
                    f'wrist={msg.position[2]:.2f} rad'
                )

    def get_statistics(self):
        """
        Returns monitoring statistics.

        Returns:
            dict: Statistics including message count and warning count
        """
        return {
            'messages_received': self.message_count,
            'warnings_issued': self.warning_count,
            'warning_rate': self.warning_count / max(self.message_count, 1)
        }


def main(args=None):
    """
    Main function to initialize ROS 2 and run the node.

    Args:
        args: Command-line arguments (optional)
    """
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create node instance
    node = JointController()

    try:
        # Spin node (process callbacks until shutdown)
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Print statistics on shutdown
        stats = node.get_statistics()
        node.get_logger().info('Keyboard interrupt, shutting down...')
        node.get_logger().info(
            f'Final statistics: {stats["messages_received"]} messages, '
            f'{stats["warnings_issued"]} warnings '
            f'({stats["warning_rate"]*100:.1f}% rate)'
        )
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
