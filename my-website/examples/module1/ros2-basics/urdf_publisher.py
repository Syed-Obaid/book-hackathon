#!/usr/bin/env python3
"""
URDF Joint State Publisher Node
Reference: Chapter 1.4 Nodes and Topics

Publishes joint states for the 3-DOF robotic arm defined in simple_arm.urdf.
This node demonstrates:
- Creating a ROS 2 publisher
- Publishing sensor_msgs/JointState messages
- Using timers for periodic publishing
- Generating dynamic joint motion (sinusoidal for demo)

Requirements:
- rclpy >= 3.3.0
- sensor_msgs

Usage:
    python3 urdf_publisher.py

Topics Published:
    /joint_states (sensor_msgs/JointState): Joint positions, velocities, efforts
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
import math


class URDFPublisher(Node):
    """
    Publishes joint states for a 3-DOF robotic arm.

    The node publishes to /joint_states at 50 Hz, simulating joint motion
    with sinusoidal patterns for demonstration purposes.
    """

    def __init__(self):
        super().__init__('urdf_publisher')

        # Create publisher: topic name, message type, queue size
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        # Create timer: 50 Hz (0.02 second period)
        self.timer = self.create_timer(0.02, self.timer_callback)

        # State variables for animation
        self.time_elapsed = 0.0

        # Joint names must match URDF
        self.joint_names = ['shoulder_joint', 'elbow_joint', 'wrist_joint']

        self.get_logger().info('URDF Publisher started')
        self.get_logger().info('Publishing to /joint_states at 50 Hz')
        self.get_logger().info(f'Joint names: {self.joint_names}')

    def timer_callback(self):
        """
        Timer callback executed at 50 Hz.

        Publishes JointState message with:
        - header.stamp: Current ROS time
        - name: List of joint names
        - position: Joint angles in radians
        - velocity: Joint angular velocities in rad/s
        - effort: Joint torques in N·m (set to 0 for simulation)
        """
        msg = JointState()

        # Populate header with current timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''  # Not used for joint states

        # Joint names from URDF
        msg.name = self.joint_names

        # Generate dynamic joint positions (sinusoidal motion for demo)
        # Shoulder: oscillates between -1.5 and +1.5 rad with 2s period
        shoulder_pos = 1.5 * math.sin(2 * math.pi * self.time_elapsed / 2.0)

        # Elbow: oscillates between 0.5 and 2.0 rad with 3s period
        elbow_pos = 1.25 + 0.75 * math.sin(2 * math.pi * self.time_elapsed / 3.0)

        # Wrist: continuous rotation at 0.5 rad/s
        wrist_pos = 0.5 * self.time_elapsed

        msg.position = [shoulder_pos, elbow_pos, wrist_pos]

        # Calculate velocities (derivatives of positions)
        shoulder_vel = 1.5 * (2 * math.pi / 2.0) * math.cos(2 * math.pi * self.time_elapsed / 2.0)
        elbow_vel = 0.75 * (2 * math.pi / 3.0) * math.cos(2 * math.pi * self.time_elapsed / 3.0)
        wrist_vel = 0.5

        msg.velocity = [shoulder_vel, elbow_vel, wrist_vel]

        # Efforts (torques) - set to 0 for kinematic simulation
        msg.effort = [0.0, 0.0, 0.0]

        # Publish message
        self.publisher_.publish(msg)

        # Update time (increment by 20ms)
        self.time_elapsed += 0.02

        # Log every 50 messages (1 Hz feedback to console)
        if int(self.time_elapsed * 50) % 50 == 0:
            self.get_logger().info(
                f't={self.time_elapsed:.1f}s | '
                f'shoulder={shoulder_pos:.2f} rad, '
                f'elbow={elbow_pos:.2f} rad, '
                f'wrist={wrist_pos:.2f} rad'
            )


def main(args=None):
    """
    Main function to initialize ROS 2 and run the node.

    Args:
        args: Command-line arguments (optional)
    """
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create node instance
    node = URDFPublisher()

    try:
        # Spin node (process callbacks until shutdown)
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
