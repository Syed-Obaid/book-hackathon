#!/usr/bin/env python3
"""
URDF Visualizer Launch File
Reference: Chapter 1.4 Nodes and Topics

Launches multiple nodes for visualizing the 3-DOF robotic arm in RViz:
1. robot_state_publisher: Publishes robot transforms based on URDF
2. joint_state_publisher_gui: Provides GUI sliders for manual joint control
3. RViz2: 3D visualization of robot state

Requirements:
- robot_state_publisher
- joint_state_publisher_gui
- rviz2
- simple_arm.urdf file

Usage:
    ros2 launch urdf_visualizer.launch.py

Or with custom URDF:
    ros2 launch urdf_visualizer.launch.py urdf_file:=/path/to/robot.urdf
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description with multiple nodes for URDF visualization.

    Returns:
        LaunchDescription: Launch configuration with all required nodes
    """

    # Declare launch arguments
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(
            os.path.dirname(__file__),
            '..',
            'urdf-models',
            'simple_arm.urdf'
        ),
        description='Path to URDF file'
    )

    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Use joint_state_publisher_gui (true) or joint_state_publisher (false)'
    )

    # Get launch configuration values
    urdf_file = LaunchConfiguration('urdf_file')
    use_gui = LaunchConfiguration('use_gui')

    # Read URDF file content
    # Using Command substitution to read file at runtime
    robot_description = Command(['cat ', urdf_file])

    # Node 1: robot_state_publisher
    # Publishes robot transforms (/tf) based on URDF and joint states
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # Node 2: joint_state_publisher_gui
    # Provides GUI with sliders to control joint positions
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=launch.conditions.IfCondition(use_gui)
    )

    # Node 3 (alternative): joint_state_publisher (no GUI)
    # Publishes default joint states
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=launch.conditions.UnlessCondition(use_gui)
    )

    # Node 4: RViz2
    # 3D visualization of robot
    # Use default config (users can save custom configs later)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            os.path.dirname(__file__),
            'default.rviz'
        )] if os.path.exists(os.path.join(
            os.path.dirname(__file__),
            'default.rviz'
        )) else []
    )

    # Create launch description
    return LaunchDescription([
        # Launch arguments
        urdf_file_arg,
        use_gui_arg,

        # Nodes
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        rviz_node
    ])


# Required import for conditions
from launch import conditions as launch_conditions
launch.conditions = launch_conditions
