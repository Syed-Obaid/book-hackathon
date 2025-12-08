# Module 1: ROS 2 Foundation - Code Examples

This directory contains working code examples for Module 1 of the Physical AI & Humanoid Robotics Hackathon Guide.

## Contents

- **Installation Scripts**
  - `install_ros2.sh` - Automated ROS 2 Humble installation for Ubuntu 22.04
  - `create_workspace.sh` - ROS 2 workspace creation with colcon

- **Python Nodes**
  - `urdf_publisher.py` - Publishes joint states at 50 Hz
  - `joint_controller.py` - Subscribes to joint states with safety monitoring
  - `urdf_visualizer.launch.py` - Launch file for RViz visualization

- **URDF Models**
  - `../urdf-models/simple_arm.urdf` - 3-DOF robotic arm model

- **Dependencies**
  - `requirements.txt` - Python package dependencies

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble (install using `install_ros2.sh` or manually per Chapter 1.2)
- Python 3.10+

## Quick Start

### 1. Install ROS 2 (if not already installed)

```bash
chmod +x install_ros2.sh
./install_ros2.sh
```

### 2. Create ROS 2 Workspace

```bash
chmod +x create_workspace.sh
./create_workspace.sh
# Default workspace name: ros2_ws
# Custom name: ./create_workspace.sh my_workspace
```

### 3. Install Python Dependencies

```bash
pip3 install -r requirements.txt
```

### 4. Visualize URDF in RViz

**Option A: Using launch file (recommended)**

```bash
# Install required ROS 2 packages
sudo apt install ros-humble-joint-state-publisher-gui \
                 ros-humble-robot-state-publisher \
                 ros-humble-rviz2

# Launch visualization
ros2 launch urdf_visualizer.launch.py
```

This launches:
- `robot_state_publisher` - Publishes transforms from URDF
- `joint_state_publisher_gui` - GUI sliders for joint control
- `rviz2` - 3D visualization

**Option B: Manual node execution**

Terminal 1 (Publisher):
```bash
python3 urdf_publisher.py
```

Terminal 2 (Controller):
```bash
python3 joint_controller.py
```

Terminal 3 (Visualization):
```bash
ros2 run rviz2 rviz2
# In RViz: Add > RobotModel, set Fixed Frame to "base_link"
```

### 5. Monitor Joint States

```bash
# List active topics
ros2 topic list

# Echo joint state messages
ros2 topic echo /joint_states

# Measure publishing rate (should be ~50 Hz)
ros2 topic hz /joint_states
```

## Code Examples Explained

### urdf_publisher.py

Demonstrates:
- Creating a ROS 2 publisher
- Publishing `sensor_msgs/JointState` messages
- Using timers for periodic callbacks (50 Hz)
- Generating sinusoidal joint motion for animation

**Key concepts (Chapter 1.4)**:
```python
# Create publisher
self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

# Create timer (50 Hz)
self.timer = self.create_timer(0.02, self.timer_callback)

# Publish message
self.publisher_.publish(msg)
```

### joint_controller.py

Demonstrates:
- Creating a ROS 2 subscriber
- Processing incoming messages in callbacks
- Implementing safety checks (joint limit validation)
- State tracking and statistics

**Key concepts (Chapter 1.4)**:
```python
# Create subscription
self.subscription = self.create_subscription(
    JointState, '/joint_states', self.listener_callback, 10
)

# Process messages
def listener_callback(self, msg):
    # Validate joint limits
    if position < limits['lower'] or position > limits['upper']:
        self.get_logger().warn('Joint limit exceeded')
```

### urdf_visualizer.launch.py

Demonstrates:
- Writing Python launch files
- Launching multiple nodes simultaneously
- Using launch arguments for configuration
- Reading URDF files at runtime

**Key concepts (Chapter 1.4)**:
```python
# Launch argument
urdf_file_arg = DeclareLaunchArgument(
    'urdf_file', default_value='simple_arm.urdf'
)

# Launch multiple nodes
return LaunchDescription([
    robot_state_publisher_node,
    joint_state_publisher_gui_node,
    rviz_node
])
```

## Expected Output

### urdf_publisher.py

```
[INFO] [urdf_publisher]: URDF Publisher started
[INFO] [urdf_publisher]: Publishing to /joint_states at 50 Hz
[INFO] [urdf_publisher]: Joint names: ['shoulder_joint', 'elbow_joint', 'wrist_joint']
[INFO] [urdf_publisher]: t=1.0s | shoulder=0.95 rad, elbow=1.88 rad, wrist=0.50 rad
[INFO] [urdf_publisher]: t=2.0s | shoulder=-0.31 rad, elbow=0.50 rad, wrist=1.00 rad
...
```

### joint_controller.py

```
[INFO] [joint_controller]: Joint Controller started
[INFO] [joint_controller]: Subscribing to /joint_states
[INFO] [joint_controller]: Monitoring 3 joints for safety
[INFO] [joint_controller]: Processed 100 messages, 0 warnings
[INFO] [joint_controller]: Current positions: shoulder=1.20 rad, elbow=1.50 rad, wrist=0.80 rad
...
```

## Troubleshooting

### Issue: ros2 command not found

**Solution**: Source ROS 2 environment
```bash
source /opt/ros/humble/setup.bash
```

### Issue: Permission denied when running scripts

**Solution**: Make scripts executable
```bash
chmod +x install_ros2.sh create_workspace.sh
chmod +x urdf_publisher.py joint_controller.py
```

### Issue: Module 'rclpy' has no attribute 'init'

**Solution**: Install rclpy
```bash
sudo apt install python3-rclpy
# Or using pip:
pip3 install rclpy
```

### Issue: URDF file not found in launch file

**Solution**: Use absolute path
```bash
ros2 launch urdf_visualizer.launch.py \
  urdf_file:=/full/path/to/simple_arm.urdf
```

### Issue: RViz shows "No tf data"

**Solution**: Ensure robot_state_publisher is running and check Fixed Frame
```bash
# Check if transforms are published
ros2 topic echo /tf

# In RViz: Global Options > Fixed Frame > set to "base_link"
```

## Related Chapters

- **Chapter 1.2**: Installation & Workspace Setup
- **Chapter 1.3**: URDF Basics
- **Chapter 1.4**: Nodes and Topics
- **Chapter 1.5**: Services and Actions

## Additional Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [rclpy API Reference](https://docs.ros2.org/humble/api/rclpy/)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [RViz User Guide](https://github.com/ros2/rviz)

## License

These examples are part of the Physical AI & Humanoid Robotics Hackathon Guide.
Educational use only.
