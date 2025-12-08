# Module 2 Code Examples: Gazebo Digital Twin Simulation

## Files

- `gazebo_world.sdf` - Complete Gazebo world with obstacles and lighting
- `sensor_publisher.py` - ROS 2 node for processing simulated sensor data

## Usage

### Launch Gazebo World
```bash
gz sim gazebo_world.sdf
```

### Run Sensor Publisher
```bash
chmod +x sensor_publisher.py
ros2 run <your_package> sensor_publisher.py
```

## Requirements

- ROS 2 Humble
- Gazebo Garden
- Python 3.10+
- rclpy >= 3.3.0
