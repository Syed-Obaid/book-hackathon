# Module 1 Outline: ROS 2 Foundation for Humanoid Control

**Module Duration**: Week 1-3 (12 hours total)
**Target Word Count**: 5000-6000 words total
**Prerequisites**: Python programming, Linux command line basics
**Learning Outcome**: Students can install ROS 2 Humble, create workspaces, define URDF robot models, write publisher/subscriber nodes, and visualize robots in RViz.

---

## Chapter 1.1: Overview (500 words)

### Learning Objectives
By the end of this chapter, students will be able to:
1. Explain why roboticists use ROS 2 instead of writing custom communication protocols
2. Identify the key components of ROS 2 architecture (nodes, topics, DDS layer)
3. Describe how ROS 2 enables modular robot software development
4. Recognize when to use ROS 2 vs other robotics frameworks

### Content Structure

**Section 1: What is ROS 2?** (150 words)
- Definition: Robot Operating System 2 as middleware framework
- Evolution from ROS 1 (2007) to ROS 2 (2017+)
- Key difference: DDS (Data Distribution Service) for real-time communication
- Industry adoption: NASA, Boston Dynamics proxies, academic labs

**Section 2: Why Roboticists Use ROS 2** (200 words)
- **Problem**: Custom robot software is monolithic and hard to debug
- **Solution**: ROS 2 provides modular architecture with standardized interfaces
- **Benefits**: Code reuse, community packages, hardware abstraction
- **Example**: Controlling a humanoid requires coordination of 20+ motors, sensors (IMU, cameras), and AI perception—ROS 2 lets each subsystem run as independent node

**Section 3: Key Concepts Preview** (100 words)
- **Nodes**: Independent processes (e.g., joint_controller, camera_driver)
- **Topics**: Asynchronous message streams (e.g., /joint_states publishes at 50 Hz)
- **Services**: Request-reply patterns (e.g., /reset_robot)
- **Actions**: Long-running tasks with feedback (e.g., /navigate_to_goal)
- **URDF**: Robot description format defining links, joints, sensors

**Section 4: Motivation for Humanoid Control** (50 words)
- Humanoid robots require real-time coordination of 30+ degrees of freedom
- ROS 2's decoupled architecture allows parallel development of perception, planning, and control
- Visualization tools (RViz) enable debugging without hardware

### Diagrams
- **Mermaid Diagram** (inline): ROS 2 high-level architecture showing nodes communicating via topics over DDS layer (T038)

### Exercises
None (this is introductory chapter)

---

## Chapter 1.2: Installation & Workspace Setup (800 words)

### Learning Objectives
By the end of this chapter, students will be able to:
1. Install ROS 2 Humble on Ubuntu 22.04 via apt
2. Create and build a ROS 2 workspace using colcon
3. Source environment variables correctly for workspace activation
4. Troubleshoot common installation errors (missing dependencies, path issues)

### Content Structure

**Section 1: Prerequisites Check** (100 words)
- **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish) required
- **Hardware**: 4GB RAM minimum, 20GB disk space for ROS 2 + dependencies
- **Verification commands**:
  ```bash
  lsb_release -a  # Should show Ubuntu 22.04
  uname -r        # Kernel version check
  ```
- **Why Ubuntu 22.04?** ROS 2 Humble targets this LTS version for best compatibility

**Section 2: ROS 2 Humble Installation** (300 words)
- **Step 1**: Set up sources and keys
  ```bash
  sudo apt update && sudo apt install software-properties-common
  sudo add-apt-repository universe
  sudo apt update && sudo apt install curl -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  ```
- **Step 2**: Install ROS 2 Humble desktop (includes RViz, rqt, demos)
  ```bash
  sudo apt update
  sudo apt install ros-humble-desktop python3-argcomplete -y
  ```
- **Step 3**: Install development tools
  ```bash
  sudo apt install ros-dev-tools python3-colcon-common-extensions -y
  ```
- **Verification**:
  ```bash
  source /opt/ros/humble/setup.bash
  ros2 run demo_nodes_cpp talker  # Should see messages publishing
  ```

**Section 3: Workspace Creation** (250 words)
- **What is a workspace?** Directory structure for organizing custom ROS 2 packages
- **Workspace structure**:
  ```
  ros2_ws/
  ├── src/          # Source code (Git repositories, custom packages)
  ├── build/        # Intermediate build files (auto-generated)
  ├── install/      # Executables and libraries (auto-generated)
  └── log/          # Build logs (auto-generated)
  ```
- **Creating workspace**:
  ```bash
  mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws
  colcon build --symlink-install  # Initial build (creates build/, install/, log/)
  ```
- **Sourcing workspace**:
  ```bash
  source ~/ros2_ws/install/setup.bash
  echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc  # Auto-source on terminal startup
  ```
- **Testing**: Verify workspace is active with `echo $AMENT_PREFIX_PATH` (should show workspace path)

**Section 4: Troubleshooting Common Errors** (150 words)
- **Error 1**: `ros2: command not found`
  - **Cause**: Setup script not sourced
  - **Fix**: `source /opt/ros/humble/setup.bash`
- **Error 2**: `Package 'ros-humble-desktop' has no installation candidate`
  - **Cause**: ROS 2 repository not added correctly
  - **Fix**: Re-run setup sources commands from Section 2, Step 1
- **Error 3**: `colcon: command not found`
  - **Cause**: Development tools not installed
  - **Fix**: `sudo apt install python3-colcon-common-extensions`
- **Error 4**: Build fails with "No such file or directory"
  - **Cause**: Missing dependencies in package.xml
  - **Fix**: Install dependencies with `rosdep install --from-paths src --ignore-src -y`

### Diagrams
- **Mermaid Diagram** (inline): Workspace directory tree showing src/, build/, install/, log/ structure (T039)

### Exercises
1. **Exercise 1.1** (Easy): Verify ROS 2 installation by running `ros2 topic list` and identifying 2 active topics
2. **Exercise 1.2** (Medium): Create a workspace named `humanoid_ws`, build it, and confirm `AMENT_PREFIX_PATH` includes your workspace
3. **Exercise 1.3** (Medium): Intentionally break your workspace by removing `install/setup.bash`, then troubleshoot and fix it

---

## Chapter 1.3: URDF Basics (1000 words)

### Learning Objectives
By the end of this chapter, students will be able to:
1. Write a valid URDF XML file defining a multi-joint robot
2. Explain the relationship between links, joints, and coordinate frames
3. Visualize URDF models in RViz and verify kinematic constraints
4. Debug common URDF errors (joint limits, collision geometry mismatches)

### Content Structure

**Section 1: What is URDF?** (150 words)
- **Definition**: Unified Robot Description Format—XML specification for robot models
- **Purpose**: Describes robot's physical structure (links, joints) and visual/collision geometry
- **Used by**: RViz (visualization), Gazebo (simulation), MoveIt (motion planning), control systems
- **Example use case**: Humanoid robot with 12-DOF legs, 8-DOF arms, 2-DOF head requires URDF to define kinematics

**Section 2: URDF Structure—Links and Joints** (300 words)
- **Links**: Rigid bodies with visual, collision, and inertial properties
  ```xml
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.05"/>  <!-- 10cm x 10cm x 5cm box -->
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>  <!-- Blue color for visualization -->
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.05"/>  <!-- Same as visual for simplicity -->
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>  <!-- 1 kg -->
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  ```
- **Joints**: Connections between links defining motion constraints
  - **Fixed**: No movement (e.g., camera mounted on head)
  - **Revolute**: Rotation around axis with limits (e.g., elbow joint: 0° to 150°)
  - **Continuous**: Unlimited rotation (e.g., wheel)
  - **Prismatic**: Linear sliding (e.g., telescope arm)
  ```xml
  <joint name="shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="upper_arm"/>
    <origin xyz="0 0.15 0.3" rpy="0 0 0"/>  <!-- Joint position relative to parent -->
    <axis xyz="0 1 0"/>  <!-- Rotates around y-axis -->
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>  <!-- ±90° limits -->
  </joint>
  ```

**Section 3: Coordinate Frames and Transformations** (200 words)
- **Frame conventions**: Right-hand rule (x-forward, y-left, z-up for humanoid robotics)
- **Origin tag**: Defines child frame relative to parent using xyz (position in meters) and rpy (roll-pitch-yaw in radians)
- **Example**: Shoulder joint at (0, 0.15, 0.3) means 15cm left, 30cm up from torso origin
- **Common pitfall**: Forgetting to account for link geometry offset—visual geometry is centered at link origin, but joint origin may differ
- **Denavit-Hartenberg parameters** (brief intro): Standard convention for defining serial manipulator frames (covered in detail in exercises)

**Section 4: 3-DOF Arm Example** (250 words)
- **Goal**: Define a simple robotic arm with 3 revolute joints (shoulder, elbow, wrist)
- **Link structure**:
  - `base_link` (fixed platform)
  - `upper_arm` (30cm long cylinder)
  - `forearm` (25cm long cylinder)
  - `hand` (10cm box)
- **Joint structure**:
  - `shoulder_joint`: Revolute, ±90° limits, axis=[0,1,0]
  - `elbow_joint`: Revolute, 0° to 150° limits, axis=[0,1,0]
  - `wrist_joint`: Continuous (unlimited rotation), axis=[1,0,0]
- **Code snippet**: Complete URDF with 4 links and 3 joints (available in `examples/module1/urdf-models/simple_arm.urdf`)
- **Visualization test**: Load in RViz using `ros2 launch urdf_tutorial display.launch.py model:=simple_arm.urdf` and move joints with GUI sliders

**Section 5: Common URDF Pitfalls** (100 words)
- **Pitfall 1**: Joint limits reversed (lower > upper) → RViz displays red error
- **Pitfall 2**: Missing `<inertial>` tag → Gazebo simulation crashes
- **Pitfall 3**: Collision geometry too large → Robot self-collides immediately
- **Pitfall 4**: Origin xyz/rpy typo → Robot appears disjointed in RViz
- **Debugging tip**: Use `check_urdf simple_arm.urdf` command to validate XML syntax and joint tree

### Diagrams
- **Mermaid Diagram** (inline): Kinematic tree visualization showing parent-child link relationships for 3-DOF arm (T040)
- **Excalidraw Diagram**: Coordinate frame conventions with DH parameters (exported to SVG) (T041)

### Exercises
1. **Exercise 1.4** (Easy): Modify `simple_arm.urdf` to change upper arm length from 30cm to 40cm and verify in RViz
2. **Exercise 1.5** (Medium): Add a 4th joint (`gripper_joint`, prismatic, 0-0.05m range) to the wrist and update visualization
3. **Exercise 1.6** (Hard): Create a URDF for a 6-DOF humanoid leg with hip (3-DOF ball joint approximated as 3 revolute joints), knee (1-DOF), ankle (2-DOF). Include realistic joint limits based on human anatomy.

---

## Chapter 1.4: Nodes and Topics (900 words)

### Learning Objectives
By the end of this chapter, students will be able to:
1. Write Python ROS 2 nodes using rclpy for publishing and subscribing
2. Explain the publisher-subscriber pattern and when to use it
3. Launch multiple nodes using ROS 2 launch files
4. Debug topic communication using `ros2 topic` CLI tools

### Content Structure

**Section 1: What are Nodes?** (150 words)
- **Definition**: Independent processes that perform specific tasks (modular design)
- **Examples**:
  - `joint_state_publisher`: Publishes robot joint angles
  - `camera_driver`: Publishes image data from sensors
  - `motion_planner`: Subscribes to sensor data, publishes movement commands
- **Why separate nodes?** Fault isolation (crash in one node doesn't kill entire system), parallel development, easy testing
- **Communication**: Nodes discover each other automatically via DDS (no central server in ROS 2, unlike ROS 1)

**Section 2: Publisher-Subscriber Pattern** (200 words)
- **Concept**: Asynchronous, many-to-many message passing
- **Publisher**: Node that sends data on a topic
- **Subscriber**: Node that receives data from a topic
- **Topic**: Named communication channel (e.g., `/joint_states`, `/camera/image_raw`)
- **Message types**: Predefined structures (e.g., `sensor_msgs/JointState`, `geometry_msgs/Twist`)
- **QoS (Quality of Service)**: Reliability settings (reliable vs best-effort), history depth
- **Example flow**:
  1. `joint_controller` node publishes joint angles on `/joint_commands` at 50 Hz
  2. `robot_driver` node subscribes to `/joint_commands` and sends to hardware
  3. `visualizer` node also subscribes to `/joint_commands` for RViz display
- **Decoupling benefit**: `joint_controller` doesn't need to know how many subscribers exist

**Section 3: Writing a Python Publisher Node** (250 words)
- **Setup**: Import rclpy and message types
  ```python
  import rclpy
  from rclpy.node import Node
  from sensor_msgs.msg import JointState
  ```
- **Node class structure**:
  ```python
  class JointPublisher(Node):
      def __init__(self):
          super().__init__('joint_publisher')
          self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
          self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz
          self.joint_angle = 0.0

      def timer_callback(self):
          msg = JointState()
          msg.header.stamp = self.get_clock().now().to_msg()
          msg.name = ['shoulder_joint', 'elbow_joint', 'wrist_joint']
          msg.position = [self.joint_angle, 0.5, 0.0]
          self.publisher_.publish(msg)
          self.joint_angle += 0.01  # Increment shoulder angle
          self.get_logger().info(f'Publishing: {msg.position[0]:.2f}')
  ```
- **Main function**:
  ```python
  def main(args=None):
      rclpy.init(args=args)
      node = JointPublisher()
      rclpy.spin(node)
      node.destroy_node()
      rclpy.shutdown()
  ```
- **Running**: `python3 joint_publisher.py` (or `ros2 run my_package joint_publisher` if installed)

**Section 4: Writing a Python Subscriber Node** (200 words)
- **Subscriber setup**:
  ```python
  class JointController(Node):
      def __init__(self):
          super().__init__('joint_controller')
          self.subscription = self.create_subscription(
              JointState, '/joint_states', self.listener_callback, 10)

      def listener_callback(self, msg):
          self.get_logger().info(f'Received joint angles: {msg.position}')
          # Add control logic here (e.g., PID controller, safety checks)
  ```
- **QoS considerations**: Default QoS is `Reliable` with `KeepLast(10)` history
- **Callback threading**: Callbacks execute in node's executor thread—avoid blocking operations (> 10ms)

**Section 5: ROS 2 Launch Files** (100 words)
- **Purpose**: Start multiple nodes with one command
- **Python launch file** (`urdf_visualizer.launch.py`):
  ```python
  from launch import LaunchDescription
  from launch_ros.actions import Node

  def generate_launch_description():
      return LaunchDescription([
          Node(package='robot_state_publisher', executable='robot_state_publisher',
               arguments=['simple_arm.urdf']),
          Node(package='rviz2', executable='rviz2', arguments=['-d', 'config.rviz']),
          Node(package='my_package', executable='joint_publisher'),
      ])
  ```
- **Running**: `ros2 launch my_package urdf_visualizer.launch.py`

### Diagrams
- **Mermaid Diagram** (inline): ROS 2 node graph showing publisher→topic→subscriber relationships with multiple nodes (T042)

### Exercises
1. **Exercise 1.7** (Easy): Run `ros2 topic list` and `ros2 topic echo /joint_states` to inspect published messages
2. **Exercise 1.8** (Medium): Modify `joint_publisher.py` to publish at 100 Hz instead of 50 Hz and measure latency
3. **Exercise 1.9** (Hard): Create a subscriber node that reads `/joint_states`, applies a low-pass filter (moving average over 10 samples), and republishes filtered data on `/joint_states_filtered`

---

## Chapter 1.5: Services and Actions (700 words)

### Learning Objectives
By the end of this chapter, students will be able to:
1. Differentiate between topics, services, and actions based on communication patterns
2. Implement a simple ROS 2 service for request-reply interactions
3. Use action servers for long-running tasks with feedback
4. Decide when to use each communication primitive

### Content Structure

**Section 1: When to Use Services vs Topics** (150 words)
- **Topics** (asynchronous streaming):
  - Use for: Sensor data (continuous streams), robot state (joint angles at 50 Hz)
  - Example: `/camera/image_raw` published at 30 FPS
- **Services** (synchronous request-reply):
  - Use for: Occasional commands with guaranteed response (reset robot, query state)
  - Example: `/get_robot_pose` service returns current position
- **Actions** (asynchronous tasks with feedback):
  - Use for: Long-running operations with progress updates (navigate to goal, pick object)
  - Example: `/navigate_to_goal` action sends progress updates every 0.5s

**Section 2: Service Definitions** (150 words)
- **Service type structure**: Request + Response
- **Example** (`GetJointLimits.srv`):
  ```
  string joint_name
  ---
  float64 lower_limit
  float64 upper_limit
  bool success
  ```
- **Built-in service types**: `std_srvs/Trigger`, `example_interfaces/AddTwoInts`
- **Service server implementation**:
  ```python
  from example_interfaces.srv import AddTwoInts

  class MinimalService(Node):
      def __init__(self):
          super().__init__('minimal_service')
          self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)

      def add_callback(self, request, response):
          response.sum = request.a + request.b
          self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
          return response
  ```

**Section 3: Service Client** (100 words)
- **Client implementation**:
  ```python
  client = self.create_client(AddTwoInts, 'add_two_ints')
  request = AddTwoInts.Request()
  request.a = 5
  request.b = 7
  future = client.call_async(request)
  rclpy.spin_until_future_complete(self, future)
  result = future.result()
  self.get_logger().info(f'Result: {result.sum}')
  ```
- **Blocking vs async calls**: Use `call_async()` to avoid blocking node execution

**Section 4: Actions for Long-Running Tasks** (200 words)
- **Action structure**: Goal + Result + Feedback
- **Example** (`NavigateToGoal.action`):
  ```
  geometry_msgs/Pose target_pose
  ---
  bool success
  float64 final_distance
  ---
  float64 distance_remaining
  float64 time_elapsed
  ```
- **Action server** (simplified):
  ```python
  from action_msgs.msg import GoalStatus
  from my_actions.action import NavigateToGoal

  class NavigationActionServer(Node):
      def __init__(self):
          super().__init__('navigation_action_server')
          self._action_server = ActionServer(
              self, NavigateToGoal, 'navigate_to_goal', self.execute_callback)

      def execute_callback(self, goal_handle):
          feedback_msg = NavigateToGoal.Feedback()
          for i in range(10):
              feedback_msg.distance_remaining = 10.0 - i
              goal_handle.publish_feedback(feedback_msg)
              time.sleep(1.0)  # Simulate navigation progress
          goal_handle.succeed()
          result = NavigateToGoal.Result()
          result.success = True
          return result
  ```
- **Action client**: Subscribe to feedback, handle preemption (cancel requests)

**Section 5: Decision Matrix** (100 words)
| Communication | Latency | Use Case | Example |
|---------------|---------|----------|---------|
| **Topic** | Low (<1ms) | Continuous data streams | Joint states at 50 Hz |
| **Service** | Medium (~10ms) | Occasional commands | Reset simulation |
| **Action** | High (seconds) | Long tasks with feedback | Navigate 10 meters |

### Exercises
1. **Exercise 1.10** (Easy): Call the `add_two_ints` service from command line: `ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 5}"`
2. **Exercise 1.11** (Medium): Create a service `/reset_joint` that resets a specified joint to angle 0.0
3. **Exercise 1.12** (Hard): Implement an action server for `/move_arm` that moves a 3-DOF arm to target angles with feedback showing progress percentage

---

## Chapter 1.6: Exercises (1100 words)

### Learning Objectives
By the end of this chapter, students will be able to:
1. Apply all Module 1 concepts to complete 18 graded exercises
2. Integrate URDF, nodes, topics, services, and actions into a cohesive project
3. Debug complex ROS 2 systems using CLI tools and visualization
4. Demonstrate mastery through a graded capstone mini-project

### Content Structure

**Section 1: Overview of Exercise Structure** (100 words)
- **Total exercises**: 18 (3 per chapter: easy, medium, hard)
- **Grading rubric**:
  - **Easy** (⭐): Basic comprehension, 5 points each
  - **Medium** (⭐⭐): Applied understanding, 10 points each
  - **Hard** (⭐⭐⭐): Synthesis and debugging, 15 points each
- **Total points**: 180 (Easy: 30, Medium: 60, Hard: 90)
- **Passing score**: 140/180 (78%)
- **Submission format**: GitHub repository with code, screenshots, and 1-page technical report per hard exercise

**Section 2: Installation & Workspace Exercises** (100 words)
- **Exercise 1.1** ⭐: Verify ROS 2 installation (covered in Chapter 1.2)
- **Exercise 1.2** ⭐⭐: Create workspace (covered in Chapter 1.2)
- **Exercise 1.3** ⭐⭐: Troubleshoot broken workspace (covered in Chapter 1.2)

**Section 3: URDF Exercises** (150 words)
- **Exercise 1.4** ⭐: Modify arm length (covered in Chapter 1.3)
- **Exercise 1.5** ⭐⭐: Add gripper joint (covered in Chapter 1.3)
- **Exercise 1.6** ⭐⭐⭐: Design 6-DOF leg (covered in Chapter 1.3)
  - **Grading criteria**:
    - URDF passes `check_urdf` validation (20%)
    - Joint limits match human anatomy (hip: ±120°, knee: 0-150°, ankle: ±30°) (30%)
    - Visualizes correctly in RViz without self-collision (30%)
    - Technical report explains DH parameter choices (20%)

**Section 4: Nodes and Topics Exercises** (150 words)
- **Exercise 1.7** ⭐: Inspect topics with CLI (covered in Chapter 1.4)
- **Exercise 1.8** ⭐⭐: Modify publishing rate (covered in Chapter 1.4)
- **Exercise 1.9** ⭐⭐⭐: Implement low-pass filter node (covered in Chapter 1.4)
  - **Grading criteria**:
    - Node subscribes to `/joint_states` and publishes to `/joint_states_filtered` (25%)
    - Implements moving average filter over 10 samples (25%)
    - Maintains 50 Hz publishing rate (no buffer overflow) (25%)
    - Technical report shows frequency response plot (signal vs filtered signal) (25%)

**Section 5: Services and Actions Exercises** (150 words)
- **Exercise 1.10** ⭐: Call service from CLI (covered in Chapter 1.5)
- **Exercise 1.11** ⭐⭐: Create `/reset_joint` service (covered in Chapter 1.5)
- **Exercise 1.12** ⭐⭐⭐: Implement `/move_arm` action server (covered in Chapter 1.5)
  - **Grading criteria**:
    - Action server accepts 3 target joint angles (25%)
    - Publishes feedback with progress percentage every 0.5s (25%)
    - Uses cubic trajectory interpolation (smooth motion) (25%)
    - Technical report includes motion profile plot (position vs time) (25%)

**Section 6: Integration Exercises** (250 words)
- **Exercise 1.13** ⭐: Launch RViz with custom URDF and verify joint state visualization
  - **Instructions**: Use `ros2 launch` with provided `urdf_visualizer.launch.py`
  - **Expected output**: RViz displays 3-DOF arm with movable joint sliders

- **Exercise 1.14** ⭐⭐: Create a keyboard teleoperation node
  - **Task**: Write a node that listens to keyboard input (arrow keys) and publishes joint velocity commands
  - **Requirements**: Use `pynput` library for keyboard capture, publish to `/joint_velocity_commands` (Float64MultiArray)
  - **Grading**: Smooth control (no jerky motion), emergency stop on 'Space' key (30%), code comments explaining input handling (20%)

- **Exercise 1.15** ⭐⭐⭐: Multi-robot coordination
  - **Task**: Launch 2 simulated robots (different URDF namespaces) and synchronize their joint movements
  - **Requirements**:
    - Robot A subscribes to `/robot_a/joint_commands`
    - Robot B mirrors movements with 1-second delay
    - Use ROS 2 parameters to configure delay dynamically
  - **Grading criteria**:
    - Correct namespace usage (no topic collision) (30%)
    - Delay implemented using `threading.Timer` or `rclpy.Rate` (30%)
    - Technical report discusses synchronization challenges (latency, drift) (40%)

**Section 7: Graded Capstone Mini-Project** (200 words)
- **Exercise 1.16-1.18**: Comprehensive project integrating all Module 1 concepts

**Project Prompt**: Design and implement a 5-DOF robotic arm that can pick and place objects in RViz simulation.

**Requirements**:
1. **URDF Model** (Exercise 1.16, ⭐⭐⭐):
   - 5-DOF arm (3-DOF shoulder/elbow, 2-DOF wrist)
   - End-effector gripper (prismatic joint, 0-0.1m)
   - Visual geometry using basic shapes (cylinders, boxes)
   - Realistic joint limits and inertial properties
   - **Deliverable**: `capstone_arm.urdf` file

2. **Inverse Kinematics Node** (Exercise 1.17, ⭐⭐⭐):
   - Service `/compute_ik` that takes target end-effector pose (x, y, z, roll, pitch, yaw)
   - Returns joint angles to reach target (use analytical IK or PyKDL library)
   - **Deliverable**: `ik_service.py` node

3. **Pick-and-Place Action Server** (Exercise 1.18, ⭐⭐⭐):
   - Action `/pick_and_place` with goal (object_pose, place_pose)
   - Executes motion in 4 phases: approach, grasp, lift, place
   - Publishes feedback with current phase and progress percentage
   - **Deliverable**: `pick_place_action.py` node

**Grading Rubric** (Total: 45 points):
- **URDF Quality** (15 pts): Syntax correctness (5), joint limits (5), visualization (5)
- **IK Correctness** (15 pts): Accuracy within 1cm (10), handles unreachable targets gracefully (5)
- **Action Server** (15 pts): Smooth trajectories (5), accurate feedback (5), error handling (5)

**Submission**:
- GitHub repository with all code, URDF, launch files
- 5-minute video demo showing pick-and-place in RViz
- 3-page technical report covering: design decisions, IK algorithm, challenges faced, future improvements

### Final Notes
- **Due date**: End of Week 3
- **Collaboration policy**: Individual work only (code reviews allowed)
- **Resources**: Office hours, ROS 2 documentation, forum discussions
- **Late policy**: 10% deduction per day

---

## Module 1 Summary

**Total Content**: ~5100 words across 6 chapters
**Learning Outcome Verification**: Students completing all exercises can independently set up ROS 2 workspaces, design URDF models, write communication nodes, and debug robot systems—preparing them for Module 2 (simulation) and beyond.

**Dependencies for Next Module**:
- URDF models will be spawned in Gazebo (Module 2)
- ROS 2 nodes will interface with simulated sensors (Module 2)
- Understanding of topics/services/actions required for AI integration (Module 3)

**Checkpoint**: Module 1 must be fully functional before students proceed to Module 2. Independent test: Can student complete Exercise 1.16-1.18 capstone project without external help?
