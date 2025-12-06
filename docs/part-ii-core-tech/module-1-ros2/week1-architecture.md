---
title: Week 1 - ROS 2 Architecture & Nodes
description: Understanding ROS 2 architecture and creating your first nodes
hide_table_of_contents: false
---

# Week 1: ROS 2 Architecture & Nodes

## Learning Objectives

By the end of this week, you will be able to:
- Understand the fundamental architecture of ROS 2
- Create and manage ROS 2 nodes for different robot functionalities
- Implement basic node structure with proper error handling
- Understand the communication patterns in ROS 2

## Introduction to ROS 2

Robot Operating System 2 (ROS 2) is not an operating system but rather a flexible framework for writing robot software. It provides libraries and tools to help software developers create robot applications. ROS 2 is the next generation of the Robot Operating System, designed to address the limitations of ROS 1 and provide improved capabilities for modern robotics applications.

### Key Improvements in ROS 2

```
ROS 1 vs ROS 2 Architecture Comparison:

ROS 1:                          ROS 2:
┌─────────────────┐             ┌─────────────────┐
│   Master        │             │   DDS (RMW)     │
│  (Centralized)  │             │ (Decentralized) │
└─────────┬───────┘             └─────────┬───────┘
          │                             │
    ┌─────▼─────┐                 ┌─────▼─────┐
    │  Nodes    │                 │  Nodes    │
    │           │                 │           │
    │ Publishers│                 │ Publishers│
    │ Subscribers│                │ Subscribers│
    └───────────┘                 └───────────┘
```

## ROS 2 Architecture Components

### 1. Nodes
Nodes are the fundamental building blocks of ROS 2 applications. A node is a process that performs computation. In ROS 2, nodes are:
- Lightweight processes that perform computation
- Designed to be single-purpose
- Communicate with other nodes through topics, services, and actions
- Managed by the ROS 2 runtime system

### 2. Communication Primitives
- **Topics**: Publish/subscribe communication pattern for streaming data
- **Services**: Request/response communication pattern for single requests
- **Actions**: Goal-based communication pattern for long-running tasks with feedback

### 3. DDS (Data Distribution Service)
ROS 2 uses DDS as its middleware, which provides:
- Discovery of nodes
- Message passing between nodes
- Quality of Service (QoS) policies
- Platform independence

## Setting Up Your First ROS 2 Node

### Creating a Package

Let's create a workspace and a simple package for our first node:

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Create a package for our robot nodes
colcon build
source install/setup.bash
cd src
ros2 pkg create --build-type ament_python robot_basics --dependencies rclpy std_msgs
```

### Basic Node Structure

Here's the basic structure of a ROS 2 node in Python:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Minimal node has been started')

def main(args=None):
    rclpy.init(args=args)
    minimal_node = MinimalNode()

    try:
        rclpy.spin(minimal_node)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Simple Publisher Node

Let's create a node that publishes robot status information:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class RobotStatusPublisher(Node):
    def __init__(self):
        super().__init__('robot_status_publisher')
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.status_msg = String()
        self.status_msg.data = 'Robot operational'
        self.get_logger().info('Robot Status Publisher started')

    def timer_callback(self):
        # Update status message with timestamp
        self.status_msg.data = f'Robot operational at {time.time()}'
        self.publisher_.publish(self.status_msg)
        self.get_logger().info(f'Publishing: "{self.status_msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    robot_status_publisher = RobotStatusPublisher()

    try:
        rclpy.spin(robot_status_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        robot_status_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Simple Subscriber Node

Now, let's create a node that subscribes to the robot status:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotStatusSubscriber(Node):
    def __init__(self):
        super().__init__('robot_status_subscriber')
        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Robot Status Subscriber started')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    robot_status_subscriber = RobotStatusSubscriber()

    try:
        rclpy.spin(robot_status_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        robot_status_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quality of Service (QoS) Settings

QoS settings allow you to control how messages are delivered between nodes:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Create a QoS profile for reliable communication
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST
)

self.publisher_ = self.create_publisher(String, 'robot_status', qos_profile)
```

## Node Parameters

Nodes can accept parameters to configure their behavior:

```python
from rcl_interfaces.msg import ParameterDescriptor

class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')

        # Declare parameters with default values and descriptions
        self.declare_parameter('robot_name', 'DefaultRobot',
                              ParameterDescriptor(description='Name of the robot'))
        self.declare_parameter('update_rate', 1.0,
                              ParameterDescriptor(description='Update rate in Hz'))

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.update_rate = self.get_parameter('update_rate').value

        self.get_logger().info(f'Robot name: {self.robot_name}, Update rate: {self.update_rate}')
```

## Practical Exercise: Robot Controller Node

Let's implement a more complex node that controls a simple robot:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        self.scan_subscriber = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.status_subscriber = self.create_subscription(
            String, 'robot_status', self.status_callback, 10)

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Robot state
        self.scan_data = None
        self.status = "idle"
        self.obstacle_distance = float('inf')

        self.get_logger().info('Robot Controller initialized')

    def scan_callback(self, msg):
        # Find the closest obstacle in front of the robot
        if len(msg.ranges) > 0:
            # Consider only the front 60 degrees
            front_ranges = msg.ranges[len(msg.ranges)//2 - 30:len(msg.ranges)//2 + 30]
            valid_ranges = [r for r in front_ranges if not math.isnan(r) and r > 0]
            if valid_ranges:
                self.obstacle_distance = min(valid_ranges)
            else:
                self.obstacle_distance = float('inf')

    def status_callback(self, msg):
        self.status = msg.data

    def control_loop(self):
        cmd = Twist()

        # Simple obstacle avoidance
        if self.obstacle_distance < 1.0:  # Obstacle within 1 meter
            cmd.linear.x = 0.0  # Stop
            cmd.angular.z = 0.5  # Turn right
            self.get_logger().info('Obstacle detected! Turning right.')
        else:
            cmd.linear.x = 0.5  # Move forward
            cmd.angular.z = 0.0  # No turn

        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()

    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch Files

Launch files allow you to start multiple nodes with a single command:

Create `robot_basics/launch/basic_robot.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'robot_name',
            default_value='DefaultRobot',
            description='Name of the robot'
        ),

        # Robot status publisher
        Node(
            package='robot_basics',
            executable='robot_status_publisher',
            name='robot_status_publisher',
            output='screen'
        ),

        # Robot status subscriber
        Node(
            package='robot_basics',
            executable='robot_status_subscriber',
            name='robot_status_subscriber',
            output='screen'
        ),

        # Robot controller
        Node(
            package='robot_basics',
            executable='robot_controller',
            name='robot_controller',
            parameters=[
                {'robot_name': LaunchConfiguration('robot_name')},
                {'update_rate': 10.0}
            ],
            output='screen'
        )
    ])
```

## Running Your Nodes

### Terminal 1: Start the ROS 2 daemon
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Terminal 2: Run the publisher
```bash
source ~/ros2_ws/install/setup.bash
ros2 run robot_basics robot_status_publisher
```

### Terminal 3: Run the subscriber
```bash
source ~/ros2_ws/install/setup.bash
ros2 run robot_basics robot_status_subscriber
```

### Using Launch Files
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_basics basic_robot.launch.py robot_name:=MyRobot
```

## Debugging Tips

### 1. Check Active Nodes
```bash
ros2 node list
```

### 2. Check Topics
```bash
ros2 topic list
ros2 topic echo /topic_name
```

### 3. Check Services
```bash
ros2 service list
```

### 4. Visualize the Node Graph
```bash
rqt_graph
```

## Lab Exercise: Create Your Own Node

### Objective
Create a node that monitors robot battery level and publishes warnings when the battery is low.

### Requirements
1. Create a publisher that publishes battery level (0-100%) as a Float32 message
2. Create a subscriber that listens to battery level and publishes warnings when below 20%
3. Use appropriate QoS settings for battery monitoring
4. Create a launch file to start both nodes

### Solution Structure
```python
# battery_monitor.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.battery_pub = self.create_publisher(Float32, 'battery_level', 10)
        self.warning_pub = self.create_publisher(String, 'battery_warning', 10)
        self.timer = self.create_timer(1.0, self.publish_battery_level)
        self.battery_level = 100.0

    def publish_battery_level(self):
        # Simulate battery drain
        self.battery_level = max(0.0, self.battery_level - 0.1)

        # Publish battery level
        battery_msg = Float32()
        battery_msg.data = self.battery_level
        self.battery_pub.publish(battery_msg)

        # Check for low battery
        if self.battery_level < 20.0:
            warning_msg = String()
            warning_msg.data = f'LOW BATTERY: {self.battery_level:.1f}% remaining'
            self.warning_pub.publish(warning_msg)
            self.get_logger().warn(warning_msg.data)

        self.get_logger().info(f'Battery level: {self.battery_level:.1f}%')

def main(args=None):
    rclpy.init(args=args)
    battery_monitor = BatteryMonitor()

    try:
        rclpy.spin(battery_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        battery_monitor.destroy_node()
        rclpy.shutdown()
```

## Summary

This week, you've learned the fundamental architecture of ROS 2 and created your first nodes. You now understand:
- The core components of ROS 2 architecture
- How to create publishers and subscribers
- How to use QoS settings for different communication needs
- How to work with parameters and launch files
- How to debug ROS 2 applications

In the next week, we'll dive deeper into topics, services, and actions, exploring more complex communication patterns in ROS 2.