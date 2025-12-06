# Week 2 Lab Exercise: ROS 2 Core Concepts and Communication Patterns

## Overview

In this lab exercise, you will implement the core ROS 2 concepts learned in the theory section. You will create custom message types, implement publisher/subscriber nodes with different Quality of Service (QoS) settings, create service and action servers, and work with TF transforms. This builds upon your Week 1 foundation to create more sophisticated robot communication systems.

## Prerequisites

- Completion of Week 1 lab exercises
- Working ROS 2 Humble Hawksbill installation
- Working Week 1 ROS 2 workspace
- Basic understanding of Python or C++ for ROS 2 development

## Learning Objectives

By completing this lab, you will:
- Create custom message types for ROS 2 communication
- Implement publisher and subscriber nodes with different QoS settings
- Create service servers and clients for synchronous communication
- Implement action servers and clients for long-running tasks
- Work with TF transforms to manage coordinate frames
- Create launch files to coordinate multiple nodes
- Use ROS 2 command line tools for debugging and monitoring

## Task 1: Create Custom Message Types

### Step 1.1: Create a custom message package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake custom_messages
```

### Step 1.2: Create a custom message definition
Create the directory structure and message file:

```bash
mkdir -p ~/ros2_ws/src/custom_messages/msg
```

Create `~/ros2_ws/src/custom_messages/msg/RobotStatus.msg`:
```
# RobotStatus.msg
string robot_name
int32 battery_level
bool is_moving
float64[] joint_positions
float64[] joint_velocities
```

### Step 1.3: Create a custom service definition
Create `~/ros2_ws/src/custom_messages/srv/MoveRobot.srv`:
```
# MoveRobot.srv
float64 x
float64 y
float64 theta
---
bool success
string message
```

### Step 1.4: Create a custom action definition
Create `~/ros2_ws/src/custom_messages/action/Navigation.action`:
```
# Navigation.action
# Goal definition
float64 x
float64 y
float64 theta

---
# Result definition
bool success
float64 distance_traveled
string message

---
# Feedback definition
float64 distance_to_goal
float64 remaining_time
string status
```

### Step 1.5: Update package.xml
Edit `~/ros2_ws/src/custom_messages/package.xml` to include message generation dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>custom_messages</name>
  <version>0.0.0</version>
  <description>Custom messages for robotics applications</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Step 1.6: Update CMakeLists.txt
Edit `~/ros2_ws/src/custom_messages/CMakeLists.txt` to include message generation:

```cmake
cmake_minimum_required(VERSION 3.8)
project(custom_messages)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define message files
set(msg_files
  "msg/RobotStatus.msg"
)

set(srv_files
  "srv/MoveRobot.srv"
)

set(action_files
  "action/Navigation.action"
)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

ament_package()
```

### Step 1.7: Build the custom messages package
```bash
cd ~/ros2_ws
colcon build --packages-select custom_messages
source install/setup.bash
```

## Task 2: Implement Publisher/Subscriber with QoS Settings

### Step 2.1: Create a new package for QoS examples
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python qos_examples
```

### Step 2.2: Create a publisher with custom QoS settings
Create `~/ros2_ws/src/qos_examples/qos_examples/qos_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from custom_messages.msg import RobotStatus


class QoSPublisher(Node):

    def __init__(self):
        super().__init__('qos_publisher')

        # Create QoS profile with different settings
        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.publisher_ = self.create_publisher(RobotStatus, 'robot_status', qos_profile)

        # Create another publisher with best-effort settings
        best_effort_qos = QoSProfile(
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_ALL
        )
        self.best_effort_publisher = self.create_publisher(String, 'sensor_data', best_effort_qos)

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # Publish robot status message
        msg = RobotStatus()
        msg.robot_name = 'Robot1'
        msg.battery_level = 100 - self.i % 100
        msg.is_moving = self.i % 2 == 0
        msg.joint_positions = [1.0, 2.0, 3.0]
        msg.joint_velocities = [0.1, 0.2, 0.3]

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing robot status: {msg.robot_name}, battery: {msg.battery_level}%')

        # Publish sensor data with best-effort
        sensor_msg = String()
        sensor_msg.data = f'Sensor reading: {self.i}'
        self.best_effort_publisher.publish(sensor_msg)

        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    qos_publisher = QoSPublisher()
    rclpy.spin(qos_publisher)
    qos_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 2.3: Create a subscriber with matching QoS settings
Create `~/ros2_ws/src/qos_examples/qos_examples/qos_subscriber.py`:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from custom_messages.msg import RobotStatus


class QoSSubscriber(Node):

    def __init__(self):
        super().__init__('qos_subscriber')

        # Match the publisher's QoS settings
        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.subscription = self.create_subscription(
            RobotStatus,
            'robot_status',
            self.robot_status_callback,
            qos_profile
        )

        # Subscribe to sensor data with best-effort settings
        best_effort_qos = QoSProfile(
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_ALL
        )

        self.sensor_subscription = self.create_subscription(
            String,
            'sensor_data',
            self.sensor_callback,
            best_effort_qos
        )

    def robot_status_callback(self, msg):
        self.get_logger().info(
            f'Robot: {msg.robot_name}, Battery: {msg.battery_level}%, '
            f'Moving: {msg.is_moving}, Joints: {msg.joint_positions}'
        )

    def sensor_callback(self, msg):
        self.get_logger().info(f'Sensor data: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    qos_subscriber = QoSSubscriber()
    rclpy.spin(qos_subscriber)
    qos_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 2.4: Update setup.py for qos_examples package
Edit `~/ros2_ws/src/qos_examples/setup.py` to add entry points:

```python
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'qos_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Examples of QoS settings in ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qos_publisher = qos_examples.qos_publisher:main',
            'qos_subscriber = qos_examples.qos_subscriber:main',
        ],
    },
)
```

## Task 3: Create Service Server and Client

### Step 3.1: Create service server
Create `~/ros2_ws/src/qos_examples/qos_examples/move_robot_server.py`:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from custom_messages.srv import MoveRobot


class MoveRobotService(Node):

    def __init__(self):
        super().__init__('move_robot_service')
        self.srv = self.create_service(
            MoveRobot,
            'move_robot',
            self.move_robot_callback
        )
        self.get_logger().info('Move Robot service is ready.')

    def move_robot_callback(self, request, response):
        self.get_logger().info(
            f'Received request to move to x: {request.x}, y: {request.y}, theta: {request.theta}'
        )

        # Simulate robot movement (in a real robot, this would control actual hardware)
        # For simulation, we'll just return success
        response.success = True
        response.message = f'Moved to position ({request.x}, {request.y}, {request.theta})'

        return response


def main(args=None):
    rclpy.init(args=args)
    move_robot_service = MoveRobotService()

    try:
        rclpy.spin(move_robot_service)
    except KeyboardInterrupt:
        pass
    finally:
        move_robot_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 3.2: Create service client
Create `~/ros2_ws/src/qos_examples/qos_examples/move_robot_client.py`:

```python
import sys
import rclpy
from rclpy.node import Node
from custom_messages.srv import MoveRobot


class MoveRobotClient(Node):

    def __init__(self):
        super().__init__('move_robot_client')
        self.cli = self.create_client(MoveRobot, 'move_robot')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = MoveRobot.Request()

    def send_request(self, x, y, theta):
        self.req.x = x
        self.req.y = y
        self.req.theta = theta
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    # Get parameters from command line
    if len(sys.argv) != 4:
        print('Usage: ros2 run qos_examples move_robot_client <x> <y> <theta>')
        return

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    theta = float(sys.argv[3])

    move_robot_client = MoveRobotClient()
    response = move_robot_client.send_request(x, y, theta)

    if response:
        move_robot_client.get_logger().info(
            f'Result: {response.success}, Message: {response.message}'
        )
    else:
        move_robot_client.get_logger().info('Service call failed')

    move_robot_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 3.3: Update setup.py to include service nodes
Add to the entry_points section in `~/ros2_ws/src/qos_examples/setup.py`:

```python
entry_points={
    'console_scripts': [
        'qos_publisher = qos_examples.qos_publisher:main',
        'qos_subscriber = qos_examples.qos_subscriber:main',
        'move_robot_server = qos_examples.move_robot_server:main',
        'move_robot_client = qos_examples.move_robot_client:main',
    ],
},
```

## Task 4: Create Action Server and Client

### Step 4.1: Create action server
Create `~/ros2_ws/src/qos_examples/qos_examples/navigation_action_server.py`:

```python
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from custom_messages.action import Navigation


class NavigationActionServer(Node):

    def __init__(self):
        super().__init__('navigation_action_server')
        self._action_server = ActionServer(
            self,
            Navigation,
            'navigate_to_pose',
            self.execute_callback)

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received navigation goal')

        # Get goal parameters
        goal = goal_handle.request
        x = goal.x
        y = goal.y
        theta = goal.theta

        # Simulate navigation process
        feedback_msg = Navigation.Feedback()
        result = Navigation.Result()

        # Simulate progress (in a real robot, this would be actual navigation)
        for i in range(10):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Goal canceled'
                return result

            # Update feedback
            feedback_msg.distance_to_goal = 10.0 - i
            feedback_msg.remaining_time = 10.0 - i
            feedback_msg.status = f'Navigating... {i*10}% complete'

            self.get_logger().info(f'Feedback: {feedback_msg.status}')
            goal_handle.publish_feedback(feedback_msg)

            # Simulate time delay
            time.sleep(0.5)

        # Navigation completed successfully
        goal_handle.succeed()
        result.success = True
        result.distance_traveled = 10.0
        result.message = f'Navigated to ({x}, {y}, {theta}) successfully'

        self.get_logger().info(f'Result: {result.message}')
        return result


def main(args=None):
    rclpy.init(args=args)
    navigation_action_server = NavigationActionServer()

    try:
        rclpy.spin(navigation_action_server)
    except KeyboardInterrupt:
        pass
    finally:
        navigation_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 4.2: Create action client
Create `~/ros2_ws/src/qos_examples/qos_examples/navigation_action_client.py`:

```python
import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from custom_messages.action import Navigation


class NavigationActionClient(Node):

    def __init__(self):
        super().__init__('navigation_action_client')
        self._action_client = ActionClient(
            self,
            Navigation,
            'navigate_to_pose')

    def send_goal(self, x, y, theta):
        goal_msg = Navigation.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.theta = theta

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Received feedback: {feedback.status} - '
            f'Distance to goal: {feedback.distance_to_goal:.2f}, '
            f'Remaining time: {feedback.remaining_time:.2f}'
        )

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.message}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    # Get parameters from command line
    if len(sys.argv) != 4:
        print('Usage: ros2 run qos_examples navigation_action_client <x> <y> <theta>')
        return

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    theta = float(sys.argv[3])

    action_client = NavigationActionClient()
    action_client.send_goal(x, y, theta)

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
```

### Step 4.3: Update setup.py to include action nodes
Add to the entry_points section in `~/ros2_ws/src/qos_examples/setup.py`:

```python
entry_points={
    'console_scripts': [
        'qos_publisher = qos_examples.qos_publisher:main',
        'qos_subscriber = qos_examples.qos_subscriber:main',
        'move_robot_server = qos_examples.move_robot_server:main',
        'move_robot_client = qos_examples.move_robot_client:main',
        'navigation_action_server = qos_examples.navigation_action_server:main',
        'navigation_action_client = qos_examples.navigation_action_client:main',
    ],
},
```

## Task 5: Work with TF Transforms

### Step 5.1: Create TF broadcaster
Create `~/ros2_ws/src/qos_examples/qos_examples/tf_broadcaster.py`:

```python
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float64


class StaticFramePublisher(Node):

    def __init__(self):
        super().__init__('static_turtle_tf2_broadcaster')

        # Declare and acquire `turtlename` parameter
        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a timer to broadcast transforms
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

        # Initialize position for dynamic transform
        self.time = 0.0

    def broadcast_timer_callback(self):
        # Create and populate the transform message
        t = TransformStamped()

        # Set header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.robot_name + '/base_link'

        # Set transform values (dynamic movement)
        t.transform.translation.x = math.sin(self.time)
        t.transform.translation.y = math.cos(self.time)
        t.transform.translation.z = 0.0

        # Simple rotation around Z-axis
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.time / 2.0)
        t.transform.rotation.w = math.cos(self.time / 2.0)

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

        # Increment time for animation
        self.time += 0.1


def main(args=None):
    rclpy.init(args=args)
    static_frame_publisher = StaticFramePublisher()

    try:
        rclpy.spin(static_frame_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        static_frame_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 5.2: Create TF listener
Create `~/ros2_ws/src/qos_examples/qos_examples/tf_listener.py`:

```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String


class FrameListener(Node):

    def __init__(self):
        super().__init__('frame_listener')

        self.declare_parameter('target_frame', 'robot1/base_link')
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create publisher for transform results
        self.publisher = self.create_publisher(String, 'robot_position', 1)

        # Create timer to check transforms
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        from_frame_rel = 'world'
        to_frame_rel = self.target_frame

        try:
            # Look up the transform between the two frames
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())

            # Format and publish the position
            position_str = (
                f'Position of {self.target_frame} in world frame: '
                f'x={t.transform.translation.x:.2f}, '
                f'y={t.transform.translation.y:.2f}, '
                f'z={t.transform.translation.z:.2f}'
            )

            msg = String()
            msg.data = position_str
            self.publisher.publish(msg)
            self.get_logger().info(position_str)

        except TransformException as ex:
            self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')


def main(args=None):
    rclpy.init(args=args)
    frame_listener = FrameListener()

    try:
        rclpy.spin(frame_listener)
    except KeyboardInterrupt:
        pass
    finally:
        frame_listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 5.3: Update setup.py to include TF nodes
Add to the entry_points section in `~/ros2_ws/src/qos_examples/setup.py`:

```python
entry_points={
    'console_scripts': [
        'qos_publisher = qos_examples.qos_publisher:main',
        'qos_subscriber = qos_examples.qos_subscriber:main',
        'move_robot_server = qos_examples.move_robot_server:main',
        'move_robot_client = qos_examples.move_robot_client:main',
        'navigation_action_server = qos_examples.navigation_action_server:main',
        'navigation_action_client = qos_examples.navigation_action_client:main',
        'tf_broadcaster = qos_examples.tf_broadcaster:main',
        'tf_listener = qos_examples.tf_listener:main',
    ],
},
```

## Task 6: Create Launch Files

### Step 6.1: Create a launch file for the entire system
Create the launch directory and file:

```bash
mkdir -p ~/ros2_ws/src/qos_examples/launch
```

Create `~/ros2_ws/src/qos_examples/launch/robot_system.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('qos_examples')

    return LaunchDescription([
        # QoS publisher and subscriber
        Node(
            package='qos_examples',
            executable='qos_publisher',
            name='qos_publisher',
            output='screen'
        ),
        Node(
            package='qos_examples',
            executable='qos_subscriber',
            name='qos_subscriber',
            output='screen'
        ),

        # Service server and client (the client would need to be run separately)
        Node(
            package='qos_examples',
            executable='move_robot_server',
            name='move_robot_server',
            output='screen'
        ),

        # Action server
        Node(
            package='qos_examples',
            executable='navigation_action_server',
            name='navigation_action_server',
            output='screen'
        ),

        # TF broadcaster and listener
        Node(
            package='qos_examples',
            executable='tf_broadcaster',
            name='tf_broadcaster',
            parameters=[
                {'robot_name': 'turtle1'}
            ],
            output='screen'
        ),
        Node(
            package='qos_examples',
            executable='tf_listener',
            name='tf_listener',
            parameters=[
                {'target_frame': 'turtle1/base_link'}
            ],
            output='screen'
        ),
    ])
```

## Task 7: Build and Test the System

### Step 7.1: Build all packages
```bash
cd ~/ros2_ws
colcon build --packages-select custom_messages qos_examples
source install/setup.bash
```

### Step 7.2: Test the launch file
```bash
ros2 launch qos_examples robot_system.launch.py
```

### Step 7.3: Test the service client in a separate terminal
```bash
source ~/ros2_ws/install/setup.bash
ros2 run qos_examples move_robot_client 1.0 2.0 3.14
```

### Step 7.4: Test the action client in a separate terminal
```bash
source ~/ros2_ws/install/setup.bash
ros2 run qos_examples navigation_action_client 5.0 5.0 1.57
```

## Task 8: Use ROS 2 Command Line Tools

### Step 8.1: Explore your system
```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# List all services
ros2 service list

# List all actions
ros2 action list

# Check the details of a specific topic
ros2 topic info /robot_status

# Check the details of a specific service
ros2 service info /move_robot
```

### Step 8.2: Monitor topics
```bash
# Echo messages from the robot status topic
ros2 topic echo /robot_status

# Echo messages from the sensor data topic
ros2 topic echo /sensor_data
```

### Step 8.3: Call services directly
```bash
# Call the move robot service directly
ros2 service call /move_robot custom_messages/srv/MoveRobot "{x: 1.0, y: 2.0, theta: 3.14}"
```

## Assessment Questions

1. Explain the differences between ROS 2 topics, services, and actions. When would you use each communication pattern?

2. What are Quality of Service (QoS) settings in ROS 2? Describe the different QoS policies and when to use them.

3. How do transforms (TF) work in ROS 2? What is the purpose of the TF tree?

4. What is the purpose of launch files? How do they improve the organization of ROS 2 systems?

5. Compare the reliability of different communication patterns in ROS 2.

## Troubleshooting Tips

- If custom messages don't build, ensure your package.xml and CMakeLists.txt are properly configured
- If nodes can't communicate, check that they're using compatible QoS settings
- If TF transforms aren't working, verify that frame names match exactly
- If launch files fail, check that all package names and executable names are correct
- Use `ros2 doctor` to diagnose common ROS 2 system issues

## Summary

In this lab, you've implemented all the core ROS 2 communication patterns:
- Custom message types for specialized data
- Publisher/subscriber with different QoS settings
- Services for synchronous request/response
- Actions for long-running tasks with feedback
- TF for coordinate frame management
- Launch files for system coordination

These components form the foundation for all complex robotics applications and will be used throughout the remainder of this textbook.