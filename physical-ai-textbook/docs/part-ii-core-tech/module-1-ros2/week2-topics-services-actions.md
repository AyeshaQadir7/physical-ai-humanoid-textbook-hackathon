---
title: Week 2 - Topics, Services, Actions & Message Passing
description: Deep dive into ROS 2 communication patterns and message passing
hide_table_of_contents: false
---

# Week 2: Topics, Services, Actions & Message Passing

## Learning Objectives

By the end of this week, you will be able to:
- Implement publish-subscribe communication patterns for sensor and actuator coordination
- Use services for request-response interactions between robot components
- Implement actions for goal-oriented tasks with feedback and preemption
- Design efficient message passing systems for robot applications

## Overview of ROS 2 Communication Patterns

ROS 2 provides three primary communication patterns for inter-node communication:

### 1. Topics (Publish/Subscribe)
- **Pattern**: One-to-many, asynchronous communication
- **Use Case**: Streaming data like sensor readings, robot status
- **Characteristics**: Fire-and-forget, no acknowledgment

### 2. Services (Request/Response)
- **Pattern**: One-to-one, synchronous communication
- **Use Case**: Configuration requests, one-time computations
- **Characteristics**: Request-response with acknowledgment

### 3. Actions (Goal-Based)
- **Pattern**: One-to-one, asynchronous with feedback
- **Use Case**: Long-running tasks with progress tracking
- **Characteristics**: Goal, feedback, result, preemption

## Topics: Publish/Subscribe Pattern

### Understanding Topics

Topics enable asynchronous, one-way communication between nodes. Publishers send messages to a topic, and subscribers receive messages from the same topic. This pattern is ideal for streaming data like sensor readings, robot status, or control commands.

```
Publisher Node                    Subscriber Node
     │                                  │
     │     ┌─────────────────┐          │
     │     │   Topic: /scan  │          │
     │     └─────────┬───────┘          │
     │               │                  │
     │         ┌─────▼─────┐            │
     │         │   DDS     │            │
     │         │ (DDS/RMW) │            │
     │         └─────┬─────┘            │
     │               │                  │
     │               │  ┌───────────────▼─────────┐
     └───────────────┼──► Subscribe to /scan      │
                       │   Process laser scan data │
                       └───────────────────────────┘
```

### Creating Publishers and Subscribers

Let's create a more sophisticated example with custom messages:

First, create a custom message definition. In your package, create a `msg` directory:

```
robot_basics/
├── msg/
│   └── RobotPose.msg
├── srv/
│   └── NavigateTo.srv
├── action/
│   └── Navigate.action
└── ...
```

Create `msg/RobotPose.msg`:
```
float64 x
float64 y
float64 theta
string frame_id
```

### Advanced Publisher Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from robot_basics.msg import RobotPose  # Custom message
import math
import numpy as np

class RobotSensorPublisher(Node):
    def __init__(self):
        super().__init__('robot_sensor_publisher')

        # Publishers for different sensor types
        self.odom_publisher = self.create_publisher(RobotPose, 'robot_pose', 10)
        self.scan_publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for sensor simulation
        self.timer = self.create_timer(0.1, self.publish_sensors)

        # Robot state simulation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.velocity = 0.0
        self.angular_velocity = 0.0

        self.get_logger().info('Robot Sensor Publisher started')

    def publish_sensors(self):
        # Update robot pose based on current velocity
        dt = 0.1  # 10 Hz
        self.x += self.velocity * math.cos(self.theta) * dt
        self.y += self.velocity * math.sin(self.theta) * dt
        self.theta += self.angular_velocity * dt

        # Publish robot pose
        pose_msg = RobotPose()
        pose_msg.x = self.x
        pose_msg.y = self.y
        pose_msg.theta = self.theta
        pose_msg.frame_id = 'odom'
        self.odom_publisher.publish(pose_msg)

        # Publish simulated laser scan
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_frame'
        scan_msg.angle_min = -math.pi / 2
        scan_msg.angle_max = math.pi / 2
        scan_msg.angle_increment = math.pi / 180  # 1 degree
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0

        # Simulate some obstacles
        num_ranges = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
        ranges = []
        for i in range(num_ranges):
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            # Simulate obstacles at certain angles
            distance = 2.0 + 0.5 * math.sin(5 * angle)  # Add some variation
            ranges.append(distance)

        scan_msg.ranges = ranges
        self.scan_publisher.publish(scan_msg)

        self.get_logger().info(f'Published pose: ({self.x:.2f}, {self.y:.2f}, {self.theta:.2f})')

def main(args=None):
    rclpy.init(args=args)
    robot_sensor_publisher = RobotSensorPublisher()

    try:
        rclpy.spin(robot_sensor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        robot_sensor_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced Subscriber Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from robot_basics.msg import RobotPose
import math

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Subscribers
        self.scan_subscriber = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.pose_subscriber = self.create_subscription(
            RobotPose, 'robot_pose', self.pose_callback, 10)

        # Publisher
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Robot state
        self.scan_data = None
        self.pose_data = None
        self.obstacle_distance = float('inf')
        self.obstacle_angle = 0.0

        self.get_logger().info('Obstacle Avoidance Node started')

    def scan_callback(self, msg):
        # Process laser scan to detect obstacles
        if len(msg.ranges) > 0:
            # Find the closest obstacle in front (±30 degrees)
            center_idx = len(msg.ranges) // 2
            front_ranges = msg.ranges[center_idx-15:center_idx+15]
            valid_ranges = [r for r in front_ranges if not math.isnan(r) and r > 0]

            if valid_ranges:
                self.obstacle_distance = min(valid_ranges)
                min_idx = front_ranges.index(self.obstacle_distance)
                self.obstacle_angle = msg.angle_min + (center_idx-15+min_idx) * msg.angle_increment
            else:
                self.obstacle_distance = float('inf')

    def pose_callback(self, msg):
        self.pose_data = msg

    def control_loop(self):
        cmd = Twist()

        if self.obstacle_distance < 1.0:  # Obstacle within 1 meter
            # Avoid obstacle
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5 if self.obstacle_angle > 0 else -0.5
            self.get_logger().info(f'Obstacle at {self.obstacle_distance:.2f}m, turning away')
        else:
            # Move forward
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
            self.get_logger().info(f'Moving forward, obstacle at {self.obstacle_distance:.2f}m')

        self.cmd_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()

    # Run control loop at 10 Hz
    timer = obstacle_avoidance_node.create_timer(0.1, obstacle_avoidance_node.control_loop)

    try:
        rclpy.spin(obstacle_avoidance_node)
    except KeyboardInterrupt:
        pass
    finally:
        obstacle_avoidance_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services: Request/Response Pattern

### Understanding Services

Services provide synchronous, request-response communication between nodes. A service client sends a request to a service server, which processes the request and sends back a response. This pattern is ideal for one-time operations like configuration changes, computations, or state queries.

```
Service Client                    Service Server
      │                                │
      │    Request: Get Robot State    │
      │ ──────────────────────────────►│
      │                                │
      │                                │─┐ Process Request
      │                                │ │
      │                                │◄┘
      │                                │
      │◄────────────────────────────── │
      │   Response: Robot State Data   │
```

### Creating a Service Definition

Create `srv/NavigateTo.srv`:
```
# Request
float64 x
float64 y
string frame_id

---
# Response
bool success
string message
float64 distance_traveled
```

### Service Server Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_basics.srv import NavigateTo
from geometry_msgs.msg import Twist
import math

class NavigationServer(Node):
    def __init__(self):
        super().__init__('navigation_server')

        # Create service server
        self.srv = self.create_service(
            NavigateTo, 'navigate_to', self.navigate_to_callback)

        # Publisher for movement commands
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Current position (simulated)
        self.current_x = 0.0
        self.current_y = 0.0

        self.get_logger().info('Navigation Server started')

    def navigate_to_callback(self, request, response):
        self.get_logger().info(f'Received navigation request to ({request.x}, {request.y})')

        # Calculate distance to target
        dx = request.x - self.current_x
        dy = request.y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)

        if distance < 0.1:  # Already at target
            response.success = True
            response.message = 'Already at target location'
            response.distance_traveled = 0.0
            return response

        # Move toward target (simplified)
        target_x = request.x
        target_y = request.y

        # In a real implementation, this would be a more complex navigation algorithm
        # For this example, we'll just report success immediately
        self.current_x = target_x
        self.current_y = target_y

        response.success = True
        response.message = f'Reached target ({target_x}, {target_y})'
        response.distance_traveled = distance

        self.get_logger().info(f'Navigation completed: {response.message}')
        return response

def main(args=None):
    rclpy.init(args=args)
    navigation_server = NavigationServer()

    try:
        rclpy.spin(navigation_server)
    except KeyboardInterrupt:
        pass
    finally:
        navigation_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_basics.srv import NavigateTo

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')

        # Create client
        self.client = self.create_client(NavigateTo, 'navigate_to')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = NavigateTo.Request()

        # Request navigation to a specific point
        self.req.x = 5.0
        self.req.y = 3.0
        self.req.frame_id = 'map'

        self.get_logger().info('Sending navigation request')

    def send_request(self):
        self.future = self.client.call_async(self.req)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f'Response received: Success={response.success}, '
                f'Message="{response.message}", Distance={response.distance_traveled:.2f}m')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    navigation_client = NavigationClient()

    # Send the request
    navigation_client.send_request()

    try:
        rclpy.spin(navigation_client)
    except KeyboardInterrupt:
        pass
    finally:
        navigation_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions: Goal-Based Communication

### Understanding Actions

Actions provide a goal-based communication pattern for long-running tasks that require feedback and the ability to be preempted. They are ideal for navigation, manipulation, and other complex robot behaviors.

```
Action Client                    Action Server
     │                                │
     │    Goal: Navigate to (x,y)     │
     │ ──────────────────────────────►│
     │                                │
     │                                │─┐ Start Navigation
     │                                │ │
     │                                │◄┘
     │                                │
     │◄────────────────────────────── │
     │    Feedback: 30% complete      │
     │                                │
     │                                │─┐ Continue Processing
     │                                │ │
     │                                │◄┘
     │                                │
     │◄────────────────────────────── │
     │    Feedback: 60% complete      │
     │                                │
     │                                │─┐ Check for Preemption
     │                                │ │
     │                                │◄┘
     │                                │
     │◄────────────────────────────── │
     │    Result: Success/Failed      │
```

### Creating an Action Definition

Create `action/Navigate.action`:
```
# Goal
float64 x
float64 y
string frame_id

---
# Result
bool success
string message
float64 distance_traveled
int32 path_points

---
# Feedback
float64 distance_to_goal
float64 progress_percentage
string current_status
```

### Action Server Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from robot_basics.action import Navigate
from geometry_msgs.msg import Twist
import math
import time

class NavigateActionServer(Node):
    def __init__(self):
        super().__init__('navigate_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            Navigate,
            'navigate',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        # Publisher for movement commands
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Current position (simulated)
        self.current_x = 0.0
        self.current_y = 0.0

        self.get_logger().info('Navigate Action Server started')

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')

        # Get goal parameters
        goal = goal_handle.request
        target_x = goal.x
        target_y = goal.y

        # Calculate initial distance
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        initial_distance = math.sqrt(dx*dx + dy*dy)
        remaining_distance = initial_distance

        # Initialize feedback
        feedback_msg = Navigate.Feedback()
        feedback_msg.distance_to_goal = remaining_distance
        feedback_msg.progress_percentage = 0.0
        feedback_msg.current_status = 'Moving toward target'

        # Navigation parameters
        linear_speed = 0.3  # m/s
        angular_speed = 0.5  # rad/s

        # Navigation loop
        while remaining_distance > 0.1 and not goal_handle.is_cancel_requested:
            # Calculate direction to target
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - self.current_x  # Simplified for example

            # Create and publish command
            cmd = Twist()
            cmd.linear.x = min(linear_speed, remaining_distance)  # Slow down when close
            cmd.angular.z = angular_speed if angle_diff > 0.1 else -angular_speed if angle_diff < -0.1 else 0.0
            self.cmd_publisher.publish(cmd)

            # Update position (simulated)
            self.current_x += cmd.linear.x * 0.1 * math.cos(self.current_x)  # Simplified
            self.current_y += cmd.linear.x * 0.1 * math.sin(self.current_x)  # Simplified

            # Update remaining distance
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            remaining_distance = math.sqrt(dx*dx + dy*dy)

            # Calculate progress
            progress = max(0.0, ((initial_distance - remaining_distance) / initial_distance) * 100)

            # Update feedback
            feedback_msg.distance_to_goal = remaining_distance
            feedback_msg.progress_percentage = progress
            feedback_msg.current_status = f'Moving, {progress:.1f}% complete'

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(f'Progress: {progress:.1f}%, Distance: {remaining_distance:.2f}m')

            # Sleep to control loop rate
            time.sleep(0.1)

        # Check if goal was canceled
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')

            # Stop the robot
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_publisher.publish(cmd)

            result = Navigate.Result()
            result.success = False
            result.message = 'Goal canceled'
            result.distance_traveled = initial_distance - remaining_distance
            result.path_points = 0  # Simplified
            return result

        # Goal completed successfully
        goal_handle.succeed()

        # Stop the robot
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_publisher.publish(cmd)

        result = Navigate.Result()
        result.success = True
        result.message = f'Reached target ({target_x}, {target_y})'
        result.distance_traveled = initial_distance
        result.path_points = int(initial_distance / 0.1)  # Simplified

        self.get_logger().info(f'Goal succeeded: {result.message}')
        return result

def main(args=None):
    rclpy.init(args=args)
    navigate_action_server = NavigateActionServer()

    try:
        rclpy.spin(navigate_action_server)
    except KeyboardInterrupt:
        pass
    finally:
        navigate_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Action Client Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_basics.action import Navigate

class NavigateActionClient(Node):
    def __init__(self):
        super().__init__('navigate_action_client')

        # Create action client
        self._action_client = ActionClient(
            self,
            Navigate,
            'navigate')

    def send_goal(self, x, y):
        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create the goal
        goal_msg = Navigate.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.frame_id = 'map'

        # Send the goal
        self.get_logger().info(f'Sending goal to ({x}, {y})')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback received: {feedback.current_status}, '
            f'Distance to goal: {feedback.distance_to_goal:.2f}m, '
            f'Progress: {feedback.progress_percentage:.1f}%')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.message}, '
                              f'Success: {result.success}, '
                              f'Distance traveled: {result.distance_traveled:.2f}m')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = NavigateActionClient()

    # Send a goal
    action_client.send_goal(5.0, 3.0)

    # Spin to process callbacks
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

## Quality of Service (QoS) for Different Communication Patterns

### Topics QoS Settings

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# For sensor data (frequent updates, may drop some)
sensor_qos = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)

# For critical control commands (must be reliable)
control_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)

# For configuration data (keep last value)
config_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST
)

# Usage
self.sensor_publisher = self.create_publisher(LaserScan, 'scan', sensor_qos)
self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', control_qos)
```

### Services and Actions QoS

Services and actions use the default QoS settings, but you can customize them:

```python
from rclpy.qos import qos_profile_services_default

# For services
service_qos = qos_profile_services_default
self.srv = self.create_service(NavigateTo, 'navigate_to', callback, qos_profile=service_qos)
```

## Message Design Best Practices

### 1. Efficient Message Structure
- Use appropriate data types (int8 instead of int32 for small values)
- Consider message size for bandwidth-constrained environments
- Group related data in custom messages

### 2. Standard Message Types
- Use standard message types when possible (`std_msgs`, `geometry_msgs`, etc.)
- Create custom messages for robot-specific data
- Follow naming conventions and documentation standards

### 3. Error Handling
- Include status fields in custom messages
- Design messages with extensibility in mind
- Consider backward compatibility

## Lab Exercise: Implement a Complete Navigation System

### Objective
Create a complete navigation system using all three communication patterns:
1. Topics for sensor data and robot commands
2. Service for setting navigation goals
3. Action for long-running navigation tasks

### Requirements
1. Create a sensor simulator that publishes laser scan and odometry data
2. Create a navigation server that can handle both service requests and action goals
3. Create a client that can use both service and action interfaces
4. Implement obstacle avoidance in the navigation system

### Solution Structure

```python
# navigation_system.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from robot_basics.srv import NavigateTo
from robot_basics.action import Navigate
import math

class NavigationSystem(Node):
    def __init__(self):
        super().__init__('navigation_system')

        # Publishers
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, 'nav_status', 10)

        # Subscribers
        self.scan_subscriber = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        # Service server
        self.nav_service = self.create_service(
            NavigateTo, 'navigate_to', self.navigate_to_callback)

        # Action server
        self.nav_action_server = ActionServer(
            self,
            Navigate,
            'navigate',
            execute_callback=self.execute_navigate_action,
            goal_callback=self.goal_callback)

        # Robot state
        self.scan_data = None
        self.current_x = 0.0
        self.current_y = 0.0

        self.get_logger().info('Navigation System initialized')

    def scan_callback(self, msg):
        self.scan_data = msg

    def navigate_to_callback(self, request, response):
        self.get_logger().info(f'Received service request to ({request.x}, {request.y})')

        # Calculate distance
        dx = request.x - self.current_x
        dy = request.y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)

        # For this example, just acknowledge the request
        response.success = True
        response.message = f'Starting navigation to ({request.x}, {request.y})'
        response.distance_traveled = distance

        # Publish status
        status_msg = String()
        status_msg.data = f'Navigating to ({request.x}, {request.y})'
        self.status_publisher.publish(status_msg)

        return response

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    async def execute_navigate_action(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(f'Executing action goal to ({goal.x}, {goal.y})')

        # Navigation logic would go here
        # This is a simplified version
        cmd = Twist()
        cmd.linear.x = 0.3
        cmd.angular.z = 0.0
        self.cmd_publisher.publish(cmd)

        # Wait a bit for demonstration
        import time
        for i in range(10):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                cmd.linear.x = 0.0
                self.cmd_publisher.publish(cmd)

                result = Navigate.Result()
                result.success = False
                result.message = 'Goal canceled'
                result.distance_traveled = 0.0
                result.path_points = 0
                return result

            # Publish feedback
            feedback = Navigate.Feedback()
            feedback.distance_to_goal = 10.0 - i
            feedback.progress_percentage = (i / 10.0) * 100
            feedback.current_status = f'In progress: {i}/10'
            goal_handle.publish_feedback(feedback)

            time.sleep(0.5)

        goal_handle.succeed()
        cmd.linear.x = 0.0
        self.cmd_publisher.publish(cmd)

        result = Navigate.Result()
        result.success = True
        result.message = f'Reached ({goal.x}, {goal.y})'
        result.distance_traveled = 10.0
        result.path_points = 10
        return result

def main(args=None):
    rclpy.init(args=args)
    navigation_system = NavigationSystem()

    try:
        rclpy.spin(navigation_system)
    except KeyboardInterrupt:
        pass
    finally:
        navigation_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

This week, you've learned about the three primary communication patterns in ROS 2:
- **Topics**: For streaming data using publish/subscribe pattern
- **Services**: For request/response interactions
- **Actions**: For goal-based communication with feedback

You now understand how to:
- Create custom messages, services, and actions
- Implement publishers, subscribers, servers, and clients
- Use appropriate QoS settings for different communication needs
- Design efficient message passing systems for robot applications

These communication patterns form the foundation of distributed robotics systems and will be essential as we move forward with more complex robot applications in the coming weeks.