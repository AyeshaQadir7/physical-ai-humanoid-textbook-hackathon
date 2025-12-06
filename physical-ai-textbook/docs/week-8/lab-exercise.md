# Week 8 Lab Exercise: Integration & Deployment

## Overview

In this lab exercise, you will integrate all the components learned in previous weeks into a complete robotic system and deploy it in a simulated real-world scenario. You will combine perception, control, navigation, multi-robot coordination, and machine learning systems into a cohesive application. This serves as a capstone experience, bringing together all the knowledge from the previous weeks to create deployable robotic solutions.

## Prerequisites

- Completion of Week 1-7 lab exercises
- Working ROS 2 Humble Hawksbill installation
- Complete Week 1-7 ROS 2 workspace with all packages
- Robot simulation environment (Gazebo with TurtleBot3 or similar)
- Understanding of all previous week topics
- Experience with ROS 2 launch files and system composition

## Learning Objectives

By completing this lab, you will:
- Integrate multiple robotic subsystems into cohesive applications
- Design and implement complete robotic workflows
- Deploy robotic systems in simulated real-world environments
- Optimize system performance and resource utilization
- Implement monitoring, logging, and debugging strategies
- Evaluate system reliability and maintainability

## Exercise 1: Complete System Architecture (2 hours)

### Task 1.1: System Design and Planning
1. Design a complete robotic system architecture for a warehouse automation scenario:
   - Autonomous mobile robot that navigates to pick up and deliver items
   - Uses perception to detect items and obstacles
   - Employs ML for object recognition
   - Integrates with multi-robot coordination for traffic management

2. Create a system architecture diagram showing:
   - All major components (perception, planning, control, etc.)
   - Communication patterns between components
   - Data flow throughout the system
   - Hardware interfaces

3. Identify the ROS 2 packages needed for each component and how they'll interact.

### Task 1.2: Launch File Integration
1. Create a main launch file that brings up all system components:
   ```python
   # integration_demo/launch/complete_system.launch.py
   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch.substitutions import PathJoinSubstitution, TextSubstitution
   from launch_ros.actions import Node, ComposableNodeContainer
   from launch_ros.descriptions import ComposableNode
   from launch_ros.substitutions import FindPackageShare

   def generate_launch_description():
       # Declare launch arguments
       use_sim_time = DeclareLaunchArgument(
           'use_sim_time',
           default_value='true',
           description='Use simulation clock if true'
       )

       # Include robot simulation
       robot_sim = IncludeLaunchDescription(
           PythonLaunchDescriptionSource([
               PathJoinSubstitution([
                   FindPackageShare('turtlebot3_gazebo'),
                   'launch',
                   'turtlebot3_world.launch.py'
               ])
           ])
       )

       # Perception system
       perception_container = ComposableNodeContainer(
           name='perception_container',
           namespace='',
           package='rclcpp_components',
           executable='component_container',
           composable_node_descriptions=[
               ComposableNode(
                   package='cv_bridge',
                   plugin='image_proc::RectifyNode',
                   name='image_rectifier'
               ),
               ComposableNode(
                   package='object_detection_ros',
                   plugin='ObjectDetectionNode',
                   name='object_detector',
                   parameters=[{'model_path': '/path/to/model'}]
               )
           ]
       )

       # Navigation system
       navigation_system = IncludeLaunchDescription(
           PythonLaunchDescriptionSource([
               PathJoinSubstitution([
                   FindPackageShare('nav2_bringup'),
                   'launch',
                   'navigation_launch.py'
               ])
           ])
       )

       # Custom integration nodes
       integration_nodes = [
           Node(
               package='integration_demo',
               executable='task_manager',
               name='task_manager',
               parameters=[
                   {'use_sim_time': use_sim_time}
               ]
           ),
           Node(
               package='integration_demo',
               executable='system_monitor',
               name='system_monitor',
               parameters=[
                   {'use_sim_time': use_sim_time}
               ]
           )
       ]

       return LaunchDescription([
           use_sim_time,
           robot_sim,
           navigation_system,
           perception_container
       ] + integration_nodes)
   ```

2. Create a comprehensive launch file that starts all necessary components in the right order.

### Task 1.3: System Configuration
1. Create configuration files for different system modes:
   - Development mode (with extra debugging)
   - Performance mode (optimized for speed)
   - Safety mode (with extra safety checks)

2. Implement parameter management for different deployment scenarios.

## Exercise 2: Perception-Action Integration (2 hours)

### Task 2.1: Object Detection Integration
1. Create a node that integrates object detection with navigation:
   ```python
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image, LaserScan
   from geometry_msgs.msg import Twist, PoseStamped
   from nav_msgs.msg import Odometry
   from std_msgs.msg import String
   from cv_bridge import CvBridge
   import cv2
   import numpy as np
   import math

   class PerceptionActionIntegrator(Node):
       def __init__(self):
           super().__init__('perception_action_integrator')

           # Initialize components
           self.bridge = CvBridge()
           self.robot_pose = None
           self.detected_objects = []
           self.current_goal = None

           # Subscriptions
           self.image_sub = self.create_subscription(
               Image, '/camera/image_raw', self.image_callback, 10)
           self.odom_sub = self.create_subscription(
               Odometry, '/odom', self.odom_callback, 10)
           self.scan_sub = self.create_subscription(
               LaserScan, '/scan', self.scan_callback, 10)
           self.goal_sub = self.create_subscription(
               PoseStamped, '/move_base_simple/goal', self.goal_callback, 10)

           # Publishers
           self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
           self.object_pub = self.create_publisher(String, '/detected_objects', 10)

           # Timers
           self.control_timer = self.create_timer(0.1, self.control_loop)

           self.get_logger().info('Perception-Action Integrator Started')

       def odom_callback(self, msg):
           self.robot_pose = msg.pose.pose

       def image_callback(self, msg):
           try:
               cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

               # Simulate object detection (in real implementation, use actual ML model)
               # For this example, detect colored objects using simple color filtering
               hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

               # Detect red objects (simulated objects of interest)
               lower_red = np.array([0, 50, 50])
               upper_red = np.array([10, 255, 255])
               mask1 = cv2.inRange(hsv, lower_red, upper_red)

               lower_red = np.array([170, 50, 50])
               upper_red = np.array([180, 255, 255])
               mask2 = cv2.inRange(hsv, lower_red, upper_red)

               mask = mask1 + mask2
               contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

               detected_objects = []
               for contour in contours:
                   if cv2.contourArea(contour) > 500:  # Filter small detections
                       x, y, w, h = cv2.boundingRect(contour)
                       # Convert image coordinates to world coordinates (simplified)
                       # In real implementation, use proper camera calibration
                       object_info = {
                           'x': self.robot_pose.position.x + (x - 320) * 0.01,  # Approximate conversion
                           'y': self.robot_pose.position.y + (y - 240) * 0.01,
                           'type': 'item',
                           'confidence': 0.8
                       }
                       detected_objects.append(object_info)

               self.detected_objects = detected_objects

               # Publish detected objects
               if detected_objects:
                   obj_str = f"Detected {len(detected_objects)} objects"
                   self.object_pub.publish(String(data=obj_str))

           except Exception as e:
               self.get_logger().error(f'Error in image processing: {str(e)}')

       def scan_callback(self, msg):
           # Process laser scan for obstacle detection
           # In real implementation, this would be more sophisticated
           pass

       def goal_callback(self, msg):
           self.current_goal = msg.pose

       def control_loop(self):
           if self.robot_pose and self.current_goal:
               # Calculate distance to goal
               dx = self.current_goal.position.x - self.robot_pose.position.x
               dy = self.current_goal.position.y - self.robot_pose.position.y
               distance_to_goal = math.sqrt(dx*dx + dy*dy)

               cmd_msg = Twist()

               if distance_to_goal > 0.5:  # Not at goal yet
                   # Navigate toward goal
                   angle_to_goal = math.atan2(dy, dx)
                   current_yaw = self.get_yaw_from_quaternion(self.robot_pose.orientation)

                   angle_diff = angle_to_goal - current_yaw
                   # Normalize angle
                   while angle_diff > math.pi:
                       angle_diff -= 2 * math.pi
                   while angle_diff < -math.pi:
                       angle_diff += 2 * math.pi

                   # Simple proportional controller
                   cmd_msg.linear.x = min(0.5, distance_to_goal * 0.5)
                   cmd_msg.angular.z = angle_diff * 1.5

                   # Check for detected objects that might be the target
                   for obj in self.detected_objects:
                       obj_dist = math.sqrt(
                           (obj['x'] - self.robot_pose.position.x)**2 +
                           (obj['y'] - self.robot_pose.position.y)**2
                       )
                       if obj_dist < 1.0 and obj['type'] == 'item':  # Found target item
                           self.get_logger().info(f'Found target item at ({obj["x"]}, {obj["y"]})')
                           # Stop and perform pickup action
                           cmd_msg.linear.x = 0.0
                           cmd_msg.angular.z = 0.0
                           break
               else:
                   # At goal, look for objects
                   cmd_msg.angular.z = 0.5  # Rotate slowly to look for objects

               self.cmd_vel_pub.publish(cmd_msg)

       def get_yaw_from_quaternion(self, orientation):
           # Convert quaternion to yaw angle
           import math
           siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
           cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
           return math.atan2(siny_cosp, cosy_cosp)

   def main(args=None):
       rclpy.init(args=args)
       integrator = PerceptionActionIntegrator()

       try:
           rclpy.spin(integrator)
       except KeyboardInterrupt:
           pass
       finally:
           integrator.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Create the integration node and add it to your package.

### Task 2.2: Navigation with Perception Feedback
1. Modify the navigation behavior based on perception results:
   - Adjust navigation when objects are detected
   - Implement object-specific behaviors
   - Handle cases where perception affects navigation

## Exercise 3: Multi-Robot Coordination Integration (2 hours)

### Task 3.1: Task Management System
1. Create a task management system that coordinates multiple robots:
   ```python
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String, Int32
   from geometry_msgs.msg import PoseStamped
   from nav_msgs.msg import Odometry
   import json
   import math
   from enum import Enum

   class TaskStatus(Enum):
       PENDING = 1
       ASSIGNED = 2
       IN_PROGRESS = 3
       COMPLETED = 4
       FAILED = 5

   class Task:
       def __init__(self, task_id, task_type, location, priority=1):
           self.task_id = task_id
           self.task_type = task_type  # 'pickup', 'delivery', 'inspection'
           self.location = location
           self.priority = priority
           self.status = TaskStatus.PENDING
           self.assigned_robot = None

   class RobotStatus:
       def __init__(self, robot_id):
           self.robot_id = robot_id
           self.current_pose = None
           self.current_task = None
           self.status = 'IDLE'  # IDLE, MOVING, WORKING

   class TaskManager(Node):
       def __init__(self):
           super().__init__('task_manager')

           self.robot_id = self.declare_parameter('robot_id', 'robot1').value
           self.is_coordinator = self.declare_parameter('is_coordinator', False).value

           self.tasks = []
           self.robots = {}
           self.next_task_id = 1

           if self.is_coordinator:
               # Coordinator manages tasks for all robots
               self.task_sub = self.create_subscription(
                   String, 'new_task_request', self.task_request_callback, 10)
               self.robot_status_sub = self.create_subscription(
                   String, 'robot_status', self.robot_status_callback, 10)
               self.task_assignment_pub = self.create_publisher(
                   String, 'task_assignment', 10)
               self.task_status_pub = self.create_publisher(
                   String, 'task_status', 10)

               # Timer for task allocation
               self.task_allocation_timer = self.create_timer(1.0, self.allocate_tasks)
           else:
               # Regular robot reports status and receives tasks
               self.status_pub = self.create_publisher(String, 'robot_status', 10)
               self.task_assignment_sub = self.create_subscription(
                   String, 'task_assignment', self.task_assignment_callback, 10)
               self.odom_sub = self.create_subscription(
                   Odometry, 'odom', self.odom_callback, 10)

               # Timer for status reporting
               self.status_timer = self.create_timer(2.0, self.report_status)

           self.get_logger().info(f'Task Manager for {self.robot_id} started')

       def odom_callback(self, msg):
           if self.robot_id not in self.robots:
               self.robots[self.robot_id] = RobotStatus(self.robot_id)
           self.robots[self.robot_id].current_pose = msg.pose.pose

       def task_request_callback(self, msg):
           try:
               task_data = json.loads(msg.data)
               new_task = Task(
                   task_id=self.next_task_id,
                   task_type=task_data['type'],
                   location=task_data['location'],
                   priority=task_data.get('priority', 1)
               )
               self.tasks.append(new_task)
               self.next_task_id += 1
               self.get_logger().info(f'Received new task: {new_task.task_id}')
           except Exception as e:
               self.get_logger().error(f'Error processing task request: {str(e)}')

       def robot_status_callback(self, msg):
           try:
               status_data = json.loads(msg.data)
               robot_id = status_data['robot_id']

               if robot_id not in self.robots:
                   self.robots[robot_id] = RobotStatus(robot_id)

               self.robots[robot_id].status = status_data['status']
               if 'pose' in status_data:
                   self.robots[robot_id].current_pose = status_data['pose']
               if 'current_task' in status_data:
                   self.robots[robot_id].current_task = status_data['current_task']

           except Exception as e:
               self.get_logger().error(f'Error processing robot status: {str(e)}')

       def task_assignment_callback(self, msg):
           try:
               assignment_data = json.loads(msg.data)
               if assignment_data['robot_id'] == self.robot_id:
                   task_id = assignment_data['task_id']
                   # Find and accept the assigned task
                   for task in self.tasks:
                       if task.task_id == task_id:
                           task.assigned_robot = self.robot_id
                           task.status = TaskStatus.ASSIGNED
                           self.robots[self.robot_id].current_task = task
                           self.robots[self.robot_id].status = 'MOVING'
                           self.get_logger().info(f'Task {task_id} assigned to {self.robot_id}')
                           break
           except Exception as e:
               self.get_logger().error(f'Error processing task assignment: {str(e)}')

       def allocate_tasks(self):
           if not self.is_coordinator:
               return

           # Simple task allocation algorithm
           available_robots = [r for r in self.robots.values() if r.status == 'IDLE']
           unassigned_tasks = [t for t in self.tasks if t.status == TaskStatus.PENDING]

           # Sort tasks by priority
           unassigned_tasks.sort(key=lambda t: t.priority, reverse=True)

           for task in unassigned_tasks:
               if available_robots:
                   # Assign to closest available robot
                   closest_robot = min(
                       available_robots,
                       key=lambda r: self.calculate_distance(task.location, r.current_pose)
                   )

                   task.assigned_robot = closest_robot.robot_id
                   task.status = TaskStatus.ASSIGNED

                   # Publish assignment
                   assignment_msg = String()
                   assignment_msg.data = json.dumps({
                       'task_id': task.task_id,
                       'robot_id': closest_robot.robot_id,
                       'task_type': task.task_type,
                       'location': task.location
                   })
                   self.task_assignment_pub.publish(assignment_msg)

                   self.get_logger().info(f'Assigned task {task.task_id} to {closest_robot.robot_id}')

                   # Remove robot from available list
                   available_robots.remove(closest_robot)

       def report_status(self):
           if self.is_coordinator:
               return

           status_msg = String()
           status_data = {
               'robot_id': self.robot_id,
               'status': self.robots.get(self.robot_id, RobotStatus(self.robot_id)).status,
               'pose': self.robots.get(self.robot_id, RobotStatus(self.robot_id)).current_pose,
               'current_task': self.robots.get(self.robot_id, RobotStatus(self.robot_id)).current_task
           }
           status_msg.data = json.dumps(status_data)
           self.status_pub.publish(status_msg)

       def calculate_distance(self, loc1, pose2):
           if pose2 is None:
               return float('inf')
           return math.sqrt(
               (loc1['x'] - pose2.position.x)**2 +
               (loc1['y'] - pose2.position.y)**2
           )

   def main(args=None):
       rclpy.init(args=args)
       task_manager = TaskManager()

       try:
           rclpy.spin(task_manager)
       except KeyboardInterrupt:
           pass
       finally:
           task_manager.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Create and test the task management system with multiple simulated robots.

### Task 3.2: Traffic Management
1. Implement collision avoidance between multiple robots:
   - Path coordination to prevent conflicts
   - Priority-based right-of-way
   - Dynamic replanning when conflicts arise

## Exercise 4: System Monitoring and Optimization (1.5 hours)

### Task 4.1: Performance Monitoring
1. Create a system monitoring node:
   ```python
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String, Float32
   from sensor_msgs.msg import BatteryState
   import psutil
   import time
   import json

   class SystemMonitor(Node):
       def __init__(self):
           super().__init__('system_monitor')

           self.status_pub = self.create_publisher(String, 'system_status', 10)
           self.cpu_pub = self.create_publisher(Float32, 'cpu_usage', 10)
           self.memory_pub = self.create_publisher(Float32, 'memory_usage', 10)

           # Timer for system monitoring
           self.monitor_timer = self.create_timer(1.0, self.monitor_system)

           # Performance tracking
           self.performance_log = []
           self.start_time = time.time()

           self.get_logger().info('System Monitor Started')

       def monitor_system(self):
           # Get system metrics
           cpu_percent = psutil.cpu_percent(interval=1)
           memory_percent = psutil.virtual_memory().percent
           disk_percent = psutil.disk_usage('/').percent

           # Create status message
           status_data = {
               'timestamp': time.time(),
               'uptime': time.time() - self.start_time,
               'cpu_percent': cpu_percent,
               'memory_percent': memory_percent,
               'disk_percent': disk_percent,
               'process_count': len(psutil.pids())
           }

           status_msg = String()
           status_msg.data = json.dumps(status_data)
           self.status_pub.publish(status_msg)

           # Publish individual metrics
           cpu_msg = Float32()
           cpu_msg.data = float(cpu_percent)
           self.cpu_pub.publish(cpu_msg)

           memory_msg = Float32()
           memory_msg.data = float(memory_percent)
           self.memory_pub.publish(memory_msg)

           # Log performance data
           self.performance_log.append(status_data)

           self.get_logger().info(f'System: CPU={cpu_percent:.1f}%, Mem={memory_percent:.1f}%')

           # Check for performance issues
           if cpu_percent > 90:
               self.get_logger().warn(f'High CPU usage: {cpu_percent}%')
           if memory_percent > 90:
               self.get_logger().warn(f'High memory usage: {memory_percent}%')

   def main(args=None):
       rclpy.init(args=args)
       monitor = SystemMonitor()

       try:
           rclpy.spin(monitor)
       except KeyboardInterrupt:
           # Print performance summary
           if monitor.performance_log:
               avg_cpu = sum(d['cpu_percent'] for d in monitor.performance_log) / len(monitor.performance_log)
               avg_memory = sum(d['memory_percent'] for d in monitor.performance_log) / len(monitor.performance_log)
               monitor.get_logger().info(f'Performance Summary - Avg CPU: {avg_cpu:.1f}%, Avg Memory: {avg_memory:.1f}%')

           pass
       finally:
           monitor.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Create and test the system monitoring node.

### Task 4.2: Resource Optimization
1. Implement a resource manager that adjusts system behavior based on resource availability:
   - Reduce processing rate when CPU is high
   - Switch to lower-resolution processing when memory is low
   - Prioritize critical tasks when resources are constrained

## Exercise 5: Deployment and Testing (2.5 hours)

### Task 5.1: Complete System Test
1. Create a comprehensive test scenario that exercises all integrated components:
   - Robot navigates to multiple waypoints
   - Detects and identifies objects at each waypoint
   - Performs actions based on object types
   - Coordinates with other robots when applicable
   - Handles various error conditions gracefully

2. Create a test script that validates the complete system:
   ```python
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from geometry_msgs.msg import PoseStamped
   from std_msgs.msg import String
   import time
   import json

   class SystemTester(Node):
       def __init__(self):
           super().__init__('system_tester')

           self.test_publisher = self.create_publisher(
               PoseStamped, '/move_base_simple/goal', 10)
           self.task_publisher = self.create_publisher(
               String, '/new_task_request', 10)
           self.status_subscriber = self.create_subscription(
               String, '/system_status', self.status_callback, 10)

           self.test_results = []
           self.current_test = 0

           # Timer to run tests
           self.test_timer = self.create_timer(5.0, self.run_next_test)

           self.get_logger().info('System Tester Started')

       def status_callback(self, msg):
           try:
               status = json.loads(msg.data)
               # Log system status for analysis
               pass
           except Exception as e:
               self.get_logger().error(f'Error parsing status: {str(e)}')

       def run_next_test(self):
           if self.current_test == 0:
               self.test_navigation()
           elif self.current_test == 1:
               self.test_perception()
           elif self.current_test == 2:
               self.test_coordination()
           elif self.current_test == 3:
               self.test_error_handling()
           else:
               self.finish_tests()
               return

           self.current_test += 1

       def test_navigation(self):
           self.get_logger().info('Running navigation test...')
           # Send navigation goal
           goal = PoseStamped()
           goal.header.stamp = self.get_clock().now().to_msg()
           goal.header.frame_id = 'map'
           goal.pose.position.x = 2.0
           goal.pose.position.y = 2.0
           goal.pose.orientation.w = 1.0

           self.test_publisher.publish(goal)
           self.test_results.append({
               'test': 'navigation',
               'timestamp': time.time(),
               'status': 'executing'
           })

       def test_perception(self):
           self.get_logger().info('Running perception test...')
           # Request task that involves perception
           task_request = String()
           task_request.data = json.dumps({
               'type': 'inspection',
               'location': {'x': 3.0, 'y': 1.0},
               'priority': 2
           })
           self.task_publisher.publish(task_request)

       def test_coordination(self):
           self.get_logger().info('Running coordination test...')
           # In a multi-robot setup, this would test coordination

       def test_error_handling(self):
           self.get_logger().info('Running error handling test...')
           # Test system behavior under simulated errors

       def finish_tests(self):
           self.get_logger().info('All tests completed')
           for result in self.test_results:
               self.get_logger().info(f"Test: {result['test']}, Status: {result['status']}")
           self.test_timer.cancel()

   def main(args=None):
       rclpy.init(args=args)
       tester = SystemTester()

       try:
           rclpy.spin(tester)
       except KeyboardInterrupt:
           pass
       finally:
           tester.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

### Task 5.2: Deployment Validation
1. Validate the complete system under various conditions:
   - Different lighting conditions (in simulation)
   - Various obstacle configurations
   - Network latency simulation
   - Component failure scenarios

2. Document the system's performance and limitations.

## Assessment Questions

1. How would you modify your system architecture to handle a 10x increase in the number of robots?
2. What strategies would you use to optimize the system for real-time performance?
3. How would you implement a rollback mechanism for failed updates in a deployed system?
4. What metrics would you track to monitor system health in a production environment?
5. How would you design the system to handle new types of tasks without major rewrites?

## Troubleshooting Tips

- **Timing issues**: Use ROS 2 time tools to analyze message delays
- **Resource constraints**: Monitor CPU/memory usage and optimize accordingly
- **Integration problems**: Test components individually before integration
- **Communication failures**: Implement retry mechanisms and timeouts
- **Performance bottlenecks**: Profile individual components to identify issues

## Extensions

1. Implement OTA (Over-The-Air) update capabilities
2. Add predictive maintenance features using ML
3. Create a web-based dashboard for system monitoring
4. Implement advanced path planning with dynamic obstacle avoidance
5. Add security features for network communication

## Summary

This lab exercise provided hands-on experience with integrating all components of a robotic system. You combined perception, control, navigation, and coordination systems into a complete application, implemented monitoring and optimization strategies, and validated the system through comprehensive testing. These skills are essential for deploying real-world robotic systems.