#  Lab Exercise: Project Development & Capstone

## Overview

In this capstone lab exercise, you will design, implement, and evaluate a comprehensive robotic system that integrates multiple concepts from throughout the course. You will apply system design principles, project management techniques, and integration strategies to create a complete robotic solution addressing a real-world challenge. This exercise synthesizes all the knowledge and skills acquired in previous weeks.

## Prerequisites

- Completion of Week 1-11 lab exercises
- Working ROS 2 Humble Hawksbill installation
- Complete Week 1-11 ROS 2 workspace with all packages
- Understanding of all previous robotics concepts
- Experience with system integration and deployment
- Knowledge of project management principles

## Learning Objectives

By completing this lab, you will:
- Integrate multiple robotics subsystems into a cohesive application
- Apply design thinking to complex robotic system development
- Implement a complete robotic solution addressing real-world challenges
- Evaluate and validate robotic systems using appropriate metrics
- Document and present robotic system designs effectively

## Capstone Project Options

Choose ONE of the following project options to implement:

### Option A: Autonomous Warehouse Assistant
An autonomous robot that navigates warehouse environments to assist with inventory management, object identification, and basic transport tasks.

### Option B: Agricultural Monitoring Robot
A field robot that monitors crop health, identifies issues, and provides data for precision agriculture applications.

### Option C: Service Robot for Indoor Environments
A social robot that assists in indoor environments such as offices, hospitals, or elderly care facilities.

### Option D: Search and Rescue Assistant
A robot designed to assist in search and rescue operations by mapping unknown environments and detecting humans.

## Exercise 1: Project Planning and Design (4 hours)

### Task 1.1: Requirements Analysis
1. Select your chosen project option and define specific requirements:
   - Create a requirements document with functional and non-functional requirements
   - Identify stakeholder needs and expectations
   - Define success criteria and acceptance tests
   - Specify technical constraints and limitations

2. Create a system architecture diagram showing:
   - Major system components
   - Data flow between components
   - External interfaces
   - Hardware-software boundaries

3. Develop a preliminary project plan:
   - Break down the project into manageable tasks
   - Estimate time and resources needed
   - Identify critical dependencies
   - Plan for risk mitigation

### Task 1.2: System Design
1. Design the software architecture:
   ```python
   # Create a package for your capstone project
   cd ~/ros2_ws/src
   ros2 pkg create --dependencies rclpy std_msgs geometry_msgs sensor_msgs cv_bridge nav_msgs tf2_msgs --build-type ament_python capstone_robot
   cd capstone_robot
   ```

2. Plan the ROS 2 node structure for your system:
   - Navigation and path planning nodes
   - Perception and sensor processing nodes
   - Control and actuation nodes
   - Human-robot interaction nodes
   - System monitoring and coordination nodes

3. Design the message interfaces between nodes:
   - Standard message types (geometry_msgs, sensor_msgs, etc.)
   - Custom message types if needed
   - Service interfaces for synchronous operations
   - Action interfaces for long-running operations

### Task 1.3: Technology Stack Selection
1. Select appropriate technologies for your implementation:
   - Perception algorithms (computer vision, sensor fusion, etc.)
   - Control strategies (PID, model predictive control, etc.)
   - Navigation approaches (global/local planners, etc.)
   - Learning methods (if applicable to your project)

2. Plan the development environment:
   - Simulation environment setup
   - Development tools and frameworks
   - Testing and debugging tools
   - Version control strategy

## Exercise 2: Core System Implementation (8 hours)

### Task 2.1: Basic System Framework
1. Implement the basic ROS 2 framework for your project:
   ```python
   # capstone_robot/system_manager.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String, Bool
   from geometry_msgs.msg import PoseStamped
   from nav_msgs.msg import Odometry
   import json
   import time
   from enum import Enum

   class RobotState(Enum):
       IDLE = 1
       NAVIGATING = 2
       PERCEIVING = 3
       INTERACTING = 4
       EMERGENCY = 5

   class CapstoneSystemManager(Node):
       def __init__(self):
           super().__init__('capstone_system_manager')

           # System state
           self.current_state = RobotState.IDLE
           self.system_health = 1.0
           self.active_tasks = []
           self.system_metrics = {}

           # Publishers
           self.state_pub = self.create_publisher(String, 'system_state', 10)
           self.health_pub = self.create_publisher(String, 'system_health', 10)
           self.status_pub = self.create_publisher(String, 'system_status', 10)

           # Subscribers
           self.odom_sub = self.create_subscription(
               Odometry, 'odom', self.odom_callback, 10)
           self.task_sub = self.create_subscription(
               String, 'task_request', self.task_callback, 10)

           # Timer for system monitoring
           self.monitor_timer = self.create_timer(1.0, self.system_monitor)

           # Initialize subsystems based on project option
           self.initialize_subsystems()

           self.get_logger().info('Capstone System Manager Started')

       def initialize_subsystems(self):
           """Initialize subsystems based on project option"""
           self.get_logger().info('Initializing subsystems...')
           # This will be customized based on chosen project option

       def odom_callback(self, msg):
           # Update robot position and state
           pass

       def task_callback(self, msg):
           # Process task requests
           task_data = json.loads(msg.data)
           self.active_tasks.append(task_data)
           self.get_logger().info(f'Received task: {task_data}')

       def system_monitor(self):
           # Monitor system health and performance
           self.system_metrics = {
               'timestamp': time.time(),
               'state': self.current_state.name,
               'active_tasks': len(self.active_tasks),
               'health': self.system_health
           }

           # Publish system state
           state_msg = String()
           state_msg.data = self.current_state.name
           self.state_pub.publish(state_msg)

           # Publish system health
           health_msg = String()
           health_msg.data = json.dumps({
               'health': self.system_health,
               'metrics': self.system_metrics
           })
           self.health_pub.publish(health_msg)

           # Publish system status
           status_msg = String()
           status_msg.data = json.dumps(self.system_metrics)
           self.status_pub.publish(status_msg)

   def main(args=None):
       rclpy.init(args=args)
       system_manager = CapstoneSystemManager()

       try:
           rclpy.spin(system_manager)
       except KeyboardInterrupt:
           pass
       finally:
           system_manager.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Update the package setup in `setup.py`:
   ```python
   entry_points={
       'console_scripts': [
           'system_manager = capstone_robot.system_manager:main',
       ],
   },
   ```

### Task 2.2: Navigation and Path Planning Module
1. Implement navigation capabilities appropriate to your project:
   ```python
   # capstone_robot/navigation_module.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from geometry_msgs.msg import Twist, PoseStamped
   from sensor_msgs.msg import LaserScan
   from nav_msgs.msg import Odometry, Path
   import numpy as np
   import math

   class NavigationModule(Node):
       def __init__(self):
           super().__init__('navigation_module')

           # Navigation parameters
           self.goal_tolerance = 0.5
           self.max_linear_speed = 0.5
           self.max_angular_speed = 1.0
           self.robot_pose = None
           self.current_goal = None
           self.path = None

           # Publishers
           self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
           self.path_pub = self.create_publisher(Path, 'navigation_path', 10)

           # Subscribers
           self.odom_sub = self.create_subscription(
               Odometry, 'odom', self.odom_callback, 10)
           self.scan_sub = self.create_subscription(
               LaserScan, 'scan', self.scan_callback, 10)
           self.goal_sub = self.create_subscription(
               PoseStamped, 'move_base_simple/goal', self.goal_callback, 10)

           # Timer for navigation control
           self.nav_timer = self.create_timer(0.1, self.navigation_control)

           self.get_logger().info('Navigation Module Started')

       def odom_callback(self, msg):
           self.robot_pose = msg.pose.pose

       def scan_callback(self, msg):
           # Process laser scan for obstacle detection
           # Implement obstacle avoidance
           pass

       def goal_callback(self, msg):
           self.current_goal = msg.pose
           self.get_logger().info(f'New goal received: {self.current_goal.position}')

       def navigation_control(self):
           if self.robot_pose and self.current_goal:
               # Calculate control commands to reach goal
               cmd_msg = Twist()

               # Calculate distance to goal
               dx = self.current_goal.position.x - self.robot_pose.position.x
               dy = self.current_goal.position.y - self.robot_pose.position.y
               distance_to_goal = math.sqrt(dx*dx + dy*dy)

               if distance_to_goal > self.goal_tolerance:
                   # Calculate angle to goal
                   angle_to_goal = math.atan2(dy, dx)
                   current_yaw = self.get_yaw_from_quaternion(self.robot_pose.orientation)
                   angle_diff = self.normalize_angle(angle_to_goal - current_yaw)

                   # Simple proportional controller
                   cmd_msg.linear.x = min(self.max_linear_speed, distance_to_goal * 0.5)
                   cmd_msg.angular.z = angle_diff * 2.0

                   # Obstacle avoidance (simplified)
                   if hasattr(self, 'laser_ranges') and self.laser_ranges:
                       min_range = min(self.laser_ranges) if self.laser_ranges else float('inf')
                       if min_range < 0.8:  # Too close to obstacle
                           cmd_msg.linear.x = 0.0
                           cmd_msg.angular.z = 0.5  # Turn away from obstacle
               else:
                   # At goal, stop
                   cmd_msg.linear.x = 0.0
                   cmd_msg.angular.z = 0.0
                   self.current_goal = None

               self.cmd_vel_pub.publish(cmd_msg)

       def get_yaw_from_quaternion(self, orientation):
           # Convert quaternion to yaw angle
           import math
           siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
           cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
           return math.atan2(siny_cosp, cosy_cosp)

       def normalize_angle(self, angle):
           # Normalize angle to [-pi, pi]
           while angle > math.pi:
               angle -= 2 * math.pi
           while angle < -math.pi:
               angle += 2 * math.pi
           return angle

   def main(args=None):
       rclpy.init(args=args)
       nav_module = NavigationModule()

       try:
           rclpy.spin(nav_module)
       except KeyboardInterrupt:
           pass
       finally:
           nav_module.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Add the navigation module to `setup.py`.

### Task 2.3: Perception and Sensing Module
1. Implement perception capabilities for your project:
   ```python
   # capstone_robot/perception_module.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image, LaserScan
   from std_msgs.msg import String
   from cv_bridge import CvBridge
   import cv2
   import numpy as np
   from geometry_msgs.msg import PointStamped
   import json

   class PerceptionModule(Node):
       def __init__(self):
           super().__init__('perception_module')

           # Initialize components
           self.bridge = CvBridge()
           self.camera_image = None
           self.laser_scan = None
           self.detected_objects = []

           # Publishers
           self.object_pub = self.create_publisher(String, 'detected_objects', 10)
           self.vision_pub = self.create_publisher(String, 'vision_processing', 10)

           # Subscribers
           self.image_sub = self.create_subscription(
               Image, '/camera/image_raw', self.image_callback, 10)
           self.scan_sub = self.create_subscription(
               LaserScan, 'scan', self.scan_callback, 10)

           # Timer for perception processing
           self.perception_timer = self.create_timer(0.5, self.perception_processing)

           self.get_logger().info('Perception Module Started')

       def image_callback(self, msg):
           try:
               # Convert ROS Image to OpenCV
               self.camera_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
           except Exception as e:
               self.get_logger().error(f'Image callback error: {str(e)}')

       def scan_callback(self, msg):
           self.laser_scan = msg

       def perception_processing(self):
           if self.camera_image is not None:
               # Process image for object detection (simplified)
               detected_objects = self.detect_objects_in_image(self.camera_image)

               if detected_objects:
                   # Publish detected objects
                   obj_msg = String()
                   obj_msg.data = json.dumps(detected_objects)
                   self.object_pub.publish(obj_msg)
                   self.get_logger().info(f'Detected {len(detected_objects)} objects')

                   # Store for system use
                   self.detected_objects = detected_objects

           if self.laser_scan is not None:
               # Process laser scan for obstacle detection
               obstacles = self.detect_obstacles_in_scan(self.laser_scan)
               if obstacles:
                   self.get_logger().info(f'Detected {len(obstacles)} obstacles from scan')

       def detect_objects_in_image(self, image):
           # Simplified object detection (in real implementation, use ML models)
           # For this example, detect colored objects using basic color filtering
           hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

           # Detect red objects (as example)
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
                   obj_info = {
                       'type': 'red_object',
                       'bbox': [x, y, w, h],
                       'confidence': 0.8,
                       'center': [x + w/2, y + h/2]
                   }
                   detected_objects.append(obj_info)

           return detected_objects

       def detect_obstacles_in_scan(self, scan_msg):
           obstacles = []
           for i, range_val in enumerate(scan_msg.ranges):
               if not (np.isnan(range_val) or np.isinf(range_val)):
                   if range_val < 1.0:  # Obstacle within 1 meter
                       angle = scan_msg.angle_min + i * scan_msg.angle_increment
                       x = range_val * np.cos(angle)
                       y = range_val * np.sin(angle)

                       obstacles.append({
                           'position': [x, y],
                           'distance': range_val,
                           'angle': angle
                       })

           return obstacles

   def main(args=None):
       rclpy.init(args=args)
       perception_module = PerceptionModule()

       try:
           rclpy.spin(perception_module)
       except KeyboardInterrupt:
           pass
       finally:
           perception_module.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Add the perception module to `setup.py`.

## Exercise 3: Integration and Coordination (4 hours)

### Task 3.1: System Integration
1. Create a coordinator node that manages the interaction between different modules:
   ```python
   # capstone_robot/system_coordinator.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String, Bool
   from geometry_msgs.msg import PoseStamped
   import json
   import time
   from enum import Enum

   class TaskType(Enum):
       NAVIGATION = 1
       PERCEPTION = 2
       INTERACTION = 3
       MONITORING = 4

   class SystemCoordinator(Node):
       def __init__(self):
           super().__init__('system_coordinator')

           # System state
           self.active_tasks = []
           self.task_queue = []
           self.system_status = 'IDLE'
           self.health_status = 1.0
           self.last_update = time.time()

           # Publishers
           self.task_pub = self.create_publisher(String, 'task_request', 10)
           self.goal_pub = self.create_publisher(PoseStamped, 'move_base_simple/goal', 10)
           self.status_pub = self.create_publisher(String, 'system_status', 10)

           # Subscribers
           self.object_sub = self.create_subscription(
               String, 'detected_objects', self.object_callback, 10)
           self.system_state_sub = self.create_subscription(
               String, 'system_state', self.system_state_callback, 10)
           self.system_health_sub = self.create_subscription(
               String, 'system_health', self.system_health_callback, 10)

           # Timer for task coordination
           self.coordination_timer = self.create_timer(1.0, self.coordination_loop)

           # Initialize based on project option
           self.initialize_project_specifics()

           self.get_logger().info('System Coordinator Started')

       def initialize_project_specifics(self):
           """Initialize project-specific behaviors"""
           # This would be customized based on the chosen project option
           self.get_logger().info('Project-specific initialization complete')

       def object_callback(self, msg):
           try:
               objects = json.loads(msg.data)
               if objects:
                   self.process_detected_objects(objects)
           except Exception as e:
               self.get_logger().error(f'Error processing objects: {str(e)}')

       def system_state_callback(self, msg):
           self.system_status = msg.data

       def system_health_callback(self, msg):
           try:
               health_data = json.loads(msg.data)
               self.health_status = health_data.get('health', 1.0)
           except Exception as e:
               self.get_logger().error(f'Error processing health: {str(e)}')

       def process_detected_objects(self, objects):
           """Process detected objects and plan appropriate actions"""
           for obj in objects:
               if obj['type'] == 'red_object' and obj['confidence'] > 0.7:
                   # Example: Navigate toward high-confidence red objects
                   self.request_navigation_to_object(obj)

       def request_navigation_to_object(self, obj):
           """Request navigation to a detected object"""
           # Create a goal pose near the object (simplified)
           goal_msg = PoseStamped()
           goal_msg.header.stamp = self.get_clock().now().to_msg()
           goal_msg.header.frame_id = 'map'
           # Convert image coordinates to world coordinates (simplified)
           goal_msg.pose.position.x = 2.0  # Example position
           goal_msg.pose.position.y = 2.0  # Example position
           goal_msg.pose.orientation.w = 1.0

           self.goal_pub.publish(goal_msg.pose)
           self.get_logger().info(f'Requested navigation to object at {obj["center"]}')

       def coordination_loop(self):
           # Check system health
           if self.health_status < 0.3:
               self.get_logger().warn('System health critical, reducing activity')
               return

           # Process task queue
           if self.task_queue and self.system_status == 'IDLE':
               next_task = self.task_queue.pop(0)
               self.execute_task(next_task)

           # Publish system status
           status_msg = String()
           status_msg.data = json.dumps({
               'status': self.system_status,
               'health': self.health_status,
               'active_tasks': len(self.active_tasks),
               'queued_tasks': len(self.task_queue),
               'timestamp': time.time()
           })
           self.status_pub.publish(status_msg)

       def execute_task(self, task):
           """Execute a specific task"""
           task_type = task.get('type')
           if task_type == TaskType.NAVIGATION.name:
               self.execute_navigation_task(task)
           elif task_type == TaskType.PERCEPTION.name:
               self.execute_perception_task(task)
           # Add other task types as needed

       def execute_navigation_task(self, task):
           """Execute navigation task"""
           goal = task.get('goal')
           if goal:
               goal_msg = PoseStamped()
               goal_msg.header.stamp = self.get_clock().now().to_msg()
               goal_msg.header.frame_id = 'map'
               goal_msg.pose = goal
               self.goal_pub.publish(goal_msg.pose)

   def main(args=None):
       rclpy.init(args=args)
       coordinator = SystemCoordinator()

       try:
           rclpy.spin(coordinator)
       except KeyboardInterrupt:
           pass
       finally:
           coordinator.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Add the coordinator to `setup.py`.

### Task 3.2: Human-Robot Interaction Module
1. Implement human-robot interaction capabilities:
   ```python
   # capstone_robot/hri_module.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String
   from geometry_msgs.msg import Twist
   import json

   class HRIModule(Node):
       def __init__(self):
           super().__init__('hri_module')

           # Interaction state
           self.current_interaction = None
           self.user_preferences = {}
           self.conversation_context = {}

           # Publishers
           self.response_pub = self.create_publisher(String, 'robot_response', 10)
           self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

           # Subscribers
           self.speech_sub = self.create_subscription(
               String, 'speech_commands', self.speech_callback, 10)
           self.gesture_sub = self.create_subscription(
               String, 'detected_gestures', self.gesture_callback, 10)

           self.get_logger().info('HRI Module Started')

       def speech_callback(self, msg):
           user_input = msg.data.lower()
           response = self.process_speech_command(user_input)

           response_msg = String()
           response_msg.data = response
           self.response_pub.publish(response_msg)

           self.get_logger().info(f'Processed: {user_input} -> {response}')

       def gesture_callback(self, msg):
           gesture = msg.data
           self.process_gesture(gesture)

       def process_speech_command(self, text):
           """Process natural language commands"""
           if any(greeting in text for greeting in ['hello', 'hi', 'hey']):
               return "Hello! How can I assist you today?"
           elif 'go to' in text or 'navigate to' in text:
               # Extract destination from command (simplified)
               return "I can help with navigation. Please specify the location."
           elif 'what do you see' in text or 'perceive' in text:
               return "I'm continuously monitoring my environment and will alert you to important findings."
           else:
               return "I'm not sure I understand. Could you rephrase that?"

       def process_gesture(self, gesture):
           """Process gesture commands"""
           if gesture == 'five':  # Open hand
               self.get_logger().info('Gesture detected: Open hand - possible greeting or attention request')
           elif gesture == 'fist':  # Closed fist
               self.get_logger().info('Gesture detected: Closed fist - possible stop command')

   def main(args=None):
       rclpy.init(args=args)
       hri_module = HRIModule()

       try:
           rclpy.spin(hri_module)
       except KeyboardInterrupt:
           pass
       finally:
           hri_module.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Add the HRI module to `setup.py` and build the package.

## Exercise 4: Testing and Evaluation (2 hours)

### Task 4.1: System Testing Framework
1. Create a testing framework for your integrated system:
   ```python
   # capstone_robot/testing_framework.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String
   import json
   import time
   from datetime import datetime

   class TestingFramework(Node):
       def __init__(self):
           super().__init__('testing_framework')

           # Test parameters
           self.test_scenarios = []
           self.test_results = {}
           self.current_test = None
           self.test_start_time = None

           # Publishers
           self.test_status_pub = self.create_publisher(String, 'test_status', 10)
           self.test_results_pub = self.create_publisher(String, 'test_results', 10)

           # Initialize test scenarios
           self.initialize_test_scenarios()

           self.get_logger().info('Testing Framework Started')

       def initialize_test_scenarios(self):
           """Define test scenarios for the capstone project"""
           self.test_scenarios = [
               {
                   'id': 'navigation_basic',
                   'description': 'Basic navigation to goal',
                   'steps': [
                       {'action': 'publish_goal', 'params': {'x': 2.0, 'y': 2.0}},
                       {'action': 'wait_for_completion', 'timeout': 30.0}
                   ],
                   'success_criteria': 'reach_goal_within_tolerance'
               },
               {
                   'id': 'perception_basic',
                   'description': 'Basic object detection',
                   'steps': [
                       {'action': 'activate_perception', 'duration': 10.0},
                       {'action': 'verify_detection', 'min_objects': 1}
                   ],
                   'success_criteria': 'detect_objects'
               },
               {
                   'id': 'integration_basic',
                   'description': 'System integration test',
                   'steps': [
                       {'action': 'request_task', 'task': 'navigation_with_perception'},
                       {'action': 'monitor_system_state', 'duration': 60.0}
                   ],
                   'success_criteria': 'complete_task_safely'
               }
           ]

       def run_all_tests(self):
           """Run all defined test scenarios"""
           results = {}

           for scenario in self.test_scenarios:
               self.get_logger().info(f'Running test: {scenario["description"]}')
               result = self.run_single_test(scenario)
               results[scenario['id']] = result

           # Publish final results
           results_msg = String()
           results_msg.data = json.dumps(results, indent=2)
           self.test_results_pub.publish(results_msg)

           self.get_logger().info('All tests completed')
           self.get_logger().info(json.dumps(results, indent=2))

           return results

       def run_single_test(self, scenario):
           """Run a single test scenario"""
           start_time = time.time()
           result = {
               'id': scenario['id'],
               'description': scenario['description'],
               'status': 'RUNNING',
               'start_time': datetime.now().isoformat(),
               'steps_executed': 0,
               'steps_passed': 0,
               'steps_failed': 0,
               'errors': [],
               'metrics': {}
           }

           try:
               for step in scenario['steps']:
                   step_result = self.execute_test_step(step)
                   result['steps_executed'] += 1
                   if step_result['success']:
                       result['steps_passed'] += 1
                   else:
                       result['steps_failed'] += 1
                       result['errors'].append(step_result['error'])

               # Determine overall success
               if result['steps_failed'] == 0:
                   result['status'] = 'PASSED'
               elif result['steps_passed'] > 0:
                   result['status'] = 'PARTIAL'
               else:
                   result['status'] = 'FAILED'

           except Exception as e:
               result['status'] = 'ERROR'
               result['errors'].append(str(e))

           result['end_time'] = datetime.now().isoformat()
           result['duration'] = time.time() - start_time

           return result

       def execute_test_step(self, step):
           """Execute a single test step"""
           action = step['action']
           params = {k: v for k, v in step.items() if k != 'action'}

           # In a real implementation, these would interface with the actual system
           # For this example, we'll simulate the actions
           if action == 'publish_goal':
               # Simulate publishing a navigation goal
               self.get_logger().info(f'Publishing goal: {params}')
               return {'success': True, 'details': 'Goal published successfully'}
           elif action == 'wait_for_completion':
               # Simulate waiting for task completion
               self.get_logger().info(f'Waiting for completion with timeout: {params["timeout"]}')
               time.sleep(min(2.0, params['timeout']))  # Simulate wait
               return {'success': True, 'details': 'Task completed successfully'}
           elif action == 'activate_perception':
               # Simulate activating perception system
               self.get_logger().info(f'Activating perception for {params["duration"]} seconds')
               return {'success': True, 'details': 'Perception activated'}
           elif action == 'verify_detection':
               # Simulate verifying object detection
               self.get_logger().info(f'Verifying detection of at least {params["min_objects"]} objects')
               return {'success': True, 'details': 'Objects detected successfully'}
           else:
               return {'success': False, 'error': f'Unknown action: {action}'}

   def main(args=None):
       rclpy.init(args=args)
       test_framework = TestingFramework()

       try:
           # Run tests after a short delay to allow system to initialize
           test_framework.create_timer(5.0, lambda: test_framework.run_all_tests())
           rclpy.spin(test_framework)
       except KeyboardInterrupt:
           pass
       finally:
           test_framework.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Add the testing framework to `setup.py`.

### Task 4.2: Performance Evaluation
1. Create a performance evaluation system:
   ```python
   # capstone_robot/performance_evaluator.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String, Float32
   import json
   import time
   import statistics
   from collections import deque

   class PerformanceEvaluator(Node):
       def __init__(self):
           super().__init__('performance_evaluator')

           # Performance metrics tracking
           self.metrics = {
               'task_completion_times': deque(maxlen=100),
               'success_rates': deque(maxlen=100),
               'resource_usage': deque(maxlen=100),
               'response_times': deque(maxlen=100),
               'navigation_accuracy': deque(maxlen=100)
           }

           self.evaluation_start = time.time()

           # Publishers
           self.metrics_pub = self.create_publisher(String, 'performance_metrics', 10)
           self.report_pub = self.create_publisher(String, 'performance_report', 10)

           # Subscribers
           self.task_completion_sub = self.create_subscription(
               String, 'task_completion', self.task_completion_callback, 10)
           self.navigation_accuracy_sub = self.create_subscription(
               Float32, 'navigation_accuracy', self.navigation_accuracy_callback, 10)

           # Timer for periodic evaluation
           self.evaluation_timer = self.create_timer(5.0, self.evaluate_performance)

           self.get_logger().info('Performance Evaluator Started')

       def task_completion_callback(self, msg):
           try:
               data = json.loads(msg.data)
               if 'completion_time' in data:
                   self.metrics['task_completion_times'].append(data['completion_time'])
               if 'success' in data:
                   self.metrics['success_rates'].append(1.0 if data['success'] else 0.0)
               if 'response_time' in data:
                   self.metrics['response_times'].append(data['response_time'])
           except Exception as e:
               self.get_logger().error(f'Error processing task completion: {str(e)}')

       def navigation_accuracy_callback(self, msg):
           self.metrics['navigation_accuracy'].append(msg.data)

       def evaluate_performance(self):
           # Calculate performance metrics
           metrics_summary = {}

           for metric_name, values in self.metrics.items():
               if values:
                   values_list = list(values)
                   metrics_summary[metric_name] = {
                       'count': len(values_list),
                       'mean': statistics.mean(values_list),
                       'std': statistics.stdev(values_list) if len(values_list) > 1 else 0,
                       'min': min(values_list),
                       'max': max(values_list),
                       'latest': values_list[-1] if values_list else 0
                   }

           # Calculate derived metrics
           total_runtime = time.time() - self.evaluation_start
           metrics_summary['system_runtime'] = total_runtime

           if 'success_rates' in metrics_summary:
               overall_success_rate = metrics_summary['success_rates']['mean']
               metrics_summary['overall_success_rate'] = overall_success_rate

           # Create performance report
           report = {
               'timestamp': time.time(),
               'runtime_hours': total_runtime / 3600,
               'metrics': metrics_summary,
               'system_health': self.assess_system_health(metrics_summary)
           }

           # Publish metrics
           metrics_msg = String()
           metrics_msg.data = json.dumps(metrics_summary, indent=2)
           self.metrics_pub.publish(metrics_msg)

           # Publish report
           report_msg = String()
           report_msg.data = json.dumps(report, indent=2)
           self.report_pub.publish(report_msg)

           self.get_logger().info(f'Performance Report:\n{json.dumps(report, indent=2)}')

       def assess_system_health(self, metrics_summary):
           """Assess overall system health based on metrics"""
           health_score = 1.0

           # Check success rate
           if 'overall_success_rate' in metrics_summary:
               success_rate = metrics_summary['overall_success_rate']
               if success_rate < 0.7:
                   health_score *= 0.5
               elif success_rate < 0.9:
                   health_score *= 0.8

           # Check response times
           if 'response_times' in metrics_summary:
               avg_response = metrics_summary['response_times']['mean']
               if avg_response > 5.0:  # More than 5 seconds average
                   health_score *= 0.7

           # Check navigation accuracy
           if 'navigation_accuracy' in metrics_summary:
               avg_accuracy = metrics_summary['navigation_accuracy']['mean']
               if avg_accuracy > 0.5:  # More than 0.5m average error
                   health_score *= 0.8

           return max(0.0, min(1.0, health_score))

   def main(args=None):
       rclpy.init(args=args)
       evaluator = PerformanceEvaluator()

       try:
           rclpy.spin(evaluator)
       except KeyboardInterrupt:
           # Print final summary
           evaluator.get_logger().info('Final Performance Summary:')
           evaluator.evaluate_performance()
           pass
       finally:
           evaluator.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

## Assessment Questions

1. How would you modify your system architecture to handle a 10x increase in operational complexity?
2. What strategies would you implement to ensure system reliability during extended autonomous operation?
3. How would you design your system to accommodate future feature additions without major rewrites?
4. What metrics would you track to monitor system performance in a production environment?
5. How would you handle system failures and ensure graceful degradation of capabilities?

## Troubleshooting Tips

- **Integration issues**: Test components individually before integration
- **Performance bottlenecks**: Profile individual modules to identify issues
- **Communication failures**: Implement robust error handling and recovery
- **Resource constraints**: Monitor CPU, memory, and power usage
- **Timing issues**: Use ROS 2 time tools to analyze message delays

## Extensions

1. Implement machine learning capabilities for adaptive behavior
2. Add multi-robot coordination features
3. Create a web-based monitoring interface
4. Implement advanced perception algorithms
5. Add predictive maintenance capabilities

## Summary

This capstone lab exercise provided comprehensive experience in designing, implementing, and evaluating a complete robotic system. You integrated multiple subsystems, managed system complexity, and applied professional development practices. The skills developed in this exercise provide a foundation for real-world robotics projects and professional practice in the field.