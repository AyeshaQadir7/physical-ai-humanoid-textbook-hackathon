# Week 11 Lab Exercise: Field Robotics & Real-World Applications

## Overview

In this lab exercise, you will implement robotic systems for specific field applications, address environmental challenges in unstructured environments, and develop robust perception and navigation capabilities. You will work with specialized sensors, implement environmental adaptation strategies, and create systems capable of operating in harsh conditions. This builds upon your Week 1-10 foundations to create robots suitable for real-world deployment.

## Prerequisites

- Completion of Week 1-10 lab exercises
- Working ROS 2 Humble Hawksbill installation
- Access to field robotics simulation environments
- Domain-specific sensor and actuator models
- Understanding of environmental science and domain-specific applications
- Experience with robot deployment and integration

## Learning Objectives

By completing this lab, you will:
- Design robotic systems for specific field applications (agriculture, construction, etc.)
- Address environmental challenges such as weather, terrain, and lighting
- Implement robust perception and navigation for unstructured environments
- Develop maintenance and reliability strategies for field deployment
- Evaluate field robotics systems in real-world scenarios

## Exercise 1: Agricultural Robotics Application (2.5 hours)

### Task 1.1: Crop Monitoring System
1. Create a ROS 2 package for agricultural robotics:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --dependencies rclpy std_msgs sensor_msgs geometry_msgs cv_bridge message_filters --build-type ament_python field_robotics_lab
   cd field_robotics_lab
   ```

2. Install required Python packages:
   ```bash
   pip3 install opencv-python numpy
   ```

3. Create an agricultural monitoring node `field_robotics_lab/crop_monitoring.py`:
   ```python
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image, NavSatFix, LaserScan
   from geometry_msgs.msg import Twist, PointStamped
   from cv_bridge import CvBridge
   import cv2
   import numpy as np
   from std_msgs.msg import String, Float32
   import message_filters

   class CropMonitoringNode(Node):
       def __init__(self):
           super().__init__('crop_monitoring_node')

           # Initialize components
           self.bridge = CvBridge()
           self.gps_data = None
           self.camera_image = None
           self.laser_scan = None

           # Publishers
           self.health_pub = self.create_publisher(Float32, 'crop_health', 10)
           self.alert_pub = self.create_publisher(String, 'field_alerts', 10)
           self.navigation_cmd = self.create_publisher(Twist, 'cmd_vel', 10)

           # Subscribers
           self.image_sub = self.create_subscription(
               Image, '/camera/image_raw', self.image_callback, 10)
           self.gps_sub = self.create_subscription(
               NavSatFix, '/gps/fix', self.gps_callback, 10)
           self.scan_sub = self.create_subscription(
               LaserScan, '/scan', self.scan_callback, 10)

           # Timer for monitoring loop
           self.monitor_timer = self.create_timer(2.0, self.monitoring_loop)

           self.get_logger().info('Crop Monitoring Node Started')

       def image_callback(self, msg):
           try:
               # Convert ROS Image message to OpenCV image
               cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
               self.camera_image = cv_image

               # Analyze crop health (simplified - in real application, use advanced computer vision)
               health_score = self.assess_crop_health(cv_image)

               health_msg = Float32()
               health_msg.data = health_score
               self.health_pub.publish(health_msg)

           except Exception as e:
               self.get_logger().error(f'Error processing image: {str(e)}')

       def gps_callback(self, msg):
           self.gps_data = {
               'latitude': msg.latitude,
               'longitude': msg.longitude,
               'altitude': msg.altitude
           }

       def scan_callback(self, msg):
           self.laser_scan = msg

       def assess_crop_health(self, image):
           # Simplified crop health assessment using vegetation indices
           # In real application, use more sophisticated methods like NDVI
           hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

           # Define range for green vegetation (adjust based on lighting conditions)
           lower_green = np.array([35, 40, 40])
           upper_green = np.array([85, 255, 255])
           mask = cv2.inRange(hsv, lower_green, upper_green)

           # Calculate vegetation ratio
           total_pixels = image.shape[0] * image.shape[1]
           vegetation_pixels = np.sum(mask > 0)
           vegetation_ratio = vegetation_pixels / total_pixels

           # Calculate health score (0-1 scale)
           health_score = min(1.0, vegetation_ratio * 2)  # Adjust multiplier as needed

           return health_score

       def monitoring_loop(self):
           if self.camera_image is not None:
               # Assess crop health
               health_score = self.assess_crop_health(self.camera_image)

               # Check for anomalies
               if health_score < 0.3:  # Low health threshold
                   alert_msg = String()
                   alert_msg.data = f"LOW_CROP_HEALTH at {self.gps_data}: {health_score:.2f}"
                   self.alert_pub.publish(alert_msg)
                   self.get_logger().warn(f'Crop health alert: {alert_msg.data}')

               # Navigate to interesting areas
               self.navigate_to_interesting_areas()

               self.get_logger().info(f'Crop health score: {health_score:.2f}')

       def navigate_to_interesting_areas(self):
           # Simplified navigation to areas of interest
           # In real application, implement more sophisticated path planning
           cmd_msg = Twist()

           # If laser scan shows clear path, move forward
           if self.laser_scan and len(self.laser_scan.ranges) > 0:
               front_distance = self.laser_scan.ranges[len(self.laser_scan.ranges)//2]
               if front_distance > 1.0:  # Safe distance
                   cmd_msg.linear.x = 0.3
               else:
                   cmd_msg.angular.z = 0.5  # Turn to avoid obstacle

           self.navigation_cmd.publish(cmd_msg)

   def main(args=None):
       rclpy.init(args=args)
       crop_monitor = CropMonitoringNode()

       try:
           rclpy.spin(crop_monitor)
       except KeyboardInterrupt:
           pass
       finally:
           crop_monitor.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

4. Update the package setup in `setup.py`:
   ```python
   entry_points={
       'console_scripts': [
           'crop_monitoring = field_robotics_lab.crop_monitoring:main',
       ],
   },
   ```

### Task 1.2: Variable Rate Application System
1. Create a precision agriculture application node:
   ```python
   # field_robotics_lab/precision_agriculture.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import Float32, String
   from geometry_msgs.msg import Twist
   from sensor_msgs.msg import NavSatFix
   import numpy as np
   import json

   class PrecisionAgricultureNode(Node):
       def __init__(self):
           super().__init__('precision_agriculture_node')

           # Application system parameters
           self.current_position = None
           self.crop_health_map = {}  # GPS coordinates -> health score
           self.application_rates = {}  # GPS coordinates -> application rate
           self.field_boundary = None

           # Publishers
           self.application_cmd = self.create_publisher(Float32, 'application_rate', 10)
           self.status_pub = self.create_publisher(String, 'field_status', 10)

           # Subscribers
           self.gps_sub = self.create_subscription(
               NavSatFix, '/gps/fix', self.gps_callback, 10)
           self.health_sub = self.create_subscription(
               Float32, 'crop_health', self.health_callback, 10)

           # Timer for application control
           self.application_timer = self.create_timer(0.5, self.application_control)

           self.get_logger().info('Precision Agriculture Node Started')

       def gps_callback(self, msg):
           self.current_position = {
               'latitude': msg.latitude,
               'longitude': msg.longitude,
               'altitude': msg.altitude
           }

       def health_callback(self, msg):
           if self.current_position:
               # Store crop health at current position
               pos_key = f"{self.current_position['latitude']:.6f},{self.current_position['longitude']:.6f}"
               self.crop_health_map[pos_key] = msg.data

               # Calculate application rate based on health
               application_rate = self.calculate_application_rate(msg.data)
               self.application_rates[pos_key] = application_rate

               self.get_logger().info(f'Health: {msg.data:.2f}, Rate: {application_rate:.2f}')

       def calculate_application_rate(self, health_score):
           # Calculate application rate based on crop health
           # Lower health = higher application rate
           if health_score < 0.3:
               return 1.0  # High application rate for poor health
           elif health_score < 0.6:
               return 0.7  # Medium application rate for moderate health
           else:
               return 0.3  # Low application rate for good health

       def application_control(self):
           if self.current_position:
               pos_key = f"{self.current_position['latitude']:.6f},{self.current_position['longitude']:.6f}"

               if pos_key in self.application_rates:
                   rate = self.application_rates[pos_key]

                   # Publish application rate
                   rate_msg = Float32()
                   rate_msg.data = rate
                   self.application_cmd.publish(rate_msg)

                   # Publish status
                   status_msg = String()
                   status_msg.data = json.dumps({
                       'position': self.current_position,
                       'application_rate': rate,
                       'timestamp': self.get_clock().now().nanoseconds
                   })
                   self.status_pub.publish(status_msg)

   def main(args=None):
       rclpy.init(args=args)
       ag_node = PrecisionAgricultureNode()

       try:
           rclpy.spin(ag_node)
       except KeyboardInterrupt:
           pass
       finally:
           ag_node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Add the precision agriculture node to `setup.py` and build the package.

## Exercise 2: Robust Perception in Challenging Environments (2.5 hours)

### Task 2.1: Multi-Modal Perception Fusion
1. Create a robust perception system that combines multiple sensors:
   ```python
   # field_robotics_lab/robust_perception.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image, LaserScan, PointCloud2
   from geometry_msgs.msg import PoseStamped
   from cv_bridge import CvBridge
   import cv2
   import numpy as np
   from std_msgs.msg import String
   import sensor_msgs.point_cloud2 as pc2
   from scipy.spatial.transform import Rotation as R

   class RobustPerceptionNode(Node):
       def __init__(self):
           super().__init__('robust_perception_node')

           # Initialize components
           self.bridge = CvBridge()
           self.camera_image = None
           self.laser_scan = None
           self.point_cloud = None
           self.fused_obstacles = []

           # Publishers
           self.obstacle_pub = self.create_publisher(String, 'fused_obstacles', 10)
           self.perception_status = self.create_publisher(String, 'perception_status', 10)

           # Subscribers
           self.image_sub = self.create_subscription(
               Image, '/camera/image_raw', self.image_callback, 10)
           self.scan_sub = self.create_subscription(
               LaserScan, '/scan', self.scan_callback, 10)
           self.pc_sub = self.create_subscription(
               PointCloud2, '/point_cloud', self.pc_callback, 10)

           # Timer for perception fusion
           self.fusion_timer = self.create_timer(0.1, self.perception_fusion)

           self.get_logger().info('Robust Perception Node Started')

       def image_callback(self, msg):
           try:
               self.camera_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
           except Exception as e:
               self.get_logger().error(f'Image callback error: {str(e)}')

       def scan_callback(self, msg):
           self.laser_scan = msg

       def pc_callback(self, msg):
           try:
               # Convert point cloud to numpy array
               points = []
               for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                   points.append([point[0], point[1], point[2]])
               self.point_cloud = np.array(points)
           except Exception as e:
               self.get_logger().error(f'Point cloud callback error: {str(e)}')

       def detect_obstacles_from_image(self, image):
           # Simplified obstacle detection in image
           # In real application, use deep learning models
           gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
           edges = cv2.Canny(gray, 50, 150)

           # Find contours (potential obstacles)
           contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

           obstacles = []
           for contour in contours:
               if cv2.contourArea(contour) > 100:  # Filter small contours
                   x, y, w, h = cv2.boundingRect(contour)
                   obstacles.append({
                       'type': 'image_obstacle',
                       'bbox': [x, y, w, h],
                       'confidence': 0.7
                   })

           return obstacles

       def detect_obstacles_from_scan(self, scan):
           # Detect obstacles from laser scan
           obstacles = []

           for i, range_val in enumerate(scan.ranges):
               if not np.isnan(range_val) and not np.isinf(range_val):
                   if range_val < 2.0:  # Obstacle within 2 meters
                       angle = scan.angle_min + i * scan.angle_increment
                       x = range_val * np.cos(angle)
                       y = range_val * np.sin(angle)

                       obstacles.append({
                           'type': 'laser_obstacle',
                           'position': [x, y],
                           'distance': range_val,
                           'confidence': 0.8
                       })

           return obstacles

       def detect_obstacles_from_pointcloud(self, point_cloud):
           # Detect obstacles from point cloud
           if point_cloud is None or len(point_cloud) == 0:
               return []

           # Cluster points to identify obstacles
           obstacles = []

           # Simple clustering: group nearby points
           for i, point in enumerate(point_cloud):
               if point[2] < 1.0:  # Only consider ground-level obstacles
                   obstacles.append({
                       'type': 'pc_obstacle',
                       'position': point.tolist(),
                       'confidence': 0.9
                   })

           return obstacles

       def perception_fusion(self):
           all_obstacles = []

           # Process image data if available
           if self.camera_image is not None:
               image_obstacles = self.detect_obstacles_from_image(self.camera_image)
               all_obstacles.extend(image_obstacles)

           # Process laser scan data if available
           if self.laser_scan is not None:
               scan_obstacles = self.detect_obstacles_from_scan(self.laser_scan)
               all_obstacles.extend(scan_obstacles)

           # Process point cloud data if available
           if self.point_cloud is not None:
               pc_obstacles = self.detect_obstacles_from_pointcloud(self.point_cloud)
               all_obstacles.extend(pc_obstacles)

           # Fuse detections based on confidence and spatial proximity
           fused_obstacles = self.fuse_obstacles(all_obstacles)

           # Publish fused obstacles
           obstacle_msg = String()
           obstacle_msg.data = str(fused_obstacles)
           self.obstacle_pub.publish(obstacle_msg)

           # Log perception status
           status_msg = String()
           status_msg.data = f"Processed {len(all_obstacles)} raw detections, {len(fused_obstacles)} fused obstacles"
           self.perception_status.publish(status_msg)

           self.get_logger().info(f'Fused {len(fused_obstacles)} obstacles')

       def fuse_obstacles(self, obstacles):
           # Simple fusion algorithm: group obstacles that are close together
           if not obstacles:
               return []

           fused = []
           used = [False] * len(obstacles)

           for i, obs1 in enumerate(obstacles):
               if used[i]:
                   continue

               # Find similar obstacles
               similar = [obs1]
               used[i] = True

               for j, obs2 in enumerate(obstacles[i+1:], i+1):
                   if used[j]:
                       continue

                   # Check if obstacles are similar (for this example, just check if they're close in space)
                   if (obs1['type'] in ['laser_obstacle', 'pc_obstacle'] and
                       obs2['type'] in ['laser_obstacle', 'pc_obstacle']):
                       # Compare positions if both have position data
                       pos1 = obs1.get('position')
                       pos2 = obs2.get('position')
                       if pos1 and pos2:
                           dist = np.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2)
                           if dist < 0.5:  # 50cm threshold
                               similar.append(obs2)
                               used[j] = True

               # Create fused obstacle
               fused_obstacle = {
                   'type': 'fused_obstacle',
                   'count': len(similar),
                   'confidence': max([o.get('confidence', 0.5) for o in similar]),
                   'sources': [o['type'] for o in similar]
               }

               # Average position if available
               positions = [o.get('position') for o in similar if o.get('position')]
               if positions:
                   avg_pos = np.mean(positions, axis=0)
                   fused_obstacle['position'] = avg_pos.tolist()

               fused.append(fused_obstacle)

           return fused

   def main(args=None):
       rclpy.init(args=args)
       perception_node = RobustPerceptionNode()

       try:
           rclpy.spin(perception_node)
       except KeyboardInterrupt:
           pass
       finally:
           perception_node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Add the robust perception node to `setup.py`.

### Task 2.2: Environmental Adaptation System
1. Create a system that adapts to changing environmental conditions:
   ```python
   # field_robotics_lab/env_adaptation.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image, LaserScan, Illuminance, Temperature
   from std_msgs.msg import String, Float32
   from cv_bridge import CvBridge
   import cv2
   import numpy as np
   import json

   class EnvironmentalAdaptationNode(Node):
       def __init__(self):
           super().__init__('env_adaptation_node')

           # Initialize components
           self.bridge = CvBridge()
           self.camera_image = None
           self.laser_scan = None
           self.illuminance = None
           self.temperature = None

           # Environmental parameters
           self.current_env_conditions = {
               'lighting': 'normal',  # 'low', 'normal', 'bright'
               'temperature': 'normal',  # 'cold', 'normal', 'hot'
               'visibility': 'good'  # 'poor', 'good', 'excellent'
           }

           # Adaptation parameters
           self.adaptation_params = {
               'image_brightness': 1.0,
               'image_contrast': 1.0,
               'scan_range_threshold': 30.0,  # meters
               'processing_rate': 10.0  # Hz
           }

           # Publishers
           self.adaptation_pub = self.create_publisher(String, 'adaptation_params', 10)
           self.env_status_pub = self.create_publisher(String, 'environment_status', 10)

           # Subscribers
           self.image_sub = self.create_subscription(
               Image, '/camera/image_raw', self.image_callback, 10)
           self.scan_sub = self.create_subscription(
               LaserScan, '/scan', self.scan_callback, 10)
           self.illuminance_sub = self.create_subscription(
               Illuminance, '/illuminance', self.illuminance_callback, 10)
           self.temp_sub = self.create_subscription(
               Temperature, '/temperature', self.temperature_callback, 10)

           # Timer for environmental assessment
           self.assessment_timer = self.create_timer(1.0, self.environmental_assessment)

           self.get_logger().info('Environmental Adaptation Node Started')

       def image_callback(self, msg):
           try:
               self.camera_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
           except Exception as e:
               self.get_logger().error(f'Image callback error: {str(e)}')

       def scan_callback(self, msg):
           self.laser_scan = msg

       def illuminance_callback(self, msg):
           self.illuminance = msg.illuminance

       def temperature_callback(self, msg):
           self.temperature = msg.temperature

       def assess_lighting_conditions(self):
           if self.camera_image is not None:
               # Calculate image brightness
               gray = cv2.cvtColor(self.camera_image, cv2.COLOR_BGR2GRAY)
               mean_brightness = np.mean(gray)

               if mean_brightness < 50:  # Low light threshold
                   return 'low'
               elif mean_brightness > 200:  # Bright light threshold
                   return 'bright'
               else:
                   return 'normal'
           elif self.illuminance is not None:
               if self.illuminance < 100:  # Lux threshold
                   return 'low'
               elif self.illuminance > 10000:  # Lux threshold
                   return 'bright'
               else:
                   return 'normal'
           else:
               return 'normal'  # Default

       def assess_temperature_conditions(self):
           if self.temperature is not None:
               if self.temperature < 0:  # Celsius threshold
                   return 'cold'
               elif self.temperature > 40:  # Celsius threshold
                   return 'hot'
               else:
                   return 'normal'
           else:
               return 'normal'  # Default

       def assess_visibility_conditions(self):
           if self.laser_scan is not None:
               # Calculate visibility based on scan quality
               valid_ranges = [r for r in self.laser_scan.ranges if not (np.isnan(r) or np.isinf(r))]
               if len(valid_ranges) < len(self.laser_scan.ranges) * 0.5:
                   return 'poor'
               else:
                   return 'good'
           else:
               return 'good'  # Default

       def adapt_to_environment(self):
           # Update environmental conditions
           self.current_env_conditions['lighting'] = self.assess_lighting_conditions()
           self.current_env_conditions['temperature'] = self.assess_temperature_conditions()
           self.current_env_conditions['visibility'] = self.assess_visibility_conditions()

           # Adjust parameters based on conditions
           if self.current_env_conditions['lighting'] == 'low':
               # Increase brightness and contrast for low light
               self.adaptation_params['image_brightness'] = 1.5
               self.adaptation_params['image_contrast'] = 1.2
               self.adaptation_params['processing_rate'] = 5.0  # Lower rate to process more carefully
           elif self.current_env_conditions['lighting'] == 'bright':
               # Reduce brightness to avoid saturation
               self.adaptation_params['image_brightness'] = 0.8
               self.adaptation_params['image_contrast'] = 0.9
           else:
               # Normal conditions
               self.adaptation_params['image_brightness'] = 1.0
               self.adaptation_params['image_contrast'] = 1.0
               self.adaptation_params['processing_rate'] = 10.0

           if self.current_env_conditions['temperature'] == 'hot':
               # Reduce processing rate to prevent overheating
               self.adaptation_params['processing_rate'] = max(2.0, self.adaptation_params['processing_rate'] * 0.7)
           elif self.current_env_conditions['temperature'] == 'cold':
               # May need to warm up sensors
               pass

           if self.current_env_conditions['visibility'] == 'poor':
               # Increase scan sensitivity
               self.adaptation_params['scan_range_threshold'] = 15.0  # Shorter range for poor visibility

       def environmental_assessment(self):
           # Assess current environmental conditions
           self.adapt_to_environment()

           # Publish adaptation parameters
           params_msg = String()
           params_msg.data = json.dumps({
               'conditions': self.current_env_conditions,
               'adaptations': self.adaptation_params,
               'timestamp': self.get_clock().now().nanoseconds
           })
           self.adaptation_pub.publish(params_msg)

           # Publish environment status
           status_msg = String()
           status_msg.data = json.dumps(self.current_env_conditions)
           self.env_status_pub.publish(status_msg)

           self.get_logger().info(f'Environment: {self.current_env_conditions}, Adaptations: {self.adaptation_params}')

   def main(args=None):
       rclpy.init(args=args)
       env_node = EnvironmentalAdaptationNode()

       try:
           rclpy.spin(env_node)
       except KeyboardInterrupt:
           pass
       finally:
           env_node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

## Exercise 3: Communication and Power Management (2 hours)

### Task 3.1: Communication in Remote Locations
1. Create a communication management system for field robotics:
   ```python
   # field_robotics_lab/comm_manager.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String, Int8, Float32
   import json
   import time
   from collections import deque
   import threading

   class CommunicationManagerNode(Node):
       def __init__(self):
           super().__init__('comm_manager_node')

           # Communication status
           self.comm_status = {
               'connection_type': 'none',  # 'cellular', 'satellite', 'mesh', 'none'
               'signal_strength': 0.0,
               'bandwidth': 0.0,  # kbps
               'latency': 999.0,  # ms
               'available': False
           }

           # Data queues for different priority levels
           self.critical_data_queue = deque(maxlen=100)  # High priority
           self.standard_data_queue = deque(maxlen=1000)  # Medium priority
           self.bulk_data_queue = deque(maxlen=10000)  # Low priority

           # Communication parameters
           self.data_transmission_rate = 0.0  # kbps
           self.last_transmission_time = 0
           self.transmission_log = []

           # Publishers
           self.comm_status_pub = self.create_publisher(String, 'comm_status', 10)
           self.data_priority_pub = self.create_publisher(Int8, 'data_priority', 10)

           # Subscribers
           self.critical_data_sub = self.create_subscription(
               String, 'critical_data', self.critical_data_callback, 10)
           self.standard_data_sub = self.create_subscription(
               String, 'standard_data', self.standard_data_callback, 10)
           self.bulk_data_sub = self.create_subscription(
               String, 'bulk_data', self.bulk_data_callback, 10)

           # Timer for communication management
           self.comm_timer = self.create_timer(5.0, self.manage_communication)

           self.get_logger().info('Communication Manager Node Started')

       def critical_data_callback(self, msg):
           # Add critical data to high-priority queue
           data_item = {
               'data': msg.data,
               'timestamp': time.time(),
               'priority': 'critical'
           }
           self.critical_data_queue.append(data_item)

       def standard_data_callback(self, msg):
           # Add standard data to medium-priority queue
           data_item = {
               'data': msg.data,
               'timestamp': time.time(),
               'priority': 'standard'
           }
           self.standard_data_queue.append(data_item)

       def bulk_data_callback(self, msg):
           # Add bulk data to low-priority queue
           data_item = {
               'data': msg.data,
               'timestamp': time.time(),
               'priority': 'bulk'
           }
           self.bulk_data_queue.append(data_item)

       def simulate_communication_status(self):
           # Simulate changing communication conditions
           # In real implementation, this would interface with actual communication modules
           import random

           # Randomly change communication status
           if random.random() < 0.1:  # 10% chance of status change
               comm_types = ['cellular', 'satellite', 'mesh', 'none']
               self.comm_status['connection_type'] = random.choice(comm_types)

               if self.comm_status['connection_type'] == 'cellular':
                   self.comm_status['signal_strength'] = random.uniform(0.3, 1.0)
                   self.comm_status['bandwidth'] = random.uniform(100, 1000)
                   self.comm_status['latency'] = random.uniform(20, 200)
                   self.comm_status['available'] = True
               elif self.comm_status['connection_type'] == 'satellite':
                   self.comm_status['signal_strength'] = random.uniform(0.1, 0.6)
                   self.comm_status['bandwidth'] = random.uniform(10, 100)
                   self.comm_status['latency'] = random.uniform(500, 1500)
                   self.comm_status['available'] = True
               elif self.comm_status['connection_type'] == 'mesh':
                   self.comm_status['signal_strength'] = random.uniform(0.4, 0.9)
                   self.comm_status['bandwidth'] = random.uniform(50, 500)
                   self.comm_status['latency'] = random.uniform(10, 100)
                   self.comm_status['available'] = True
               else:  # none
                   self.comm_status['signal_strength'] = 0.0
                   self.comm_status['bandwidth'] = 0.0
                   self.comm_status['latency'] = 999.0
                   self.comm_status['available'] = False

       def transmit_data(self):
           # Transmit data based on available bandwidth and priority
           if not self.comm_status['available']:
               return 0  # No transmission possible

           # Calculate available transmission capacity (in KB per cycle)
           available_capacity = (self.comm_status['bandwidth'] * 1024 / 8) * (5.0 / 1000)  # 5 seconds worth

           transmitted_bytes = 0

           # Transmit critical data first
           while self.critical_data_queue and transmitted_bytes < available_capacity:
               data_item = self.critical_data_queue.popleft()
               data_size = len(data_item['data'].encode('utf-8'))
               if transmitted_bytes + data_size <= available_capacity:
                   transmitted_bytes += data_size
                   self.transmission_log.append({
                       'data': data_item['data'],
                       'priority': 'critical',
                       'timestamp': time.time()
                   })
                   self.get_logger().info(f'Transmitted critical data: {data_item["data"][:50]}...')

           # Transmit standard data if capacity remains
           while self.standard_data_queue and transmitted_bytes < available_capacity:
               data_item = self.standard_data_queue.popleft()
               data_size = len(data_item['data'].encode('utf-8'))
               if transmitted_bytes + data_size <= available_capacity:
                   transmitted_bytes += data_size
                   self.transmission_log.append({
                       'data': data_item['data'],
                       'priority': 'standard',
                       'timestamp': time.time()
                   })

           # Transmit bulk data only if connection is good and other data is transmitted
           if (self.comm_status['connection_type'] in ['cellular', 'mesh'] and
               transmitted_bytes < available_capacity * 0.8):  # Only if we have capacity left
               while self.bulk_data_queue and transmitted_bytes < available_capacity:
                   data_item = self.bulk_data_queue.popleft()
                   data_size = len(data_item['data'].encode('utf-8'))
                   if transmitted_bytes + data_size <= available_capacity:
                       transmitted_bytes += data_size
                       self.transmission_log.append({
                           'data': data_item['data'],
                           'priority': 'bulk',
                           'timestamp': time.time()
                       })

           return transmitted_bytes

       def manage_communication(self):
           # Simulate communication status changes
           self.simulate_communication_status()

           # Transmit data based on current conditions
           bytes_transmitted = self.transmit_data()

           # Update transmission rate
           time_now = time.time()
           if self.last_transmission_time > 0:
               time_diff = time_now - self.last_transmission_time
               self.data_transmission_rate = bytes_transmitted / time_diff if time_diff > 0 else 0
           self.last_transmission_time = time_now

           # Publish communication status
           status_msg = String()
           status_msg.data = json.dumps({
               'status': self.comm_status,
               'transmission_rate': self.data_transmission_rate,
               'queues': {
                   'critical': len(self.critical_data_queue),
                   'standard': len(self.standard_data_queue),
                   'bulk': len(self.bulk_data_queue)
               }
           })
           self.comm_status_pub.publish(status_msg)

           self.get_logger().info(f'Comm: {self.comm_status["connection_type"]}, '
                                  f'Bandwidth: {self.comm_status["bandwidth"]:.1f}kbps, '
                                  f'Transmitted: {bytes_transmitted} bytes')

   def main(args=None):
       rclpy.init(args=args)
       comm_node = CommunicationManagerNode()

       try:
           rclpy.spin(comm_node)
       except KeyboardInterrupt:
           pass
       finally:
           comm_node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Add the communication manager to `setup.py`.

### Task 3.2: Power Management System
1. Create a power management system for field robotics:
   ```python
   # field_robotics_lab/power_manager.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import Float32, String
   from sensor_msgs.msg import BatteryState
   import json
   import time
   from collections import deque

   class PowerManagerNode(Node):
       def __init__(self):
           super().__init__('power_manager_node')

           # Power system parameters
           self.battery_voltage = 0.0
           self.battery_current = 0.0
           self.battery_level = 0.0
           self.power_consumption = 0.0
           self.power_generation = 0.0  # From solar panels or other sources

           # Power management parameters
           self.power_budget = {
               'navigation': 50.0,  # watts
               'perception': 100.0,
               'communication': 20.0,
               'computation': 80.0,
               'actuation': 150.0
           }

           # Power modes
           self.power_mode = 'normal'  # 'high', 'normal', 'conservation', 'emergency'
           self.power_mode_thresholds = {
               'conservation': 0.3,  # 30% battery
               'emergency': 0.15     # 15% battery
           }

           # Power consumption tracking
           self.consumption_history = deque(maxlen=100)
           self.last_power_update = time.time()

           # Publishers
           self.power_mode_pub = self.create_publisher(String, 'power_mode', 10)
           self.power_status_pub = self.create_publisher(String, 'power_status', 10)
           self.power_command_pub = self.create_publisher(Float32, 'power_command', 10)

           # Subscribers
           self.battery_sub = self.create_subscription(
               BatteryState, '/battery_state', self.battery_callback, 10)

           # Timer for power management
           self.power_timer = self.create_timer(1.0, self.power_management_loop)

           self.get_logger().info('Power Manager Node Started')

       def battery_callback(self, msg):
           self.battery_voltage = msg.voltage
           self.battery_current = msg.current  # Negative when discharging
           self.battery_level = msg.percentage
           self.power_consumption = abs(self.battery_voltage * self.battery_current)

           # Add to consumption history
           self.consumption_history.append({
               'time': time.time(),
               'power': self.power_consumption,
               'level': self.battery_level
           })

       def calculate_power_mode(self):
           # Determine power mode based on battery level and consumption rate
           if self.battery_level <= self.power_mode_thresholds['emergency']:
               return 'emergency'
           elif self.battery_level <= self.power_mode_thresholds['conservation']:
               return 'conservation'
           else:
               # Check consumption rate trends
               if len(self.consumption_history) >= 10:
                   recent_avg = sum([item['power'] for item in list(self.consumption_history)[-10:]]) / 10
                   if recent_avg > sum([v for v in self.power_budget.values()]) * 0.8:  # 80% of max
                       return 'conservation'

               return 'normal'

       def get_power_allocation(self, mode):
           # Define power allocation based on mode
           allocations = {
               'high': {
                   'navigation': 50.0,
                   'perception': 100.0,
                   'communication': 20.0,
                   'computation': 80.0,
                   'actuation': 150.0
               },
               'normal': {
                   'navigation': 40.0,
                   'perception': 80.0,
                   'communication': 15.0,
                   'computation': 60.0,
                   'actuation': 120.0
               },
               'conservation': {
                   'navigation': 20.0,
                   'perception': 40.0,
                   'communication': 5.0,
                   'computation': 30.0,
                   'actuation': 60.0
               },
               'emergency': {
                   'navigation': 10.0,
                   'perception': 10.0,
                   'communication': 2.0,
                   'computation': 10.0,
                   'actuation': 30.0
               }
           }
           return allocations.get(mode, allocations['normal'])

       def estimate_runtime(self):
           # Estimate remaining runtime based on current consumption
           if not self.consumption_history or self.power_consumption <= 0:
               return 0.0

           # Calculate average consumption over recent period
           recent_consumption = [item['power'] for item in list(self.consumption_history)[-10:]]
           avg_consumption = sum(recent_consumption) / len(recent_consumption) if recent_consumption else 0.0

           # Calculate remaining energy (assuming linear discharge)
           remaining_energy = self.battery_level * 100  # Simplified model

           if avg_consumption > 0:
               estimated_runtime_hours = remaining_energy / avg_consumption
               return estimated_runtime_hours
           else:
               return float('inf')

       def power_management_loop(self):
           # Update power mode based on current conditions
           new_power_mode = self.calculate_power_mode()
           power_allocation = self.get_power_allocation(new_power_mode)

           # Publish power mode
           mode_msg = String()
           mode_msg.data = new_power_mode
           self.power_mode_pub.publish(mode_msg)

           # Publish power status
           status_msg = String()
           status_msg.data = json.dumps({
               'mode': new_power_mode,
               'allocation': power_allocation,
               'battery_level': self.battery_level,
               'consumption': self.power_consumption,
               'estimated_runtime_hours': self.estimate_runtime(),
               'timestamp': time.time()
           })
           self.power_status_pub.publish(status_msg)

           # Log power status
           self.get_logger().info(f'Power Mode: {new_power_mode}, '
                                  f'Battery: {self.battery_level:.1f}%, '
                                  f'Consumption: {self.power_consumption:.1f}W, '
                                  f'Runtime: {self.estimate_runtime():.1f}h')

           # Update current mode
           self.power_mode = new_power_mode

   def main(args=None):
       rclpy.init(args=args)
       power_node = PowerManagerNode()

       try:
           rclpy.spin(power_node)
       except KeyboardInterrupt:
           pass
       finally:
           power_node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

## Exercise 4: Reliability and Safety Systems (1 hour)

### Task 4.1: System Health Monitoring
1. Create a comprehensive health monitoring system:
   ```python
   # field_robotics_lab/health_monitor.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String, Float32
   import json
   import time
   from collections import deque

   class HealthMonitorNode(Node):
       def __init__(self):
           super().__init__('health_monitor_node')

           # System health parameters
           self.system_health = {
               'overall_health': 1.0,
               'mechanical_health': 1.0,
               'electrical_health': 1.0,
               'software_health': 1.0,
               'environmental_health': 1.0
           }

           # Component health tracking
           self.component_health = {
               'motors': {'health': 1.0, 'temperature': 25.0, 'vibration': 0.1},
               'sensors': {'health': 1.0, 'accuracy': 1.0, 'coverage': 1.0},
               'computing': {'health': 1.0, 'temperature': 40.0, 'utilization': 0.5},
               'communication': {'health': 1.0, 'signal_strength': 1.0, 'reliability': 1.0}
           }

           # Health history for trend analysis
           self.health_history = deque(maxlen=1000)

           # Publishers
           self.health_pub = self.create_publisher(String, 'system_health', 10)
           self.alert_pub = self.create_publisher(String, 'system_alerts', 10)

           # Timer for health monitoring
           self.health_timer = self.create_timer(2.0, self.health_monitoring_loop)

           self.get_logger().info('Health Monitor Node Started')

       def calculate_component_health(self):
           # Calculate health for each component based on various metrics
           for comp_name, comp_data in self.component_health.items():
               health_score = 1.0

               # Mechanical health factors
               if comp_name == 'motors':
                   # Temperature-based degradation
                   temp_factor = max(0, 1 - (comp_data['temperature'] - 25) / 50)
                   # Vibration-based degradation
                   vib_factor = max(0, 1 - comp_data['vibration'])
                   health_score = min(temp_factor, vib_factor)

               elif comp_name == 'sensors':
                   health_score = comp_data['accuracy'] * comp_data['coverage']

               elif comp_name == 'computing':
                   # Temperature and utilization factors
                   temp_factor = max(0, 1 - (comp_data['temperature'] - 30) / 60)
                   util_factor = max(0, 1 - comp_data['utilization'])
                   health_score = min(temp_factor, util_factor)

               elif comp_name == 'communication':
                   health_score = comp_data['signal_strength'] * comp_data['reliability']

               # Update component health
               self.component_health[comp_name]['health'] = max(0.0, min(1.0, health_score))

       def calculate_system_health(self):
           # Calculate overall system health as weighted average
           weights = {'motors': 0.3, 'sensors': 0.25, 'computing': 0.25, 'communication': 0.2}
           total_health = 0.0
           total_weight = 0.0

           for comp_name, weight in weights.items():
               if comp_name in self.component_health:
                   total_health += self.component_health[comp_name]['health'] * weight
                   total_weight += weight

           self.system_health['overall_health'] = total_health / total_weight if total_weight > 0 else 1.0

           # Calculate subsystem health
           self.system_health['mechanical_health'] = self.component_health['motors']['health']
           self.system_health['electrical_health'] = (self.component_health['sensors']['health'] +
                                                     self.component_health['computing']['health']) / 2
           self.system_health['software_health'] = self.component_health['computing']['health']
           self.system_health['environmental_health'] = 1.0  # Simplified

       def generate_alerts(self):
           alerts = []

           # Check for critical issues
           if self.system_health['overall_health'] < 0.3:
               alerts.append("CRITICAL_SYSTEM_FAILURE")
           elif self.system_health['overall_health'] < 0.6:
               alerts.append("SYSTEM_DEGRADED")

           # Check individual components
           for comp_name, comp_data in self.component_health.items():
               if comp_data['health'] < 0.4:
                   alerts.append(f"CRITICAL_{comp_name.upper()}_FAILURE")
               elif comp_data['health'] < 0.7:
                   alerts.append(f"{comp_name.upper()}_DEGRADED")

           # Publish alerts
           for alert in alerts:
               alert_msg = String()
               alert_msg.data = alert
               self.alert_pub.publish(alert_msg)
               self.get_logger().warn(f'System Alert: {alert}')

       def health_monitoring_loop(self):
           # Update component health
           self.calculate_component_health()

           # Calculate system health
           self.calculate_system_health()

           # Generate alerts if needed
           self.generate_alerts()

           # Store in history
           health_snapshot = {
               'timestamp': time.time(),
               'system_health': dict(self.system_health),
               'component_health': dict(self.component_health)
           }
           self.health_history.append(health_snapshot)

           # Publish health status
           health_msg = String()
           health_msg.data = json.dumps({
               'system_health': self.system_health,
               'component_health': self.component_health,
               'timestamp': time.time()
           })
           self.health_pub.publish(health_msg)

           self.get_logger().info(f'System Health: {self.system_health["overall_health"]:.2f}')

   def main(args=None):
       rclpy.init(args=args)
       health_node = HealthMonitorNode()

       try:
           rclpy.spin(health_node)
       except KeyboardInterrupt:
           pass
       finally:
           health_node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

## Assessment Questions

1. How would you design a fault-tolerant navigation system for field robotics in GPS-denied environments?
2. What strategies would you implement for energy harvesting in remote field robotics applications?
3. How would you ensure safe operation of field robots around humans and animals?
4. What communication protocols would you use for multi-robot coordination in remote locations?
5. How would you design a predictive maintenance system for field robotics?

## Troubleshooting Tips

- **Environmental sealing**: Ensure all connections are properly sealed
- **Power management**: Monitor consumption and plan for worst-case scenarios
- **Communication reliability**: Implement fallback communication methods
- **Sensor calibration**: Regularly check and recalibrate sensors
- **System monitoring**: Continuously monitor all subsystem health

## Extensions

1. Implement a multi-robot coordination system for field applications
2. Create an autonomous recharging station for field robots
3. Develop a remote monitoring and control system
4. Implement learning-based adaptation to environmental changes
5. Add regulatory compliance checking for different jurisdictions

## Summary

This lab exercise provided hands-on experience with field robotics applications and challenges. You implemented systems for agricultural monitoring, robust perception in challenging environments, communication management in remote locations, and power management for extended autonomy. These skills are essential for creating robots capable of operating in real-world field conditions.