# Week 3 Lab Exercise: Robot Perception and Sensor Integration

## Overview

In this lab exercise, you will implement robot perception systems and integrate various sensors into your ROS 2 framework. You will work with camera data, process LIDAR point clouds, implement computer vision algorithms with OpenCV, and use Rviz for visualization. This builds upon your Week 1-2 foundations to create robots that can perceive and understand their environment.

## Prerequisites

- Completion of Week 1 and Week 2 lab exercises
- Working ROS 2 Humble Hawksbill installation
- OpenCV installed (usually included with ROS 2)
- Working Week 1-2 ROS 2 workspace
- Basic understanding of Python and image processing concepts

## Learning Objectives

By completing this lab, you will:
- Integrate RGB and depth cameras into ROS 2 systems
- Process LIDAR data for environment perception
- Implement basic computer vision algorithms with OpenCV
- Work with point cloud data for 3D perception
- Use Rviz to visualize sensor data and perception results
- Design a simple sensor fusion approach

## Task 1: Camera Integration and Image Processing

### Step 1.1: Create a camera processing package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python camera_perception
```

### Step 1.2: Install required dependencies
```bash
sudo apt update
sudo apt install ros-humble-vision-opencv ros-humble-cv-bridge ros-humble-image-transport ros-humble-compressed-image-transport
```

### Step 1.3: Create a simple image subscriber
Create `~/ros2_ws/src/camera_perception/camera_perception/image_subscriber.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Perform basic image processing
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Display original and processed images
        cv2.imshow('Original', cv_image)
        cv2.imshow('Edges', edges)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()

    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        image_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 1.4: Create an image publisher for testing
Create `~/ros2_ws/src/camera_perception/camera_perception/image_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()

        # Create a test pattern for demonstration
        self.frame_count = 0

    def timer_callback(self):
        # Create a synthetic test image
        width, height = 640, 480
        image = np.zeros((height, width, 3), dtype=np.uint8)

        # Draw a moving circle
        center_x = int(50 + 50 * np.sin(self.frame_count * 0.1))
        center_y = int(50 + 50 * np.cos(self.frame_count * 0.1))
        cv2.circle(image, (center_x, center_y), 30, (0, 255, 0), -1)

        # Draw some text
        cv2.putText(image, f'Frame: {self.frame_count}', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # Convert to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = 'camera_frame'

        self.publisher_.publish(ros_image)
        self.frame_count += 1


def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()

    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        image_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 1.5: Create a feature detection node
Create `~/ros2_ws/src/camera_perception/camera_perception/feature_detector.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class FeatureDetector(Node):

    def __init__(self):
        super().__init__('feature_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.feature_pub = self.create_publisher(Image, '/camera/features', 10)

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to grayscale for processing
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect features using various methods
        # 1. Corner detection (Shi-Tomasi)
        corners = cv2.goodFeaturesToTrack(gray, maxCorners=100,
                                         qualityLevel=0.01, minDistance=10)

        # 2. Harris corner detection
        harris = cv2.cornerHarris(gray, 2, 3, 0.04)
        harris = cv2.dilate(harris, None)
        harris_img = cv_image.copy()
        harris_img[harris > 0.01 * harris.max()] = [0, 0, 255]  # Red for Harris corners

        # 3. SIFT (if available) - use ORB as alternative
        orb = cv2.ORB_create()
        keypoints, descriptors = orb.detectAndCompute(gray, None)

        # Draw features on the image
        feature_img = cv_image.copy()
        if corners is not None:
            corners = np.int0(corners)
            for corner in corners:
                x, y = corner.ravel()
                cv2.circle(feature_img, (x, y), 5, (0, 255, 0), -1)

        if keypoints is not None:
            feature_img = cv2.drawKeypoints(feature_img, keypoints, None,
                                          flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Publish the feature-detected image
        feature_ros_img = self.bridge.cv2_to_imgmsg(feature_img, encoding='bgr8')
        feature_ros_img.header = msg.header
        self.feature_pub.publish(feature_ros_img)

        # Display results
        cv2.imshow('Original', cv_image)
        cv2.imshow('Features', feature_img)
        cv2.imshow('Harris Corners', harris_img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    feature_detector = FeatureDetector()

    try:
        rclpy.spin(feature_detector)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        feature_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 1.6: Update setup.py for camera_perception package
Edit `~/ros2_ws/src/camera_perception/setup.py` to add entry points:

```python
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'camera_perception'

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
    description='Camera perception examples for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = camera_perception.image_publisher:main',
            'image_subscriber = camera_perception.image_subscriber:main',
            'feature_detector = camera_perception.feature_detector:main',
        ],
    },
)
```

## Task 2: LIDAR Data Processing

### Step 2.1: Create LIDAR processing package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python lidar_perception
```

### Step 2.2: Create a LIDAR subscriber
Create `~/ros2_ws/src/lidar_perception/lidar_perception/lidar_subscriber.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np


class LidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.subscription  # prevent unused variable warning

    def lidar_callback(self, msg):
        # Extract range data
        ranges = np.array(msg.ranges)

        # Filter out invalid measurements (inf, nan)
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) > 0:
            # Calculate basic statistics
            min_range = np.min(valid_ranges)
            max_range = np.max(valid_ranges)
            avg_range = np.mean(valid_ranges)

            # Detect obstacles (objects closer than threshold)
            obstacle_threshold = 1.0  # meters
            obstacles = valid_ranges < obstacle_threshold
            obstacle_count = np.sum(obstacles)

            # Calculate angles for each range measurement
            angle_increment = msg.angle_increment
            angles = np.arange(msg.angle_min, msg.angle_max, angle_increment)
            if len(angles) != len(ranges):
                # Handle the case where the last measurement is included
                angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

            # Find the closest obstacle and its angle
            if np.any(obstacles):
                obstacle_ranges = ranges[obstacles]
                obstacle_angles = angles[obstacles]
                closest_obstacle_idx = np.argmin(obstacle_ranges)
                closest_distance = obstacle_ranges[closest_obstacle_idx]
                closest_angle = obstacle_angles[closest_obstacle_idx]

                self.get_logger().info(
                    f'LIDAR: Min={min_range:.2f}m, Max={max_range:.2f}m, '
                    f'Avg={avg_range:.2f}m, Obstacles={obstacle_count}, '
                    f'Closest: {closest_distance:.2f}m at {closest_angle:.2f}rad'
                )
            else:
                self.get_logger().info(
                    f'LIDAR: Min={min_range:.2f}m, Max={max_range:.2f}m, '
                    f'Avg={avg_range:.2f}m, No obstacles detected'
                )
        else:
            self.get_logger().info('LIDAR: No valid measurements')


def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()

    try:
        rclpy.spin(lidar_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 2.3: Create a LIDAR simulator
Create `~/ros2_ws/src/lidar_perception/lidar_perception/lidar_simulator.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math


class LidarSimulator(Node):

    def __init__(self):
        super().__init__('lidar_simulator')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # LIDAR parameters
        self.angle_min = -math.pi / 2  # -90 degrees
        self.angle_max = math.pi / 2   # 90 degrees
        self.angle_increment = math.pi / 180  # 1 degree
        self.scan_time = 0.1
        self.range_min = 0.1
        self.range_max = 10.0
        self.time_increment = 0.0

        # Simulate some obstacles
        self.sim_time = 0.0

    def timer_callback(self):
        # Create simulated LIDAR scan
        num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment) + 1

        # Initialize ranges with max distance (free space)
        ranges = np.full(num_ranges, self.range_max, dtype=np.float32)

        # Add simulated obstacles
        angles = np.linspace(self.angle_min, self.angle_max, num_ranges)

        # Add a wall at 2m straight ahead
        for i, angle in enumerate(angles):
            if abs(angle) < 0.1:  # Straight ahead
                ranges[i] = 2.0 + 0.1 * np.sin(self.sim_time * 2)  # Slight movement

        # Add a cylindrical obstacle to the right
        for i, angle in enumerate(angles):
            if 0.3 < angle < 0.6:  # Right side
                distance = 1.5 + 0.2 * np.sin(self.sim_time * 3)
                ranges[i] = min(ranges[i], distance)

        # Add some noise to simulate real sensor
        noise = np.random.normal(0, 0.02, size=ranges.shape)
        ranges = np.clip(ranges + noise, self.range_min, self.range_max)

        # Create LaserScan message
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_frame'
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment = self.time_increment
        scan_msg.scan_time = self.scan_time
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        scan_msg.ranges = ranges.tolist()

        self.publisher_.publish(scan_msg)
        self.sim_time += self.timer.timer_period_ns / 1e9


def main(args=None):
    rclpy.init(args=args)
    lidar_simulator = LidarSimulator()

    try:
        rclpy.spin(lidar_simulator)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 2.4: Update setup.py for lidar_perception package
Edit `~/ros2_ws/src/lidar_perception/setup.py` to add entry points:

```python
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'lidar_perception'

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
    description='LIDAR perception examples for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_simulator = lidar_perception.lidar_simulator:main',
            'lidar_subscriber = lidar_perception.lidar_subscriber:main',
        ],
    },
)
```

## Task 3: Point Cloud Processing

### Step 3.1: Install PCL dependencies
```bash
sudo apt install ros-humble-pcl-conversions ros-humble-pcl-ros
```

### Step 3.2: Create point cloud processing package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python pointcloud_perception
```

### Step 3.3: Create a point cloud processor
Create `~/ros2_ws/src/pointcloud_perception/pointcloud_perception/pointcloud_processor.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import numpy as np
import struct


class PointCloudProcessor(Node):

    def __init__(self):
        super().__init__('pointcloud_processor')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/pointcloud',
            self.pointcloud_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(PointCloud2, '/filtered_pointcloud', 10)

    def pointcloud_callback(self, msg):
        self.get_logger().info(f'Received point cloud with {msg.height * msg.width} points')

        # Convert PointCloud2 to list of points
        points_list = []
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([point[0], point[1], point[2]])

        if points_list:
            points = np.array(points_list)

            # Basic statistics
            self.get_logger().info(f'Point cloud statistics:')
            self.get_logger().info(f'  X: min={np.min(points[:, 0]):.2f}, max={np.max(points[:, 0]):.2f}')
            self.get_logger().info(f'  Y: min={np.min(points[:, 1]):.2f}, max={np.max(points[:, 1]):.2f}')
            self.get_logger().info(f'  Z: min={np.min(points[:, 2]):.2f}, max={np.max(points[:, 2]):.2f}')

            # Filter points (example: remove points above a certain height)
            height_threshold = 1.0  # meter
            filtered_points = points[points[:, 2] < height_threshold]

            self.get_logger().info(f'Filtered {len(points)} -> {len(filtered_points)} points')

            # Create filtered point cloud message
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = msg.header.frame_id

            # Convert back to PointCloud2 format
            filtered_cloud = self.create_pointcloud2(header, filtered_points)
            self.publisher.publish(filtered_cloud)

    def create_pointcloud2(self, header, points):
        """Create a PointCloud2 message from numpy array of points."""
        # Define the format of the point cloud
        fields = [
            point_cloud2.PointField(name='x', offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='y', offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='z', offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1),
        ]

        # Convert points to list of tuples
        points_tuple = [tuple(point) for point in points]

        # Create PointCloud2 message
        return point_cloud2.create_cloud(header, fields, points_tuple)


def main(args=None):
    rclpy.init(args=args)
    pointcloud_processor = PointCloudProcessor()

    try:
        rclpy.spin(pointcloud_processor)
    except KeyboardInterrupt:
        pass
    finally:
        pointcloud_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 3.4: Create a point cloud simulator
Create `~/ros2_ws/src/pointcloud_perception/pointcloud_perception/pointcloud_simulator.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import numpy as np
import struct


class PointCloudSimulator(Node):

    def __init__(self):
        super().__init__('pointcloud_simulator')
        self.publisher_ = self.create_publisher(PointCloud2, '/pointcloud', 10)
        timer_period = 0.5  # seconds (2 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sim_time = 0.0

    def timer_callback(self):
        # Generate simulated point cloud data
        # Create a grid of points with some objects
        x = np.linspace(-2, 2, 20)
        y = np.linspace(-2, 2, 20)
        X, Y = np.meshgrid(x, y)
        Z = np.zeros_like(X)

        # Add a plane with some height variation
        Z += 0.1 * np.sin(X * 2) * np.cos(Y * 2)

        # Add some obstacles (cylinders)
        center_x, center_y = 0.5, 0.5
        radius = 0.5
        mask = (X - center_x)**2 + (Y - center_y)**2 < radius**2
        Z[mask] = 1.0  # Height of obstacle

        # Flatten the grid to get points
        points = np.column_stack([X.ravel(), Y.ravel(), Z.ravel()])

        # Add some noise
        points += 0.01 * np.random.randn(*points.shape)

        # Add some random points to simulate noise
        n_noise = 50
        noise_points = np.random.uniform([-3, -3, -1], [3, 3, 2], (n_noise, 3))
        points = np.vstack([points, noise_points])

        # Create PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'sensor_frame'

        # Define fields
        fields = [
            point_cloud2.PointField(name='x', offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='y', offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='z', offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1),
        ]

        # Convert to PointCloud2
        points_tuple = [tuple(point) for point in points]
        pointcloud_msg = point_cloud2.create_cloud(header, fields, points_tuple)

        self.publisher_.publish(pointcloud_msg)
        self.get_logger().info(f'Published point cloud with {len(points)} points')

        self.sim_time += timer_period


def main(args=None):
    rclpy.init(args=args)
    pointcloud_simulator = PointCloudSimulator()

    try:
        rclpy.spin(pointcloud_simulator)
    except KeyboardInterrupt:
        pass
    finally:
        pointcloud_simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 3.5: Update setup.py for pointcloud_perception package
Edit `~/ros2_ws/src/pointcloud_perception/setup.py` to add entry points:

```python
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'pointcloud_perception'

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
    description='Point cloud perception examples for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_simulator = pointcloud_perception.pointcloud_simulator:main',
            'pointcloud_processor = pointcloud_perception.pointcloud_processor:main',
        ],
    },
)
```

## Task 4: Sensor Fusion Implementation

### Step 4.1: Create a sensor fusion package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python sensor_fusion
```

### Step 4.2: Create a simple sensor fusion node
Create `~/ros2_ws/src/sensor_fusion/sensor_fusion/simple_fusion.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from sensor_msgs_py import point_cloud2
import numpy as np
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class SimpleFusion(Node):

    def __init__(self):
        super().__init__('simple_fusion')

        # Subscriptions for different sensor types
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)

        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            '/pointcloud',
            self.pointcloud_callback,
            10)

        # Publisher for fused results
        self.obstacle_publisher = self.create_publisher(
            PointStamped, '/obstacles', 10)

        self.fused_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/fused_position', 10)

        # TF buffer for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Store sensor data
        self.lidar_data = None
        self.pointcloud_data = None

        # Timer for fusion processing
        self.timer = self.create_timer(0.5, self.fusion_callback)

    def lidar_callback(self, msg):
        # Store LIDAR data
        self.lidar_data = msg
        self.get_logger().debug(f'Received LIDAR data with {len(msg.ranges)} points')

    def pointcloud_callback(self, msg):
        # Store point cloud data
        self.pointcloud_data = msg
        self.get_logger().debug(f'Received point cloud with {msg.height * msg.width} points')

    def fusion_callback(self):
        # Perform simple fusion if we have data from both sensors
        if self.lidar_data is not None and self.pointcloud_data is not None:
            # Process LIDAR data to find closest obstacle
            valid_ranges = np.array(self.lidar_data.ranges)
            valid_ranges = valid_ranges[np.isfinite(valid_ranges)]

            if len(valid_ranges) > 0:
                closest_distance = np.min(valid_ranges)

                # Create an obstacle message based on LIDAR data
                obstacle_msg = PointStamped()
                obstacle_msg.header.stamp = self.get_clock().now().to_msg()
                obstacle_msg.header.frame_id = self.lidar_data.header.frame_id

                # For simplicity, assume closest obstacle is at 0 degrees
                angle_increment = self.lidar_data.angle_increment
                closest_idx = np.argmin(np.array(self.lidar_data.ranges)[np.isfinite(self.lidar_data.ranges)])
                closest_angle = self.lidar_data.angle_min + closest_idx * angle_increment

                # Convert polar to Cartesian
                obstacle_msg.point.x = closest_distance * np.cos(closest_angle)
                obstacle_msg.point.y = closest_distance * np.sin(closest_angle)
                obstacle_msg.point.z = 0.0  # Assume ground level

                self.obstacle_publisher.publish(obstacle_msg)
                self.get_logger().info(
                    f'Fused obstacle detection: distance={closest_distance:.2f}m, '
                    f'angle={closest_angle:.2f}rad'
                )

            # Process point cloud data
            try:
                points_list = []
                for point in point_cloud2.read_points(
                    self.pointcloud_data,
                    field_names=("x", "y", "z"),
                    skip_nans=True
                ):
                    points_list.append([point[0], point[1], point[2]])

                if points_list:
                    points = np.array(points_list)

                    # Calculate center of point cloud
                    center = np.mean(points, axis=0)

                    # Create fused position estimate
                    fused_pose = PoseWithCovarianceStamped()
                    fused_pose.header.stamp = self.get_clock().now().to_msg()
                    fused_pose.header.frame_id = self.pointcloud_data.header.frame_id

                    fused_pose.pose.pose.position.x = float(center[0])
                    fused_pose.pose.pose.position.y = float(center[1])
                    fused_pose.pose.pose.position.z = float(center[2])

                    # Simple covariance (would be more sophisticated in real system)
                    fused_pose.pose.covariance[0] = 0.1  # x variance
                    fused_pose.pose.covariance[7] = 0.1  # y variance
                    fused_pose.pose.covariance[14] = 0.1  # z variance

                    self.fused_publisher.publish(fused_pose)
                    self.get_logger().info(f'Fused position: ({center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f})')

            except Exception as e:
                self.get_logger().error(f'Error processing point cloud: {e}')

        elif self.lidar_data is not None:
            # Process LIDAR-only data
            valid_ranges = np.array(self.lidar_data.ranges)
            valid_ranges = valid_ranges[np.isfinite(valid_ranges)]

            if len(valid_ranges) > 0:
                closest_distance = np.min(valid_ranges)
                self.get_logger().info(f'LIDAR-only: closest obstacle at {closest_distance:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    simple_fusion = SimpleFusion()

    try:
        rclpy.spin(simple_fusion)
    except KeyboardInterrupt:
        pass
    finally:
        simple_fusion.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 4.3: Update setup.py for sensor_fusion package
Edit `~/ros2_ws/src/sensor_fusion/setup.py` to add entry points:

```python
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'sensor_fusion'

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
    description='Sensor fusion examples for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_fusion = sensor_fusion.simple_fusion:main',
        ],
    },
)
```

## Task 5: Create a launch file for the perception system

### Step 5.1: Create launch directory and file
```bash
mkdir -p ~/ros2_ws/src/sensor_fusion/launch
```

Create `~/ros2_ws/src/sensor_fusion/launch/perception_system.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        # Camera simulation and processing
        Node(
            package='camera_perception',
            executable='image_publisher',
            name='image_publisher',
            output='screen'
        ),
        Node(
            package='camera_perception',
            executable='feature_detector',
            name='feature_detector',
            output='screen'
        ),

        # LIDAR simulation and processing
        Node(
            package='lidar_perception',
            executable='lidar_simulator',
            name='lidar_simulator',
            output='screen'
        ),
        Node(
            package='lidar_perception',
            executable='lidar_subscriber',
            name='lidar_subscriber',
            output='screen'
        ),

        # Point cloud processing
        Node(
            package='pointcloud_perception',
            executable='pointcloud_simulator',
            name='pointcloud_simulator',
            output='screen'
        ),
        Node(
            package='pointcloud_perception',
            executable='pointcloud_processor',
            name='pointcloud_processor',
            output='screen'
        ),

        # Sensor fusion
        Node(
            package='sensor_fusion',
            executable='simple_fusion',
            name='simple_fusion',
            output='screen'
        ),
    ])
```

## Task 6: Build and Test the System

### Step 6.1: Build all perception packages
```bash
cd ~/ros2_ws
colcon build --packages-select camera_perception lidar_perception pointcloud_perception sensor_fusion
source install/setup.bash
```

### Step 6.2: Test the perception system
```bash
ros2 launch sensor_fusion perception_system.launch.py
```

### Step 6.3: Visualize with Rviz (in a separate terminal)
```bash
# First, install Rviz if not already installed
sudo apt install ros-humble-rviz2

# Then run Rviz
rviz2
```

In Rviz, you can add displays for:
- Image: Subscribe to `/camera/features` to see processed camera images
- LaserScan: Subscribe to `/scan` to see LIDAR data
- PointCloud2: Subscribe to `/pointcloud` or `/filtered_pointcloud` to see point clouds

## Task 7: Use Rviz for Visualization

### Step 7.1: Create an Rviz configuration file
Create `~/ros2_ws/src/sensor_fusion/config/perception_demo.rviz`:

```yaml
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /Image1
        - /LaserScan1
        - /PointCloud21
      Splitter Ratio: 0.5
    Tree Height: 617
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Goal Pose1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/Image
      Enabled: true
      Max Value: 1
      Min Value: 0
      Name: Image
      Overlay Alpha: 0.5
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /camera/features
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 255; 255
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 0
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: LaserScan
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.009999999776482582
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /scan
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 255
      Color Transformer: RGB8
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: PointCloud2
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.009999999776482582
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /pointcloud
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: sensor_frame
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.5
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.5
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 846
  Hide Left Dock: false
  Hide Right Dock: false
  Image:
    collapsed: false
  QMainWindow State: 000000ff00000000fd000000040000000000000156000002f4fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000002f4000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa000025a9000002a0fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f000002f4fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073000000003d000002f4000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004420000000000000000fb0000000800540069006d00650100000000000004500000000000000000000003a3000002f400000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Width: 1200
  X: 60
  Y: 60
```

## Assessment Questions

1. Explain the differences between various sensor types (camera, LIDAR, IMU) and their appropriate use cases in robot perception.

2. Describe the process of sensor fusion and explain why it's important for robust robot perception.

3. What are the challenges of working with point cloud data, and how do you address them?

4. How does the ROS 2 image transport system help with efficient image transmission?

5. Explain the role of calibration in sensor integration and how it affects perception accuracy.

## Troubleshooting Tips

- If OpenCV is not found, ensure `ros-humble-vision-opencv` is installed
- If PCL functions don't work, verify `ros-humble-pcl-conversions` is installed
- For visualization issues, check that frame IDs match between sensors and TF
- If point cloud processing is slow, consider downsampling or using voxel filtering
- Use `ros2 doctor` to diagnose communication issues between nodes

## Summary

In this lab, you've implemented comprehensive robot perception systems:
- Camera integration with OpenCV-based image processing
- LIDAR data processing for environment scanning
- Point cloud processing for 3D scene understanding
- Sensor fusion to combine multiple sensor inputs
- Visualization tools to understand sensor data

These perception capabilities are essential for any autonomous robot system and form the foundation for more advanced robotics applications in subsequent weeks.