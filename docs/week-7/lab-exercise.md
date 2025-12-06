# Week 7 Lab Exercise: Machine Learning for Robotics

## Overview

In this lab exercise, you will implement deep learning models for robot perception, develop reinforcement learning algorithms for robot control, and apply imitation learning for robot skill acquisition. You will work with neural networks for vision processing, train RL agents in simulation, and integrate ML models with ROS 2 systems. This builds upon your Week 1-6 foundations to create intelligent robot systems that can learn and adapt.

## Prerequisites

- Completion of Week 1-6 lab exercises
- Working ROS 2 Humble Hawksbill installation
- Python development environment with ML libraries (TensorFlow/PyTorch)
- Robot simulation environment with sensor data
- Basic understanding of neural networks and ML concepts
- Experience with Python programming

## Learning Objectives

By completing this lab, you will:
- Implement deep learning models for robot perception tasks
- Apply reinforcement learning algorithms for robot control
- Develop learning-based navigation and path planning systems
- Use imitation learning for robot skill acquisition
- Integrate machine learning models with ROS 2 systems
- Evaluate and validate machine learning models in robotic applications

## Exercise 1: Deep Learning for Perception (2 hours)

### Task 1.1: Object Detection with CNN
1. Create a ROS 2 package for ML-based perception:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --dependencies rclpy sensor_msgs cv_bridge std_msgs --build-type ament_python ml_perception_lab
   cd ml_perception_lab
   ```

2. Install required Python packages:
   ```bash
   pip3 install torch torchvision torchaudio
   pip3 install opencv-python
   pip3 install numpy
   ```

3. Create a simple CNN-based object detection node `ml_perception_lab/object_detector.py`:
   ```python
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image
   from cv_bridge import CvBridge
   import cv2
   import torch
   import torch.nn as nn
   import numpy as np
   from std_msgs.msg import String

   class SimpleCNN(nn.Module):
       def __init__(self, num_classes=10):
           super(SimpleCNN, self).__init__()
           self.conv1 = nn.Conv2d(3, 32, kernel_size=3, padding=1)
           self.conv2 = nn.Conv2d(32, 64, kernel_size=3, padding=1)
           self.pool = nn.MaxPool2d(2, 2)
           self.fc1 = nn.Linear(64 * 8 * 8, 512)
           self.fc2 = nn.Linear(512, num_classes)
           self.dropout = nn.Dropout(0.5)

       def forward(self, x):
           x = self.pool(torch.relu(self.conv1(x)))
           x = self.pool(torch.relu(self.conv2(x)))
           x = x.view(-1, 64 * 8 * 8)
           x = torch.relu(self.fc1(x))
           x = self.dropout(x)
           x = self.fc2(x)
           return x

   class ObjectDetector(Node):
       def __init__(self):
           super().__init__('object_detector')

           # Initialize CV bridge
           self.bridge = CvBridge()

           # Load pre-trained model (for simulation, we'll create a simple one)
           self.model = SimpleCNN(num_classes=5)  # 5 object classes
           self.model.eval()  # Set to evaluation mode

           # Create subscription to camera feed
           self.subscription = self.create_subscription(
               Image,
               '/camera/image_raw',
               self.image_callback,
               10)

           # Create publisher for detection results
           self.detection_pub = self.create_publisher(String, 'object_detections', 10)

           self.get_logger().info('Object Detector Node Started')

       def image_callback(self, msg):
           try:
               # Convert ROS Image message to OpenCV image
               cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

               # Preprocess image for model
               img_resized = cv2.resize(cv_image, (32, 32))
               img_tensor = torch.from_numpy(img_resized).float().permute(2, 0, 1).unsqueeze(0) / 255.0

               # Run inference (simulated - in real scenario, use actual model)
               # For this example, we'll simulate detection results
               class_names = ['robot', 'obstacle', 'person', 'charger', 'tool']
               detections = []

               # Simulate object detection by finding contours in the image
               gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
               _, thresh = cv2.threshold(gray, 127, 255, 0)
               contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

               for i, contour in enumerate(contours[:3]):  # Limit to 3 detections
                   if cv2.contourArea(contour) > 100:  # Filter small contours
                       x, y, w, h = cv2.boundingRect(contour)
                       class_idx = i % len(class_names)
                       confidence = 0.8 + (i * 0.05)  # Simulated confidence

                       detection = {
                           'class': class_names[class_idx],
                           'confidence': confidence,
                           'bbox': [x, y, w, h]
                       }
                       detections.append(detection)

               # Publish detection results
               if detections:
                   result_str = f"Detections: {detections}"
                   self.detection_pub.publish(String(data=result_str))
                   self.get_logger().info(f'Published detections: {result_str}')

           except Exception as e:
               self.get_logger().error(f'Error processing image: {str(e)}')

   def main(args=None):
       rclpy.init(args=args)
       object_detector = ObjectDetector()

       try:
           rclpy.spin(object_detector)
       except KeyboardInterrupt:
           pass
       finally:
           object_detector.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

4. Update the package setup in `setup.py` to include the new executable:
   ```python
   from setuptools import setup

   package_name = 'ml_perception_lab'

   setup(
       name=package_name,
       version='0.0.0',
       packages=[package_name],
       data_files=[
           ('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='your_name',
       maintainer_email='your_email@example.com',
       description='ML Perception Lab Package',
       license='Apache License 2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'object_detector = ml_perception_lab.object_detector:main',
           ],
       },
   )
   ```

5. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ml_perception_lab
   source install/setup.bash
   ```

### Task 1.2: Semantic Segmentation
1. Create a semantic segmentation node that labels different regions in the camera image:
   ```python
   # ml_perception_lab/semantic_segmentation.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image
   from cv_bridge import CvBridge
   import cv2
   import numpy as np
   import torch
   import torch.nn as nn

   class SemanticSegmentationNode(Node):
       def __init__(self):
           super().__init__('semantic_segmentation')

           self.bridge = CvBridge()

           # Create a simple segmentation model (simulated)
           # In practice, you would load a pre-trained model

           self.subscription = self.create_subscription(
               Image,
               '/camera/image_raw',
               self.image_callback,
               10)

           self.segmentation_pub = self.create_publisher(Image, 'segmentation_result', 10)

           self.get_logger().info('Semantic Segmentation Node Started')

       def image_callback(self, msg):
           try:
               cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

               # Simulate semantic segmentation by applying color-based segmentation
               # This is a simple example - real segmentation would use a neural network
               hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

               # Define color ranges for different "classes"
               lower_floor = np.array([0, 0, 200])
               upper_floor = np.array([180, 50, 255])
               mask_floor = cv2.inRange(hsv, lower_floor, upper_floor)

               lower_obstacle = np.array([0, 50, 50])
               upper_obstacle = np.array([20, 255, 255])
               mask_obstacle = cv2.inRange(hsv, lower_obstacle, upper_obstacle)

               # Combine masks with different colors
               segmentation_result = np.zeros_like(cv_image)
               segmentation_result[mask_floor > 0] = [0, 255, 0]    # Green for floor
               segmentation_result[mask_obstacle > 0] = [0, 0, 255] # Red for obstacles

               # Convert back to ROS Image message
               segmented_msg = self.bridge.cv2_to_imgmsg(segmentation_result, "bgr8")
               segmented_msg.header = msg.header

               self.segmentation_pub.publish(segmented_msg)

           except Exception as e:
               self.get_logger().error(f'Error in segmentation: {str(e)}')

   def main(args=None):
       rclpy.init(args=args)
       seg_node = SemanticSegmentationNode()

       try:
           rclpy.spin(seg_node)
       except KeyboardInterrupt:
           pass
       finally:
           seg_node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Add the new executable to `setup.py` and rebuild the package.

## Exercise 2: Reinforcement Learning for Control (2.5 hours)

### Task 2.1: Q-Learning for Navigation
1. Create a reinforcement learning package:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --dependencies rclpy std_msgs geometry_msgs nav_msgs --build-type ament_python rl_navigation_lab
   cd rl_navigation_lab
   ```

2. Create a Q-learning navigation node `rl_navigation_lab/q_learning_navigation.py`:
   ```python
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from geometry_msgs.msg import Twist
   from nav_msgs.msg import Odometry
   from sensor_msgs.msg import LaserScan
   import numpy as np
   import random
   import math

   class QLearningNavigation(Node):
       def __init__(self):
           super().__init__('q_learning_navigation')

           # Q-table parameters
           self.state_size = (10, 10, 4)  # x, y, orientation
           self.action_size = 4  # forward, left, right, backward
           self.q_table = np.zeros(self.state_size + (self.action_size,))

           # Learning parameters
           self.learning_rate = 0.1
           self.discount_factor = 0.95
           self.epsilon = 1.0
           self.epsilon_decay = 0.995
           self.epsilon_min = 0.01

           # Robot state
           self.position_x = 0.0
           self.position_y = 0.0
           self.orientation = 0.0
           self.laser_ranges = []

           # Goal position
           self.goal_x = 5.0
           self.goal_y = 5.0

           # Publishers and subscribers
           self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
           self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
           self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

           # Timer for control loop
           self.control_timer = self.create_timer(0.1, self.control_loop)

           self.get_logger().info('Q-Learning Navigation Node Started')

       def odom_callback(self, msg):
           self.position_x = msg.pose.pose.position.x
           self.position_y = msg.pose.pose.position.y

           # Extract orientation from quaternion
           orientation_q = msg.pose.pose.orientation
           self.orientation = math.atan2(2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y),
                                              1.0 - 2.0 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z))

       def scan_callback(self, msg):
           self.laser_ranges = list(msg.ranges)

       def discretize_state(self):
           # Discretize continuous state space
           x_disc = min(int((self.position_x + 5) / 1), 9)  # Assuming workspace is 10x10
           y_disc = min(int((self.position_y + 5) / 1), 9)
           angle_disc = min(int((self.orientation + math.pi) / (math.pi / 2)), 3)

           # Ensure indices are within bounds
           x_disc = max(0, min(x_disc, 9))
           y_disc = max(0, min(y_disc, 9))
           angle_disc = max(0, min(angle_disc, 3))

           return (x_disc, y_disc, angle_disc)

       def get_reward(self):
           # Calculate distance to goal
           dist_to_goal = math.sqrt((self.position_x - self.goal_x)**2 + (self.position_y - self.goal_y)**2)

           # Reward based on distance to goal and obstacle avoidance
           reward = -dist_to_goal  # Negative distance as reward (closer is better)

           # Penalty for being close to obstacles
           if self.laser_ranges:
               min_range = min(self.laser_ranges)
               if min_range < 0.5:
                   reward -= 10  # Large penalty for being too close to obstacles
               elif min_range < 1.0:
                   reward -= 2   # Smaller penalty for being near obstacles

           # Bonus for reaching goal
           if dist_to_goal < 0.5:
               reward += 100

           return reward

       def select_action(self, state):
           if np.random.random() <= self.epsilon:
               # Explore: random action
               return random.choice(range(self.action_size))
           else:
               # Exploit: best known action
               return np.argmax(self.q_table[state])

       def control_loop(self):
           state = self.discretize_state()
           action = self.select_action(state)

           # Execute action
           cmd_msg = Twist()
           if action == 0:  # Forward
               cmd_msg.linear.x = 0.5
           elif action == 1:  # Turn left
               cmd_msg.angular.z = 0.5
           elif action == 2:  # Turn right
               cmd_msg.angular.z = -0.5
           elif action == 3:  # Backward
               cmd_msg.linear.x = -0.3

           self.cmd_vel_pub.publish(cmd_msg)

           # Get reward
           reward = self.get_reward()

           # Update Q-table
           next_state = self.discretize_state()
           best_next_action = np.argmax(self.q_table[next_state])
           td_target = reward + self.discount_factor * self.q_table[next_state + (best_next_action,)]
           td_error = td_target - self.q_table[state + (action,)]
           self.q_table[state + (action,)] += self.learning_rate * td_error

           # Decay epsilon
           if self.epsilon > self.epsilon_min:
               self.epsilon *= self.epsilon_decay

           # Log information
           self.get_logger().info(f'State: {state}, Action: {action}, Reward: {reward:.2f}, Epsilon: {self.epsilon:.3f}')

   def main(args=None):
       rclpy.init(args=args)
       rl_nav = QLearningNavigation()

       try:
           rclpy.spin(rl_nav)
       except KeyboardInterrupt:
           pass
       finally:
           rl_nav.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

3. Update `setup.py` and build the package:
   ```python
   entry_points={
       'console_scripts': [
           'q_learning_navigation = rl_navigation_lab.q_learning_navigation:main',
       ],
   },
   ```

### Task 2.2: Deep Q-Network (DQN) Simulation
1. Create a simplified DQN implementation that demonstrates the concepts:
   ```python
   # rl_navigation_lab/dqn_navigation.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   import torch
   import torch.nn as nn
   import torch.optim as optim
   import numpy as np
   import random
   from collections import deque
   from geometry_msgs.msg import Twist
   from nav_msgs.msg import Odometry
   import math

   class DQN(nn.Module):
       def __init__(self, state_size, action_size):
           super(DQN, self).__init__()
           self.fc1 = nn.Linear(state_size, 64)
           self.fc2 = nn.Linear(64, 64)
           self.fc3 = nn.Linear(64, action_size)

       def forward(self, x):
           x = torch.relu(self.fc1(x))
           x = torch.relu(self.fc2(x))
           return self.fc3(x)

   class DQNNavigation(Node):
       def __init__(self):
           super().__init__('dqn_navigation')

           # DQN parameters
           self.state_size = 4  # x, y, goal_x, goal_y
           self.action_size = 4
           self.memory = deque(maxlen=10000)
           self.epsilon = 1.0
           self.epsilon_min = 0.01
           self.epsilon_decay = 0.995
           self.learning_rate = 0.001
           self.gamma = 0.95
           self.batch_size = 32

           # Neural networks
           self.q_network = DQN(self.state_size, self.action_size)
           self.target_network = DQN(self.state_size, self.action_size)
           self.optimizer = optim.Adam(self.q_network.parameters(), lr=self.learning_rate)

           # Robot state
           self.position_x = 0.0
           self.position_y = 0.0
           self.goal_x = 5.0
           self.goal_y = 5.0

           # Publishers and subscribers
           self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
           self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

           # Timer for control loop
           self.control_timer = self.create_timer(0.1, self.control_loop)

           self.get_logger().info('DQN Navigation Node Started')

       def odom_callback(self, msg):
           self.position_x = msg.pose.pose.position.x
           self.position_y = msg.pose.pose.position.y

       def get_state(self):
           return np.array([self.position_x, self.position_y, self.goal_x, self.goal_y])

       def get_reward(self):
           dist_to_goal = math.sqrt((self.position_x - self.goal_x)**2 + (self.position_y - self.goal_y)**2)
           reward = -dist_to_goal

           if dist_to_goal < 0.5:
               reward += 100  # Bonus for reaching goal

           return reward

       def remember(self, state, action, reward, next_state, done):
           self.memory.append((state, action, reward, next_state, done))

       def act(self, state):
           if np.random.random() <= self.epsilon:
               return random.choice(range(self.action_size))

           state_tensor = torch.FloatTensor(state).unsqueeze(0)
           q_values = self.q_network(state_tensor)
           return np.argmax(q_values.cpu().data.numpy())

       def replay(self):
           if len(self.memory) < self.batch_size:
               return

           batch = random.sample(self.memory, self.batch_size)
           states = torch.FloatTensor([e[0] for e in batch])
           actions = torch.LongTensor([e[1] for e in batch])
           rewards = torch.FloatTensor([e[2] for e in batch])
           next_states = torch.FloatTensor([e[3] for e in batch])
           dones = torch.BoolTensor([e[4] for e in batch])

           current_q_values = self.q_network(states).gather(1, actions.unsqueeze(1))
           next_q_values = self.target_network(next_states).max(1)[0].detach()
           target_q_values = rewards + (self.gamma * next_q_values * ~dones)

           loss = nn.MSELoss()(current_q_values.squeeze(), target_q_values)

           self.optimizer.zero_grad()
           loss.backward()
           self.optimizer.step()

           if self.epsilon > self.epsilon_min:
               self.epsilon *= self.epsilon_decay

       def control_loop(self):
           state = self.get_state()
           action = self.act(state)

           # Execute action (simplified)
           cmd_msg = Twist()
           if action == 0:  # Move forward
               cmd_msg.linear.x = 0.5
           elif action == 1:  # Turn left
               cmd_msg.angular.z = 0.5
           elif action == 2:  # Turn right
               cmd_msg.angular.z = -0.5
           elif action == 3:  # Move backward
               cmd_msg.linear.x = -0.3

           self.cmd_vel_pub.publish(cmd_msg)

           # Get reward and next state
           reward = self.get_reward()
           next_state = self.get_state()
           done = abs(self.position_x - self.goal_x) < 0.5 and abs(self.position_y - self.goal_y) < 0.5

           # Store experience
           self.remember(state, action, reward, next_state, done)

           # Train network
           self.replay()

           self.get_logger().info(f'Reward: {reward:.2f}, Epsilon: {self.epsilon:.3f}')

   def main(args=None):
       rclpy.init(args=args)
       dqn_nav = DQNNavigation()

       try:
           rclpy.spin(dqn_nav)
       except KeyboardInterrupt:
           pass
       finally:
           dqn_nav.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Add the DQN node to `setup.py` and rebuild.

## Exercise 3: Imitation Learning (1.5 hours)

### Task 3.1: Behavioral Cloning
1. Create an imitation learning package:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --dependencies rclpy std_msgs geometry_msgs sensor_msgs --build-type ament_python imitation_learning_lab
   cd imitation_learning_lab
   ```

2. Create a behavioral cloning node that learns from demonstration data:
   ```python
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from geometry_msgs.msg import Twist
   from sensor_msgs.msg import LaserScan
   from std_msgs.msg import Float32MultiArray
   import torch
   import torch.nn as nn
   import torch.optim as optim
   import numpy as np
   import pickle

   class SimpleBehavioralCloning(nn.Module):
       def __init__(self, sensor_input_size, action_size):
           super(SimpleBehavioralCloning, self).__init__()
           self.fc1 = nn.Linear(sensor_input_size, 128)
           self.fc2 = nn.Linear(128, 64)
           self.fc3 = nn.Linear(64, action_size)
           self.dropout = nn.Dropout(0.2)

       def forward(self, x):
           x = torch.relu(self.fc1(x))
           x = self.dropout(x)
           x = torch.relu(self.fc2(x))
           x = self.dropout(x)
           return torch.tanh(self.fc3(x))  # tanh for bounded output

   class BehavioralCloningNode(Node):
       def __init__(self):
           super().__init__('behavioral_cloning')

           # Neural network parameters
           self.sensor_input_size = 10  # Simplified laser scan
           self.action_size = 2  # linear velocity, angular velocity

           self.model = SimpleBehavioralCloning(self.sensor_input_size, self.action_size)
           self.optimizer = optim.Adam(self.model.parameters(), lr=0.001)
           self.criterion = nn.MSELoss()

           # Data storage for training
           self.demonstration_states = []
           self.demonstration_actions = []
           self.is_collecting_demonstrations = True

           # Publishers and subscribers
           self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
           self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
           self.expert_cmd_sub = self.create_subscription(Twist, 'expert_cmd', self.expert_cmd_callback, 10)

           # Timer for control loop
           self.control_timer = self.create_timer(0.1, self.control_loop)

           self.current_scan = None
           self.current_expert_action = None

           self.get_logger().info('Behavioral Cloning Node Started')

       def scan_callback(self, msg):
           # Simplify laser scan to fixed size input
           if len(msg.ranges) >= self.sensor_input_size:
               step = len(msg.ranges) // self.sensor_input_size
               simplified_scan = [msg.ranges[i] for i in range(0, len(msg.ranges), step)][:self.sensor_input_size]
               # Replace invalid ranges with max range
               for i in range(len(simplified_scan)):
                   if np.isnan(simplified_scan[i]) or np.isinf(simplified_scan[i]):
                       simplified_scan[i] = msg.range_max
               self.current_scan = simplified_scan

       def expert_cmd_callback(self, msg):
           self.current_expert_action = [msg.linear.x, msg.angular.z]

       def collect_demonstration(self):
           if self.current_scan is not None and self.current_expert_action is not None:
               self.demonstration_states.append(self.current_scan.copy())
               self.demonstration_actions.append(self.current_expert_action.copy())
               self.get_logger().info(f'Collected demonstration {len(self.demonstration_states)}')

       def train_model(self):
           if len(self.demonstration_states) < 10:
               return

           # Convert to tensors
           states_tensor = torch.FloatTensor(self.demonstration_states)
           actions_tensor = torch.FloatTensor(self.demonstration_actions)

           # Train the model
           self.optimizer.zero_grad()
           predicted_actions = self.model(states_tensor)
           loss = self.criterion(predicted_actions, actions_tensor)
           loss.backward()
           self.optimizer.step()

           self.get_logger().info(f'Training loss: {loss.item():.4f}')

       def control_loop(self):
           if self.current_scan is not None:
               if self.is_collecting_demonstrations and self.current_expert_action is not None:
                   # Collect demonstrations
                   self.collect_demonstration()

                   # After collecting enough demonstrations, start training
                   if len(self.demonstration_states) >= 50:
                       self.train_model()
                       if len(self.demonstration_states) >= 100:  # Enough data to start using the model
                           self.is_collecting_demonstrations = False
                           self.get_logger().info('Switching to learned policy')
               else:
                   # Use learned policy
                   state_tensor = torch.FloatTensor(self.current_scan).unsqueeze(0)
                   action_tensor = self.model(state_tensor)
                   action = action_tensor.squeeze().detach().numpy()

                   cmd_msg = Twist()
                   cmd_msg.linear.x = float(action[0])
                   cmd_msg.angular.z = float(action[1])
                   self.cmd_vel_pub.publish(cmd_msg)

                   self.get_logger().info(f'Learned action - Linear: {action[0]:.2f}, Angular: {action[1]:.2f}')

   def main(args=None):
       rclpy.init(args=args)
       bc_node = BehavioralCloningNode()

       try:
           rclpy.spin(bc_node)
       except KeyboardInterrupt:
           # Save the trained model
           rclpy.try_shutdown()

   if __name__ == '__main__':
       main()
   ```

### Task 3.2: Model Saving and Loading
1. Add functionality to save and load the trained model:
   ```python
   def save_model(self, filepath='behavioral_cloning_model.pth'):
       torch.save({
           'model_state_dict': self.model.state_dict(),
           'optimizer_state_dict': self.optimizer.state_dict(),
           'demonstration_states': self.demonstration_states,
           'demonstration_actions': self.demonstration_actions
       }, filepath)
       self.get_logger().info(f'Model saved to {filepath}')

   def load_model(self, filepath='behavioral_cloning_model.pth'):
       try:
           checkpoint = torch.load(filepath)
           self.model.load_state_dict(checkpoint['model_state_dict'])
           self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
           self.demonstration_states = checkpoint['demonstration_states']
           self.demonstration_actions = checkpoint['demonstration_actions']
           self.get_logger().info(f'Model loaded from {filepath}')
           return True
       except FileNotFoundError:
           self.get_logger().info(f'No saved model found at {filepath}')
           return False
   ```

## Exercise 4: ML Integration with ROS 2 (2 hours)

### Task 4.1: ROS 2 Service for ML Inference
1. Create a service definition for ML inference (create `srv/MLInference.srv` in your package):
   ```
   float32[] input_data
   ---
   float32[] output_data
   string model_name
   float32 confidence
   ```

2. Create a node that provides ML inference as a ROS 2 service:
   ```python
   # ml_integration_lab/ml_inference_service.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from ml_integration_lab.srv import MLInference
   import torch
   import torch.nn as nn
   import numpy as np

   class SimpleMLModel(nn.Module):
       def __init__(self):
           super(SimpleMLModel, self).__init__()
           self.fc1 = nn.Linear(10, 32)
           self.fc2 = nn.Linear(32, 16)
           self.fc3 = nn.Linear(16, 4)  # 4 outputs for 4 different predictions

       def forward(self, x):
           x = torch.relu(self.fc1(x))
           x = torch.relu(self.fc2(x))
           return torch.softmax(self.fc3(x), dim=1)

   class MLInferenceService(Node):
       def __init__(self):
           super().__init__('ml_inference_service')

           # Initialize a simple model (in practice, load a pre-trained model)
           self.model = SimpleMLModel()
           self.model.eval()

           # Create service
           self.srv = self.create_service(MLInference, 'ml_inference', self.inference_callback)

           self.get_logger().info('ML Inference Service Started')

       def inference_callback(self, request, response):
           try:
               # Convert input to tensor
               input_tensor = torch.FloatTensor(request.input_data).unsqueeze(0)

               # Run inference
               with torch.no_grad():
                   output = self.model(input_tensor)
                   probabilities = output.squeeze().numpy()

               # Prepare response
               response.output_data = probabilities.tolist()
               response.model_name = 'simple_ml_model'
               response.confidence = float(np.max(probabilities))

               self.get_logger().info(f'Inference completed with confidence: {response.confidence:.3f}')

           except Exception as e:
               self.get_logger().error(f'Inference error: {str(e)}')
               response.output_data = [0.0] * 4
               response.confidence = 0.0
               response.model_name = 'error'

           return response

   def main(args=None):
       rclpy.init(args=args)
       ml_service = MLInferenceService()

       try:
           rclpy.spin(ml_service)
       except KeyboardInterrupt:
           pass
       finally:
           ml_service.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

### Task 4.2: Model Deployment and Evaluation
1. Create a node to evaluate ML model performance:
   ```python
   # ml_integration_lab/model_evaluator.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import LaserScan
   from geometry_msgs.msg import Twist
   import numpy as np
   import time

   class ModelEvaluator(Node):
       def __init__(self):
           super().__init__('model_evaluator')

           # Performance tracking
           self.inference_times = []
           self.inference_count = 0

           # Subscribers
           self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

           self.get_logger().info('Model Evaluator Started')

       def scan_callback(self, msg):
           # Simulate model inference timing
           start_time = time.time()

           # Process scan data (simulated ML inference)
           processed_data = self.process_scan_data(msg)

           end_time = time.time()
           inference_time = end_time - start_time

           self.inference_times.append(inference_time)
           self.inference_count += 1

           # Log performance metrics periodically
           if self.inference_count % 10 == 0:
               avg_time = np.mean(self.inference_times[-10:])
               self.get_logger().info(f'Avg inference time (last 10): {avg_time:.4f}s, Rate: {1/avg_time:.2f} Hz')

       def process_scan_data(self, scan_msg):
           # Simulate processing scan data with ML model
           # In real implementation, this would call the actual ML model
           return [np.mean(scan_msg.ranges)]  # Simplified processing

   def main(args=None):
       rclpy.init(args=args)
       evaluator = ModelEvaluator()

       try:
           rclpy.spin(evaluator)
       except KeyboardInterrupt:
           pass
       finally:
           evaluator.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

## Assessment Questions

1. What are the main differences between deep learning approaches and classical computer vision for robot perception?
2. How does the exploration-exploitation tradeoff work in reinforcement learning for robotics?
3. What challenges arise when applying imitation learning to robotics, and how can they be addressed?
4. How would you validate the safety of a machine learning model deployed on a physical robot?
5. What are the computational constraints when deploying ML models on resource-constrained robots?

## Troubleshooting Tips

- **Memory issues**: Monitor GPU/CPU memory usage during training
- **Training instability**: Adjust learning rate and add regularization
- **Real-time performance**: Optimize model architecture for inference speed
- **Overfitting**: Use validation sets and regularization techniques
- **Sim-to-real gap**: Apply domain randomization during training

## Extensions

1. Implement a more sophisticated perception model using a pre-trained network
2. Add uncertainty quantification to your ML models
3. Create a reinforcement learning environment with Gazebo simulation
4. Implement federated learning across multiple robots
5. Add explainability features to understand model decisions

## Summary

This lab exercise provided hands-on experience with machine learning for robotics. You implemented perception models, reinforcement learning algorithms, imitation learning, and integrated ML systems with ROS 2. These skills are essential for creating intelligent robotic systems that can learn and adapt to their environment.