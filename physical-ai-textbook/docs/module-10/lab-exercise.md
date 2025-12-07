#  Lab Exercise: Advanced Topics in Robot Learning

## Overview

In this lab exercise, you will implement deep reinforcement learning algorithms, meta-learning techniques, transfer learning approaches, and other advanced learning methods for robotics. You will work with sophisticated learning algorithms that enable robots to acquire complex skills, adapt to new environments, and learn from minimal human supervision. This builds upon your Week 1-9 foundations to create robots with advanced learning capabilities.

## Prerequisites

- Completion of Week 1-9 lab exercises
- Working ROS 2 Humble Hawksbill installation
- Python development environment with ML libraries (PyTorch/TensorFlow)
- Robot simulation environment (Gazebo with complex scenarios)
- Experience with deep learning frameworks
- Strong understanding of machine learning concepts
- Programming experience in Python and C++

## Learning Objectives

By completing this lab, you will:
- Implement deep reinforcement learning algorithms for robotic tasks
- Apply meta-learning and few-shot learning techniques to robotics
- Use transfer learning to adapt models across different robots and environments
- Develop imitation learning systems with minimal human demonstrations
- Implement curiosity-driven and intrinsic motivation learning
- Evaluate and validate advanced learning systems in robotic applications

## Exercise 1: Deep Reinforcement Learning (2.5 hours)

### Task 1.1: Deep Q-Network Implementation
1. Create a ROS 2 package for deep reinforcement learning:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --dependencies rclpy std_msgs geometry_msgs sensor_msgs --build-type ament_python advanced_rl_lab
   cd advanced_rl_lab
   ```

2. Install required Python packages:
   ```bash
   pip3 install torch torchvision torchaudio
   pip3 install numpy matplotlib
   ```

3. Create a DQN implementation for robotic navigation `advanced_rl_lab/dqn_navigation.py`:
   ```python
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
   from sensor_msgs.msg import LaserScan
   from nav_msgs.msg import Odometry
   import math

   class DQN(nn.Module):
       def __init__(self, state_size, action_size):
           super(DQN, self).__init__()
           self.fc1 = nn.Linear(state_size, 128)
           self.fc2 = nn.Linear(128, 128)
           self.fc3 = nn.Linear(128, action_size)

       def forward(self, x):
           x = torch.relu(self.fc1(x))
           x = torch.relu(self.fc2(x))
           return self.fc3(x)

   class DQNNavigationNode(Node):
       def __init__(self):
           super().__init__('dqn_navigation_node')

           # DQN parameters
           self.state_size = 10  # Simplified laser scan
           self.action_size = 4  # forward, left, right, stop
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

           # Update target network
           self.update_target_network()

           # Robot state
           self.laser_scan = None
           self.position = None
           self.goal = [5.0, 5.0]  # Fixed goal for this example

           # Publishers and subscribers
           self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
           self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
           self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

           # Timer for control loop
           self.control_timer = self.create_timer(0.1, self.control_loop)

           self.get_logger().info('DQN Navigation Node Started')

       def odom_callback(self, msg):
           self.position = [msg.pose.pose.position.x, msg.pose.pose.position.y]

       def scan_callback(self, msg):
           # Simplify laser scan to fixed size input
           if len(msg.ranges) >= self.state_size:
               step = len(msg.ranges) // self.state_size
               simplified_scan = [min(r, 10.0) for r in msg.ranges[::step][:self.state_size]]
               self.laser_scan = simplified_scan

       def get_state(self):
           if self.laser_scan is None or self.position is None:
               return np.zeros(self.state_size)

           # Combine laser scan with position relative to goal
           state = self.laser_scan.copy()
           state.extend([self.position[0], self.position[1],
                        self.goal[0], self.goal[1]])
           return np.array(state[:self.state_size])

       def get_reward(self):
           if self.position is None:
               return 0.0

           # Calculate distance to goal
           dist_to_goal = math.sqrt((self.position[0] - self.goal[0])**2 +
                                   (self.position[1] - self.goal[1])**2)

           # Reward based on distance and obstacle avoidance
           reward = -dist_to_goal  # Negative distance as reward

           # Penalty for being close to obstacles
           if self.laser_scan:
               min_range = min(self.laser_scan) if self.laser_scan else float('inf')
               if min_range < 0.5:
                   reward -= 10  # Large penalty for being too close to obstacles

           # Bonus for reaching goal
           if dist_to_goal < 0.5:
               reward += 100

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

       def update_target_network(self):
           self.target_network.load_state_dict(self.q_network.state_dict())

       def control_loop(self):
           state = self.get_state()
           action = self.act(state)

           # Execute action
           cmd_msg = Twist()
           if action == 0:  # Forward
               cmd_msg.linear.x = 0.5
           elif action == 1:  # Turn left
               cmd_msg.angular.z = 0.5
           elif action == 2:  # Turn right
               cmd_msg.angular.z = -0.5
           elif action == 3:  # Stop
               cmd_msg.linear.x = 0.0

           self.cmd_vel_pub.publish(cmd_msg)

           # Get reward and next state
           reward = self.get_reward()
           next_state = self.get_state()
           done = (self.position is not None and
                   math.sqrt((self.position[0] - self.goal[0])**2 +
                            (self.position[1] - self.goal[1])**2) < 0.5)

           # Store experience
           self.remember(state, action, reward, next_state, done)

           # Train network
           self.replay()

           # Update target network periodically
           if len(self.memory) % 100 == 0:
               self.update_target_network()

           self.get_logger().info(f'Reward: {reward:.2f}, Epsilon: {self.epsilon:.3f}, Action: {action}')

   def main(args=None):
       rclpy.init(args=args)
       dqn_node = DQNNavigationNode()

       try:
           rclpy.spin(dqn_node)
       except KeyboardInterrupt:
           pass
       finally:
           dqn_node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

4. Update the package setup in `setup.py`:
   ```python
   entry_points={
       'console_scripts': [
           'dqn_navigation = advanced_rl_lab.dqn_navigation:main',
       ],
   },
   ```

### Task 1.2: Soft Actor-Critic Implementation
1. Create a SAC implementation for continuous control:
   ```python
   # advanced_rl_lab/sac_navigation.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   import torch
   import torch.nn as nn
   import torch.optim as optim
   import torch.nn.functional as F
   import numpy as np
   from collections import deque
   import random
   from geometry_msgs.msg import Twist
   from sensor_msgs.msg import LaserScan
   from nav_msgs.msg import Odometry
   import math

   class Actor(nn.Module):
       def __init__(self, state_dim, action_dim, max_action):
           super(Actor, self).__init__()

           self.l1 = nn.Linear(state_dim, 256)
           self.l2 = nn.Linear(256, 256)
           self.l3 = nn.Linear(256, action_dim)

           self.max_action = max_action

       def forward(self, state):
           a = F.relu(self.l1(state))
           a = F.relu(self.l2(a))
           return self.max_action * torch.tanh(self.l3(a))

   class Critic(nn.Module):
       def __init__(self, state_dim, action_dim):
           super(Critic, self).__init__()

           # Q1 architecture
           self.l1 = nn.Linear(state_dim + action_dim, 256)
           self.l2 = nn.Linear(256, 256)
           self.l3 = nn.Linear(256, 1)

           # Q2 architecture
           self.l4 = nn.Linear(state_dim + action_dim, 256)
           self.l5 = nn.Linear(256, 256)
           self.l6 = nn.Linear(256, 1)

       def forward(self, state, action):
           sa = torch.cat([state, action], 1)

           q1 = F.relu(self.l1(sa))
           q1 = F.relu(self.l2(q1))
           q1 = self.l3(q1)

           q2 = F.relu(self.l4(sa))
           q2 = F.relu(self.l5(q2))
           q2 = self.l6(q2)
           return q1, q2

       def Q1(self, state, action):
           sa = torch.cat([state, action], 1)

           q1 = F.relu(self.l1(sa))
           q1 = F.relu(self.l2(q1))
           q1 = self.l3(q1)
           return q1

   class SACNode(Node):
       def __init__(self):
           super().__init__('sac_navigation_node')

           # SAC parameters
           self.state_dim = 10  # Simplified laser scan
           self.action_dim = 2  # linear and angular velocity
           self.max_action = 1.0
           self.memory = deque(maxlen=50000)
           self.learning_rate = 0.0003
           self.gamma = 0.99
           self.tau = 0.005
           self.batch_size = 64
           self.alpha = 0.2  # Temperature parameter

           # Initialize networks
           self.actor = Actor(self.state_dim, self.action_dim, self.max_action)
           self.critic = Critic(self.state_dim, self.action_dim)
           self.critic_target = Critic(self.state_dim, self.action_dim)
           self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=self.learning_rate)
           self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=self.learning_rate)

           # Copy critic weights to target
           for target_param, param in zip(self.critic_target.parameters(), self.critic.parameters()):
               target_param.data.copy_(param.data)

           # Robot state
           self.laser_scan = None
           self.position = None
           self.goal = [5.0, 5.0]

           # Publishers and subscribers
           self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
           self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
           self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

           # Timer for control loop
           self.control_timer = self.create_timer(0.1, self.control_loop)

           self.get_logger().info('SAC Navigation Node Started')

       def odom_callback(self, msg):
           self.position = [msg.pose.pose.position.x, msg.pose.pose.position.y]

       def scan_callback(self, msg):
           if len(msg.ranges) >= self.state_dim:
               step = len(msg.ranges) // self.state_dim
               simplified_scan = [min(r, 10.0) for r in msg.ranges[::step][:self.state_dim]]
               self.laser_scan = simplified_scan

       def get_state(self):
           if self.laser_scan is None or self.position is None:
               return np.zeros(self.state_dim)

           state = self.laser_scan.copy()
           state.extend([self.position[0], self.position[1],
                        self.goal[0], self.goal[1]])
           return np.array(state[:self.state_dim])

       def get_reward(self):
           if self.position is None:
               return 0.0

           dist_to_goal = math.sqrt((self.position[0] - self.goal[0])**2 +
                                   (self.position[1] - self.goal[1])**2)

           reward = -dist_to_goal

           if self.laser_scan:
               min_range = min(self.laser_scan) if self.laser_scan else float('inf')
               if min_range < 0.5:
                   reward -= 10

           if dist_to_goal < 0.5:
               reward += 100

           return reward

       def remember(self, state, action, reward, next_state, done):
           self.memory.append((state, action, reward, next_state, done))

       def train(self):
           if len(self.memory) < self.batch_size:
               return

           batch = random.sample(self.memory, self.batch_size)
           state_batch = torch.FloatTensor([e[0] for e in batch])
           action_batch = torch.FloatTensor([e[1] for e in batch])
           reward_batch = torch.FloatTensor([e[2] for e in batch]).unsqueeze(1)
           next_state_batch = torch.FloatTensor([e[3] for e in batch])
           done_batch = torch.BoolTensor([e[4] for e in batch]).unsqueeze(1)

           with torch.no_grad():
               next_action = self.actor(next_state_batch)
               target_q1, target_q2 = self.critic_target(next_state_batch, next_action)
               target_q = torch.min(target_q1, target_q2)
               target_q = reward_batch + (self.gamma * target_q * ~done_batch)

           # Update critic
           current_q1, current_q2 = self.critic(state_batch, action_batch)
           critic_loss = F.mse_loss(current_q1, target_q) + F.mse_loss(current_q2, target_q)

           self.critic_optimizer.zero_grad()
           critic_loss.backward()
           self.critic_optimizer.step()

           # Update actor
           actor_action = self.actor(state_batch)
           actor_q1, actor_q2 = self.critic(state_batch, actor_action)
           actor_q = torch.min(actor_q1, actor_q2)
           actor_loss = -actor_q.mean()

           self.actor_optimizer.zero_grad()
           actor_loss.backward()
           self.actor_optimizer.step()

           # Update target networks
           for target_param, param in zip(self.critic_target.parameters(), self.critic.parameters()):
               target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)

       def control_loop(self):
           state = self.get_state()
           state_tensor = torch.FloatTensor(state).unsqueeze(0)
           action = self.actor(state_tensor).cpu().data.numpy().flatten()

           # Execute action
           cmd_msg = Twist()
           cmd_msg.linear.x = float(action[0])
           cmd_msg.angular.z = float(action[1])
           self.cmd_vel_pub.publish(cmd_msg)

           # Get reward and next state
           reward = self.get_reward()
           next_state = self.get_state()
           done = (self.position is not None and
                   math.sqrt((self.position[0] - self.goal[0])**2 +
                            (self.position[1] - self.goal[1])**2) < 0.5)

           # Store experience
           self.remember(state, action, reward, next_state, done)

           # Train network
           self.train()

           self.get_logger().info(f'Reward: {reward:.2f}, Action: [{action[0]:.2f}, {action[1]:.2f}]')

   def main(args=None):
       rclpy.init(args=args)
       sac_node = SACNode()

       try:
           rclpy.spin(sac_node)
       except KeyboardInterrupt:
           pass
       finally:
           sac_node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Add the SAC node to `setup.py` and build the package.

## Exercise 2: Meta-Learning and Transfer Learning (2.5 hours)

### Task 2.1: Model-Agnostic Meta-Learning (MAML)
1. Create a MAML implementation for rapid adaptation:
   ```python
   # advanced_rl_lab/maml_adaptation.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   import torch
   import torch.nn as nn
   import torch.optim as optim
   import numpy as np
   from geometry_msgs.msg import Twist
   from sensor_msgs.msg import LaserScan
   from nav_msgs.msg import Odometry
   import math
   import random

   class MetaLearner(nn.Module):
       def __init__(self, input_size, output_size):
           super(MetaLearner, self).__init__()
           self.network = nn.Sequential(
               nn.Linear(input_size, 64),
               nn.ReLU(),
               nn.Linear(64, 64),
               nn.ReLU(),
               nn.Linear(64, output_size)
           )

       def forward(self, x):
           return self.network(x)

       def update_parameters(self, loss, step_size=0.01):
           """Update parameters with one step of gradient descent"""
           gradients = torch.autograd.grad(loss, self.parameters(), create_graph=True)
           new_params = []
           for param, grad in zip(self.parameters(), gradients):
               new_params.append(param - step_size * grad)

           # Create a copy of the model with updated parameters
           updated_model = MetaLearner(self.network[0].in_features, self.network[-1].out_features)
           updated_params = list(updated_model.parameters())
           for i, new_param in enumerate(new_params):
               updated_params[i].data.copy_(new_param.data)

           return updated_model

   class MAMLNode(Node):
       def __init__(self):
           super().__init__('maml_node')

           # Meta-learning parameters
           self.state_size = 10
           self.action_size = 4
           self.meta_lr = 0.001
           self.update_lr = 0.01
           self.meta_batch_size = 16
           self.update_batch_size = 10

           # Initialize meta-learner
           self.meta_learner = MetaLearner(self.state_size, self.action_size)
           self.meta_optimizer = optim.Adam(self.meta_learner.parameters(), lr=self.meta_lr)

           # Robot state
           self.laser_scan = None
           self.position = None
           self.goal = [5.0, 5.0]

           # Task-specific variables
           self.current_task = 0
           self.task_goals = [[5.0, 5.0], [-5.0, 5.0], [5.0, -5.0], [-5.0, -5.0]]
           self.episode_buffer = []

           # Publishers and subscribers
           self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
           self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
           self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

           # Timer for control loop
           self.control_timer = self.create_timer(0.1, self.control_loop)

           self.get_logger().info('MAML Node Started')

       def odom_callback(self, msg):
           self.position = [msg.pose.pose.position.x, msg.pose.pose.position.y]

       def scan_callback(self, msg):
           if len(msg.ranges) >= self.state_size:
               step = len(msg.ranges) // self.state_size
               simplified_scan = [min(r, 10.0) for r in msg.ranges[::step][:self.state_size]]
               self.laser_scan = simplified_scan

       def get_state(self):
           if self.laser_scan is None or self.position is None:
               return np.zeros(self.state_size)

           state = self.laser_scan.copy()
           current_goal = self.task_goals[self.current_task]
           state.extend([self.position[0], self.position[1],
                        current_goal[0], current_goal[1]])
           return np.array(state[:self.state_size])

       def get_reward(self):
           if self.position is None:
               return 0.0

           current_goal = self.task_goals[self.current_task]
           dist_to_goal = math.sqrt((self.position[0] - current_goal[0])**2 +
                                   (self.position[1] - current_goal[1])**2)

           reward = -dist_to_goal

           if self.laser_scan:
               min_range = min(self.laser_scan) if self.laser_scan else float('inf')
               if min_range < 0.5:
                   reward -= 10

           if dist_to_goal < 0.5:
               reward += 100

           return reward

       def select_action(self, state):
           state_tensor = torch.FloatTensor(state).unsqueeze(0)
           q_values = self.meta_learner(state_tensor)
           return np.argmax(q_values.cpu().data.numpy())

       def control_loop(self):
           state = self.get_state()
           action = self.select_action(state)

           # Execute action
           cmd_msg = Twist()
           if action == 0:  # Forward
               cmd_msg.linear.x = 0.5
           elif action == 1:  # Turn left
               cmd_msg.angular.z = 0.5
           elif action == 2:  # Turn right
               cmd_msg.angular.z = -0.5
           elif action == 3:  # Stop
               cmd_msg.linear.x = 0.0

           self.cmd_vel_pub.publish(cmd_msg)

           # Get reward and store in episode buffer
           reward = self.get_reward()
           self.episode_buffer.append((state, action, reward))

           # Update goal if reached
           current_goal = self.task_goals[self.current_task]
           if (self.position is not None and
               math.sqrt((self.position[0] - current_goal[0])**2 +
                        (self.position[1] - current_goal[1])**2) < 0.5):
               self.current_task = (self.current_task + 1) % len(self.task_goals)
               self.episode_buffer = []  # Reset buffer for new task

           # Perform meta-learning update periodically
           if len(self.episode_buffer) >= self.update_batch_size:
               self.meta_update()
               self.episode_buffer = []  # Reset buffer

           self.get_logger().info(f'Task: {self.current_task}, Reward: {reward:.2f}, Action: {action}')

       def meta_update(self):
           if len(self.episode_buffer) < self.update_batch_size:
               return

           # Sample tasks for meta-update
           sampled_tasks = random.sample(range(len(self.task_goals)), min(4, len(self.task_goals)))

           meta_losses = []
           for task_id in sampled_tasks:
               # Get data for this task
               task_data = self.episode_buffer[:self.update_batch_size]

               # Create a fast-adapted model for this task
               adapted_model = self.meta_learner.update_parameters(
                   self.compute_loss(task_data, self.meta_learner),
                   step_size=self.update_lr
               )

               # Compute meta loss using adapted model
               meta_loss = self.compute_loss(task_data, adapted_model)
               meta_losses.append(meta_loss)

           # Update meta-learner with average meta loss
           if meta_losses:
               total_meta_loss = sum(meta_losses) / len(meta_losses)
               self.meta_optimizer.zero_grad()
               total_meta_loss.backward()
               self.meta_optimizer.step()

       def compute_loss(self, data, model):
           states = torch.FloatTensor([d[0] for d in data])
           actions = torch.LongTensor([d[1] for d in data])
           rewards = torch.FloatTensor([d[2] for d in data])

           q_values = model(states)
           selected_q_values = q_values.gather(1, actions.unsqueeze(1))
           loss = torch.nn.MSELoss()(selected_q_values.squeeze(), rewards)
           return loss

   def main(args=None):
       rclpy.init(args=args)
       maml_node = MAMLNode()

       try:
           rclpy.spin(maml_node)
       except KeyboardInterrupt:
           pass
       finally:
           maml_node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Add the MAML node to `setup.py`.

### Task 2.2: Transfer Learning Implementation
1. Create a transfer learning system that adapts between different robot morphologies:
   ```python
   # advanced_rl_lab/transfer_learning.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   import torch
   import torch.nn as nn
   import torch.optim as optim
   import numpy as np
   from geometry_msgs.msg import Twist
   from sensor_msgs.msg import LaserScan
   from nav_msgs.msg import Odometry
   import math

   class BasePolicy(nn.Module):
       def __init__(self, input_size, output_size):
           super(BasePolicy, self).__init__()
           self.feature_extractor = nn.Sequential(
               nn.Linear(input_size, 128),
               nn.ReLU(),
               nn.Linear(128, 64),
               nn.ReLU()
           )
           self.actor = nn.Linear(64, output_size)

       def forward(self, x):
           features = self.feature_extractor(x)
           return self.actor(features)

   class TransferLearningNode(Node):
       def __init__(self):
           super().__init__('transfer_learning_node')

           # Transfer learning parameters
           self.source_robot_state_size = 10
           self.target_robot_state_size = 12  # Different robot has more sensors
           self.action_size = 4
           self.transfer_lr = 0.0005
           self.fine_tune_lr = 0.001

           # Initialize base policy with source robot parameters
           self.base_policy = BasePolicy(self.source_robot_state_size, self.action_size)
           self.target_policy = BasePolicy(self.target_robot_state_size, self.action_size)

           # Copy feature extractor weights (transfer knowledge)
           self.transfer_weights()

           # Optimizers
           self.target_optimizer = optim.Adam(self.target_policy.parameters(), lr=self.transfer_lr)

           # Robot state
           self.laser_scan = None
           self.position = None
           self.goal = [5.0, 5.0]

           # Publishers and subscribers
           self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
           self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
           self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

           # Timer for control loop
           self.control_timer = self.create_timer(0.1, self.control_loop)

           self.get_logger().info('Transfer Learning Node Started')

       def transfer_weights(self):
           # Transfer feature extractor weights from source to target
           # Only transfer the common part, initialize the rest randomly
           source_features = list(self.base_policy.feature_extractor.parameters())
           target_features = list(self.target_policy.feature_extractor.parameters())

           # Copy weights for the common layers
           for src_param, tgt_param in zip(source_features[:2], target_features[:2]):  # First 2 layers
               if src_param.shape == tgt_param.shape:
                   tgt_param.data.copy_(src_param.data)
               else:
                   # If shapes differ (due to different input sizes), skip or adapt
                   continue

       def odom_callback(self, msg):
           self.position = [msg.pose.pose.position.x, msg.pose.pose.position.y]

       def scan_callback(self, msg):
           if len(msg.ranges) >= self.target_robot_state_size - 2:  # Account for position features
               step = len(msg.ranges) // (self.target_robot_state_size - 2)
               simplified_scan = [min(r, 10.0) for r in msg.ranges[::step][:(self.target_robot_state_size - 2)]]
               self.laser_scan = simplified_scan

       def get_state(self):
           if self.laser_scan is None or self.position is None:
               return np.zeros(self.target_robot_state_size)

           state = self.laser_scan.copy()
           state.extend([self.position[0], self.position[1]])
           return np.array(state[:self.target_robot_state_size])

       def get_reward(self):
           if self.position is None:
               return 0.0

           dist_to_goal = math.sqrt((self.position[0] - self.goal[0])**2 +
                                   (self.position[1] - self.goal[1])**2)

           reward = -dist_to_goal

           if self.laser_scan:
               min_range = min(self.laser_scan) if self.laser_scan else float('inf')
               if min_range < 0.5:
                   reward -= 10

           if dist_to_goal < 0.5:
               reward += 100

           return reward

       def select_action(self, state):
           state_tensor = torch.FloatTensor(state).unsqueeze(0)
           q_values = self.target_policy(state_tensor)
           return np.argmax(q_values.cpu().data.numpy())

       def control_loop(self):
           state = self.get_state()
           action = self.select_action(state)

           # Execute action
           cmd_msg = Twist()
           if action == 0:  # Forward
               cmd_msg.linear.x = 0.5
           elif action == 1:  # Turn left
               cmd_msg.angular.z = 0.5
           elif action == 2:  # Turn right
               cmd_msg.angular.z = -0.5
           elif action == 3:  # Stop
               cmd_msg.linear.x = 0.0

           self.cmd_vel_pub.publish(cmd_msg)

           # Get reward and perform learning
           reward = self.get_state()
           reward = self.get_reward()

           # Update target policy
           state_tensor = torch.FloatTensor(state).unsqueeze(0)
           action_tensor = torch.LongTensor([action]).unsqueeze(1)
           reward_tensor = torch.FloatTensor([reward]).unsqueeze(1)

           q_values = self.target_policy(state_tensor)
           selected_q = q_values.gather(1, action_tensor)
           loss = torch.nn.MSELoss()(selected_q, reward_tensor)

           self.target_optimizer.zero_grad()
           loss.backward()
           self.target_optimizer.step()

           self.get_logger().info(f'Reward: {reward:.2f}, Action: {action}, Loss: {loss.item():.4f}')

   def main(args=None):
       rclpy.init(args=args)
       transfer_node = TransferLearningNode()

       try:
           rclpy.spin(transfer_node)
       except KeyboardInterrupt:
           pass
       finally:
           transfer_node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

## Exercise 3: Advanced Imitation Learning (2 hours)

### Task 3.1: Generative Adversarial Imitation Learning (GAIL)
1. Create a GAIL implementation for learning from demonstrations:
   ```python
   # advanced_rl_lab/gail_learning.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   import torch
   import torch.nn as nn
   import torch.optim as optim
   import numpy as np
   from collections import deque
   from geometry_msgs.msg import Twist
   from sensor_msgs.msg import LaserScan
   from nav_msgs.msg import Odometry
   import math

   class Discriminator(nn.Module):
       def __init__(self, state_dim, action_dim):
           super(Discriminator, self).__init__()
           self.network = nn.Sequential(
               nn.Linear(state_dim + action_dim, 128),
               nn.ReLU(),
               nn.Linear(128, 128),
               nn.ReLU(),
               nn.Linear(128, 1),
               nn.Sigmoid()
           )

       def forward(self, state, action):
           x = torch.cat([state, action], dim=1)
           return self.network(x)

   class GAILNode(Node):
       def __init__(self):
           super().__init__('gail_node')

           # GAIL parameters
           self.state_dim = 10
           self.action_dim = 4  # One-hot encoded actions
           self.learning_rate = 0.001
           self.gamma = 0.99
           self.memory = deque(maxlen=10000)
           self.demo_memory = deque(maxlen=1000)  # Expert demonstrations

           # Initialize networks
           self.discriminator = Discriminator(self.state_dim, self.action_dim)
           self.policy_network = nn.Sequential(
               nn.Linear(self.state_dim, 128),
               nn.ReLU(),
               nn.Linear(128, 128),
               nn.ReLU(),
               nn.Linear(128, self.action_dim)
           )
           self.discriminator_optimizer = optim.Adam(self.discriminator.parameters(), lr=self.learning_rate)
           self.policy_optimizer = optim.Adam(self.policy_network.parameters(), lr=self.learning_rate)

           # Robot state
           self.laser_scan = None
           self.position = None
           self.goal = [5.0, 5.0]

           # Publishers and subscribers
           self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
           self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
           self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

           # Timer for control loop
           self.control_timer = self.create_timer(0.1, self.control_loop)

           # Initialize with some expert demonstrations (simulated)
           self.generate_expert_demonstrations()

           self.get_logger().info('GAIL Node Started')

       def generate_expert_demonstrations(self):
           # Simulate expert demonstrations for training
           # In real implementation, these would come from human demonstrations
           for _ in range(100):
               # Create simulated expert state-action pairs
               state = np.random.rand(self.state_dim)
               # Simulate "expert" action based on goal proximity
               action = np.random.randint(0, self.action_dim)
               self.demo_memory.append((state, action))

       def odom_callback(self, msg):
           self.position = [msg.pose.pose.position.x, msg.pose.pose.position.y]

       def scan_callback(self, msg):
           if len(msg.ranges) >= self.state_dim:
               step = len(msg.ranges) // self.state_dim
               simplified_scan = [min(r, 10.0) for r in msg.ranges[::step][:self.state_dim]]
               self.laser_scan = simplified_scan

       def get_state(self):
           if self.laser_scan is None or self.position is None:
               return np.zeros(self.state_dim)

           state = self.laser_scan.copy()
           state.extend([self.position[0], self.position[1],
                        self.goal[0], self.goal[1]])
           return np.array(state[:self.state_dim])

       def select_action(self, state):
           state_tensor = torch.FloatTensor(state).unsqueeze(0)
           action_probs = torch.softmax(self.policy_network(state_tensor), dim=1)
           action = torch.multinomial(action_probs, 1).item()
           return action

       def get_reward_from_discriminator(self, state, action):
           # Convert action to one-hot
           action_onehot = np.zeros(self.action_dim)
           action_onehot[action] = 1

           state_tensor = torch.FloatTensor(state).unsqueeze(0)
           action_tensor = torch.FloatTensor(action_onehot).unsqueeze(0)

           with torch.no_grad():
               prob_real = self.discriminator(state_tensor, action_tensor)
               # Reward is log-probability of being expert-like
               reward = torch.log(prob_real + 1e-8).item()
           return reward

       def train_discriminator(self, states, actions, next_states, demos_states, demos_actions):
           # Convert to tensors
           states_tensor = torch.FloatTensor(states)
           actions_tensor = torch.FloatTensor(np.eye(self.action_dim)[actions])  # One-hot encode
           demos_states_tensor = torch.FloatTensor(demos_states)
           demos_actions_tensor = torch.FloatTensor(np.eye(self.action_dim)[demos_actions])  # One-hot encode

           # Train discriminator to distinguish expert from agent
           self.discriminator_optimizer.zero_grad()

           # Loss on expert demonstrations (should output high probability)
           expert_loss = -torch.log(self.discriminator(demos_states_tensor, demos_actions_tensor) + 1e-8).mean()

           # Loss on agent demonstrations (should output low probability)
           agent_loss = -torch.log(1 - self.discriminator(states_tensor, actions_tensor) + 1e-8).mean()

           discriminator_loss = expert_loss + agent_loss
           discriminator_loss.backward()
           self.discriminator_optimizer.step()

           return discriminator_loss.item()

       def control_loop(self):
           state = self.get_state()
           action = self.select_action(state)

           # Execute action
           cmd_msg = Twist()
           if action == 0:  # Forward
               cmd_msg.linear.x = 0.5
           elif action == 1:  # Turn left
               cmd_msg.angular.z = 0.5
           elif action == 2:  # Turn right
               cmd_msg.angular.z = -0.5
           elif action == 3:  # Stop
               cmd_msg.linear.x = 0.0

           self.cmd_vel_pub.publish(cmd_msg)

           # Store experience
           self.memory.append((state, action))

           # Train discriminator and policy periodically
           if len(self.memory) >= 32 and len(self.demo_memory) >= 32:
               # Sample from memory and demonstrations
               batch_size = min(32, len(self.memory), len(self.demo_memory))
               agent_batch = random.sample(self.memory, batch_size)
               demo_batch = random.sample(self.demo_memory, batch_size)

               agent_states = [s[0] for s in agent_batch]
               agent_actions = [s[1] for s in agent_batch]
               demo_states = [s[0] for s in demo_batch]
               demo_actions = [s[1] for s in demo_batch]

               # Train discriminator
               disc_loss = self.train_discriminator(
                   agent_states, agent_actions, [], demo_states, demo_actions
               )

               # Update policy using discriminator reward
               if len(agent_batch) > 0:
                   agent_state = torch.FloatTensor(agent_states[0]).unsqueeze(0)
                   agent_action = torch.LongTensor([agent_actions[0]]).unsqueeze(1)

                   # Get reward from discriminator
                   disc_reward = self.get_reward_from_discriminator(agent_states[0], agent_actions[0])
                   disc_reward_tensor = torch.FloatTensor([disc_reward])

                   # Policy gradient update
                   action_probs = torch.softmax(self.policy_network(agent_state), dim=1)
                   selected_prob = action_probs.gather(1, agent_action)
                   policy_loss = -torch.log(selected_prob + 1e-8) * disc_reward_tensor

                   self.policy_optimizer.zero_grad()
                   policy_loss.backward()
                   self.policy_optimizer.step()

           self.get_logger().info(f'Action: {action}, D Loss: {disc_loss:.4f}')

   def main(args=None):
       rclpy.init(args=args)
       gail_node = GAILNode()

       try:
           rclpy.spin(gail_node)
       except KeyboardInterrupt:
           pass
       finally:
           gail_node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Add the GAIL node to `setup.py`.

### Task 3.2: Curiosity-Driven Learning
1. Create a curiosity-driven exploration system:
   ```python
   # advanced_rl_lab/curiosity_learning.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   import torch
   import torch.nn as nn
   import torch.optim as optim
   import numpy as np
   from collections import deque
   from geometry_msgs.msg import Twist
   from sensor_msgs.msg import LaserScan
   from nav_msgs.msg import Odometry
   import math

   class ForwardModel(nn.Module):
       def __init__(self, state_dim, action_dim):
           super(ForwardModel, self).__init__()
           self.network = nn.Sequential(
               nn.Linear(state_dim + action_dim, 128),
               nn.ReLU(),
               nn.Linear(128, 128),
               nn.ReLU(),
               nn.Linear(128, state_dim)
           )

       def forward(self, state, action):
           x = torch.cat([state, action], dim=1)
           return self.network(x)

   class InverseModel(nn.Module):
       def __init__(self, state_dim, action_dim):
           super(InverseModel, self).__init__()
           self.network = nn.Sequential(
               nn.Linear(state_dim * 2, 128),  # Current and next state
               nn.ReLU(),
               nn.Linear(128, 128),
               nn.ReLU(),
               nn.Linear(128, action_dim)
           )

       def forward(self, state, next_state):
           x = torch.cat([state, next_state], dim=1)
           return self.network(x)

   class CuriosityNode(Node):
       def __init__(self):
           super().__init__('curiosity_node')

           # Curiosity learning parameters
           self.state_dim = 10
           self.action_dim = 4
           self.learning_rate = 0.001
           self.memory = deque(maxlen=10000)

           # Initialize models
           self.forward_model = ForwardModel(self.state_dim, self.action_dim)
           self.inverse_model = InverseModel(self.state_dim, self.action_dim)
           self.policy_network = nn.Sequential(
               nn.Linear(self.state_dim, 128),
               nn.ReLU(),
               nn.Linear(128, 128),
               nn.ReLU(),
               nn.Linear(128, self.action_dim)
           )

           # Optimizers
           self.forward_optimizer = optim.Adam(self.forward_model.parameters(), lr=self.learning_rate)
           self.inverse_optimizer = optim.Adam(self.inverse_model.parameters(), lr=self.learning_rate)
           self.policy_optimizer = optim.Adam(self.policy_network.parameters(), lr=self.learning_rate)

           # Robot state
           self.laser_scan = None
           self.position = None
           self.goal = [5.0, 5.0]
           self.previous_state = None

           # Publishers and subscribers
           self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
           self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
           self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

           # Timer for control loop
           self.control_timer = self.create_timer(0.1, self.control_loop)

           self.get_logger().info('Curiosity Learning Node Started')

       def odom_callback(self, msg):
           self.position = [msg.pose.pose.position.x, msg.pose.pose.position.y]

       def scan_callback(self, msg):
           if len(msg.ranges) >= self.state_dim:
               step = len(msg.ranges) // self.state_dim
               simplified_scan = [min(r, 10.0) for r in msg.ranges[::step][:self.state_dim]]
               self.laser_scan = simplified_scan

       def get_state(self):
           if self.laser_scan is None or self.position is None:
               return np.zeros(self.state_dim)

           state = self.laser_scan.copy()
           state.extend([self.position[0], self.position[1],
                        self.goal[0], self.goal[1]])
           return np.array(state[:self.state_dim])

       def select_action(self, state):
           state_tensor = torch.FloatTensor(state).unsqueeze(0)
           action_probs = torch.softmax(self.policy_network(state_tensor), dim=1)
           action = torch.multinomial(action_probs, 1).item()
           return action

       def compute_curiosity_reward(self, state, action, next_state):
           # Convert to tensors
           state_tensor = torch.FloatTensor(state).unsqueeze(0)
           next_state_tensor = torch.FloatTensor(next_state).unsqueeze(0)
           action_tensor = torch.FloatTensor(np.eye(self.action_dim)[action]).unsqueeze(0)

           # Predict next state using forward model
           predicted_next_state = self.forward_model(state_tensor, action_tensor)

           # Compute prediction error (curiosity reward)
           curiosity_reward = torch.mean((predicted_next_state - next_state_tensor) ** 2).item()
           return curiosity_reward

       def train_models(self, state, action, next_state):
           # Convert to tensors
           state_tensor = torch.FloatTensor(state).unsqueeze(0)
           next_state_tensor = torch.FloatTensor(next_state).unsqueeze(0)
           action_tensor = torch.FloatTensor(np.eye(self.action_dim)[action]).unsqueeze(0)

           # Train forward model
           self.forward_optimizer.zero_grad()
           predicted_next_state = self.forward_model(state_tensor, action_tensor)
           forward_loss = torch.nn.MSELoss()(predicted_next_state, next_state_tensor)
           forward_loss.backward()
           self.forward_optimizer.step()

           # Train inverse model
           self.inverse_optimizer.zero_grad()
           predicted_action = self.inverse_model(state_tensor, next_state_tensor)
           inverse_loss = torch.nn.MSELoss()(predicted_action, action_tensor)
           inverse_loss.backward()
           self.inverse_optimizer.step()

           return forward_loss.item(), inverse_loss.item()

       def control_loop(self):
           current_state = self.get_state()
           action = self.select_action(current_state)

           # Execute action
           cmd_msg = Twist()
           if action == 0:  # Forward
               cmd_msg.linear.x = 0.5
           elif action == 1:  # Turn left
               cmd_msg.angular.z = 0.5
           elif action == 2:  # Turn right
               cmd_msg.angular.z = -0.5
           elif action == 3:  # Stop
               cmd_msg.linear.x = 0.0

           self.cmd_vel_pub.publish(cmd_msg)

           # Compute curiosity reward if we have previous state
           curiosity_reward = 0.0
           if self.previous_state is not None:
               curiosity_reward = self.compute_curiosity_reward(self.previous_state, action, current_state)

               # Train models
               forward_loss, inverse_loss = self.train_models(self.previous_state, action, current_state)

               # Update policy with curiosity reward
               state_tensor = torch.FloatTensor(self.previous_state).unsqueeze(0)
               action_tensor = torch.LongTensor([action]).unsqueeze(1)
               curiosity_reward_tensor = torch.FloatTensor([curiosity_reward])

               action_probs = torch.softmax(self.policy_network(state_tensor), dim=1)
               selected_prob = action_probs.gather(1, action_tensor)
               policy_loss = -torch.log(selected_prob + 1e-8) * curiosity_reward_tensor

               self.policy_optimizer.zero_grad()
               policy_loss.backward()
               self.policy_optimizer.step()

           # Store current state for next iteration
           self.previous_state = current_state.copy()

           self.get_logger().info(f'Action: {action}, Curiosity Reward: {curiosity_reward:.4f}')

   def main(args=None):
       rclpy.init(args=args)
       curiosity_node = CuriosityNode()

       try:
           rclpy.spin(curiosity_node)
       except KeyboardInterrupt:
           pass
       finally:
           curiosity_node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

## Exercise 4: Learning Evaluation and Safety (1 hour)

### Task 4.1: Learning Performance Evaluation
1. Create an evaluation system for advanced learning algorithms:
   ```python
   # advanced_rl_lab/learning_evaluator.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import Float32, String
   import time
   import numpy as np
   from collections import deque

   class LearningEvaluator(Node):
       def __init__(self):
           super().__init__('learning_evaluator')

           # Performance tracking
           self.episode_rewards = deque(maxlen=100)
           self.episode_lengths = deque(maxlen=100)
           self.success_rates = deque(maxlen=50)
           self.exploration_rates = deque(maxlen=100)
           self.safety_violations = 0
           self.start_time = time.time()

           # Subscriptions
           self.reward_sub = self.create_subscription(
               Float32, 'episode_reward', self.reward_callback, 10)
           self.success_sub = self.create_subscription(
               Float32, 'success_rate', self.success_callback, 10)
           self.exploration_sub = self.create_subscription(
               Float32, 'exploration_rate', self.exploration_callback, 10)
           self.safety_sub = self.create_subscription(
               String, 'safety_violation', self.safety_callback, 10)

           # Publishers
           self.performance_pub = self.create_publisher(
               String, 'learning_performance', 10)

           # Timer for periodic evaluation
           self.eval_timer = self.create_timer(10.0, self.evaluate_performance)

           self.get_logger().info('Learning Evaluator Started')

       def reward_callback(self, msg):
           self.episode_rewards.append(msg.data)

       def success_callback(self, msg):
           self.success_rates.append(msg.data)

       def exploration_callback(self, msg):
           self.exploration_rates.append(msg.data)

       def safety_callback(self, msg):
           self.safety_violations += 1

       def evaluate_performance(self):
           # Calculate performance metrics
           avg_reward = np.mean(self.episode_rewards) if self.episode_rewards else 0
           avg_success_rate = np.mean(self.success_rates) if self.success_rates else 0
           avg_exploration = np.mean(self.exploration_rates) if self.exploration_rates else 0
           total_time = time.time() - self.start_time

           # Calculate sample efficiency
           episodes_completed = len(self.episode_rewards)
           sample_efficiency = episodes_completed / total_time if total_time > 0 else 0

           # Create performance report
           performance_report = {
               'timestamp': time.time(),
               'total_time_seconds': total_time,
               'episodes_completed': episodes_completed,
               'avg_reward': avg_reward,
               'avg_success_rate': avg_success_rate,
               'avg_exploration_rate': avg_exploration,
               'safety_violations': self.safety_violations,
               'sample_efficiency': sample_efficiency,
               'learning_progress': self.assess_learning_progress()
           }

           # Publish performance metrics
           from std_msgs.msg import String
           perf_msg = String()
           import json
           perf_msg.data = json.dumps(performance_report, indent=2)
           self.performance_pub.publish(perf_msg)

           self.get_logger().info(f'Learning Performance: {json.dumps(performance_report, indent=2)}')

       def assess_learning_progress(self):
           if len(self.episode_rewards) < 10:
               return "Insufficient data for assessment"

           # Check if performance is improving
           recent_avg = np.mean(list(self.episode_rewards)[-10:])
           early_avg = np.mean(list(self.episode_rewards)[:10]) if len(self.episode_rewards) > 10 else recent_avg

           if recent_avg > early_avg * 1.1:
               return "Learning progressing well"
           elif abs(recent_avg - early_avg) < 0.1:
               return "Learning plateau detected"
           else:
               return "Performance degradation detected"

   def main(args=None):
       rclpy.init(args=args)
       evaluator = LearningEvaluator()

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

1. How would you modify the DQN algorithm to handle continuous action spaces?
2. What are the advantages and disadvantages of model-based vs. model-free RL in robotics?
3. How would you implement safe exploration in a real robotic system?
4. What challenges arise when applying meta-learning to robotics and how would you address them?
5. How would you evaluate the transferability of learned policies across different robot platforms?

## Troubleshooting Tips

- **Training instability**: Use experience replay and target networks
- **Sample inefficiency**: Implement curiosity-driven exploration
- **Safety concerns**: Add safety constraints and shielding
- **Domain gap**: Use domain randomization and adaptation
- **Computational complexity**: Optimize network architectures

## Extensions

1. Implement a hierarchical reinforcement learning system
2. Add attention mechanisms to focus on relevant sensory inputs
3. Create a multi-task learning system that learns multiple skills simultaneously
4. Implement learning from human feedback with preference learning
5. Add uncertainty quantification to the learning system

## Summary

This lab exercise provided hands-on experience with advanced robot learning techniques. You implemented deep reinforcement learning algorithms, meta-learning approaches, transfer learning systems, and curiosity-driven exploration. These skills are essential for creating robots that can learn complex behaviors and adapt to new situations efficiently.