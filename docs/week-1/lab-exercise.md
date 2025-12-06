# Week 1 Lab Exercise: Setting Up Your Development Environment

## Overview

In this lab exercise, you will set up your development environment for Physical AI and humanoid robotics development. You will install ROS 2, create your first ROS 2 package, implement publisher/subscriber nodes, and run your first robot simulation in Gazebo.

## Prerequisites

- Ubuntu 22.04 LTS installed (or appropriate OS for your system)
- Internet connection for package installation
- Basic command line familiarity
- Administrative access to install software

## Learning Objectives

By completing this lab, you will:
- Successfully install ROS 2 Humble Hawksbill
- Create and build a ROS 2 package
- Implement publisher and subscriber nodes
- Launch a basic robot simulation in Gazebo
- Publish sensor data and control basic robot movements

## Task 1: Install ROS 2 Humble Hawksbill

### Step 1.1: Set up your sources.list
```bash
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 1.2: Install ROS 2 packages
```bash
sudo apt update
sudo apt install ros-humble-desktop
```

### Step 1.3: Install additional tools
```bash
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

### Step 1.4: Source the ROS 2 environment
```bash
source /opt/ros/humble/setup.bash
```

To make this permanent, add the following to your `~/.bashrc`:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Task 2: Create Your First ROS 2 Workspace and Package

### Step 2.1: Create a workspace directory
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### Step 2.2: Create a new package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python beginner_tutorials
```

### Step 2.3: Navigate to the package
```bash
cd ~/ros2_ws/src/beginner_tutorials
ls -la
```

You should see the package structure with directories like `launch/`, `test/`, and files like `setup.py`, `package.xml`.

## Task 3: Create a Publisher Node

### Step 3.1: Create the publisher script
Navigate to the package's Python directory and create a publisher script:

```bash
mkdir -p ~/ros2_ws/src/beginner_tutorials/beginner_tutorials
```

Create a file `~/ros2_ws/src/beginner_tutorials/beginner_tutorials/talker.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 3.2: Make the script executable
```bash
chmod +x ~/ros2_ws/src/beginner_tutorials/beginner_tutorials/talker.py
```

## Task 4: Create a Subscriber Node

Create a file `~/ros2_ws/src/beginner_tutorials/beginner_tutorials/listener.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 4.1: Make the subscriber script executable
```bash
chmod +x ~/ros2_ws/src/beginner_tutorials/beginner_tutorials/listener.py
```

## Task 5: Update Package Configuration

### Step 5.1: Update setup.py
Edit the `setup.py` file in your package directory to add entry points for your nodes:

```python
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'beginner_tutorials'

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
    description='Beginner tutorials for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = beginner_tutorials.talker:main',
            'listener = beginner_tutorials.listener:main',
        ],
    },
)
```

## Task 6: Build and Test Your Package

### Step 6.1: Build your package
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash  # if not in .bashrc
colcon build --packages-select beginner_tutorials
```

### Step 6.2: Source the workspace
```bash
source ~/ros2_ws/install/setup.bash
```

### Step 6.3: Test the publisher and subscriber
Open a new terminal and run the publisher:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run beginner_tutorials talker
```

In another terminal, run the subscriber:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run beginner_tutorials listener
```

You should see the publisher sending messages and the subscriber receiving them.

## Task 7: Install and Run Gazebo

### Step 7.1: Install Gazebo Garden
```bash
sudo apt install ros-humble-gazebo-*
sudo apt install gazebo
```

### Step 7.2: Test Gazebo installation
```bash
gazebo
```

This should open the Gazebo simulation environment with a default world.

## Task 8: Launch a Basic Robot Simulation

### Step 8.1: Create a simple robot model
Create a directory for your robot models:
```bash
mkdir -p ~/ros2_ws/src/beginner_tutorials/models/simple_robot
```

Create a URDF file `~/ros2_ws/src/beginner_tutorials/models/simple_robot/simple_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <link name="sensor_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="sensor_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor_link"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
  </joint>
</robot>
```

### Step 8.2: Launch Gazebo with your robot
```bash
# Start Gazebo
gazebo --verbose

# In another terminal, spawn your robot (after Gazebo is running):
gz topic -t /world/default/create -m gz.msgs.EntityFactory -p 'name: "simple_robot", sdf: "<sdf version=\"1.7\"><model name=\"simple_robot\"><pose>0 0 0.1 0 0 0</pose><link name=\"base_link\"><visual name=\"visual\"><geometry><box><size>0.5 0.5 0.2</size></box></geometry></visual><collision name=\"collision\"><geometry><box><size>0.5 0.5 0.2</size></box></geometry></collision><inertial><mass>1.0</mass><inertia><ixx>0.1</ixx><ixy>0.0</ixy><ixz>0.0</ixz><iyy>0.1</iyy><iyz>0.0</iyz><izz>0.1</izz></inertia></inertial></link></model></sdf>"'
```

## Task 9: Connect ROS 2 with Gazebo (ROS 2 Control)

### Step 9.1: Install ROS 2 control packages
```bash
sudo apt install ros-humble-ros2-control*
sudo apt install ros-humble-ros2-controllers*
```

### Step 9.2: Test ROS 2 with Gazebo connection
```bash
# Check available ROS 2 topics
source ~/ros2_ws/install/setup.bash
ros2 topic list
```

## Task 10: Document Your Setup Process

Create a README.md file in your workspace documenting:
1. The steps you took to set up your environment
2. Any issues you encountered and how you resolved them
3. A summary of what you learned about ROS 2 and Gazebo

## Assessment Questions

1. Explain the difference between a ROS 2 node, topic, and message.
2. What is the purpose of the `colcon build` command?
3. Why is simulation important in robotics development?
4. What are the advantages of using ROS 2 over other robotics frameworks?

## Troubleshooting Tips

- If you encounter issues with package installation, ensure your Ubuntu system is updated: `sudo apt update && sudo apt upgrade`
- If ROS commands are not found, ensure you've sourced the correct setup.bash file
- If Gazebo doesn't launch, check that your graphics drivers are properly installed
- For Python permission issues, make sure your scripts are executable with `chmod +x`

## Summary

In this lab, you successfully:
- Installed ROS 2 Humble Hawksbill
- Created your first ROS 2 package with publisher and subscriber nodes
- Ran your nodes and observed message passing
- Installed and tested Gazebo simulation environment
- Created a basic robot model

This completes the foundational setup for your Physical AI and humanoid robotics development environment. In future weeks, you'll build upon this foundation to create more sophisticated robot behaviors and capabilities.