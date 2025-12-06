# Week 4 Lab Exercise: Robot Simulation with Gazebo

## Overview

In this lab exercise, you will create detailed robot models for simulation, implement realistic physics and sensor models, and design simulation environments for testing. You will learn to use Gazebo for robot simulation and connect your simulated robots to ROS 2 using the Gazebo-ROS bridge. This builds upon your Week 1-3 foundations to create realistic simulation environments for robot development and testing.

## Prerequisites

- Completion of Week 1-3 lab exercises
- Working ROS 2 Humble Hawksbill installation
- Gazebo Garden (or compatible version) installed
- Understanding of URDF/XACRO for robot modeling
- Basic knowledge of ROS 2 communication patterns

## Learning Objectives

By completing this lab, you will:
- Create detailed robot models for simulation using URDF/XACRO
- Implement realistic physics models and Gazebo plugins for robot simulation
- Design simulation environments that accurately represent real-world scenarios
- Integrate sensors into simulated robots with realistic models
- Use Gazebo tools for robot testing and validation
- Connect simulated robots to ROS 2 using the Gazebo-ROS bridge

## Task 1: Install Gazebo and ROS 2 Integration

### Step 1.1: Verify Gazebo installation
```bash
# Check if Gazebo is installed
gz --version

# If not installed, install Gazebo Garden
sudo apt update
sudo apt install gz-garden

# Install ROS 2 Gazebo integration packages
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros
sudo apt install ros-humble-gazebo-plugins ros-humble-gazebo-dev
```

### Step 1.2: Install additional simulation packages
```bash
sudo apt install ros-humble-ros-gz ros-humble-ros-gz-bridge ros-humble-gz-ros2-control
sudo apt install ros-humble-joint-state-publisher ros-humble-robot-state-publisher
```

## Task 2: Create a Simple Robot Model

### Step 2.1: Create a robot description package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake robot_description
```

### Step 2.2: Create the URDF directory structure
```bash
mkdir -p ~/ros2_ws/src/robot_description/urdf
mkdir -p ~/ros2_ws/src/robot_description/meshes
mkdir -p ~/ros2_ws/src/robot_description/config
mkdir -p ~/ros2_ws/src/robot_description/launch
```

### Step 2.3: Create a simple differential drive robot URDF
Create `~/ros2_ws/src/robot_description/urdf/simple_robot.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_robot">

  <!-- Constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_width" value="0.04" />
  <xacro:property name="wheel_mass" value="0.5" />
  <xacro:property name="base_mass" value="5.0" />
  <xacro:property name="base_length" value="0.5" />
  <xacro:property name="base_width" value="0.3" />
  <xacro:property name="base_height" value="0.2" />
  <xacro:property name="wheel_offset_x" value="0.15" />
  <xacro:property name="wheel_offset_y" value="0.2" />

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${base_mass}"/>
      <inertia
        ixx="${base_mass/12.0 * (base_width*base_width + base_height*base_height)}"
        ixy="0.0"
        ixz="0.0"
        iyy="${base_mass/12.0 * (base_length*base_length + base_height*base_height)}"
        iyz="0.0"
        izz="${base_mass/12.0 * (base_length*base_length + base_width*base_width)}"/>
    </inertial>
  </link>

  <!-- Left Wheel -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel_link"/>
    <origin xyz="${wheel_offset_x} ${wheel_offset_y} 0" rpy="${-M_PI/2} 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="left_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${wheel_mass}"/>
      <inertia
        ixx="${wheel_mass/12.0 * (3*wheel_radius*wheel_radius + wheel_width*wheel_width)}"
        ixy="0.0"
        ixz="0.0"
        iyy="${wheel_mass/12.0 * (3*wheel_radius*wheel_radius + wheel_width*wheel_width)}"
        iyz="0.0"
        izz="${wheel_mass * wheel_radius * wheel_radius / 2.0}"/>
    </inertial>
  </link>

  <!-- Right Wheel -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel_link"/>
    <origin xyz="${wheel_offset_x} ${-wheel_offset_y} 0" rpy="${-M_PI/2} 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="right_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${wheel_mass}"/>
      <inertia
        ixx="${wheel_mass/12.0 * (3*wheel_radius*wheel_radius + wheel_width*wheel_width)}"
        ixy="0.0"
        ixz="0.0"
        iyy="${wheel_mass/12.0 * (3*wheel_radius*wheel_radius + wheel_width*wheel_width)}"
        iyz="0.0"
        izz="${wheel_mass * wheel_radius * wheel_radius / 2.0}"/>
    </inertial>
  </link>

  <!-- Caster Wheel -->
  <joint name="caster_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel_link"/>
    <origin xyz="${-wheel_offset_x} 0 ${-wheel_radius}" rpy="0 0 0"/>
  </joint>

  <link name="caster_wheel_link">
    <visual>
      <geometry>
        <sphere radius="${wheel_radius/2}"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="${wheel_radius/2}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Gazebo-specific tags -->
  <gazebo reference="base_link">
    <material>Gazebo/Grey</material>
  </gazebo>

  <gazebo reference="left_wheel_link">
    <material>Gazebo/Black</material>
    <mu1>0.5</mu1>
    <mu2>0.5</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
  </gazebo>

  <gazebo reference="right_wheel_link">
    <material>Gazebo/Black</material>
    <mu1>0.5</mu1>
    <mu2>0.5</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
  </gazebo>

  <gazebo reference="caster_wheel_link">
    <material>Gazebo/Black</material>
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
  </gazebo>

  <!-- Gazebo plugins -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find robot_description)/config/simple_robot_controllers.yaml</parameters>
    </plugin>
  </gazebo>

</robot>
```

## Task 3: Create Robot Controllers Configuration

### Step 3.1: Create controller configuration file
Create `~/ros2_ws/src/robot_description/config/simple_robot_controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]

    wheel_separation: 0.4
    wheel_radius: 0.1

    # Publish rate for the robot's state
    publish_rate: 50.0

    # Odometry parameters
    odom_frame_id: odom
    base_frame_id: base_link
    pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
    twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]

    # Velocity commands
    cmd_vel_timeout: 0.5
    publish_limited_velocity: true
    use_stamped_vel: true
```

## Task 4: Create Launch Files

### Step 4.1: Create robot spawn launch file
Create `~/ros2_ws/src/robot_description/launch/spawn_simple_robot.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot description
    robot_description_path = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'urdf',
        'simple_robot.urdf.xacro'
    ])

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_path
        }]
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )

    # Spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'simple_robot',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity
    ])
```

### Step 4.2: Create complete simulation launch file
Create `~/ros2_ws/src/robot_description/launch/simple_robot_simulation.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot description
    robot_description_path = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'urdf',
        'simple_robot.urdf.xacro'
    ])

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_path
        }]
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )

    # Spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'simple_robot',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # Controller manager
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Delay diff_drive_spawner after joint_state_broadcaster_spawner
    diff_drive_spawner_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        diff_drive_spawner_after_joint_state_broadcaster
    ])
```

## Task 5: Create a More Complex Robot with Sensors

### Step 5.1: Create a robot with sensors URDF
Create `~/ros2_ws/src/robot_description/urdf/robot_with_sensors.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot_with_sensors">

  <!-- Include the base robot -->
  <xacro:include filename="simple_robot.urdf.xacro" />

  <!-- Camera Mount -->
  <joint name="camera_mount_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- LIDAR Mount -->
  <joint name="lidar_mount_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>

  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Gazebo Camera Plugin -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera_sensor">
      <update_rate>30</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin filename="libgazebo_ros_camera.so" name="camera_controller">
        <frame_name>camera_link</frame_name>
        <min_depth>0.1</min_depth>
        <max_depth>100</max_depth>
        <update_rate>30.0</update_rate>
        <hack_baseline>0.07</hack_baseline>
        <distortion_k1>0.0</distortion_k1>
        <distortion_k2>0.0</distortion_k2>
        <distortion_k3>0.0</distortion_k3>
        <distortion_t1>0.0</distortion_t1>
        <distortion_t2>0.0</distortion_t2>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Gazebo LIDAR Plugin -->
  <gazebo reference="lidar_link">
    <sensor type="ray" name="lidar_sensor">
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin filename="libgazebo_ros_laser.so" name="laser_controller">
        <frame_name>lidar_link</frame_name>
        <topic_name>scan</topic_name>
        <update_rate>10</update_rate>
      </plugin>
    </sensor>
  </gazebo>

  <!-- IMU Sensor -->
  <gazebo reference="base_link">
    <sensor type="imu" name="imu_sensor">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
      <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
        <topic_name>imu</topic_name>
        <body_name>base_link</body_name>
        <update_rate>100</update_rate>
        <gaussian_noise>0.01</gaussian_noise>
        <frame_name>base_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

## Task 6: Create World Files

### Step 6.1: Create a simple world file
Create `~/ros2_ws/src/robot_description/worlds/simple_room.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_room">
    <!-- Physics -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Room walls -->
    <model name="wall_1">
      <pose>0 5 1 0 0 0</pose>
      <link name="link">
        <pose>0 0 1 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <gravity>1</gravity>
      </link>
    </model>

    <model name="wall_2">
      <pose>5 0 1 0 0 1.5707</pose>
      <link name="link">
        <pose>0 0 1 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <gravity>1</gravity>
      </link>
    </model>

    <model name="wall_3">
      <pose>0 -5 1 0 0 0</pose>
      <link name="link">
        <pose>0 0 1 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <gravity>1</gravity>
      </link>
    </model>

    <model name="wall_4">
      <pose>-5 0 1 0 0 1.5707</pose>
      <link name="link">
        <pose>0 0 1 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <gravity>1</gravity>
      </link>
    </model>

    <!-- Simple obstacle -->
    <model name="obstacle">
      <pose>2 2 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 1 1</ambient>
            <diffuse>0.5 0.5 1 1</diffuse>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <gravity>1</gravity>
      </link>
    </model>

  </world>
</sdf>
```

## Task 7: Create a Launch File with Custom World

### Step 7.1: Create launch file for custom world
Create `~/ros2_ws/src/robot_description/launch/robot_with_sensors.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='simple_room.sdf')

    # Robot description
    robot_description_path = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'urdf',
        'robot_with_sensors.urdf.xacro'
    ])

    # World path
    world_path = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'worlds',
        world
    ])

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_path
        }]
    )

    # Gazebo with custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_path,
            'use_sim_time': use_sim_time
        }.items()
    )

    # Spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot_with_sensors',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # Controller manager
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Delay diff_drive_spawner after joint_state_broadcaster_spawner
    diff_drive_spawner_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='simple_room.sdf',
            description='Choose one of the world files from `/robot_description/worlds`'
        ),
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        diff_drive_spawner_after_joint_state_broadcaster
    ])
```

## Task 8: Create a Teleoperation Node

### Step 8.1: Create a simple teleoperation node
Create `~/ros2_ws/src/robot_description/robot_description/teleop_keyboard.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
        'i': (1, 0),
        'o': (1, -1),
        'j': (0, 1),
        'l': (0, -1),
        'u': (1, 1),
        ',': (-1, 0),
        '.': (-1, 1),
        'm': (-1, -1),
    }

speedBindings = {
        'q': (1.1, 1.1),
        'z': (.9, .9),
        'w': (1.1, 1),
        'x': (.9, 1),
        'e': (1, 1.1),
        'c': (1, .9),
    }


class TeleopKeyboard(Node):

    def __init__(self):
        super().__init__('teleop_keyboard')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.speed = 0.5
        self.turn = 1.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.status = 0

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed, turn):
        return f"currently:\tspeed {speed}\tturn {turn}"

    def run(self):
        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info(msg)
        self.get_logger().info(self.vels(self.speed, self.turn))

        try:
            while True:
                key = self.getKey()
                if key in moveBindings.keys():
                    self.x = moveBindings[key][0]
                    self.th = moveBindings[key][1]
                elif key in speedBindings.keys():
                    self.speed = self.speed * speedBindings[key][0]
                    self.turn = self.turn * speedBindings[key][1]

                    self.get_logger().info(self.vels(self.speed, self.turn))
                    if (self.status == 14):
                        self.get_logger().info(msg)
                    self.status = (self.status + 1) % 15
                elif key == ' ' or key == 'k':
                    self.x = 0.0
                    self.y = 0.0
                    self.z = 0.0
                    self.th = 0.0
                else:
                    self.x = 0.0
                    self.y = 0.0
                    self.z = 0.0
                    self.th = 0.0
                    if (key == '\x03'):
                        break

                twist = Twist()
                twist.linear.x = self.x * self.speed
                twist.linear.y = self.y * self.speed
                twist.linear.z = self.z * self.speed
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = self.th * self.turn
                self.pub.publish(twist)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

        finally:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopKeyboard()

    try:
        teleop_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 8.2: Update setup.py for the robot_description package
Edit `~/ros2_ws/src/robot_description/setup.py` to add entry points:

```python
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Robot description package for Gazebo simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = robot_description.teleop_keyboard:main',
        ],
    },
)
```

## Task 9: Build and Test the Simulation

### Step 9.1: Build the robot_description package
```bash
cd ~/ros2_ws
colcon build --packages-select robot_description
source install/setup.bash
```

### Step 9.2: Test the simple robot simulation
```bash
# Launch the simulation with the simple robot
ros2 launch robot_description simple_robot_simulation.launch.py
```

### Step 9.3: In another terminal, teleoperate the robot
```bash
# Make sure to source the workspace
source ~/ros2_ws/install/setup.bash

# Run the teleoperation node
ros2 run robot_description teleop_keyboard
```

### Step 9.4: Test the robot with sensors simulation
```bash
# Launch the simulation with the robot that has sensors
ros2 launch robot_description robot_with_sensors.launch.py
```

### Step 9.5: Visualize sensor data in Rviz
```bash
# In a separate terminal
source ~/ros2_ws/install/setup.bash
rviz2
```

In Rviz, you can add displays for:
- Robot model: Use RobotModel display, set topic to `/robot_description`
- Laser scan: Set topic to `/scan` to see LIDAR data
- Camera feed: Set topic to `/camera/image_raw` to see camera feed
- IMU data: Set topic to `/imu` to see IMU readings
- Odometry: Set topic to `/odom` to see robot position

## Task 10: Create Additional World Files

### Step 10.1: Create an outdoor world
Create `~/ros2_ws/src/robot_description/worlds/outdoor_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="outdoor_world">
    <!-- Physics -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Simple maze -->
    <model name="maze_wall_1">
      <pose>0 3 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>5 0.2 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>5 0.2 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.7 0.5 1</ambient>
            <diffuse>0.5 0.7 0.5 1</diffuse>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <gravity>1</gravity>
      </link>
    </model>

    <model name="maze_wall_2">
      <pose>2 1 0.5 0 0 1.5707</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>3 0.2 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>3 0.2 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.7 0.5 1</ambient>
            <diffuse>0.5 0.7 0.5 1</diffuse>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <gravity>1</gravity>
      </link>
    </model>

    <model name="maze_wall_3">
      <pose>-2 -1 0.5 0 0 1.5707</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>3 0.2 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>3 0.2 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.7 0.5 1</ambient>
            <diffuse>0.5 0.7 0.5 1</diffuse>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <gravity>1</gravity>
      </link>
    </model>

    <model name="maze_wall_4">
      <pose>0 -3 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>5 0.2 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>5 0.2 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.7 0.5 1</ambient>
            <diffuse>0.5 0.7 0.5 1</diffuse>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <gravity>1</gravity>
      </link>
    </model>

    <!-- Goal object -->
    <model name="goal">
      <pose>4 0 0.2 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>0.4</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>0.4</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <gravity>1</gravity>
      </link>
    </model>

  </world>
</sdf>
```

## Assessment Questions

1. Explain the differences between URDF and XACRO. What are the advantages of using XACRO for robot modeling?

2. Describe the key physics parameters in Gazebo and how they affect robot simulation behavior.

3. What are the essential Gazebo plugins needed for a mobile robot with sensors, and what does each plugin do?

4. Explain the simulation-to-reality gap and strategies to minimize it.

5. How do you integrate sensors (camera, LIDAR, IMU) into a Gazebo simulation, and what parameters are important for realistic simulation?

## Troubleshooting Tips

- If Gazebo doesn't start, check that it's properly installed with `gz --version`
- If robot doesn't spawn, verify that the URDF is valid and the topic names match
- If controllers don't load, check that the controller configuration file is properly formatted
- If sensors don't publish data, verify plugin configuration and topic names
- For performance issues, reduce physics update rate or simplify collision models
- Use `ros2 topic list` and `ros2 topic echo` to verify that topics are being published

## Summary

In this lab, you've implemented comprehensive robot simulation systems:
- Created detailed robot models using URDF/XACRO
- Implemented realistic physics and sensor models in Gazebo
- Designed simulation environments for testing
- Connected simulated robots to ROS 2 using the Gazebo-ROS bridge
- Created launch files for easy simulation setup
- Implemented teleoperation for robot control

These simulation capabilities are essential for safe and cost-effective robot development and form the foundation for testing more complex robot behaviors in subsequent weeks.