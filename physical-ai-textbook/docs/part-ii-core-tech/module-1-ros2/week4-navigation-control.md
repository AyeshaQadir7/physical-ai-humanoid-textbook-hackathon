---
title: Week 4 - ROS 2 Navigation & Control Systems
description: Implementing navigation and control systems for humanoid robots
hide_table_of_contents: false
---

# Week 4: ROS 2 Navigation & Control Systems

## Learning Objectives

By the end of this week, you will be able to:
- Implement navigation stacks for humanoid robot locomotion
- Design control systems for robot movement and manipulation
- Integrate sensors and actuators with the navigation system
- Create custom controllers for humanoid robot joints

## Overview of Navigation and Control Systems

Navigation and control systems are the backbone of autonomous robots. Navigation enables robots to move from one location to another safely and efficiently, while control systems manage the low-level actuator commands that make the robot move.

### Navigation System Components

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Global        │    │   Local         │    │   Controller    │
│   Planner       │───▶│   Planner       │───▶│   & Driver      │
│   (Path)        │    │   (Trajectory)  │    │   (Commands)    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
        │                       │                       │
        ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Map           │    │   Costmap       │    │   Robot         │
│   (Static)      │    │   (Dynamic)     │    │   (Physical)    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Navigation Stack Architecture

### 1. Global Planner
- Computes a path from start to goal using static map
- Outputs a series of waypoints
- Uses algorithms like A*, Dijkstra, or NavFn

### 2. Local Planner
- Creates executable trajectories from global path
- Handles dynamic obstacles and real-time adjustments
- Uses algorithms like DWA, TEB, or MPC

### 3. Costmap
- Maintains representation of environment
- Static costmap: permanent obstacles
- Local costmap: temporary obstacles and robot footprint

## Navigation2 (Nav2) Implementation

### Basic Navigation Stack Setup

```python
# launch/navigation.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    autostart_launch_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Auto start navigator'
    )

    params_file_launch_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('nav2_bringup'),
            'params',
            'nav2_params.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        use_sim_time_launch_arg,
        autostart_launch_arg,
        params_file_launch_arg,

        # Navigation stack
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel', '/cmd_vel_nav')]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                       {'autostart': autostart},
                       {'node_names': ['controller_server',
                                     'planner_server',
                                     'recoveries_server',
                                     'bt_navigator',
                                     'waypoint_follower']}]
        )
    ])
```

### Navigation Parameters Configuration

Create `config/nav2_params.yaml`:

```yaml
amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.1
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

amcl_map_client:
  ros__parameters:
    use_sim_time: False

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: False

bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "odom"
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    navigate_through_poses: False
    navigate_to_pose_rclcpp_node:
      ros__parameters:
        use_sim_time: False

bt_navigator_rclcpp_node:
  ros__parameters:
    use_sim_time: False

controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker parameters
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    # Controller parameters
    FollowPath:
      plugin: "nav2_rotation_shim::RotationShimController"
      progress_checker_plugin: "progress_checker"
      goal_checker_plugin: "goal_checker"
      RotateToGoal:
        plugin: "nav2_controller::SimpleGoal Oriented"
        desired_linear_vel: 0.5
        lookahead_dist: 0.6
        max_angular_accel: 1.6
        rotation_tol: 0.05
      LineFollow:
        plugin: "nav2_controller::SimplePurePlanner"
        lookahead_dist: 0.6
        linear_scale: 2.0
        angular_scale: 3.0
        lookahead_time: 1.5

controller_server_rclcpp_node:
  ros__parameters:
    use_sim_time: False

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: "odom"
      robot_base_frame: "base_link"
      use_sim_time: False
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      always_send_full_costmap: True
  local_costmap_client:
    ros__parameters:
      use_sim_time: False
  local_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: False

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: "map"
      robot_base_frame: "base_link"
      use_sim_time: False
      robot_radius: 0.22
      resolution: 0.05
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True
  global_costmap_client:
    ros__parameters:
      use_sim_time: False
  global_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: False

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: False
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

planner_server_rclcpp_node:
  ros__parameters:
    use_sim_time: False

recoveries_server:
  ros__parameters:
    costmap_topic: "local_costmap/costmap_raw"
    footprint_topic: "local_costmap/published_footprint"
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries::Spin"
      sim_granularity: 0.017
      angle: 1.57
      time_allowance: 10.0
    backup:
      plugin: "nav2_recoveries::BackUp"
      sim_granularity: 0.0044
      duration: 10.0
      sim_time: 2.0
    wait:
      plugin: "nav2_recoveries::Wait"
      sim_time: 2.0
      time_allowance: 10.0

recoveries_server_rclcpp_node:
  ros__parameters:
    use_sim_time: False

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200
```

## Custom Navigation Controller for Humanoid Robots

### Humanoid-Specific Navigation Considerations

Humanoid robots have unique navigation requirements:
- Bipedal locomotion constraints
- Center of gravity management
- Balance preservation
- Step planning for walking

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import math

class HumanoidNavigationController(Node):
    def __init__(self):
        super().__init__('humanoid_navigation_controller')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_cmd_publisher = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        self.balance_cmd_publisher = self.create_publisher(Float64MultiArray, 'balance_commands', 10)

        # Subscribers
        self.path_subscriber = self.create_subscription(
            Path, 'global_plan', self.path_callback, 10)
        self.odom_subscriber = self.create_subscription(
            PoseStamped, 'current_pose', self.odom_callback, 10)
        self.joint_state_subscriber = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)

        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        # Navigation state
        self.current_path = []
        self.current_pose = None
        self.joint_states = None
        self.current_waypoint_idx = 0
        self.navigation_active = False

        # Humanoid-specific parameters
        self.step_size = 0.3  # meters per step
        self.step_height = 0.05  # meters
        self.max_step_velocity = 0.5  # m/s
        self.balance_threshold = 0.1  # balance error threshold

        # Walking pattern parameters
        self.walk_phase = 0.0  # 0.0 to 1.0
        self.walk_frequency = 1.0  # steps per second

        self.get_logger().info('Humanoid Navigation Controller initialized')

    def path_callback(self, msg):
        """Receive global path from planner"""
        self.current_path = msg.poses
        self.current_waypoint_idx = 0
        self.navigation_active = len(self.current_path) > 0
        self.get_logger().info(f'Received path with {len(self.current_path)} waypoints')

    def odom_callback(self, msg):
        """Receive current robot pose"""
        self.current_pose = msg.pose

    def joint_state_callback(self, msg):
        """Receive current joint states"""
        self.joint_states = msg

    def calculate_footstep_sequence(self, start_pose, end_pose):
        """Calculate sequence of footsteps from start to end pose"""
        # Calculate distance and direction
        dx = end_pose.position.x - start_pose.position.x
        dy = end_pose.position.y - start_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        if distance < 0.01:  # Very close, no need to move
            return []

        # Calculate direction
        direction = math.atan2(dy, dx)

        # Generate footsteps
        num_steps = int(distance / self.step_size) + 1
        footsteps = []

        for i in range(num_steps):
            step_progress = i / num_steps
            step_x = start_pose.position.x + dx * step_progress
            step_y = start_pose.position.y + dy * step_progress

            # Add small offset for alternating feet
            if i % 2 == 0:  # Left foot
                step_y += 0.1  # Offset to the left
            else:  # Right foot
                step_y -= 0.1  # Offset to the right

            footsteps.append((step_x, step_y, direction))

        return footsteps

    def generate_walking_pattern(self, target_velocity):
        """Generate walking pattern based on target velocity"""
        # Calculate walking parameters based on desired velocity
        if abs(target_velocity.linear.x) < 0.01:
            # Stopped, maintain balance
            return self.generate_balance_pattern()

        # Calculate step timing based on velocity
        step_timing = min(1.0 / self.walk_frequency, 0.5 / abs(target_velocity.linear.x))

        # Generate walking joint commands
        joint_commands = Float64MultiArray()

        # Simplified walking pattern (in real implementation, this would be more complex)
        # Calculate joint angles for walking based on current phase
        left_hip_angle = 0.1 * math.sin(2 * math.pi * self.walk_phase)
        right_hip_angle = 0.1 * math.sin(2 * math.pi * self.walk_phase + math.pi)
        left_knee_angle = 0.05 * math.sin(2 * math.pi * self.walk_phase)
        right_knee_angle = 0.05 * math.sin(2 * math.pi * self.walk_phase + math.pi)

        # Balance adjustments
        balance_offset = target_velocity.angular.z * 0.2  # Turn compensation

        joint_commands.data = [
            left_hip_angle + balance_offset,
            right_hip_angle - balance_offset,
            left_knee_angle,
            right_knee_angle,
            # Add other joint angles as needed
        ]

        return joint_commands

    def generate_balance_pattern(self):
        """Generate balance maintenance pattern"""
        joint_commands = Float64MultiArray()

        # Maintain neutral standing position with slight balance adjustments
        joint_commands.data = [0.0, 0.0, 0.0, 0.0]  # Simplified

        return joint_commands

    def check_balance(self):
        """Check if robot is in balance"""
        if self.joint_states is None:
            return True  # Assume balanced if no data

        # Check joint positions for balance (simplified)
        # In real implementation, this would use IMU data and more sophisticated checks
        balance_error = 0.0

        # Example: Check if center of mass is within support polygon
        # This is a simplified check - real implementation would be more complex
        if self.joint_states.position:
            # Calculate approximate center of mass position
            # For now, just return True
            pass

        return abs(balance_error) < self.balance_threshold

    def control_loop(self):
        """Main control loop"""
        if not self.navigation_active or self.current_pose is None:
            # If not navigating, maintain balance
            balance_cmd = self.generate_balance_pattern()
            self.balance_cmd_publisher.publish(balance_cmd)
            return

        if self.current_waypoint_idx >= len(self.current_path):
            # Reached goal, stop and maintain balance
            self.navigation_active = False
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_publisher.publish(cmd)

            balance_cmd = self.generate_balance_pattern()
            self.balance_cmd_publisher.publish(balance_cmd)
            return

        # Get current target waypoint
        target_pose = self.current_path[self.current_waypoint_idx].pose

        # Calculate distance to target
        dx = target_pose.position.x - self.current_pose.position.x
        dy = target_pose.position.y - self.current_pose.position.y
        distance_to_target = math.sqrt(dx*dx + dy*dy)

        # Check if reached current waypoint
        if distance_to_target < 0.2:  # 20 cm tolerance
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.current_path):
                # Reached final waypoint
                self.navigation_active = False
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_publisher.publish(cmd)
                return
            else:
                # Move to next waypoint
                target_pose = self.current_path[self.current_waypoint_idx].pose

        # Calculate desired velocity toward target
        cmd = Twist()

        # Calculate distance and angle to target
        angle_to_target = math.atan2(dy, dx)
        angle_diff = angle_to_target - self.current_pose.orientation.z  # Simplified orientation

        # Normalize angle difference
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Set angular velocity to turn toward target
        cmd.angular.z = max(-1.0, min(1.0, angle_diff * 2.0))  # Limit angular velocity

        # Set linear velocity based on distance (slow down when close)
        cmd.linear.x = min(self.max_step_velocity, max(0.1, distance_to_target * 0.5))

        # Generate walking pattern based on desired velocity
        walking_pattern = self.generate_walking_pattern(cmd)

        # Check balance before publishing commands
        if self.check_balance():
            self.cmd_vel_publisher.publish(cmd)
            self.joint_cmd_publisher.publish(walking_pattern)
        else:
            # Robot is not in balance, stop and try to recover
            emergency_cmd = Twist()
            emergency_cmd.linear.x = 0.0
            emergency_cmd.angular.z = 0.0
            self.cmd_vel_publisher.publish(emergency_cmd)

            balance_recovery = self.generate_balance_pattern()
            self.balance_cmd_publisher.publish(balance_recovery)
            self.get_logger().warn('Balance error detected, executing recovery')

        # Update walk phase
        self.walk_phase = (self.walk_phase + 0.05) % 1.0  # Increment phase

def main(args=None):
    rclpy.init(args=args)
    humanoid_navigation_controller = HumanoidNavigationController()

    try:
        rclpy.spin(humanoid_navigation_controller)
    except KeyboardInterrupt:
        pass
    finally:
        humanoid_navigation_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Joint Control Systems for Humanoid Robots

### Joint State Publisher and Controller

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
import numpy as np
import math

class HumanoidJointController(Node):
    def __init__(self):
        super().__init__('humanoid_joint_controller')

        # Publishers
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.trajectory_publisher = self.create_publisher(JointTrajectory, 'joint_trajectory', 10)
        self.controller_state_publisher = self.create_publisher(JointTrajectoryControllerState, 'controller_state', 10)

        # Subscribers
        self.trajectory_subscriber = self.create_subscription(
            JointTrajectory, 'joint_trajectory_commands', self.trajectory_callback, 10)

        # Timer for joint state publishing
        self.state_timer = self.create_timer(0.01, self.publish_joint_states)  # 100 Hz

        # Robot joint parameters
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint',
            'head_pan_joint', 'head_tilt_joint'
        ]

        # Initialize joint positions (standing position)
        self.joint_positions = [0.0] * len(self.joint_names)
        self.joint_velocities = [0.0] * len(self.joint_names)
        self.joint_efforts = [0.0] * len(self.joint_names)

        # Joint limits (simplified)
        self.joint_limits = {
            'left_hip_joint': (-1.57, 1.57),    # -90° to 90°
            'left_knee_joint': (0.0, 2.35),     # 0° to 135°
            'left_ankle_joint': (-0.78, 0.78),  # -45° to 45°
            'right_hip_joint': (-1.57, 1.57),
            'right_knee_joint': (0.0, 2.35),
            'right_ankle_joint': (-0.78, 0.78),
            'left_shoulder_joint': (-1.57, 1.57),
            'left_elbow_joint': (-2.35, 0.0),
            'right_shoulder_joint': (-1.57, 1.57),
            'right_elbow_joint': (-2.35, 0.0),
            'head_pan_joint': (-1.57, 1.57),
            'head_tilt_joint': (-0.78, 0.78)
        }

        # Trajectory execution
        self.trajectory_active = False
        self.current_trajectory = None
        self.trajectory_point_idx = 0
        self.trajectory_start_time = None

        self.get_logger().info('Humanoid Joint Controller initialized')

    def trajectory_callback(self, msg):
        """Receive joint trajectory commands"""
        self.current_trajectory = msg
        self.trajectory_point_idx = 0
        self.trajectory_active = True
        self.trajectory_start_time = self.get_clock().now()

        self.get_logger().info(f'Received trajectory with {len(msg.points)} points')

    def interpolate_trajectory(self, start_time, current_time, start_point, end_point):
        """Interpolate between two trajectory points"""
        total_duration = (end_point.time_from_start.sec + end_point.time_from_start.nanosec / 1e9)
        elapsed_duration = (current_time.nanoseconds - start_time.nanoseconds) / 1e9

        if elapsed_duration >= total_duration or total_duration <= 0:
            # Return end point
            return end_point.positions, end_point.velocities, end_point.accelerations

        # Linear interpolation
        t = elapsed_duration / total_duration

        interpolated_positions = []
        interpolated_velocities = []
        interpolated_accelerations = []

        for i in range(len(start_point.positions)):
            pos = start_point.positions[i] + t * (end_point.positions[i] - start_point.positions[i])
            interpolated_positions.append(pos)

            if len(start_point.velocities) > i and len(end_point.velocities) > i:
                vel = start_point.velocities[i] + t * (end_point.velocities[i] - start_point.velocities[i])
                interpolated_velocities.append(vel)
            else:
                interpolated_velocities.append(0.0)

            if len(start_point.accelerations) > i and len(end_point.accelerations) > i:
                acc = start_point.accelerations[i] + t * (end_point.accelerations[i] - start_point.accelerations[i])
                interpolated_accelerations.append(acc)
            else:
                interpolated_accelerations.append(0.0)

        return interpolated_positions, interpolated_velocities, interpolated_accelerations

    def execute_trajectory(self):
        """Execute the current trajectory"""
        if not self.trajectory_active or self.current_trajectory is None:
            return

        if self.trajectory_point_idx >= len(self.current_trajectory.points):
            # Trajectory completed
            self.trajectory_active = False
            self.get_logger().info('Trajectory completed')
            return

        # Get current time
        current_time = self.get_clock().now()

        # Get current and next trajectory points
        start_point = self.current_trajectory.points[self.trajectory_point_idx]

        if self.trajectory_point_idx + 1 < len(self.current_trajectory.points):
            end_point = self.current_trajectory.points[self.trajectory_point_idx + 1]
            start_time = (self.trajectory_start_time if self.trajectory_point_idx == 0
                         else self.trajectory_start_time +
                         rclpy.time.Duration(seconds=self.current_trajectory.points[self.trajectory_point_idx].time_from_start.sec,
                                           nanoseconds=self.current_trajectory.points[self.trajectory_point_idx].time_from_start.nanosec))

            # Interpolate between points
            positions, velocities, accelerations = self.interpolate_trajectory(
                start_time, current_time, start_point, end_point)
        else:
            # Last point - just hold position
            positions = start_point.positions
            velocities = start_point.velocities if start_point.velocities else [0.0] * len(positions)
            accelerations = start_point.accelerations if start_point.accelerations else [0.0] * len(positions)

        # Update joint positions with trajectory
        for i, joint_name in enumerate(self.current_trajectory.joint_names):
            try:
                joint_idx = self.joint_names.index(joint_name)
                self.joint_positions[joint_idx] = positions[i]

                # Apply joint limits
                min_limit, max_limit = self.joint_limits.get(joint_name, (-float('inf'), float('inf')))
                self.joint_positions[joint_idx] = max(min_limit, min(max_limit, self.joint_positions[joint_idx]))

            except ValueError:
                # Joint name not found in our list
                continue

        # Check if reached current trajectory point
        point_duration = rclpy.time.Duration(
            seconds=start_point.time_from_start.sec,
            nanoseconds=start_point.time_from_start.nanosec
        )
        elapsed = current_time - self.trajectory_start_time

        if elapsed.nanoseconds >= point_duration.nanoseconds:
            self.trajectory_point_idx += 1
            self.trajectory_start_time = current_time

    def publish_joint_states(self):
        """Publish current joint states"""
        # Execute trajectory if active
        if self.trajectory_active:
            self.execute_trajectory()

        # Create and publish joint state message
        msg = JointState()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = self.joint_velocities
        msg.effort = self.joint_efforts
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        self.joint_state_publisher.publish(msg)

        # Publish controller state
        controller_msg = JointTrajectoryControllerState()
        controller_msg.header = msg.header
        controller_msg.joint_names = self.joint_names
        controller_msg.desired.positions = self.joint_positions
        controller_msg.desired.velocities = self.joint_velocities
        controller_msg.actual.positions = self.joint_positions
        controller_msg.error.positions = [0.0] * len(self.joint_positions)  # Simplified

        self.controller_state_publisher.publish(controller_msg)

def main(args=None):
    rclpy.init(args=args)
    humanoid_joint_controller = HumanoidJointController()

    try:
        rclpy.spin(humanoid_joint_controller)
    except KeyboardInterrupt:
        pass
    finally:
        humanoid_joint_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Balance Control System

### Center of Mass and Stability Control

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Point, Wrench
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import math

class HumanoidBalanceController(Node):
    def __init__(self):
        super().__init__('humanoid_balance_controller')

        # Publishers
        self.balance_cmd_publisher = self.create_publisher(Float64MultiArray, 'balance_commands', 10)
        self.com_publisher = self.create_publisher(Point, 'center_of_mass', 10)
        self.zmp_publisher = self.create_publisher(Point, 'zero_moment_point', 10)
        self.stability_publisher = self.create_publisher(Wrench, 'stability_metrics', 10)

        # Subscribers
        self.imu_subscriber = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10)
        self.joint_state_subscriber = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)

        # Timer for balance control
        self.balance_timer = self.create_timer(0.01, self.balance_control_loop)  # 100 Hz

        # Robot parameters
        self.robot_height = 0.8  # meters
        self.robot_mass = 30.0   # kg
        self.gravity = 9.81      # m/s^2

        # Balance state
        self.imu_data = None
        self.joint_states = None
        self.center_of_mass = Point(x=0.0, y=0.0, z=self.robot_height/2.0)
        self.support_polygon = []  # Points defining support polygon (feet positions)

        # PID controller parameters for balance
        self.balance_pid = {
            'kp': 50.0,  # Proportional gain
            'ki': 1.0,   # Integral gain
            'kd': 10.0   # Derivative gain
        }
        self.balance_error_integral = 0.0
        self.previous_balance_error = 0.0

        # Zero Moment Point (ZMP) calculation
        self.zmp = Point(x=0.0, y=0.0, z=0.0)

        self.get_logger().info('Humanoid Balance Controller initialized')

    def imu_callback(self, msg):
        """Receive IMU data for orientation and acceleration"""
        self.imu_data = msg

    def joint_state_callback(self, msg):
        """Receive joint states to calculate CoM and support polygon"""
        self.joint_states = msg
        self.update_com_and_support_polygon()

    def update_com_and_support_polygon(self):
        """Update center of mass and support polygon based on joint states"""
        if self.joint_states is None:
            return

        # Simplified center of mass calculation
        # In real implementation, this would use full kinematic model
        total_mass = self.robot_mass
        com_x = 0.0
        com_y = 0.0
        com_z = self.robot_height / 2.0  # Approximate CoM height

        # Calculate CoM based on joint positions (simplified)
        # This is a basic approximation - real implementation would be more complex
        if 'left_ankle_joint' in self.joint_states.name and 'right_ankle_joint' in self.joint_states.name:
            left_ankle_idx = self.joint_states.name.index('left_ankle_joint')
            right_ankle_idx = self.joint_states.name.index('right_ankle_joint')

            # Calculate approximate foot positions based on joint angles
            # This is highly simplified - real implementation would use forward kinematics
            left_foot_x = 0.0  # Placeholder
            left_foot_y = 0.1  # Approximate left foot position
            right_foot_x = 0.0  # Placeholder
            right_foot_y = -0.1  # Approximate right foot position

            # Support polygon vertices (simplified as rectangle between feet)
            self.support_polygon = [
                Point(x=left_foot_x, y=left_foot_y, z=0.0),
                Point(x=right_foot_x, y=right_foot_y, z=0.0)
            ]

        self.center_of_mass.x = com_x
        self.center_of_mass.y = com_y
        self.center_of_mass.z = com_z

        # Publish center of mass
        self.com_publisher.publish(self.center_of_mass)

    def calculate_zmp(self):
        """Calculate Zero Moment Point (ZMP)"""
        if self.imu_data is None:
            return

        # Simplified ZMP calculation
        # ZMP_x = CoM_x - (h/g) * CoM_acc_x
        # ZMP_y = CoM_y - (h/g) * CoM_acc_y

        # Get linear acceleration from IMU (simplified)
        acc_x = self.imu_data.linear_acceleration.x
        acc_y = self.imu_data.linear_acceleration.y

        # Use CoM height as the effective height
        h = self.center_of_mass.z

        zmp_x = self.center_of_mass.x - (h / self.gravity) * acc_x
        zmp_y = self.center_of_mass.y - (h / self.gravity) * acc_y

        self.zmp.x = zmp_x
        self.zmp.y = zmp_y
        self.zmp.z = 0.0  # ZMP is on ground plane

        return self.zmp

    def is_stable(self):
        """Check if the robot is stable based on ZMP and support polygon"""
        zmp = self.calculate_zmp()

        # Simplified stability check - in real implementation this would use
        # proper polygon inclusion algorithms
        if not self.support_polygon:
            return False

        # For now, check if ZMP is within a simple bounding box of support polygon
        min_x = min(p.x for p in self.support_polygon)
        max_x = max(p.x for p in self.support_polygon)
        min_y = min(p.y for p in self.support_polygon)
        max_y = max(p.y for p in self.support_polygon)

        # Add safety margin
        margin = 0.05  # 5cm safety margin
        return (min_x - margin <= zmp.x <= max_x + margin and
                min_y - margin <= zmp.y <= max_y + margin)

    def balance_control_loop(self):
        """Main balance control loop"""
        if self.joint_states is None or self.imu_data is None:
            return

        # Calculate ZMP
        zmp = self.calculate_zmp()
        self.zmp_publisher.publish(self.zmp)

        # Calculate balance error (distance from ZMP to desired position)
        # For standing, desired ZMP is typically under the feet center
        if self.support_polygon:
            desired_x = sum(p.x for p in self.support_polygon) / len(self.support_polygon)
            desired_y = sum(p.y for p in self.support_polygon) / len(self.support_polygon)
        else:
            desired_x, desired_y = 0.0, 0.0

        balance_error_x = zmp.x - desired_x
        balance_error_y = zmp.y - desired_y

        # Use PID controller to calculate balance correction
        dt = 0.01  # 100 Hz control loop

        # Update integral term
        self.balance_error_integral += balance_error_x * dt

        # Calculate derivative term
        derivative_error = (balance_error_x - self.previous_balance_error) / dt if dt > 0 else 0.0

        # PID control output
        pid_output = (self.balance_pid['kp'] * balance_error_x +
                     self.balance_pid['ki'] * self.balance_error_integral +
                     self.balance_pid['kd'] * derivative_error)

        # Store current error for next iteration
        self.previous_balance_error = balance_error_x

        # Generate balance correction commands
        balance_cmd = Float64MultiArray()

        # Simplified balance correction - in real implementation this would
        # generate specific joint commands to adjust posture
        balance_cmd.data = [
            pid_output * 0.1,  # Hip adjustment
            -pid_output * 0.1, # Opposite hip adjustment for balance
            pid_output * 0.05, # Ankle adjustment
            -pid_output * 0.05 # Opposite ankle adjustment
        ]

        # Publish balance commands if stable, otherwise emergency stop
        if self.is_stable():
            self.balance_cmd_publisher.publish(balance_cmd)

            # Publish stability metrics
            stability_msg = Wrench()
            stability_msg.force.x = balance_error_x
            stability_msg.force.y = balance_error_y
            stability_msg.force.z = 1.0 if self.is_stable() else 0.0  # Stable flag
            stability_msg.torque.x = self.balance_error_integral
            stability_msg.torque.y = derivative_error
            stability_msg.torque.z = abs(balance_error_x) + abs(balance_error_y)  # Instability measure
            self.stability_publisher.publish(stability_msg)
        else:
            # Robot is unstable, publish zero commands to stop movement
            emergency_cmd = Float64MultiArray()
            emergency_cmd.data = [0.0, 0.0, 0.0, 0.0]
            self.balance_cmd_publisher.publish(emergency_cmd)
            self.get_logger().warn('INSTABILITY DETECTED - EMERGENCY BALANCE CORRECTION')

        # Log balance status
        stability_msg = "STABLE" if self.is_stable() else "UNSTABLE"
        self.get_logger().info(f'Balance status: {stability_msg}, ZMP: ({zmp.x:.3f}, {zmp.y:.3f}), Error: ({balance_error_x:.3f}, {balance_error_y:.3f})')

def main(args=None):
    rclpy.init(args=args)
    humanoid_balance_controller = HumanoidBalanceController()

    try:
        rclpy.spin(humanoid_balance_controller)
    except KeyboardInterrupt:
        pass
    finally:
        humanoid_balance_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with Navigation System

### Coordinated Navigation and Balance

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray, Bool
import math

class HumanoidNavigationIntegration(Node):
    def __init__(self):
        super().__init__('humanoid_navigation_integration')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.balance_cmd_publisher = self.create_publisher(Float64MultiArray, 'balance_commands', 10)
        self.joint_cmd_publisher = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        self.safety_status_publisher = self.create_publisher(Bool, 'safety_status', 10)

        # Subscribers
        self.path_subscriber = self.create_subscription(
            Path, 'global_plan', self.path_callback, 10)
        self.odom_subscriber = self.create_subscription(
            PoseStamped, 'current_pose', self.odom_callback, 10)
        self.imu_subscriber = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10)
        self.joint_state_subscriber = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        self.stability_subscriber = self.create_subscription(
            Bool, 'balance_stable', self.stability_callback, 10)

        # Timer for integrated control
        self.control_timer = self.create_timer(0.05, self.integrated_control_loop)

        # Navigation state
        self.current_path = []
        self.current_pose = None
        self.navigation_active = False
        self.current_waypoint_idx = 0

        # Balance state
        self.imu_data = None
        self.joint_states = None
        self.balance_stable = True
        self.emergency_stop = False

        # Walking parameters
        self.step_size = 0.3
        self.max_walk_velocity = 0.5
        self.balance_threshold = 0.2

        self.get_logger().info('Humanoid Navigation Integration initialized')

    def path_callback(self, msg):
        """Receive navigation path"""
        self.current_path = msg.poses
        self.current_waypoint_idx = 0
        self.navigation_active = len(self.current_path) > 0
        if self.navigation_active:
            self.get_logger().info(f'Starting navigation to {len(self.current_path)} waypoints')

    def odom_callback(self, msg):
        """Receive robot pose"""
        self.current_pose = msg.pose

    def imu_callback(self, msg):
        """Receive IMU data"""
        self.imu_data = msg

    def joint_state_callback(self, msg):
        """Receive joint states"""
        self.joint_states = msg

    def stability_callback(self, msg):
        """Receive balance stability status"""
        self.balance_stable = msg.data

    def integrated_control_loop(self):
        """Main integrated control loop combining navigation and balance"""
        # Check safety conditions
        if not self.balance_stable or self.emergency_stop:
            # Emergency stop - zero all commands
            self.emergency_stop_procedure()
            return

        if not self.navigation_active or self.current_pose is None:
            # Not navigating - maintain balance
            self.maintain_balance()
            return

        # Check if path is complete
        if self.current_waypoint_idx >= len(self.current_path):
            self.navigation_complete()
            return

        # Get current target
        target_pose = self.current_path[self.current_waypoint_idx].pose

        # Calculate navigation commands
        nav_cmd = self.calculate_navigation_command(target_pose)

        # Calculate balance commands based on navigation intent
        balance_cmd = self.calculate_balance_command(nav_cmd)

        # Check if reached current waypoint
        distance_to_target = self.calculate_distance_to_pose(self.current_pose, target_pose)
        if distance_to_target < 0.3:  # Waypoint tolerance
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.current_path):
                self.navigation_complete()
                return

        # Publish commands
        self.cmd_vel_publisher.publish(nav_cmd)
        self.balance_cmd_publisher.publish(balance_cmd)

        # Log status
        self.get_logger().info(f'Navigating: v={nav_cmd.linear.x:.2f}, w={nav_cmd.angular.z:.2f}, stable={self.balance_stable}')

    def calculate_navigation_command(self, target_pose):
        """Calculate navigation command toward target pose"""
        cmd = Twist()

        # Calculate direction to target
        dx = target_pose.position.x - self.current_pose.position.x
        dy = target_pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        # Calculate angle to target
        target_angle = math.atan2(dy, dx)

        # Simplified orientation (this would be more complex in practice)
        current_yaw = 0.0  # Placeholder - would need to extract from orientation quaternion
        angle_diff = target_angle - current_yaw

        # Normalize angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Set commands
        cmd.angular.z = max(-1.0, min(1.0, angle_diff * 2.0))  # Turn toward target
        cmd.linear.x = min(self.max_walk_velocity, max(0.1, distance * 0.5))  # Move toward target

        return cmd

    def calculate_balance_command(self, nav_cmd):
        """Calculate balance commands based on navigation intent"""
        balance_cmd = Float64MultiArray()

        # Adjust balance based on movement intent
        # When moving forward, shift weight appropriately
        # When turning, adjust for centripetal forces
        forward_weight_shift = nav_cmd.linear.x * 0.1  # Shift weight forward when moving
        turn_compensation = nav_cmd.angular.z * 0.2    # Compensate for turning

        balance_cmd.data = [
            forward_weight_shift + turn_compensation,  # Left hip adjustment
            forward_weight_shift - turn_compensation,  # Right hip adjustment
            0.0,  # Left ankle
            0.0   # Right ankle
        ]

        return balance_cmd

    def calculate_distance_to_pose(self, pose1, pose2):
        """Calculate 2D distance between two poses"""
        dx = pose2.position.x - pose1.position.x
        dy = pose2.position.y - pose1.position.y
        return math.sqrt(dx*dx + dy*dy)

    def maintain_balance(self):
        """Maintain balance when not navigating"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)

        balance_cmd = Float64MultiArray()
        balance_cmd.data = [0.0, 0.0, 0.0, 0.0]  # Neutral balance
        self.balance_cmd_publisher.publish(balance_cmd)

    def navigation_complete(self):
        """Handle navigation completion"""
        self.navigation_active = False
        self.maintain_balance()
        self.get_logger().info('Navigation completed successfully')

    def emergency_stop_procedure(self):
        """Execute emergency stop"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)

        balance_cmd = Float64MultiArray()
        balance_cmd.data = [0.0, 0.0, 0.0, 0.0]  # Stabilize
        self.balance_cmd_publisher.publish(balance_cmd)

        safety_msg = Bool()
        safety_msg.data = False  # Not safe to continue
        self.safety_status_publisher.publish(safety_msg)

        self.get_logger().error('EMERGENCY STOP ACTIVATED')

def main(args=None):
    rclpy.init(args=args)
    humanoid_navigation_integration = HumanoidNavigationIntegration()

    try:
        rclpy.spin(humanoid_navigation_integration)
    except KeyboardInterrupt:
        pass
    finally:
        humanoid_navigation_integration.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Lab Exercise: Implement a Complete Humanoid Navigation System

### Objective
Create a complete humanoid navigation system that integrates:
1. Path planning and following
2. Joint control for walking
3. Balance maintenance
4. Safety monitoring

### Requirements
1. Implement a humanoid-specific path planner
2. Create walking pattern generators
3. Implement balance control algorithms
4. Add safety features and emergency stops
5. Test the system with simulated sensors

### Solution Structure

The implementation would combine all the components we've covered:
- Navigation stack with humanoid-specific parameters
- Joint controllers for leg and arm movements
- Balance control using IMU and kinematic data
- Integrated safety monitoring

## Summary

This week, you've learned how to implement navigation and control systems for humanoid robots, including:

- **Navigation Architecture**: Understanding the components of a navigation stack
- **Humanoid-Specific Navigation**: Adapting navigation for bipedal locomotion
- **Joint Control Systems**: Managing the multiple degrees of freedom in humanoid robots
- **Balance Control**: Maintaining stability during movement
- **System Integration**: Coordinating navigation, control, and balance systems

These systems form the core of autonomous humanoid robot operation, enabling them to move safely and effectively in human environments. The integration of navigation, control, and balance is essential for creating stable, reliable humanoid robots that can operate in real-world conditions.