---
title: Week 3 - Launch Files & Parameter Management
description: Creating and managing launch files and parameters for complex robot systems
hide_table_of_contents: false
---

# Week 3: Launch Files & Parameter Management

## Learning Objectives

By the end of this week, you will be able to:
- Create launch files for complex robot systems with multiple nodes
- Manage parameters and configurations for scalable robot software
- Organize robot software architecture for maintainability
- Use launch arguments and conditional execution in launch files

## Overview of Launch Files

Launch files in ROS 2 are Python scripts that define how to start multiple nodes with specific configurations. They are essential for managing complex robot systems where multiple nodes need to be started with specific parameters, remappings, and dependencies.

### Why Use Launch Files?

- **Convenience**: Start multiple nodes with a single command
- **Configuration**: Set parameters and configurations for nodes
- **Reusability**: Define common launch patterns that can be reused
- **Flexibility**: Use arguments to customize launch behavior
- **Organization**: Keep related nodes together in logical groups

## Basic Launch File Structure

### Simple Launch File

```python
# launch/simple_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop',
            remappings=[
                ('/turtle1/cmd_vel', '/cmd_vel')
            ]
        )
    ])
```

To run this launch file:
```bash
ros2 launch my_package simple_launch.py
```

## Launch File Components

### 1. LaunchDescription
The main container for all launch actions. It contains a list of actions to execute.

### 2. Node Action
The most common action, used to start ROS 2 nodes with specific configurations.

### 3. Other Launch Actions
- `ExecuteProcess`: Run non-ROS executables
- `RegisterEventHandler`: Handle events during launch
- `TimerAction`: Delay execution of other actions
- `LogInfo`: Print information during launch

## Advanced Node Configuration

### Parameters in Launch Files

```python
# launch/parameterized_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    robot_name_launch_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot1',
        description='Name of the robot'
    )

    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot'
    )

    # Get launch configurations
    robot_name = LaunchConfiguration('robot_name')
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        # Declare launch arguments
        robot_name_launch_arg,
        namespace_launch_arg,

        # Robot controller node with parameters
        Node(
            package='robot_control',
            executable='robot_controller',
            name='robot_controller',
            namespace=namespace,
            parameters=[
                # Inline parameter dictionary
                {
                    'robot_name': robot_name,
                    'max_velocity': 1.0,
                    'min_velocity': 0.1,
                    'acceleration_limit': 0.5,
                    'deceleration_limit': 1.0,
                },
                # Load parameters from YAML file
                PathJoinSubstitution([
                    FindPackageShare('robot_control'),
                    'config',
                    'robot_params.yaml'
                ])
            ],
            remappings=[
                ('/cmd_vel', [namespace, '/cmd_vel']),
                ('/odom', [namespace, '/odom']),
            ],
            # Additional configuration options
            respawn=True,  # Restart if the node dies
            respawn_delay=5.0,  # Wait 5 seconds before restarting
            output='screen',  # Output to screen
            arguments=['--ros-args', '--log-level', 'INFO']  # Additional arguments
        )
    ])
```

### YAML Parameter Files

Create `config/robot_params.yaml`:

```yaml
/**:  # Apply to all nodes
  ros__parameters:
    robot_name: "default_robot"
    max_velocity: 1.0
    min_velocity: 0.1
    acceleration_limit: 0.5
    deceleration_limit: 1.0
    safety:
      emergency_stop_distance: 0.5
      max_joint_velocity: 2.0
    navigation:
      goal_tolerance: 0.1
      obstacle_threshold: 1.0
      max_planning_time: 5.0
```

### Launch Arguments and Substitutions

```python
# launch/advanced_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare launch arguments with descriptions
    sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    config_file_launch_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('robot_control'),
            'config',
            'default_config.yaml'
        ]),
        description='Path to configuration file'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        sim_time_launch_arg,
        config_file_launch_arg,

        # Sensor node with conditional parameters
        Node(
            package='sensor_package',
            executable='lidar_driver',
            name='lidar_driver',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time},
                {'sensor_model': 'VLP-16'},
                {'scan_topic': 'laser_scan'},
                {'frame_id': 'lidar_link'},
                {'range_min': 0.1},
                {'range_max': 100.0},
                {'scan_frequency': 10.0}
            ],
            output='screen'
        ),

        # Robot state publisher with robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description':
                    PathJoinSubstitution([
                        FindPackageShare('robot_description'),
                        'urdf',
                        'robot.urdf'
                    ])
                }
            ],
            remappings=[
                ('/joint_states', 'joint_states')
            ]
        )
    ])
```

## Conditional Launch Actions

### Using IfCondition and UnlessCondition

```python
# launch/conditional_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    use_gpu_launch_arg = DeclareLaunchArgument(
        'use_gpu',
        default_value='true',
        description='Use GPU acceleration'
    )

    debug_launch_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode'
    )

    # Get launch configurations
    use_gpu = LaunchConfiguration('use_gpu')
    debug = LaunchConfiguration('debug')

    return LaunchDescription([
        use_gpu_launch_arg,
        debug_launch_arg,

        # Set environment variable if using GPU
        SetEnvironmentVariable(
            name='CUDA_VISIBLE_DEVICES',
            value='0',
            condition=IfCondition(use_gpu)
        ),

        # GPU-accelerated perception node
        Node(
            package='perception_package',
            executable='gpu_perception',
            name='gpu_perception',
            condition=IfCondition(use_gpu),
            parameters=[
                {'use_gpu': True},
                {'gpu_device': 0}
            ]
        ),

        # CPU-based perception node (alternative)
        Node(
            package='perception_package',
            executable='cpu_perception',
            name='cpu_perception',
            condition=UnlessCondition(use_gpu),
            parameters=[
                {'use_gpu': False}
            ]
        ),

        # Debug tools if debug mode is enabled
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='debug_gui',
            condition=IfCondition(debug),
            output='screen'
        )
    ])
```

## Launch File Composition

### Including Other Launch Files

```python
# launch/main_launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    robot_name_launch_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='default_robot',
        description='Name of the robot to launch'
    )

    # Get launch configuration
    robot_name = LaunchConfiguration('robot_name')

    return LaunchDescription([
        robot_name_launch_arg,

        # Include robot-specific launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('robot_bringup'),
                '/launch/',
                robot_name,
                '_bringup.launch.py'
            ])
        ),

        # Include navigation launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('nav2_bringup'),
                '/launch/navigation_launch.py'
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'params/nav2_params.yaml'
                ])
            }.items()
        ),

        # Include perception launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('perception_bringup'),
                '/launch/perception.launch.py'
            ])
        )
    ])
```

## Parameter Management Strategies

### 1. Hierarchical Parameter Organization

```
robot_system/
├── config/
│   ├── base_params.yaml          # Base robot parameters
│   ├── sensors/
│   │   ├── lidar_params.yaml     # LiDAR-specific parameters
│   │   ├── camera_params.yaml    # Camera-specific parameters
│   │   └── imu_params.yaml       # IMU-specific parameters
│   ├── controllers/
│   │   ├── joint_params.yaml     # Joint controller parameters
│   │   └── trajectory_params.yaml # Trajectory controller parameters
│   └── navigation/
│       ├── costmap_params.yaml   # Costmap parameters
│       └── planner_params.yaml   # Path planner parameters
└── launch/
    ├── robot_bringup.launch.py   # Main launch file
    └── sensor_launch.py          # Sensor-specific launch
```

### 2. Parameter Validation

```python
# launch/validated_launch.py
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

def validate_parameters(context):
    """Validate parameters before launching nodes"""
    # Get launch configurations
    robot_name = LaunchConfiguration('robot_name').perform(context)
    max_velocity = float(LaunchConfiguration('max_velocity').perform(context))

    # Validate parameters
    if max_velocity <= 0:
        raise ValueError(f"max_velocity must be positive, got {max_velocity}")

    if len(robot_name) == 0:
        raise ValueError("robot_name cannot be empty")

    print(f"Validated parameters: robot_name={robot_name}, max_velocity={max_velocity}")

def generate_launch_description():
    return LaunchDescription([
        # Validation function
        OpaqueFunction(function=validate_parameters),

        # Nodes with validated parameters
        Node(
            package='robot_control',
            executable='velocity_controller',
            name='velocity_controller',
            parameters=[
                {'max_velocity': LaunchConfiguration('max_velocity')},
                {'robot_name': LaunchConfiguration('robot_name')}
            ]
        )
    ])
```

## Complex Launch Example: Multi-Robot System

```python
# launch/multi_robot.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetRemap
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # Declare launch arguments
    num_robots_launch_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='2',
        description='Number of robots to launch'
    )

    use_rviz_launch_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    # Get launch configurations
    num_robots = LaunchConfiguration('num_robots')
    use_rviz = LaunchConfiguration('use_rviz')

    # Create robot groups
    robot_groups = []

    for i in range(int(LaunchConfiguration('num_robots').perform({}))):
        robot_name = f'robot_{i}'

        # Group all nodes for this robot under its namespace
        robot_group = GroupAction(
            actions=[
                PushRosNamespace(robot_name),  # Set namespace for all nodes in this group

                # Robot controller
                Node(
                    package='robot_control',
                    executable='robot_controller',
                    name='controller',
                    parameters=[
                        {'robot_name': robot_name},
                        {'robot_id': i}
                    ]
                ),

                # Robot sensor
                Node(
                    package='sensor_package',
                    executable='sensor_driver',
                    name='sensor',
                    parameters=[
                        {'robot_name': robot_name},
                        {'sensor_topic': f'/{robot_name}/scan'}
                    ]
                ),

                # Robot localization
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter',
                    parameters=[
                        PathJoinSubstitution([
                            FindPackageShare('robot_localization'),
                            'config',
                            f'robot_{i}_ekf.yaml'
                        ])
                    ]
                )
            ]
        )
        robot_groups.append(robot_group)

    return LaunchDescription([
        num_robots_launch_arg,
        use_rviz_launch_arg,

        # Launch all robot groups
        *robot_groups,

        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('multi_robot_bringup'),
                'rviz',
                'multi_robot.rviz'
            ])],
            condition=IfCondition(use_rviz)
        )
    ])
```

## Launch File Best Practices

### 1. Use Descriptive Names
```python
# Good
robot_name_launch_arg = DeclareLaunchArgument(
    'robot_name',
    default_value='turtlebot4',
    description='Name of the robot to launch'
)

# Avoid
arg1 = DeclareLaunchArgument('arg1', default_value='default')
```

### 2. Organize Related Nodes
```python
# Group related functionality together
sensor_group = GroupAction(
    actions=[
        Node(package='sensor_pkg', executable='lidar_driver', name='lidar'),
        Node(package='sensor_pkg', executable='camera_driver', name='camera'),
        Node(package='perception_pkg', executable='object_detector', name='detector')
    ]
)
```

### 3. Use Default Values and Validation
```python
# Always provide meaningful defaults
DeclareLaunchArgument(
    'update_rate',
    default_value='10.0',
    description='Update rate in Hz (must be positive)'
)
```

### 4. Modular Design
```python
# Separate concerns into different launch files
# sensors.launch.py
# controllers.launch.py
# navigation.launch.py
# main.launch.py (includes others)
```

## Parameter Management Best Practices

### 1. YAML Configuration Files
```yaml
# config/navigation_params.yaml
amcl:
  ros__parameters:
    use_sim_time: false
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
```

### 2. Parameter Hierarchies
```python
# Organize parameters by subsystem
parameters = [
    {
        'robot': {
            'name': 'turtlebot4',
            'model': 'standard',
            'id': 1
        },
        'sensors': {
            'lidar': {
                'enabled': True,
                'topic': '/scan',
                'frame_id': 'lidar_link'
            },
            'camera': {
                'enabled': True,
                'topic': '/camera/image_raw',
                'frame_id': 'camera_link'
            }
        },
        'control': {
            'max_velocity': 1.0,
            'acceleration_limit': 0.5,
            'deceleration_limit': 1.0
        }
    }
]
```

## Launch File Debugging

### 1. Enable Launch File Debugging
```bash
# Print launch file details
ros2 launch my_package my_launch.py --show-args

# Verbose output
ros2 launch my_package my_launch.py --log-level debug
```

### 2. Common Issues and Solutions

**Issue**: Nodes not starting
```python
# Check if package and executable names are correct
Node(
    package='correct_package_name',  # Verify this exists
    executable='correct_executable_name',  # Verify this exists
    name='unique_name'
)
```

**Issue**: Parameters not loading
```python
# Check file paths and permissions
parameters=[
    # Use PathJoinSubstitution for reliable paths
    PathJoinSubstitution([
        FindPackageShare('my_package'),
        'config',
        'params.yaml'
    ])
]
```

**Issue**: Namespace conflicts
```python
# Use proper namespace management
Node(
    package='pkg',
    executable='exec',
    namespace='robot1',  # Use unique namespaces
    name='node_name'
)
```

## Lab Exercise: Create a Complete Robot Launch System

### Objective
Create a comprehensive launch system for a humanoid robot with:
1. Multiple launch files for different subsystems
2. Parameter configuration files
3. Launch arguments for customization
4. Conditional node launching

### Requirements
1. Create a main launch file that includes subsystem launches
2. Create separate launch files for sensors, controllers, and perception
3. Create YAML parameter files for each subsystem
4. Implement conditional launching based on arguments
5. Add parameter validation

### Solution Structure

```python
# launch/humanoid_robot.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Robot configuration arguments
    robot_model_launch_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='nao_v6',
        description='Robot model to launch'
    )

    use_gpu_launch_arg = DeclareLaunchArgument(
        'use_gpu',
        default_value='true',
        description='Enable GPU acceleration'
    )

    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Get launch configurations
    robot_model = LaunchConfiguration('robot_model')
    use_gpu = LaunchConfiguration('use_gpu')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        robot_model_launch_arg,
        use_gpu_launch_arg,
        use_sim_time_launch_arg,

        # Include sensor launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('humanoid_bringup'),
                '/launch/sensors.launch.py'
            ]),
            launch_arguments={
                'robot_model': robot_model,
                'use_sim_time': use_sim_time
            }.items()
        ),

        # Include controller launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('humanoid_bringup'),
                '/launch/controllers.launch.py'
            ]),
            launch_arguments={
                'robot_model': robot_model,
                'use_gpu': use_gpu
            }.items()
        ),

        # Include perception launch (conditional on GPU)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('humanoid_bringup'),
                '/launch/perception.launch.py'
            ]),
            launch_arguments={
                'robot_model': robot_model,
                'use_gpu': use_gpu,
                'use_sim_time': use_sim_time
            }.items()
        ),

        # Include navigation launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('humanoid_bringup'),
                '/launch/navigation.launch.py'
            ]),
            launch_arguments={
                'robot_model': robot_model,
                'use_sim_time': use_sim_time
            }.items()
        )
    ])
```

## Summary

This week, you've learned how to:
- Create complex launch files for multi-node robot systems
- Manage parameters using both inline definitions and YAML files
- Use launch arguments to customize launch behavior
- Implement conditional launching based on arguments
- Organize robot software architecture for scalability and maintainability

Launch files and parameter management are crucial for creating professional robot systems that can be easily configured, deployed, and maintained. These skills will be essential as we move forward with more complex robot applications in the coming weeks.