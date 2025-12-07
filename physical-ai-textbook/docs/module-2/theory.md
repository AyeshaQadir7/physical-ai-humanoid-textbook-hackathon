# Theory: Robot Operating Systems and ROS 2 Fundamentals

## Introduction to ROS 2 Architecture

### What is ROS 2?

ROS 2 (Robot Operating System 2) is not an operating system but a flexible framework for writing robot software. It provides libraries and tools to help software developers create robot applications. ROS 2 is the next generation of ROS, designed to be production-ready with improved security, real-time support, and multi-robot systems.

ROS 2 uses a distributed architecture where nodes can run on different machines and communicate over a network. The system uses the Data Distribution Service (DDS) as the underlying communication middleware, providing reliable message delivery and discovery of nodes.

### Key Improvements in ROS 2

1. **Security**: Built-in security features for secure robot operations
2. **Real-time Support**: Better support for real-time systems
3. **Multi-robot Systems**: Improved support for coordinating multiple robots
4. **Production Readiness**: More stable and reliable for commercial applications
5. **Quality of Service (QoS)**: Configurable communication patterns for different needs

## Core ROS 2 Concepts

### Nodes

A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are typically organized to perform specific tasks within a robot system. For example, one node might handle sensor data processing, while another handles motor control.

#### Node Characteristics:

- Each node runs in its own process
- Nodes can be written in different programming languages (C++, Python, etc.)
- Nodes communicate with each other through topics, services, and actions
- Nodes can be started and stopped independently

#### Node Lifecycle:

- **Unconfigured**: Node exists but not yet configured
- **Inactive**: Node is configured but not active
- **Active**: Node is running and processing data
- **Finalized**: Node is shutting down

### Topics

Topics are named buses over which nodes exchange messages. Each topic has a specific message type that defines the structure of the data being transmitted. Nodes can publish data to a topic or subscribe to receive data from a topic.

#### Topic Communication:

- **Publisher**: Sends messages to a topic
- **Subscriber**: Receives messages from a topic
- **Unidirectional**: Data flows from publisher to subscriber
- **Many-to-many**: Multiple publishers can send to a topic, multiple subscribers can receive from it

#### Quality of Service (QoS) Settings:

- **Reliability**: Best effort vs. reliable delivery
- **Durability**: Volatile vs. transient local
- **History**: Keep last N messages vs. keep all messages
- **Depth**: Number of messages to queue

### Services

Services provide a request/reply communication pattern. A service client sends a request to a service server, which processes the request and returns a response. This is useful for tasks that require immediate responses or specific actions.

#### Service Characteristics:

- **Synchronous**: Client waits for response from server
- **Request/Response**: Client sends request, server returns response
- **One-to-one**: One client talks to one server at a time
- **Blocking**: Client blocks until response is received

### Actions

Actions are similar to services but designed for long-running tasks. They allow clients to send goals to action servers, receive feedback during execution, and get results when the goal is completed.

#### Action Components:

- **Goal**: Request sent to the action server
- **Feedback**: Periodic updates during goal execution
- **Result**: Final outcome when goal is completed or canceled

#### Action States:

- **Pending**: Goal accepted but not yet started
- **Active**: Goal is currently being executed
- **Succeeded**: Goal completed successfully
- **Aborted**: Goal execution failed
- **Canceled**: Goal was canceled by client

### Packages

Packages are the basic building blocks of ROS 2. They contain libraries, executables, configuration files, and other resources needed to implement specific functionality.

#### Package Structure:

```
package_name/
├── CMakeLists.txt        # Build configuration (for C++)
├── package.xml           # Package metadata
├── src/                  # Source code
├── include/              # Header files (for C++)
├── scripts/              # Executable scripts
├── launch/               # Launch files
├── config/               # Configuration files
├── test/                 # Test files
└── README.md             # Package documentation
```

## ROS 2 Communication Patterns

### Publisher-Subscriber Pattern

The publisher-subscriber pattern is the most common communication pattern in ROS 2. It enables decoupled, asynchronous communication between nodes.

```python
# Publisher example
publisher = node.create_publisher(MessageType, 'topic_name', qos_profile)
publisher.publish(message)

# Subscriber example
subscription = node.create_subscription(
    MessageType,
    'topic_name',
    callback_function,
    qos_profile
)
```

### Client-Server Pattern

The client-server pattern is used for synchronous request-response communication.

```python
# Service server example
service = node.create_service(RequestType, 'service_name', callback_function)

# Service client example
client = node.create_client(RequestType, 'service_name')
future = client.call_async(request)
```

### Action Client-Server Pattern

The action pattern is used for long-running tasks with feedback.

```python
# Action server example
action_server = ActionServer(
    node,
    ActionType,
    'action_name',
    execute_callback
)

# Action client example
action_client = ActionClient(node, ActionType, 'action_name')
goal_future = action_client.send_goal_async(goal)
```

## Launch Files and Parameter Management

### Launch Files

Launch files allow you to start multiple nodes with a single command. They're written in Python using the launch system.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='package_name',
            executable='node_name',
            name='node_name',
            parameters=[
                {'param_name': 'param_value'}
            ],
            remappings=[
                ('original_topic', 'new_topic')
            ]
        )
    ])
```

### Parameter Management

Parameters allow you to configure nodes without recompiling. They can be set at runtime and are stored in parameter servers.

```python
# Declaring parameters in a node
self.declare_parameter('param_name', default_value)

# Getting parameter values
param_value = self.get_parameter('param_name').value

# Setting parameters
self.set_parameters([Parameter('param_name', Parameter.Type.STRING, 'new_value')])
```

## TF (Transforms) and Coordinate Systems

### What is TF?

TF (Transforms) is a package in ROS that keeps track of multiple coordinate frames over time. It allows you to transform points, vectors, etc. between coordinate frames at any time.

### Coordinate Frames

Coordinate frames define the position and orientation of objects in space. Common frames in robotics:

- **base_link**: The main body of the robot
- **odom**: Odometry frame for position tracking
- **map**: Global map frame
- **camera_frame**: Camera sensor frame
- **laser_frame**: LIDAR sensor frame

### TF Tree

The TF tree represents the relationships between all coordinate frames in a robot system. Each frame has a parent, and transformations are computed from the root to any frame.

```python
# Publishing transforms
from tf2_ros import TransformBroadcaster
tf_broadcaster = TransformBroadcaster(node)
t = TransformStamped()
t.header.stamp = node.get_clock().now().to_msg()
t.header.frame_id = 'parent_frame'
t.child_frame_id = 'child_frame'
# Set transform values...
tf_broadcaster.sendTransform(t)

# Looking up transforms
from tf2_ros import TransformListener
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)
trans = tf_buffer.lookup_transform('target_frame', 'source_frame', time)
```

## ROS 2 Command Line Tools

### Essential ROS 2 Commands

- `ros2 node list`: List all active nodes
- `ros2 topic list`: List all active topics
- `ros2 service list`: List all available services
- `ros2 action list`: List all available actions
- `ros2 param list <node_name>`: List parameters for a specific node

### Topic Commands

- `ros2 topic echo <topic_name>`: Print messages from a topic
- `ros2 topic info <topic_name>`: Show information about a topic
- `ros2 topic pub <topic_name> <msg_type> <args>`: Publish a message to a topic

### Service Commands

- `ros2 service call <service_name> <service_type> <args>`: Call a service
- `ros2 service info <service_name>`: Show information about a service

### Action Commands

- `ros2 action send_goal <action_name> <action_type> <goal>`: Send an action goal
- `ros2 action list`: List all available actions

## Best Practices for ROS 2 Development

### Naming Conventions

- Use lowercase with underscores for node names: `sensor_driver`
- Use lowercase with underscores for topic names: `laser_scan`
- Use lowercase with underscores for service names: `get_map`
- Use descriptive names that clearly indicate purpose

### Error Handling

- Always check for successful initialization
- Handle exceptions in callbacks gracefully
- Use appropriate logging levels (DEBUG, INFO, WARN, ERROR)
- Implement proper cleanup in node destruction

### Performance Considerations

- Use appropriate QoS settings for your use case
- Avoid publishing unnecessary data
- Use message filters for synchronization when needed
- Consider the computational cost of transforms

## Summary

This week has provided a deep dive into ROS 2 fundamentals, covering the core concepts that form the backbone of modern robotics software development. You've learned about the different communication patterns (topics, services, actions), how to organize your code into packages, and how to manage configuration through launch files and parameters.

Understanding these concepts is crucial for all subsequent weeks, as ROS 2 serves as the communication backbone for all robot systems. The TF system provides the spatial reasoning capabilities needed for navigation and manipulation, while the various communication patterns allow for flexible and robust robot architectures.

## Key Terms

- **Node**: An executable process that uses ROS 2 to communicate
- **Topic**: Named bus for message exchange between nodes
- **Service**: Synchronous request/reply communication pattern
- **Action**: Asynchronous communication pattern for long-running tasks
- **Package**: Basic building block containing libraries and executables
- **TF**: Transform library for managing coordinate frames
- **QoS**: Quality of Service settings for communication patterns
- **Launch File**: Configuration file to start multiple nodes
