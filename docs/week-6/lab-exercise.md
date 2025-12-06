# Week 6 Lab Exercise: Multi-Robot Systems & Coordination

## Overview

In this lab exercise, you will implement multi-robot communication architectures, develop coordination algorithms for collaborative tasks, and implement distributed algorithms for multi-robot systems. You will work with multiple simulated robots, implement consensus algorithms, and create coordinated behaviors. This builds upon your Week 1-5 foundations to create coordinated multi-robot behaviors and systems.

## Prerequisites

- Completion of Week 1-5 lab exercises
- Working ROS 2 Humble Hawksbill installation
- Working Gazebo simulation environment with multiple robots
- Understanding of ROS 2 communication patterns
- Experience with single robot control systems
- Basic knowledge of networking concepts

## Learning Objectives

By completing this lab, you will:
- Implement multi-robot communication architectures using ROS 2
- Develop coordination algorithms for collaborative tasks
- Implement distributed consensus and decision-making algorithms
- Create formation control and swarm robotics systems
- Design task allocation and scheduling systems for multi-robot teams
- Evaluate multi-robot system performance and scalability

## Exercise 1: Multi-Robot Communication Setup (1.5 hours)

### Task 1.1: Multiple Robot Simulation Environment
1. Create a Gazebo world with multiple robots (at least 3 TurtleBots or similar differential drive robots):
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --dependencies rclcpp rclpy std_msgs geometry_msgs nav_msgs tf2_msgs gazebo_ros_pkgs --build-type ament_cmake multi_robot_lab
   cd multi_robot_lab
   ```

2. Create a launch file `launch/multi_robot_world.launch.py`:
   ```python
   from launch import LaunchDescription
   from launch.actions import IncludeLaunchDescription
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch.substitutions import PathJoinSubstitution
   from launch_ros.actions import Node
   from launch_ros.substitutions import FindPackageShare
   import os

   def generate_launch_description():
       # Launch Gazebo
       gazebo = IncludeLaunchDescription(
           PythonLaunchDescriptionSource([
               PathJoinSubstitution([
                   FindPackageShare('gazebo_ros'),
                   'launch',
                   'gazebo.launch.py'
               ])
           ]),
           launch_arguments={
               'world': PathJoinSubstitution([
                   FindPackageShare('multi_robot_lab'),
                   'worlds',
                   'multi_robot.world'
               ])
           }.items()
       )

       # Spawn multiple robots
       robots = []
       robot_names = ['robot1', 'robot2', 'robot3']
       for i, name in enumerate(robot_names):
           spawn_robot = Node(
               package='gazebo_ros',
               executable='spawn_entity.py',
               arguments=[
                   '-entity', name,
                   '-x', str(i * 2.0),  # Space robots apart
                   '-y', '0.0',
                   '-z', '0.0',
                   '-file', os.path.join(os.getenv('HOME'), 'ros2_ws/src/multi_robot_lab/models', f'{name}_model.sdf')
               ],
               output='screen'
           )
           robots.append(spawn_robot)

       return LaunchDescription([
           gazebo
       ] + robots)
   ```

3. Create a world file `worlds/multi_robot.world` with appropriate environment for multi-robot operation.

### Task 1.2: Namespaced Communication
1. Create a ROS 2 node that manages communication between robots using namespaces:
   ```cpp
   // multi_robot_manager.cpp
   #include <rclcpp/rclcpp.hpp>
   #include <geometry_msgs/msg/twist.hpp>
   #include <std_msgs/msg/string.hpp>
   #include <tf2_ros/transform_listener.h>

   class MultiRobotManager : public rclcpp::Node
   {
   public:
       MultiRobotManager() : Node("multi_robot_manager")
       {
           // Initialize robot names
           robot_names_ = {"robot1", "robot2", "robot3"};

           // Create publishers for each robot
           for (const auto& name : robot_names_) {
               auto pub = this->create_publisher<geometry_msgs::msg::Twist>(
                   name + "/cmd_vel", 10);
               cmd_pubs_[name] = pub;
           }

           // Create subscribers for each robot's position
           for (const auto& name : robot_names_) {
               auto sub = this->create_subscription<geometry_msgs::msg::TransformStamped>(
                   name + "/odom", 10,
                   [this, name](const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
                       robot_positions_[name] = msg->transform.translation;
                   });
           }

           // Create communication topic for robot coordination
           comm_pub_ = this->create_publisher<std_msgs::msg::String>(
               "robot_communication", 10);

           comm_sub_ = this->create_subscription<std_msgs::msg::String>(
               "robot_communication", 10,
               std::bind(&MultiRobotManager::comm_callback, this, std::placeholders::_1));

           // Timer for coordination tasks
           coord_timer_ = this->create_wall_timer(
               std::chrono::milliseconds(100),
               std::bind(&MultiRobotManager::coordination_loop, this));
       }

   private:
       void comm_callback(const std_msgs::msg::String::SharedPtr msg)
       {
           RCLCPP_INFO(this->get_logger(), "Received message: %s", msg->data.c_str());
       }

       void coordination_loop()
       {
           // Implement coordination logic here
           // For example, calculate distances between robots
           for (size_t i = 0; i < robot_names_.size(); ++i) {
               for (size_t j = i + 1; j < robot_names_.size(); ++j) {
                   const auto& pos1 = robot_positions_[robot_names_[i]];
                   const auto& pos2 = robot_positions_[robot_names_[j]];

                   double distance = sqrt(
                       pow(pos1.x - pos2.x, 2) +
                       pow(pos1.y - pos2.y, 2) +
                       pow(pos1.z - pos2.z, 2)
                   );

                   RCLCPP_INFO(this->get_logger(),
                       "Distance between %s and %s: %.2f",
                       robot_names_[i].c_str(), robot_names_[j].c_str(), distance);
               }
           }
       }

       std::vector<std::string> robot_names_;
       std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> cmd_pubs_;
       std::map<std::string, geometry_msgs::msg::Vector3> robot_positions_;

       rclcpp::Publisher<std_msgs::msg::String>::SharedPtr comm_pub_;
       rclcpp::Subscription<std_msgs::msg::String>::SharedPtr comm_sub_;
       rclcpp::TimerBase::SharedPtr coord_timer_;
   };

   int main(int argc, char * argv[])
   {
       rclcpp::init(argc, argv);
       rclcpp::spin(std::make_shared<MultiRobotManager>());
       rclcpp::shutdown();
       return 0;
   }
   ```

2. Build and test the multi-robot communication system.

## Exercise 2: Consensus Algorithm Implementation (2 hours)

### Task 2.1: Average Consensus
1. Implement a distributed average consensus algorithm:
   ```cpp
   // consensus_node.cpp
   #include <rclcpp/rclcpp.hpp>
   #include <std_msgs/msg/float64_multi_array.hpp>
   #include <vector>

   class ConsensusNode : public rclcpp::Node
   {
   public:
       ConsensusNode() : Node("consensus_node")
       {
           robot_id_ = this->declare_parameter("robot_id", 0);
           initial_value_ = this->declare_parameter("initial_value", 0.0);

           // Define neighbors (for this example, assume ring topology)
           neighbors_ = this->declare_parameter("neighbors", std::vector<int>{});

           current_value_ = initial_value_;
           consensus_value_ = initial_value_;

           // Create publisher and subscriber for consensus
           consensus_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
               "consensus_values", 10);

           consensus_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
               "consensus_values", 10,
               std::bind(&ConsensusNode::consensus_callback, this, std::placeholders::_1));

           // Timer for consensus update
           consensus_timer_ = this->create_wall_timer(
               std::chrono::milliseconds(100),
               std::bind(&ConsensusNode::consensus_update, this));
       }

   private:
       void consensus_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
       {
           // Store values from neighbors
           if (msg->data.size() >= 2) {  // First value is robot_id, second is value
               int sender_id = static_cast<int>(msg->data[0]);
               double sender_value = msg->data[1];

               neighbor_values_[sender_id] = sender_value;
           }
       }

       void consensus_update()
       {
           // Update consensus value using weighted average
           double sum = current_value_;
           int count = 1;

           for (int neighbor_id : neighbors_) {
               if (neighbor_values_.find(neighbor_id) != neighbor_values_.end()) {
                   sum += neighbor_values_[neighbor_id];
                   count++;
               }
           }

           consensus_value_ = sum / count;

           // Publish current value to neighbors
           auto msg = std_msgs::msg::Float64MultiArray();
           msg.data = {static_cast<double>(robot_id_), consensus_value_};
           consensus_pub_->publish(msg);

           RCLCPP_INFO(this->get_logger(),
               "Robot %d: Current value = %.2f, Consensus value = %.2f",
               robot_id_, current_value_, consensus_value_);
       }

       int robot_id_;
       double initial_value_, current_value_, consensus_value_;
       std::vector<int> neighbors_;
       std::map<int, double> neighbor_values_;

       rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr consensus_pub_;
       rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr consensus_sub_;
       rclcpp::TimerBase::SharedPtr consensus_timer_;
   };

   int main(int argc, char * argv[])
   {
       rclcpp::init(argc, argv);
       rclcpp::spin(std::make_shared<ConsensusNode>());
       rclcpp::shutdown();
       return 0;
   }
   ```

2. Create a launch file to start multiple consensus nodes with different initial values.

3. Test the consensus algorithm and observe convergence to the average of initial values.

### Task 2.2: Consensus Performance Analysis
1. Implement a node to measure convergence time and accuracy of the consensus algorithm.
2. Test with different network topologies (ring, star, fully connected).
3. Analyze the effect of communication delays on consensus performance.

## Exercise 3: Formation Control (2.5 hours)

### Task 3.1: Leader-Follower Formation
1. Implement a leader-follower formation control system:
   ```cpp
   // formation_controller.cpp
   #include <rclcpp/rclcpp.hpp>
   #include <geometry_msgs/msg/twist.hpp>
   #include <geometry_msgs/msg/pose_stamped.hpp>
   #include <tf2/LinearMath/Quaternion.h>
   #include <tf2/LinearMath/Matrix3x3.h>
   #include <cmath>

   class FormationController : public rclcpp::Node
   {
   public:
       FormationController() : Node("formation_controller")
       {
           robot_id_ = this->declare_parameter("robot_id", 0);
           is_leader_ = this->declare_parameter("is_leader", false);
           follower_offset_x_ = this->declare_parameter("offset_x", 0.0);
           follower_offset_y_ = this->declare_parameter("offset_y", 0.0);

           // Create publisher for velocity commands
           cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

           // Create subscriber for leader position (if follower)
           if (!is_leader_) {
               leader_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                   "leader/pose", 10,
                   std::bind(&FormationController::leader_pose_callback, this, std::placeholders::_1));
           }

           // Create publisher for own pose (if leader)
           if (is_leader_) {
               pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
           }

           // Timer for control loop
           control_timer_ = this->create_wall_timer(
               std::chrono::milliseconds(50),
               std::bind(&FormationController::control_loop, this));
       }

   private:
       void leader_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
       {
           leader_pose_ = msg->pose;
       }

       void control_loop()
       {
           if (is_leader_) {
               // Leader moves in a predefined pattern (e.g., circle)
               double t = this->now().seconds();
               double radius = 2.0;
               double angular_velocity = 0.5;

               double target_x = radius * cos(angular_velocity * t);
               double target_y = radius * sin(angular_velocity * t);
               double target_theta = atan2(angular_velocity * cos(angular_velocity * t),
                                          -angular_velocity * sin(angular_velocity * t));

               // Publish leader pose
               auto pose_msg = geometry_msgs::msg::PoseStamped();
               pose_msg.header.stamp = this->now();
               pose_msg.pose.position.x = target_x;
               pose_msg.pose.position.y = target_y;
               tf2::Quaternion q;
               q.setRPY(0, 0, target_theta);
               pose_msg.pose.orientation.x = q.x();
               pose_msg.pose.orientation.y = q.y();
               pose_msg.pose.orientation.z = q.z();
               pose_msg.pose.orientation.w = q.w();
               pose_pub_->publish(pose_msg);

               // Simple leader control (move to target)
               auto cmd_msg = geometry_msgs::msg::Twist();
               cmd_msg.linear.x = 0.5;  // Simple forward motion
               cmd_msg.angular.z = angular_velocity;
               cmd_pub_->publish(cmd_msg);
           } else {
               // Follower tries to maintain position relative to leader
               if (leader_pose_.position.x != 0 || leader_pose_.position.y != 0) {
                   // Calculate desired position based on leader + offset
                   double desired_x = leader_pose_.position.x + follower_offset_x_;
                   double desired_y = leader_pose_.position.y + follower_offset_y_;

                   // Simple proportional controller
                   double error_x = desired_x - current_pose_.position.x;
                   double error_y = desired_y - current_pose_.position.y;

                   // Calculate distance and angle to target
                   double distance = sqrt(error_x * error_x + error_y * error_y);
                   double angle_to_target = atan2(error_y, error_x);

                   // Get current robot orientation
                   tf2::Quaternion q(
                       current_pose_.orientation.x,
                       current_pose_.orientation.y,
                       current_pose_.orientation.z,
                       current_pose_.orientation.w
                   );
                   tf2::Matrix3x3 m(q);
                   double roll, pitch, yaw;
                   m.getRPY(roll, pitch, yaw);

                   // Calculate angular error
                   double angle_error = angle_to_target - yaw;
                   // Normalize angle to [-pi, pi]
                   while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
                   while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

                   // Create velocity command
                   auto cmd_msg = geometry_msgs::msg::Twist();
                   cmd_msg.linear.x = std::min(0.5 * distance, 1.0);  // Limit speed
                   cmd_msg.angular.z = 2.0 * angle_error;  // Proportional control

                   cmd_pub_->publish(cmd_msg);
               }
           }
       }

       int robot_id_;
       bool is_leader_;
       double follower_offset_x_, follower_offset_y_;
       geometry_msgs::msg::Pose leader_pose_, current_pose_;

       rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
       rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr leader_pose_sub_;
       rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
       rclcpp::TimerBase::SharedPtr control_timer_;
   };

   int main(int argc, char * argv[])
   {
       rclcpp::init(argc, argv);
       rclcpp::spin(std::make_shared<FormationController>());
       rclcpp::shutdown();
       return 0;
   }
   ```

2. Create a launch file to start one leader and multiple followers in formation.

3. Test different formation patterns (line, circle, triangle).

### Task 3.2: Behavior-Based Formation
1. Implement a behavior-based formation using potential fields or flocking rules.
2. Compare the performance of leader-follower vs. behavior-based formations.

## Exercise 4: Task Allocation (2 hours)

### Task 4.1: Market-Based Task Allocation
1. Implement a simple auction-based task allocation system:
   ```cpp
   // task_allocator.cpp
   #include <rclcpp/rclcpp.hpp>
   #include <std_msgs/msg/string.hpp>
   #include <std_msgs/msg/int32.hpp>
   #include <vector>
   #include <map>

   struct Task {
       int id;
       double x, y;  // Task location
       int priority;
       bool assigned;
   };

   class TaskAllocator : public rclcpp::Node
   {
   public:
       TaskAllocator() : Node("task_allocator")
       {
           robot_id_ = this->declare_parameter("robot_id", 0);
           is_auctioneer_ = this->declare_parameter("is_auctioneer", false);

           if (is_auctioneer_) {
               // Auctioneer manages task allocation
               auctioneer_pub_ = this->create_publisher<std_msgs::msg::String>(
                   "task_assignment", 10);
               task_request_sub_ = this->create_subscription<std_msgs::msg::String>(
                   "task_request", 10,
                   std::bind(&TaskAllocator::task_request_callback, this, std::placeholders::_1));
           } else {
               // Robot bids on tasks
               task_request_pub_ = this->create_publisher<std_msgs::msg::String>(
                   "task_request", 10);
               task_assignment_sub_ = this->create_subscription<std_msgs::msg::String>(
                   "task_assignment", 10,
                   std::bind(&TaskAllocator::task_assignment_callback, this, std::placeholders::_1));
           }

           // Timer for robot operations
           robot_timer_ = this->create_wall_timer(
               std::chrono::milliseconds(1000),
               std::bind(&TaskAllocator::robot_operation, this));
       }

   private:
       void task_request_callback(const std_msgs::msg::String::SharedPtr msg)
       {
           // Simple auction: assign task to robot that requested it
           if (!msg->data.empty()) {
               auto assignment_msg = std_msgs::msg::String();
               assignment_msg.data = msg->data + ";" + std::to_string(robot_id_);
               auctioneer_pub_->publish(assignment_msg);
               RCLCPP_INFO(this->get_logger(), "Assigned task to robot %d", robot_id_);
           }
       }

       void task_assignment_callback(const std_msgs::msg::String::SharedPtr msg)
       {
           // Process task assignment
           std::string assignment = msg->data;
           RCLCPP_INFO(this->get_logger(), "Robot %d received assignment: %s", robot_id_, assignment.c_str());
       }

       void robot_operation()
       {
           if (!is_auctioneer_) {
               // Robot periodically requests tasks
               auto request_msg = std_msgs::msg::String();
               request_msg.data = "task_request_from_robot_" + std::to_string(robot_id_);
               task_request_pub_->publish(request_msg);
           }
       }

       int robot_id_;
       bool is_auctioneer_;

       // Auctioneer interfaces
       rclcpp::Publisher<std_msgs::msg::String>::SharedPtr auctioneer_pub_;
       rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_request_sub_;

       // Robot interfaces
       rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_request_pub_;
       rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_assignment_sub_;

       rclcpp::TimerBase::SharedPtr robot_timer_;
   };

   int main(int argc, char * argv[])
   {
       rclcpp::init(argc, argv);
       rclcpp::spin(std::make_shared<TaskAllocator>());
       rclcpp::shutdown();
       return 0;
   }
   ```

2. Create a launch file to test the task allocation system with multiple robots and tasks.

### Task 4.2: Performance Evaluation
1. Create nodes to evaluate task allocation performance:
   - Time to complete all tasks
   - Robot utilization
   - Load balancing
   - Communication overhead

2. Compare different allocation strategies (auction-based vs. round-robin vs. random).

## Assessment Questions

1. What are the advantages and disadvantages of centralized vs. decentralized communication in multi-robot systems?
2. How does network topology affect the performance of consensus algorithms?
3. What factors influence the stability of formation control systems?
4. How do you handle communication delays in multi-robot coordination?
5. What metrics would you use to evaluate the performance of a multi-robot system?

## Troubleshooting Tips

- **Communication issues**: Check ROS 2 namespaces and network configuration
- **Synchronization problems**: Ensure all robots have synchronized clocks
- **Coordination failures**: Verify communication ranges and reliability
- **Formation instability**: Tune control parameters and check sensor accuracy
- **Task allocation conflicts**: Implement conflict resolution mechanisms

## Extensions

1. Implement a bio-inspired swarm algorithm (e.g., ant colony optimization)
2. Add obstacle avoidance to formation control
3. Create a hybrid centralized/decentralized coordination system
4. Implement fault-tolerant coordination that handles robot failures
5. Add machine learning components for adaptive coordination

## Summary

This lab exercise provided hands-on experience with multi-robot systems and coordination. You implemented communication architectures, consensus algorithms, formation control, and task allocation systems. These skills are essential for creating effective multi-robot applications.