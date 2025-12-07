# Lab Exercise: Advanced Control Systems & Trajectory Planning

## Overview

In this lab exercise, you will implement and tune advanced control systems for robot motion, develop trajectory planning algorithms for complex movements, and integrate control systems with perception and navigation. You will work with PID controllers, implement trajectory planning techniques, and evaluate control system performance. This builds upon your Week 1-4 foundations to create more sophisticated robot behaviors and movements.

## Prerequisites

- Completion of Week 1-4 lab exercises
- Working ROS 2 Humble Hawksbill installation
- Working Gazebo simulation environment
- Understanding of ROS 2 control frameworks
- Basic knowledge of control theory and mathematics
- Access to robot simulation models from previous weeks

## Learning Objectives

By completing this lab, you will:

- Implement and tune PID controllers for robot motion control
- Design trajectory planning algorithms for complex movements
- Integrate control systems with perception and navigation modules
- Evaluate control system performance and stability
- Implement adaptive control systems that respond to environmental changes
- Create and execute time-parameterized trajectories

## Exercise 1: PID Controller Implementation (2 hours)

### Task 1.1: Basic PID Controller

1. Create a new ROS 2 package for control experiments:

   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --dependencies rclcpp std_msgs sensor_msgs geometry_msgs --build-type ament_cmake advanced_control_lab
   cd advanced_control_lab
   ```

2. Create a basic PID controller node in `src/pid_controller.cpp`:

   ```cpp
   #include <rclcpp/rclcpp.hpp>
   #include <std_msgs/msg/float64.hpp>
   #include <geometry_msgs/msg/twist.hpp>

   class PIDController : public rclcpp::Node
   {
   public:
       PIDController() : Node("pid_controller")
       {
           // Initialize PID parameters
           kp_ = this->declare_parameter("kp", 1.0);
           ki_ = this->declare_parameter("ki", 0.1);
           kd_ = this->declare_parameter("kd", 0.05);

           // Initialize error terms
           error_prev_ = 0.0;
           error_int_ = 0.0;

           // Create subscribers and publishers
           target_sub_ = this->create_subscription<std_msgs::msg::Float64>(
               "target_position", 10,
               std::bind(&PIDController::target_callback, this, std::placeholders::_1));

           actual_sub_ = this->create_subscription<std_msgs::msg::Float64>(
               "actual_position", 10,
               std::bind(&PIDController::actual_callback, this, std::placeholders::_1));

           command_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
               "cmd_vel", 10);

           // Create timer for control loop
           control_timer_ = this->create_wall_timer(
               std::chrono::milliseconds(50),
               std::bind(&PIDController::control_loop, this));
       }

   private:
       void target_callback(const std_msgs::msg::Float64::SharedPtr msg)
       {
           target_position_ = msg->data;
       }

       void actual_callback(const std_msgs::msg::Float64::SharedPtr msg)
       {
           actual_position_ = msg->data;
       }

       void control_loop()
       {
           // Calculate error
           double error = target_position_ - actual_position_;

           // Calculate integral and derivative terms
           error_int_ += error * 0.05; // dt = 0.05s
           double error_deriv = (error - error_prev_) / 0.05;

           // Calculate control output
           double output = kp_ * error + ki_ * error_int_ + kd_ * error_deriv;

           // Publish control command
           auto cmd_msg = geometry_msgs::msg::Twist();
           cmd_msg.linear.x = output;
           command_pub_->publish(cmd_msg);

           // Update previous error
           error_prev_ = error;
       }

       // PID parameters
       double kp_, ki_, kd_;
       double target_position_, actual_position_;
       double error_prev_, error_int_;

       // ROS 2 interfaces
       rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_sub_;
       rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr actual_sub_;
       rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
       rclcpp::TimerBase::SharedPtr control_timer_;
   };

   int main(int argc, char * argv[])
   {
       rclcpp::init(argc, argv);
       rclcpp::spin(std::make_shared<PIDController>());
       rclcpp::shutdown();
       return 0;
   }
   ```

3. Create a launch file `launch/pid_test.launch.py`:

   ```python
   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='advanced_control_lab',
               executable='pid_controller',
               name='pid_controller',
               parameters=[
                   {'kp': 1.0},
                   {'ki': 0.1},
                   {'kd': 0.05}
               ]
           ),
           Node(
               package='advanced_control_lab',
               executable='position_simulator',
               name='position_simulator'
           )
       ])
   ```

4. Create a simple position simulator in `src/position_simulator.cpp`:

   ```cpp
   #include <rclcpp/rclcpp.hpp>
   #include <std_msgs/msg/float64.hpp>
   #include <geometry_msgs/msg/twist.hpp>

   class PositionSimulator : public rclcpp::Node
   {
   public:
       PositionSimulator() : Node("position_simulator")
       {
           // Create subscriber for velocity commands
           cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
               "cmd_vel", 10,
               std::bind(&PositionSimulator::cmd_callback, this, std::placeholders::_1));

           // Create publisher for actual position
           pos_pub_ = this->create_publisher<std_msgs::msg::Float64>(
               "actual_position", 10);

           // Create timer for simulation
           sim_timer_ = this->create_wall_timer(
               std::chrono::milliseconds(50),
               std::bind(&PositionSimulator::simulate, this));

           actual_position_ = 0.0;
       }

   private:
       void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
       {
           velocity_cmd_ = msg->linear.x;
       }

       void simulate()
       {
           // Simple first-order system simulation
           actual_position_ += velocity_cmd_ * 0.05; // dt = 0.05s

           // Add some damping
           actual_position_ *= 0.99;

           // Publish actual position
           auto pos_msg = std_msgs::msg::Float64();
           pos_msg.data = actual_position_;
           pos_pub_->publish(pos_msg);
       }

       double actual_position_, velocity_cmd_;

       rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
       rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pos_pub_;
       rclcpp::TimerBase::SharedPtr sim_timer_;
   };

   int main(int argc, char * argv[])
   {
       rclcpp::init(argc, argv);
       rclcpp::spin(std::make_shared<PositionSimulator>());
       rclcpp::shutdown();
       return 0;
   }
   ```

5. Build and test your PID controller with different parameter values to observe the effects on system response.

### Task 1.2: PID Tuning

1. Experiment with different PID parameters (Kp, Ki, Kd) and document their effects on:

   - Rise time
   - Overshoot
   - Settling time
   - Steady-state error

2. Implement a parameter server that allows real-time tuning of PID parameters using ROS 2 services.

## Exercise 2: Trajectory Planning (2 hours)

### Task 2.1: Cubic Trajectory Generator

1. Create a trajectory generator node in `src/cubic_trajectory.cpp`:

   ```cpp
   #include <rclcpp/rclcpp.hpp>
   #include <std_msgs/msg/float64.hpp>
   #include <geometry_msgs/msg/pose_stamped.hpp>

   class CubicTrajectoryGenerator : public rclcpp::Node
   {
   public:
       CubicTrajectoryGenerator() : Node("cubic_trajectory_generator")
       {
           // Create service server for trajectory generation
           trajectory_srv_ = this->create_service<trajectory_msgs::srv::GetTrajectory>(
               "generate_trajectory",
               std::bind(&CubicTrajectoryGenerator::handle_trajectory_request,
                        this, std::placeholders::_1, std::placeholders::_2));

           // Create publisher for trajectory points
           trajectory_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
               "trajectory_point", 10);
       }

   private:
       void handle_trajectory_request(
           const std::shared_ptr<trajectory_msgs::srv::GetTrajectory::Request> request,
           std::shared_ptr<trajectory_msgs::srv::GetTrajectory::Response> response)
       {
           // Calculate cubic polynomial coefficients
           double t0 = request->start_time;
           double t1 = request->end_time;
           double q0 = request->start_position;
           double q1 = request->end_position;
           double v0 = request->start_velocity;
           double v1 = request->end_velocity;

           double dt = t1 - t0;

           // Calculate coefficients for cubic polynomial:
           // q(t) = a0 + a1*t + a2*t^2 + a3*t^3
           double a0 = q0;
           double a1 = v0;
           double a2 = (3.0/dt/dt) * (q1 - q0) - (2.0/dt) * v0 - (1.0/dt) * v1;
           double a3 = (-2.0/dt/dt/dt) * (q1 - q0) + (1.0/dt/dt) * (v1 + v0);

           // Generate trajectory points
           for (double t = t0; t <= t1; t += 0.1) {  // 0.1 second intervals
               double tau = t - t0;  // Time from start of segment
               double position = a0 + a1*tau + a2*tau*tau + a3*tau*tau*tau;
               double velocity = a1 + 2*a2*tau + 3*a3*tau*tau;

               // Publish trajectory point
               auto traj_msg = geometry_msgs::msg::PoseStamped();
               traj_msg.header.stamp = this->now();
               traj_msg.pose.position.x = position;
               trajectory_pub_->publish(traj_msg);
           }
       }

       rclcpp::Service<trajectory_msgs::srv::GetTrajectory>::SharedPtr trajectory_srv_;
       rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr trajectory_pub_;
   };

   int main(int argc, char * argv[])
   {
       rclcpp::init(argc, argv);
       rclcpp::spin(std::make_shared<CubicTrajectoryGenerator>());
       rclcpp::shutdown();
       return 0;
   }
   ```

2. Create a launch file to test the trajectory generator with different start/end conditions.

### Task 2.2: Quintic Trajectory Generator

1. Implement a quintic trajectory generator that provides smooth acceleration profiles:

   ```cpp
   // Similar to cubic but with 5th order polynomial
   // q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
   // Enforce position, velocity, and acceleration continuity
   ```

2. Compare the smoothness of cubic vs quintic trajectories.

## Exercise 3: ROS 2 Control Integration (2 hours)

### Task 3.1: ros2_control Implementation

1. Set up a ros2_control configuration for your robot model:

   - Create a `config` directory in your package
   - Define hardware interface configuration
   - Configure controller manager

2. Create a controller configuration file `config/position_controller.yaml`:

   ```yaml
   controller_manager:
     ros__parameters:
       update_rate: 100 # Hz

   position_trajectory_controller:
     ros__parameters:
       joints:
         - joint1
         - joint2
       interface_name: position

       gains:
         joint1: { p: 1.0, i: 0.1, d: 0.05 }
         joint2: { p: 1.0, i: 0.1, d: 0.05 }
   ```

3. Implement a simple position trajectory controller that follows time-parameterized trajectories.

### Task 3.2: Control System Evaluation

1. Create a node to evaluate control system performance:

   - Calculate rise time, settling time, overshoot
   - Compute integral error metrics (IAE, ISE, ITAE)
   - Visualize system response using RViz or plotjuggler

2. Implement a system identification tool that estimates system parameters from input/output data.

## Assessment Questions

1. What are the advantages and disadvantages of PID control compared to more advanced control techniques?
2. How do you choose appropriate sampling rates for digital control systems?
3. What are the key differences between cubic and quintic trajectory planning?
4. How does feedforward control improve system performance?
5. What factors affect the stability of digital control systems?

## Troubleshooting Tips

- **Oscillation**: Reduce proportional gain (Kp) or increase derivative gain (Kd)
- **Slow response**: Increase proportional gain (Kp)
- **Steady-state error**: Add or increase integral gain (Ki)
- **Actuator saturation**: Implement anti-windup mechanisms
- **Noise sensitivity**: Filter derivative term or reduce derivative gain

## Extensions

1. Implement a Model Predictive Controller (MPC) for trajectory tracking
2. Add adaptive control capabilities that adjust parameters based on system behavior
3. Create a GUI for real-time control parameter tuning
4. Implement control allocation for redundant robot systems
5. Add safety constraints to trajectory planning algorithms

## Summary

This lab exercise provided hands-on experience with advanced control systems and trajectory planning. You implemented PID controllers, designed trajectory generation algorithms, and integrated control systems with ROS 2. These skills are essential for creating sophisticated robot behaviors and movements.
