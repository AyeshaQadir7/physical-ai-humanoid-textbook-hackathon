# Week 2 Assessment: Robot Operating Systems and ROS 2 Fundamentals

## Overview

This assessment evaluates your understanding of ROS 2 core concepts including nodes, topics, services, actions, QoS settings, TF transforms, and launch files. You will demonstrate both theoretical knowledge and practical implementation skills.

## Learning Objectives Assessment

By completing this assessment, you will demonstrate your ability to:
- Explain ROS 2 communication patterns and their appropriate use cases
- Implement custom message types and use different QoS settings
- Create and use services and actions for robot communication
- Work with TF transforms for coordinate frame management
- Organize robot systems using launch files

## Part A: Theoretical Knowledge (40 points)

### Question 1: ROS 2 Communication Patterns (10 points)
Compare and contrast the three main ROS 2 communication patterns (topics, services, actions). For each pattern, provide:
- A definition and purpose
- An appropriate use case
- Advantages and disadvantages
- An example scenario where it would be the best choice

### Question 2: Quality of Service (QoS) Settings (10 points)
Explain Quality of Service (QoS) settings in ROS 2. Describe the following QoS policies and when you would use them:
- Reliability policy (Reliable vs Best Effort)
- Durability policy (Transient Local vs Volatile)
- History policy (Keep Last vs Keep All)
- Depth parameter

### Question 3: TF (Transforms) System (10 points)
Describe the TF (Transforms) system in ROS 2. Explain:
- What coordinate frames are and why they're important
- How the TF tree structure works
- The purpose of transforms between frames
- Common coordinate frames used in robotics

### Question 4: Launch Files and Parameters (10 points)
Explain the purpose and benefits of launch files in ROS 2. Describe:
- How launch files improve system organization
- The structure of a launch file
- How parameters are managed in ROS 2
- The advantages of using launch files over manual node startup

## Part B: Practical Implementation (40 points)

### Task 1: Custom Message Implementation (10 points)
Create a new ROS 2 package called `assessment_msgs` with custom message types for a humanoid robot:
- Create a `JointState` message with fields for joint names, positions, velocities, and efforts
- Create a `HumanoidCommand` message for controlling humanoid robot joints
- Create a `HumanoidStatus` message for reporting robot status
- Build and verify the messages work correctly

### Task 2: Communication Pattern Implementation (15 points)
Create a new ROS 2 package called `comms_examples` that demonstrates all three communication patterns:
- **Topic**: Implement a publisher/subscriber for sensor data with appropriate QoS settings
- **Service**: Implement a service for requesting robot calibration
- **Action**: Implement an action for performing a complex robot task (e.g., walking to a location)
- Include proper error handling and logging

### Task 3: TF Integration (15 points)
Extend the communication example to include TF transforms:
- Create a TF broadcaster that publishes transforms for a simple robot model
- Create a TF listener that subscribes to transforms and uses them for navigation
- Demonstrate coordinate frame transformations
- Show how TF is used in the robot's navigation system

## Part C: System Integration (20 points)

### Task 4: Launch File Configuration (10 points)
Create comprehensive launch files for your robot system:
- Create a main launch file that starts all necessary nodes
- Create separate launch files for different system configurations
- Include parameter files for system configuration
- Implement conditional launching based on parameters

### Task 5: System Testing and Validation (10 points)
Test and validate your complete system:
- Verify all communication patterns work correctly
- Test TF transformations and coordinate frame management
- Validate system performance under different QoS settings
- Document your testing process and results

## Part D: Analysis and Reflection (20 points)

### Question 5: System Design Analysis (10 points)
Analyze the robot system you've built in terms of:
- Scalability: How would the system handle additional nodes or robots?
- Robustness: How does the system handle failures or communication issues?
- Performance: What are potential bottlenecks and how could they be addressed?
- Maintainability: How easy would it be to modify or extend the system?

### Question 6: Future Considerations (10 points)
Consider the future development of your robot system:
- What additional features would you add to improve functionality?
- How would you adapt the system for different robot platforms?
- What security considerations would be important for a production system?
- How would you optimize the system for real-time performance?

## Submission Requirements

### Deliverables:
1. A written document answering all theoretical questions (Part A)
2. All source code for practical implementations (Part B)
3. Launch files and configuration (Part C)
4. Analysis and reflection document (Part D)
5. A README file explaining your implementation approach

### Code Requirements:
- Well-documented source code with comments
- Proper ROS 2 package structure
- Error handling and validation
- Clean, readable code following ROS 2 conventions
- Proper use of logging and debugging tools

### Format:
- Theoretical answers: PDF or text document
- Code: Properly structured ROS 2 packages
- Documentation: README files with setup and usage instructions
- Reflection: 500-700 words discussing your analysis

### Submission Method:
- Create a ZIP file containing all deliverables
- Name the file: `week2_assessment_yourname.zip`
- Include a cover sheet with your name, date, and course information

## Grading Rubric

### Part A: Theoretical Knowledge
- **Excellent (9-10 points)**: Complete, accurate explanations with clear examples
- **Good (7-8 points)**: Mostly accurate with minor omissions
- **Satisfactory (5-6 points)**: Basic understanding with some errors
- **Needs Improvement (0-4 points)**: Incomplete or significantly incorrect

### Part B: Practical Implementation
- **Excellent (36-40 points)**: All tasks completed successfully with clean, well-documented code
- **Good (30-35 points)**: Most tasks completed with minor issues
- **Satisfactory (24-29 points)**: Basic functionality achieved with some errors
- **Needs Improvement (0-23 points)**: Significant issues or incomplete tasks

### Part C: System Integration
- **Excellent (18-20 points)**: Complete integration with proper configuration and documentation
- **Good (15-17 points)**: Good integration with minor issues
- **Satisfactory (12-14 points)**: Basic integration achieved
- **Needs Improvement (0-11 points)**: Significant integration issues

### Part D: Analysis and Reflection
- **Excellent (18-20 points)**: Insightful analysis with well-reasoned arguments
- **Good (15-17 points)**: Good analysis with some depth
- **Satisfactory (12-14 points)**: Basic understanding with limited analysis
- **Needs Improvement (0-11 points)**: Superficial or unclear responses

## Additional Resources

For reference during the assessment, you may use:
- This textbook's Week 1 and Week 2 content
- Official ROS 2 documentation
- Your lab exercise notes and code
- ROS 2 tutorials and examples

## Time Allocation

- Theoretical Questions: 60 minutes
- Practical Implementation: 120 minutes
- System Integration: 60 minutes
- Analysis and Reflection: 30 minutes
- Documentation and Submission: 30 minutes
- Total: 5 hours

## Evaluation Criteria

Your assessment will be evaluated based on:
1. **Accuracy**: Correctness of theoretical knowledge and practical implementation
2. **Completeness**: Thoroughness in addressing all requirements
3. **Understanding**: Demonstration of deep comprehension of concepts
4. **Documentation**: Clarity and quality of code and written explanations
5. **Problem-solving**: Ability to design and implement appropriate solutions
6. **Code Quality**: Following ROS 2 conventions and best practices

## Passing Criteria

To successfully complete this assessment, you must achieve:
- Minimum 70% overall score
- At least 60% on theoretical knowledge (Part A)
- At least 60% on practical implementation (Part B)
- At least 60% on system integration (Part C)
- At least 60% on analysis and reflection (Part D)

## Feedback and Resubmission

After grading, you will receive detailed feedback on your submission. If your score is below the passing threshold, you will have one opportunity to resubmit improved work within one week of receiving feedback.

## Academic Integrity

This assessment must be completed individually. You may reference the textbook and official documentation, but all code and written work must be your own. Any collaboration or plagiarism will result in a failing grade for this assessment.

## Extension Activities (Optional - for additional learning)

For students seeking additional challenge:
1. Implement a custom QoS profile for safety-critical communications
2. Create a parameter server for dynamic configuration
3. Implement a custom action server with preemption capabilities
4. Design a multi-robot coordination system using ROS 2
5. Explore ROS 2 security features and implement basic security configuration