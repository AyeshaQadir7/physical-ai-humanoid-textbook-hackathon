# Lab Exercise: Human-Robot Interaction & Social Robotics

## Overview

In this lab exercise, you will implement natural communication interfaces for robots, design social behaviors that enhance human-robot collaboration, and create adaptive interaction systems. You will work with speech recognition, gesture detection, and social behavior implementation. This builds upon your Week 1-8 foundations to create robots that can interact naturally and effectively with humans.

## Prerequisites

- Completion of Week 1-8 lab exercises
- Working ROS 2 Humble Hawksbill installation
- Robot platform with speech, vision, and display capabilities
- Microphone and speaker for audio interaction
- Camera for gesture and facial recognition
- Experience with perception and control systems
- Understanding of user interface design principles

## Learning Objectives

By completing this lab, you will:
- Implement natural communication interfaces for robots (speech, gesture, facial expressions)
- Design social behaviors that enhance human-robot collaboration
- Apply ethical considerations in social robotics design
- Evaluate human-robot interaction quality and user experience
- Develop robots that can adapt to different user preferences and contexts

## Exercise 1: Speech-Based Interaction (2.5 hours)

### Task 1.1: Speech Recognition and Response System
1. Create a ROS 2 package for speech-based interaction:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --dependencies rclpy std_msgs sensor_msgs geometry_msgs sound_play --build-type ament_python hri_lab
   cd hri_lab
   ```

2. Install required Python packages:
   ```bash
   pip3 install speechrecognition pyttsx3 pocketsphinx
   ```

3. Create a speech interaction node `hri_lab/speech_interaction.py`:
   ```python
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String
   from geometry_msgs.msg import Twist
   import speech_recognition as sr
   import pyttsx3
   import threading
   import time

   class SpeechInteractionNode(Node):
       def __init__(self):
           super().__init__('speech_interaction_node')

           # Initialize speech recognition and synthesis
           self.recognizer = sr.Recognizer()
           self.microphone = sr.Microphone()
           self.tts_engine = pyttsx3.init()

           # Set up text-to-speech properties
           self.tts_engine.setProperty('rate', 150)  # Speed of speech
           self.tts_engine.setProperty('volume', 0.9)  # Volume level

           # Publishers and subscribers
           self.speech_cmd_pub = self.create_publisher(String, 'speech_commands', 10)
           self.robot_cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
           self.response_pub = self.create_publisher(String, 'robot_response', 10)

           # Robot state
           self.robot_state = 'idle'  # idle, moving, etc.
           self.user_name = 'User'

           # Start speech recognition in a separate thread
           self.speech_thread = threading.Thread(target=self.listen_for_speech, daemon=True)
           self.speech_thread.start()

           self.get_logger().info('Speech Interaction Node Started')

       def listen_for_speech(self):
           with self.microphone as source:
               self.recognizer.adjust_for_ambient_noise(source)  # Adjust for background noise

           while rclpy.ok():
               try:
                   with self.microphone as source:
                       self.get_logger().info('Listening for speech...')
                       audio = self.recognizer.listen(source, timeout=2, phrase_time_limit=5)

                   # Recognize speech
                   text = self.recognizer.recognize_google(audio).lower()
                   self.get_logger().info(f'Recognized: {text}')

                   # Process the recognized text
                   response = self.process_speech_command(text)
                   if response:
                       self.speak_response(response)

                   # Publish the recognized command
                   cmd_msg = String()
                   cmd_msg.data = text
                   self.speech_cmd_pub.publish(cmd_msg)

               except sr.WaitTimeoutError:
                   # No speech detected, continue listening
                   continue
               except sr.UnknownValueError:
                   self.get_logger().info('Could not understand audio')
               except sr.RequestError as e:
                   self.get_logger().error(f'Speech recognition error: {e}')
               except Exception as e:
                   self.get_logger().error(f'Error in speech recognition: {e}')

               time.sleep(0.1)  # Small delay to prevent excessive CPU usage

       def process_speech_command(self, text):
           """Process recognized speech and generate appropriate response"""
           text = text.lower()

           # Greeting responses
           if any(greeting in text for greeting in ['hello', 'hi', 'hey', 'greetings']):
               return f"Hello {self.user_name}! How can I assist you today?"

           # Movement commands
           elif 'move forward' in text or 'go forward' in text:
               self.move_robot('forward')
               return "Moving forward as requested."
           elif 'turn left' in text or 'rotate left' in text:
               self.move_robot('left')
               return "Turning left as requested."
           elif 'turn right' in text or 'rotate right' in text:
               self.move_robot('right')
               return "Turning right as requested."
           elif 'stop' in text or 'halt' in text:
               self.move_robot('stop')
               return "Stopping movement."
           elif 'backward' in text or 'back up' in text:
               self.move_robot('backward')
               return "Moving backward as requested."

           # Information requests
           elif 'your name' in text:
               return "I am your robotic assistant. You can call me Assistant."
           elif 'how are you' in text:
               return "I am functioning well, thank you for asking!"
           elif 'time' in text:
               current_time = time.strftime("%H:%M")
               return f"The current time is {current_time}."

           # Unknown command
           else:
               return f"I'm sorry, I didn't understand '{text}'. Please try again."

       def move_robot(self, direction):
           """Send movement commands to the robot"""
           cmd_msg = Twist()

           if direction == 'forward':
               cmd_msg.linear.x = 0.3
           elif direction == 'backward':
               cmd_msg.linear.x = -0.3
           elif direction == 'left':
               cmd_msg.angular.z = 0.5
           elif direction == 'right':
               cmd_msg.angular.z = -0.5
           elif direction == 'stop':
               cmd_msg.linear.x = 0.0
               cmd_msg.angular.z = 0.0

           self.robot_cmd_pub.publish(cmd_msg)

       def speak_response(self, response):
           """Speak the response using text-to-speech"""
           self.get_logger().info(f'Responding: {response}')

           # Publish response
           response_msg = String()
           response_msg.data = response
           self.response_pub.publish(response_msg)

           # Speak using text-to-speech
           self.tts_engine.say(response)
           self.tts_engine.runAndWait()

   def main(args=None):
       rclpy.init(args=args)
       speech_node = SpeechInteractionNode()

       try:
           rclpy.spin(speech_node)
       except KeyboardInterrupt:
           pass
       finally:
           speech_node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

4. Update the package setup in `setup.py` to include the new executable:
   ```python
   entry_points={
       'console_scripts': [
           'speech_interaction = hri_lab.speech_interaction:main',
       ],
   },
   ```

### Task 1.2: Enhanced Speech Processing
1. Create a more sophisticated dialogue manager that can handle context and multi-turn conversations:
   ```python
   # hri_lab/dialogue_manager.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String
   import json
   import re

   class DialogueManager(Node):
       def __init__(self):
           super().__init__('dialogue_manager')

           self.conversation_context = {}
           self.user_profiles = {}
           self.current_user = 'default'

           # Subscriptions and publications
           self.speech_sub = self.create_subscription(
               String, 'speech_commands', self.speech_callback, 10)
           self.response_pub = self.create_publisher(
               String, 'robot_response', 10)

           self.get_logger().info('Dialogue Manager Started')

       def speech_callback(self, msg):
           user_input = msg.data.lower()
           response = self.process_input(user_input)

           response_msg = String()
           response_msg.data = response
           self.response_pub.publish(response_msg)

       def process_input(self, user_input):
           # Intent recognition using pattern matching
           if self.is_greeting(user_input):
               return self.handle_greeting(user_input)
           elif self.is_navigation_command(user_input):
               return self.handle_navigation(user_input)
           elif self.is_question(user_input):
               return self.handle_question(user_input)
           else:
               return self.handle_unknown(user_input)

       def is_greeting(self, text):
           greetings = ['hello', 'hi', 'hey', 'greetings', 'good morning', 'good afternoon', 'good evening']
           return any(greeting in text for greeting in greetings)

       def is_navigation_command(self, text):
           nav_commands = ['go', 'move', 'turn', 'stop', 'forward', 'backward', 'left', 'right']
           return any(cmd in text for cmd in nav_commands)

       def is_question(self, text):
           return '?' in text or any(q_word in text for q_word in ['what', 'how', 'when', 'where', 'who', 'why'])

       def handle_greeting(self, text):
           # Extract potential name
           name_match = re.search(r'\b(?:my name is|i am|i\'?m)\s+(\w+)', text)
           if name_match:
               name = name_match.group(1)
               self.user_profiles[self.current_user] = {'name': name}
               return f"Nice to meet you, {name}! How can I help you today?"
           else:
               user_name = self.user_profiles.get(self.current_user, {}).get('name', 'there')
               return f"Hello {user_name}! How can I assist you?"

       def handle_navigation(self, text):
           # This would interface with navigation system
           return "I can help with navigation. Please specify where you'd like me to go."

       def handle_question(self, text):
           # Simple question answering
           if 'time' in text:
               import time
               current_time = time.strftime("%H:%M")
               return f"The current time is {current_time}."
           elif 'name' in text:
               user_name = self.user_profiles.get(self.current_user, {}).get('name', 'I')
               return f"My name is Assistant. {user_name} is your name."
           else:
               return "That's an interesting question. Could you provide more details?"

       def handle_unknown(self, text):
           return f"I'm not sure I understand. Could you rephrase that?"

   def main(args=None):
       rclpy.init(args=args)
       dm = DialogueManager()

       try:
           rclpy.spin(dm)
       except KeyboardInterrupt:
           pass
       finally:
           dm.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Add the dialogue manager to `setup.py` and build the package.

## Exercise 2: Gesture-Based Interaction (2.5 hours)

### Task 2.1: Hand Gesture Recognition
1. Create a gesture recognition node using OpenCV:
   ```python
   # hri_lab/gesture_recognition.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image
   from std_msgs.msg import String
   from cv_bridge import CvBridge
   import cv2
   import numpy as np
   from collections import deque

   class GestureRecognitionNode(Node):
       def __init__(self):
           super().__init__('gesture_recognition_node')

           self.bridge = CvBridge()
           self.gesture_history = deque(maxlen=10)

           # Publishers and subscribers
           self.image_sub = self.create_subscription(
               Image, '/camera/image_raw', self.image_callback, 10)
           self.gesture_pub = self.create_publisher(
               String, 'detected_gestures', 10)

           self.get_logger().info('Gesture Recognition Node Started')

       def image_callback(self, msg):
           try:
               # Convert ROS Image message to OpenCV image
               cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

               # Process image for gesture recognition
               gesture, processed_image = self.recognize_gesture(cv_image)

               if gesture:
                   # Publish detected gesture
                   gesture_msg = String()
                   gesture_msg.data = gesture
                   self.gesture_pub.publish(gesture_msg)
                   self.get_logger().info(f'Detected gesture: {gesture}')

                   # Add to history for temporal analysis
                   self.gesture_history.append(gesture)

           except Exception as e:
               self.get_logger().error(f'Error processing image: {str(e)}')

       def recognize_gesture(self, image):
           # Convert to HSV for better color segmentation
           hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

           # Define range for skin color (adjust as needed for lighting conditions)
           lower_skin = np.array([0, 20, 70], dtype=np.uint8)
           upper_skin = np.array([20, 255, 255], dtype=np.uint8)

           # Create mask for skin color
           mask = cv2.inRange(hsv, lower_skin, upper_skin)

           # Apply morphological operations to clean up the mask
           kernel = np.ones((5,5), np.uint8)
           mask = cv2.dilate(mask, kernel, iterations=1)
           mask = cv2.GaussianBlur(mask, (3,3), 0)

           # Find contours
           contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

           if contours:
               # Find the largest contour (assumed to be the hand)
               hand_contour = max(contours, key=cv2.contourArea)

               if cv2.contourArea(hand_contour) > 5000:  # Minimum area threshold
                   # Calculate the convex hull
                   hull = cv2.convexHull(hand_contour)
                   hull_area = cv2.contourArea(hull)

                   # Calculate the convexity defects
                   hull2 = cv2.convexHull(hand_contour, returnPoints=False)
                   defects = cv2.convexityDefects(hand_contour, hull2)

                   if defects is not None:
                       # Count defects (gaps between fingers)
                       defect_count = 0
                       for i in range(defects.shape[0]):
                           s, e, f, d = defects[i, 0]
                           start = tuple(hand_contour[s][0])
                           end = tuple(hand_contour[e][0])
                           far = tuple(hand_contour[f][0])

                           # Filter defects based on distance and angle
                           if d > 10000:  # Adjust threshold as needed
                               defect_count += 1

                       # Determine gesture based on defect count
                       gesture = self.classify_gesture(defect_count, hull_area)
                       return gesture, image

           return None, image

       def classify_gesture(self, defect_count, hull_area):
           # Simple gesture classification based on number of defects
           # More sophisticated methods would use ML models
           if defect_count == 0:
               return "fist"  # Closed hand
           elif defect_count == 1:
               return "one"   # One finger up
           elif defect_count == 2:
               return "two"   # Two fingers up
           elif defect_count == 3:
               return "three" # Three fingers up
           elif defect_count == 4:
               return "five"  # Open hand (5 fingers)
           else:
               return "unknown"

   def main(args=None):
       rclpy.init(args=args)
       gesture_node = GestureRecognitionNode()

       try:
           rclpy.spin(gesture_node)
       except KeyboardInterrupt:
           pass
       finally:
           gesture_node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Add the gesture recognition node to `setup.py` and build the package.

### Task 2.2: Gesture-Command Mapping
1. Create a node that maps gestures to robot commands:
   ```python
   # hri_lab/gesture_command_mapper.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String
   from geometry_msgs.msg import Twist
   from std_msgs.msg import Bool

   class GestureCommandMapper(Node):
       def __init__(self):
           super().__init__('gesture_command_mapper')

           # Gesture to command mapping
           self.gesture_commands = {
               'five': 'forward',      # Open hand = move forward
               'fist': 'stop',         # Closed fist = stop
               'one': 'turn_left',     # One finger = turn left
               'two': 'turn_right',    # Two fingers = turn right
               'three': 'backward'     # Three fingers = move backward
           }

           # Publishers and subscribers
           self.gesture_sub = self.create_subscription(
               String, 'detected_gestures', self.gesture_callback, 10)
           self.cmd_vel_pub = self.create_publisher(
               Twist, 'cmd_vel', 10)
           self.feedback_pub = self.create_publisher(
               String, 'hri_feedback', 10)

           self.get_logger().info('Gesture Command Mapper Started')

       def gesture_callback(self, msg):
           gesture = msg.data

           if gesture in self.gesture_commands:
               command = self.gesture_commands[gesture]
               self.execute_command(command, gesture)
           else:
               self.get_logger().info(f'Unknown gesture: {gesture}')

       def execute_command(self, command, gesture):
           cmd_msg = Twist()

           if command == 'forward':
               cmd_msg.linear.x = 0.3
               feedback = f'Moving forward based on {gesture} gesture'
           elif command == 'stop':
               cmd_msg.linear.x = 0.0
               cmd_msg.angular.z = 0.0
               feedback = f'Stopping based on {gesture} gesture'
           elif command == 'turn_left':
               cmd_msg.angular.z = 0.5
               feedback = f'Turning left based on {gesture} gesture'
           elif command == 'turn_right':
               cmd_msg.angular.z = -0.5
               feedback = f'Turning right based on {gesture} gesture'
           elif command == 'backward':
               cmd_msg.linear.x = -0.3
               feedback = f'Moving backward based on {gesture} gesture'
           else:
               feedback = f'Unknown command for gesture: {gesture}'

           self.cmd_vel_pub.publish(cmd_msg)
           self.get_logger().info(feedback)

           # Publish feedback
           feedback_msg = String()
           feedback_msg.data = feedback
           self.feedback_pub.publish(feedback_msg)

   def main(args=None):
       rclpy.init(args=args)
       mapper = GestureCommandMapper()

       try:
           rclpy.spin(mapper)
       except KeyboardInterrupt:
           pass
       finally:
           mapper.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

## Exercise 3: Social Behavior Implementation (2 hours)

### Task 3.1: Social Robot Personality
1. Create a node that implements social behaviors and personality traits:
   ```python
   # hri_lab/social_behavior.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String, Bool
   from geometry_msgs.msg import Twist
   import random
   import time

   class SocialBehaviorNode(Node):
       def __init__(self):
           super().__init__('social_behavior_node')

           # Personality parameters
           self.personality_traits = {
               'extroversion': 0.7,  # 0-1 scale
               'agreeableness': 0.8,
               'conscientiousness': 0.6,
               'emotional_stability': 0.9,
               'openness': 0.5
           }

           # Interaction state
           self.last_interaction_time = time.time()
           self.user_engagement_level = 0.5  # 0-1 scale
           self.social_energy = 1.0  # 0-1 scale

           # Publishers
           self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
           self.response_pub = self.create_publisher(String, 'robot_response', 10)
           self.social_action_pub = self.create_publisher(String, 'social_actions', 10)

           # Subscriptions
           self.user_presence_sub = self.create_subscription(
               Bool, 'user_present', self.user_presence_callback, 10)
           self.interaction_sub = self.create_subscription(
               String, 'speech_commands', self.interaction_callback, 10)

           # Timer for social behaviors
           self.social_behavior_timer = self.create_timer(10.0, self.social_behavior_loop)

           self.get_logger().info('Social Behavior Node Started')

       def user_presence_callback(self, msg):
           if msg.data:  # User is present
               time_since_interaction = time.time() - self.last_interaction_time
               if time_since_interaction > 30:  # 30 seconds since last interaction
                   self.initiate_social_interaction()

       def interaction_callback(self, msg):
           self.last_interaction_time = time.time()
           self.user_engagement_level = min(1.0, self.user_engagement_level + 0.1)
           self.social_energy = min(1.0, self.social_energy + 0.05)

       def initiate_social_interaction(self):
           # Based on personality traits, decide how to approach user
           extroversion = self.personality_traits['extroversion']

           if random.random() < extroversion:
               # Approach the user
               self.approach_user()
               self.greet_user()
           else:
               # Wait for user to approach
               self.wait_for_interaction()

       def approach_user(self):
           self.get_logger().info('Approaching user')
           cmd_msg = Twist()
           cmd_msg.linear.x = 0.2  # Move toward user
           self.cmd_vel_pub.publish(cmd_msg)

       def greet_user(self):
           greetings = [
               "Hello! I'm glad to see you!",
               "Hi there! How are you doing today?",
               "Greetings! I was hoping you'd come by!",
               "Hello friend! What would you like to do together?"
           ]

           greeting = random.choice(greetings)
           self.get_logger().info(f'Greeting user: {greeting}')

           response_msg = String()
           response_msg.data = greeting
           self.response_pub.publish(response_msg)

       def wait_for_interaction(self):
           self.get_logger().info('Waiting for user to initiate interaction')
           # For now, just log the waiting state
           pass

       def social_behavior_loop(self):
           # Periodically update social energy and engagement
           self.social_energy = max(0.1, self.social_energy - 0.01)  # Gradually decrease energy
           self.user_engagement_level = max(0.1, self.user_engagement_level - 0.02)  # Decrease if no interaction

           # If social energy is high and user engagement is low, try to engage
           if self.social_energy > 0.7 and self.user_engagement_level < 0.3:
               self.attempt_engagement()

       def attempt_engagement(self):
           if random.random() < self.personality_traits['extroversion']:
               engagement_attempts = [
                   "I have something interesting to show you!",
                   "Would you like to try something new together?",
                   "I was thinking we could do an activity together.",
                   "I'm ready for some interaction when you are!"
               ]

               attempt = random.choice(engagement_attempts)
               self.get_logger().info(f'Attempting to engage: {attempt}')

               response_msg = String()
               response_msg.data = attempt
               self.response_pub.publish(response_msg)

   def main(args=None):
       rclpy.init(args=args)
       social_node = SocialBehaviorNode()

       try:
           rclpy.spin(social_node)
       except KeyboardInterrupt:
           pass
       finally:
           social_node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Add the social behavior node to `setup.py`.

### Task 3.2: Adaptive Interaction System
1. Create a system that adapts to user preferences and context:
   ```python
   # hri_lab/adaptation_system.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String, Float32
   import json
   import time
   from collections import defaultdict

   class AdaptationSystem(Node):
       def __init__(self):
           super().__init__('adaptation_system')

           # User models and preferences
           self.user_models = {}
           self.current_user = 'default'
           self.interaction_history = defaultdict(list)

           # Interaction parameters that can be adapted
           self.interaction_params = {
               'response_speed': 0.5,  # 0-1, how quickly to respond
               'formality_level': 0.7, # 0-1, formal vs casual
               'initiative_level': 0.3, # 0-1, how proactive to be
               'patience_level': 0.8   # 0-1, how patient to be
           }

           # Subscriptions
           self.user_interaction_sub = self.create_subscription(
               String, 'speech_commands', self.interaction_callback, 10)
           self.response_time_sub = self.create_subscription(
               Float32, 'response_time', self.response_time_callback, 10)

           # Publishers
           self.adaptation_pub = self.create_publisher(
               String, 'adaptation_updates', 10)

           self.get_logger().info('Adaptation System Started')

       def interaction_callback(self, msg):
           user_input = msg.data
           self.interaction_history[self.current_user].append({
               'timestamp': time.time(),
               'input': user_input,
               'response_time': self.interaction_params['response_speed']
           })

           # Update user model based on interaction
           self.update_user_model()

       def response_time_callback(self, msg):
           # Update response time in history
           if self.interaction_history[self.current_user]:
               self.interaction_history[self.current_user][-1]['actual_response_time'] = msg.data

       def update_user_model(self):
           # Analyze interaction history to update user preferences
           interactions = self.interaction_history[self.current_user][-10:]  # Last 10 interactions

           if len(interactions) < 3:
               return  # Not enough data yet

           # Calculate average response time expectation
           if all('actual_response_time' in i for i in interactions):
               avg_response_time = sum(i['actual_response_time'] for i in interactions) / len(interactions)

               # Adjust response speed based on user expectations
               if avg_response_time < 2.0:  # User expects quick responses
                   self.interaction_params['response_speed'] = min(1.0, self.interaction_params['response_speed'] + 0.1)
               elif avg_response_time > 5.0:  # User is patient
                   self.interaction_params['response_speed'] = max(0.1, self.interaction_params['response_speed'] - 0.1)

           # Determine formality based on greeting patterns
           formal_greetings = ['good morning', 'good afternoon', 'good evening', 'hello', 'greetings']
           casual_greetings = ['hi', 'hey', 'yo']

           formal_count = sum(1 for i in interactions if any(g in i['input'].lower() for g in formal_greetings))
           casual_count = sum(1 for i in interactions if any(g in i['input'].lower() for g in casual_greetings))

           if formal_count > casual_count:
               self.interaction_params['formality_level'] = min(1.0, self.interaction_params['formality_level'] + 0.1)
           elif casual_count > formal_count:
               self.interaction_params['formality_level'] = max(0.0, self.interaction_params['formality_level'] - 0.1)

           self.get_logger().info(f'Updated interaction parameters: {self.interaction_params}')

       def get_adapted_response_style(self):
           """Return parameters for adapting responses based on user model"""
           return {
               'speed': self.interaction_params['response_speed'],
               'formality': self.interaction_params['formality_level'],
               'initiative': self.interaction_params['initiative_level'],
               'patience': self.interaction_params['patience_level']
           }

   def main(args=None):
       rclpy.init(args=args)
       adapter = AdaptationSystem()

       try:
           rclpy.spin(adapter)
       except KeyboardInterrupt:
           # Save user models
           adapter.get_logger().info('Saving user models...')
           pass
       finally:
           adapter.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

## Exercise 4: HRI Evaluation (1 hour)

### Task 4.1: Interaction Quality Metrics
1. Create a node to evaluate and monitor HRI quality:
   ```python
   # hri_lab/hri_evaluator.py
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String, Float32
   import time
   import statistics

   class HRIEvaluator(Node):
       def __init__(self):
           super().__init__('hri_evaluator')

           # Metrics tracking
           self.interaction_count = 0
           self.response_times = []
           self.successful_interactions = 0
           self.user_satisfaction_scores = []
           self.engagement_level = 0.5

           # Subscriptions
           self.interaction_sub = self.create_subscription(
               String, 'speech_commands', self.interaction_callback, 10)
           self.response_time_sub = self.create_subscription(
               Float32, 'response_time', self.response_time_callback, 10)
           self.satisfaction_sub = self.create_subscription(
               Float32, 'user_satisfaction', self.satisfaction_callback, 10)

           # Publishers
           self.metrics_pub = self.create_publisher(
               String, 'hri_metrics', 10)

           # Timer for periodic evaluation
           self.evaluation_timer = self.create_timer(30.0, self.evaluate_performance)

           self.start_time = time.time()

           self.get_logger().info('HRI Evaluator Started')

       def interaction_callback(self, msg):
           self.interaction_count += 1

       def response_time_callback(self, msg):
           self.response_times.append(msg.data)

       def satisfaction_callback(self, msg):
           self.user_satisfaction_scores.append(msg.data.data)

       def evaluate_performance(self):
           # Calculate metrics
           uptime = time.time() - self.start_time
           interaction_rate = self.interaction_count / (uptime / 60) if uptime > 0 else 0

           avg_response_time = statistics.mean(self.response_times) if self.response_times else 0
           avg_satisfaction = statistics.mean(self.user_satisfaction_scores) if self.user_satisfaction_scores else 0

           # Create metrics report
           metrics = {
               'uptime_minutes': uptime / 60,
               'total_interactions': self.interaction_count,
               'interaction_rate_per_minute': interaction_rate,
               'avg_response_time': avg_response_time,
               'avg_satisfaction': avg_satisfaction,
               'engagement_level': self.engagement_level
           }

           # Publish metrics
           metrics_msg = String()
           metrics_msg.data = json.dumps(metrics, indent=2)
           self.metrics_pub.publish(metrics_msg)

           self.get_logger().info(f'HRI Metrics: {json.dumps(metrics, indent=2)}')

   def main(args=None):
       rclpy.init(args=args)
       evaluator = HRIEvaluator()

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

1. How would you design a multimodal interface that combines speech, gesture, and visual feedback?
2. What ethical considerations should be taken into account when designing social robots for elderly care?
3. How would you evaluate the effectiveness of a social robot's adaptive interaction system?
4. What safety mechanisms would you implement for a social robot operating in a public space?
5. How would you design a social robot to be culturally sensitive across different populations?

## Troubleshooting Tips

- **Speech recognition issues**: Ensure proper microphone setup and reduce background noise
- **Gesture recognition accuracy**: Adjust color thresholds based on lighting conditions
- **Privacy concerns**: Implement proper data handling and user consent mechanisms
- **Social acceptance**: Test with target users and iterate based on feedback
- **Performance issues**: Optimize algorithms for real-time operation

## Extensions

1. Implement emotion recognition using facial expression analysis
2. Add personality customization based on user preferences
3. Create a learning system that improves interaction over time
4. Implement cultural adaptation for different user populations
5. Add haptic feedback for enhanced interaction

## Summary

This lab exercise provided hands-on experience with human-robot interaction and social robotics. You implemented speech recognition, gesture detection, social behaviors, and adaptive interaction systems. These skills are essential for creating robots that can interact naturally and effectively with humans in various applications.