import os
import sys
import time
import speech_recognition as sr
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# TurtleBot3 velocity limits
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84
WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.2
ANG_VEL_STEP_SIZE = 0.2

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')

class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')

        qos = QoSProfile(depth=10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self.voice_pub = self.create_publisher(String, '/recognized_command', qos)

        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0

        # Timer to listen for voice input
        self.timer = self.create_timer(0.5, self.speech_to_text_callback)

        # Timer to continuously publish velocity at 10Hz
        self.vel_timer = self.create_timer(0.1, self.publish_velocity)

        # Timer to auto-stop if no command received in 5 seconds
        self.last_command_time = time.time()
        self.timeout_duration = 5.0  # seconds

        self.get_logger().info("Voice Control Node for TurtleBot3 is running!")

    def speech_to_text_callback(self):
        recognizer = sr.Recognizer()
        try:
            with sr.Microphone() as source:
                self.get_logger().info("Listening for voice command...")
                recognizer.adjust_for_ambient_noise(source)
                try:
                    audio = recognizer.listen(source, timeout=3)
                    text = recognizer.recognize_google(audio).lower()
                    self.get_logger().info(f"Recognized command: {text}")

                    # 🔊 Beep feedback
                    print('\a')  # system bell (simple beep)

                    # Publish the recognized voice command
                    msg = String()
                    msg.data = text
                    self.voice_pub.publish(msg)

                    # Update movement
                    self.process_voice_command(text)
                    self.last_command_time = time.time()  # reset timeout timer

                except sr.WaitTimeoutError:
                    self.get_logger().warn("No voice detected. Retrying...")
        except sr.UnknownValueError:
            self.get_logger().warn("Could not recognize speech. Please try again.")
        except sr.RequestError as e:
            self.get_logger().error(f"Speech recognition error: {e}")

    def process_voice_command(self, command):
        if "forward" in command:
            self.target_linear_velocity = self.check_linear_limit(
                self.target_linear_velocity + LIN_VEL_STEP_SIZE)
            self.get_logger().info("Moving forward")
        elif "backward" in command or "reverse" in command:
            self.target_linear_velocity = self.check_linear_limit(
                self.target_linear_velocity - LIN_VEL_STEP_SIZE)
            self.get_logger().info("Moving backward")
        elif "left" in command:
            self.target_angular_velocity = self.check_angular_limit(
                self.target_angular_velocity + ANG_VEL_STEP_SIZE)
            self.get_logger().info("Turning left")
        elif "right" in command:
            self.target_angular_velocity = self.check_angular_limit(
                self.target_angular_velocity - ANG_VEL_STEP_SIZE)
            self.get_logger().info("Turning right")
        elif "stop" in command:
            self.target_linear_velocity = 0.0
            self.target_angular_velocity = 0.0
            self.get_logger().info("Stopping the robot")
        else:
            self.get_logger().warn("Unrecognized command. Ignoring.")

    def publish_velocity(self):
        # Check for command timeout
        if time.time() - self.last_command_time > self.timeout_duration:
            if self.target_linear_velocity != 0.0 or self.target_angular_velocity != 0.0:
                self.get_logger().info("No voice input. Auto-stopping robot.")
            self.target_linear_velocity = 0.0
            self.target_angular_velocity = 0.0

        twist = Twist()
        twist.linear.x = self.target_linear_velocity
        twist.angular.z = self.target_angular_velocity
        self.cmd_vel_pub.publish(twist)

    def check_linear_limit(self, velocity):
        if TURTLEBOT3_MODEL == 'burger':
            return max(min(velocity, BURGER_MAX_LIN_VEL), -BURGER_MAX_LIN_VEL)
        else:
            return max(min(velocity, WAFFLE_MAX_LIN_VEL), -WAFFLE_MAX_LIN_VEL)

    def check_angular_limit(self, velocity):
        if TURTLEBOT3_MODEL == 'burger':
            return max(min(velocity, BURGER_MAX_ANG_VEL), -BURGER_MAX_ANG_VEL)
        else:
            return max(min(velocity, WAFFLE_MAX_ANG_VEL), -WAFFLE_MAX_ANG_VEL)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
