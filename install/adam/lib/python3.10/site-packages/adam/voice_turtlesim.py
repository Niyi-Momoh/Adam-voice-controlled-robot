import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import speech_recognition as sr
import pyttsx3

class VoiceControlledTurtlesim(Node):
    def __init__(self):
        super().__init__('voice_controlled_turtlesim')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.recognizer = sr.Recognizer()
        self.engine = pyttsx3.init()
        self.car_running = False
        self.get_logger().info("Voice-controlled Turtlesim Node Initialized")

    def SpeakText(self, command):
        """Convert text to speech."""
        self.engine.say(command)
        self.engine.runAndWait()

    def map_command_to_twist(self, command):
        """Map voice commands to Twist messages."""
        twist = Twist()
        if command == "start adam":
            self.car_running = True
            self.SpeakText("Car is now active")
            return None
        elif command == "adam stop":
            self.car_running = False
            self.SpeakText("Car is stopping")
            return twist  # A zero velocity Twist stops the turtle
        elif not self.car_running:
            self.SpeakText("Car is not active. Say 'start adam' to activate.")
            return None

        # Map commands to actions when the car is active
        if command == "go forward":
            twist.linear.x = 2.0  # Move forward
            twist.angular.z = 0.0
        elif command == "turn left":
            twist.linear.x = 0.0
            twist.angular.z = 2.0  # Turn left
        elif command == "turn right":
            twist.linear.x = 0.0
            twist.angular.z = -2.0  # Turn right
        else:
            self.SpeakText("Command not recognized")
            return None
        return twist

    def listen_and_process(self):
        """Listen for voice commands and publish corresponding Twist messages."""
        with sr.Microphone() as source:
            self.get_logger().info("Listening for commands...")
            self.recognizer.adjust_for_ambient_noise(source, duration=0.2)
            try:
                audio = self.recognizer.listen(source)
                command = self.recognizer.recognize_google(audio).lower()
                self.get_logger().info(f"Command received: {command}")
                self.SpeakText(f"Did you say: {command}?")

                twist = self.map_command_to_twist(command)
                if twist is not None:
                    self.publisher_.publish(twist)
                    self.get_logger().info("Command executed")

            except sr.UnknownValueError:
                self.get_logger().info("Could not understand the audio")
                self.SpeakText("Sorry, I could not understand the audio")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlledTurtlesim()
    try:
        while rclpy.ok():
            node.listen_and_process()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Voice-Controlled Turtlesim Node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
