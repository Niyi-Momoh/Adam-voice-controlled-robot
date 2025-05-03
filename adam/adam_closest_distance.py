import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import speech_recognition as sr
import math


class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)

        self.timer = self.create_timer(0.8, self.listen_for_command)

        self.front_clear = True
        self.left_clear = True
        self.right_clear = True

        self.get_logger().info("Voice Control Node with Directional Obstacle Avoidance started")

    def listen_for_command(self):
        recognizer = sr.Recognizer()

        try:
            with sr.Microphone() as source:
                self.get_logger().info("Listening for command...")
                recognizer.adjust_for_ambient_noise(source)
                audio = recognizer.listen(source, timeout=3)
                command = recognizer.recognize_google(audio).lower()
                self.get_logger().info(f"Recognized command: {command}")
                self.handle_command(command)

        except sr.WaitTimeoutError:
            self.get_logger().warn("Listening timed out")
        except sr.UnknownValueError:
            self.get_logger().warn("Could not understand audio")
        except sr.RequestError as e:
            self.get_logger().error(f"Speech recognition error: {e}")

    def handle_command(self, command):
        twist = Twist()

        if "forward" in command:
            if self.front_clear:
                twist.linear.x = 0.2
                self.get_logger().info("Moving forward")
            else:
                self.get_logger().warn("Front is blocked!")

        elif "reverse" in command or "backward" in command:
            twist.linear.x = -0.2
            self.get_logger().info("Reversing")

        elif "left" in command:
            if self.left_clear:
                twist.angular.z = 0.3
                self.get_logger().info("Turning left")
            else:
                self.get_logger().warn("Left is blocked!")

        elif "right" in command:
            if self.right_clear:
                twist.angular.z = -0.3
                self.get_logger().info("Turning right")
            else:
                self.get_logger().warn("Right is blocked!")

        elif "stop" in command:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Robot stopped")

        else:
            self.get_logger().info("Unknown command")

        self.publisher.publish(twist)

    def scan_callback(self, msg):
        ranges = msg.ranges

        self.front_clear = True
        self.left_clear = True
        self.right_clear = True

        # Safe bounds
        front_center = len(ranges) // 2
        front_min = float('inf')
        for i in range(max(0, front_center - 5), min(len(ranges), front_center + 5)):
            if 0.0 < ranges[i] < front_min:
                front_min = ranges[i]
        if front_min <= 0.25:
            self.front_clear = False

        left_min = float('inf')
        for i in range(90, min(len(ranges), 110)):
            if 0.0 < ranges[i] < left_min:
                left_min = ranges[i]
        if left_min <= 0.25:
            self.left_clear = False

        right_min = float('inf')
        for i in range(250, min(len(ranges), 270)):
            if 0.0 < ranges[i] < right_min:
                right_min = ranges[i]
        if right_min <= 0.25:
            self.right_clear = False
    
    """def lidar_callback(self, msg: LaserScan):
        min_distance = min([r for r in msg.ranges if not self._is_invalid(r)], default=float('inf'))

        if min_distance < 0.25:
            self.get_logger().warn(f"[LIDAR] Obstacle too close: {min_distance:.2f}m - Robot should stop.")
        else:
            self.get_logger().info(f"[LIDAR] Closest object: {min_distance:.2f}m")"""


def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()