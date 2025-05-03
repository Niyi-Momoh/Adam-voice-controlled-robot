import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Publisher to send stop command to /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # QoS profile suitable for LIDAR
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # Subscriber to the /scan topic
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos
        )

        self.threshold_distance = 0.25  # 25 cm threshold
        self.get_logger().info("Obstacle Avoidance Node with directional scan filtering is running!")

    def scan_callback(self, msg):
        ranges = msg.ranges
        front_clear = True
        left_clear = True
        right_clear = True

        def get_min_range(start_deg, end_deg):
            angle_min_deg = math.degrees(msg.angle_min)
            angle_increment_deg = math.degrees(msg.angle_increment)

            start_idx = int((start_deg - angle_min_deg) / angle_increment_deg)
            end_idx = int((end_deg - angle_min_deg) / angle_increment_deg)
            section = ranges[start_idx:end_idx]


            return min([r for r in section if 0.01 < r < 10.0], default=10.0)

        front_dist = get_min_range(0, 10)
        left_dist = get_min_range(60, 100)
        right_dist = get_min_range(260, 300)
        min_distance = get_min_range(0, 360)

        if front_dist < self.threshold_distance:
            front_clear = False
        if left_dist < self.threshold_distance:
            left_clear = False
        if right_dist < self.threshold_distance:
            right_clear = False

        # Log results
        self.get_logger().info(f"[LIDAR] Front: {front_dist:.2f}m | Left: {left_dist:.2f}m | Right: {right_dist:.2f}m")
        

        # Stop robot if any direction is too close
        if min_distance < 0.25: #not front_clear:
            self.get_logger().warn("Obstacle ahead! Stopping robot.")
            self.stop_robot()
            self.get_logger().warn(f"[LIDAR] Front: {front_dist:.2f}m | Left: {left_dist:.2f}m | Right: {right_dist:.2f}m")

    def stop_robot(self):
        """ Publishes zero velocities to stop the robot """
        twist = Twist()
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
