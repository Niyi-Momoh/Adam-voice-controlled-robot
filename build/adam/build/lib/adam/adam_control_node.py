import rclpy
from rclpy.node import Node

class CarControlNode(Node):
    def __init__(self):
        super().__init__('car_control_node')
        self.get_logger().info("Car Control Node is running!")

def main(args=None):
    rclpy.init(args=args)
    node = CarControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
