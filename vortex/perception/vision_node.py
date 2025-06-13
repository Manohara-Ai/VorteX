import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.get_logger().info("hello")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"{node.get_name()} received Ctrl+C, shutting down...")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        print(f"{node.get_name()} shutdown complete.")

if __name__ == '__main__':
    main()