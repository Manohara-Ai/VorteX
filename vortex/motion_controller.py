import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.data_subscription = self.create_subscription(
            Float32, '/vision_data', self.data_callback, 10)

        self.vel_subscription = self.create_subscription(
            Twist, '/cmd_vel', self.vel_callback, 10)

        self.latest_twist = Twist()  # Store latest velocity

    def data_callback(self, msg):
        self.get_logger().info(f"Received vision data: {msg.data}")

    def vel_callback(self, msg):
        self.latest_twist = msg
        self.get_logger().info(f"Received velocity: linear_x={msg.linear.x}, angular_z={msg.angular.z}")

        # Publish velocity command to move robot
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
