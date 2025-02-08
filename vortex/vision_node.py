import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from cv_bridge import CvBridge

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()

        # Subscribers
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # Publishers
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.data_publisher = self.create_publisher(Float32, '/vision_data', 10)

    def image_callback(self, msg):
        self.get_logger().info("Received image frame.")

        # Dummy processing (replace this with actual logic)
        error_value = 10.0  # Dummy error value
        vel_msg = Twist()
        vel_msg.linear.x = 0.2  # Dummy forward speed
        vel_msg.angular.z = -0.1  # Dummy turn

        # Publish dummy values
        self.data_publisher.publish(Float32(data=error_value))
        self.vel_publisher.publish(vel_msg)
        self.get_logger().info(f"Published: error={error_value}, linear_x={vel_msg.linear.x}, angular_z={vel_msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
