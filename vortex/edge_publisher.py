import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

import cv2 as cv
from cv_bridge import CvBridge

class EdgePublisher(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()

        # Subscribers
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Publishers
        self.vectors_publisher_ = self.create_publisher(Float32MultiArray, '/edge vectors', 10)
        self.data_publisher = self.create_publisher(Float32, '/edge_processed_image', 10)

    def image_callback(self, msg):
        image_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        cv.imshow('image', image_frame)
        cv.waitKey(1)
        
        # Dummy processing (replace this with actual logic)
        error_value = 10.0  # Dummy error value
        vel_msg = Twist()
        vel_msg.linear.x = 0.2  # Dummy forward speed
        vel_msg.angular.z = -0.1  # Dummy turn

        # Publish the values
        self.data_publisher.publish(Float32(data=error_value))
        self.vectors_publisher_.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EdgePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()