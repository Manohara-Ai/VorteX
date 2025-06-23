import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from .edge_detectors import process_image
from .depth_map import get_depth_map

class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.get_logger().info("Vision node has started perception")
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.subscription

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            extrapolated, output = process_image(frame) 
            depth_map = get_depth_map(frame)

            cv2.imshow("Lane Detection", output)
            depth_vis = (depth_map - depth_map.min()) / (depth_map.max() - depth_map.min())
            depth_color = cv2.applyColorMap((depth_vis * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)
            cv2.imshow("Depth Map", depth_color)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()

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