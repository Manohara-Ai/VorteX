import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json 
import numpy as np
from .edge_detectors import process_image
from .depth_map import get_depth_map

class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.get_logger().info("Vision node has started perception")

        self.bridge = CvBridge()

        # Subscriber to raw camera feed
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publishers for output images
        self.lane_pub = self.create_publisher(Image, '/vision/lane', 10)
        self.depth_pub = self.create_publisher(Image, '/vision/depth', 10)
        self.depth_raw_pub = self.create_publisher(Image, '/vision/depth_raw', 10)  # Optional: raw float32 depth
        self.lane_coords_pub = self.create_publisher(String, '/vision/lane_coords', 10)

    def image_callback(self, msg):
        try:
            # Convert image from ROS to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Lane Detection
            extrapolated, output = process_image(frame)
            lane_msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
            lane_msg.header = msg.header  # Maintain timestamp and frame_id
            self.lane_pub.publish(lane_msg)

            # Publish extrapolated lane coordinates (if available)
            if extrapolated and isinstance(extrapolated[0], list):
                line_coords = extrapolated[0]  # List of [x1, y1, x2, y2]
                serializable_coords = [list(map(int, line)) for line in line_coords]

                lane_str_msg = String()  # 🛠 Renamed to avoid conflict
                lane_str_msg.data = json.dumps(serializable_coords)
                self.lane_coords_pub.publish(lane_str_msg)

            # Depth Map Generation
            depth_map = get_depth_map(frame)

            if depth_map is None or not np.isfinite(depth_map).any():
                self.get_logger().warn("Depth map is empty or contains invalid values.")
                return

            # Publish raw depth as mono16/float32 image (optional, for real depth use)
            depth_raw = depth_map.astype(np.float32)
            depth_raw_msg = self.bridge.cv2_to_imgmsg(depth_raw, encoding='32FC1')
            depth_raw_msg.header = msg.header
            self.depth_raw_pub.publish(depth_raw_msg)

            # Normalized colored depth for visualization
            vis = (depth_raw - depth_raw.min()) / (depth_raw.max() - depth_raw.min() + 1e-8)
            vis_color = cv2.applyColorMap((vis * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)

            depth_vis_msg = self.bridge.cv2_to_imgmsg(vis_color, encoding='bgr8')
            depth_vis_msg.header = msg.header
            self.depth_pub.publish(depth_vis_msg)

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