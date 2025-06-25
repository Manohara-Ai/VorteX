import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
import numpy as np

class MyNode(Node):
    def __init__(self):
        super().__init__("lidar_node")
        self.get_logger().info("Lidar node has started mapping")

        self.bridge = CvBridge()
        self.latest_lane_coords = None

        # Subscriber to lane coordinates
        self.lane_coords_sub = self.create_subscription(
            String,
            '/vision/lane_coords',
            self.lane_coords_callback,
            10
        )

        # Subscriber to depth map
        self.depth_sub = self.create_subscription(
            Image,
            '/vision/depth_raw',
            self.image_callback,
            10
        )

    def lane_coords_callback(self, msg):
        try:
            self.latest_lane_coords = json.loads(msg.data)  # List of [x1, y1, x2, y2]
        except Exception as e:
            self.get_logger().error(f"Failed to process lane coordinates: {e}")

    def image_callback(self, msg):
        try:
            if self.latest_lane_coords is None:
                self.get_logger().warn("No lane coordinates yet, skipping depth crop")
                return

            # Convert ROS image to OpenCV format
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

            # Convert lane line endpoints to points
            points = []
            for x1, y1, x2, y2 in self.latest_lane_coords:
                points.append((x1, y1))
                points.append((x2, y2))

            points = np.array(points)
            x_min, y_min = np.min(points, axis=0)
            x_max, y_max = np.max(points, axis=0)

            # Clip to image boundaries
            height, width = depth_image.shape
            x_min = max(0, int(x_min))
            x_max = min(width, int(x_max))
            y_min = max(0, int(y_min))
            y_max = min(height, int(y_max))

            # Crop the region
            cropped = depth_image[y_min:y_max, x_min:x_max]

            if cropped.size == 0:
                self.get_logger().warn("Cropped image is empty.")
                return

            # Normalize and convert to displayable image
            vis = (cropped - cropped.min()) / (cropped.max() - cropped.min() + 1e-8)
            vis_color = cv2.applyColorMap((vis * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)

            cv2.imshow("Cropped Lane Depth", vis_color)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")

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
