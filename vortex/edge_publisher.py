import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Header

import cv2 as cv
from cv_bridge import CvBridge

class EdgePublisher(Node):
    def __init__(self):
        super().__init__('edge_publisher')
        self.bridge = CvBridge()

        # Subscribers
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Publishers
        self.vectors_publisher_ = self.create_publisher(Float32MultiArray, '/edge_vectors', 10)
        self.processed_image_publisher = self.create_publisher(Image, '/edge_processed_image', 10)

    def HSL_color_selection(self, image):
        #Convert the input image to HSL
        converted_image = cv.cvtColor(image, cv.COLOR_BGR2HLS)
        
        #White color mask
        lower_threshold = np.uint8([0, 200, 0])
        upper_threshold = np.uint8([255, 255, 255])
        white_mask = cv.inRange(converted_image, lower_threshold, upper_threshold)
        
        #Yellow color mask
        lower_threshold = np.uint8([10, 0, 100])
        upper_threshold = np.uint8([40, 255, 255])
        yellow_mask = cv.inRange(converted_image, lower_threshold, upper_threshold)
        
        #Combine white and yellow masks
        mask = cv.bitwise_or(white_mask, yellow_mask)
        masked_image = cv.bitwise_and(image, image, mask = mask)
        
        return masked_image 

    def region_selection(self, image):
        mask = np.zeros_like(image)   
        #Defining a 3 channel or 1 channel color to fill the mask with depending on the input image
        if len(image.shape) > 2:
            channel_count = image.shape[2]
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255
        #We could have used fixed numbers as the vertices of the polygon,
        #but they will not be applicable to images with different dimesnions.
        rows, cols = image.shape[:2]
        bottom_left  = [cols * 0.1, rows * 0.95]
        top_left     = [cols * 0.4, rows * 0.6]
        bottom_right = [cols * 0.9, rows * 0.95]
        top_right    = [cols * 0.6, rows * 0.6]
        vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
        cv.fillPoly(mask, vertices, ignore_mask_color)
        masked_image = cv.bitwise_and(image, mask)
        return masked_image

    def hough_transform(self, image):
        rho = 1              
        theta = np.pi/180    
        threshold = 20       
        minLineLength = 20   
        maxLineGap = 300
        return cv.HoughLinesP(image, rho = rho, theta = theta, threshold = threshold,
                            minLineLength = minLineLength, maxLineGap = maxLineGap)

    def draw_lines(self, image, lines, color = [255, 0, 0], thickness = 2):
        image = np.copy(image)
        for line in lines:
            for x1,y1,x2,y2 in line:
                cv.line(image, (x1, y1), (x2, y2), color, thickness)
        return image
    
    def average_slope_intercept(self, lines):
        left_lines    = [] 
        left_weights  = [] 
        right_lines   = [] 
        right_weights = [] 
        
        for line in lines:
            for x1, y1, x2, y2 in line:
                if x1 == x2:
                    continue
                slope = (y2 - y1) / (x2 - x1)
                intercept = y1 - (slope * x1)
                length = np.sqrt(((y2 - y1) ** 2) + ((x2 - x1) ** 2))
                if slope < 0:
                    left_lines.append((slope, intercept))
                    left_weights.append((length))
                else:
                    right_lines.append((slope, intercept))
                    right_weights.append((length))
        left_lane  = np.dot(left_weights,  left_lines) / np.sum(left_weights)  if len(left_weights) > 0 else None
        right_lane = np.dot(right_weights, right_lines) / np.sum(right_weights) if len(right_weights) > 0 else None
        return left_lane, right_lane
    
    def pixel_points(self, y1, y2, line):
        if line is None:
            return ((0, 0), (0, 0))
        slope, intercept = line
        x1 = int((y1 - intercept)/slope)
        x2 = int((y2 - intercept)/slope)
        y1 = int(y1)
        y2 = int(y2)
        return ((x1, y1), (x2, y2))
    
    def lane_lines(self, image, lines):
        left_lane, right_lane = self.average_slope_intercept(lines)
        y1 = image.shape[0]
        y2 = y1 * 0.6
        left_line  = self.pixel_points(y1, y2, left_lane)
        right_line = self.pixel_points(y1, y2, right_lane)
        return left_line, right_line

        
    def draw_lane_lines(self, image, lines, color=[255, 0, 0], thickness=12):
        line_image = np.zeros_like(image)
        for line in lines:
            if line is not None:
                cv.line(line_image, *line,  color, thickness)
        return cv.addWeighted(image, 1.0, line_image, 1.0, 0.0)
    
    def image_callback(self, msg):
        image_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        hsl_selected = self.HSL_color_selection(image_frame)
        gray = cv.cvtColor(hsl_selected, cv.COLOR_RGB2GRAY)
        blurred = cv.GaussianBlur(gray, (13, 13), 0)
        edges = cv.Canny(blurred, 50, 150)

        roi = self.region_selection(edges)
        lines = self.hough_transform(roi)

        if lines is not None:
            lane = self.lane_lines(gray, lines)
            result = self.draw_lane_lines(gray, lane)

            flattened_edge_points = [float(coord) for pair in lane for point in pair for coord in point]
            edge_vectors_msg = Float32MultiArray()
            edge_vectors_msg.data = flattened_edge_points
            edge_vectors_header = Header()
            edge_vectors_header.stamp = self.get_clock().now().to_msg()
            edge_vectors_header.frame_id = msg.header.frame_id 
            self.vectors_publisher_.publish(edge_vectors_msg)
        else:
            result = gray

        processed_image_msg = self.bridge.cv2_to_imgmsg(result, encoding='mono8')
        processed_image_msg.header = msg.header
        self.processed_image_publisher.publish(processed_image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EdgePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()