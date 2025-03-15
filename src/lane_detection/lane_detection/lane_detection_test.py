#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None

        # Subscribe to color and depth image topics
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)

    def color_callback(self, msg):
        # Convert color image message to OpenCV image
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_images()

    def depth_callback(self, msg):
        # Convert depth image message to OpenCV image (keep original scale)
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.process_images()

    def process_images(self):
        # Ensure both images are available
        if self.color_image is None or self.depth_image is None:
            return

        # Work on a copy for visualization
        output_image = self.color_image.copy()

        # Convert color image to grayscale and apply Gaussian blur
        gray = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)

        # Use Canny edge detection
        edges = cv2.Canny(blurred, 30, 100)

        # Define a region of interest (ROI) mask – usually the lower half of the image
        height, width = edges.shape
        mask = np.zeros_like(edges)
        polygon = np.array([[
            (0, height),
            (width, height),
            (int(width * 0.55), int(height * 0.6)),
            (int(width * 0.45), int(height * 0.6))
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        masked_edges = cv2.bitwise_and(edges, mask)

        # Use Hough transform to detect lines from the edge map
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, threshold=50, minLineLength=50, maxLineGap=150)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # Calculate the midpoint of the line segment
                mid_x = int((x1 + x2) / 2)
                mid_y = int((y1 + y2) / 2)
                # Read the depth value at the midpoint (assumes depth image is aligned)
                depth = self.depth_image[mid_y, mid_x]
                # Filter the line based on depth (e.g., only consider lines between 0.5m and 5.0m)
                if 0.5 < depth < 5.0:
                    cv2.line(output_image, (x1, y1), (x2, y2), (0, 255, 0), 5)

        # Display the result in an OpenCV window
        cv2.imshow("Lane Detection", output_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    lane_detector = LaneDetector()
    rclpy.spin(lane_detector)
    lane_detector.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
