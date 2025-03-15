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
        # Convert color image message to OpenCV image in RGB format
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.process_images()

    def depth_callback(self, msg):
        # Convert depth image message to OpenCV image (keep original scale)
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.process_images()

    def process_images(self):
        # Ensure both images are available
        if self.color_image is None or self.depth_image is None:
            return

        # Work on a copy for visualization (still in RGB)
        output_image = self.color_image.copy()

        # ---------------------------------------------------------------------
        # 1. Optional: Color Thresholding for White Tape
        #    Convert to HLS and create a mask for white pixels
        # ---------------------------------------------------------------------
        hls_image = cv2.cvtColor(self.color_image, cv2.COLOR_RGB2HLS)
        # Define a range for white (tweak these as needed)
        lower_white = np.array([0, 200, 0])      # (H=0, L=200, S=0)
        upper_white = np.array([180, 255, 255])  # (H=180, L=255, S=255)
        mask_white = cv2.inRange(hls_image, lower_white, upper_white)

        # ---------------------------------------------------------------------
        # 2. Edge Detection on the White Mask
        #    Lower thresholds make it more sensitive to faint edges
        # ---------------------------------------------------------------------
        edges = cv2.Canny(mask_white, 20, 60)

        # ---------------------------------------------------------------------
        # 3. (Optional) Dilation to Merge Narrow/Parallel Edges
        # ---------------------------------------------------------------------
        kernel = np.ones((3, 3), np.uint8)
        dilated_edges = cv2.dilate(edges, kernel, iterations=1)

        # ---------------------------------------------------------------------
        # 4. Define a Region of Interest (ROI) Mask
        # ---------------------------------------------------------------------
        height, width = dilated_edges.shape
        mask = np.zeros_like(dilated_edges)
        polygon = np.array([[
            (0, 0),
            (width, 0),
            (width,height),
            (0, height)
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        masked_edges = cv2.bitwise_and(dilated_edges, mask)

        # ---------------------------------------------------------------------
        # Debug: Draw the ROI on the image
        # ---------------------------------------------------------------------
        roi_debug_image = self.color_image.copy()
        cv2.polylines(roi_debug_image, [polygon], isClosed=True, color=(255, 0, 0), thickness=2)
        cv2.imshow("ROI Debug", cv2.cvtColor(roi_debug_image, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)

        # ---------------------------------------------------------------------
        # 5. Hough Transform with Lower Threshold and MinLineLength
        # ---------------------------------------------------------------------
        lines = cv2.HoughLinesP(
            masked_edges,
            rho=1,
            theta=np.pi / 180,
            threshold=10,       # Lower threshold for more line detections
            minLineLength=10,   # Lower for short line segments (e.g., tape)
            maxLineGap=150
        )

        # ---------------------------------------------------------------------
        # 6. Draw Lines That Pass the Depth Check
        # ---------------------------------------------------------------------
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # Calculate midpoint
                mid_x = (x1 + x2) // 2
                mid_y = (y1 + y2) // 2
                # Read depth (assuming alignment)
                depth = self.depth_image[mid_y, mid_x]

                # Adjust these depth limits based on your sensor/environment
                if 0.3 < depth < 5.0:
                    cv2.line(output_image, (x1, y1), (x2, y2), (0, 255, 0), 5)

        # Convert from RGB to BGR for display
        display_image = cv2.cvtColor(output_image, cv2.COLOR_RGB2BGR)
        cv2.imshow("Lane Detection", display_image)
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
