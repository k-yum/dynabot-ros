#!/usr/bin/env python3

""" Launch camera by running this in bash (this also tweaks the autoexposure):
ros2 run realsense2_camera realsense2_camera_node   --ros-args   -p enable_sync:=true   -p rgb_camera.enable_auto_exposure:=false   -p rgb_camera.exposure:=156   -p rgb_camera.gain:=16
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def color_threshold_white(image_bgr):
    """
    Convert the BGR image to HLS and create a mask for white.
    Adjust the HLS range as needed for your lighting/floor conditions.
    """
    image_hls = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HLS)
    # Define a range that isolates "bright white" pixels
    lower_white = np.array([0, 200, 0], dtype=np.uint8)
    upper_white = np.array([180, 255, 255], dtype=np.uint8)
    mask_white = cv2.inRange(image_hls, lower_white, upper_white)
    return mask_white

def region_selection(image):
    """
    (Optional) Create a mask that focuses on a 'road area'.
    If your tape is near the bottom of the image, adjust or remove this.
    """
    mask = np.zeros_like(image)
    if len(image.shape) > 2:
        channel_count = image.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    rows, cols = image.shape[:2]
    # Polygon that covers most of the bottom half
    bottom_left  = [int(cols * 0.0), int(rows * 1.0)]
    top_left     = [int(cols * 0.0), int(rows * 0.5)]
    top_right    = [int(cols * 1.0), int(rows * 0.5)]
    bottom_right = [int(cols * 1.0), int(rows * 1.0)]
    vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image

def hough_transform(binary_image):
    """
    Apply Hough Transform to detect line segments in a binary (edge) image.
    """
    rho = 1
    theta = np.pi / 180
    threshold = 20
    min_line_length = 20
    max_line_gap = 500
    lines = cv2.HoughLinesP(
        binary_image,
        rho=rho,
        theta=theta,
        threshold=threshold,
        minLineLength=min_line_length,
        maxLineGap=max_line_gap
    )
    return lines

def draw_all_hough_lines(color_image, lines, color=(255, 0, 0), thickness=5):
    """
    Draw *all* Hough line segments directly, ignoring slope-based lane logic.
    """
    if lines is None:
        return color_image  # No lines found, return original
    overlay = color_image.copy()
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(overlay, (x1, y1), (x2, y2), color, thickness)
    # Blend the overlay with the original
    output = cv2.addWeighted(color_image, 1.0, overlay, 0.8, 0.0)
    return output

def frame_processor(image_bgr):
    """
    Process each frame to detect white tape as lines.
    """
    # 1. Threshold for white color
    mask_white = color_threshold_white(image_bgr)

    # 2. (Optional) Blur or morph the mask if needed
    # mask_white = cv2.GaussianBlur(mask_white, (3, 3), 0)

    # 3. Edge detection on the mask
    edges = cv2.Canny(mask_white, 20, 60)

    # 4. (Optional) Region of Interest
    roi_edges = region_selection(edges)

    # 5. Hough transform
    lines = hough_transform(roi_edges)

    # 6. Draw the detected line segments on the original image
    output = draw_all_hough_lines(image_bgr, lines)
    return output

class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector_node')
        self.bridge = CvBridge()
        # Subscribe to the camera image topic (change to your camera topic if needed)
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def image_callback(self, msg):
        try:
            # Convert the ROS2 image message to an OpenCV BGR image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # Process the image to detect white tape
        processed_image = frame_processor(cv_image)
        
        # Display the processed image in an OpenCV window (for testing)
        cv2.imshow("Lane (Tape) Detection", processed_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down lane_detector_node.")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
