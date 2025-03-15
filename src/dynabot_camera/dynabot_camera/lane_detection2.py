#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

##############################################################################
# 1. COLOR THRESHOLD FOR WHITE
##############################################################################
def color_threshold_white(image_bgr):
    """
    Convert the BGR image to HLS and create a mask for white.
    Adjust the HLS range as needed for your lighting/floor conditions.
    """
    image_hls = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HLS)
    # Define a range that isolates "bright white" pixels
    # Lower the L channel if your tape is not super bright compared to the floor.
    lower_white = np.array([0, 160, 0], dtype=np.uint8)  
    upper_white = np.array([180, 255, 255], dtype=np.uint8)
    mask_white = cv2.inRange(image_hls, lower_white, upper_white)
    return mask_white

##############################################################################
# 2. OPTIONAL MORPHOLOGICAL OPERATIONS
##############################################################################
def clean_mask(mask):
    """
    Remove noise and fill small holes in the white mask using morphological ops.
    """
    kernel = np.ones((3, 3), np.uint8)
    # Remove small white specks
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    # Fill small black holes in the tape region
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask

##############################################################################
# 3. (OPTIONAL) REGION OF INTEREST
##############################################################################
def region_selection(image):
    """
    Create a mask that focuses on the bottom portion of the frame (where tape might be).
    If your tape can appear anywhere, skip or adjust the polygon.
    """
    mask = np.zeros_like(image)
    if len(image.shape) > 2:
        ignore_mask_color = (255,) * image.shape[2]
    else:
        ignore_mask_color = 255

    rows, cols = image.shape[:2]
    # Example polygon covering bottom ~70% of the image
    polygon = np.array([[
        (0, rows),
        (cols, rows),
        (cols, int(rows * 0.3)),
        (0, int(rows * 0.3))
    ]], dtype=np.int32)

    cv2.fillPoly(mask, polygon, ignore_mask_color)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image

##############################################################################
# 4. HOUGH TRANSFORM
##############################################################################
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

##############################################################################
# 5. LINE FILTERING
##############################################################################
def filter_lines(lines):
    """
    Filter out lines that don't match our expectations:
    - Minimum length
    - Approximate orientation (if the tape is near-vertical or near-horizontal)
    Adjust as needed for your tape layout.
    """
    if lines is None:
        return None

    filtered = []
    min_length = 50       # Minimum pixel length of a valid tape line
    slope_threshold = 2.0 # If tape is near-vertical, slope's absolute value > 2

    for line in lines:
        x1, y1, x2, y2 = line[0]
        dx = x2 - x1
        dy = y2 - y1
        length = np.sqrt(dx*dx + dy*dy)
        
        # 1. Filter by length
        if length < min_length:
            continue
        
        # 2. If near-vertical, slope is large in magnitude
        if dx == 0:
            # Perfectly vertical line
            filtered.append(line)
            continue
        slope = abs(dy / dx)
        if slope < slope_threshold:
            # If you only want near-vertical lines, skip lines that aren't steep
            continue
        
        filtered.append(line)

    return filtered if filtered else None

##############################################################################
# 6. DRAW LINES
##############################################################################
def draw_all_hough_lines(color_image, lines, color=(255, 0, 0), thickness=5):
    """
    Draw the Hough line segments directly on top of the color image.
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

##############################################################################
# 7. MAIN PIPELINE
##############################################################################
def frame_processor(image_bgr):
    """
    Process each frame to detect white tape as lines.
    1. White color threshold
    2. Morphological cleanup
    3. Edge detection
    4. (Optional) Region of Interest
    5. Hough transform
    6. Filter lines
    7. Draw lines
    """
    # 1. Threshold for white
    mask_white = color_threshold_white(image_bgr)

    # 2. Morphological cleanup
    mask_white = clean_mask(mask_white)

    # 3. Edge detection
    edges = cv2.Canny(mask_white, 20, 60)

    # 4. Region of Interest (comment out if you don't need it)
    roi_edges = region_selection(edges)

    # 5. Hough transform
    lines = hough_transform(roi_edges)

    # 6. Filter lines (optional, but helps remove spurious lines)
    lines = filter_lines(lines)

    # 7. Draw lines
    output = draw_all_hough_lines(image_bgr, lines)
    return output

##############################################################################
# ROS2 NODE
##############################################################################
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
        cv2.imshow("Tape Detection", processed_image)
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
