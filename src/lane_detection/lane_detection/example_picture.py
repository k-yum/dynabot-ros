#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np

def main(args=None):
    # Create a blank black image (simulate a scene)
    image = np.zeros((200, 300, 3), dtype=np.uint8)

    # Draw a white rectangle (simulate a white lane or crosswalk stripe)
    # The rectangle is white (255, 255, 255) on a dark background.
    cv2.rectangle(image, (100, 150), (200, 170), (255, 255, 255), -1)

    # Convert the image from BGR (default in OpenCV) to HLS
    hls_image = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)

    # Define white thresholds in HLS space
    lower_white = np.array([0, 200, 0])
    upper_white = np.array([180, 255, 255])

    # Create the white mask: pixels within the threshold become 255 (white), others become 0 (black)
    mask_white = cv2.inRange(hls_image, lower_white, upper_white)

    # Display the original image and the white mask
    cv2.imshow("Original Image", image)
    cv2.imshow("White Mask", mask_white)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
