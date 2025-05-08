#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import Imu

imu_pub = None


def imuCallback(imu):
    global imu_pub
    imu.header.frame_id = "base_link_ekf"
    imu_pub.publish(imu)


def main():
    global imu_pub
    rclpy.init()
    node = Node("imu_republisher_node")
    node.get_logger().info("IMU republisher node initialized")
    time.sleep(1)

    imu_pub = node.create_publisher(Imu, "/vectornav/imu", 10)
    imu_sub = node.create_subscription(Imu, "/vectornav/imu", imuCallback, 10)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
