#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class LightController(Node):
    def __init__(self):
        super().__init__('light_controller')
        # Publisher on /led
        self.led_pub = self.create_publisher(Empty, '/led', 10)
        # Subscription to /cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

    def cmd_vel_callback(self, msg: Twist):
        if msg.linear.x != 0.0:
            self.led_pub.publish(Empty())
            self.get_logger().info('Flashing LED (published to /led)')

def main(args=None):
    rclpy.init(args=args)
    node = LightController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
