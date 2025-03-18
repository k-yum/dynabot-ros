#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from rclpy.qos import QoSProfile, DurabilityPolicy

class ResetEncodersNode(Node):
    def __init__(self):
        super().__init__('reset_encoders_node')
        # Setup QoS to mimic "latching" in ROS1 (transient local durability)
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        # Create the publisher with the custom QoS settings
        self.publisher = self.create_publisher(Empty, '/reset_encoders', qos_profile)
        
        # Create a timer that waits 3 seconds before publishing the message
        self.timer = self.create_timer(3.0, self.publish_message)

    def publish_message(self):
        msg = Empty()
        self.publisher.publish(msg)
        self.get_logger().info("Published reset message to /reset_encoders. Shutting down.")
        # Shutdown the node after publishing
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ResetEncodersNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
