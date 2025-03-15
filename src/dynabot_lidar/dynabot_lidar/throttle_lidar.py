import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ThrottleNode(Node):
    def __init__(self):
        super().__init__('throttle_node')
        # Subscribe to the original topic
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        # Publisher for the throttled output
        self.publisher_ = self.create_publisher(LaserScan, 'throttled_scan', 10)
        # Cache for the latest message
        self.latest_msg = None
        # Create a timer that triggers at 10 Hz (0.1 sec period)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Throttle node started, publishing at 10 Hz")

    def listener_callback(self, msg):
        # Save the latest message from the subscription
        self.latest_msg = msg

    def timer_callback(self):
        # If a message is available, publish it
        if self.latest_msg is not None:
            self.publisher_.publish(self.latest_msg)
            self.get_logger().info("Published throttled message")
            # Optionally, reset the cache if you only want to publish new data
            self.latest_msg = None

def main(args=None):
    rclpy.init(args=args)
    node = ThrottleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
