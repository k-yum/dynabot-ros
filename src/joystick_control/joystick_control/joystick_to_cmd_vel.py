#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy 
from geometry_msgs.msg import Twist

class MyNode(Node):

    def __init__(self):
        super().__init__("joystick_to_cmd_vel_node")

        # Subscribe to the /joy topic
        self.joy_subscriber = self.create_subscription(
            Joy,            # Message type
            '/joy',         # Topic name
            self.joy_callback,  # Callback function
            10              # QoS history depth
        )
        self.get_logger().info("Subscribed to /joy")

        # Publisher to /cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Publisher for /cmd_vel created")

    def joy_callback(self, msg: Joy):
        # Process the received Joy message
        # Process the incoming Joy message and convert it into a Twist command
        twist = Twist()
        # Example: Map joystick axes to linear and angular velocities
        if not any(msg.buttons):
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            #self.get_logger().info("Safety engaged: No button pressed. Robot is stopped.")
        else:
            twist.linear.x = msg.axes[1]   # forward/backward movement
            twist.angular.z = msg.axes[0]  # rotation

        # Publish the command
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f'Published cmd_vel: {twist}')

        #self.get_logger().info(f"Received joy message: {msg}")

        



def main(args=None):
    rclpy.init(args=args)
    #put main code in here
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()