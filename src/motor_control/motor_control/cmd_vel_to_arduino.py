#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time



class MyNode(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_arduino_node")

        # Subscribe to the /cmd_vel topic
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,            # Message type
            '/cmd_vel',         # Topic name
            self.cmd_vel_callback,  # Callback function
            10              # QoS history depth
        )
        self.get_logger().info("Subscribed to /cmd_vel")
        # Open a serial connection to the Arduino (adjust the port as needed)
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            time.sleep(2)  # Give Arduino time to reset
            self.get_logger().info("Serial connection established with Arduino")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")




    def cmd_vel_callback(self, msg: Twist):
        #this will interpret the cmd_vel info it recieves
        # Create a simple CSV formatted string: "linear,angular\n"
        '''command_str = f"{msg.linear.x},{msg.angular.z}\n"
        try:
            self.serial_port.write(command_str.encode('utf-8'))
            self.get_logger().info(f"Sent command: {command_str.strip()}")
        except Exception as e:
            self.get_logger().error(f"Error writing to serial: {e}") '''
        



#This stuff should be in all the nodes you create
def main(args=None):
    rclpy.init(args=args)
    #put main code in here
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
