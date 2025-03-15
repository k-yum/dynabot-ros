# Creates node to send pwm signals using cmd_vel to motor driver microcontroller (motor_pwm.py)

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist

class MotorPWMController(Node):
    def __init__(self):
        super().__init__('motor_pwm_controller')

        # Connect to Arduino over USB Serial
        self.serial_port = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        # Convert linear.x to PWM range (0-255)
        speed = int(msg.linear.x * 100)
        turn = int(msg.angular.z * 50)

        left_pwm = speed - turn
        right_pwm = speed + turn

        command = "{} {}\n".format(left_pwm, right_pwm)
        self.serial_port.write(command.encode())

        self.get_logger().info(f'Sent: {command.strip()}')

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorPWMController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
