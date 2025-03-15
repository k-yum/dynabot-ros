# Fixed motor_pwm so that the arduino isn't overloaded

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time
from geometry_msgs.msg import Twist

class MotorPWMController(Node):
    def __init__(self):
        super().__init__('motor_pwm2_controller')

        # Connect to Arduino over USB Serial
        self.serial_port = serial.Serial('/dev/arduino_motor', 115200, timeout=1)
        # Allow time for Arduino to reset and send its initial READY message
        time.sleep(2)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def wait_for_ack(self, timeout=1.0):
        """
        Wait for the Arduino to send a "READY" acknowledgment.
        Returns True if received within the timeout period, otherwise False.
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.serial_port.in_waiting:
                ack = self.serial_port.readline().decode().strip()
                if ack == "READY":
                    return True
            time.sleep(0.01)  # Short delay before checking again
        return False

    def cmd_vel_callback(self, msg):
        # Wait for Arduino acknowledgment before sending a command
        if not self.wait_for_ack(timeout=1.0):
            self.get_logger().info("Arduino not ready; command not sent.")
            return

        # Convert linear.x and angular.z to PWM values
        speed = int(msg.linear.x * 100)
        turn = int(msg.angular.z * 50)
        left_pwm = speed - turn
        right_pwm = speed + turn

        command = "{} {}\n".format(left_pwm, right_pwm)
        self.serial_port.write(command.encode())
        self.serial_port.flush()

        self.get_logger().info(f'Sent: {command.strip()}')

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorPWMController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

