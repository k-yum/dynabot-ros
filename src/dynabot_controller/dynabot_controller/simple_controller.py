#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import numpy as np

class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")

        #declare velocity publisher ,node parameters can be reused by other nodes
        self.declare_parameter("wheel_radius", 0.1905)   
        self.declare_parameter("separation", 0.7747)

        #get parameters
        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("separation").get_parameter_value().double_value
        
        #print parameters
        self.get_logger().info("wheel_radius: %f" % self.wheel_radius_)
        self.get_logger().info("wheel_separation: %f" % self.wheel_separation_)

        #create velocity publisher, published to simple_velocity_controller/commands topic
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray,"simple_velocity_controller/commands",10)
        #create subscriber for joint states
        self.joint_state_sub_ = self.create_subscription(TwistStamped,"dynabot_controller/cmd_vel",self.velCallback,10)

        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2],
                                        [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])
        
        self.get_logger().info("the conversion matrix is %s" % self.speed_conversion_)

    #callback function for velocity subscriber, dynabot_controller/cmd_vel topic
    def velCallback(self,msg):
        # Implements the differential kinematic model
        # Given v and w, calculate the velocities of the wheels
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed) 
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]

        self.wheel_cmd_pub_.publish(wheel_speed_msg)

def main():
    rclpy.init()
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
        