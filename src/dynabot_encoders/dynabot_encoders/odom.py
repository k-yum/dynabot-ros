#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros


def euler_to_quaternion(yaw):
   """
   Convert a yaw angle (in radians) into a Quaternion.
   Roll and pitch are assumed to be zero.
   """
   qz = math.sin(yaw / 2.0)
   qw = math.cos(yaw / 2.0)
   return Quaternion(x=0.0, y=0.0, z=qz, w=qw)


class OdometryPublisher(Node):
   def __init__(self):
       super().__init__('encoder_node')
      
       # Parameters (adjust based on your robot)
       self.wheel_radius = 0.1905          # meters
       self.wheelbase = 0.7747             # meters (distance between wheels)
       self.encoder_ticks_per_revolution = 32768  # ticks/rev


       # Initialize pose and velocity state variables
       self.x = 0.0
       self.y = 0.0
       self.th = 0.0
       self.vx = 0.0
       self.vth = 0.0


       # Initialize tick storage
       self.previous_left_ticks = 0
       self.previous_right_ticks = 0
       self.current_left_ticks = 0
       self.current_right_ticks = 0


       # Set the initial time using the node's clock
       self.last_time = self.get_clock().now()


       # Create subscriptions for encoder ticks
       self.create_subscription(Int64, 'left_encoder_ticks', self.left_encoder_callback, 10)
       self.create_subscription(Int64, 'right_encoder_ticks', self.right_encoder_callback, 10)


       # Create an odometry publisher
       self.odom_pub = self.create_publisher(Odometry, 'odom', 50)


       # Create a TF broadcaster for publishing transforms
       self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)


       # Create a timer callback (20 Hz, i.e. every 0.05 sec)
       self.create_timer(0.05, self.timer_callback)


   def left_encoder_callback(self, msg: Int64):
       self.current_left_ticks = msg.data


   def right_encoder_callback(self, msg: Int64):
       self.current_right_ticks = msg.data


   def calculate_odometry(self):
       current_time = self.get_clock().now()
       # Compute delta time in seconds
       dt = (current_time - self.last_time).nanoseconds / 1e9
       dt = max(dt, 0.001)  # Prevent division by zero


       # Compute tick differences
       left_delta_ticks = self.current_left_ticks - self.previous_left_ticks
       right_delta_ticks = self.current_right_ticks - self.previous_right_ticks


       # Update previous tick counts
       self.previous_left_ticks = self.current_left_ticks
       self.previous_right_ticks = self.current_right_ticks


       # Convert tick differences to wheel travel distances (meters)
       left_distance = (left_delta_ticks / self.encoder_ticks_per_revolution) * (2 * math.pi * self.wheel_radius)
       right_distance = (right_delta_ticks / self.encoder_ticks_per_revolution) * (2 * math.pi * self.wheel_radius)


       # Compute the linear displacement and change in orientation
       linear_velocity = (left_distance + right_distance) / 2.0
       angular_velocity = (right_distance - left_distance) / self.wheelbase


       # Apply a low-pass filter to smooth the velocities
       alpha = 0.85
       self.vx = alpha * self.vx + (1 - alpha) * linear_velocity
       self.vth = alpha * self.vth + (1 - alpha) * angular_velocity


       # Update pose (using a mid-point approximation)
       delta_x = self.vx * math.cos(self.th) * dt
       delta_y = self.vx * math.sin(self.th) * dt
       delta_th = self.vth * dt


       self.x += delta_x
       self.y += delta_y
       self.th += delta_th


       self.last_time = current_time


   def publish_odometry(self):
       # Create and populate the Odometry message
       odom = Odometry()
       now = self.get_clock().now().to_msg()
       odom.header.stamp = now
       odom.header.frame_id = "odom"
       odom.child_frame_id = "base_link"
       odom.pose.pose.position.x = self.x
       odom.pose.pose.position.y = self.y
       odom.pose.pose.position.z = 0.0


       # Convert yaw (self.th) to a quaternion using our custom function
       quat = euler_to_quaternion(self.th)
       odom.pose.pose.orientation = quat


       # Set linear and angular velocities
       odom.twist.twist.linear.x = self.vx
       odom.twist.twist.angular.z = self.vth


       self.odom_pub.publish(odom)


       # Publish the transform over TF
       t = TransformStamped()
       t.header.stamp = now
       t.header.frame_id = "odom"
       t.child_frame_id = "base_link"
       t.transform.translation.x = self.x
       t.transform.translation.y = self.y
       t.transform.translation.z = 0.0
       t.transform.rotation = quat
       self.tf_broadcaster.sendTransform(t)


   def timer_callback(self):
       self.calculate_odometry()
       self.publish_odometry()


def main(args=None):
   rclpy.init(args=args)
   node = OdometryPublisher()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()