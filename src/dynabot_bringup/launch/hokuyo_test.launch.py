#! /usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='urg_node',
            executable='urg_node_driver',
            name='hokuyo_driver',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyACM2',   # Adjust this as needed
                'serial_baudrate': 115200,        # Typical baudrate for many Hokuyo models
                'frame_id': 'laser_link',         # Frame id used in RViz2
                'angle_min': -1.5708,             # Example: -90° in radians
                'angle_max': 1.5708,              # Example: +90° in radians
                'range_min': 0.1,                 # Minimum measurable range
                'range_max': 30.0                 # Maximum measurable range
            }]
        )
    ])
