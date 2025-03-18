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
                'serial_port': '/dev/serial/by-id/usb-Hokuyo_Data_Flex_for_USB_URG-Series_USB_Driver-if00',   # Adjust this as needed
                'serial_baudrate': 115200,        # Typical baudrate for many Hokuyo models
                'frame_id': 'laser',         # Frame id used in RViz2
                'cluster' : 1,
                'angle_min': -1.5708,             # Example: -90° in radians
                'angle_max': 1.5708,              # Example: +90° in radians
                'skip' : 0,
                'time_offset' : 0.0,
                'range_min': 0.1,                 # Minimum measurable range
                'range_max': 30.0                 # Maximum measurable range
            }]
        )
    ])
