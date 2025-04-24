#! /usr/bin/env python3
"""
Launch file for Hokuyo UTM-30LX LiDAR with static TF to bridge `laser` → `laser_frame`.
Keeps driver publishing on `laser` (the Humble default) and lets your URDF continue using `laser_frame`.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Hokuyo driver: publishes scan on frame `laser`
    hokuyo_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='hokuyo_driver',
        output='screen',
        parameters=[{
            'serial_port': '/dev/serial/by-id/usb-Hokuyo_Data_Flex_for_USB_URG-Series_USB_Driver-if00',  # adjust if needed
            'serial_baudrate': 115200, #921600
            'frame_id': 'laser',
            'cluster': 1,
            'angle_min': -1.5708,
            'angle_max': 1.5708,
            'skip': 0,
            'time_offset': 0.0,
            'range_min': 0.1,
            'range_max': 30.0
        }]
    )

    # Static TF: bridge `laser` → `laser_frame` so URDF and other nodes see the expected link
    # static_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_laser_tf',
    #     output='screen',
    #     arguments=[
    #         '0', '0', '0',    # x y z
    #         '0', '0', '0',    # roll pitch yaw
    #         'laser',          # parent frame (what driver publishes)
    #         'laser_frame'     # child frame (what your URDF expects)
    #     ]
    # )

    return LaunchDescription([
        hokuyo_node,
        #static_tf,
    ])
