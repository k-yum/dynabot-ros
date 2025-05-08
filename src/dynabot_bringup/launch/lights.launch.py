#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Node that bridges to the Arduino via rosserial
    # rosserial_node = Node(
    #     package='rosserial_python',
    #     executable='serial_node',
    #     name='rosserial_node',
    #     output='screen',
    #     parameters=[{
    #         'port': '/dev/serial/by-id/usb-Teensyduino_USB_Serial_16896960-if00',
    #         'baud': 115200
    #     }]
    # )

    # Node that runs your light controller script
    light_controller_node = Node(
        package='dynabot_encoders',
        executable='lights',
        name='light_controller_node',
        output='screen'
    )

    return LaunchDescription([
        # rosserial_node,
        light_controller_node,
    ])
