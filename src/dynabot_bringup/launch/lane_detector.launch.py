#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Launch the RealSense camera node
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        output='screen'
    )

    # Launch the lane detection node from your dynabot_camera package
    lane_detection_node = Node(
        package='dynabot_camera',
        executable='lane_detection',  
        output='screen'
    )

    return LaunchDescription([
        realsense_node,
        lane_detection_node,
    ])
