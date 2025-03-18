#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # Start the micro-ROS agent (runs continuously)
    micro_ros_agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/teensy'],
        output='screen'
    )

    # Node for odom.py from package dynabot_encoders
    odom_node = Node(
        package='dynabot_encoders',
        executable='odom.py',
        name='odom_node',
        output='screen'
    )

    # Node for reset_encoders.py from package dynabot_encoders
    reset_encoders_node = Node(
        package='dynabot_encoders',
        executable='reset_encoders.py',
        name='reset_encoders_node',
        output='screen'
    )

    # Delay launching odom and reset_encoders nodes for 3 seconds
    # to allow micro_ros_agent to start up first.
    delayed_nodes = TimerAction(
        period=3.0,
        actions=[odom_node, reset_encoders_node]
    )

    return LaunchDescription([
        micro_ros_agent,
        delayed_nodes
    ])
