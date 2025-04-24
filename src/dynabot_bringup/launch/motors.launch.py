#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Node that converts joystick input to Twist commands.
    joystick_control_node = Node(
        package='joystick_control',
        executable='joystick_to_cmd_velocity_node',
        name='joystick_to_cmd_velocity_node',
        output='screen'
    )

    # Node that publishes joystick messages.
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    # Node that converts Twist commands into Arduino-compatible messages.
    motor_control_node = Node(
        package='motor_control',
        executable='cmd_vel_to_arduino_node',
        name='cmd_vel_to_arduino_node',
        output='screen'
    )

    # Node that sends motor PWM signals to the motors.
    motor_test_node = Node(
        package='motor_test',
        executable='motor_pwm',
        name='motor_pwm',
        output='screen'
    )

    return LaunchDescription([
        joystick_control_node,
        joy_node,
        motor_control_node,
        motor_test_node,
    ])
