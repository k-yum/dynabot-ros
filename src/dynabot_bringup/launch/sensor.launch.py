#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # this_dir → dynabot_bringup/launch
    this_dir = os.path.dirname(__file__)

    # launch encoders and odom with micro-ros
    encoders = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(this_dir, 'encoders.launch.py')
        )
    )

    # launch hokuyo lidar
    hokuyo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(this_dir, 'hokuyo.launch.py')
        )
    )

    return LaunchDescription([
        encoders,
        hokuyo,
    ])
