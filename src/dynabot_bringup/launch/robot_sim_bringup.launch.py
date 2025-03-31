import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get directories
    bringup_dir = get_package_share_directory('dynabot_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Paths to existing launch files
    robot_model_launch = os.path.join(bringup_dir, 'launch', 'robot_model.launch.py')
    nav2_launch = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
    nav2_params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([

        # Launch Gazebo + robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_model_launch),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),

        # Delay Nav2 startup by 5 seconds (allows Gazebo to load robot)
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch),
                    launch_arguments={
                        'params_file': nav2_params_file,
                        'use_sim_time': 'true',
                        'map': ''  # No prebuilt map for IGVC
                    }.items()
                )
            ]
        )
    ])
