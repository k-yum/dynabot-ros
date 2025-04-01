import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    dynabot_bringup_dir = get_package_share_directory('dynabot_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    nav2_params_file = os.path.join(dynabot_bringup_dir, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(dynabot_bringup_dir, 'config', 'slam_toolbox_online_async.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Enforce the use_sim_time config globally
        SetLaunchConfiguration('use_sim_time', LaunchConfiguration('use_sim_time')),

        # Launch SLAM Toolbox (online mode)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': slam_params_file
            }.items()
        ),

        # Launch core Nav2 stack (no map_server or amcl)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': nav2_params_file
            }.items()
        )
    ])
