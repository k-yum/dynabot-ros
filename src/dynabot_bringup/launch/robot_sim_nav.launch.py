import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the share directory of the nav2_bringup package.
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    # Path to your nav2_params.yaml file.
    default_params_file = os.path.join(
        get_package_share_directory('dynabot_bringup'),
        'config', 'nav2_params.yaml'
    )
    # Provide a default map file. If you don't have one, use an empty string.
    default_map_file = ''

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the Nav2 parameters file to use'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map_file,
            description='Full path to map yaml file to load (can be empty if not used)'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch_file),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map': LaunchConfiguration('map')
            }.items()
        )
    ])
