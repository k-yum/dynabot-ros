import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    bringup_dir = get_package_share_directory('dynabot_bringup')
    robot_model_launch = os.path.join(bringup_dir, 'launch', 'robot_model.launch.py')
    robot_nav_launch = os.path.join(bringup_dir, 'launch', 'robot_navigation.launch.py')
    #robot_sim_launch = os.path.join(bringup_dir, 'launch', 'robot_sim_bringup.launch.py')
    robot_ekf_launch = os.path.join(bringup_dir, 'launch', 'robot_ekf.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_model_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_nav_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(robot_sim_launch),
        #    launch_arguments={'use_sim_time': use_sim_time}.items()
        #),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_ekf_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        )
    ])
