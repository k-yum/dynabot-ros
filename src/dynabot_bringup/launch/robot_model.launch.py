import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Paths to needed files
    rsp_launch_file = os.path.join(
        get_package_share_directory('dynabot_description'),
        'launch',
        'rsp.launch.py'
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('dynabot_description'),
        'config',
        'dynabot_view.rviz'
    )

    gazebo_launch_file = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time (Gazebo) or real time'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rsp_launch_file),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments={'verbose': 'true'}.items()
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_robot',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'bot_name',
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        )
    ])
