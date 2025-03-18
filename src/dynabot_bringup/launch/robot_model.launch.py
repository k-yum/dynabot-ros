from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
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
        # Include Robot State Publisher Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rsp_launch_file),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),

        # Joint State Publisher GUI
        ExecuteProcess(
            cmd=['ros2', 'run', 'joint_state_publisher_gui', 'joint_state_publisher_gui'],
            output='screen'
        ),

        # RViz with pre-configured settings
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_file],
            output='screen'
        ),

        # Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file)
        ),

        # Spawn Robot in Gazebo (delayed)
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                        '-topic', 'robot_description',
                        '-entity', 'bot_name'
                    ],
                    output='screen'
                )
            ]
        ),
    ])
