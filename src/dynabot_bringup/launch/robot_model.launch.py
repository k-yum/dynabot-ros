import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

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
        # Include Robot State Publisher Launch with use_sim_time
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rsp_launch_file),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),

        # Launch Joint State Publisher GUI as a Node with sim time
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # Launch RViz2 as a Node with sim time
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': True}]
        ),

        # Include Gazebo Launch (Gazebo publishes /clock automatically)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments={'verbose': 'true'}.items()
        ),

        # Spawn Robot using spawn_entity.py (this node typically doesn't use sim_time)
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