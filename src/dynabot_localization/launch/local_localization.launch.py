import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Include the vectornav launch file
    vectornav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('vectornav'), 'launch', 'vectornav.launch.py')
        )
    )

    # Static transform publisher to publish the imu_link_ekf to base_link_ekf
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["--x", "0", "--y", "0", "--z", "0", "--yaw", "0", "--pitch", "0", "--roll", "0", 
                    "--frame-id", "base_link_ekf", "--child-frame-id", "imu_link_ekf"]
    )

    # Robot localization node
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('dynabot_localization'), 'config', 'ekf.yaml')]
    )
    
    # IMU republisher node
    imu_republisher = Node(
        package='dynabot_localization',
        executable='imu_republisher.py',
        name='imu_republisher_node',
        output='screen'
    )

    return LaunchDescription([
        vectornav_launch,
        static_transform_publisher,
        robot_localization,
        imu_republisher,
    ])
