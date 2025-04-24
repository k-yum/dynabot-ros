from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["--x", "0", "--y", "0", "--z", "0", "--yaw", "0", "--pitch", "0", "--roll", "0", 
                    "--frame-id", "base_link_ekf", "--child-frame-id", "imu_link_ekf"]
    )
    

    
    
    return LaunchDescription(

    )
