import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the package share directory, variable is used to get the path to the model file
    dynabot_desription_dir = get_package_share_directory("dynabot_description")

    # Declare the model argument, it is used to get the path to the model file
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(dynabot_desription_dir,"description", "dynabot_gazebo.xacro"),
        description="path to the model file"
    )
    
    # Declare the robot description parameter, it is used to get the path to the model file
    robot_description = ParameterValue(Command([
        'xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # Declare the robot state publisher node    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # Declare the gazebo resource path environment variable
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(dynabot_desription_dir).parent.resolve())
        ]
    )

    # Declare the gazebo launch file
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
            ),
            launch_arguments=[
                ("gz_args",[" -v 4"," -r"," empty.sdf"])
            ]
        )    
    
    # Declare the spawn entity node
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "/robot_description", "-name", "dynabot"]
        )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        gazebo_resource_path,
        gazebo,
        spawn_entity
    ])