import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    robot_name = 'sobit_light_1'

    joint_node = Node(
        package="sobit_light_library_python",
        executable="sobit_light_joint_controller",
        name="joint_controller_test",
        namespace=robot_name,
        output="screen",
    )

    return LaunchDescription([
        joint_node,
    ])
