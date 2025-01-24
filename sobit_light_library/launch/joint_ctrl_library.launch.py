import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node



def generate_launch_description():

    robot_name = "sobit_light_1"

    param_node = Node(
        package="sobit_light_library",
        executable="sobit_light_action_server",
        name="sobit_light_action_server",
        namespace=robot_name,
        output="screen",
    )



    return LaunchDescription([
        param_node,
    ])
