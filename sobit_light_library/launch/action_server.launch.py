import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    arg_robot_name = DeclareLaunchArgument('robot_name', default_value='sobit_light')

    return LaunchDescription([
        arg_robot_name,
        OpaqueFunction(function = launch_gz),
    ])


def launch_gz(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)

    pose_config = os.path.join(
        get_package_share_directory("sobit_light_library"),
        "config",
        "pose_list.yaml",
    )

    joint_action_server_node = Node(
        package="sobit_light_library",
        executable="joint_action_server",
        name="joint_action_server",
        namespace=robot_name,
        parameters=[pose_config],
        output="screen",
    )

    wheel_action_server_node = Node(
        package="sobit_light_library",
        executable="wheel_action_server",
        name="wheel_action_server",
        namespace=robot_name,
        output="screen",
    )


    return [
        joint_action_server_node,
        wheel_action_server_node,
    ]
