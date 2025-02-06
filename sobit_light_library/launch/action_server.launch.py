from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = "sobit_light"
    robot_id = 0

    joint_action_server_node = Node(
        package="sobit_light_library",
        executable="joint_action_server",
        name="joint_action_server",
        namespace=robot_name if robot_id == 0 else f"{robot_name}_{robot_id}",
        output="screen",
    )

    wheel_action_server_node = Node(
        package="sobit_light_library",
        executable="wheel_action_server",
        name="wheel_action_server",
        namespace=robot_name if robot_id == 0 else f"{robot_name}_{robot_id}",
        output="screen",
    )


    return LaunchDescription([
        joint_action_server_node,
        wheel_action_server_node,
    ])
