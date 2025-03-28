from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = "sobit_light"
    robot_id = 0

    wheel_linear_action_client_node = Node(
        package="sobit_light_library",
        executable="wheel_linear_action_client",
        name="wheel_linear_action_client",
        namespace=robot_name if robot_id == 0 else f"{robot_name}_{robot_id}",
        output="screen",
    )
    wheel_rotate_action_client_node = Node(
        package="sobit_light_library",
        executable="wheel_rotate_action_client",
        name="wheel_rotate_action_client",
        namespace=robot_name if robot_id == 0 else f"{robot_name}_{robot_id}",
        output="screen",
    )


    return LaunchDescription([
        wheel_linear_action_client_node,
        # wheel_rotate_action_client_node,
    ])
