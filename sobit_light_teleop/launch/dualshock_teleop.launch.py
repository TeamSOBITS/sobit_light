from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sobit_light_teleop',
            executable='dualshock_teleop',
            name='dualshock_teleop'
        ),
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux_node',
            remappings=[
                ('/joy', '/sobit_light/joy')
            ]
        )
    ])