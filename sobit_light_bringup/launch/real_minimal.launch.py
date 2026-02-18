from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = 'sobit_light'
    robot_id = 0

    rviz_config = PathJoinSubstitution([
            FindPackageShare('sobit_light_bringup'),
            'rviz',
            'real.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        # Launch Robot No. 1
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('sobit_light_bringup'),
                    'launch',
                    'robot.launch.py'
                ])
            ]),
            launch_arguments={
                'robot_name': robot_name if robot_id == 0 else robot_name + '_' + str(robot_id),
                'enable_mobile_base'   : 'True',
                'enable_head'          : 'True',
                'enable_arm'           : 'True',
                'enable_hand'          : 'True',
                'enable_real_head_cam' : 'True', # TODO: toggle head camera
                'enable_real_hand_cam' : 'False', # TODO: toggle hand camera
                'enable_gz' : 'False',
            }.items()
        ),
        rviz_node,
    ])
