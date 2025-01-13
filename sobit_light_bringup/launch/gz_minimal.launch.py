import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                    "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
                    "/tf" + "@tf2_msgs/msg/TFMessage" + "[ignition.msgs.TFMessage",
                   ],
        output='screen'
    )

    rviz_config = PathJoinSubstitution([
            FindPackageShare('sobit_light_bringup'),
            'rviz',
            'gazebo.rviz'
        ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    world_file = os.path.join(get_package_share_directory(
        'sobit_light_description'), 
        'worlds',
        'empty_w_physics.sdf'
    )

    return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                ])
            ]),
            launch_arguments={
                'gz_args' : ' -r -v 4 ' + world_file,
            }.items()
        ),
        gz_bridge_node,
        # Launch Robot No. 1
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('sobit_light_bringup'),
                    'launch',
                    'gz_robot.launch.py'
                ])
            ]),
            launch_arguments={
                'robot_name': 'sobit_light_1',
                'robot_coords_x': '0', # x 
                'robot_coords_y': '0', # y
                'robot_coords_Y': '0', # yaw
                'enable_gz_front_cam_color' : 'True',
                'enable_gz_back_cam_color' : 'True',
                'enable_gz_head_cam_color' : 'True',
                'enable_gz_head_cam_depth' : 'True',
                'enable_gz_hand_cam_color' : 'True',
                'enable_gz_hand_cam_depth' : 'True',
                'enable_gz_lidar' : 'True',
                'enable_gz_imu' : 'True',
            }.items()
        ),
        # Launch Robot No. 2
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('sobit_light_bringup'),
        #             'launch',
        #             'gz_robot.launch.py'
        #         ])
        #     ]),
        #     launch_arguments={
        #         'robot_name': 'sobit_light_2',
        #         'robot_coords_x': '0', # x 
        #         'robot_coords_y': '2', # y
        #         'robot_coords_Y': '0', # yaw
        #         'enable_gz_front_cam_color' : 'True',
        #         'enable_gz_back_cam_color' : 'True',
        #         'enable_gz_head_cam_color' : 'True',
        #         'enable_gz_head_cam_depth' : 'True',
        #         'enable_gz_hand_cam_color' : 'True',
        #         'enable_gz_hand_cam_depth' : 'True',
        #         'enable_gz_lidar' : 'True',
        #         'enable_gz_imu' : 'True',
        #     }.items()
        # ),
        rviz_node,
    ])
