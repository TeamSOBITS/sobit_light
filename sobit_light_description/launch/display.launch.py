import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='True'),
        DeclareLaunchArgument('enable_tf_prefix', default_value='True'),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context, *args, **kwargs):

    use_gui = LaunchConfiguration('use_gui')
    enable_tf_prefix = LaunchConfiguration('enable_tf_prefix').perform(context).lower() in ('true', '1', 'yes')

    robot_name = "sobit_light"


    rviz_config = os.path.join(get_package_share_directory(
        'sobit_light_description'), "rviz", "display.rviz")

    robot_description = os.path.join(get_package_share_directory(
        'sobit_light_description'),
        'robots',
        'sobit_light_robot.urdf.xacro'
    )
    robot_description_config = xacro.process_file(
        robot_description,
        mappings={
            'enable_mobile_base' : 'True',
            'enable_head'        : 'True',
            'enable_arm'         : 'True',
            'enable_hand'        : 'True',
            'enable_gz'                 : 'True',
            'enable_gz_front_cam_color' : 'True',
            'enable_gz_back_cam_color'  : 'True',
            'enable_gz_head_cam_color'  : 'True',
            'enable_gz_head_cam_depth'  : 'True',
            'enable_gz_hand_cam_color'  : 'True',
            'enable_gz_hand_cam_depth'  : 'True',
            'enable_gz_lidar'           : 'True',
            'enable_gz_imu'             : 'True',
            'enable_tf_prefix'          : 'True' if enable_tf_prefix else 'False',
            'robot_name' : robot_name,
        }
    )

    robot_state_publisher_params = [
        {"robot_description": robot_description_config.toxml()},
        {"use_sim_time": True},
    ]
    if enable_tf_prefix:
        robot_state_publisher_params.append({"frame_prefix": robot_name + '/'})

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_name,
        parameters=robot_state_publisher_params,
        output="screen",
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        namespace=robot_name,
        condition=UnlessCondition(use_gui)
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        namespace=robot_name,
        condition=IfCondition(use_gui)
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return [
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ]
