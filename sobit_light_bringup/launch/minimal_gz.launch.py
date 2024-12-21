import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnExecutionComplete, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():

    robot_name = "sobit_light"

    bringup_pkg = robot_name + "_bringup"
    description_pkg = robot_name + "_description"

    rviz_config = os.path.join(get_package_share_directory(
        bringup_pkg), "rviz", "gazebo.rviz")
    
    robot_description = os.path.join(get_package_share_directory(
        description_pkg), "robots", robot_name + "_robot.urdf.xacro")
    robot_description_config = \
        xacro.process_file(robot_description, mappings={'enable_gz' : 'True', 'robot_name' : robot_name})


    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active',
             '--controller-manager', robot_name+'/controller_manager',
             '--use-sim-time',
             'joint_state_broadcaster'],
        output='screen'
    )

    joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active',
             '--controller-manager', robot_name+'/controller_manager',
             '--use-sim-time',
             'joint_trajectory_controller'],
        output='screen'
    )

    velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active',
             '--controller-manager', robot_name+'/controller_manager',
             '--use-sim-time',
             'velocity_controller'],
        output='screen'
    )

    diff_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active',
             '--controller-manager', robot_name+'/controller_manager',
             '--use-sim-time',
             'diff_controller'],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_name,
        parameters=[
            {"frame_prefix": robot_name+"/"},
            {"robot_description": robot_description_config.toxml()},
            {"use_sim_time": True},],
        output="screen",
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=robot_name,
        parameters=[
            {'frame_prefix': robot_name+'/'},
            {"robot_description": robot_description_config.toxml()},
            {'use_sim_time': True},],
        output="screen"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    gz_spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=robot_name,
        arguments=['-topic', 'robot_description',
                   '-entity', robot_name,
                   '-x', '0', '-y', '0', '-z', '0',],
        output='screen',
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                    "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
                    "/sobit_light/lidar/scan" + "@sensor_msgs/msg/LaserScan" + "[ignition.msgs.LaserScan",
                    "/sobit_light/lidar/scan/points" + "@sensor_msgs/msg/PointCloud2" + "[ignition.msgs.PointCloudPacked",
                    # "/tf" + "@tf2_msgs/msg/TFMessage" + "[ignition.msgs.TFMessage",
                    # "/model/sobit_light/pose" + "@geometry_msgs/msg/Pose" + "[ignition.msgs.Pose",
                    "/sobit_light/joint_states" + "@sensor_msgs/msg/JointState" + "[ignition.msgs.Model",
                    "/sobit_light/base_front_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[ignition.msgs.CameraInfo",
                    "/sobit_light/base_front_camera/color" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                    "/sobit_light/base_front_camera/depth" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                    "/sobit_light/base_back_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[ignition.msgs.CameraInfo",
                    "/sobit_light/base_back_camera/color" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                    "/sobit_light/base_back_camera/depth" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                    "/sobit_light/head_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[ignition.msgs.CameraInfo",
                    "/sobit_light/head_camera/color" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                    "/sobit_light/head_camera/depth" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                    "/sobit_light/head_camera/depth/points" + "@sensor_msgs/msg/PointCloud2" + "[ignition.msgs.PointCloudPacked",
                    "/sobit_light/hand_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[ignition.msgs.CameraInfo",
                    "/sobit_light/hand_camera/color" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                    "/sobit_light/hand_camera/depth" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                    "/sobit_light/hand_camera/depth/points" + "@sensor_msgs/msg/PointCloud2" + "[ignition.msgs.PointCloudPacked",
                   ],
        output='screen'
    )

    gz_tf_head_cam_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'sobit_light/head_camera_depth_optical_frame',
                   '--child-frame-id', 'sobit_light/head_pitch_link/head_camera_depth',
                   '--pitch', '-1.57',
                   '--roll', '1.57'],
        output='screen',
    )

    gz_tf_hand_cam_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'sobit_light/hand_camera_depth_optical_frame',
                   '--child-frame-id', 'sobit_light/arm_wrist_roll_link/hand_camera_depth',
                   '--pitch', '-1.57',
                   '--roll', '1.57'],
        output='screen',
    )

    return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]),
        gz_spawn_entity_node,
        gz_bridge_node,
        # gz_tf_head_cam_node,
        # gz_tf_hand_cam_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity_node,
                on_exit=[joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[joint_trajectory_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[velocity_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[diff_controller],
            )
        ),
        robot_state_publisher_node,
        # joint_state_publisher_node,
        rviz_node,
    ])
