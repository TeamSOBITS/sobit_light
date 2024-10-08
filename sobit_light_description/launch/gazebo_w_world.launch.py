import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    ignition_ros2_control_demos_path = os.path.join(
        get_package_share_directory('sobit_light_description'),)

    xacro_file = os.path.join(ignition_ros2_control_demos_path,
                              'robots',
                              'sobit_light_robot.urdf.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml(), 'use_sim_time': use_sim_time}

    print(params)

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', doc.toxml(),
                   '-name', 'sobit_light',
                   '-allow_renaming', 'true',
                   '-x', '-2.5',
                   '-y', '2.5',
                   '-z', '0',
                   '-R', '0',
                   '-P', '0',
                   '-Y', '-1.57'],
    )

    # node_controller_manager = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[]


    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             '--use-sim-time',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             '--use-sim-time',
             'arm_controller'],
        output='screen'
    )

    load_head_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             '--use-sim-time',
             'head_controller'],
        output='screen'
    )

    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             '--use-sim-time',
             'gripper_controller'],
        output='screen'
    )

    load_diff_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             '--use-sim-time',
             'diff_controller'],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                    "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
                    "/kachaka/lidar/scan" + "@sensor_msgs/msg/LaserScan" + "[ignition.msgs.LaserScan",
                    "/kachaka/lidar/scan/points" + "@sensor_msgs/msg/PointCloud2" + "[ignition.msgs.PointCloudPacked",
                    # "/tf" + "@tf2_msgs/msg/TFMessage" + "[ignition.msgs.TFMessage",
                    "/model/sobit_light/pose" + "@geometry_msgs/msg/Pose" + "[ignition.msgs.Pose",
                    "/joint_states" + "@sensor_msgs/msg/JointState" + "[ignition.msgs.Model",
                    "/head_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[ignition.msgs.CameraInfo",
                    "/head_camera/color" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                    "/head_camera/depth" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                    "/head_camera/depth/points" + "@sensor_msgs/msg/PointCloud2" + "[ignition.msgs.PointCloudPacked",
                   ],
        output='screen'
    )

    world_file_path = PathJoinSubstitution([
        get_package_share_directory('wrs_arena_gz'),
        'worlds',
        'wrs2020.world.xacro'
    ])

    return LaunchDescription([
        bridge,
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            # launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]),
            launch_arguments=[('gz_args', world_file_path)]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_arm_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_head_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_gripper_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_controller],
            )
        ),
        node_robot_state_publisher,
        ignition_spawn_entity,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])