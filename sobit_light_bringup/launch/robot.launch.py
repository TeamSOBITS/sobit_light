import os
import tempfile
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    arg_robot_name = DeclareLaunchArgument('robot_name', default_value='sobit_light')

    arg_robot_coords_x = DeclareLaunchArgument('robot_coords_x', default_value='0')
    arg_robot_coords_y = DeclareLaunchArgument('robot_coords_y', default_value='0')
    arg_robot_coords_Y = DeclareLaunchArgument('robot_coords_Y', default_value='0')

    arg_enable_mobile_base = DeclareLaunchArgument('enable_mobile_base', default_value='True')
    arg_enable_head        = DeclareLaunchArgument('enable_head', default_value='True')
    arg_enable_arm         = DeclareLaunchArgument('enable_arm', default_value='True')
    arg_enable_hand        = DeclareLaunchArgument('enable_hand', default_value='True')

    arg_enable_gz                 = DeclareLaunchArgument('enable_gz', default_value='True')
    arg_enable_gz_front_cam_color = DeclareLaunchArgument('enable_gz_front_cam_color', default_value='True')
    arg_enable_gz_back_cam_color  = DeclareLaunchArgument('enable_gz_back_cam_color', default_value='True')
    arg_enable_gz_head_cam_color  = DeclareLaunchArgument('enable_gz_head_cam_color', default_value='True')
    arg_enable_gz_head_cam_depth  = DeclareLaunchArgument('enable_gz_head_cam_depth', default_value='True')
    arg_enable_gz_hand_cam_color  = DeclareLaunchArgument('enable_gz_hand_cam_color', default_value='True')
    arg_enable_gz_hand_cam_depth  = DeclareLaunchArgument('enable_gz_hand_cam_depth', default_value='True')
    arg_enable_gz_lidar           = DeclareLaunchArgument('enable_gz_lidar', default_value='True')
    arg_enable_gz_imu             = DeclareLaunchArgument('enable_gz_imu', default_value='True')

    arg_enable_real_head_cam = DeclareLaunchArgument('enable_real_head_cam', default_value='True')
    arg_enable_real_hand_cam = DeclareLaunchArgument('enable_real_hand_cam', default_value='True')

    arg_enable_tf_prefix = DeclareLaunchArgument('enable_tf_prefix', default_value='True')

    arg_enable_moveit   = DeclareLaunchArgument('enable_moveit', default_value='True')
    arg_enable_teleop   = DeclareLaunchArgument('enable_teleop', default_value='True')
    arg_use_moveit_rviz = DeclareLaunchArgument('use_moveit_rviz', default_value='True')

    return LaunchDescription([
        arg_robot_name,
        arg_robot_coords_x,
        arg_robot_coords_y,
        arg_robot_coords_Y,
        arg_enable_mobile_base,
        arg_enable_head,
        arg_enable_arm,
        arg_enable_hand,
        arg_enable_gz,
        arg_enable_gz_front_cam_color,
        arg_enable_gz_back_cam_color,
        arg_enable_gz_head_cam_color,
        arg_enable_gz_head_cam_depth,
        arg_enable_gz_hand_cam_color,
        arg_enable_gz_hand_cam_depth,
        arg_enable_gz_lidar,
        arg_enable_gz_imu,
        arg_enable_real_head_cam,
        arg_enable_real_hand_cam,
        arg_enable_tf_prefix,
        arg_enable_moveit,
        arg_enable_teleop,
        arg_use_moveit_rviz,
        OpaqueFunction(function = launch_gz),
    ])


def _bool_str(val):
    return 'True' if val.lower() in ('true', '1', 'yes') else 'False'


def launch_gz(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)

    robot_coords_x = LaunchConfiguration('robot_coords_x').perform(context)
    robot_coords_y = LaunchConfiguration('robot_coords_y').perform(context)
    robot_coords_Y = LaunchConfiguration('robot_coords_Y').perform(context)

    enable_mobile_base = _bool_str(LaunchConfiguration('enable_mobile_base').perform(context))
    enable_head        = _bool_str(LaunchConfiguration('enable_head').perform(context))
    enable_arm         = _bool_str(LaunchConfiguration('enable_arm').perform(context))
    enable_hand        = _bool_str(LaunchConfiguration('enable_hand').perform(context))

    enable_gz                 = _bool_str(LaunchConfiguration('enable_gz').perform(context))
    enable_gz_front_cam_color = _bool_str(LaunchConfiguration('enable_gz_front_cam_color').perform(context))
    enable_gz_back_cam_color  = _bool_str(LaunchConfiguration('enable_gz_back_cam_color').perform(context))
    enable_gz_head_cam_color  = _bool_str(LaunchConfiguration('enable_gz_head_cam_color').perform(context))
    enable_gz_head_cam_depth  = _bool_str(LaunchConfiguration('enable_gz_head_cam_depth').perform(context))
    enable_gz_hand_cam_color  = _bool_str(LaunchConfiguration('enable_gz_hand_cam_color').perform(context))
    enable_gz_hand_cam_depth  = _bool_str(LaunchConfiguration('enable_gz_hand_cam_depth').perform(context))
    enable_gz_lidar           = _bool_str(LaunchConfiguration('enable_gz_lidar').perform(context))
    enable_gz_imu             = _bool_str(LaunchConfiguration('enable_gz_imu').perform(context))

    enable_real_head_cam = _bool_str(LaunchConfiguration('enable_real_head_cam').perform(context)) # TODO: Implement
    enable_real_hand_cam = _bool_str(LaunchConfiguration('enable_real_hand_cam').perform(context)) # TODO: Implement

    enable_tf_prefix = _bool_str(LaunchConfiguration('enable_tf_prefix').perform(context)) == 'True'
    tf_prefix = robot_name + '/' if enable_tf_prefix else ''

    enable_moveit   = _bool_str(LaunchConfiguration('enable_moveit').perform(context))
    enable_teleop   = _bool_str(LaunchConfiguration('enable_teleop').perform(context))
    use_moveit_rviz = _bool_str(LaunchConfiguration('use_moveit_rviz').perform(context))

    # Find Dynamixel Port name from DXL_LOWER_PORT/DXL_UPPER_PORT environment variable
    dxl_sl_port = ''
    if enable_gz == 'False':
        dxl_sl_port = str(os.environ.get('DXL_SL_PORT'))
        print('Dynamixel SOBIT LIGHT Port : ' + dxl_sl_port)


    robot_description = os.path.join(get_package_share_directory(
        'sobit_light_description'), 
        'robots',
        'sobit_light_robot.urdf.xacro'
    )
    robot_description_config = xacro.process_file(
        robot_description,
        mappings={
            'robot_name'                : robot_name,
            'enable_mobile_base'        : enable_mobile_base,
            'enable_head'               : enable_head,
            'enable_arm'                : enable_arm,
            'enable_hand'               : enable_hand,
            'enable_gz'                 : enable_gz,
            'enable_gz_front_cam_color' : enable_gz_front_cam_color,
            'enable_gz_back_cam_color'  : enable_gz_back_cam_color,
            'enable_gz_head_cam_color'  : enable_gz_head_cam_color,
            'enable_gz_head_cam_depth'  : enable_gz_head_cam_depth,
            'enable_gz_hand_cam_color'  : enable_gz_hand_cam_color,
            'enable_gz_hand_cam_depth'  : enable_gz_hand_cam_depth,
            'enable_gz_lidar'           : enable_gz_lidar,
            'enable_gz_imu'             : enable_gz_imu,
            'enable_tf_prefix'          : 'True' if enable_tf_prefix else 'False',
            'dxl_sl_port'               : dxl_sl_port,
        })
    
    head_cam_config = os.path.join(get_package_share_directory(
        'sobit_light_bringup'),
        'launch',
        'include',
        'head_cam_param.yaml'
    )

    hand_cam_config = os.path.join(get_package_share_directory(
        'sobit_light_bringup'),
        'launch',
        'include',
        'hand_cam_param.yaml'
    )

    robot_state_publisher_params = [
        {"robot_description": robot_description_config.toxml()},
        {"use_sim_time": True if enable_gz == 'True' else False},
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
    
    controller_config_name = 'gz_controllers.yaml' if enable_gz == 'True' else 'real_controllers.yaml'
    controller_config = os.path.join(get_package_share_directory(
        'sobit_light_control'),
        'config',
        controller_config_name
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # name="controller_manager",
        namespace=robot_name,
        parameters=[controller_config],
        remappings=[
            ("controller_manager/robot_description", "robot_description"),
        ],
        output="both",
    )

    controllers = []
    nodes = []

    if enable_head == 'True':
        head_position_controller = Node(
            package='controller_manager',
            executable='spawner',
            # name='head_position_controller',
            namespace=robot_name,
            arguments=[
                'head_position_controller',
                '-c', 'controller_manager', '--activate'
                ],
        )
        controllers.append(head_position_controller)

    if enable_arm == 'True':
        arm_position_controller = Node(
            package='controller_manager',
            executable='spawner',
            # name='arm_position_controller',
            namespace=robot_name,
            arguments=[
                'arm_position_controller',
                '-c', 'controller_manager', '--activate'
                ],
        )
        controllers.append(arm_position_controller)

    if enable_hand == 'True':
        hand_position_controller = Node(
            package='controller_manager',
            executable='spawner',
            # name='hand_position_controller',
            namespace=robot_name,
            arguments=[
                'hand_position_controller',
                '-c', 'controller_manager', '--activate'
                ],
        )
        controllers.append(hand_position_controller)
        

    if enable_mobile_base == 'True' and enable_gz == 'True':
        wheel_controller_args = [
            'wheel_controller',
            '-c', 'controller_manager', '--activate'
        ]
        # diff_drive_controller namespace-prefixes its odom TF frames by default,
        # which only matches the TF tree when the prefix is enabled.
        if not enable_tf_prefix:
            override = tempfile.NamedTemporaryFile(
                'w', prefix='wheel_controller_no_tf_prefix_', suffix='.yaml', delete=False)
            override.write('/**:\n  ros__parameters:\n    tf_frame_prefix_enable: false\n')
            override.close()
            wheel_controller_args += ['--param-file', override.name]
        wheel_controller = Node(
            package='controller_manager',
            executable='spawner',
            # name='wheel_controller',
            namespace=robot_name,
            arguments=wheel_controller_args,
        )
        controllers.append(wheel_controller)

    gz_spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=robot_name,
        arguments=[
            '-topic', '/' + robot_name + '/robot_description',
            '-name', robot_name,
            '-x', robot_coords_x,
            '-y', robot_coords_y,
            '-Y', robot_coords_Y,
        ],
        output='screen',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        # name='joint_state_broadcaster',
        namespace=robot_name,
        arguments=[
            'joint_state_broadcaster',
            '-c', 'controller_manager',
            ],
    )

    delayed_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity_node,
            on_exit=joint_state_broadcaster,
        )
    )

    delayed_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=controllers,
        )
    )

    vel_remap_node = Node(
        package="twist_stamper",
        executable="twist_stamper",
        namespace=robot_name,
        name="vel_remap",
        arguments=["-r", f"cmd_vel_in:=/{robot_name}/manual_control/cmd_vel", "-r", f"cmd_vel_out:=/{robot_name}/wheel_controller/cmd_vel", "-p", f"frame_id:={tf_prefix}base_footprint"]
    )

    delayed_vel_remap_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[vel_remap_node],
        )
    )

    odom_remap_node = Node(
        package="topic_tools",
        executable="relay",
        namespace=robot_name,
        name="odom_remap",
        arguments=[f"/{robot_name}/wheel_controller/odom", f"/{robot_name}/odometry/odometry"]
    )

    delayed_odom_remap_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[odom_remap_node],
        )
    )


    action_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sobit_light_library'),
                'launch',
                'action_server.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': robot_name,
            'enable_gz': enable_gz,
        }.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sobit_light_moveit_config'),
                'launch',
                'move_group.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name'         : robot_name,
            'use_sim_time'       : 'true' if enable_gz == 'True' else 'false',
            'use_rviz'           : 'true' if use_moveit_rviz == 'True' else 'false',
            'enable_teleop'      : enable_teleop,
            # Module switches -> SRDF xacro args.
            'enable_mobile_base' : enable_mobile_base,
            'enable_arm'         : enable_arm,
            'enable_hand'        : enable_hand,
            'enable_head'        : enable_head,
            'enable_tf_prefix'   : 'true' if enable_tf_prefix else 'false',
        }.items(),
    )

    if enable_gz == 'False':
        if enable_real_head_cam == 'True':
            rs_head_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('realsense2_camera'),
                        'launch',
                        'rs_launch.py'
                    ])
                ]),
                launch_arguments={
                    'camera_name': 'head_camera',
                    'camera_namespace': robot_name,
                    'config_file': head_cam_config,
                    'log_level': 'error',
                }.items(),
            )
            nodes.append(rs_head_launch)

        if enable_real_hand_cam == 'True':
            rs_hand_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('realsense2_camera'),
                        'launch',
                        'rs_launch.py'
                    ])
                ]),
                launch_arguments={
                    'camera_name': 'hand_camera',
                    'camera_namespace': robot_name,
                    'config_file': hand_cam_config,
                    'log_level': 'error',
                }.items(),
            )
            nodes.append(rs_hand_launch)

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=robot_name,
        arguments=[
                    "/" + robot_name + "/joint_states" + "@sensor_msgs/msg/JointState" + "[gz.msgs.Model",
                    # "/model/" + robot_name + "/pose" + "@geometry_msgs/msg/Pose" + "[gz.msgs.Pose",
                    "/" + robot_name + "/base_front_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[gz.msgs.CameraInfo",
                    "/" + robot_name + "/base_front_camera/color" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                    "/" + robot_name + "/base_front_camera/depth" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                    "/" + robot_name + "/base_back_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[gz.msgs.CameraInfo",
                    "/" + robot_name + "/base_back_camera/color" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                    "/" + robot_name + "/base_back_camera/depth" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                    "/" + robot_name + "/head_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[gz.msgs.CameraInfo",
                    "/" + robot_name + "/head_camera/color" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                    "/" + robot_name + "/head_camera/depth" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                    "/" + robot_name + "/head_camera/depth/points" + "@sensor_msgs/msg/PointCloud2" + "[gz.msgs.PointCloudPacked",
                    "/" + robot_name + "/hand_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[gz.msgs.CameraInfo",
                    "/" + robot_name + "/hand_camera/color" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                    "/" + robot_name + "/hand_camera/depth" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                    "/" + robot_name + "/hand_camera/depth/points" + "@sensor_msgs/msg/PointCloud2" + "[gz.msgs.PointCloudPacked",
                    "/" + robot_name + "/lidar/scan" + "@sensor_msgs/msg/LaserScan" + "[gz.msgs.LaserScan",
                    "/" + robot_name + "/lidar/scan/points" + "@sensor_msgs/msg/PointCloud2" + "[gz.msgs.PointCloudPacked",
                    "/" + robot_name + "/imu" + "@sensor_msgs/msg/Imu" + "[gz.msgs.IMU",
                ],
        output='screen'
    )

    if enable_gz == 'True':
        nodes.append(gz_bridge_node)
        nodes.append(gz_spawn_entity_node)
        nodes.append(delayed_joint_state_broadcaster)
        nodes.append(delayed_vel_remap_node)
        nodes.append(delayed_odom_remap_node)
        nodes.append(delayed_controllers)
    else:
        nodes.append(joint_state_broadcaster)
        nodes.append(control_node)
        nodes.extend(controllers)

    nodes.append(robot_state_publisher_node)
    nodes.append(action_server_launch)

    if enable_moveit == 'True':
        nodes.append(moveit_launch)


    return nodes
