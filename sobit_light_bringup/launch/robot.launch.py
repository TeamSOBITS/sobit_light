import os
import tempfile
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import xacro
import yaml


def _mujoco_plugin_params(path, robot_name, cameras_on, enable_lidar, frame_prefix):
    """mujoco_plugins.yaml with the enable_* flags applied: the CameraPlugin renders every MJCF
    camera, so disabled ones are only polled; the lidar plugin is dropped without the lidar."""
    with open(path) as f:
        params = yaml.safe_load(f)['/**']['ros__parameters']
    plugins = params['mujoco_plugins']
    for cam, on in cameras_on.items():
        plugins['cameras'][cam]['policy'] = 'streaming' if on else 'polled'
    if not enable_lidar:
        del plugins['lidars']
    for label in plugins.values():
        for sensor in label.values():
            if isinstance(sensor, dict):
                sensor['frame_name'] = frame_prefix + sensor['frame_name']
                # Relative topics would land under /<robot_name>/<label>/
                for key in [k for k in sensor if k.endswith('topic')]:
                    sensor[key] = f'/{robot_name}/{sensor[key]}'
    return params


def generate_launch_description():
    arg_robot_name = DeclareLaunchArgument('robot_name', default_value='sobit_light')

    arg_robot_coords_x = DeclareLaunchArgument('robot_coords_x', default_value='0')
    arg_robot_coords_y = DeclareLaunchArgument('robot_coords_y', default_value='0')
    arg_robot_coords_Y = DeclareLaunchArgument('robot_coords_Y', default_value='0')

    arg_enable_mobile_base = DeclareLaunchArgument('enable_mobile_base', default_value='True')
    arg_enable_head        = DeclareLaunchArgument('enable_head', default_value='True')
    arg_enable_arm         = DeclareLaunchArgument('enable_arm', default_value='True')
    arg_enable_hand        = DeclareLaunchArgument('enable_hand', default_value='True')

    arg_enable_gz                 = DeclareLaunchArgument(
        'enable_gz', default_value='True',
        description='Gazebo (True) or real hardware (False). Superseded by simulator when that is set.')
    arg_simulator                 = DeclareLaunchArgument(
        'simulator', default_value='',
        description="'' (derive from enable_gz) | none (real hardware) | gz | isaac (started externally) | mujoco")
    arg_mujoco_model              = DeclareLaunchArgument(
        'mujoco_model', default_value='',
        description='simulator:=mujoco only: absolute path of the MJCF scene (world + robot)')
    arg_mujoco_headless           = DeclareLaunchArgument(
        'mujoco_headless', default_value='false',
        description='simulator:=mujoco only: run without the Simulate window')
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
    arg_enable_moveit_rviz = DeclareLaunchArgument('enable_moveit_rviz', default_value='false')

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
        arg_simulator,
        arg_mujoco_model,
        arg_mujoco_headless,
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
        arg_enable_moveit_rviz,
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
    simulator                 = LaunchConfiguration('simulator').perform(context).strip().lower()
    mujoco_model              = LaunchConfiguration('mujoco_model').perform(context)
    mujoco_headless           = _bool_str(LaunchConfiguration('mujoco_headless').perform(context))

    enable_real_head_cam = _bool_str(LaunchConfiguration('enable_real_head_cam').perform(context)) # TODO: Implement
    enable_real_hand_cam = _bool_str(LaunchConfiguration('enable_real_hand_cam').perform(context)) # TODO: Implement

    enable_tf_prefix = _bool_str(LaunchConfiguration('enable_tf_prefix').perform(context)) == 'True'
    tf_prefix = robot_name + '/' if enable_tf_prefix else ''

    enable_moveit   = _bool_str(LaunchConfiguration('enable_moveit').perform(context))
    enable_teleop   = _bool_str(LaunchConfiguration('enable_teleop').perform(context))
    enable_moveit_rviz = _bool_str(LaunchConfiguration('enable_moveit_rviz').perform(context))

    if not simulator:
        simulator = 'gz' if enable_gz == 'True' else 'none'
    elif simulator not in ('none', 'gz', 'isaac', 'mujoco'):
        print(f"Unknown simulator '{simulator}'. Use 'none', 'gz', 'isaac' or 'mujoco'.")
        exit(1)
    if simulator == 'mujoco' and not os.path.isfile(mujoco_model):
        print(f"simulator:=mujoco needs mujoco_model:=<scene.xml>; '{mujoco_model}' does not exist.")
        exit(1)
    enable_gz = 'True' if simulator == 'gz' else 'False'
    is_sim = simulator in ('gz', 'isaac', 'mujoco')
    # Isaac's controller_manager lives in the robot USD and only appears once the sim plays
    spawner_timeout = ['--controller-manager-timeout', '120'] if simulator == 'isaac' else []

    # Find Dynamixel Port name from DXL_LOWER_PORT/DXL_UPPER_PORT environment variable
    dxl_sl_port = ''
    if simulator == 'none':
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
            'enable_mujoco'             : 'True' if simulator == 'mujoco' else 'False',
            'mujoco_model'              : mujoco_model,
            'mujoco_headless'           : mujoco_headless.lower(),
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
        {"use_sim_time": is_sim},
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
    
    # gz_controllers.yaml serves every simulator (Isaac ships its own copy in the USD)
    controller_config_name = 'gz_controllers.yaml' if is_sim else 'real_controllers.yaml'
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

    if simulator == 'mujoco':
        # Patched controller_manager hosting MuJoCo (+ Simulate window); publishes /clock
        mujoco_plugins_config = os.path.join(get_package_share_directory(
            'sobit_light_bringup'), 'config', 'mujoco_plugins.yaml')
        control_node = Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            name="controller_manager",
            namespace=robot_name,
            parameters=[
                controller_config,
                {"use_sim_time": True},
                _mujoco_plugin_params(
                    mujoco_plugins_config,
                    robot_name,
                    {
                        'head_camera':  enable_head == 'True' and 'True' in (enable_gz_head_cam_color, enable_gz_head_cam_depth),
                        'hand_camera':  enable_hand == 'True' and 'True' in (enable_gz_hand_cam_color, enable_gz_hand_cam_depth),
                        'camera_front': enable_mobile_base == 'True' and enable_gz_front_cam_color == 'True',
                        'camera_rear':  enable_mobile_base == 'True' and enable_gz_back_cam_color == 'True',
                    },
                    enable_mobile_base == 'True' and enable_gz_lidar == 'True',
                    tf_prefix),
            ],
            remappings=[
                ("controller_manager/robot_description", "robot_description"),
            ],
            output="both",
            on_exit=Shutdown(),
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
                '-c', 'controller_manager', '--activate', *spawner_timeout
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
                '-c', 'controller_manager', '--activate', *spawner_timeout
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
                '-c', 'controller_manager', '--activate', *spawner_timeout
                ],
        )
        controllers.append(hand_position_controller)
        

    if enable_mobile_base == 'True' and is_sim:
        wheel_controller_args = [
            'wheel_controller',
            '-c', 'controller_manager', '--activate', *spawner_timeout
        ]
        # diff_drive_controller namespace-prefixes its odom TF frames by default,
        # which only matches the TF tree when the prefix is enabled.
        # Isaac's copy in the USD pins it false, so there it is always set explicitly.
        if not enable_tf_prefix or simulator == 'isaac':
            override = tempfile.NamedTemporaryFile(
                'w', prefix='wheel_controller_tf_prefix_', suffix='.yaml', delete=False)
            override.write('/**:\n  ros__parameters:\n    tf_frame_prefix_enable: '
                           f'{str(enable_tf_prefix).lower()}\n')
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
            '-c', 'controller_manager', *spawner_timeout,
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
        remappings=[
            ("cmd_vel_in",  f"/{robot_name}/manual_control/cmd_vel"),
            ("cmd_vel_out", f"/{robot_name}/wheel_controller/cmd_vel"),
        ],
        parameters=[{
            'frame_id': f'{tf_prefix}base_footprint',
            'use_sim_time': is_sim,
        }],
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
            'enable_gz': 'True' if is_sim else 'False',
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
            'use_sim_time'       : 'true' if is_sim else 'false',
            'use_rviz'           : 'true' if enable_moveit_rviz == 'True' else 'false',
            'enable_teleop'      : enable_teleop,
            # Module switches -> SRDF xacro args.
            'enable_mobile_base' : enable_mobile_base,
            'enable_arm'         : enable_arm,
            'enable_hand'        : enable_hand,
            'enable_head'        : enable_head,
            'enable_tf_prefix'   : 'true' if enable_tf_prefix else 'false',
        }.items(),
    )

    if simulator == 'none':
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
                    'tf_prefix': tf_prefix,
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
                    'tf_prefix': tf_prefix,
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
                    "/" + robot_name + "/base_back_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[gz.msgs.CameraInfo",
                    "/" + robot_name + "/base_back_camera/color" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                    "/" + robot_name + "/head_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[gz.msgs.CameraInfo",
                    "/" + robot_name + "/head_camera/color" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                    "/" + robot_name + "/hand_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[gz.msgs.CameraInfo",
                    "/" + robot_name + "/hand_camera/color" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
                    "/" + robot_name + "/lidar/scan" + "@sensor_msgs/msg/LaserScan" + "[gz.msgs.LaserScan",
                    "/" + robot_name + "/lidar/scan/points" + "@sensor_msgs/msg/PointCloud2" + "[gz.msgs.PointCloudPacked",
                    "/" + robot_name + "/imu" + "@sensor_msgs/msg/Imu" + "[gz.msgs.IMU",
                ],
        remappings=[
                    ("/" + robot_name + "/head_camera/camera_info", "/" + robot_name + "/head_camera/color/camera_info"),
                    ("/" + robot_name + "/head_camera/color", "/" + robot_name + "/head_camera/color/image_raw"),
                    ("/" + robot_name + "/hand_camera/camera_info", "/" + robot_name + "/hand_camera/color/camera_info"),
                    ("/" + robot_name + "/hand_camera/color", "/" + robot_name + "/hand_camera/color/image_raw"),
                    ("/" + robot_name + "/base_front_camera/camera_info", "/" + robot_name + "/front_camera/camera_info"),
                    ("/" + robot_name + "/base_front_camera/color", "/" + robot_name + "/front_camera/image_raw"),
                    ("/" + robot_name + "/base_back_camera/camera_info", "/" + robot_name + "/back_camera/camera_info"),
                    ("/" + robot_name + "/base_back_camera/color", "/" + robot_name + "/back_camera/image_raw"),
                ],
        parameters=[{
            'use_sim_time': True if enable_gz == 'True' else False,
            f'qos_overrides./{robot_name}/head_camera/color/image_raw.publisher.reliability': 'best_effort',
            f'qos_overrides./{robot_name}/head_camera/color/image_raw.publisher.depth': 1,
            f'qos_overrides./{robot_name}/hand_camera/color/image_raw.publisher.reliability': 'best_effort',
            f'qos_overrides./{robot_name}/hand_camera/color/image_raw.publisher.depth': 1,
            f'qos_overrides./{robot_name}/front_camera/image_raw.publisher.reliability': 'best_effort',
            f'qos_overrides./{robot_name}/front_camera/image_raw.publisher.depth': 1,
            f'qos_overrides./{robot_name}/back_camera/image_raw.publisher.reliability': 'best_effort',
            f'qos_overrides./{robot_name}/back_camera/image_raw.publisher.depth': 1,
        }],
        output='screen'
    )

    depth_cams = [('head_camera', enable_gz_head_cam_depth), ('hand_camera', enable_gz_hand_cam_depth)]

    # Depth gets its own bridge, stamped with the optical frame, and a ROS-side
    # point cloud, so the topics match the real RealSense driver's layout.
    gz_bridge_depth_nodes = {cam: Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'parameter_bridge_{cam}_depth',
        namespace=robot_name,
        arguments=[
            f"/{robot_name}/{cam}/depth" + "@sensor_msgs/msg/Image" + "[gz.msgs.Image",
            f"/{robot_name}/{cam}/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[gz.msgs.CameraInfo",
        ],
        remappings=[
            (f"/{robot_name}/{cam}/depth", f"/{robot_name}/{cam}/depth/image_rect_raw"),
            (f"/{robot_name}/{cam}/camera_info", f"/{robot_name}/{cam}/depth/camera_info"),
        ],
        parameters=[{
            'use_sim_time': True,
            'override_frame_id': f'{tf_prefix}{cam}_depth_optical_frame',
            f'qos_overrides./{robot_name}/{cam}/depth/image_rect_raw.publisher.reliability': 'best_effort',
            f'qos_overrides./{robot_name}/{cam}/depth/image_rect_raw.publisher.depth': 1,
            f'qos_overrides./{robot_name}/{cam}/depth/camera_info.publisher.reliability': 'best_effort',
            f'qos_overrides./{robot_name}/{cam}/depth/camera_info.publisher.depth': 1,
        }],
        output='screen'
    ) for cam, _ in depth_cams}

    point_cloud_nodes = {cam: Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name=f'{cam}_point_cloud_xyz',
        namespace=robot_name,
        parameters=[{
            'use_sim_time': True,
            f'qos_overrides./{robot_name}/{cam}/depth/image_rect_raw.subscription.reliability': 'best_effort',
            f'qos_overrides./{robot_name}/{cam}/depth/camera_info.subscription.reliability': 'best_effort',
            f'qos_overrides./{robot_name}/{cam}/depth/points.publisher.reliability': 'best_effort',
            f'qos_overrides./{robot_name}/{cam}/depth/points.publisher.depth': 1,
        }],
        remappings=[
            ('image_rect',  f'/{robot_name}/{cam}/depth/image_rect_raw'),
            ('camera_info', f'/{robot_name}/{cam}/depth/camera_info'),
            ('points',      f'/{robot_name}/{cam}/depth/points'),
        ],
        output='screen'
    ) for cam, _ in depth_cams}

    depth_compressed_nodes = {cam: Node(
        package='image_transport',
        executable='republish',
        name=f'{cam}_depth_compressed_republisher',
        namespace=robot_name,
        remappings=[
            ('in',                  f'/{robot_name}/{cam}/depth/image_rect_raw'),
            ('out/compressedDepth', f'/{robot_name}/{cam}/depth/image_rect_raw/compressedDepth'),
        ],
        parameters=[{
            # image_transport 5.x ignores positional transports; an empty out_transport loads every plugin (out, out/theora, ...)
            'in_transport': 'raw',
            'out_transport': 'compressedDepth',
            'use_sim_time': True,
            f'qos_overrides./{robot_name}/{cam}/depth/image_rect_raw.subscription.reliability': 'best_effort',
            f'qos_overrides./{robot_name}/{cam}/depth/image_rect_raw/compressedDepth.publisher.reliability': 'best_effort',
            f'qos_overrides./{robot_name}/{cam}/depth/image_rect_raw/compressedDepth.publisher.depth': 1,
        }],
        output='log'
    ) for cam, _ in depth_cams}

    # MuJoCo's CameraPlugin has one camera_info (color); depth_image_proc needs depth/camera_info
    mujoco_depth_info_relays = {cam: Node(
        package='topic_tools',
        executable='relay',
        name=f'{cam}_depth_info_relay',
        namespace=robot_name,
        parameters=[{
            'use_sim_time': True,
            'input_topic':  f'/{robot_name}/{cam}/color/camera_info',
            'output_topic': f'/{robot_name}/{cam}/depth/camera_info',
        }],
        output='log',
    ) for cam, _ in depth_cams}

    # Republish raw sim images as compressed for each camera
    # Base cameras have no /color/ segment in their topic, unlike head/hand.
    color_compressed_nodes = []
    for cam, enabled, in_topic in [
        ('head_camera', enable_gz_head_cam_color, f'/{robot_name}/head_camera/color/image_raw'),
        ('hand_camera', enable_gz_hand_cam_color, f'/{robot_name}/hand_camera/color/image_raw'),
        ('front_camera', enable_gz_front_cam_color, f'/{robot_name}/front_camera/image_raw'),
        ('back_camera', enable_gz_back_cam_color, f'/{robot_name}/back_camera/image_raw'),
    ]:
        if enabled == 'True':
            out_topic = f'{in_topic}/compressed'
            color_compressed_nodes.append(Node(
                package='image_transport',
                executable='republish',
                name=f'{cam}_compressed_republisher',
                namespace=robot_name,
                remappings=[
                    ('in',             in_topic),
                    ('out/compressed', out_topic),
                ],
                parameters=[{
                    'in_transport': 'raw',
                    'out_transport': 'compressed',
                    'use_sim_time': True,
                    f'qos_overrides.{in_topic}.subscription.reliability': 'best_effort',
                    f'qos_overrides.{out_topic}.publisher.reliability': 'best_effort',
                }],
                output='log',
            ))

    if simulator == 'gz':
        nodes.append(gz_bridge_node)
        nodes.append(gz_spawn_entity_node)
        nodes.append(delayed_joint_state_broadcaster)
        nodes.append(delayed_vel_remap_node)
        nodes.append(delayed_odom_remap_node)
        nodes.append(delayed_controllers)

        for cam, enabled in depth_cams:
            if enabled == 'True':
                nodes += [gz_bridge_depth_nodes[cam], point_cloud_nodes[cam], depth_compressed_nodes[cam]]
        nodes += color_compressed_nodes
    elif simulator == 'isaac':
        # The robot USD's OmniGraphs publish cameras (H.264 included), depth points, the lidar, /clock
        # and host controller_manager; spawners start directly and wait for it to appear after play.
        nodes.append(joint_state_broadcaster)
        nodes.append(delayed_vel_remap_node)
        nodes.append(delayed_odom_remap_node)
        nodes.append(delayed_controllers)
        for cam, enabled in depth_cams:
            if enabled == 'True':
                nodes.append(depth_compressed_nodes[cam])
    elif simulator == 'mujoco':
        # Serialize startup so control_node doesn't race robot_state_publisher
        # for 'robot_description' and spawners don't pile up.
        nodes.append(RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=robot_state_publisher_node,
                on_start=[control_node],
            )
        ))
        nodes.append(RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=control_node,
                on_start=[joint_state_broadcaster],
            )
        ))
        nodes.append(delayed_vel_remap_node)
        nodes.append(delayed_odom_remap_node)
        nodes.append(delayed_controllers)
        # MuJoCo publishes raw depth only: points + compressedDepth come from the ROS side
        for cam, enabled in depth_cams:
            if enabled == 'True':
                nodes += [mujoco_depth_info_relays[cam], point_cloud_nodes[cam], depth_compressed_nodes[cam]]
        nodes += color_compressed_nodes
    else:
        nodes.append(joint_state_broadcaster)
        nodes.append(control_node)
        nodes.extend(controllers)

    nodes.append(robot_state_publisher_node)
    nodes.append(action_server_launch)

    if enable_moveit == 'True':
        nodes.append(moveit_launch)


    return nodes
