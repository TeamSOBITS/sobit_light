import os
import re

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name',                 default_value='sobit_light'),
        DeclareLaunchArgument('robot_id',                   default_value='0'),
        DeclareLaunchArgument('world_model',                default_value='empty',
                              description='empty | wrs | small_house | rcjo2025'),
        DeclareLaunchArgument('robot_coords_x',             default_value='-5.5'),
        DeclareLaunchArgument('robot_coords_y',             default_value='1.5'),
        DeclareLaunchArgument('robot_coords_Y',             default_value='0.0'),
        DeclareLaunchArgument('use_rviz',                   default_value='true'),
        DeclareLaunchArgument('use_moveit_rviz',            default_value='true'),
        DeclareLaunchArgument('enable_mobile_base',         default_value='true'),
        DeclareLaunchArgument('enable_head',                default_value='true'),
        DeclareLaunchArgument('enable_arm',                 default_value='true'),
        DeclareLaunchArgument('enable_hand',                default_value='true'),
        DeclareLaunchArgument('enable_gz_front_cam_color',  default_value='true'),
        DeclareLaunchArgument('enable_gz_back_cam_color',   default_value='true'),
        DeclareLaunchArgument('enable_gz_head_cam_color',   default_value='true'),
        DeclareLaunchArgument('enable_gz_head_cam_depth',   default_value='true'),
        DeclareLaunchArgument('enable_gz_hand_cam_color',   default_value='true'),
        DeclareLaunchArgument('enable_gz_hand_cam_depth',   default_value='true'),
        DeclareLaunchArgument('enable_gz_lidar',            default_value='true'),
        DeclareLaunchArgument('enable_gz_imu',              default_value='true'),
        DeclareLaunchArgument('enable_moveit',              default_value='true'),
        DeclareLaunchArgument('enable_teleop',              default_value='true'),
        DeclareLaunchArgument('enable_tf_prefix',           default_value='true'),
        DeclareLaunchArgument('headless',                   default_value='false',
                              description='Run Gazebo in headless mode (--headless-rendering). '
                                          'Saves GPU memory when the GUI is not needed.'),
        OpaqueFunction(function=launch_setup),
    ])


def _bool(lc, context):
    """Normalize CLI true/True/1 → 'True', else → 'False' for xacro + robot.launch.py."""
    return 'True' if lc.perform(context).lower() in ('true', '1', 'yes') else 'False'


def _gz_world_name(path):
    """Read the <world name='...'> attribute for /world/<name>/... bridge topics.
    Shipped worlds declare it literally; falls back to Gazebo's 'default'."""
    try:
        with open(path) as f:
            m = re.search(r"<world\s+name=['\"]([^'\"]+)['\"]", f.read())
    except OSError:
        return 'default'
    return m.group(1) if m else 'default'


def launch_setup(context, *args, **kwargs):
    robot_name  = LaunchConfiguration('robot_name').perform(context)
    robot_id    = int(LaunchConfiguration('robot_id').perform(context))
    world_model = LaunchConfiguration('world_model').perform(context)
    headless    = LaunchConfiguration('headless').perform(context).lower() in ('true', '1', 'yes')

    # Resolve world file from world_model string
    if world_model == 'wrs':
        world_file = os.path.join(
            get_package_share_directory('tmc_wrs_gz_worlds'),
            'worlds', 'wrs2020.world.xacro')
    elif world_model == 'small_house':
        world_file = os.path.join(
            get_package_share_directory('aws_small_house_world'),
            'worlds', 'small_house.world')
    elif world_model == 'rcjo2025':
        world_file = os.path.join(
            get_package_share_directory('sobits_gazebo_worlds'),
            'worlds', 'rcjo2025_arena.world.xacro')
    else:  # 'empty' and anything unknown
        world_file = os.path.join(
            get_package_share_directory('sobit_light_description'),
            'worlds', 'empty_w_physics.sdf')

    gz_world_name = _gz_world_name(world_file)

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[gz.msgs.Clock",
            "/tf"    + "@tf2_msgs/msg/TFMessage"  + "[gz.msgs.Pose_V",
            # Entity control services
            f"/world/{gz_world_name}/control@ros_gz_interfaces/srv/ControlWorld",
            f"/world/{gz_world_name}/create@ros_gz_interfaces/srv/SpawnEntity",
            f"/world/{gz_world_name}/remove@ros_gz_interfaces/srv/DeleteEntity",
            f"/world/{gz_world_name}/set_pose@ros_gz_interfaces/srv/SetEntityPose",
        ],
        output='screen',
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare('sobit_light_bringup'), 'rviz', 'gazebo.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    effective_robot_name = robot_name if robot_id == 0 else f'{robot_name}_{robot_id}'

    headless_flag = ' --headless-rendering -s' if headless else ''
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': f'{headless_flag} -r -v 4 {world_file}'}.items(),
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('sobit_light_bringup'), 'launch', 'robot.launch.py'])
        ]),
        launch_arguments={
            'robot_name'                : effective_robot_name,
            'robot_coords_x'            : LaunchConfiguration('robot_coords_x').perform(context),
            'robot_coords_y'            : LaunchConfiguration('robot_coords_y').perform(context),
            'robot_coords_Y'            : LaunchConfiguration('robot_coords_Y').perform(context),
            'enable_mobile_base'        : _bool(LaunchConfiguration('enable_mobile_base'), context),
            'enable_head'               : _bool(LaunchConfiguration('enable_head'), context),
            'enable_arm'                : _bool(LaunchConfiguration('enable_arm'), context),
            'enable_hand'               : _bool(LaunchConfiguration('enable_hand'), context),
            'enable_gz'                 : 'True',
            'enable_gz_front_cam_color' : _bool(LaunchConfiguration('enable_gz_front_cam_color'), context),
            'enable_gz_back_cam_color'  : _bool(LaunchConfiguration('enable_gz_back_cam_color'), context),
            'enable_gz_head_cam_color'  : _bool(LaunchConfiguration('enable_gz_head_cam_color'), context),
            'enable_gz_head_cam_depth'  : _bool(LaunchConfiguration('enable_gz_head_cam_depth'), context),
            'enable_gz_hand_cam_color'  : _bool(LaunchConfiguration('enable_gz_hand_cam_color'), context),
            'enable_gz_hand_cam_depth'  : _bool(LaunchConfiguration('enable_gz_hand_cam_depth'), context),
            'enable_gz_lidar'           : _bool(LaunchConfiguration('enable_gz_lidar'), context),
            'enable_gz_imu'             : _bool(LaunchConfiguration('enable_gz_imu'), context),
            'enable_moveit'             : _bool(LaunchConfiguration('enable_moveit'), context),
            'enable_teleop'             : _bool(LaunchConfiguration('enable_teleop'), context),
            'enable_tf_prefix'          : _bool(LaunchConfiguration('enable_tf_prefix'), context),
            'use_moveit_rviz'           : _bool(LaunchConfiguration('use_moveit_rviz'), context),
        }.items(),
    )

    return [gz_sim, gz_bridge_node, robot, rviz_node]
