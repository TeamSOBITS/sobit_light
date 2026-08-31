from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def _bool(lc, context):
    """Normalize CLI true/True/1 → 'True', else → 'False' for xacro + robot.launch.py."""
    return 'True' if lc.perform(context).lower() in ('true', '1', 'yes') else 'False'


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name',           default_value='sobit_light'),
        DeclareLaunchArgument('robot_id',             default_value='0'),
        DeclareLaunchArgument('use_rviz',             default_value='true'),
        DeclareLaunchArgument('use_moveit_rviz',      default_value='true'),
        DeclareLaunchArgument('enable_mobile_base',   default_value='true'),
        DeclareLaunchArgument('enable_head',          default_value='true'),
        DeclareLaunchArgument('enable_arm',           default_value='true'),
        DeclareLaunchArgument('enable_hand',          default_value='true'),
        DeclareLaunchArgument('enable_real_head_cam', default_value='true'),
        DeclareLaunchArgument('enable_real_hand_cam', default_value='true'),
        DeclareLaunchArgument('enable_moveit',        default_value='true'),
        DeclareLaunchArgument('enable_teleop',        default_value='true'),
        DeclareLaunchArgument('enable_tf_prefix',     default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    robot_id   = int(LaunchConfiguration('robot_id').perform(context))

    effective_robot_name = robot_name if robot_id == 0 else f'{robot_name}_{robot_id}'

    rviz_config = PathJoinSubstitution([
        FindPackageShare('sobit_light_bringup'), 'rviz', 'real.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('sobit_light_bringup'), 'launch', 'robot.launch.py'])
        ]),
        launch_arguments={
            'robot_name'           : effective_robot_name,
            'enable_mobile_base'   : _bool(LaunchConfiguration('enable_mobile_base'), context),
            'enable_head'          : _bool(LaunchConfiguration('enable_head'), context),
            'enable_arm'           : _bool(LaunchConfiguration('enable_arm'), context),
            'enable_hand'          : _bool(LaunchConfiguration('enable_hand'), context),
            'enable_real_head_cam' : _bool(LaunchConfiguration('enable_real_head_cam'), context),
            'enable_real_hand_cam' : _bool(LaunchConfiguration('enable_real_hand_cam'), context),
            'enable_gz'            : 'False',  # never Gazebo on real hardware
            'enable_moveit'        : _bool(LaunchConfiguration('enable_moveit'), context),
            'enable_teleop'        : _bool(LaunchConfiguration('enable_teleop'), context),
            'enable_tf_prefix'     : _bool(LaunchConfiguration('enable_tf_prefix'), context),
            'use_moveit_rviz'      : _bool(LaunchConfiguration('use_moveit_rviz'), context),
        }.items(),
    )

    return [robot, rviz_node]
