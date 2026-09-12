import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


# Which enable_* flag owns each entry of moveit_controllers.yaml.
_CONTROLLER_OWNER = {
    'arm_position_controller': 'enable_arm',
    'hand_position_controller': 'enable_hand',
    'head_position_controller': 'enable_head',
}


def _prune_disabled_controllers(trajectory_execution, module_mappings):
    """Strip disabled modules' controllers so MoveIt never advertises a
    controller that no spawner will create."""
    scm = trajectory_execution.get('moveit_simple_controller_manager')
    if not scm:
        return

    def _enabled(name):
        owner = _CONTROLLER_OWNER.get(name)
        if owner is None:
            return True
        return module_mappings.get(owner, 'true').lower() in ('true', '1', 'yes')

    kept = [n for n in scm.get('controller_names', []) if _enabled(n)]
    for name in list(scm):
        if name != 'controller_names' and name not in kept:
            del scm[name]
    scm['controller_names'] = kept


def _launch_setup(context, *args, **kwargs):

    package_name_moveit_config = 'sobit_light_moveit_config'

    pkg_share_moveit_config = FindPackageShare(package=package_name_moveit_config).find(package_name_moveit_config)

    # Configuration file paths
    # One xacro SRDF for every configuration; teleop is just an argument.
    srdf_model_path = os.path.join(pkg_share_moveit_config, 'config', 'sobit_light.srdf.xacro')
    moveit_controllers_file_path = os.path.join(pkg_share_moveit_config, 'config', 'moveit_controllers.yaml')
    joint_limits_file_path = os.path.join(pkg_share_moveit_config, 'config', 'joint_limits.yaml')
    kinematics_file_path = os.path.join(pkg_share_moveit_config, 'config', 'kinematics.yaml')
    pilz_cartesian_limits_file_path = os.path.join(pkg_share_moveit_config, 'config', 'pilz_cartesian_limits.yaml')
    rviz_config_file = os.path.join(pkg_share_moveit_config, 'rviz', 'moveit.rviz')
    sensors_file_path = os.path.join(pkg_share_moveit_config, 'config', 'sensors_3d.yaml')

    # Launch configuration variables
    robot_name = LaunchConfiguration('robot_name')
    robot_name_str = robot_name.perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    enable_tf_prefix = LaunchConfiguration('enable_tf_prefix').perform(context).lower() in ('true', '1', 'yes')

    # Module switches shared by the URDF and the SRDF.
    module_mappings = {
        name: LaunchConfiguration(name).perform(context)
        for name in (
            'enable_mobile_base',
            'enable_arm',
            'enable_hand',
            'enable_head',
        )
    }
    # Forward the module flags so the SRDF matches the spawned robot: absent
    # modules lose their planning groups, states, end effector and collision pairs.
    srdf_mappings = dict(
        module_mappings,
        enable_teleop=LaunchConfiguration('enable_teleop').perform(context),
    )
    # Without an explicit robot_description the builder expands the URDF with its
    # xacro defaults (every module on), mismatching the spawned robot.
    urdf_model_path = os.path.join(
        FindPackageShare(package='sobit_light_description').find('sobit_light_description'),
        'robots', 'sobit_light_robot.urdf.xacro')
    # The URDF evaluates its flags as bare Python (${$(arg enable_head)}), so they
    # must be capitalized literals; the SRDF lowercases and compares as strings.
    def _py_bool(value):
        return 'True' if value.lower() in ('true', '1', 'yes') else 'False'

    # enable_gz picks the gz plugin blocks; it tracks use_sim_time, which the
    # bringup already derives from enable_gz.
    urdf_mappings = {k: _py_bool(v) for k, v in module_mappings.items()}
    urdf_mappings.update(
        robot_name=robot_name_str,
        enable_gz=_py_bool(use_sim_time.perform(context)),
        enable_tf_prefix='True' if enable_tf_prefix else 'False',
    )

    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("sobit_light", package_name=package_name_moveit_config)
        .robot_description(file_path=urdf_model_path, mappings=urdf_mappings)
        .trajectory_execution(file_path=moveit_controllers_file_path)
        .robot_description_semantic(file_path=srdf_model_path, mappings=srdf_mappings)
        .joint_limits(file_path=joint_limits_file_path)
        .robot_description_kinematics(file_path=kinematics_file_path)
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl"
        )
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
        # .sensors_3d(file_path=sensors_file_path)
        .to_moveit_configs()
    )

    # Drop controllers whose module is disabled: MoveIt otherwise keeps trying to
    # connect to a FollowJointTrajectory action that no spawner ever created.
    _prune_disabled_controllers(moveit_config.trajectory_execution, module_mappings)

    # Add frame_prefix so MoveIt maps URDF frames to TF frames; must match the
    # robot_state_publisher frame_prefix set by robot.launch.py.
    config_dict = moveit_config.to_dict()
    frame_prefix_params = {}
    if enable_tf_prefix:
        config_dict['robot_description_planning.frame_prefix'] = robot_name_str + '/'
        frame_prefix_params['robot_description_planning.frame_prefix'] = robot_name_str + '/'

    # move_group node
    start_move_group_node_cmd = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=robot_name,
        output="screen",
        parameters=[
            config_dict,
            {'use_sim_time': use_sim_time},
        ],
    )

    # RViz
    start_rviz_node_cmd = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        arguments=["-d", rviz_config_file],
        output="screen",
        # These three resolve at the node root, unlike the plugin's other
        # topics which pick up Move Group Namespace from the RViz config.
        remappings=[
            ("planning_scene",          f"/{robot_name_str}/planning_scene"),
            ("planning_scene_world",    f"/{robot_name_str}/planning_scene_world"),
            ("recognized_object_array", f"/{robot_name_str}/recognized_object_array"),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            # moveit_config.sensors_3d,
            {'use_sim_time': use_sim_time,
             **frame_prefix_params},
        ],
    )

    exit_event_handler = RegisterEventHandler(
        condition=IfCondition(use_rviz),
        event_handler=OnProcessExit(
            target_action=start_rviz_node_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
        ),
    )

    return [
        start_move_group_node_cmd,
        start_rviz_node_cmd,
        exit_event_handler,
    ]


def generate_launch_description():

    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value='sobit_light',
        description='Robot name used as namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz')

    declare_enable_teleop_cmd = DeclareLaunchArgument(
        name='enable_teleop',
        default_value='true',
        description='Base driven directly (Kachaka/teleop): no base virtual joint in the SRDF. '
                    'Set false to declare the planar odom virtual joint for base-aware planning')

    declare_enable_mobile_base_cmd = DeclareLaunchArgument(
        name='enable_mobile_base',
        default_value='true',
        description='Whether the mobile base is present; disables its SRDF virtual joint when false')

    # Module switches for the SRDF xacro; names match robot.launch.py.
    declare_module_cmds = [
        DeclareLaunchArgument(
            name=name,
            default_value='true',
            description=f'Whether {label} is present; disables its SRDF planning groups when false')
        for name, label in (
            ('enable_arm', 'the arm'),
            ('enable_hand', 'the gripper'),
            ('enable_head', 'the head'),
        )
    ]

    declare_enable_tf_prefix_cmd = DeclareLaunchArgument(
        name='enable_tf_prefix',
        default_value='true',
        description='Prefix TF frames with the robot name; must match robot.launch.py')

    return LaunchDescription([
        declare_robot_name_cmd,
        declare_use_sim_time_cmd,
        declare_use_rviz_cmd,
        declare_enable_teleop_cmd,
        declare_enable_mobile_base_cmd,
        *declare_module_cmds,
        declare_enable_tf_prefix_cmd,
        OpaqueFunction(function=_launch_setup),
    ])
