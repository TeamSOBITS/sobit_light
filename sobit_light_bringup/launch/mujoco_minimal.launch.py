import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name',                 default_value='sobit_light'),
        DeclareLaunchArgument('robot_id',                   default_value='0'),
        DeclareLaunchArgument('world_model',                default_value='rcjo2025_arena',
                              description='World under <asset_root>/mjcf (e.g. rcjo2025_arena, rcjo2026_arena, '
                                          'rcw2026_arena, empty) or an absolute .xml path'),
        DeclareLaunchArgument('world_closed',               default_value='false',
                              description='Load the <world_model>_closed variant (walls + ceiling + lights)'),
        DeclareLaunchArgument('robot_coords_x',             default_value='-5.5'),
        DeclareLaunchArgument('robot_coords_y',             default_value='1.5'),
        DeclareLaunchArgument('robot_coords_Y',             default_value='0.0'),
        DeclareLaunchArgument('asset_root',                 default_value='',
                              description='Directory holding mjcf/. Empty = $SOBITS_SIM_ASSET_ROOT, '
                                          'else the sobits_gazebo_worlds source export/ directory'),
        DeclareLaunchArgument('headless',                   default_value='false',
                              description='Run MuJoCo without the Simulate window'),
        DeclareLaunchArgument('enable_viz',                 default_value='',
                              description='Viewer to start: rerun, rviz, foxglove, or empty for none'),
        DeclareLaunchArgument('enable_moveit_rviz',         default_value='false'),
        DeclareLaunchArgument('enable_mobile_base',         default_value='true'),
        DeclareLaunchArgument('enable_head',                default_value='true'),
        DeclareLaunchArgument('enable_arm',                 default_value='true'),
        DeclareLaunchArgument('enable_hand',                default_value='true'),
        # Sensor flags keep gz_minimal's enable_gz_* names; here they mean "sensor on"
        DeclareLaunchArgument('enable_gz_front_cam_color',  default_value='true'),
        DeclareLaunchArgument('enable_gz_back_cam_color',   default_value='true'),
        DeclareLaunchArgument('enable_gz_head_cam_color',   default_value='true'),
        DeclareLaunchArgument('enable_gz_head_cam_depth',   default_value='true'),
        DeclareLaunchArgument('enable_gz_hand_cam_color',   default_value='true'),
        DeclareLaunchArgument('enable_gz_hand_cam_depth',   default_value='true'),
        DeclareLaunchArgument('enable_gz_lidar',            default_value='true'),
        DeclareLaunchArgument('enable_moveit',              default_value='true'),
        DeclareLaunchArgument('enable_teleop',              default_value='true'),
        DeclareLaunchArgument('enable_tf_prefix',           default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])


def _bool(lc, context):
    """Normalize CLI true/True/1 → 'True', false/False/0 → 'False' for xacro + robot.launch.py."""
    return 'True' if lc.perform(context).lower() in ('true', '1', 'yes') else 'False'


def _viewer(context, robot_name):
    """Return the launch action for the chosen viewer, or nothing."""
    choice = LaunchConfiguration('enable_viz').perform(context).strip().lower()
    if not choice:
        return []
    if choice not in ('rerun', 'rviz', 'foxglove'):
        raise RuntimeError(
            f"enable_viz must be rerun, rviz, foxglove or empty, not '{choice}'")
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare(f'sobits_viz_{choice}'), 'launch', f'{choice}.launch.py'])),
        launch_arguments={
            'robot_name': robot_name,
            'use_sim_time': 'true',
            'enable_tf_prefix': _bool(
                LaunchConfiguration('enable_tf_prefix'), context),
        }.items(),
    )]


def _worlds_src():
    """sobits_gazebo_worlds source dir: export/ and scripts/ are not installed, symlink-install
    links package.xml back to the source package."""
    pkg_xml = os.path.join(get_package_share_directory('sobits_gazebo_worlds'), 'package.xml')
    return os.path.dirname(os.path.realpath(pkg_xml))


def _asset_root(context):
    """Resolve the directory holding mjcf/ (same rule as isaac_minimal's usd/)."""
    root = LaunchConfiguration('asset_root').perform(context) or os.environ.get('SOBITS_SIM_ASSET_ROOT', '')
    if not root:
        root = os.path.join(_worlds_src(), 'export')
    root = os.path.abspath(os.path.expanduser(root))
    if not os.path.isdir(os.path.join(root, 'mjcf')):
        raise RuntimeError(
            f"No mjcf/ directory under '{root}'. Pass asset_root:=<dir containing mjcf/> "
            'or set SOBITS_SIM_ASSET_ROOT.')
    return root


def _build_scene(world_path, robot_xml, pose, out_path):
    """Merge world + robot MJCF into one scene file at launch time; returns its path."""
    script = os.path.join(_worlds_src(), 'scripts', 'mujoco_scene.py')
    cmd = ['python3', script, '--world', world_path, '--robot', robot_xml,
           '--pose', *pose, '--out', out_path]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True)
    except OSError as e:
        raise RuntimeError(f'[mujoco_minimal] cannot run {script}: {e}')
    if result.returncode != 0:
        raise RuntimeError(
            f'[mujoco_minimal] scene build failed (exit {result.returncode}): {" ".join(cmd)}\n'
            f'{result.stderr.strip() or result.stdout.strip()}')
    lines = result.stdout.strip().splitlines()
    scene = lines[-1].strip() if lines else out_path
    if not os.path.isfile(scene):
        raise RuntimeError(f"[mujoco_minimal] scene build reported '{scene}', which does not exist")
    return scene


def launch_setup(context, *args, **kwargs):
    robot_name  = LaunchConfiguration('robot_name').perform(context)
    robot_id    = int(LaunchConfiguration('robot_id').perform(context))
    world_model = LaunchConfiguration('world_model').perform(context)
    x, y, yaw = [LaunchConfiguration(f'robot_coords_{k}').perform(context) for k in ('x', 'y', 'Y')]

    effective_robot_name = robot_name if robot_id == 0 else f'{robot_name}_{robot_id}'
    asset_root = _asset_root(context)

    if os.path.isabs(world_model):
        world_path = world_model
    else:
        suffix = '_closed' if _bool(LaunchConfiguration('world_closed'), context) == 'True' else ''
        world_path = os.path.join(asset_root, 'mjcf', world_model + suffix, f'{world_model}{suffix}.xml')
    robot_xml = os.path.join(asset_root, 'mjcf', 'robots', robot_name, f'{robot_name}.xml')
    for path in (world_path, robot_xml):
        if not os.path.isfile(path):
            raise RuntimeError(f"MuJoCo asset not found: '{path}'")
    # SOBIT LIGHT spawns on the floor (no z argument, as in gz_minimal)
    scene = _build_scene(world_path, robot_xml, [x, y, '0', yaw],
                         os.path.join(os.path.dirname(world_path), f'scene_{effective_robot_name}.xml'))

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('sobit_light_bringup'), 'launch', 'robot.launch.py'])
        ]),
        launch_arguments={
            'robot_name'               : effective_robot_name,
            'simulator'                : 'mujoco',
            'mujoco_model'             : scene,
            'mujoco_headless'          : _bool(LaunchConfiguration('headless'), context),
            'robot_coords_x'           : x,
            'robot_coords_y'           : y,
            'robot_coords_Y'           : yaw,
            'enable_mobile_base'       : _bool(LaunchConfiguration('enable_mobile_base'), context),
            'enable_head'              : _bool(LaunchConfiguration('enable_head'), context),
            'enable_arm'               : _bool(LaunchConfiguration('enable_arm'), context),
            'enable_hand'              : _bool(LaunchConfiguration('enable_hand'), context),
            'enable_gz_front_cam_color': _bool(LaunchConfiguration('enable_gz_front_cam_color'), context),
            'enable_gz_back_cam_color' : _bool(LaunchConfiguration('enable_gz_back_cam_color'), context),
            'enable_gz_head_cam_color' : _bool(LaunchConfiguration('enable_gz_head_cam_color'), context),
            'enable_gz_head_cam_depth' : _bool(LaunchConfiguration('enable_gz_head_cam_depth'), context),
            'enable_gz_hand_cam_color' : _bool(LaunchConfiguration('enable_gz_hand_cam_color'), context),
            'enable_gz_hand_cam_depth' : _bool(LaunchConfiguration('enable_gz_hand_cam_depth'), context),
            'enable_gz_lidar'          : _bool(LaunchConfiguration('enable_gz_lidar'), context),
            'enable_moveit'            : _bool(LaunchConfiguration('enable_moveit'), context),
            'enable_teleop'            : _bool(LaunchConfiguration('enable_teleop'), context),
            'enable_tf_prefix'         : _bool(LaunchConfiguration('enable_tf_prefix'), context),
            'enable_moveit_rviz'       : _bool(LaunchConfiguration('enable_moveit_rviz'), context),
        }.items(),
    )

    return [robot] + _viewer(context, 'sobit_light')
