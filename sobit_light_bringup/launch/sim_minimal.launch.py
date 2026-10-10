import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

SIMULATORS = ('gz', 'isaac', 'mujoco')

# Union of the <sim>_minimal arguments, forwarded only to the launchers that declare them. Empty values are
# not forwarded, so world_model falls back to each launcher's own default (gz: empty, isaac/mujoco: rcjo2025_arena)
ARGUMENTS = [
    ('robot_name',                'sobit_light', 'Robot model (and namespace when robot_id is 0)'),
    ('robot_id',                  '0',           'Non-zero appends _<id> to the namespace (multi-robot)'),
    ('world_model',               '',            'World. gz: empty | wrs | small_house | rcjo2025; isaac / mujoco: '
                                                 'rcjo2025_arena, rcjo2026_arena, rcw2026_arena, empty, ... '
                                                 'Empty = the launcher default'),
    ('world_closed',              'false',       'isaac / mujoco: closed variant of the arena (walls + ceiling + lights)'),
    ('robot_coords_x',            '-5.5',        'Spawn x [m]'),
    ('robot_coords_y',            '1.5',         'Spawn y [m]'),
    ('robot_coords_Y',            '0.0',         'Spawn yaw [rad]'),
    ('headless',                  'false',       'gz / mujoco: no GUI (gz headless rendering, no MuJoCo Simulate window)'),
    ('asset_root',                '',            'isaac / mujoco: dir holding usd/ and mjcf/. Empty = $SOBITS_SIM_ASSET_ROOT or sobits_gazebo_worlds export/'),
    ('robot_usd',                 '',            'isaac: robot USD. Empty = <asset_root>/usd/robots/<robot_name>/<robot_name>.usd'),
    ('spawn_only',                'false',       'isaac: skip load-world (world already open in the Isaac GUI)'),
    ('wait_timeout',              '120',         'isaac: seconds to wait for the simulation_interfaces services'),
    ('enable_viz',                '',            'Viewer to start: rerun, rviz, foxglove, or empty for none'),
    ('enable_teleop',             'true',        'Teleop (joy) through MoveIt'),
    ('enable_mobile_base',        'true',        'Module: Kachaka base (+ base cameras, lidar)'),
    ('enable_head',               'true',        'Module: pan-tilt head (+ head camera)'),
    ('enable_arm',                'true',        'Module: arm'),
    ('enable_hand',               'true',        'Module: hand (+ hand camera)'),
    ('enable_gz_front_cam_color', 'true',        'Sensor: base front camera'),
    ('enable_gz_back_cam_color',  'true',        'Sensor: base back camera'),
    ('enable_gz_head_cam_color',  'true',        'Sensor: head camera color stream'),
    ('enable_gz_head_cam_depth',  'true',        'Sensor: head camera depth stream (+ points)'),
    ('enable_gz_hand_cam_color',  'true',        'Sensor: hand camera color stream'),
    ('enable_gz_hand_cam_depth',  'true',        'Sensor: hand camera depth stream (+ points)'),
    ('enable_gz_lidar',           'true',        'Sensor: base lidar'),
    ('enable_gz_imu',             'true',        'gz: base IMU (no IMU in isaac / mujoco)'),
    ('enable_moveit',             'true',        'Start MoveIt'),
    ('enable_moveit_rviz',        'false',       "MoveIt's own planning-scene RViz"),
    ('enable_tf_prefix',          'true',        'Prefix TF frames with <robot_name>/'),
]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulator', default_value='gz', choices=list(SIMULATORS),
                              description='Simulator to run: gz | isaac | mujoco'),
        *[DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in ARGUMENTS],
        OpaqueFunction(function=launch_setup),
    ])


def _declared(path):
    """Argument names the launch file declares, read from its own generate_launch_description()."""
    spec = importlib.util.spec_from_file_location(os.path.basename(path).split('.')[0], path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return {arg.name for arg in module.generate_launch_description().get_launch_arguments()}


def launch_setup(context, *args, **kwargs):
    simulator = LaunchConfiguration('simulator').perform(context).strip().lower()
    path = os.path.join(get_package_share_directory('sobit_light_bringup'),
                        'launch', 'include', f'{simulator}_minimal.launch.py')
    declared = _declared(path)
    forwarded = [(name, LaunchConfiguration(name).perform(context)) for name, _, _ in ARGUMENTS if name in declared]
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=[(name, value) for name, value in forwarded if value],
    )]
