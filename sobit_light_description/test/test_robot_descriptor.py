"""Checks config/sobit_light.robot.yaml against the URDF, the controller YAMLs and gz sensors."""
from pathlib import Path

import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
CONFIG = PKG / 'config' / 'sobit_light.robot.yaml'
XACRO = PKG / 'robots' / 'sobit_light_robot.urdf.xacro'
CTRL_TYPES = {
    ('trajectory', 'position'): 'joint_trajectory_controller/JointTrajectoryController',
    ('group', 'position'): 'position_controllers/JointGroupPositionController',
    ('group', 'velocity'): 'velocity_controllers/JointGroupVelocityController',
}
# Non-default xacro args that change which sensors exist.
VARIANTS = {
    'default': {},
    'no_head': {'enable_head': 'False'},
    'no_hand': {'enable_hand': 'False'},
    'no_base': {'enable_mobile_base': 'False'},
}

try:
    import sobits_robot_descriptor as srd
except ImportError:
    sys.path.insert(0, str(PKG.parents[1] / 'sobits_robot_descriptor'))
    try:
        import sobits_robot_descriptor as srd
    except ImportError:
        pytest.skip('sobits_robot_descriptor not available', allow_module_level=True)


def _render(overrides):
    """Return URDF xml text for the variant; skip if xacro or kachaka_description is missing."""
    mappings = {'enable_gz': 'True', 'enable_tf_prefix': 'false', **overrides}
    try:
        import xacro
        return xacro.process_file(str(XACRO), mappings=mappings).toxml()
    except Exception as exc:  # no xacro, or kachaka_description is not installed
        pytest.skip(f'cannot render {overrides}; xacro or kachaka_description missing: {exc!r}')


def _controllers_yaml(name):
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('sobit_light_control')) / 'config' / name
    except Exception:
        path = PKG.parent / 'sobit_light_control' / 'config' / name
    if not path.is_file():
        pytest.skip('sobit_light_control not installed and not found in the source tree')
    return yaml.safe_load(path.read_text())


def _model(overrides):
    return ET.fromstring(_render(overrides).encode())


def _joints(urdf):
    return {j.get('name'): j for j in urdf.findall('joint')}


def _links(urdf):
    return {link.get('name') for link in urdf.findall('link')}


@pytest.fixture(params=list(VARIANTS))
def variant(request):
    return request.param


@pytest.fixture
def desc():
    return srd.load_file(str(CONFIG))


@pytest.fixture(scope='module')
def default_urdf():
    return _model({})


def test_validate_clean():
    data = yaml.safe_load(CONFIG.read_text())
    assert srd.validate(data) == []


def test_group_joints_movable(desc, default_urdf):
    joints = _joints(default_urdf)
    for g in desc.groups:
        for name in g.joints:
            assert name in joints, f'{g.name}: {name} not in URDF'
            assert joints[name].get('type') != 'fixed', f'{g.name}: {name} is fixed'


def test_uncommanded_joints_mimic(desc, default_urdf):
    joints = _joints(default_urdf)
    for g in desc.groups:
        for name in g.uncommanded_joints:
            assert name in joints, f'{g.name}: {name} not in URDF'
            mimic = joints[name].find('mimic')
            assert mimic is not None, f'{name} has no <mimic>'
            assert mimic.get('joint') in g.joints, f'{name} mimics a joint outside {g.name}'


def test_every_moving_joint_is_accounted_for(desc, default_urdf):
    known = set(desc.joints(include_uncommanded=True)) | set(desc.all_excluded_joints)
    moving = {n for n, j in _joints(default_urdf).items() if j.get('type') != 'fixed'}
    assert moving == known, f'unlisted: {sorted(moving - known)}, stale: {sorted(known - moving)}'


def test_excluded_joints_exist_and_are_not_in_a_group(desc, default_urdf):
    joints = _joints(default_urdf)
    assert set(desc.all_excluded_joints) <= set(joints)
    assert not set(desc.all_excluded_joints) & set(desc.joints(include_uncommanded=True))


def test_frames_are_links(desc, default_urdf):
    links = _links(default_urdf)
    frames = {desc.base_frame}
    frames |= {e.ee_link for e in desc.ee} | {e.reference_frame for e in desc.ee}
    for cam in desc.cameras:
        frames.add(cam.frame)
        frames |= {s.frame for s in (cam.color, cam.depth) if s}
    frames |= {lidar.frame for lidar in desc.lidars} | {i.frame for i in desc.imus}
    assert not frames - links, f'frames missing from URDF: {sorted(frames - links)}'


def test_sensors_follow_xacro_args(variant):
    """A sensor is in the descriptor exactly when its mount link exists in the URDF."""
    overrides = VARIANTS[variant]
    full = srd.load_file(str(CONFIG))
    desc = srd.load_file(str(CONFIG), args=overrides)
    links = _links(_model(overrides))
    kept = ({c.name for c in desc.cameras} | {lidar.name for lidar in desc.lidars} |
            {i.name for i in desc.imus})
    for entry in list(full.cameras) + list(full.lidars) + list(full.imus):
        has_link = entry.frame in links
        assert (entry.name in kept) == has_link, \
            f'{entry.name}: kept={entry.name in kept}, {entry.frame} in URDF={has_link}'
    assert kept, 'no sensor left'


def test_real_controllers_match_descriptor(desc):
    ctrl = _controllers_yaml('real_controllers.yaml')
    manager = ctrl['/**/controller_manager']['ros__parameters']
    for g in desc.groups:
        key = f'/**/{g.controller}'
        assert key in ctrl, f'{g.name}: {g.controller} missing in real_controllers.yaml'
        assert set(ctrl[key]['ros__parameters']['joints']) == set(g.joints), g.name
        assert manager[g.controller]['type'] == CTRL_TYPES[(g.interface, g.kind)], g.name
    spawned = {k for k, v in manager.items() if isinstance(v, dict)}
    expected = {g.controller for g in desc.groups} | {'joint_state_broadcaster'}
    assert spawned == expected, f'controllers not described: {sorted(spawned ^ expected)}'
    # The real base is driven by the Kachaka driver, so it has no wheel controller here.
    assert 'wheel_controller' not in manager


def test_gz_controllers_match_descriptor(desc):
    ctrl = _controllers_yaml('gz_controllers.yaml')
    manager = ctrl['/**/controller_manager']['ros__parameters']
    for g in desc.groups:
        key = f'/**/{g.controller}'
        assert set(ctrl[key]['ros__parameters']['joints']) == set(g.joints), g.name
        assert manager[g.controller]['type'] == CTRL_TYPES[(g.interface, g.kind)], g.name
    spawned = {k for k, v in manager.items() if isinstance(v, dict)}
    expected = ({g.controller for g in desc.groups} | {'joint_state_broadcaster'} |
                {c.controller for c in desc.mobile_base.controllers})
    assert spawned == expected, f'controllers not described: {sorted(spawned ^ expected)}'
    assert desc.mobile_base.controllers, 'no base controller described'
    for c in desc.mobile_base.controllers:
        assert c.interface == 'diff_drive', c.name
        assert manager[c.controller]['type'] == 'diff_drive_controller/DiffDriveController', c.name
        params = ctrl[f'/**/{c.controller}']['ros__parameters']
        assert params['left_wheel_names'] == [c.joints[0]], c.name
        assert params['right_wheel_names'] == [c.joints[1]], c.name
        assert params['wheel_radius'] == pytest.approx(c.wheel_radius), c.name
        assert params['wheel_separation'] == pytest.approx(c.wheel_separation), c.name
    wheel = ctrl['/**/wheel_controller']['ros__parameters']
    assert wheel['odom_frame_id'] == desc.odom_frame
    assert wheel['base_frame_id'] == desc.base_frame


def test_gz_sensors_match_descriptor(desc, default_urdf):
    ns = desc.namespace
    # The bridges are what rename `<ns>/<cam>/color` to `<cam>/color/image_raw`.
    color = {f'{ns}/{c.color.raw_topic.rsplit("/", 1)[0]}': c.color.frame
             for c in desc.cameras if c.color and '/color/' in c.color.raw_topic}
    base_cam = {f'{ns}/base_{c.color.raw_topic.split("/")[0]}/color': c.color.frame
                for c in desc.cameras if c.color and '/color/' not in c.color.raw_topic}
    depth = {f'{ns}/{c.depth.raw_topic.rsplit("/", 1)[0]}': c.depth.frame
             for c in desc.cameras if c.depth}
    scans = {f'{ns}/{lidar.scan_topic}': lidar.frame for lidar in desc.lidars}
    imus = {f'{ns}/{i.topic}': i.frame for i in desc.imus}
    links = _links(default_urdf)
    seen = set()
    for s in default_urdf.iter('sensor'):
        topic = s.findtext('topic')
        frame = s.findtext('gz_frame_id').removeprefix(f'{ns}/')
        assert frame in links, f'{s.get("name")}: gz_frame_id {frame} is not a link'
        kind = s.get('type')
        if kind == 'camera':
            assert {**color, **base_cam}.get(topic) == frame, f'{s.get("name")}: {topic} / {frame}'
        elif kind == 'depth':
            # gz stamps the depth sensor frame, the bridge restamps it to the optical frame.
            assert topic in depth, f'{s.get("name")}: {topic}'
            assert depth[topic].replace('_optical_frame', '_frame') == frame
        elif kind == 'gpu_lidar':
            assert scans.get(topic) == frame, f'{s.get("name")}: {topic} / {frame}'
        elif kind == 'imu':
            assert imus.get(topic) == frame, f'{s.get("name")}: {topic} / {frame}'
        else:
            continue
        seen.add(topic)
    wanted = set(color) | set(base_cam) | set(depth) | set(scans) | set(imus)
    assert wanted <= seen, f'streams without a gz sensor: {sorted(wanted - seen)}'
