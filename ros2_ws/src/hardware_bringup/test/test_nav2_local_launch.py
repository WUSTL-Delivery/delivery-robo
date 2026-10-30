"""TDM-21: config/nav2_local.yaml and the parameter patching in launch/nav2_local.launch.py.

Needs launch_ros importable (a sourced ROS environment); skipped otherwise.
"""
import importlib.util
import math
import os

import pytest

pytest.importorskip('launch_ros')
yaml = pytest.importorskip('yaml')

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP = '/maps/boundary.yaml'


def _load_launch_module():
    path = os.path.join(PKG, 'launch', 'nav2_local.launch.py')
    spec = importlib.util.spec_from_file_location('tdm21_nav2_local_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


nav2_local = _load_launch_module()


@pytest.fixture
def tdm21_params():
    with open(os.path.join(PKG, 'config', 'nav2_local.yaml')) as f:
        return yaml.safe_load(f)


def _costmap(params):
    return params['global_costmap']['global_costmap']['ros__parameters']


def test_costmap_frames_and_layers(tdm21_params):
    costmap = _costmap(tdm21_params)
    assert costmap['global_frame'] == 'odom'      # sim map->odom has z = 2
    assert costmap['robot_base_frame'] == 'base_link'
    assert costmap['rolling_window'] is True
    assert costmap['plugins'] == ['static_layer', 'obstacle_layer', 'inflation_layer']
    assert costmap['static_layer']['map_topic'] == '/map'
    # Jazzy StaticLayer clears the footprint at the wrong place in a rolling odom costmap
    assert costmap['static_layer']['footprint_clearing_enabled'] is False


def test_every_source_goes_stale(tdm21_params):
    obstacle = _costmap(tdm21_params)['obstacle_layer']
    names = {n for mode in nav2_local.SOURCE_MODES.values() for n in mode}
    for name in names:
        assert obstacle[name]['expected_update_rate'] > 0.0, name


# (marking, clearing, expected_update_rate): TDM-52, one source per mode gates "current"
@pytest.mark.parametrize('sources, expected', [
    ('fused', {'fused': (True, False, 1.0), 'lidar': (False, True, 0.0),
               'camera': (False, True, 0.0)}),
    ('separate', {'lidar': (True, True, 0.0), 'camera': (True, True, 0.0)}),
    ('lidar', {'lidar': (True, True, 0.5)}),
    ('auto', {'fused': (True, False, 1.0), 'lidar': (False, True, 0.0),
              'camera': (False, True, 0.0)}),
])
def test_sources_wiring(tdm21_params, sources, expected):
    patched = nav2_local.patch_params(tdm21_params, map_yaml=MAP, sources=sources)
    obstacle = _costmap(patched)['obstacle_layer']
    assert obstacle['observation_sources'].split() == list(expected)
    for name, (marking, clearing, rate) in expected.items():
        assert (obstacle[name]['marking'], obstacle[name]['clearing']) == (marking, clearing)
        assert obstacle[name]['expected_update_rate'] == rate, name
    assert obstacle['lidar']['topic'] == '/scan_filtered'
    assert patched['map_server']['ros__parameters']['yaml_filename'] == MAP
    # the input tree keeps every source's rate
    assert _costmap(tdm21_params)['obstacle_layer']['camera']['expected_update_rate'] == 1.0


@pytest.mark.parametrize('sources', ['fused', 'separate', 'lidar', 'auto'])
def test_tdm52_a_lone_raw_sensor_never_gates_when_there_are_two(tdm21_params, sources):
    # The user's rule: one of lidar / camera missing -> keep planning on the other.
    obstacle = _costmap(nav2_local.patch_params(
        tdm21_params, map_yaml=MAP, sources=sources))['obstacle_layer']
    names = obstacle['observation_sources'].split()
    gating = [n for n in names if obstacle[n]['expected_update_rate'] > 0.0]
    if {'lidar', 'camera'} <= set(names):
        assert 'lidar' not in gating and 'camera' not in gating
    # at most one gate, and it is the only obstacle input that must be there
    assert gating in ([], ['fused'], ['lidar'])
    assert gating or sources == 'separate'   # separate: local_planner stops on all-silent


@pytest.mark.parametrize('sources, sim, topics', [
    ('fused', False, ['/scan_fused', '/scan_filtered', '/camera/scan']),
    ('auto', False, ['/scan_fused', '/scan_filtered', '/camera/scan']),
    ('separate', False, ['/scan_filtered', '/camera/scan']),
    ('lidar', False, ['/scan_filtered']),
    ('auto', True, ['/scan']),
    ('fused', True, ['/scan_fused', '/scan', '/camera/scan']),
])
def test_tdm52_local_planner_watches_the_costmap_scan_topics(tdm21_params, sources, sim, topics):
    patched = nav2_local.patch_params(tdm21_params, map_yaml=MAP, sources=sources, sim=sim)
    assert nav2_local.scan_topics(patched) == topics
    assert nav2_local.local_planner_params(patched, sim=sim)['scan_topics'] == topics


def test_sim_means_sim_time_and_lidar_on_scan(tdm21_params):
    patched = nav2_local.patch_params(tdm21_params, map_yaml=MAP, sources='auto', sim=True)
    obstacle = _costmap(patched)['obstacle_layer']
    assert obstacle['observation_sources'] == 'lidar'
    assert obstacle['lidar']['topic'] == '/scan'
    for node in ('map_server', 'planner_server', 'lifecycle_manager_nav2_local'):
        assert patched[node]['ros__parameters']['use_sim_time'] is True
    assert _costmap(patched)['use_sim_time'] is True
    # the input tree is left alone
    assert tdm21_params['map_server']['ros__parameters']['use_sim_time'] is False


def test_scan_topic_override(tdm21_params):
    patched = nav2_local.patch_params(tdm21_params, map_yaml=MAP, sources='lidar',
                                      scan_topic='/scan', sim=False)
    assert _costmap(patched)['obstacle_layer']['lidar']['topic'] == '/scan'


def test_standalone_costmap_fallback(tdm21_params):
    manager = nav2_local.patch_params(tdm21_params, map_yaml=MAP, sources='lidar',
                                      planner=False)['lifecycle_manager_nav2_local']['ros__parameters']
    assert manager['node_names'] == ['map_server', 'global_costmap/global_costmap']
    assert manager['bond_timeout'] == 0.0
    manager = nav2_local.patch_params(tdm21_params, map_yaml=MAP, sources='lidar',
                                      planner=True)['lifecycle_manager_nav2_local']['ros__parameters']
    assert manager['node_names'] == ['map_server', 'planner_server']


# ---------------------------------------------------------------- TDM-22: Smac Hybrid-A* planner
def test_tdm22_hybrid_astar_planner(tdm21_params):
    planner = tdm21_params['planner_server']['ros__parameters']
    assert planner['planner_plugins'] == ['GridBased']   # = local_planner's planner_id
    grid = planner['GridBased']
    assert grid['plugin'] == 'nav2_smac_planner::SmacPlannerHybrid'
    assert grid['motion_model_for_search'] == 'DUBIN'    # forward only: the MPPI follower can't drive cusps
    assert grid['minimum_turning_radius'] >= 0.78        # never tighter than the real robot can turn
    assert grid['angle_quantization_bins'] == 72
    assert 0.0 < grid['max_planning_time'] <= 1.0        # local_planner replans at 2 Hz
    assert grid['cache_obstacle_heuristic'] is False     # rolling window + live scans
    assert grid['allow_unknown'] is False                # off-map cells are unknown
    assert grid['smooth_path'] is False                  # TDM-42: the smoother pulls paths towards obstacles
    assert grid['tolerance'] == pytest.approx(0.25)
    assert grid['lookup_table_size'] >= 12.0             # m; local_planner's default carrot distance


def test_tdm22_patching_leaves_the_planner_alone(tdm21_params):
    for sim in (False, True):
        patched = nav2_local.patch_params(tdm21_params, map_yaml=MAP, sources='auto', sim=sim)
        planner = patched['planner_server']['ros__parameters']
        assert planner['GridBased'] == tdm21_params['planner_server']['ros__parameters']['GridBased']
        assert planner['use_sim_time'] is sim


# ---------------------------------------------------------------- TDM-29: inflation, params file
def _tdm29_inflation_cost(params, distance):
    """Nav2 InflationLayer cost at `distance` m from a lethal cell, per nav2_local.yaml.

    253 inside the footprint's inscribed radius, 252 * exp(-cost_scaling_factor * (distance -
    inscribed)) out to inflation_radius, 0 beyond.
    """
    costmap = _costmap(params)
    inflation = costmap['inflation_layer']
    footprint = yaml.safe_load(costmap['footprint'])
    inscribed = min(min(abs(x) for x, _ in footprint), min(abs(y) for _, y in footprint))
    if distance <= inscribed:
        return 253
    if distance > inflation['inflation_radius']:
        return 0
    return 252 * math.exp(-inflation['cost_scaling_factor'] * (distance - inscribed))


def test_tdm29_inflation_centres_paths_in_a_3m_corridor(tdm21_params):
    costmap = _costmap(tdm21_params)
    footprint = yaml.safe_load(costmap['footprint'])
    circumscribed = max(math.hypot(x, y) for x, y in footprint)
    half_width = 1.5   # boundary map: free within 1.5 m of the walkway edge
    assert costmap['inflation_layer']['inflation_radius'] >= half_width + costmap['resolution']
    centre = _tdm29_inflation_cost(tdm21_params, half_width)
    # the centreline itself carries cost, so there is no zero-cost strip to wander in ...
    assert 2.0 <= centre <= 40.0
    # ... and every step off it costs more
    off = [_tdm29_inflation_cost(tdm21_params, half_width - d) for d in (0.2, 0.4, 0.8)]
    assert centre < off[0] < off[1] < off[2]
    # 0.8 m off the centreline stays below the analytic-expansion cap (and far below inscribed,
    # which only covers 0.32 m by each wall), so a robot that starts there can still plan
    grid = tdm21_params['planner_server']['ros__parameters']['GridBased']
    assert off[2] < grid['analytic_expansion_max_cost']
    # Smac's footprint check needs a non-zero cost at the circumscribed radius
    assert _tdm29_inflation_cost(tdm21_params, circumscribed) > 0


def test_tdm29_params_path_is_per_user_and_domain():
    path = nav2_local.patched_params_path
    assert path({'ROS_HOME': '/rh', 'ROS_DOMAIN_ID': '80'}) == '/rh/nav2_local_params_domain80.yaml'
    assert path({'ROS_HOME': '/rh'}) == '/rh/nav2_local_params_domain0.yaml'   # ROS's default domain
    assert path({'ROS_HOME': '/rh', 'ROS_DOMAIN_ID': ' 7 '}) == '/rh/nav2_local_params_domain7.yaml'
    assert path({'ROS_HOME': '/rh', 'ROS_DOMAIN_ID': '../x'}) == '/rh/nav2_local_params_domain0.yaml'
    assert path({}) == os.path.join(os.path.expanduser('~'), '.ros', 'nav2_local_params_domain0.yaml')


def test_tdm29_write_params_replaces_the_file(tmp_path):
    target = str(tmp_path / 'rh' / 'nav2_local_params_domain0.yaml')
    nav2_local.write_params({'a': 1}, target)
    nav2_local.write_params({'a': 2}, target)
    assert os.listdir(tmp_path / 'rh') == ['nav2_local_params_domain0.yaml']
    with open(target) as f:
        assert yaml.safe_load(f) == {'a': 2}


def test_tdm29_failed_write_keeps_the_old_file_and_no_temp(tmp_path, monkeypatch):
    target = str(tmp_path / 'nav2_local_params_domain0.yaml')
    nav2_local.write_params({'a': 1}, target)

    def disk_full(*args, **kwargs):
        raise OSError('disk full')
    monkeypatch.setattr(yaml, 'safe_dump', disk_full)
    with pytest.raises(OSError):
        nav2_local.write_params({'a': 2}, target)
    assert os.listdir(tmp_path) == ['nav2_local_params_domain0.yaml']
    with open(target) as f:
        assert yaml.safe_load(f) == {'a': 1}


def _tdm29_launch(map_yaml, sim='false'):
    """Run the launch file's OpaqueFunction once, as `ros2 launch` would; returns its actions."""
    from launch import LaunchContext

    if not all(nav2_local._have(p) for p in ('nav2_map_server', 'nav2_costmap_2d',
                                             'nav2_lifecycle_manager')):
        pytest.skip('Nav2 packages not installed: the launch starts nothing')
    context = LaunchContext()
    context.launch_configurations.update({
        'sim': sim, 'sources': 'auto', 'scan_topic': '', 'map': str(map_yaml),
        'params_file': os.path.join(PKG, 'config', 'nav2_local.yaml'),
        'local_planner': 'false', 'drive': 'false'})
    return nav2_local._launch_setup(context)


def test_tdm29_repeated_launches_leave_one_params_file(tmp_path, monkeypatch):
    from launch_ros.actions import Node

    ros_home = tmp_path / 'ros_home'
    monkeypatch.setenv('ROS_HOME', str(ros_home))
    monkeypatch.setenv('ROS_DOMAIN_ID', '42')
    map_yaml = tmp_path / 'boundary.yaml'
    map_yaml.write_text('image: boundary.png\n')
    for sim in ('false', 'true', 'false'):
        assert any(isinstance(a, Node) for a in _tdm29_launch(map_yaml, sim))
    # (launch may also create ROS_HOME/log); a leftover temp file would be .nav2_local_*.yaml
    assert [f for f in os.listdir(ros_home) if f.endswith('.yaml')] == ['nav2_local_params_domain42.yaml']
    with open(ros_home / 'nav2_local_params_domain42.yaml') as f:
        params = yaml.safe_load(f)
    assert params['map_server']['ros__parameters']['yaml_filename'] == str(map_yaml)
    assert params['map_server']['ros__parameters']['use_sim_time'] is False   # the last launch's args


def test_tdm29_unwritable_params_file_starts_nothing(tmp_path, monkeypatch):
    from launch.actions import LogInfo

    not_a_dir = tmp_path / 'ros_home'
    not_a_dir.write_text('')            # ROS_HOME is a file: the params file cannot be written
    monkeypatch.setenv('ROS_HOME', str(not_a_dir))
    map_yaml = tmp_path / 'boundary.yaml'
    map_yaml.write_text('image: boundary.png\n')
    actions = _tdm29_launch(map_yaml)
    assert len(actions) == 1 and isinstance(actions[0], LogInfo)


def _tdm46_lib(tmp_path, source, name):
    """Compile C `source` into tmp_path/lib/<name>; skips without a C compiler."""
    import shutil
    import subprocess

    cc = shutil.which('cc') or shutil.which('gcc')
    if cc is None:
        pytest.skip('no C compiler')
    lib_dir = tmp_path / 'lib'
    lib_dir.mkdir(exist_ok=True)
    src = tmp_path / (name + '.c')
    src.write_text(source)
    subprocess.run([cc, '-shared', '-fPIC', '-o', str(lib_dir / name), str(src)], check=True)
    return lib_dir / name


def test_tdm46_typesupport_with_an_unbound_symbol_is_found(tmp_path, monkeypatch):
    # The Pi's nav2_msgs typesupport: loads lazily, but calls a fastcdr function that is not there
    _tdm46_lib(tmp_path, 'void gone(void);\nvoid serialize(void) { gone(); }\n',
               'libfake_msgs__rosidl_typesupport_fastrtps_cpp.so')
    _tdm46_lib(tmp_path, 'int ok(void) { return 1; }\n',
               'libfake_msgs__rosidl_typesupport_introspection_cpp.so')
    _tdm46_lib(tmp_path, 'void gone(void);\nvoid f(void) { gone(); }\n',
               'libfake_msgs__rosidl_generator_py.so')   # not typesupport: not checked
    monkeypatch.setattr(nav2_local, 'get_package_prefix',
                        lambda pkg: str(tmp_path) if pkg == 'fake_msgs' else '/nonexistent')
    broken = nav2_local.broken_typesupport(('fake_msgs',))
    assert [os.path.basename(lib) for lib, _ in broken] == [
        'libfake_msgs__rosidl_typesupport_fastrtps_cpp.so']
    assert 'gone' in broken[0][1]


def test_tdm46_missing_interface_package_is_not_broken(monkeypatch):
    def not_found(pkg):
        raise nav2_local.PackageNotFoundError(pkg)

    monkeypatch.setattr(nav2_local, 'get_package_prefix', not_found)
    assert nav2_local.broken_typesupport() == []


def test_tdm46_broken_typesupport_starts_nothing(tmp_path, monkeypatch):
    from launch.actions import LogInfo

    monkeypatch.setenv('ROS_HOME', str(tmp_path / 'ros_home'))
    monkeypatch.setattr(nav2_local, 'broken_typesupport', lambda: [
        ('/opt/ros/jazzy/lib/libnav2_msgs__rosidl_typesupport_fastrtps_cpp.so',
         'undefined symbol: _ZN8eprosima7fastcdr3Cdr9serializeEj')])
    map_yaml = tmp_path / 'boundary.yaml'
    map_yaml.write_text('image: boundary.png\n')
    actions = _tdm29_launch(map_yaml)
    assert len(actions) == 1 and isinstance(actions[0], LogInfo)
