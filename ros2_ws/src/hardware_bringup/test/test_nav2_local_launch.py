"""TDM-21: config/nav2_local.yaml and the parameter patching in launch/nav2_local.launch.py.

Needs launch_ros importable (a sourced ROS environment); skipped otherwise.
"""
import importlib.util
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


@pytest.mark.parametrize('sources, expected', [
    ('fused', {'fused': (True, False), 'lidar': (False, True), 'camera': (False, True)}),
    ('separate', {'lidar': (True, True), 'camera': (True, True)}),
    ('lidar', {'lidar': (True, True)}),
    ('auto', {'fused': (True, False), 'lidar': (False, True), 'camera': (False, True)}),
])
def test_sources_wiring(tdm21_params, sources, expected):
    patched = nav2_local.patch_params(tdm21_params, map_yaml=MAP, sources=sources)
    obstacle = _costmap(patched)['obstacle_layer']
    assert obstacle['observation_sources'].split() == list(expected)
    for name, (marking, clearing) in expected.items():
        assert (obstacle[name]['marking'], obstacle[name]['clearing']) == (marking, clearing)
    assert obstacle['lidar']['topic'] == '/scan_filtered'
    assert patched['map_server']['ros__parameters']['yaml_filename'] == MAP


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
    assert 0.0 < grid['max_planning_time'] <= 1.0        # local_planner replans at 1 Hz
    assert grid['cache_obstacle_heuristic'] is False     # rolling window + live scans
    assert grid['allow_unknown'] is False                # off-map cells are unknown
    assert grid['smooth_path'] is True
    assert grid['tolerance'] == pytest.approx(0.25)
    assert grid['lookup_table_size'] >= 12.0             # m; local_planner's default carrot distance


def test_tdm22_patching_leaves_the_planner_alone(tdm21_params):
    for sim in (False, True):
        patched = nav2_local.patch_params(tdm21_params, map_yaml=MAP, sources='auto', sim=sim)
        planner = patched['planner_server']['ros__parameters']
        assert planner['GridBased'] == tdm21_params['planner_server']['ros__parameters']['GridBased']
        assert planner['use_sim_time'] is sim
