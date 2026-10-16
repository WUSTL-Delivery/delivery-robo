"""TDM-24: local_planner started from launch/nav2_local.launch.py.

Needs launch_ros importable (a sourced ROS environment); skipped otherwise.
"""
import importlib.util
import os

import pytest

pytest.importorskip('launch_ros')
yaml = pytest.importorskip('yaml')

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _load_tdm24_launch_module():
    path = os.path.join(PKG, 'launch', 'nav2_local.launch.py')
    spec = importlib.util.spec_from_file_location('tdm24_nav2_local_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


tdm24_launch = _load_tdm24_launch_module()


@pytest.fixture
def tdm24_params():
    with open(os.path.join(PKG, 'config', 'nav2_local.yaml')) as f:
        return yaml.safe_load(f)


def test_tdm24_planner_id_matches_planner_plugins(tdm24_params):
    plugins = tdm24_params['planner_server']['ros__parameters']['planner_plugins']
    assert tdm24_launch.local_planner_params(tdm24_params)['planner_id'] == plugins[0]


@pytest.mark.parametrize('sim, drive', [(False, False), (True, True), (True, False)])
def test_tdm24_sim_and_drive_pass_through(tdm24_params, sim, drive):
    lp = tdm24_launch.local_planner_params(tdm24_params, sim=sim, drive=drive)
    assert lp['use_sim_time'] is sim
    assert lp['drive'] is drive


def test_tdm24_planner_id_default_without_planner_section():
    assert tdm24_launch.local_planner_params({})['planner_id'] == 'GridBased'


def test_tdm24_launch_arguments():
    if not tdm24_launch._have('hardware_bringup'):
        pytest.skip('hardware_bringup not built/sourced (the params_file default needs its share dir)')
    ld = tdm24_launch.generate_launch_description()
    args = {a.name: a for a in ld.get_launch_arguments()}
    assert args['local_planner'].default_value[0].text == 'true'
    assert args['drive'].default_value[0].text == 'false'    # never drives unless asked
    assert {'map', 'sources', 'scan_topic', 'sim', 'params_file'} <= set(args)
