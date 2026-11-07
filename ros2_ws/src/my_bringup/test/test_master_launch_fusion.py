"""
TDM-52: master_launch starts scan_fusion when Nav2's costmap reads /scan_fused.

Needs a sourced ROS environment with my_bringup built (skipped otherwise).
"""
import importlib.util
import os

import pytest

pytest.importorskip('launch_ros')

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _master():
    from ament_index_python.packages import get_package_prefix, PackageNotFoundError

    try:
        prefix = get_package_prefix('my_bringup')
    except PackageNotFoundError:
        pytest.skip('my_bringup not built/sourced')
    if not os.path.exists(os.path.join(prefix, 'lib', 'my_bringup', 'scan_fusion')):
        pytest.skip('scan_fusion executable not installed')
    path = os.path.join(PKG, 'launch', 'master_launch.py')
    spec = importlib.util.spec_from_file_location('tdm52_master_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _fusion(mode, sim, nav2, sources):
    """Return (started, launch arguments) of the scan_fusion include for these arguments."""
    from launch import LaunchContext
    from launch.actions import IncludeLaunchDescription
    from launch.substitutions import LaunchConfiguration
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

    def text(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    master = _master()
    actions = master._scan_fusion_actions(*(LaunchConfiguration(n) for n in (
        'mode', 'sim', 'nav2', 'nav2_sources')))
    (include,) = actions
    assert isinstance(include, IncludeLaunchDescription)
    context = LaunchContext()
    context.launch_configurations.update(
        {'mode': mode, 'sim': sim, 'nav2': nav2, 'nav2_sources': sources})
    include.launch_description_source.get_launch_description(context)
    assert include.launch_description_source.location.endswith('/scan_fusion.launch.py')
    args = {text(k): text(v) for k, v in include.launch_arguments}
    return include.condition.evaluate(context), args


@pytest.mark.parametrize('mode, sim, nav2, sources, started', [
    ('autonomy', 'false', 'true', 'auto', True),     # the robot's default: fused
    ('autonomy', 'false', 'true', 'fused', True),
    ('autonomy', 'false', 'true', 'separate', False),
    ('autonomy', 'false', 'true', 'lidar', False),
    ('autonomy', 'false', 'false', 'auto', False),   # no Nav2, no costmap
    ('autonomy', 'true', 'true', 'auto', False),     # auto = lidar in the sim
    ('autonomy', 'true', 'true', 'fused', True),
    ('teleop', 'false', 'true', 'fused', False),
    ('autonomous', 'false', 'true', 'fused', False),
])
def test_tdm52_scan_fusion_follows_the_costmap_source(mode, sim, nav2, sources, started):
    assert _fusion(mode, sim, nav2, sources)[0] is started


@pytest.mark.parametrize('sim, lidar', [('false', '/scan_filtered'), ('true', '/scan')])
def test_tdm52_scan_fusion_inputs_and_odom(sim, lidar):
    _, args = _fusion('autonomy', sim, 'true', 'fused')
    assert args == {'lidar_topic': lidar, 'fixed_frame': 'odom', 'use_sim_time': sim}
