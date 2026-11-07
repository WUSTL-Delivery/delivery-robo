"""
TDM-52 (sensors bundle): sensors:=false forces lidar, realsense and obstacle_guard off together,
independent of their own flags; nav2's own default/behavior is untouched.

Pure functions (no package lookups), so no sourced/built workspace is needed to run these.
"""
import importlib.util
import os

import pytest

pytest.importorskip('launch_ros')

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _master():
    path = os.path.join(PKG, 'launch', 'master_launch.py')
    spec = importlib.util.spec_from_file_location('tdm52_sensors_master_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _use_sensor(sensors, flag, mode, sim):
    from launch import LaunchContext

    master = _master()
    condition = master._use_sensor_condition(sensors, flag, mode, sim)
    return condition.evaluate(LaunchContext())


def _effective_obstacle_guard(obstacle_guard, sensors):
    from launch import LaunchContext
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

    master = _master()
    value = master._effective_obstacle_guard(obstacle_guard, sensors)
    return perform_substitutions(LaunchContext(), normalize_to_list_of_substitutions(value))


@pytest.mark.parametrize('sensors, flag, started', [
    ('true', 'true', True),    # today's default: unaffected by the new flag
    ('true', 'false', False),  # the driver's own flag still works alone
    ('false', 'true', False),  # sensors:=false overrides an explicit lidar/realsense:=true
    ('false', 'false', False),
])
def test_sensors_false_overrides_the_drivers_own_flag(sensors, flag, started):
    assert _use_sensor(sensors, flag, 'autonomy', 'false') is started


def test_sensors_false_does_not_touch_the_sim_guard():
    # sim:=true already forces lidar/realsense off (Gazebo provides the topic); unaffected
    assert _use_sensor('true', 'true', 'autonomy', 'true') is False
    assert _use_sensor('false', 'true', 'autonomy', 'true') is False


@pytest.mark.parametrize('obstacle_guard, sensors, locks', [
    ('true', 'true', 'true'),     # today's default: the guard runs and can lock
    ('false', 'true', 'false'),   # its own flag still disables it alone
    ('true', 'false', 'false'),   # sensors:=false overrides an explicit obstacle_guard:=true
    ('false', 'false', 'false'),
])
def test_sensors_false_overrides_obstacle_guard(obstacle_guard, sensors, locks):
    assert _effective_obstacle_guard(obstacle_guard, sensors) == locks
