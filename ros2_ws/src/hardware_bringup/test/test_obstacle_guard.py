"""
TDM-52: obstacle_guard's state logic (hardware_bringup.sensor_watch) and its twist_mux lock.

The user's rule: one of lidar / depth camera missing -> keep driving on the other; both missing
-> no autonomous driving, the joystick still overrides.
"""
import os

from hardware_bringup.sensor_watch import ObstacleGuardLogic, SensorWatch
import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIDAR, CAMERA = 'lidar /scan_filtered', 'camera /camera/scan'


def guard(enabled=True, timeout=1.0):
    return ObstacleGuardLogic(SensorWatch([LIDAR, CAMERA], timeout), enabled)


def test_sensor_watch_live_and_silent():
    watch = SensorWatch(['/a', '/b'], 1.0)
    assert watch.live(0.0) == () and watch.silent(0.0) == ('/a', '/b')
    assert not watch.ever_seen()
    watch.seen('/b', 10.0)
    assert watch.live(11.0) == ('/b',)            # exactly the timeout old: still live
    assert watch.live(11.01) == ()
    assert watch.silent_summary(12.5) == '/a never received, /b silent 2.5 s'
    watch.seen('/a', 12.5)
    assert watch.silent_summary(12.5) == '/b silent 2.5 s'
    assert watch.ever_seen()


@pytest.mark.parametrize('timeout', [0.0, -1.0, float('nan'), float('inf')])
def test_sensor_watch_rejects_a_bad_timeout(timeout):
    with pytest.raises(ValueError):
        SensorWatch(['/a'], timeout)


def test_locked_at_startup_until_a_first_scan():
    logic = guard()
    locked, log = logic.update(0.0)
    assert locked
    assert log[0] == 'warn' and 'no scan yet' in log[1]
    assert logic.update(5.0) == (True, None)        # logged once, still locked
    logic.watch.seen(CAMERA, 5.1)
    locked, log = logic.update(5.1)
    assert not locked
    assert log[0] == 'warn'
    assert 'lidar /scan_filtered never received' in log[1]
    assert 'driving on camera /camera/scan only' in log[1] and 'lock released' in log[1]


def test_one_sensor_missing_keeps_driving():
    logic = guard()
    logic.watch.seen(LIDAR, 0.0)
    logic.watch.seen(CAMERA, 0.0)
    assert logic.update(0.1) == (False,
                                 ('info', f'obstacle sensors live: {LIDAR} and {CAMERA}'))
    for t in (0.5, 1.0, 1.5, 2.0):          # the camera keeps publishing, the lidar stopped
        logic.watch.seen(CAMERA, t)
    locked, log = logic.update(2.0)
    assert not locked
    assert log == ('warn', f'obstacle sensor missing ({LIDAR} silent 2.0 s): '
                           f'driving on {CAMERA} only')
    assert logic.update(2.05) == (False, None)


def test_both_missing_locks_and_one_returning_releases():
    logic = guard()
    logic.watch.seen(LIDAR, 0.0)
    logic.watch.seen(CAMERA, 0.0)
    logic.update(0.0)
    assert logic.update(1.0) == (False, None)       # both exactly 1.0 s old: still live
    locked, log = logic.update(1.05)
    assert locked
    assert log[0] == 'error'
    assert f'{LIDAR} silent 1.1 s' in log[1] and f'{CAMERA} silent 1.1 s' in log[1]
    assert 'BLOCKED' in log[1] and 'joystick still drives' in log[1]
    assert logic.update(3.0) == (True, None)        # once per change, not per tick
    logic.watch.seen(LIDAR, 3.0)
    locked, log = logic.update(3.0)
    assert not locked
    assert log[0] == 'warn' and 'lock released' in log[1] and 'driving on lidar' in log[1]
    logic.watch.seen(CAMERA, 3.1)
    locked, log = logic.update(3.1)
    assert (locked, log[0]) == (False, 'info') and 'lock released' not in log[1]


def test_disabled_never_locks_and_stays_quiet():
    logic = guard(enabled=False)
    assert logic.update(0.0) == (False, None)
    assert logic.update(100.0) == (False, None)


# ------------------------------------------------------------------ twist_mux lock config
def test_twist_mux_lock_sits_between_navigation_and_joystick():
    yaml = pytest.importorskip('yaml')
    with open(os.path.join(PKG, 'config', 'twist_mux.yaml')) as f:
        mux = yaml.safe_load(f)['twist_mux']['ros__parameters']
    lock = mux['locks']['obstacle_sensors']
    topics = mux['topics']
    # twist_mux masks an input whose priority is below the highest engaged lock's
    assert topics['navigation']['priority'] < lock['priority'] <= topics['joystick']['priority']
    # a guard that stops publishing (10 Hz) must count as locked: twist_mux's lock timeout
    assert 0.0 < lock['timeout'] <= 1.0
    # the guard's default lock topic (twist_mux resolves the relative name in its namespace, /)
    assert '/' + lock['topic'] == '/obstacle_sensors_lock'
    with open(os.path.join(PKG, 'hardware_bringup', 'obstacle_guard.py')) as f:
        assert "'lock_topic', '/obstacle_sensors_lock'" in f.read()
