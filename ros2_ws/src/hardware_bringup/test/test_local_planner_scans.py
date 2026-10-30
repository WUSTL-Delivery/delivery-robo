"""
TDM-52: local_planner watches the costmap's scan topics.

It logs the silent ones (with every planning failure too) and, while a goal is active and every
topic is silent, sends one stop and does not replan until one returns. Needs a sourced ROS
environment with nav2_msgs and shared_types; skipped otherwise.
"""
from types import SimpleNamespace

from hardware_bringup.route import Route
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('nav2_msgs')
pytest.importorskip('shared_types')

import hardware_bringup.local_planner as lp  # noqa: E402, I100

TOPICS = ['/scan_fused', '/scan', '/camera/scan']


class CaptureLogger:
    """Collect log lines by level."""

    def __init__(self):
        self.lines = []

    def __getattr__(self, level):
        """Return a recorder for info / warning / error."""
        return lambda text, **_: self.lines.append((level, text))


@pytest.fixture
def planner(monkeypatch):
    rclpy.init(args=['--ros-args', '-p', 'scan_topics:=[' + ','.join(TOPICS) + ']',
                     '-p', 'scan_timeout:=1.0'])
    node = lp.LocalPlanner()
    now = [1000.0]
    monkeypatch.setattr(lp, 'time', SimpleNamespace(monotonic=lambda: now[0]))
    logger = CaptureLogger()
    monkeypatch.setattr(node, 'get_logger', lambda: logger)
    stops = []
    monkeypatch.setattr(node, '_send_stop', stops.append)
    node._scans_since = now[0] - 5.0
    yield node, now, logger, stops
    node.destroy_node()
    rclpy.shutdown()


def test_silent_topics_are_logged_once_per_change(planner):
    node, now, logger, _ = planner
    for topic in TOPICS[:2]:
        node._scans.seen(topic, now[0])
    node._tick()
    node._tick()
    assert logger.lines == [('warning', 'scan topics silent: /camera/scan never received; '
                                        'live: /scan_fused, /scan')]
    assert node._scan_report() == '; silent scan topics: /camera/scan never received'


def test_planning_failure_names_the_silent_topics(planner):
    node, now, logger, _ = planner
    node._scans.seen('/scan', now[0] - 3.0)
    node._on_plan_failure('aborted, TIMEOUT, 0 poses')
    (level, text), = [line for line in logger.lines if line[1].startswith('planning failed')]
    assert level == 'warning'
    assert text.endswith('; silent scan topics: /scan_fused never received, /scan silent 3.0 s, '
                         '/camera/scan never received')


def test_all_silent_stops_once_and_replans_when_one_returns(planner, monkeypatch):
    node, now, logger, stops = planner
    node._route = Route([(0.0, 0.0), (20.0, 0.0)], 0.5)
    poses = []
    monkeypatch.setattr(node, '_robot_pose', lambda: poses.append(1))   # None: tick ends there
    node._tick()
    node._tick()
    assert stops == ['no scans']
    errors = [text for level, text in logger.lines if level == 'error']
    assert errors[0].startswith('scan topics ALL silent')
    assert errors[1].startswith('no scan topic publishes (/scan_fused never received')
    assert not poses                     # no planning while every topic is silent
    node._scans.seen('/scan', now[0])
    node._tick()
    assert stops == ['no scans'] and len(poses) == 1
    assert ('info', 'a scan topic is back; replanning') in logger.lines


def test_no_scan_topics_watches_nothing(monkeypatch):
    rclpy.init()
    try:
        node = lp.LocalPlanner()
        assert node._scan_topics == []
        assert node._check_scans() is False
        assert node._scan_report() == ''
        node.destroy_node()
    finally:
        rclpy.shutdown()
