"""ROS wrapper tests using real messages and TF (skipped when ROS is absent)."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')

import my_bringup.scan_fusion as fusion  # noqa: E402
from diagnostic_msgs.msg import DiagnosticStatus  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from rclpy.time import Time  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402


class CapturePublisher:
    """Collect published messages without requiring a subscriber."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        """Keep each published message."""
        self.messages.append(message)


@pytest.fixture
def node(monkeypatch):
    rclpy.init()
    instance = fusion.ScanFusion()
    now_ns = [1_050_000_000]
    wall = [100.0]
    monkeypatch.setattr(instance, 'get_clock', lambda: SimpleNamespace(
        now=lambda: Time(nanoseconds=now_ns[0])))
    monkeypatch.setattr(fusion, 'time', SimpleNamespace(monotonic=lambda: wall[0]))
    instance.publisher = CapturePublisher()
    instance.diagnostics = CapturePublisher()
    yield instance, now_ns, wall
    instance.destroy_node()
    rclpy.shutdown()


def message(distance, stamp_ns=1_000_000_000, frame='base_link'):
    result = LaserScan()
    result.header.frame_id = frame
    result.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
    result.angle_min = 0.0
    result.angle_max = 0.0
    result.angle_increment = math.radians(1)
    result.range_min = 0.1
    result.range_max = 12.0
    result.scan_time = 0.1
    result.ranges = [float(distance)]
    return result


def tf(parent, child, stamp_ns=0, x=0.0, z=0.0):
    result = TransformStamped()
    result.header.frame_id = parent
    result.child_frame_id = child
    result.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
    result.transform.translation.x = x
    result.transform.translation.z = z
    result.transform.rotation.w = 1.0
    return result


def ready(instance, wall):
    """Advance the pairing deadline without sleeping."""
    wall[0] += 0.051
    instance._process()


def test_fused_output_uses_reference_stamp_and_consistent_metadata(node):
    instance, _, wall = node
    instance.tf_buffer.set_transform_static(tf('base_link', 'lidar_link', z=0.357), 'test')
    instance._on_lidar(message(5.0, frame='lidar_link'))
    instance._on_camera(message(2.0, stamp_ns=1_020_000_000))
    instance._process()
    assert not instance.publisher.messages
    ready(instance, wall)
    output = instance.publisher.messages[-1]
    assert Time.from_msg(output.header.stamp).nanoseconds == 1_000_000_000
    assert output.header.frame_id == 'base_link'
    assert output.ranges[180] == pytest.approx(2.0)
    assert len(output.ranges) == 360
    assert output.angle_max == pytest.approx(output.angle_min + 359 * output.angle_increment)
    assert output.time_increment == 0.0
    assert not np.isposinf(output.ranges).any()
    assert instance.camera_used


def test_camera_dropout_publishes_lidar_only_and_reports_warning(node):
    instance, _, wall = node
    instance._on_lidar(message(3.0))
    ready(instance, wall)
    assert instance.publisher.messages[-1].ranges[180] == pytest.approx(3.0)
    assert not instance.camera_used
    instance._diagnose()
    assert instance.diagnostics.messages[-1].status[0].level == DiagnosticStatus.WARN


def test_stale_camera_obstacle_is_not_reused(node):
    instance, _, wall = node
    instance._on_camera(message(1.0, stamp_ns=700_000_000))
    instance._on_lidar(message(3.0))
    ready(instance, wall)
    assert instance.publisher.messages[-1].ranges[180] == pytest.approx(3.0)
    assert not instance.camera_used


def test_stale_lidar_does_not_publish(node):
    instance, _, wall = node
    instance._on_lidar(message(1.0, stamp_ns=700_000_000))
    ready(instance, wall)
    assert not instance.publisher.messages
    instance._diagnose()
    assert instance.diagnostics.messages[-1].status[0].level == DiagnosticStatus.ERROR


def test_missing_lidar_tf_is_retried_then_dropped(node):
    instance, _, wall = node
    instance._on_lidar(message(3.0, frame='missing_lidar'))
    ready(instance, wall)
    assert len(instance.lidar_queue) == 1
    wall[0] += 0.051
    instance._process()
    assert not instance.lidar_queue
    assert not instance.publisher.messages
    assert 'lidar TF unavailable' in instance.error


def test_missing_camera_tf_degrades_after_bounded_wait(node):
    instance, _, wall = node
    instance._on_lidar(message(3.0))
    instance._on_camera(message(1.0, frame='missing_camera'))
    ready(instance, wall)
    assert not instance.publisher.messages
    wall[0] += 0.051
    instance._process()
    assert instance.publisher.messages[-1].ranges[180] == pytest.approx(3.0)
    assert 'camera TF unavailable' in instance.error
    assert not instance.camera_used


def test_output_stops_when_lidar_stops(node):
    instance, now_ns, wall = node
    instance._on_lidar(message(3.0))
    ready(instance, wall)
    now_ns[0] = 1_500_000_000
    wall[0] += 0.5
    instance._on_camera(message(1.0, stamp_ns=now_ns[0]))
    instance._process()
    instance._diagnose()
    assert len(instance.publisher.messages) == 1
    assert instance.diagnostics.messages[-1].status[0].level == DiagnosticStatus.ERROR


def test_duplicate_lidar_does_not_republish(node):
    instance, _, wall = node
    instance._on_lidar(message(3.0))
    ready(instance, wall)
    instance._on_lidar(message(2.0))
    ready(instance, wall)
    assert len(instance.publisher.messages) == 1


def test_backward_clock_jump_clears_observations(node):
    instance, now_ns, wall = node
    instance._on_lidar(message(3.0))
    instance._on_camera(message(1.0))
    ready(instance, wall)
    instance._on_lidar(message(2.0, stamp_ns=1_040_000_000))
    now_ns[0] = 100_000_000
    instance._process()
    assert not instance.lidar_queue and not instance.camera_queue
    assert instance.last_output_ns is None
    assert not instance.camera_used


def test_motion_compensation_even_when_camera_frame_is_output_frame(node):
    instance, _, wall = node
    instance.fixed_frame = 'odom'
    instance.tf_buffer.set_transform(
        tf('odom', 'base_link', stamp_ns=980_000_000, x=0.0), 'test')
    instance.tf_buffer.set_transform(
        tf('odom', 'base_link', stamp_ns=1_000_000_000, x=0.1), 'test')
    instance._on_lidar(message(3.0))
    instance._on_camera(message(2.0, stamp_ns=980_000_000))
    ready(instance, wall)
    assert instance.publisher.messages[-1].ranges[180] == pytest.approx(1.9)
    assert instance.camera_used
    instance._diagnose()
    assert instance.diagnostics.messages[-1].status[0].level == DiagnosticStatus.OK
