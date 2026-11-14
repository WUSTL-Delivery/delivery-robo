"""Exercise conversion and ROS delivery using a fake sensor, never Pi hardware."""

import csv
import time
from types import SimpleNamespace
from unittest.mock import Mock

from builtin_interfaces.msg import Time
from imu_package import imu_node
import pytest
import rclpy
from sensor_msgs.msg import MagneticField


def test_units_frame_and_unknown_covariance():
    msg = imu_node.magnetic_message((20.0, -40.0, 50.0), Time(sec=123), 'imu_link')
    assert msg.magnetic_field.x == pytest.approx(20e-6)
    assert msg.magnetic_field.y == pytest.approx(-40e-6)
    assert msg.magnetic_field.z == pytest.approx(50e-6)
    assert msg.header.frame_id == 'imu_link'
    assert msg.header.stamp.sec == 123
    assert list(msg.magnetic_field_covariance) == [0.0] * 9


@pytest.mark.parametrize('values', [
    None, (1, 2), (float('nan'), 0, 0), (0, float('inf'), 0)])
def test_invalid_samples_are_rejected(values):
    with pytest.raises(ValueError):
        imu_node.magnetic_message(values, Time(), 'imu_link')


def test_real_ros_publication_with_fake_sensor(monkeypatch, tmp_path):
    sensor = Mock()
    sensor.magnetic = (20.0, -40.0, 50.0)
    monkeypatch.setattr(imu_node, 'open_sensor', lambda: sensor)
    output = tmp_path / 'mag.csv'
    rclpy.init(args=['--ros-args', '-p', f'mag_csv_path:={output}',
                     '-p', 'mag_record_limit:=2'])
    node = imu_node.ImuNode()
    listener = rclpy.create_node('mag_test_listener')
    node.timer.cancel()
    node.mag_timer.cancel()
    messages = []
    listener.create_subscription(MagneticField, '/imu/mag', messages.append, 10)
    try:
        sensor.enable_feature.assert_any_call(imu_node.BNO_REPORT_MAGNETOMETER, 100000)
        deadline = time.monotonic() + 5.0
        while not messages and time.monotonic() < deadline:
            node.magnetic_callback()
            rclpy.spin_once(listener, timeout_sec=0.1)
        assert messages, 'No MagneticField delivered over ROS'
        assert messages[-1].magnetic_field.y == pytest.approx(-40e-6)
        assert messages[-1].header.frame_id == 'imu_link'
        assert messages[-1].header.stamp.sec > 0
        node.mag_publisher = Mock()
        sensor.magnetic = None
        node.magnetic_callback()
        node.mag_publisher.publish.assert_not_called()
        sensor.magnetic = (1.0, 2.0, 3.0)
        node.magnetic_callback()
        node.mag_publisher.publish.assert_called_once()
        # Complete the cap even if discovery needed only one iteration.
        node.magnetic_callback()
        assert node.mag_file is None
        with output.open() as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == 2
        assert float(rows[0]['mag_y_t']) == pytest.approx(-40e-6)
        assert rows[0]['frame_id'] == 'imu_link'
        assert int(rows[0]['stamp_sec']) > 0
    finally:
        listener.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def test_disk_failure_disables_recording():
    stream = Mock()
    writer = Mock()
    writer.writerow.side_effect = OSError('Disk full')
    logger = Mock()
    node = SimpleNamespace(mag_file=stream, mag_writer=writer, mag_rows=0,
                           mag_record_limit=10, get_logger=lambda: logger)
    node.close_recording = lambda: imu_node.ImuNode.close_recording(node)
    msg = imu_node.magnetic_message((1, 2, 3), Time(), 'imu_link')
    imu_node.ImuNode.record_magnetic(node, msg)
    assert node.mag_file is None
    stream.close.assert_called_once()
    logger.error.assert_called_once()


def test_recording_disabled_is_noop():
    node = SimpleNamespace(mag_file=None)
    imu_node.ImuNode.record_magnetic(node, None)
