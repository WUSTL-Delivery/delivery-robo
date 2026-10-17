"""Exercise obstacle geometry, invalid returns, pairing and model alignment."""

import math
import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from my_bringup.scan_fusion_core import (
    ScanGrid,
    ScanSample,
    closest_sample,
    fuse_hits,
    is_fresh,
    project_hits,
)


def scan(ranges, angle_min=0.0, angle_increment=math.pi / 2, stamp_ns=1_000_000_000):
    return ScanSample(stamp_ns, 'sensor', angle_min, angle_increment,
                      0.1, 12.0, np.asarray(ranges, dtype=float))


def transform(x=0.0, y=0.0, z=0.0, yaw=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    matrix = np.eye(4)
    matrix[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    matrix[:3, 3] = [x, y, z]
    return matrix


def test_nearest_obstacle_wins_without_averaging():
    result = fuse_hits(scan([5.0, 1.0]), np.eye(4), scan([2.0, 3.0]), np.eye(4), ScanGrid())
    assert result[180] == 2.0
    assert result[270] == 1.0
    assert np.count_nonzero(np.isfinite(result)) == 2


def test_sensor_translation_changes_range_and_bin_before_fusion():
    # A forward-facing sensor offset 1 m to the left sees an obstacle 1 m ahead.
    result = project_hits(scan([1.0]), transform(y=1.0), ScanGrid())
    assert result[225] == pytest.approx(math.sqrt(2))
    assert np.isnan(result[180])


def test_rotation_and_translation():
    result = project_hits(scan([2.0]), transform(x=1.0, yaw=math.pi / 2), ScanGrid())
    angle = math.atan2(2.0, 1.0)
    index = round((angle + math.pi) / math.radians(1))
    assert result[index] == pytest.approx(math.sqrt(5))


def test_current_model_aligns_a_shared_wall():
    # Read actual model properties; never apply this mount again to /camera/scan.
    model = Path(__file__).resolve().parents[2] / 'robot_description/urdf/robot.urdf.xacro'
    properties = {p.attrib['name']: float(p.attrib['value'])
                  for p in ET.parse(model).getroot().findall('{http://ros.org/wiki/xacro}property')}
    lidar_mount = transform(properties['lidar_x'], properties['lidar_y'],
                            properties['lidar_z'], properties['lidar_yaw'])
    camera_mount = transform(properties['camera_x'], properties['camera_y'],
                             properties['camera_z'], properties['camera_yaw'])
    wall_x = 2.0
    camera_projected = project_hits(scan([wall_x - properties['camera_x']]),
                                    camera_mount, ScanGrid())
    # /camera/scan already uses base_link, so the fusion transform is identity.
    camera = replace(scan(camera_projected, -math.pi, math.radians(1)), frame_id='base_link')
    result = fuse_hits(scan([wall_x - properties['lidar_x']]), lidar_mount,
                       camera, np.eye(4), ScanGrid())
    assert result[180] == pytest.approx(wall_x)


def test_input_range_limits_and_invalid_returns():
    sample = scan([np.nan, np.inf, -np.inf, 0.0, -1.0, 0.09, 12.1, 1.0],
                  angle_increment=math.radians(10))
    result = project_hits(sample, np.eye(4), ScanGrid())
    assert np.count_nonzero(np.isfinite(result)) == 1
    assert result[250] == 1.0


def test_free_camera_bins_do_not_clear_lidar_obstacles():
    result = fuse_hits(scan([2.0]), np.eye(4), scan([np.inf]), np.eye(4), ScanGrid())
    assert result[180] == 2.0
    assert not np.isposinf(result).any()


def test_empty_and_unknown_inputs_remain_unknown():
    result = fuse_hits(scan([]), np.eye(4), scan([np.inf, np.nan]), np.eye(4), ScanGrid())
    assert np.isnan(result).all()


def test_lidar_only_preserves_hits_without_camera():
    result = fuse_hits(scan([1.0]), np.eye(4), None, None, ScanGrid())
    assert result[180] == 1.0


@pytest.mark.parametrize('angle', [-math.pi, math.pi, math.pi - 1e-6, -math.pi + 1e-6])
def test_circular_seam_has_one_bin(angle):
    result = project_hits(scan([2.0], angle_min=angle), np.eye(4), ScanGrid())
    assert result[0] == 2.0
    assert np.count_nonzero(np.isfinite(result)) == 1


def test_multiple_endpoints_in_one_bin_keep_nearest():
    result = project_hits(scan([4.0, 3.0, 2.0], angle_increment=0.0001), np.eye(4), ScanGrid())
    assert result[180] == 2.0


def test_output_range_limits_apply_after_transform():
    result = project_hits(scan([1.0]), transform(x=12.0), ScanGrid())
    assert np.isnan(result).all()
    result = project_hits(scan([1.0]), transform(x=-1.0), ScanGrid())
    assert np.isnan(result).all()  # endpoint at virtual origin has no direction


def test_negative_input_angle_increment():
    result = project_hits(scan([1.0, 2.0], angle_increment=-math.pi / 2), np.eye(4), ScanGrid())
    assert result[180] == 1.0
    assert result[90] == 2.0


def test_grid_metadata_matches_range_count():
    grid = ScanGrid(bins=720)
    assert grid.angle_max == pytest.approx(grid.angle_min + 719 * grid.angle_increment)
    assert grid.angle_increment == pytest.approx(math.radians(0.5))
    assert grid.angle_max < math.pi


@pytest.mark.parametrize('kwargs', [{'bins': 0}, {'bins': 4.5}, {'range_min': -1},
                                    {'range_max': np.inf}, {'range_min': 2, 'range_max': 1}])
def test_invalid_grid_is_rejected(kwargs):
    with pytest.raises(ValueError):
        ScanGrid(**kwargs)


@pytest.mark.parametrize('kwargs', [{'frame_id': ''}, {'angle_increment': 0},
                                    {'angle_min': np.nan}, {'range_min': -1},
                                    {'ranges': np.zeros((2, 2))}])
def test_invalid_scan_metadata_is_rejected(kwargs):
    with pytest.raises(ValueError):
        replace(scan([1.0]), **kwargs)


def test_invalid_transform_is_rejected():
    with pytest.raises(ValueError):
        project_hits(scan([1.0]), np.full((4, 4), np.nan), ScanGrid())


def test_nearest_timestamp_works_with_out_of_order_arrival():
    samples = [scan([], stamp_ns=1_040_000_000), scan([], stamp_ns=980_000_000),
               scan([], stamp_ns=1_010_000_000)]
    assert closest_sample(samples, 1_000_000_000, 1_050_000_000,
                          300_000_000, 50_000_000) is samples[2]


def test_timestamp_tie_uses_earlier_scan():
    samples = [scan([], stamp_ns=1_020_000_000), scan([], stamp_ns=980_000_000)]
    assert closest_sample(samples, 1_000_000_000, 1_050_000_000,
                          300_000_000, 50_000_000) is samples[1]


def test_stale_mismatched_or_far_future_camera_is_not_used():
    for stamp in (0, 900_000_000, 1_200_000_000):
        assert closest_sample([scan([], stamp_ns=stamp)], 1_000_000_000,
                              1_050_000_000, 300_000_000, 50_000_000) is None
    assert closest_sample([scan([], stamp_ns=1_000_000_000)], 1_000_000_000,
                          1_400_000_000, 300_000_000, 50_000_000) is None


@pytest.mark.parametrize('stamp, expected', [
    (0, False), (699_999_999, False), (700_000_000, True),
    (1_000_000_000, True), (1_050_000_000, True), (1_050_000_001, False),
])
def test_freshness_boundaries(stamp, expected):
    assert is_fresh(stamp, 1_000_000_000, 300_000_000, 50_000_000) is expected


def test_motion_compensation_changes_camera_endpoints_before_fusion():
    # Robot has moved +0.1 m in odom since the camera observed a wall at 2 m.
    # lookup_transform_full expresses that old endpoint at 1.9 m in current base_link.
    result = fuse_hits(scan([1.9]), np.eye(4), scan([2.0]), transform(x=-0.1), ScanGrid())
    assert result[180] == pytest.approx(1.9)
