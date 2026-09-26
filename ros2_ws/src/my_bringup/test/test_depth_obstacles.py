"""
Unit tests for my_bringup.depth_obstacles' numpy core (no ROS graph needed).

A synthetic depth image is ray-cast from a camera 0.30 m above a flat floor,
pitched 20 deg down, optionally with a box in front of it, then pushed through
the same functions the node uses.
"""
import math
from types import SimpleNamespace

from my_bringup.depth_obstacles import (
    deproject, depth_to_meters, points_to_scan, scan_bin_count, split_by_height,
    transform_matrix, transform_points)
import numpy as np
import pytest

W, H = 64, 48
K = np.array([[40.0, 0.0, 32.0], [0.0, 40.0, 24.0], [0.0, 0.0, 1.0]])
CAM_POS = np.array([0.4, 0.0, 0.30])    # in the ground frame (base_footprint)
PITCH = math.radians(20.0)              # tilted down
BOX_MIN = np.array([1.4, -0.2, 0.0])    # front face 1.4 m ahead of the origin
BOX_MAX = np.array([1.7, 0.2, 0.25])    # 0.25 m tall: under a 0.357 m lidar plane


def _ground_from_optical():
    # optical (x right, y down, z forward) -> camera_link (x fwd, y left, z up)
    link_from_optical = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    c, s = math.cos(PITCH), math.sin(PITCH)
    ground_from_link = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])  # +pitch = down
    m = np.eye(4)
    m[:3, :3] = ground_from_link @ link_from_optical
    m[:3, 3] = CAM_POS
    return m


def _render(with_box):
    """Optical-z depth image (metres, 0 = no return) of floor (+ box)."""
    m = _ground_from_optical()
    vs, us = np.mgrid[0:H, 0:W]
    d_opt = np.stack([(us - K[0, 2]) / K[0, 0], (vs - K[1, 2]) / K[1, 1],
                      np.ones_like(us, dtype=float)], axis=-1)   # optical z == 1
    d = d_opt @ m[:3, :3].T
    t = np.full((H, W), np.inf)
    down = d[..., 2] < 0
    t[down] = -CAM_POS[2] / d[..., 2][down]                       # floor z = 0
    if with_box:
        with np.errstate(divide='ignore', invalid='ignore'):
            t1 = (BOX_MIN - CAM_POS) / d
            t2 = (BOX_MAX - CAM_POS) / d
        t_near = np.nanmax(np.minimum(t1, t2), axis=-1)
        t_far = np.nanmin(np.maximum(t1, t2), axis=-1)
        hit = (t_near <= t_far) & (t_near > 0)
        t = np.where(hit & (t_near < t), t_near, t)
    t[t > 10.0] = 0.0                                             # out of range
    return t.astype(np.float32), m


def _pipeline(depth, m, min_hit_points=3):
    pts = deproject(depth, K, stride=1, min_depth=0.1, max_depth=6.0)
    ground = transform_points(pts, m)
    obstacle, floor = split_by_height(ground[:, 2], 0.05, 1.0)
    # scan_frame = base_link: base_footprint shifted up, same x/y
    ranges = points_to_scan(ground[obstacle, :2], ground[floor, :2],
                            -math.pi / 2, math.pi / 2, math.radians(0.5),
                            0.05, 6.0, min_hit_points=min_hit_points, free_min_points=1)
    return ground, obstacle, floor, ranges


def test_floor_is_not_an_obstacle():
    depth, m = _render(with_box=False)
    ground, obstacle, floor, ranges = _pipeline(depth, m)
    assert floor.sum() > 0.5 * depth.size          # the camera sees mostly floor
    assert obstacle.sum() == 0
    assert np.allclose(ground[floor, 2], 0.0, atol=1e-3)
    assert not np.isfinite(ranges).any()           # no hits anywhere
    assert np.isposinf(ranges).sum() > 20          # but known-free in view
    assert np.isnan(ranges[0]) and np.isnan(ranges[-1])   # outside FOV: unknown


def test_box_range_and_height():
    depth, m = _render(with_box=True)
    ground, obstacle, _, ranges = _pipeline(depth, m)
    assert obstacle.sum() > 20
    assert ground[obstacle, 2].max() <= BOX_MAX[2] + 1e-3
    assert ground[obstacle, 0].min() == pytest.approx(BOX_MIN[0], abs=1e-3)
    centre = scan_bin_count(-math.pi / 2, 0.0, math.radians(0.5)) - 1   # angle 0
    assert ranges[centre] == pytest.approx(BOX_MIN[0], abs=0.02)
    hits = np.isfinite(ranges)
    angles = -math.pi / 2 + np.nonzero(hits)[0] * math.radians(0.5)
    # the box spans +/-0.2 m at 1.4 m: about +/-8.1 deg, nothing outside it
    assert np.all(np.abs(angles) <= math.atan2(0.2, 1.4) + math.radians(0.5))


def test_single_speckle_needs_min_hit_points():
    obstacle = np.array([[0.5, 0.0], [2.0, 0.0], [2.01, 0.0], [2.02, 0.0]])
    none = np.empty((0, 2))
    args = (-0.1, 0.1, 0.01, 0.05, 5.0)
    zero = scan_bin_count(-0.1, 0.0, 0.01) - 1
    assert points_to_scan(obstacle, none, *args, min_hit_points=1)[zero] == pytest.approx(0.5)
    # a lone near pixel is outvoted: 3rd nearest is the real wall
    assert points_to_scan(obstacle, none, *args, min_hit_points=3)[zero] == pytest.approx(2.01)
    # too few points for a hit and no floor seen -> unknown
    assert np.isnan(points_to_scan(obstacle[:2], none, *args, min_hit_points=3)[zero])


def test_depth_to_meters_16uc1_with_row_padding():
    img = np.array([[1000, 0, 2500], [65535, 1, 42]], dtype=np.uint16)
    padded = np.zeros((2, 4), dtype=np.uint16)     # step = 8 bytes for width 3
    padded[:, :3] = img
    out = depth_to_meters(padded.tobytes(), '16UC1', 3, 2, 8, 0.001)
    assert out.shape == (2, 3)
    assert out[0, 0] == pytest.approx(1.0) and out[0, 1] == 0.0
    assert out[0, 2] == pytest.approx(2.5)


def test_depth_to_meters_32fc1_nan_is_invalid():
    img = np.array([[1.5, np.nan]], dtype=np.float32)
    out = depth_to_meters(img.tobytes(), '32FC1', 2, 1, 8)
    assert out[0, 0] == pytest.approx(1.5) and out[0, 1] == 0.0
    with pytest.raises(ValueError):
        depth_to_meters(img.tobytes(), 'rgb8', 2, 1, 8)


def test_transform_matrix_yaw_90():
    h = math.sqrt(0.5)
    m = transform_matrix(SimpleNamespace(x=1.0, y=2.0, z=3.0),
                         SimpleNamespace(x=0.0, y=0.0, z=h, w=h))
    assert np.allclose(transform_points(np.array([[1.0, 0.0, 0.0]]), m), [[1.0, 3.0, 3.0]])
