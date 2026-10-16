"""Local-planner geometry tests: resampling, monotonic progress, carrot, odom -> map transform."""
import math

import pytest

from hardware_bringup.route import (
    Route,
    cumulative_lengths,
    quaternion_to_yaw,
    resample,
    rotate_vector,
    transform_pose,
    yaw_to_quaternion,
)


def _route_on_polyline(p, polyline, tol=1e-9):
    """True if ``p`` lies on one of the polyline's segments."""
    for (x0, y0), (x1, y1) in zip(polyline, polyline[1:]):
        dx, dy = x1 - x0, y1 - y0
        seg2 = dx * dx + dy * dy
        t = ((p[0] - x0) * dx + (p[1] - y0) * dy) / seg2 if seg2 > 0.0 else 0.0
        t = min(1.0, max(0.0, t))
        if math.hypot(p[0] - (x0 + t * dx), p[1] - (y0 + t * dy)) < tol:
            return True
    return False


# ----------------------------------------------------------------------------------------------
# resampling


def test_resample_straight_line_is_uniform_and_keeps_both_ends():
    pts = resample([(0.0, 0.0), (10.0, 0.0)], spacing=0.5)
    assert pts[0] == (0.0, 0.0)
    assert pts[-1] == (10.0, 0.0)
    assert len(pts) == 21
    gaps = [b[0] - a[0] for a, b in zip(pts, pts[1:])]
    assert gaps == pytest.approx([0.5] * 20)


def test_resample_irregular_osm_like_polyline():
    # uneven vertex spacing and a duplicate where two OSM edges meet
    raw = [(0.0, 0.0), (0.3, 0.0), (3.0, 0.0), (3.0, 0.0), (3.0, 0.2), (3.0, 4.0)]
    pts = resample(raw, spacing=0.5)
    assert pts[0] == (0.0, 0.0)
    assert pts[-1] == (3.0, 4.0)
    for p in pts:
        assert _route_on_polyline(p, raw)
    # 7 m of arc sampled every 0.5 m; the corner (s = 3.0) lands on a sample, so every gap is 0.5
    assert len(pts) == 15
    gaps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])]
    assert gaps == pytest.approx([0.5] * 14)
    assert (3.0, 0.0) in [pytest.approx(p) for p in pts]
    assert Route(raw, spacing=0.5).length == pytest.approx(7.0)


def test_resample_cuts_corners_by_at_most_one_chord():
    # corner between samples: the resampled length is shorter than the arc, never longer
    raw = [(0.0, 0.0), (3.2, 0.0), (3.2, 4.0)]
    route = Route(raw, spacing=0.5)
    for p in route.points:
        assert _route_on_polyline(p, raw)
    assert 7.2 - 0.5 < route.length < 7.2


def test_resample_absorbs_a_short_final_sliver():
    pts = resample([(0.0, 0.0), (1.05, 0.0)], spacing=0.5)
    # 0, 0.5, then 1.05 replaces 1.0 instead of leaving a 5 cm last segment
    assert pts == [(0.0, 0.0), (0.5, 0.0), (1.05, 0.0)]


def test_resample_degenerate_inputs():
    assert resample([(1.0, 2.0)]) == [(1.0, 2.0)]
    assert resample([(1.0, 2.0), (1.0, 2.0), (1.0, 2.0)]) == [(1.0, 2.0)]
    assert resample([(0.0, 0.0), (0.1, 0.0)], spacing=0.5) == [(0.0, 0.0), (0.1, 0.0)]
    assert resample([]) == []
    with pytest.raises(ValueError):
        resample([(0.0, 0.0), (1.0, 0.0)], spacing=0.0)


def test_cumulative_lengths():
    assert cumulative_lengths([(0, 0), (3, 4), (3, 10)]) == pytest.approx([0.0, 5.0, 11.0])


# ----------------------------------------------------------------------------------------------
# progress


def test_progress_follows_the_robot_forward():
    route = Route([(0.0, 0.0), (30.0, 0.0)])
    s = 0.0
    for x in [0.5, 2.0, 3.6, 5.1]:
        s = route.advance(x, 0.4, s, window=4.0)
        assert s == pytest.approx(x)


def test_progress_never_goes_backwards():
    route = Route([(0.0, 0.0), (30.0, 0.0)])
    s = route.advance(10.0, 0.0, 9.0, window=4.0)
    assert s == pytest.approx(10.0)
    # robot reverses / swerves back
    assert route.advance(7.0, 1.0, s, window=4.0) == pytest.approx(10.0)
    assert route.advance(-5.0, 0.0, s, window=4.0) == pytest.approx(10.0)


def test_progress_does_not_jump_to_a_later_stretch_that_passes_close():
    # out along y=0, then back along y=2: a route that doubles back on a wide walkway
    route = Route([(0.0, 0.0), (20.0, 0.0), (20.0, 2.0), (0.0, 2.0)])
    robot = (1.0, 1.1)  # nearer the return leg than the outbound one
    unconstrained = route.project(*robot)
    assert unconstrained > 30.0  # what a naive nearest-point search would pick
    s = route.advance(*robot, 0.0, window=4.0)
    assert s == pytest.approx(1.0)


def test_progress_is_clamped_to_the_route_end():
    route = Route([(0.0, 0.0), (5.0, 0.0)])
    assert route.advance(9.0, 0.0, 4.0, window=10.0) == pytest.approx(5.0)


# ----------------------------------------------------------------------------------------------
# carrot


def test_carrot_sits_lookahead_ahead_on_the_route():
    route = Route([(0.0, 0.0), (10.0, 0.0), (10.0, 20.0)])
    s = route.carrot_s(progress=3.0, lookahead=12.0)
    assert s == pytest.approx(15.0)
    x, y, yaw = route.pose_at(s)
    assert (x, y) == pytest.approx((10.0, 5.0))
    assert yaw == pytest.approx(math.pi / 2)  # tangent after the corner


def test_carrot_offsets_move_it_along_the_route():
    route = Route([(0.0, 0.0), (40.0, 0.0)])
    carrots = [route.carrot_s(5.0, 12.0, off) for off in (0.0, 2.0, -2.0, 4.0, -4.0)]
    assert carrots == pytest.approx([17.0, 19.0, 15.0, 21.0, 13.0])


def test_carrot_is_clamped_to_the_final_goal():
    route = Route([(0.0, 0.0), (20.0, 0.0)])
    s = route.carrot_s(progress=15.0, lookahead=12.0, offset=4.0)
    assert s == pytest.approx(20.0)
    x, y, yaw = route.pose_at(s)
    assert (x, y) == pytest.approx((20.0, 0.0))
    assert yaw == pytest.approx(0.0)


def test_carrot_never_falls_behind_progress():
    route = Route([(0.0, 0.0), (20.0, 0.0)])
    assert route.carrot_s(progress=10.0, lookahead=1.0, offset=-4.0) == pytest.approx(10.0)


def test_final_yaw_applies_only_at_the_end():
    route = Route([(0.0, 0.0), (20.0, 0.0)], final_yaw=1.0)
    assert route.pose_at(19.0)[2] == pytest.approx(0.0)
    assert route.pose_at(20.0)[2] == pytest.approx(1.0)
    assert route.pose_at(99.0) == pytest.approx((20.0, 0.0, 1.0))


def test_single_point_route():
    route = Route([(3.0, 4.0)], final_yaw=0.5)
    assert route.length == 0.0
    assert route.advance(0.0, 0.0, 0.0, window=4.0) == 0.0
    assert route.pose_at(route.carrot_s(0.0, 12.0)) == pytest.approx((3.0, 4.0, 0.5))


# ----------------------------------------------------------------------------------------------
# odom -> map


def test_yaw_quaternion_round_trip():
    for yaw in (-3.0, -1.0, 0.0, 0.3, 3.0):
        assert quaternion_to_yaw(yaw_to_quaternion(yaw)) == pytest.approx(yaw)


def test_odom_to_map_with_yaw():
    # sim-like translation (z = 2) with a non-zero yaw so the rotation is actually exercised
    yaw = math.radians(30.0)
    t, q = (-95.0, 25.5, 2.0), yaw_to_quaternion(yaw)
    p, o = transform_pose(t, q, (10.0, 0.0, 0.0), yaw_to_quaternion(math.radians(10.0)))
    assert p == pytest.approx((-95.0 + 10.0 * math.cos(yaw), 25.5 + 10.0 * math.sin(yaw), 2.0))
    assert math.degrees(quaternion_to_yaw(o)) == pytest.approx(40.0)

    p, _ = transform_pose(t, q, (0.0, 4.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert p == pytest.approx((-95.0 - 4.0 * math.sin(yaw), 25.5 + 4.0 * math.cos(yaw), 2.0))


def test_grid_convergence_sized_yaw_matters_at_planning_range():
    # gps_datum_tf's map->odom yaw is ~1.685 deg: ignoring it puts a 12 m carrot ~0.35 m off
    q = yaw_to_quaternion(math.radians(1.685))
    p, _ = transform_pose((0.0, 0.0, 0.0), q, (12.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert p[1] == pytest.approx(0.353, abs=1e-3)


def test_rotation_preserves_length():
    q = yaw_to_quaternion(2.2)
    v = rotate_vector(q, (3.0, -4.0, 1.0))
    assert math.sqrt(sum(c * c for c in v)) == pytest.approx(math.sqrt(26.0))
