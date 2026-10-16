"""Local-planner geometry tests: resampling, monotonic progress, carrot, odom -> map transform,
and (TDM-28) finishing near the goal: final approach, loop rejection, duplicate poses."""
import math

import pytest

from hardware_bringup.route import (
    Route,
    angle_difference,
    cumulative_lengths,
    drop_duplicate_points,
    in_final_approach,
    is_loop,
    loop_excess,
    path_length,
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


# ----------------------------------------------------------------------------------------------
# TDM-28: finishing near the goal. The paths are TDM-22's Smac Hybrid-A* answers on the boundary
# map's 3 m corridor (map x, y, sampled every ~0.5 m; robot heading 84 deg).

TDM28_ROBOT_YAW = math.radians(84.0)

# robot 0.5 m left of the centreline, goal 1.12 m ahead on it: a 12.69 m loop instead of a failure
TDM28_NEAR_LOOP = [
    (-93.79, 2.05), (-93.85, 2.56), (-93.95, 3.07), (-94.05, 3.57), (-93.97, 4.19), (-93.54, 4.61),
    (-92.94, 4.67), (-92.45, 4.32), (-92.30, 3.74), (-92.55, 3.18), (-93.07, 2.78), (-93.52, 2.52),
    (-93.92, 2.22), (-94.25, 1.67), (-94.24, 1.05), (-93.88, 0.55), (-93.31, 0.39), (-92.77, 0.62),
    (-92.49, 1.09), (-92.53, 1.62), (-92.81, 2.10), (-93.14, 2.57), (-93.19, 2.99),
]
# the same start with the goal 1.58 m ahead: an S-curve fits
TDM28_NEAR_S_CURVE = [(-93.79, 2.05), (-93.59, 2.59), (-93.27, 3.08), (-93.14, 3.49)]
# goal 5 m behind the robot, facing back (264 deg): a forward U-turn, 12.42 m; Smac repeated the
# last pose (headings -100 and -95 deg at the same point)
TDM28_U_TURN = [
    (-93.30, 2.00), (-93.12, 2.49), (-92.81, 2.91), (-92.54, 3.35), (-92.32, 3.80), (-92.29, 4.43),
    (-92.64, 4.92), (-93.22, 5.07), (-93.76, 4.82), (-94.04, 4.27), (-94.01, 3.60), (-93.92, 3.10),
    (-93.85, 2.58), (-93.84, 2.06), (-93.86, 1.54), (-93.87, 1.02), (-93.85, 0.51), (-93.81, -0.00),
    (-93.79, -0.63), (-93.80, -1.22), (-93.80, -1.80), (-93.81, -2.39), (-93.82, -2.97), (-93.82, -2.97),
]


def test_tdm28_near_goal_loop_is_rejected():
    assert path_length(TDM28_NEAR_LOOP) > 10.0
    assert loop_excess(TDM28_NEAR_LOOP) > 9.0
    assert is_loop(TDM28_NEAR_LOOP, TDM28_ROBOT_YAW, TDM28_ROBOT_YAW, margin=5.0)
    # the goal heading only has to be within 90 deg of the robot's
    assert is_loop(TDM28_NEAR_LOOP, TDM28_ROBOT_YAW, TDM28_ROBOT_YAW + math.radians(80.0), margin=5.0)


def test_tdm28_s_curve_to_a_near_goal_passes():
    assert loop_excess(TDM28_NEAR_S_CURVE) == pytest.approx(0.0, abs=0.1)
    assert not is_loop(TDM28_NEAR_S_CURVE, TDM28_ROBOT_YAW, TDM28_ROBOT_YAW, margin=5.0)


def test_tdm28_u_turn_to_a_goal_behind_passes():
    # 7+ m longer than the straight line, like a loop, but the goal faces the other way
    assert loop_excess(TDM28_U_TURN) > 5.0
    assert not is_loop(TDM28_U_TURN, TDM28_ROBOT_YAW, math.radians(264.0), margin=5.0)
    assert not is_loop(TDM28_U_TURN, TDM28_ROBOT_YAW, TDM28_ROBOT_YAW + math.radians(91.0), margin=5.0)


def test_tdm28_route_arc_is_not_a_detour():
    # a switchback walkway: forward 4 m, back 4 m, forward 4 m, 2 m between the legs. The carrot
    # 12 m along the route faces the robot's way and is only ~5.7 m away in a straight line.
    switchback = [(0.0, 0.0), (4.0, 0.0), (4.0, 2.0), (0.0, 2.0), (0.0, 4.0), (4.0, 4.0)]
    assert is_loop(switchback, 0.0, 0.0, margin=5.0)
    assert not is_loop(switchback, 0.0, 0.0, margin=5.0, expected_length=path_length(switchback))
    # an arc shorter than the straight line changes nothing
    assert loop_excess(TDM28_NEAR_LOOP, expected_length=0.5) == pytest.approx(loop_excess(TDM28_NEAR_LOOP))


def test_tdm28_loop_heading_check_wraps_around_pi():
    assert angle_difference(math.radians(179.0), math.radians(-179.0)) == pytest.approx(math.radians(-2.0))
    assert is_loop(TDM28_NEAR_LOOP, math.radians(179.0), math.radians(-179.0), margin=5.0)
    assert not is_loop(TDM28_NEAR_LOOP, math.radians(179.0), math.radians(-1.0), margin=5.0)


def test_tdm28_degenerate_paths_are_never_loops():
    assert loop_excess([]) == 0.0
    assert loop_excess([(1.0, 1.0)]) == 0.0
    assert not is_loop([(1.0, 1.0)], 0.0, 0.0, margin=5.0)


def test_tdm28_duplicate_last_pose_is_dropped_keeping_the_goal_pose():
    keep = drop_duplicate_points(TDM28_U_TURN)
    n = len(TDM28_U_TURN)
    assert keep == list(range(n - 2)) + [n - 1]   # the goal's own pose (heading -95 deg) survives
    kept = [TDM28_U_TURN[i] for i in keep]
    gaps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(kept, kept[1:])]
    assert min(gaps) > 0.4                        # no zero-length segment left to take a heading from


def test_tdm28_near_duplicates_under_a_centimetre_merge():
    pts = [(0.0, 0.0), (0.004, 0.0), (0.5, 0.0), (0.5, 0.009), (1.0, 0.0), (1.0, 0.0), (1.005, 0.0)]
    # the first of each cluster stays, except the final cluster, which keeps its last (goal) pose
    assert drop_duplicate_points(pts) == [0, 2, 6]
    # 1 cm and more is a real step
    assert drop_duplicate_points([(0.0, 0.0), (0.01, 0.0), (0.02, 0.0)]) == [0, 1, 2]


def test_tdm28_duplicate_filter_degenerate_inputs():
    assert drop_duplicate_points([]) == []
    assert drop_duplicate_points([(2.0, 3.0)]) == [0]
    # a path that never leaves the robot's spot collapses to its final (goal) pose
    assert drop_duplicate_points([(2.0, 3.0), (2.0, 3.0), (2.0, 3.0)]) == [2]
    assert drop_duplicate_points([(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)]) == [0, 1, 2]


def test_tdm28_final_approach_needs_a_path_that_ends_at_the_goal():
    assert in_final_approach(1.5, 2.0, path_ends_at_goal=True)
    assert in_final_approach(2.0, 2.0, path_ends_at_goal=True)
    assert not in_final_approach(2.1, 2.0, path_ends_at_goal=True)
    # the follower's path stops short of the goal (a carrot offset, or an empty stop path): replan
    assert not in_final_approach(0.8, 2.0, path_ends_at_goal=False)


def test_tdm28_final_approach_hysteresis():
    assert not in_final_approach(2.3, 2.0, True, active=False)
    assert in_final_approach(2.3, 2.0, True, active=True)         # a swing out stays in it
    assert in_final_approach(2.5, 2.0, True, active=True, hysteresis=0.5)
    assert not in_final_approach(2.6, 2.0, True, active=True)     # well out: replan again
    assert not in_final_approach(1.0, 2.0, False, active=True)


def test_tdm28_carrot_reaches_the_goal_once_inside_the_lookahead():
    route = Route([(0.0, 0.0), (30.0, 0.0)])
    assert not route.reaches_end(route.carrot_s(10.0, 12.0))
    assert route.reaches_end(route.carrot_s(18.0, 12.0))
    assert route.reaches_end(route.carrot_s(28.0, 12.0, offset=-4.0))
    # a recovery offset can pull the carrot short of the goal: that path does not finish the job
    assert not route.reaches_end(route.carrot_s(16.0, 12.0, offset=-4.0))
    assert Route([(1.0, 1.0)]).reaches_end(0.0)
