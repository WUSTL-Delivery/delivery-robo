"""Local planner: turns a goal into Nav2 local paths, in ``map``, for the MPPI path follower.

Two kinds of goal:

* ``/goal_pose`` (``PoseStamped``, RViz "2D Goal Pose"): a direct goal. It becomes a two-point
  route from where the robot is now to the goal, ending at the goal's heading.
* ``/nav/goal_node`` (``std_msgs/Int64``, an OSM node id): the route comes from delivery-autonomy's
  ``/compute_shortest_path`` (``shared_types/ComputePath``), a polyline in ``map``.

Then, at ``rate_hz`` with at most one request in flight, the node puts a carrot ``lookahead``
metres ahead of the robot's (monotonic) progress along the route, clamped to the final goal,
and asks Nav2's planner_server for a path to it (``ComputePathToPose``, ``planner_id``). Why a
carrot instead of the whole route: the costmap is a rolling window around the robot (40 x 40 m),
so a goal much further than ~20 m is off the costmap; and replanning a short horizon twice a
second is what lets the path bend around the obstacles the scans add.

``rate_hz`` defaults to 2.0 (TDM-36). In TDM-25's sim runs the robot passed a box centred in a
3 m corridor 0.154 m away on average at 2 Hz, against 0.096 m at 1 Hz: the 2D lidar sees a
box's side faces only late, and the faster replan reacts to them sooner at 1.5 m/s. It costs
little: each plan takes 2-15 ms. The price: a transient false obstacle (the lidar marking the
foot of a slope across the corridor for about a second) is more likely to catch a replan, fail
it and send a stop path (one of the two 2 Hz runs had 4 failures and 2 stop paths but never
halted; the 1 Hz runs had none). The timer is created once, so the rate is read at startup
only. A plan that takes longer than one period just skips ticks, because only one request is
in flight.

The planner answers in its costmap's frame (``odom``; the costmap has to be in ``odom`` because
the sim's ``map -> odom`` has z = 2 and the obstacle layer filters heights in its global frame).
The follower (``/follow_path``, delivery-autonomy's MPPI) treats every path as ``map``
coordinates and ignores ``header.frame_id``, so the path is transformed ``odom -> map`` here,
with the full ``map -> odom`` transform (on the robot that includes ~1.7 deg of grid convergence,
i.e. ~0.35 m of lateral error at 12 m if it were dropped). The result is always published on
``/nav/local_plan``; with ``drive: true`` it is also sent to ``/follow_path``, where each new
goal pre-empts the previous path.

Failures (planner rejects/aborts/times out, empty path, no transform): the first failure of a
streak sends one empty path, which the follower treats as "stop now"; the following attempts
move the carrot along the route by ``carrot_offsets`` (default 0, +2, -2, +4, -4 m, cycling;
along the route because the boundary map is free only within 1.5 m of an OSM edge). The first
success ends the streak. ``/nav/cancel`` (``std_msgs/Empty``) sends an empty path and forgets
the goal.

Finishing (TDM-28). The planner only has forward Dubins moves with a 1.0 m turning radius, and
TDM-22 measured what it does with a goal too close for an S-curve (0.5 m to the side and ~1 m
ahead): it does not fail, it answers with an 11-13 m loop that circles back to the goal, which
the follower would drive. Three things keep that from reaching the follower:

* Final approach: once the path last forwarded already ends at the final goal and the robot is
  within ``final_approach_distance`` of it, the node stops replanning (and drops any request
  still in flight) and lets the follower finish that path, which was planned from further out.
  It replans again only if the robot ends up 0.5 m further out than that. The price: in that
  last stretch a new obstacle no longer makes planning fail, so it no longer stops the robot.
  ``goal_tolerance`` now only decides when the goal counts as reached (logged, then forgotten).
* Loops: a plan more than ``loop_margin`` longer than it needs to be (than the straight line to
  its end, or the route's own arc to the carrot when that is longer) is a planning failure,
  reason "loop", when the carrot's heading is within 90 deg of the robot's. A goal behind the
  robot that faces back still gets its forward U-turn. ``loop_margin <= 0`` turns this off.
* Duplicates: consecutive poses less than 1 cm apart (Smac can repeat its last pose) are merged
  before a path is published or forwarded; the follower derives headings from segment
  directions, and a zero-length segment has none.

Runs on a single-threaded executor: every action call is asynchronous (done-callbacks), nothing
blocks a callback, and TF is read with the latest-available lookup.
"""
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from shared_types.action import ComputePath, FollowPath
from std_msgs.msg import Empty, Int64
from tf2_ros import Buffer, TransformException, TransformListener

from hardware_bringup.route import (
    Route,
    angle_difference,
    drop_duplicate_points,
    in_final_approach,
    is_loop,
    loop_excess,
    path_length,
    quaternion_to_yaw,
    transform_pose,
    yaw_to_quaternion,
)

_STATUS_NAMES = {
    getattr(GoalStatus, n): n[len('STATUS_'):].lower() for n in dir(GoalStatus) if n.startswith('STATUS_')
}
_PLAN_ERRORS = {
    getattr(ComputePathToPose.Result, n): n
    for n in dir(ComputePathToPose.Result)
    if n.isupper() and isinstance(getattr(ComputePathToPose.Result, n), int)
}


class LocalPlanner(Node):
    def __init__(self):
        super().__init__('local_planner')
        p = self._declare
        self._map_frame = p('map_frame', 'map')
        self._robot_frame = p('robot_frame', 'base_link')
        self._drive = bool(p('drive', False))
        self._planner_id = p('planner_id', 'GridBased')
        self._lookahead = float(p('lookahead', 12.0))
        self._offsets = [float(o) for o in p('carrot_offsets', [0.0, 2.0, -2.0, 4.0, -4.0])] or [0.0]
        self._goal_tolerance = float(p('goal_tolerance', 0.5))
        self._approach_distance = float(p('final_approach_distance', 2.0))
        self._loop_margin = float(p('loop_margin', 5.0))
        self._spacing = float(p('route_spacing', 0.5))
        self._window = float(p('progress_window', 4.0))
        self._planner_timeout = float(p('planner_timeout', 5.0))
        rate_hz = float(p('rate_hz', 2.0))   # TDM-36: 2 Hz cleared obstacles better than 1 Hz (TDM-25)
        planner_action = p('planner_action', '/compute_path_to_pose')
        route_action = p('route_action', '/compute_shortest_path')
        follow_action = p('follow_action', '/follow_path')
        plan_topic = p('plan_topic', '/nav/local_plan')
        goal_topic = p('goal_topic', '/goal_pose')
        goal_node_topic = p('goal_node_topic', '/nav/goal_node')
        cancel_topic = p('cancel_topic', '/nav/cancel')

        self._route = None         # Route in map, or None when there is no goal
        self._goal_gen = 0         # bumped on every new goal / cancel / arrival; stale replies compare it
        self._progress = 0.0       # arc length along the route, monotonic
        self._inflight = None      # the one outstanding ComputePathToPose request (a dict), or None
        self._failures = 0         # consecutive planning failures
        self._stop_sent = False    # one empty path per failure streak
        self._ends_at_goal = False   # the path last forwarded ends at the final goal
        self._final_approach = False  # stopped replanning near the goal (see in_final_approach)
        self._log_next_plan = True
        self._last_plan_log = 0.0
        self._plans = 0

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._planner = ActionClient(self, ComputePathToPose, planner_action)
        self._router = ActionClient(self, ComputePath, route_action)
        self._follower = ActionClient(self, FollowPath, follow_action) if self._drive else None
        self._plan_pub = self.create_publisher(Path, plan_topic, 10)
        self.create_subscription(PoseStamped, goal_topic, self._on_goal_pose, 10)
        self.create_subscription(Int64, goal_node_topic, self._on_goal_node, 10)
        self.create_subscription(Empty, cancel_topic, self._on_cancel, 10)
        self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f'local_planner: {goal_topic} | {goal_node_topic} -> {route_action}; carrot {self._lookahead} m '
            f'-> {planner_action} ({self._planner_id}) at {rate_hz} Hz -> {plan_topic} (frame {self._map_frame})'
            + (f' -> {follow_action}' if self._drive else ' [drive:=false, plan only]')
            + f'; final approach {self._approach_distance} m, loop_margin {self._loop_margin} m'
        )

    def _declare(self, name, default):
        return self.declare_parameter(name, default).value

    # ------------------------------------------------------------------ TF helpers
    def _lookup(self, target, source):
        """``(translation, rotation)`` of the latest ``target <- source`` transform."""
        t = self._tf.lookup_transform(target, source, Time()).transform
        return ((t.translation.x, t.translation.y, t.translation.z),
                (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))

    def _robot_pose(self):
        """``(x, y, yaw)`` of the robot in the map frame, or None without a transform."""
        try:
            (x, y, _), q = self._lookup(self._map_frame, self._robot_frame)
        except TransformException as ex:
            self.get_logger().warning(
                f'no {self._map_frame}->{self._robot_frame} transform: {ex}', throttle_duration_sec=5.0)
            return None
        return x, y, quaternion_to_yaw(q)

    def _robot_xy(self):
        pose = self._robot_pose()
        return pose[:2] if pose is not None else None

    def _to_map(self, path):
        """Copy of ``path`` with every pose transformed into the map frame."""
        frame = path.header.frame_id or (path.poses[0].header.frame_id if path.poses else '')
        if not frame:
            raise TransformException('path has no frame_id')
        out = Path()
        out.header.stamp = path.header.stamp
        out.header.frame_id = self._map_frame
        if frame == self._map_frame:
            out.poses = list(path.poses)
            for ps in out.poses:
                ps.header.frame_id = self._map_frame
            return out
        t, q = self._lookup(self._map_frame, frame)
        for ps in path.poses:
            pos, ori = ps.pose.position, ps.pose.orientation
            (x, y, z), (qx, qy, qz, qw) = transform_pose(t, q, (pos.x, pos.y, pos.z), (ori.x, ori.y, ori.z, ori.w))
            m = PoseStamped()
            m.header.stamp = ps.header.stamp
            m.header.frame_id = self._map_frame
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
            m.pose.orientation.x, m.pose.orientation.y = qx, qy
            m.pose.orientation.z, m.pose.orientation.w = qz, qw
            out.poses.append(m)
        return out

    # ------------------------------------------------------------------ goals
    def _new_goal(self):
        """Forget the current goal (and any reply still on its way); returns the new generation."""
        self._goal_gen += 1
        self._route = None
        self._progress = 0.0
        self._failures = 0
        self._stop_sent = False
        self._ends_at_goal = False
        self._final_approach = False
        self._log_next_plan = True
        self._abandon_plan()
        return self._goal_gen

    def _start_route(self, route, gen, what):
        if gen != self._goal_gen:
            return
        robot = self._robot_xy()
        self._route = route
        self._progress = route.advance(*robot, 0.0, self._window) if robot else 0.0
        self.get_logger().info(
            f'new goal: {what}; route {route.length:.1f} m, {len(route.points)} points, '
            f'end ({route.end[0]:.2f}, {route.end[1]:.2f}), progress {self._progress:.1f} m')
        self._tick()

    def _on_goal_pose(self, msg):
        gen = self._new_goal()
        frame = msg.header.frame_id or self._map_frame
        pos, ori = msg.pose.position, msg.pose.orientation
        q = (ori.x, ori.y, ori.z, ori.w)
        try:
            if frame != self._map_frame:
                t, r = self._lookup(self._map_frame, frame)
                (gx, gy, _), q = transform_pose(t, r, (pos.x, pos.y, pos.z), q)
            else:
                gx, gy = pos.x, pos.y
        except TransformException as ex:
            self.get_logger().error(f'ignoring goal in {frame}: no transform to {self._map_frame}: {ex}')
            return
        robot = self._robot_xy()
        if robot is None:
            self.get_logger().error('ignoring goal: the robot pose in the map frame is unknown')
            return
        # An all-zero quaternion (a hand-written goal) means "no heading wanted"
        yaw = quaternion_to_yaw(q) if sum(c * c for c in q) > 0.5 else None
        route = Route([robot, (gx, gy)], self._spacing, final_yaw=yaw)
        heading = f', heading {math.degrees(yaw):.0f} deg' if yaw is not None else ''
        self._start_route(route, gen, f'direct goal ({gx:.2f}, {gy:.2f}){heading}')

    def _on_goal_node(self, msg):
        gen = self._new_goal()
        node_id = msg.data
        if not self._router.server_is_ready():
            self.get_logger().error(f'ignoring goal node {node_id}: no route server')
            return
        self.get_logger().info(f'requesting a route to OSM node {node_id}')
        fut = self._router.send_goal_async(ComputePath.Goal(goal_node_id=node_id))
        fut.add_done_callback(lambda f: self._on_route_accepted(f, gen, node_id))

    def _on_route_accepted(self, fut, gen, node_id):
        handle = fut.result()
        if not handle.accepted:
            if gen == self._goal_gen:
                self.get_logger().error(f'route to node {node_id} rejected')
            return
        handle.get_result_async().add_done_callback(lambda f: self._on_route_result(f, gen, node_id))

    def _on_route_result(self, fut, gen, node_id):
        if gen != self._goal_gen:
            return
        res = fut.result()
        path = res.result.path
        if res.status != GoalStatus.STATUS_SUCCEEDED or not path.poses:
            self.get_logger().error(
                f'no route to node {node_id} ({_STATUS_NAMES.get(res.status, res.status)}, '
                f'{len(path.poses)} poses)')
            self._send_stop('no route')
            return
        try:
            path = self._to_map(path)
        except TransformException as ex:
            self.get_logger().error(f'cannot move the route to node {node_id} into {self._map_frame}: {ex}')
            self._send_stop('no route')
            return
        points = [(ps.pose.position.x, ps.pose.position.y) for ps in path.poses]
        self._start_route(Route(points, self._spacing), gen, f'OSM node {node_id}')

    def _on_cancel(self, _msg):
        had_goal = self._route is not None
        self._new_goal()
        self.get_logger().info('cancel: goal cleared' if had_goal else 'cancel (no goal was active)')
        self._send_stop('cancel')

    # ------------------------------------------------------------------ planning loop
    def _tick(self):
        route = self._route
        if route is None:
            return
        pose = self._robot_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose
        self._progress = route.advance(rx, ry, self._progress, self._window)
        remaining = route.length - self._progress
        to_goal = math.hypot(route.end[0] - rx, route.end[1] - ry)
        if to_goal <= self._goal_tolerance and remaining <= self._lookahead:
            self.get_logger().info(
                f'goal reached: {to_goal:.2f} m from ({route.end[0]:.2f}, {route.end[1]:.2f}) '
                f'(tolerance {self._goal_tolerance} m); goal cleared, the follower finishes the last path')
            self._new_goal()
            return

        approach = in_final_approach(to_goal, self._approach_distance, self._ends_at_goal, self._final_approach)
        if approach != self._final_approach:
            self._final_approach = approach
            if approach:
                self._abandon_plan()  # its answer could only replace a path that already ends at the goal
                self.get_logger().info(
                    f'final approach: {to_goal:.2f} m from the goal (final_approach_distance '
                    f'{self._approach_distance} m) and the last path ends there; stopped replanning, '
                    f'the follower finishes it')
            else:
                self.get_logger().warning(
                    f'left the final approach ({to_goal:.2f} m from the goal); replanning')
        if approach:
            return

        if self._inflight is not None:
            waited = time.monotonic() - self._inflight['sent']
            if waited > self._planner_timeout:
                self._abandon_plan()
                self._on_plan_failure(f'no answer after {waited:.1f} s')
            return

        if not self._planner.server_is_ready():
            self.get_logger().warning('waiting for the planner action server', throttle_duration_sec=5.0)
            return
        offset = self._offsets[self._failures % len(self._offsets)]
        s = route.carrot_s(self._progress, self._lookahead, offset)
        x, y, yaw = route.pose_at(s)
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = self._map_frame
        # Zero stamp: planner_server transforms the goal with the latest map->odom (it is static)
        goal.goal.header.stamp = Time().to_msg()
        goal.goal.pose.position.x, goal.goal.pose.position.y = x, y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        goal.goal.pose.orientation.z, goal.goal.pose.orientation.w = qz, qw
        goal.planner_id = self._planner_id
        goal.use_start = False

        req = {'gen': self._goal_gen, 'sent': time.monotonic(), 'handle': None,
               'carrot': (x, y, s, offset), 'carrot_yaw': yaw, 'robot_yaw': ryaw,
               'progress': self._progress, 'remaining': remaining}
        self._inflight = req
        self._planner.send_goal_async(goal).add_done_callback(lambda f: self._on_plan_accepted(f, req))

    def _abandon_plan(self):
        req, self._inflight = self._inflight, None
        if req is not None and req['handle'] is not None:
            req['handle'].cancel_goal_async()

    def _on_plan_accepted(self, fut, req):
        handle = fut.result()
        if self._inflight is not req:
            if handle.accepted:
                handle.cancel_goal_async()  # abandoned before the server answered
            return
        if not handle.accepted:
            self._inflight = None
            self._on_plan_failure('request rejected')
            return
        req['handle'] = handle
        handle.get_result_async().add_done_callback(lambda f: self._on_plan_result(f, req))

    def _on_plan_result(self, fut, req):
        if self._inflight is not req:
            return
        self._inflight = None
        res = fut.result()
        result = res.result
        if res.status != GoalStatus.STATUS_SUCCEEDED or not result.path.poses:
            code = _PLAN_ERRORS.get(result.error_code, result.error_code)
            detail = f': {result.error_msg}' if result.error_msg else ''
            self._on_plan_failure(
                f'{_STATUS_NAMES.get(res.status, res.status)}, {code}, {len(result.path.poses)} poses{detail}')
            return
        src = result.path.header.frame_id
        try:
            path = self._to_map(result.path)
        except TransformException as ex:
            self._on_plan_failure(f'cannot transform the path from {src!r} to {self._map_frame}: {ex}')
            return

        points = [(ps.pose.position.x, ps.pose.position.y) for ps in path.poses]
        keep = drop_duplicate_points(points)
        dropped = len(points) - len(keep)
        if dropped:
            path.poses = [path.poses[i] for i in keep]
            points = [points[i] for i in keep]
        x, y, s, offset = req['carrot']
        route_arc = s - req['progress']  # the route's own length to the carrot; a path may need that much
        if self._loop_margin > 0.0 and is_loop(
                points, req['robot_yaw'], req['carrot_yaw'], self._loop_margin, route_arc):
            (x0, y0), (x1, y1) = points[0], points[-1]
            turn = math.degrees(angle_difference(req['carrot_yaw'], req['robot_yaw']))
            self._on_plan_failure(
                f'loop: {path_length(points):.1f} m path to a carrot {math.hypot(x1 - x0, y1 - y0):.1f} m away '
                f'({route_arc:.1f} m along the route), {loop_excess(points, route_arc):.1f} m more than needed '
                f'> loop_margin {self._loop_margin} m; carrot heading {turn:+.0f} deg off the robot heading')
            return

        recovered = self._failures > 0
        self._failures = 0
        self._stop_sent = False
        self._plans += 1
        self._plan_pub.publish(path)
        forwarded = self._send_follow(path, 'plan') if self._drive else True
        if forwarded:
            self._ends_at_goal = self._route.reaches_end(s)

        # The first plan of a goal and the first after a failure streak always log; the rest every 5 s.
        # (rclpy refuses a call site whose throttle changes between calls, hence the manual throttle.)
        now = time.monotonic()
        if not (self._log_next_plan or recovered or now - self._last_plan_log >= 5.0):
            return
        self._log_next_plan = False
        self._last_plan_log = now
        a, b = result.path.poses[0].pose.position, result.path.poses[-1].pose.position
        c, d = path.poses[0].pose.position, path.poses[-1].pose.position
        self.get_logger().info(
            f'plan {self._plans}: {len(path.poses)} poses, {src} ({a.x:.2f}, {a.y:.2f})..({b.x:.2f}, {b.y:.2f}) '
            f'-> {self._map_frame} ({c.x:.2f}, {c.y:.2f})..({d.x:.2f}, {d.y:.2f}); carrot ({x:.2f}, {y:.2f}) '
            f'offset {offset:+.1f} m, progress {req["progress"]:.1f}, remaining {req["remaining"]:.1f} m'
            + (f', {dropped} duplicate poses dropped' if dropped else '')
            + (' [ends at the goal]' if self._route.reaches_end(s) else '')
            + (' [recovered]' if recovered else ''))

    def _on_plan_failure(self, reason):
        self._failures += 1
        nxt = self._offsets[self._failures % len(self._offsets)]
        self.get_logger().warning(
            f'planning failed ({reason}); {self._failures} in a row, next carrot offset {nxt:+.1f} m')
        if not self._stop_sent:
            self._stop_sent = True
            self._send_stop('planning failed')

    # ------------------------------------------------------------------ output
    def _send_stop(self, why):
        empty = Path()
        empty.header.stamp = self.get_clock().now().to_msg()
        empty.header.frame_id = self._map_frame
        self._ends_at_goal = False
        self._plan_pub.publish(empty)
        if self._drive:
            self._send_follow(empty, f'stop ({why})')
        self.get_logger().info(f'stop: empty path sent ({why})' + ('' if self._drive else ' [drive:=false]'))

    def _send_follow(self, path, what):
        """Send ``path`` to the follower; False if there is no follower server to send it to."""
        if not self._follower.server_is_ready():
            self.get_logger().warning(
                f'no path follower server; {what} not sent', throttle_duration_sec=5.0)
            return False
        fut = self._follower.send_goal_async(FollowPath.Goal(path=path))
        fut.add_done_callback(lambda f: self._on_follow_accepted(f, what))
        return True

    def _on_follow_accepted(self, fut, what):
        if not fut.result().accepted:
            self.get_logger().error(f'path follower rejected the {what} path')


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Jazzy's SIGINT handler shuts the context down and spin raises the latter
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
