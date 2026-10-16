"""Pure geometry for the local planner: route resampling, monotonic progress, carrot, transforms.

No ROS imports, so ``test/test_route.py`` runs as plain pytest.

The OSM route from ``/compute_shortest_path`` is a polyline in ``map`` with irregular vertex
spacing (edge geometries are concatenated, so it can contain duplicate points where edges
meet). ``Route`` resamples it to a uniform spacing and parameterises it by arc length ``s``.
The local planner then keeps a progress value along it and asks Nav2 for a path to a carrot
``lookahead`` metres further on.

Progress is monotonic on purpose. Projecting the robot onto the whole route would let it jump
to a later stretch that happens to pass close by (a route that doubles back along the same
walkway, or loops around a building), or fall back to an earlier one while the robot swerves
around an obstacle. So each update only searches ``[s_prev, s_prev + window]`` and never
returns less than ``s_prev``.

The carrot offsets are along the route, not sideways: the boundary map is free only within
1.5 m of an OSM edge, so a carrot moved 2-4 m to the side would land in occupied space.
"""
import math

DEFAULT_SPACING = 0.5


def resample(points, spacing=DEFAULT_SPACING):
    """Return ``points`` (a sequence of ``(x, y)``) resampled every ``spacing`` metres of arc length.

    The first and last input points are always kept. Duplicate consecutive points are dropped.
    A route of one distinct point comes back as that single point.
    """
    if spacing <= 0.0:
        raise ValueError(f'spacing must be positive, got {spacing}')
    pts = []
    for x, y in points:
        p = (float(x), float(y))
        if not pts or math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 1e-9:
            pts.append(p)
    if len(pts) < 2:
        return pts

    out = [pts[0]]
    carried = 0.0  # arc length walked since the last emitted point
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        d = spacing - carried
        while d <= seg + 1e-12:
            t = d / seg
            out.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
            d += spacing
        carried = seg - (d - spacing)
    # Keep the exact endpoint; drop a sliver of a final segment rather than emit a near-duplicate.
    last = pts[-1]
    if math.hypot(last[0] - out[-1][0], last[1] - out[-1][1]) < 0.25 * spacing and len(out) > 1:
        out[-1] = last
    elif out[-1] != last:
        out.append(last)
    return out


def cumulative_lengths(points):
    s = [0.0]
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        s.append(s[-1] + math.hypot(x1 - x0, y1 - y0))
    return s


class Route:
    """A resampled polyline in one frame, parameterised by arc length.

    ``final_yaw`` is the heading wanted at the end (a direct goal's orientation). Without it the
    end inherits the last segment's heading.
    """

    def __init__(self, points, spacing=DEFAULT_SPACING, final_yaw=None):
        self.points = resample(points, spacing)
        if not self.points:
            raise ValueError('route has no points')
        self.s = cumulative_lengths(self.points)
        self.final_yaw = final_yaw

    @property
    def length(self):
        return self.s[-1]

    @property
    def end(self):
        return self.points[-1]

    def project(self, x, y, s_min=0.0, window=math.inf):
        """Arc length of the route point closest to ``(x, y)``, searching only segments that
        overlap ``[s_min, s_min + window]``. The result is not clamped to ``s_min``."""
        if len(self.points) == 1:
            return 0.0
        best_d2, best_s = math.inf, s_min
        s_max = s_min + window
        for i in range(len(self.points) - 1):
            s0, s1 = self.s[i], self.s[i + 1]
            if s1 < s_min:
                continue
            if s0 > s_max:
                break
            (x0, y0), (x1, y1) = self.points[i], self.points[i + 1]
            dx, dy = x1 - x0, y1 - y0
            seg2 = dx * dx + dy * dy
            if seg2 <= 0.0:
                continue
            t = ((x - x0) * dx + (y - y0) * dy) / seg2
            t = min(1.0, max(0.0, t))
            px, py = x0 + t * dx, y0 + t * dy
            d2 = (x - px) ** 2 + (y - py) ** 2
            if d2 < best_d2:
                best_d2, best_s = d2, s0 + t * (s1 - s0)
        return best_s

    def advance(self, x, y, s_prev, window):
        """New progress for a robot at ``(x, y)``: never behind ``s_prev``, never past the end."""
        s = self.project(x, y, s_min=s_prev, window=window)
        return min(self.length, max(s_prev, s))

    def carrot_s(self, progress, lookahead, offset=0.0):
        """Arc length of the carrot: ``lookahead + offset`` ahead of ``progress``, kept on the route
        and never behind the robot's progress."""
        return min(self.length, max(progress, progress + lookahead + offset))

    def pose_at(self, s):
        """``(x, y, yaw)`` at arc length ``s`` (clamped to the route); yaw is the route tangent."""
        if len(self.points) == 1:
            x, y = self.points[0]
            return x, y, self.final_yaw if self.final_yaw is not None else 0.0
        s = min(self.length, max(0.0, s))
        if s >= self.length - 1e-9 and self.final_yaw is not None:
            x, y = self.end
            return x, y, self.final_yaw
        i = _segment_index(self.s, s)
        (x0, y0), (x1, y1) = self.points[i], self.points[i + 1]
        seg = self.s[i + 1] - self.s[i]
        t = (s - self.s[i]) / seg if seg > 0.0 else 0.0
        return x0 + t * (x1 - x0), y0 + t * (y1 - y0), math.atan2(y1 - y0, x1 - x0)


def _segment_index(s_list, s):
    """Index ``i`` of the segment ``[s_list[i], s_list[i+1]]`` containing ``s``."""
    lo, hi = 0, len(s_list) - 2
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if s_list[mid] <= s:
            lo = mid
        else:
            hi = mid - 1
    return lo


# ----------------------------------------------------------------------------------------------
# Rigid transforms. Quaternions are (x, y, z, w), like geometry_msgs.


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_vector(q, v):
    """Rotate the 3-vector ``v`` by the unit quaternion ``q``."""
    qv = (v[0], v[1], v[2], 0.0)
    q_conj = (-q[0], -q[1], -q[2], q[3])
    x, y, z, _ = quaternion_multiply(quaternion_multiply(q, qv), q_conj)
    return (x, y, z)


def transform_pose(translation, rotation, position, orientation):
    """Apply the transform ``(translation, rotation)`` (a tf2 ``target <- source`` transform) to a
    pose given in the source frame. Returns ``(position, orientation)`` in the target frame."""
    rx, ry, rz = rotate_vector(rotation, position)
    p = (translation[0] + rx, translation[1] + ry, translation[2] + rz)
    return p, quaternion_multiply(rotation, orientation)
