#!/usr/bin/env python3
"""
Depth obstacles: RealSense depth image -> costmap-ready obstacles.

Subscribes to the depth image + camera_info from the realsense2_camera driver
and publishes two things a Nav2 costmap obstacle layer can consume:

  /camera/obstacles  sensor_msgs/PointCloud2, in the depth optical frame (so
                     costmap raytracing starts at the camera). Only points
                     between min_obstacle_height and max_obstacle_height above
                     the ground, so the floor and overhangs above the robot
                     are removed.
  /camera/scan       sensor_msgs/LaserScan in scan_frame (base_link): the same
                     obstacles flattened into angle bins, nearest hit per bin.

Why not depthimage_to_laserscan: it reads a few pixel rows around the image
centre and has no notion of the ground, so a camera tilted down reports the
floor as a wall. This node deprojects every `pixel_stride`-th pixel (numpy,
~20k points at the default stride), moves the points into ground_frame
(base_footprint, z = 0 is the ground the wheels stand on) via TF and filters
by height there.

Scan bins follow REP-117:
  finite range  obstacle hit (min_hit_points-th nearest, so one speckle pixel
                does not make a hit)
  +inf          no obstacle, but at least free_min_points floor points were
                seen in that direction, so it is known free (costmap
                inf_is_valid: true clears it out to raytrace_max_range)
  NaN           nothing valid seen (outside the FOV, IR holes): no information

The camera is fixed on the robot, so each TF (optical -> ground_frame,
optical -> scan_frame) is looked up once and cached.

    ros2 run my_bringup depth_obstacles --ros-args -p pixel_stride:=4
"""
import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener


# --- pure numpy core (no ROS; unit-tested in test/test_depth_obstacles.py) --

def depth_to_meters(data, encoding, width, height, step, depth_scale=0.001):
    """
    Decode a sensor_msgs/Image depth payload into an HxW float32 metres array.

    16UC1 (RealSense default, millimetres) is multiplied by depth_scale;
    32FC1 is already metres. Invalid pixels come out as 0.
    """
    buf = np.frombuffer(data, dtype=np.uint8)   # bytes / array('B'), no copy
    if encoding in ('16UC1', 'mono16'):
        img = buf.view(np.uint16).reshape(height, step // 2)[:, :width]
        return img.astype(np.float32) * np.float32(depth_scale)
    if encoding == '32FC1':
        img = buf.view(np.float32).reshape(height, step // 4)[:, :width]
        return np.nan_to_num(img, nan=0.0, posinf=0.0, neginf=0.0).astype(np.float32)
    raise ValueError(f'unsupported depth encoding {encoding!r} (want 16UC1 or 32FC1)')


def deproject(depth_m, k, stride=1, min_depth=0.0, max_depth=float('inf')):
    """
    Pinhole-deproject every `stride`-th pixel of a depth image.

    depth_m: HxW metres (0 = invalid). k: 3x3 intrinsics (camera_info K).
    Returns Nx3 float32 points in the optical frame (x right, y down, z
    forward), keeping only min_depth <= z <= max_depth.
    """
    fx, fy, cx, cy = k[0][0], k[1][1], k[0][2], k[1][2]
    sub = depth_m[::stride, ::stride]
    vs, us = np.mgrid[0:depth_m.shape[0]:stride, 0:depth_m.shape[1]:stride]
    z = sub.ravel()
    keep = (z > 0.0) & (z >= min_depth) & (z <= max_depth)
    z = z[keep]
    x = (us.ravel()[keep] - cx) * z / fx
    y = (vs.ravel()[keep] - cy) * z / fy
    return np.stack([x, y, z], axis=1).astype(np.float32)


def transform_points(points, matrix):
    """Apply a 4x4 homogeneous transform to Nx3 points."""
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def split_by_height(heights, min_obstacle_height, max_obstacle_height):
    """
    Masks (obstacle, floor) from point heights above the ground.

    obstacle: min_obstacle_height < h < max_obstacle_height
    floor:    h <= min_obstacle_height (evidence the direction is free)
    Points above max_obstacle_height are neither (overhangs the robot fits
    under).
    """
    obstacle = (heights > min_obstacle_height) & (heights < max_obstacle_height)
    floor = heights <= min_obstacle_height
    return obstacle, floor


def scan_bin_count(angle_min, angle_max, angle_increment):
    """Return the number of LaserScan bins from angle_min to (at most) angle_max."""
    return int(math.floor((angle_max - angle_min) / angle_increment + 1e-9)) + 1


def points_to_scan(obstacle_xy, floor_xy, angle_min, angle_max, angle_increment,
                   range_min, range_max, min_hit_points=1, free_min_points=1):
    """
    Flatten 2D points (scan frame) into LaserScan ranges (REP-117 semantics).

    Bin i is centred on angle_min + i * angle_increment (the LaserScan angle
    of ranges[i]); there are scan_bin_count(...) of them. Each holds the
    min_hit_points-th nearest obstacle range, else +inf where at least
    free_min_points floor points fall in the bin, else NaN.
    free_min_points <= 0 disables +inf (no free-space reports).
    """
    n_bins = scan_bin_count(angle_min, angle_max, angle_increment)
    ranges = np.full(n_bins, np.nan, dtype=np.float32)

    def binned(xy):
        r = np.hypot(xy[:, 0], xy[:, 1])
        b = np.rint((np.arctan2(xy[:, 1], xy[:, 0]) - angle_min) / angle_increment)
        keep = (b >= 0) & (b < n_bins) & (r >= range_min) & (r <= range_max)
        return b[keep].astype(np.int64), r[keep]

    if free_min_points > 0 and len(floor_xy):
        fb, _ = binned(floor_xy)
        counts = np.bincount(fb, minlength=n_bins)
        ranges[counts >= free_min_points] = np.inf

    if len(obstacle_xy):
        ob, orng = binned(obstacle_xy)
        if len(ob):
            order = np.lexsort((orng, ob))        # by bin, then by range
            ob, orng = ob[order], orng[order]
            bins, first, counts = np.unique(ob, return_index=True, return_counts=True)
            k = max(int(min_hit_points), 1)
            hit = counts >= k
            ranges[bins[hit]] = orng[first[hit] + k - 1]
    return ranges


def transform_matrix(translation, rotation):
    """4x4 transform from a geometry_msgs Transform's translation + quaternion."""
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    m = np.eye(4)
    m[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    m[:3, 3] = [translation.x, translation.y, translation.z]
    return m


# --- ROS node ---------------------------------------------------------------

_XYZ_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]


class DepthObstacles(Node):

    def __init__(self):
        super().__init__('depth_obstacles')
        p = self.declare_parameter
        depth_topic = p('depth_topic', '/camera/depth/image_rect_raw').value
        info_topic = p('camera_info_topic', '/camera/depth/camera_info').value
        cloud_topic = p('cloud_topic', '/camera/obstacles').value
        scan_topic = p('scan_topic', '/camera/scan').value
        self.ground_frame = p('ground_frame', 'base_footprint').value
        self.scan_frame = p('scan_frame', 'base_link').value
        self.depth_scale = float(p('depth_scale', 0.001).value)
        self.stride = max(int(p('pixel_stride', 4).value), 1)
        self.min_depth = float(p('min_depth', 0.2).value)
        self.max_depth = float(p('max_depth', 4.0).value)
        self.min_h = float(p('min_obstacle_height', 0.05).value)
        self.max_h = float(p('max_obstacle_height', 1.0).value)
        self.angle_min = float(p('angle_min', -math.pi / 2).value)
        self.angle_max = float(p('angle_max', math.pi / 2).value)
        self.angle_inc = float(p('angle_increment', math.radians(0.5)).value)
        self.range_min = float(p('range_min', 0.05).value)
        self.range_max = float(p('range_max', 4.0).value)
        self.min_hit_points = int(p('min_hit_points', 3).value)
        self.free_min_points = int(p('free_min_points', 3).value)

        self.k = None
        self.tf_cache = {}      # (target, source) -> 4x4
        self.frames = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Reliable publishers: work for both best-effort (Nav2) and reliable
        # (RViz default) subscribers.
        self.cloud_pub = self.create_publisher(PointCloud2, cloud_topic, 5)
        self.scan_pub = self.create_publisher(LaserScan, scan_topic, 5)
        self.create_subscription(CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self._on_depth, qos_profile_sensor_data)
        self.get_logger().info(
            f'{depth_topic} -> {cloud_topic} + {scan_topic}: obstacles '
            f'{self.min_h:.2f}..{self.max_h:.2f} m above {self.ground_frame}, '
            f'depth {self.min_depth:.2f}..{self.max_depth:.2f} m, stride {self.stride}')

    def _on_info(self, msg):
        self.k = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _lookup(self, target, source):
        key = (target, source)
        if key not in self.tf_cache:
            tf = self.tf_buffer.lookup_transform(target, source, Time(),
                                                 timeout=Duration(seconds=0.0))
            self.tf_cache[key] = transform_matrix(tf.transform.translation,
                                                  tf.transform.rotation)
            self.get_logger().info(f'cached TF {source} -> {target}')
        return self.tf_cache[key]

    def _on_depth(self, msg):
        if self.k is None:
            self.get_logger().warn('no camera_info yet', throttle_duration_sec=5.0)
            return
        optical = msg.header.frame_id
        try:
            to_ground = self._lookup(self.ground_frame, optical)
            to_scan = self._lookup(self.scan_frame, optical)
        except TransformException as e:
            self.get_logger().warn(f'waiting for TF: {e}', throttle_duration_sec=5.0)
            return
        try:
            depth = depth_to_meters(msg.data, msg.encoding, msg.width, msg.height,
                                    msg.step, self.depth_scale)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=10.0)
            return

        pts = deproject(depth, self.k, self.stride, self.min_depth, self.max_depth)
        heights = transform_points(pts, to_ground)[:, 2]
        obstacle, floor = split_by_height(heights, self.min_h, self.max_h)
        obs_pts = pts[obstacle]

        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height = 1
        cloud.width = len(obs_pts)
        cloud.fields = _XYZ_FIELDS
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(obs_pts)
        cloud.is_dense = True
        cloud.data = obs_pts.astype(np.float32).tobytes()
        self.cloud_pub.publish(cloud)

        in_scan = transform_points(pts, to_scan)[:, :2]
        ranges = points_to_scan(in_scan[obstacle], in_scan[floor], self.angle_min,
                                self.angle_max, self.angle_inc, self.range_min,
                                self.range_max, self.min_hit_points, self.free_min_points)
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.scan_frame
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_min + (len(ranges) - 1) * self.angle_inc
        scan.angle_increment = self.angle_inc
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()
        self.scan_pub.publish(scan)

        self.frames += 1
        if self.frames == 1:
            self.get_logger().info(
                f'first frame: {len(pts)} valid points, {len(obs_pts)} obstacle points')


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacles()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A second SIGINT from ros2 launch can land mid-teardown; ignore it.
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
