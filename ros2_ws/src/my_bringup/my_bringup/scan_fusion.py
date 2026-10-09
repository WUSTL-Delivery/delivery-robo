"""Fuse /scan_filtered and /camera/scan into /scan_fused for obstacle marking.

The camera scan is already height-filtered and projected by depth_obstacles.
Use TF from robot_description; do not apply the camera mount a second time.
Finite hit endpoints are transformed into output_frame and rebinned. NaN and
infinite input ranges do not produce clearing rays. Configure costmap consumers
with marking=true and clearing=false; keep the physical observations for clearing.

With fixed_frame (typically odom), TF compensates the camera endpoints from their
capture time to the lidar scan's capture time. Without it, observations are paired
by timestamp and only mount transforms are applied. This mode is for bench checks.
Lidar beams are treated at the header stamp; individual beams are not deskewed.
"""

import math
import time
from collections import deque

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from my_bringup.depth_obstacles import transform_matrix
from my_bringup.scan_fusion_core import (
    ScanGrid,
    ScanSample,
    closest_sample,
    fuse_hits,
    is_fresh,
)


class ScanFusion(Node):
    """Lidar-driven fusion with bounded pairing latency and explicit health."""

    def __init__(self, **kwargs):
        super().__init__('scan_fusion', **kwargs)

        def param(name, default):
            return self.declare_parameter(
                name, default, ParameterDescriptor(read_only=True)).value

        lidar_topic = param('lidar_topic', '/scan_filtered')
        camera_topic = param('camera_topic', '/camera/scan')
        output_topic = param('output_topic', '/scan_fused')
        self.output_frame = param('output_frame', 'base_link')
        self.fixed_frame = param('fixed_frame', '')
        self.grid = ScanGrid(param('bins', 360), param('range_min', 0.05),
                             param('range_max', 12.0))
        tolerance = param('sync_tolerance_s', 0.05)
        max_age = param('max_age_s', 0.3)
        self.pair_wait = param('pair_wait_s', 0.05)
        self.tf_wait = param('tf_wait_s', 0.1)
        queue_size = param('queue_size', 10)
        timings = (tolerance, max_age, self.pair_wait, self.tf_wait)
        if (not all(math.isfinite(x) and x >= 0 for x in timings) or max_age <= 0
                or self.pair_wait > self.tf_wait or self.tf_wait >= max_age):
            raise ValueError('require nonnegative timings and pair_wait <= tf_wait < max_age')
        if queue_size < 1 or not self.output_frame:
            raise ValueError('queue_size must be positive and output_frame nonempty')
        if self.resolve_topic_name(output_topic) in (
                self.resolve_topic_name(lidar_topic), self.resolve_topic_name(camera_topic)):
            raise ValueError('output_topic must differ from input topics')
        self.tolerance_ns = round(tolerance * 1e9)
        self.max_age_ns = round(max_age * 1e9)
        self.lidar_queue = deque(maxlen=queue_size)
        self.camera_queue = deque(maxlen=queue_size)
        self.last_lidar_ns = None
        self.last_camera_ns = None
        self.last_output_ns = None
        self.last_clock_ns = None
        self.last_output_wall = None
        self.state = 'waiting for lidar'
        self.error = ''
        self.camera_used = False
        self.camera_delta_s = None
        self.queue_drops = 0
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(LaserScan, output_topic, 5)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 5)
        self.create_subscription(LaserScan, lidar_topic, self._on_lidar, qos_profile_sensor_data)
        self.create_subscription(LaserScan, camera_topic, self._on_camera, qos_profile_sensor_data)
        self.create_timer(0.01, self._process)
        self.create_timer(1.0, self._diagnose)
        self.get_logger().info(
            f'{lidar_topic} + {camera_topic} -> {output_topic} in {self.output_frame}; '
            'virtual obstacle endpoints: use marking=true, clearing=false')
        if not self.fixed_frame:
            self.get_logger().warn(
                'fixed_frame is empty: no motion compensation; set fixed_frame:=odom '
                'once odom TF is available before using fusion while driving')

    @staticmethod
    def _sample(msg):
        return ScanSample(
            msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
            msg.header.frame_id, msg.angle_min, msg.angle_increment,
            msg.range_min, msg.range_max, np.asarray(msg.ranges, dtype=np.float64))

    def _receive(self, msg):
        try:
            return self._sample(msg)
        except ValueError as error:
            self.get_logger().warn(f'invalid scan: {error}', throttle_duration_sec=5.0)
            return None

    def _on_lidar(self, msg):
        sample = self._receive(msg)
        if sample is not None:
            self.last_lidar_ns = sample.stamp_ns
            if len(self.lidar_queue) == self.lidar_queue.maxlen:
                self.queue_drops += 1
            self.lidar_queue.append((sample, time.monotonic(), msg.scan_time))

    def _on_camera(self, msg):
        sample = self._receive(msg)
        if sample is not None:
            self.last_camera_ns = sample.stamp_ns
            self.camera_queue.append(sample)

    def _matrix(self, sample, target_ns):
        source_time = Time(nanoseconds=sample.stamp_ns)
        if self.fixed_frame:
            tf = self.tf_buffer.lookup_transform_full(
                self.output_frame, Time(nanoseconds=target_ns), sample.frame_id,
                source_time, self.fixed_frame)
        elif sample.frame_id == self.output_frame:
            return np.eye(4)
        else:
            tf = self.tf_buffer.lookup_transform(
                self.output_frame, sample.frame_id, source_time)
        return transform_matrix(tf.transform.translation, tf.transform.rotation)

    def _process(self):
        now_ns = self.get_clock().now().nanoseconds
        if self.last_clock_ns is not None and now_ns < self.last_clock_ns:
            self.lidar_queue.clear()
            self.camera_queue.clear()
            self.last_lidar_ns = self.last_camera_ns = self.last_output_ns = None
            self.last_output_wall = None
            self.camera_used = False
            self.state = 'clock reset; waiting for new observations'
        self.last_clock_ns = now_ns

        while self.lidar_queue:
            lidar, received, scan_time = self.lidar_queue[0]
            elapsed = time.monotonic() - received
            if elapsed < self.pair_wait:
                return
            already_published = (self.last_output_ns is not None
                                 and lidar.stamp_ns <= self.last_output_ns)
            if (not is_fresh(lidar.stamp_ns, now_ns, self.max_age_ns, self.tolerance_ns)
                    or already_published):
                self.lidar_queue.popleft()
                self.state = 'discarded stale, future, or out-of-order lidar scan'
                continue
            camera = closest_sample(self.camera_queue, lidar.stamp_ns, now_ns,
                                    self.max_age_ns, self.tolerance_ns)
            self.error = ''
            try:
                lidar_matrix = self._matrix(lidar, lidar.stamp_ns)
            except TransformException as error:
                self.error = f'lidar TF unavailable: {error}'
                self.state = 'waiting for lidar TF'
                if elapsed < self.tf_wait:
                    return
                self.lidar_queue.popleft()
                continue
            camera_matrix = None
            if camera is not None:
                try:
                    camera_matrix = self._matrix(camera, lidar.stamp_ns)
                except TransformException as error:
                    self.error = f'camera TF unavailable: {error}'
                    self.state = 'waiting for camera TF'
                    if elapsed < self.tf_wait:
                        return
                    camera = None
            ranges = fuse_hits(lidar, lidar_matrix, camera, camera_matrix, self.grid)
            result = LaserScan()
            result.header.stamp = Time(nanoseconds=lidar.stamp_ns).to_msg()
            result.header.frame_id = self.output_frame
            result.angle_min = self.grid.angle_min
            result.angle_max = self.grid.angle_max
            result.angle_increment = self.grid.angle_increment
            # This is a virtual snapshot, not a sequentially acquired scan.
            result.time_increment = 0.0
            result.scan_time = scan_time if math.isfinite(scan_time) and scan_time >= 0 else 0.0
            result.range_min = self.grid.range_min
            result.range_max = self.grid.range_max
            result.ranges = ranges.tolist()
            self.publisher.publish(result)
            self.lidar_queue.popleft()
            self.last_output_ns = lidar.stamp_ns
            self.last_output_wall = time.monotonic()
            self.camera_used = camera is not None
            self.camera_delta_s = ((camera.stamp_ns - lidar.stamp_ns) / 1e9
                                   if camera is not None else None)
            self.state = 'fused' if self.camera_used else 'lidar only: no usable camera match'

    def _diagnose(self):
        now = self.get_clock().now()
        now_ns = now.nanoseconds

        def fresh(stamp):
            return stamp is not None and is_fresh(
                stamp, now_ns, self.max_age_ns, self.tolerance_ns)

        output_fresh = (fresh(self.last_output_ns) and self.last_output_wall is not None
                        and time.monotonic() - self.last_output_wall <= self.max_age_ns / 1e9)
        status = DiagnosticStatus()
        status.name = self.get_fully_qualified_name() + '/fusion'
        status.hardware_id = 'lidar_depth_fusion'
        status.level = (DiagnosticStatus.ERROR if not output_fresh else
                        DiagnosticStatus.WARN if not self.camera_used or not self.fixed_frame else
                        DiagnosticStatus.OK)
        status.message = self.state if output_fresh else 'no fresh fused output; ' + self.state
        if not self.fixed_frame:
            status.message += '; motion compensation disabled'
        values = {
            'lidar_fresh': fresh(self.last_lidar_ns),
            'camera_fresh': fresh(self.last_camera_ns),
            'camera_used': self.camera_used and output_fresh,
            'camera_delta_s': self.camera_delta_s,
            'fixed_frame': self.fixed_frame,
            'output_frame': self.output_frame,
            'lidar_queue_drops': self.queue_drops,
            'transform_error': self.error,
        }
        status.values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
        result = DiagnosticArray()
        result.header.stamp = now.to_msg()
        result.status = [status]
        self.diagnostics.publish(result)


def main(args=None):
    """Run the fusion node."""
    rclpy.init(args=args)
    node = None
    try:
        node = ScanFusion()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
