#!/usr/bin/env python3
"""Lidar watchdog: restarts a hung sllidar_node so a re-plugged RPLidar recovers.

Why: upstream sllidar_node (Slamtec/sllidar_ros2) exits with -1 when the serial
port is missing at startup, so launch's respawn=True keeps retrying until the
lidar is plugged in. But if the lidar is unplugged *mid-run*, its scan loop
(`while (rclcpp::ok() && !need_exit) grabScanDataHq(...)`) just keeps failing
on read errors/timeouts and never exits, so respawn never fires and a re-plug
is never picked up.

This node watches `scan_topic`. When a running `process_name` process has not
produced a scan for `stale_timeout_s` (measured from the later of its last scan
or when the watchdog first saw that PID, so a fresh process gets a startup
grace), it sends SIGINT (the driver's own clean-exit path: stops the motor and
returns), then SIGKILL after `kill_grace_s` if it is still alive. The launch
file's respawn then restarts the driver, which reopens /dev/rplidar.

Caveat: calling the driver's `stop_motor` service also stops /scan, and the
watchdog will then restart the driver (which spins the motor back up).

    ros2 run my_bringup lidar_watchdog --ros-args -p stale_timeout_s:=5.0
"""
import os
import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def _find_pids(name):
    """PIDs (owned by us) whose argv[0] basename is exactly `name`."""
    pids = []
    uid = os.getuid()
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            if os.stat('/proc/' + entry).st_uid != uid:
                continue
            with open('/proc/%s/cmdline' % entry, 'rb') as f:
                argv0 = f.read().split(b'\0', 1)[0].decode(errors='replace')
        except OSError:
            continue
        if os.path.basename(argv0) == name:
            pids.append(int(entry))
    return pids


class LidarWatchdog(Node):

    def __init__(self):
        super().__init__('lidar_watchdog')
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.process_name = self.declare_parameter('process_name', 'sllidar_node').value
        self.stale_timeout = float(self.declare_parameter('stale_timeout_s', 5.0).value)
        self.kill_grace = float(self.declare_parameter('kill_grace_s', 5.0).value)
        period = float(self.declare_parameter('check_period_s', 1.0).value)

        self.last_scan = 0.0
        self.first_seen = {}   # pid -> monotonic time the watchdog first saw it
        self.signaled = {}     # pid -> monotonic time SIGINT was sent

        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data)
        self.create_timer(period, self._check)
        self.get_logger().info(
            f'watching {self.scan_topic}; restarting {self.process_name} '
            f'after {self.stale_timeout:.1f}s without scans')

    def _on_scan(self, _msg):
        self.last_scan = time.monotonic()

    def _check(self):
        now = time.monotonic()
        pids = _find_pids(self.process_name)
        # forget processes that have exited
        self.first_seen = {p: t for p, t in self.first_seen.items() if p in pids}
        self.signaled = {p: t for p, t in self.signaled.items() if p in pids}

        for pid in pids:
            self.first_seen.setdefault(pid, now)
            if pid in self.signaled:
                if now - self.signaled[pid] > self.kill_grace:
                    self.get_logger().warn(f'{self.process_name} [{pid}] ignored SIGINT; sending SIGKILL')
                    self._send(pid, signal.SIGKILL)
                    self.signaled[pid] = now
                continue
            last_activity = max(self.last_scan, self.first_seen[pid])
            if now - last_activity > self.stale_timeout:
                self.get_logger().warn(
                    f'no {self.scan_topic} for {now - last_activity:.1f}s '
                    f'(lidar unplugged?); restarting {self.process_name} [{pid}]')
                self._send(pid, signal.SIGINT)
                self.signaled[pid] = now

    def _send(self, pid, sig):
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass
        except PermissionError as e:
            self.get_logger().error(f'cannot signal {pid}: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LidarWatchdog()
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
