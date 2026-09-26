#!/usr/bin/env python3
"""Heartbeat node: periodically POSTs the robot's status to the robo-web API.

Every `interval_s` it sends JSON to `{api_url}/api/heartbeat`. Only the stdlib
is used (urllib), and network failures are logged (throttled) but never
crash the node, so a dead hotspot can't take the robot down with it.

    ros2 run my_bringup heartbeat_node --ros-args -p api_url:=https://x.vercel.app -p robot_id:=robo-1
"""
import json
import os
import socket
import subprocess
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node

try:  # best-effort teleop activity; the node works without these msg types
    from sensor_msgs.msg import Joy
except ImportError:  # pragma: no cover
    Joy = None
try:
    from geometry_msgs.msg import Twist
except ImportError:  # pragma: no cover
    Twist = None

THERMAL = '/sys/class/thermal/thermal_zone0/temp'
# With --symlink-install this file lives inside the repo, so git finds the
# root from here; REPO env (set by startup.sh users) overrides.
REPO = os.environ.get('REPO') or os.path.dirname(os.path.realpath(__file__))


def _uptime_s():
    try:
        with open('/proc/uptime') as f:
            return float(f.read().split()[0])
    except Exception:
        return None


def _cpu_temp_c():
    try:
        with open(THERMAL) as f:
            return round(int(f.read().strip()) / 1000.0, 1)
    except Exception:
        return None


def _ips():
    try:
        out = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=2).stdout
        ips = [ip for ip in out.split() if not ip.startswith('127.')]
        if ips:
            return ips
    except Exception:
        pass
    try:
        return [socket.gethostbyname(socket.gethostname())]
    except Exception:
        return []


def _git_sha():
    try:
        return subprocess.run(
            ['git', '-C', REPO, 'rev-parse', '--short', 'HEAD'],
            capture_output=True, text=True, timeout=2,
        ).stdout.strip() or None
    except Exception:
        return None


class Heartbeat(Node):
    def __init__(self):
        super().__init__('heartbeat')
        self.declare_parameter('api_url', 'https://robo-web-ebon.vercel.app')
        self.declare_parameter('robot_id', 'robo-1')
        self.declare_parameter('mode', 'teleop')
        self.declare_parameter('interval_s', 5.0)

        self.api_url = self.get_parameter('api_url').value.rstrip('/')
        self.robot_id = self.get_parameter('robot_id').value
        self.mode = self.get_parameter('mode').value
        self.interval = float(self.get_parameter('interval_s').value)

        self.hostname = socket.gethostname()
        self.git_sha = _git_sha()  # constant for the process lifetime
        self.start = time.monotonic()
        self.ticks = 0
        self.fail_count = 0
        self.last_warn = 0.0

        self.last_joy_t = None
        self.last_cmd_vel = None
        if Joy is not None:
            self.create_subscription(Joy, '/joy', self._on_joy, 10)
        if Twist is not None:
            self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)

        self.get_logger().info(f'heartbeat {self.robot_id} -> {self.api_url}/api/heartbeat every {self.interval}s')
        self.timer = self.create_timer(self.interval, self.tick)
        self.tick()  # announce immediately, don't wait one interval

    def _on_joy(self, _msg):
        self.last_joy_t = time.monotonic()

    def _on_cmd_vel(self, msg):
        self.last_cmd_vel = {
            'linear_x': round(msg.linear.x, 3),
            'angular_z': round(msg.angular.z, 3),
        }

    def _state(self, other_nodes):
        up = time.monotonic() - self.start
        if up >= self.interval and other_nodes:
            return self.mode
        return 'booting'

    def payload(self):
        self_name = self.get_name()
        nodes = sorted({n for n in self.get_node_names() if n != self_name})
        topics = sorted(t for t, _ in self.get_topic_names_and_types())[:50]
        try:
            load = list(os.getloadavg())
        except OSError:
            load = None
        return {
            'id': self.robot_id,
            'state': self._state(nodes),
            'mode': self.mode,
            'stage': 'ros',
            'hostname': self.hostname,
            'ips': _ips(),
            'uptime_s': _uptime_s(),
            'git_sha': self.git_sha,
            'ros_distro': os.environ.get('ROS_DISTRO'),
            'ros_nodes': nodes,
            'ros_topics': topics,
            'cpu_temp_c': _cpu_temp_c(),
            'load_avg': load,
            'last_joy_age_s': (round(time.monotonic() - self.last_joy_t, 1)
                               if self.last_joy_t is not None else None),
            'last_cmd_vel': self.last_cmd_vel,
            'heartbeat_seq': self.ticks,
            'ts': time.time(),
        }

    def tick(self):
        # Never let a reporting bug take the node down (respawn would just loop).
        try:
            self._tick()
        except Exception as e:  # noqa: BLE001
            now = time.monotonic()
            if now - self.last_warn > 30:
                self.get_logger().warning(f'heartbeat tick error: {e!r}')
                self.last_warn = now

    def _tick(self):
        self.ticks += 1
        body = json.dumps(self.payload()).encode()
        req = urllib.request.Request(
            f'{self.api_url}/api/heartbeat', data=body, method='POST',
            headers={'Content-Type': 'application/json', 'User-Agent': 'delivery-robo-heartbeat'},
        )
        try:
            with urllib.request.urlopen(req, timeout=5) as resp:
                resp.read()
            if self.fail_count:
                self.get_logger().info(f'heartbeat back online after {self.fail_count} failures')
            self.fail_count = 0
        except (urllib.error.URLError, urllib.error.HTTPError, OSError, ValueError) as e:
            self.fail_count += 1
            now = time.monotonic()
            if now - self.last_warn > 30:  # throttle: hotspot outages are noisy
                self.get_logger().warning(f'heartbeat POST failed ({self.fail_count}x): {e}')
                self.last_warn = now


def main(args=None):
    rclpy.init(args=args)
    node = Heartbeat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001 - shutdown races are cosmetic
            pass


if __name__ == '__main__':
    main()
