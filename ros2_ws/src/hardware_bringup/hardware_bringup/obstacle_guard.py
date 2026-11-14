"""
Obstacle-sensor guard: no lidar and no depth camera means no autonomous driving (TDM-52).

The user's rule: with one of lidar / depth camera missing the robot keeps driving on the other;
with both missing it stops driving autonomously, because they are what it avoids obstacles
with, and the joystick still overrides. This node enforces the second half on the real robot,
whatever sends the goals (Nav2 + local_planner, or a route sent straight to /follow_path with
nav2:=false) and also in local_planner's final approach, where it no longer replans.

    lidar_topic  (/scan_filtered) ─┐
                                   ├─► obstacle_guard ─ std_msgs/Bool at rate_hz ─► lock_topic
    camera_topic (/camera/scan) ───┘                                 (/obstacle_sensors_lock)

twist_mux (config/twist_mux.yaml, lock ``obstacle_sensors``, priority 50) masks every input with
a priority below the lock's while it is locked: navigation (cmd_vel_nav, 10) is blocked, the
joystick (cmd_vel_joy, 100) still passes. The scans are subscribed raw (never deserialised):
only their arrival times matter. A sensor is live while its newest message is at most
``timeout`` s old (receive time, monotonic clock).

* Locked (True) while neither sensor is live, including at startup until the first scan.
* Unlocked (False) while at least one is live. Each change is logged once, naming the sensors
  that are silent and for how long.
* Fail-safe: twist_mux treats a lock whose publisher went quiet for its ``timeout`` (0.5 s) as
  locked (apt twist_mux 4.5.0 ``LockTopicHandle::isLocked`` = expired or true; its stamp starts
  at 0, so it is also locked before this node first publishes). A dead guard blocks navigation.
* Stopping: twist_mux never publishes a zero for a masked input, it just stops forwarding it,
  so arduino_bridge's watchdog (``watchdog_timeout`` 0.5 s) sends "m 0 <centre>". Both checked
  against the apt twist_mux with fake publishers (TDM-52).
* ``enabled: false`` (bench work without sensors): publishes False (unlocked) and logs an error
  at startup and every 30 s. twist_mux would otherwise block navigation, as for a dead guard.

    ros2 run hardware_bringup obstacle_guard --ros-args -p enabled:=false
"""
import time

from hardware_bringup.sensor_watch import ObstacleGuardLogic, SensorWatch
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ObstacleGuard(Node):
    """Publishes the twist_mux lock that blocks navigation when both scans are silent."""

    def __init__(self):
        super().__init__('obstacle_guard')
        lidar_topic = self.declare_parameter('lidar_topic', '/scan_filtered').value
        camera_topic = self.declare_parameter('camera_topic', '/camera/scan').value
        lock_topic = self.declare_parameter('lock_topic', '/obstacle_sensors_lock').value
        timeout = float(self.declare_parameter('timeout', 1.0).value)
        rate_hz = float(self.declare_parameter('rate_hz', 10.0).value)
        enabled = bool(self.declare_parameter('enabled', True).value)
        if rate_hz <= 0.0:
            raise ValueError('rate_hz must be positive')

        lidar, camera = f'lidar {lidar_topic}', f'camera {camera_topic}'
        self._watch = SensorWatch([lidar, camera], timeout)
        self._logic = ObstacleGuardLogic(self._watch, enabled)
        self._lock_pub = self.create_publisher(Bool, lock_topic, 1)
        for name, topic in ((lidar, lidar_topic), (camera, camera_topic)):
            self.create_subscription(
                LaserScan, topic, lambda _msg, name=name: self._watch.seen(name, time.monotonic()),
                qos_profile_sensor_data, raw=True)
        self.create_timer(1.0 / rate_hz, self._tick)
        if enabled:
            self.get_logger().info(
                f'obstacle_guard: {lidar} + {camera}; both silent > {timeout} s -> {lock_topic} '
                'true (twist_mux: navigation blocked, joystick passes); locked until a first scan')
        else:
            self._disabled_warning()
            self.create_timer(30.0, self._disabled_warning)

    def _disabled_warning(self):
        self.get_logger().error(
            'obstacle_guard DISABLED (enabled:=false): autonomous driving is NOT stopped when '
            'both the lidar and the depth camera are missing. Bench work only')

    def _tick(self):
        locked, log = self._logic.update(time.monotonic())
        self._lock_pub.publish(Bool(data=locked))
        if log is None:
            return
        # One call site per level: rclpy refuses a call site whose severity changes.
        level, text = log
        if level == 'error':
            self.get_logger().error(text)
        elif level == 'warn':
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
