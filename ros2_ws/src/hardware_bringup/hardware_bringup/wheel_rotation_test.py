"""
Publish ONE small Joy command, stop at a measured encoder rotation threshold.

Use wheel_rotation_test.launch.py: its isolated mapper interprets all-zero Joy as
stop. This is NOT compatible with autonomy-mode enable-button release semantics.
"""
from collections import deque
import json
import math
import re
import time

from hardware_bringup.encoder_bench import settled, tick_delta


def encoder_count(data):
    """Accept bare integer String payloads or Int64, never serial debug text."""
    if isinstance(data, str):
        data = data.strip()
        if not re.fullmatch(r'-?[0-9]+', data):
            raise ValueError('encoder payload is not an integer count')
        data = int(data)
    if type(data) is not int or not -(2**31) <= data < 2**31:
        raise ValueError('encoder count is outside signed Arduino int32 range')
    return data


class RotationTrial:
    """Pure state machine. Times are monotonic seconds; counts are signed int32."""

    def __init__(self, ticks_per_rotation, direction=1, max_seconds=5.0, stale_seconds=.3):
        if type(ticks_per_rotation) is not int or ticks_per_rotation <= 0:
            raise ValueError('ticks_per_rotation must be a measured positive integer')
        if direction not in (-1, 1):
            raise ValueError('encoder_direction must be -1 or 1')
        if not all(math.isfinite(x) and x > 0 for x in (max_seconds, stale_seconds)):
            raise ValueError('timeouts must be positive and finite')
        self.target = ticks_per_rotation
        self.direction = direction
        self.max_seconds = max_seconds
        self.stale_seconds = stale_seconds
        self.state = 'waiting'
        self.reason = ''
        self.net_ticks = 0
        self.previous = None
        self.started = None
        self.last_sample = None

    def start(self, count, now):
        if self.state != 'waiting':
            raise ValueError('Trial can only start once')
        self.previous = count
        self.started = self.last_sample = now
        self.state = 'running'

    def stop(self, reason):
        if self.state in ('waiting', 'running'):
            self.state, self.reason = 'fault', reason

    def check(self, now):
        if self.state == 'running':
            if now - self.last_sample >= self.stale_seconds:
                self.stop('encoder data timeout')
            elif now - self.started >= self.max_seconds:
                self.stop('rotation time limit')

    def sample(self, count, now):
        if self.state != 'running':
            return
        self.check(now)
        if self.state != 'running':
            return
        delta = tick_delta(count, self.previous)
        self.previous, self.last_sample = count, now
        if abs(delta) > self.target:
            self.stop('implausible count jump; possible reset')
            return
        self.net_ticks += delta
        if delta * self.direction < 0:
            self.stop('encoder moved opposite expected direction')
        elif self.net_ticks * self.direction >= self.target:
            self.state, self.reason = 'complete', 'rotation threshold reached; stop requested'


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Int64, String

    class WheelRotationTest(Node):
        def __init__(self):
            super().__init__('wheel_rotation_test')

            def p(name, default):
                return self.declare_parameter(name, default).value

            target = p('ticks_per_rotation', 0)  # Never guess a physical calibration.
            direction = p('encoder_direction', 1)
            self.throttle = float(p('throttle', .1))
            if not math.isfinite(self.throttle) or not 0 < abs(self.throttle) <= .25:
                raise ValueError('throttle magnitude must be in (0, 0.25]')
            self.trial = RotationTrial(target, direction, float(p('max_run_seconds', 5.0)),
                                       float(p('encoder_timeout', .3)))
            self.wait_seconds = float(p('startup_timeout', 15.0))
            if not math.isfinite(self.wait_seconds) or self.wait_seconds <= 0:
                raise ValueError('startup_timeout must be positive and finite')
            self.created = time.monotonic()
            self.history = deque(maxlen=200)
            encoder_topic = p('encoder_topic', '/wheel_ticks')
            encoder_type = p('encoder_message_type', 'int64')
            if encoder_type not in ('int64', 'string'):
                raise ValueError('encoder_message_type must be int64 or string')
            self.joy_pub = self.create_publisher(Joy, '/encoder_test/joy', 10)
            self.status_pub = self.create_publisher(String, '/encoder_test/rotation_status',
                                                    QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                                               durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(Int64 if encoder_type == 'int64' else String,
                                     encoder_topic, self.on_count, qos_profile_sensor_data)
            self.get_logger().info(f'Encoder input: {encoder_topic} ({encoder_type})')
            self.create_timer(.05, self.tick)
            self.last_status = None
            self.get_logger().info('Waiting for fresh stationary encoder data and Joy subscriber. '
                                   'This node automatically starts ONE trial when ready.')

        def send(self, throttle):
            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.axes = [0.0, float(throttle), 0.0, 0.0, 0.0, 0.0]
            msg.buttons = [0] * 12
            self.joy_pub.publish(msg)

        def on_count(self, msg):
            now = time.monotonic()
            try:
                count = encoder_count(msg.data)
            except ValueError:
                self.history.clear()
                if self.trial.state == 'running':
                    self.trial.stop('invalid encoder payload')
                    self.send(0.0)
                self.get_logger().warning('Invalid encoder payload; not a count',
                                          throttle_duration_sec=2.0)
                self.status()
                return
            self.history.append((now, count))
            was_running = self.trial.state == 'running'
            self.trial.sample(count, now)
            if was_running and self.trial.state in ('complete', 'fault'):
                self.send(0.0)  # Request stop directly in the encoder callback.
            self.status()

        def status(self):
            state = (self.trial.state, self.trial.reason)
            if state != self.last_status:
                data = {'state': self.trial.state, 'reason': self.trial.reason,
                        'net_ticks': self.trial.net_ticks, 'target_ticks': self.trial.target}
                self.status_pub.publish(String(data=json.dumps(data)))
                self.get_logger().info(json.dumps(data))
                self.last_status = state

        def tick(self):
            now = time.monotonic()
            if self.trial.state == 'waiting':
                if now - self.created >= self.wait_seconds:
                    self.trial.stop('startup timeout: no settled encoder or Joy subscriber')
                elif self.joy_pub.get_subscription_count() > 0 and settled(list(self.history), now):
                    self.trial.start(self.history[-1][1], now)
                    self.send(self.throttle)  # The ONLY nonzero message in the trial.
            else:
                self.trial.check(now)
            if self.trial.state != 'running':
                self.send(0.0)  # Repeated zeros hold the stop; no automatic restart.
            self.status()

    rclpy.init(args=args)
    node = None
    try:
        node = WheelRotationTest()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.send(0.0)
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
