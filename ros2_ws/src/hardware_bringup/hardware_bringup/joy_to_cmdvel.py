"""Joystick -> ``geometry_msgs/Twist`` on ``/cmd_vel_joy`` for ``twist_mux``.

Replaces ``joystick_node`` in the ``autonomy`` mode. Same stick mapping as that node
(throttle on ``axes[1]``, steer on ``axes[0]``) but the output is a Twist so that
``twist_mux`` can arbitrate it against the autonomy controller:

* joystick priority 100 beats the controller's 10, so moving the stick with the enable
  button held takes over immediately (the plan's override requirement);
* releasing the enable button stops publishing, and after ``twist_mux``'s 0.5 s timeout the
  controller's commands flow again;
* holding the enable button with the stick centred publishes zeros: an e-stop.

Steering is encoded as a yaw rate via the same ``min_speed_for_steer`` convention that
``arduino_bridge`` uses to decode it, so the servo follows the stick directly (also while
stopped) exactly as it did with ``joystick_node``. The three geometry parameters must match
the bridge's; ``config/hardware.yaml`` sets them once for both nodes.

If no fresh ``/joy`` message has arrived for ``joy_timeout`` seconds (joystick unplugged,
``joy_node`` dead) and ``require_joystick`` is true, zeros are published at top priority so
the robot stops and stays stopped: without a joystick nobody can override the autonomy.
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy

from hardware_bringup.ackermann import DriveGeometry, yaw_rate_from_steering


def _clamp_unit(x):
    return max(-1.0, min(1.0, x))


class JoyToCmdVel(Node):
    def __init__(self):
        super().__init__('joy_to_cmdvel')
        p = self._declare
        joy_topic = p('joy_topic', '/joy')
        output_topic = p('output_topic', '/cmd_vel_joy')
        self._axis_throttle = int(p('axis_throttle', 1))
        self._axis_steer = int(p('axis_steer', 0))
        self._invert_throttle = bool(p('invert_throttle', False))
        self._invert_steer = bool(p('invert_steer', False))
        self._max_speed = float(p('max_speed', 1.5))
        self._geom = DriveGeometry(
            wheelbase=float(p('wheelbase', 0.36322)),
            min_speed_for_steer=float(p('min_speed_for_steer', 0.05)),
            max_steer_rad=float(p('max_steer_rad', 0.4363)),
        )
        self._enable_button = int(p('enable_button', 4))
        self._require_enable = bool(p('require_enable_button', True))
        self._joy_timeout = float(p('joy_timeout', 1.0))
        self._require_joystick = bool(p('require_joystick', True))
        publish_rate = float(p('publish_rate', 20.0))

        self._last_joy = None
        self._last_joy_time = None
        self._override_active = False

        self._pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Joy, joy_topic, self._on_joy, 10)
        self.create_timer(1.0 / publish_rate, self._tick)

        self.get_logger().info(
            f'joy_to_cmdvel: {joy_topic} -> {output_topic}; throttle axis {self._axis_throttle}, '
            f'steer axis {self._axis_steer}, max {self._max_speed} m/s, '
            f'enable button {self._enable_button} ({"required" if self._require_enable else "ignored"}), '
            f'require_joystick={self._require_joystick}'
        )

    def _declare(self, name, default):
        return self.declare_parameter(name, default).value

    def _on_joy(self, msg):
        self._last_joy = msg
        self._last_joy_time = time.monotonic()

    def _set_override(self, active):
        if active != self._override_active:
            self._override_active = active
            self.get_logger().info('manual override ACTIVE' if active else 'manual override released')

    def _tick(self):
        now = time.monotonic()
        if self._last_joy is None or now - self._last_joy_time > self._joy_timeout:
            if self._require_joystick:
                self.get_logger().warning(
                    'no joystick input: publishing zero at top priority (robot held stopped; '
                    'set require_joystick:=false to allow autonomy without a joystick)',
                    throttle_duration_sec=5.0,
                )
                self._pub.publish(Twist())
                self._set_override(True)
            else:
                self._set_override(False)
            return

        msg = self._last_joy
        if self._require_enable:
            pressed = len(msg.buttons) > self._enable_button and msg.buttons[self._enable_button]
            if not pressed:
                self._set_override(False)
                return
        if len(msg.axes) <= max(self._axis_throttle, self._axis_steer):
            self.get_logger().warning(
                f'/joy has only {len(msg.axes)} axes; need indices {self._axis_throttle} and {self._axis_steer}',
                throttle_duration_sec=5.0,
            )
            return

        throttle = _clamp_unit(msg.axes[self._axis_throttle]) * (-1.0 if self._invert_throttle else 1.0)
        steer = _clamp_unit(msg.axes[self._axis_steer]) * (-1.0 if self._invert_steer else 1.0)
        v = throttle * self._max_speed
        delta = steer * self._geom.max_steer_rad

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(yaw_rate_from_steering(v, delta, self._geom))
        self._pub.publish(cmd)
        self._set_override(True)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
