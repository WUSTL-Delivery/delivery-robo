r"""
Sole owner of the Arduino serial port: ``/cmd_vel_out`` in, ``/odom`` out.

Why one node owns the port: ``joystick_node`` and ``wheel_encoder_node`` open the same
board and cannot share it. In the ``autonomy`` mode this node is the only process that opens
it. The port is ``/dev/motor``, the board's udev symlink
(``deployment/udev/99-motor.rules``), never a raw ``/dev/ttyUSBn``: the RPLidar C1 is one too
and plug order decides which is ``ttyUSB0``. When it is missing the node says what to run.

Data flow::

    /cmd_vel_out (Twist, from twist_mux) --> Ackermann inversion --> "m <pwm> <servo>\r"
    "e\r" polled at 20 Hz  --> AS5047P angle (deg) -> unwrap, gear ratio --> /odom (twist only)
                                                                          --> /wheel_angle (Float64, deg)
                                                                          --> /wheel_ticks (Float64, raw deg)
    the command actually sent (after clipping / watchdog)          --> /cmd_vel_executed

``/cmd_vel_executed`` is what ``state_estimation`` should use as its ``cmd_vel_topic``: it is
zero while the watchdog holds the robot stopped and reflects PWM/steer saturation, so the
EKF predicts with what the motor was really told to do.

Safety behaviour:
* commands with NaN/inf are rejected and do NOT refresh the watchdog;
* no command for ``watchdog_timeout`` seconds -> ``m 0 <centre>`` is sent at ``command_hz``
  until commands resume;
* serial errors close the port and the node keeps retrying to reopen it; nothing is sent to
  the Arduino during its ~2 s auto-reset after the port opens;
* on shutdown a final stop command is written.
"""
from collections import deque
import math
import os
import re
import threading
import time

from geometry_msgs.msg import Twist
from hardware_bringup.ackermann import (
    check_finite,
    DriveGeometry,
    DriveMap,
    odom_from_ticks,
    pwm_from_speed,
    servo_degrees,
    ServoMap,
    speed_from_pwm,
    steering_from_twist,
)
from hardware_bringup.serial_protocol import (
    drive_command,
    ENCODER_QUERY,
    is_error_response,
    LineSplitter,
)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import serial
from std_msgs.msg import Float64

DEFAULT_PORT = '/dev/motor'
MOTOR_RULES = '/etc/udev/rules.d/99-motor.rules'
UDEV_INSTALL = 'sudo deployment/udev/install_udev.sh'


def missing_port_hint(port, rules=MOTOR_RULES):
    """Say why ``port`` is missing and what to do; never suggests a raw /dev/ttyUSB*."""
    if port != DEFAULT_PORT:
        return f'{port} does not exist (check the `port` parameter / port:= argument)'
    if not os.path.exists(rules):
        return (f'{port} does not exist because its udev rule is not installed ({rules} '
                f'missing). Run `{UDEV_INSTALL}` in the delivery-robo checkout, see '
                f'deployment/README.md. Not falling back to /dev/ttyUSB*: that can be the lidar')
    return f'{port} does not exist: is the motor-controller board plugged in?'


_FLOAT_LINE = re.compile(rb'^-?\d+(?:\.\d+)?$')


def parse_encoder_angle(line):
    """
    Return the AS5047P angle in degrees if ``line`` is a bare number, else ``None``.

    The firmware's ``e`` reply used to be a cumulative quadrature count (bare int); it is now
    ``Serial.println(getEncoder())`` on an AS5047P absolute-angle sensor (bare float, degrees,
    bounded to one turn, 2-decimal precision from ``println``'s default). Accepting int- or
    float-shaped lines keeps this tolerant of either firmware generation. ``arduino_bridge``
    never sends the battery query ('b'), so a bare number on this stream is unambiguously the
    encoder reply, not readVoltage()'s bare float.
    """
    return float(line) if _FLOAT_LINE.match(line) else None


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        p = self._declare
        # --- serial ---
        self._port = p('port', DEFAULT_PORT)
        self._baud = int(p('baud_rate', 57600))
        self._reset_s = float(p('arduino_reset_s', 2.0))
        # --- topics / frames ---
        cmd_vel_topic = p('cmd_vel_topic', '/cmd_vel_out')
        executed_topic = p('executed_topic', '/cmd_vel_executed')
        odom_topic = p('odom_topic', '/odom')
        ticks_topic = p('ticks_topic', '/wheel_ticks')
        wheel_angle_topic = p('wheel_angle_topic', '/wheel_angle')
        self._odom_frame = p('odom_frame', 'odom')
        self._base_frame = p('base_frame', 'base_link')
        # --- rates / timeouts ---
        command_hz = float(p('command_hz', 20.0))
        poll_hz = float(p('encoder_poll_hz', 20.0))
        self._watchdog_s = float(p('watchdog_timeout', 0.5))
        self._min_enc_dt = float(p('min_encoder_dt', 0.02))
        self._max_enc_dt = float(p('max_encoder_dt', 0.5))
        # --- calibration (see README "Calibration") ---
        self._geom = DriveGeometry(
            wheelbase=float(p('wheelbase', 0.36322)),
            min_speed_for_steer=float(p('min_speed_for_steer', 0.05)),
            max_steer_rad=float(p('max_steer_rad', 0.4363)),
        )
        self._servo = ServoMap(
            center_deg=float(p('servo_center_deg', 45.0)),
            deg_per_rad=float(p('servo_deg_per_rad', math.degrees(1.0))),
            min_deg=float(p('servo_min_deg', 0.0)),
            max_deg=float(p('servo_max_deg', 180.0)),
        )
        self._drive = DriveMap(
            pwm_per_mps=float(p('pwm_per_mps', 110.0 / 1.5)),
            pwm_limit=int(p('pwm_limit', 110)),
        )
        self._gear_ratio = float(p('gear_ratio', 10.0 / 12.0))         # motor-gear teeth / wheel-gear teeth
        self._wheel_radius_m = float(p('wheel_radius_m', 0.001))       # PLACEHOLDER: measure and set, m
        self._encoder_direction = float(p('encoder_direction', 1.0))   # +1/-1: flip if angle grows reversed
        self._validate_params()

        # --- state ---
        self._ser = None
        self._ser_lock = threading.Lock()
        self._tx_enabled_at = float('inf')      # monotonic time after which writes are allowed
        self._splitter = LineSplitter()
        self._samples = deque()                 # (ros_time_ns, angle_deg) filled by the reader thread
        self._prev_sample = None
        self._ready = False
        self._error_replies = 0
        self._last_cmd = None                   # (v, omega)
        self._last_cmd_time = None              # monotonic
        self._watchdog_tripped = True           # start in the safe state until a command arrives
        self._delta_cmd = 0.0                   # steering angle currently commanded (for /odom yaw rate)
        self._wheel_angle_deg = 0.0             # cumulative gear-corrected wheel-shaft angle (deg)

        # --- ROS interfaces ---
        self._executed_pub = self.create_publisher(Twist, executed_topic, 10)
        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self._ticks_pub = self.create_publisher(Float64, ticks_topic, 10)
        self._wheel_angle_pub = self.create_publisher(Float64, wheel_angle_topic, 10)
        self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_timer(1.0 / command_hz, self._command_tick)
        self.create_timer(1.0 / poll_hz, self._poll_tick)
        self.create_timer(1.0, self._reconnect_tick)

        self._stop_event = threading.Event()
        self._reader = threading.Thread(target=self._reader_loop, name='arduino-reader', daemon=True)
        self._reader.start()
        self._open_serial()

        self.get_logger().info(
            f'arduino_bridge: {self._port}@{self._baud}, {cmd_vel_topic} -> serial, '
            f'serial -> {odom_topic}; watchdog {self._watchdog_s:.2f} s; '
            f'L={self._geom.wheelbase} m, servo centre {self._servo.center_deg} deg, '
            f'{self._drive.pwm_per_mps:.1f} PWM/(m/s) limit {self._drive.pwm_limit}, '
            f'{self._wheel_radius_m} m wheel radius, gear ratio {self._gear_ratio:.4f}'
        )

    # ------------------------------------------------------------------ params
    def _declare(self, name, default):
        return self.declare_parameter(name, default).value

    def _validate_params(self):
        if not 0 < self._drive.pwm_limit <= 255:
            raise ValueError('pwm_limit must be in (0, 255]')
        if self._drive.pwm_per_mps <= 0:
            raise ValueError('pwm_per_mps must be positive')
        if self._geom.wheelbase <= 0:
            raise ValueError('wheelbase must be positive')
        if self._wheel_radius_m <= 0:
            raise ValueError('wheel_radius_m must be positive: measure it and set the parameter')
        if self._gear_ratio <= 0:
            raise ValueError('gear_ratio must be positive (motor-gear teeth / wheel-gear teeth)')
        if self._encoder_direction not in (1.0, -1.0):
            raise ValueError('encoder_direction must be 1 or -1')
        if not 0 < self._geom.max_steer_rad < math.pi / 2:
            raise ValueError('max_steer_rad must be in (0, pi/2)')
        if self._watchdog_s <= 0:
            raise ValueError('watchdog_timeout must be positive')
        if self._servo.deg_per_rad == 0.0:
            raise ValueError('servo_deg_per_rad must be non-zero (negative flips direction)')

    # ------------------------------------------------------------------ serial
    def _open_serial(self):
        if self._ser is not None:
            return True
        if not os.path.exists(self._port):
            self.get_logger().error(
                f'{missing_port_hint(self._port)}; retrying every second',
                throttle_duration_sec=10.0,
            )
            return False
        try:
            ser = serial.Serial(self._port, self._baud, timeout=0.05, write_timeout=0.2)
            ser.reset_input_buffer()
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warning(
                f'cannot open {self._port}: {exc}; retrying every second', throttle_duration_sec=10.0
            )
            return False
        self._splitter = LineSplitter()
        self._prev_sample = None
        self._ready = False
        # Opening the port toggles DTR and resets the Arduino; the bootloader owns the line for
        # ~2 s. Nothing is written until then (test_serial.py sleeps for the same reason).
        self._tx_enabled_at = time.monotonic() + self._reset_s
        self._ser = ser
        self.get_logger().info(f'opened {self._port}; waiting {self._reset_s:.1f} s for the Arduino reset')
        return True

    def _close_serial(self, reason):
        ser, self._ser = self._ser, None
        self._tx_enabled_at = float('inf')
        self._ready = False
        if ser is not None:
            self.get_logger().error(f'serial link lost ({reason}); will retry')
            try:
                ser.close()
            except Exception:  # noqa: BLE001 - closing a broken port
                pass

    def _write(self, data):
        ser = self._ser
        if ser is None or time.monotonic() < self._tx_enabled_at:
            return False
        try:
            with self._ser_lock:
                ser.write(data)
            return True
        except (serial.SerialException, OSError) as exc:
            self._close_serial(f'write failed: {exc}')
            return False

    def _reconnect_tick(self):
        if self._ser is None:
            self._open_serial()
        elif not self._ready and time.monotonic() > self._tx_enabled_at + 5.0:
            self.get_logger().warning(
                'no encoder reply from the Arduino yet (is the motor-controller sketch flashed, '
                'is this the right port?)', throttle_duration_sec=10.0
            )

    def _reader_loop(self):
        """Background thread: turns serial bytes into encoder samples."""
        while not self._stop_event.is_set():
            ser = self._ser
            if ser is None:
                time.sleep(0.1)
                continue
            try:
                data = ser.read(1)
                if data and ser.in_waiting:
                    data += ser.read(ser.in_waiting)
            except (serial.SerialException, OSError, TypeError) as exc:
                # TypeError: pyserial raises it when the port is closed under us
                if self._ser is ser:
                    self._close_serial(f'read failed: {exc}')
                time.sleep(0.1)
                continue
            if not data:
                continue
            for line in self._splitter.feed(data):
                angle_deg = parse_encoder_angle(line)
                if angle_deg is not None:
                    self._samples.append((self.get_clock().now().nanoseconds, angle_deg))
                    if not self._ready:
                        self._ready = True
                        self.get_logger().info(f'Arduino ready (encoder angle {angle_deg:.2f} deg)')
                elif is_error_response(line):
                    self._error_replies += 1
                    self.get_logger().warning(
                        f'Arduino rejected a command (X), {self._error_replies} so far',
                        throttle_duration_sec=5.0,
                    )

    # ------------------------------------------------------------------ commands
    def _on_cmd_vel(self, msg):
        v, omega = msg.linear.x, msg.angular.z
        try:
            check_finite(v, omega)
        except ValueError:
            # Not refreshing the watchdog: a source that only sends NaN gets the robot stopped.
            self.get_logger().error(
                f'rejecting non-finite command v={v} omega={omega}', throttle_duration_sec=1.0
            )
            return
        self._last_cmd = (v, omega)
        self._last_cmd_time = time.monotonic()

    def _command_tick(self):
        now = time.monotonic()
        stale = self._last_cmd_time is None or now - self._last_cmd_time > self._watchdog_s
        if stale != self._watchdog_tripped:
            self._watchdog_tripped = stale
            if stale:
                self.get_logger().warning(
                    f'no command for {self._watchdog_s:.2f} s: stopping (m 0 {self._servo.center_deg:.0f})'
                )
            else:
                self.get_logger().info('commands resumed')

        if stale:
            delta, pwm = 0.0, 0
        else:
            v, omega = self._last_cmd
            delta = steering_from_twist(v, omega, self._geom)
            pwm = pwm_from_speed(v, self._drive)
        servo = int(round(servo_degrees(delta, self._servo)))
        # The steering angle the servo really gets, after clipping and integer rounding; this is
        # what /odom's yaw rate and /cmd_vel_executed should be based on.
        if self._servo.deg_per_rad != 0.0:
            delta = (servo - self._servo.center_deg) / self._servo.deg_per_rad
        self._delta_cmd = delta

        sent = self._write(drive_command(pwm, servo))

        executed = Twist()
        if sent:
            v_exec = speed_from_pwm(pwm, self._drive)
            executed.linear.x = float(v_exec)
            executed.angular.z = float(v_exec / self._geom.wheelbase * math.tan(delta))
        self._executed_pub.publish(executed)

    # ------------------------------------------------------------------ odometry
    def _poll_tick(self):
        self._write(ENCODER_QUERY)
        while self._samples:
            t_ns, angle_deg = self._samples.popleft()
            self._ticks_pub.publish(Float64(data=float(angle_deg)))
            if self._prev_sample is None:
                self._prev_sample = (t_ns, angle_deg)
                continue
            prev_t, prev_angle_deg = self._prev_sample
            dt = (t_ns - prev_t) * 1e-9
            if dt < self._min_enc_dt:
                continue                        # replies bunched after a firmware stall: merge with the next
            if dt > self._max_enc_dt:
                self._prev_sample = (t_ns, angle_deg)  # gap too long to differentiate across
                continue
            # Shortest signed step between two bounded (0-360 deg) absolute readings; only wrong
            # if the motor shaft turns more than half a revolution between polls (encoder_poll_hz
            # too low for the motor's top speed).
            delta_motor_deg = angle_deg - prev_angle_deg
            delta_motor_deg -= 360.0 * round(delta_motor_deg / 360.0)
            delta_wheel_rad = math.radians(delta_motor_deg) * self._gear_ratio * self._encoder_direction
            self._wheel_angle_deg += math.degrees(delta_wheel_rad)
            self._wheel_angle_pub.publish(Float64(data=self._wheel_angle_deg))
            v, omega = odom_from_ticks(delta_wheel_rad, dt, self._wheel_radius_m, self._delta_cmd,
                                       self._geom)
            self._prev_sample = (t_ns, angle_deg)

            odom = Odometry()
            odom.header.stamp = Time(nanoseconds=t_ns).to_msg()
            odom.header.frame_id = self._odom_frame
            odom.child_frame_id = self._base_frame
            odom.twist.twist.linear.x = float(v)
            odom.twist.twist.angular.z = float(omega)
            self._odom_pub.publish(odom)

    # ------------------------------------------------------------------ shutdown
    def shutdown_hardware(self):
        self._stop_event.set()
        ser = self._ser
        if ser is not None:
            try:
                with self._ser_lock:
                    ser.write(drive_command(0, self._servo.center_deg))
                    ser.flush()
            except Exception:  # noqa: BLE001 - best effort on the way out
                pass
            finally:
                self._ser = None
                try:
                    ser.close()
                except Exception:  # noqa: BLE001
                    pass


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_hardware()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
