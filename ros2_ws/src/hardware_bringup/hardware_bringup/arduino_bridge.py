"""Sole owner of the Arduino serial port: ``/cmd_vel_out`` in, ``/odom`` out.

Why one node owns the port: ``joystick_node``, ``wheel_encoder_node`` and the ``sim/``
``ackdrive_arduino`` HAL all default to ``/dev/ttyUSB0`` and cannot coexist. In the
``autonomy`` mode this node is the only process that opens it.

Data flow::

    /cmd_vel_out (Twist, from twist_mux) --> Ackermann inversion --> "m <pwm> <servo>\\r"
    "e\\r" polled at 20 Hz                 --> tick delta / dt      --> /odom (twist only)
                                                                    --> /wheel_ticks (Int64)
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
import math
import threading
import time
from collections import deque

import rclpy
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Int64

from hardware_bringup.ackermann import (
    DriveGeometry,
    DriveMap,
    ServoMap,
    check_finite,
    odom_from_ticks,
    pwm_from_speed,
    servo_degrees,
    speed_from_pwm,
    steering_from_twist,
    wrap_int32,
)
from hardware_bringup.serial_protocol import (
    ENCODER_QUERY,
    LineSplitter,
    drive_command,
    is_error_response,
    parse_encoder_count,
)


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        p = self._declare
        # --- serial ---
        self._port = p('port', '/dev/ttyUSB0')
        self._baud = int(p('baud_rate', 57600))
        self._reset_s = float(p('arduino_reset_s', 2.0))
        # --- topics / frames ---
        cmd_vel_topic = p('cmd_vel_topic', '/cmd_vel_out')
        executed_topic = p('executed_topic', '/cmd_vel_executed')
        odom_topic = p('odom_topic', '/odom')
        ticks_topic = p('ticks_topic', '/wheel_ticks')
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
        self._metres_per_tick = float(p('metres_per_tick', 0.001))
        self._validate_params()

        # --- state ---
        self._ser = None
        self._ser_lock = threading.Lock()
        self._tx_enabled_at = float('inf')      # monotonic time after which writes are allowed
        self._splitter = LineSplitter()
        self._samples = deque()                 # (ros_time_ns, ticks) filled by the reader thread
        self._prev_sample = None
        self._ready = False
        self._error_replies = 0
        self._last_cmd = None                   # (v, omega)
        self._last_cmd_time = None              # monotonic
        self._watchdog_tripped = True           # start in the safe state until a command arrives
        self._delta_cmd = 0.0                   # steering angle currently commanded (for /odom yaw rate)

        # --- ROS interfaces ---
        self._executed_pub = self.create_publisher(Twist, executed_topic, 10)
        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self._ticks_pub = self.create_publisher(Int64, ticks_topic, 10)
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
            f'{self._metres_per_tick} m/tick'
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
        if self._metres_per_tick == 0:
            raise ValueError('metres_per_tick must be non-zero (negative flips direction)')
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
                count = parse_encoder_count(line)
                if count is not None:
                    self._samples.append((self.get_clock().now().nanoseconds, count))
                    if not self._ready:
                        self._ready = True
                        self.get_logger().info(f'Arduino ready (encoder count {count})')
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
            t_ns, ticks = self._samples.popleft()
            self._ticks_pub.publish(Int64(data=int(ticks)))
            if self._prev_sample is None:
                self._prev_sample = (t_ns, ticks)
                continue
            prev_t, prev_ticks = self._prev_sample
            dt = (t_ns - prev_t) * 1e-9
            if dt < self._min_enc_dt:
                continue                        # replies bunched after a firmware stall: merge with the next
            if dt > self._max_enc_dt:
                self._prev_sample = (t_ns, ticks)  # gap too long to differentiate across
                continue
            delta_ticks = wrap_int32(ticks - prev_ticks)
            v, omega = odom_from_ticks(delta_ticks, dt, self._metres_per_tick, self._delta_cmd, self._geom)
            self._prev_sample = (t_ns, ticks)

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
