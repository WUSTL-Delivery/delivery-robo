"""Ackermann conversions shared by ``joy_to_cmdvel`` (encode) and ``arduino_bridge`` (decode).

The autonomy stack speaks ``geometry_msgs/Twist`` as (forward speed, yaw rate). The Arduino
speaks ``m <pwm> <servo_deg>``. Everything that converts between the two lives here so the
two nodes cannot drift apart, and so it can be unit-tested without ROS.

Conventions
-----------
* ``delta`` is the road-wheel steering angle in radians, positive = left (CCW yaw).
* ``omega = v / L * tan(delta)``, the kinematic bicycle model the autonomy stack uses too.
* At standstill a yaw rate has no steering solution, so the inversion uses a speed floor
  (``min_speed_for_steer``) instead of forcing ``delta = 0``. That lets a joystick steer
  while stopped; ``joy_to_cmdvel`` applies the same floor when encoding, so the round trip
  is exact (see ``test_joystick_roundtrip_recovers_steering``).
"""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class DriveGeometry:
    wheelbase: float = 0.36322          # m, rear axle to front axle (URDF value; tape-measure it)
    min_speed_for_steer: float = 0.05   # m/s floor used when converting yaw rate <-> steering
    max_steer_rad: float = 0.4363       # rad road-wheel limit (25 deg = joystick_node's range)


@dataclass(frozen=True)
class ServoMap:
    center_deg: float = 45.0                  # servo command that points the wheels straight
    deg_per_rad: float = 57.29577951308232    # servo degrees per radian of road-wheel angle (negative flips)
    min_deg: float = 0.0
    max_deg: float = 180.0


@dataclass(frozen=True)
class DriveMap:
    pwm_per_mps: float = 110.0 / 1.5    # open-loop: PWM counts per m/s (calibrate, see README)
    pwm_limit: int = 110                # matches joystick_node's MAX_PWM


def check_finite(*values):
    """Raise ``ValueError`` if any value is NaN or infinite.

    ``min``/``max``/``np.clip`` in the controller let NaN straight through, so a diverged MPPI
    solve arrives here verbatim. Reject it before it reaches the motor.
    """
    for value in values:
        if not math.isfinite(value):
            raise ValueError(f'non-finite command value {value!r}')


def _clip(x, lo, hi):
    return max(lo, min(hi, x))


def effective_speed(v, min_speed):
    """Speed with a magnitude floor; exactly zero is treated as forward."""
    if abs(v) >= min_speed:
        return v
    return -min_speed if v < 0 else min_speed


def steering_from_twist(v, omega, geom: DriveGeometry):
    """Road-wheel angle that produces yaw rate ``omega`` at speed ``v`` (clipped to the limit)."""
    v_eff = effective_speed(v, geom.min_speed_for_steer)
    delta = math.atan(omega * geom.wheelbase / v_eff)
    return _clip(delta, -geom.max_steer_rad, geom.max_steer_rad)


def yaw_rate_from_steering(v, delta, geom: DriveGeometry):
    """Inverse of :func:`steering_from_twist`; used by the joystick encoder."""
    delta = _clip(delta, -geom.max_steer_rad, geom.max_steer_rad)
    v_eff = effective_speed(v, geom.min_speed_for_steer)
    return v_eff / geom.wheelbase * math.tan(delta)


def servo_degrees(delta, servo: ServoMap):
    return _clip(servo.center_deg + delta * servo.deg_per_rad, servo.min_deg, servo.max_deg)


def pwm_from_speed(v, drive: DriveMap):
    return int(_clip(round(v * drive.pwm_per_mps), -drive.pwm_limit, drive.pwm_limit))


def speed_from_pwm(pwm, drive: DriveMap):
    return pwm / drive.pwm_per_mps


def wrap_int32(x):
    """Wrap into the int32 range; the firmware's ``long`` tick counter is 32-bit."""
    return (int(x) + 2**31) % 2**32 - 2**31


def odom_from_ticks(delta_ticks, dt, metres_per_tick, delta_steer, geom: DriveGeometry):
    """Body speed and yaw rate from an encoder tick delta and the steering angle in force.

    There is one drive encoder and no steering feedback, so the yaw rate is synthesised from
    the commanded steering angle (plan §7 risk 3). ``metres_per_tick`` may be negative to
    flip the encoder's counting direction.
    """
    if dt <= 0:
        raise ValueError(f'dt must be positive, got {dt}')
    v = delta_ticks * metres_per_tick / dt
    omega = v / geom.wheelbase * math.tan(delta_steer)
    return v, omega
