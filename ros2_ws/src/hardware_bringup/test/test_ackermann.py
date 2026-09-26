"""Pure-math tests for the Ackermann command/odometry conversions (no ROS needed)."""
import math

import pytest

from hardware_bringup.ackermann import (
    DriveGeometry,
    DriveMap,
    ServoMap,
    check_finite,
    effective_speed,
    odom_from_ticks,
    pwm_from_speed,
    servo_degrees,
    speed_from_pwm,
    steering_from_twist,
    wrap_int32,
    yaw_rate_from_steering,
)

L = 0.36322
GEOM = DriveGeometry(wheelbase=L, min_speed_for_steer=0.05, max_steer_rad=0.4363)
SERVO = ServoMap(center_deg=45.0, deg_per_rad=math.degrees(1.0), min_deg=0.0, max_deg=180.0)
DRIVE = DriveMap(pwm_per_mps=110.0 / 1.5, pwm_limit=110)


def test_effective_speed_keeps_sign_and_floor():
    assert effective_speed(1.0, 0.05) == 1.0
    assert effective_speed(-1.0, 0.05) == -1.0
    assert effective_speed(0.01, 0.05) == 0.05
    assert effective_speed(-0.01, 0.05) == -0.05
    # exactly stopped counts as "forward" so a steering command still has a defined sign
    assert effective_speed(0.0, 0.05) == 0.05


def test_straight_line_is_zero_steer():
    assert steering_from_twist(1.0, 0.0, GEOM) == 0.0
    assert servo_degrees(0.0, SERVO) == 45.0


def test_left_turn_is_positive_steer():
    delta = steering_from_twist(1.0, 0.5, GEOM)
    assert delta == pytest.approx(math.atan(0.5 * L / 1.0))
    assert delta > 0


def test_reversing_with_left_steer_gives_negative_yaw_rate():
    omega = yaw_rate_from_steering(-1.0, 0.2, GEOM)
    assert omega < 0
    assert steering_from_twist(-1.0, omega, GEOM) == pytest.approx(0.2)


def test_standstill_steering_uses_min_speed_and_clips():
    # omega != 0 at v = 0 would be a singularity; the min-speed floor turns it into a
    # (saturated) steering command so a joystick can steer while stopped.
    delta = steering_from_twist(0.0, 0.3, GEOM)
    assert delta == pytest.approx(GEOM.max_steer_rad)
    delta_small = steering_from_twist(0.0, 0.01, GEOM)
    assert delta_small == pytest.approx(math.atan(0.01 * L / 0.05))


def test_steering_is_clipped_both_ways():
    assert steering_from_twist(0.2, 5.0, GEOM) == pytest.approx(GEOM.max_steer_rad)
    assert steering_from_twist(0.2, -5.0, GEOM) == pytest.approx(-GEOM.max_steer_rad)
    assert yaw_rate_from_steering(1.0, 2.0, GEOM) == pytest.approx(math.tan(GEOM.max_steer_rad) / L)


@pytest.mark.parametrize('v', [0.0, 0.3, 1.0, -0.4, 1.5])
@pytest.mark.parametrize('delta', [-0.4, -0.1, 0.0, 0.2, 0.4])
def test_joystick_roundtrip_recovers_steering(v, delta):
    # joy_to_cmdvel encodes (v, delta) as a Twist; arduino_bridge decodes it. They must agree.
    omega = yaw_rate_from_steering(v, delta, GEOM)
    assert steering_from_twist(v, omega, GEOM) == pytest.approx(delta, abs=1e-9)


def test_servo_map_scale_direction_and_clip():
    assert servo_degrees(math.radians(25.0), SERVO) == pytest.approx(45.0 + 25.0)
    assert servo_degrees(-math.radians(25.0), SERVO) == pytest.approx(45.0 - 25.0)
    flipped = ServoMap(center_deg=45.0, deg_per_rad=-math.degrees(1.0))
    assert servo_degrees(math.radians(25.0), flipped) == pytest.approx(20.0)
    narrow = ServoMap(center_deg=90.0, deg_per_rad=200.0, min_deg=60.0, max_deg=120.0)
    assert servo_degrees(1.0, narrow) == 120.0
    assert servo_degrees(-1.0, narrow) == 60.0


def test_pwm_map_rounds_and_clips():
    assert pwm_from_speed(1.5, DRIVE) == 110
    assert pwm_from_speed(0.0, DRIVE) == 0
    assert pwm_from_speed(5.0, DRIVE) == 110
    assert pwm_from_speed(-5.0, DRIVE) == -110
    assert pwm_from_speed(0.01, DRIVE) == 1
    assert isinstance(pwm_from_speed(0.7, DRIVE), int)
    assert speed_from_pwm(110, DRIVE) == pytest.approx(1.5)
    assert speed_from_pwm(-55, DRIVE) == pytest.approx(-0.75)


def test_check_finite_rejects_nan_and_inf():
    check_finite(0.0, 1, -2.5)
    with pytest.raises(ValueError):
        check_finite(float('nan'), 1.0)
    with pytest.raises(ValueError):
        check_finite(1.0, float('inf'))
    with pytest.raises(ValueError):
        check_finite(-float('inf'))


def test_wrap_int32_handles_counter_overflow():
    assert wrap_int32(5) == 5
    assert wrap_int32(-5) == -5
    # counter went from 2^31-3 to -2^31+2: a real increase of 5
    prev, cur = 2**31 - 3, -2**31 + 2
    assert wrap_int32(cur - prev) == 5
    assert wrap_int32(prev - cur) == -5
    assert wrap_int32(2**31) == -2**31
    assert wrap_int32(-2**31 - 1) == 2**31 - 1


def test_odom_from_ticks():
    v, omega = odom_from_ticks(100, 0.05, 0.001, 0.0, GEOM)
    assert v == pytest.approx(2.0)
    assert omega == 0.0
    v, omega = odom_from_ticks(100, 0.05, 0.001, 0.2, GEOM)
    assert omega == pytest.approx(2.0 / L * math.tan(0.2))
    # a negative metres-per-tick flips the encoder direction
    v, _ = odom_from_ticks(100, 0.05, -0.001, 0.0, GEOM)
    assert v == pytest.approx(-2.0)
    v, omega = odom_from_ticks(0, 0.05, 0.001, 0.3, GEOM)
    assert v == 0.0 and omega == 0.0
    with pytest.raises(ValueError):
        odom_from_ticks(1, 0.0, 0.001, 0.0, GEOM)
