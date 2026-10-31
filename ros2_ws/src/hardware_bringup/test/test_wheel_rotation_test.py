from hardware_bringup.wheel_rotation_test import encoder_count, RotationTrial
import pytest


@pytest.mark.parametrize('payload,expected', [(' 123\r\n', 123), ('-42', -42), (0, 0)])
def test_encoder_payload(payload, expected):
    assert encoder_count(payload) == expected


@pytest.mark.parametrize('payload', ['No response received', 'Parsed - Vel: 12',
                                     '', '1.5', '12 extra', 2**31, True])
def test_invalid_encoder_payload(payload):
    with pytest.raises(ValueError):
        encoder_count(payload)


def test_exact_rotation_and_no_restart():
    t = RotationTrial(100)
    t.start(123, 0)
    for i in range(1, 11):
        t.sample(123 + i * 10, i * .05)
    assert t.state == 'complete'
    assert t.net_ticks == 100
    t.sample(250, .6)
    assert t.net_ticks == 100
    with pytest.raises(ValueError):
        t.start(250, 1)


def test_crossing_threshold():
    t = RotationTrial(100)
    t.start(0, 0)
    t.sample(60, .05)
    t.sample(110, .1)
    assert t.state == 'complete'


def test_reverse_rotation_and_wrap():
    t = RotationTrial(10, direction=-1)
    t.start(-2147483645, 0)
    t.sample(2147483641, .05)
    assert t.state == 'complete'


def test_wrong_direction_and_jump():
    for count in (-1, 101):
        t = RotationTrial(100)
        t.start(0, 0)
        t.sample(count, .05)
        assert t.state == 'fault'


def test_missing_encoder():
    t = RotationTrial(100)
    t.start(0, 0)
    t.check(.31)
    assert t.state == 'fault'
    t.sample(100, .4)
    assert t.state == 'fault'


def test_stalled_wheel_even_with_fresh_messages():
    t = RotationTrial(100, max_seconds=.5)
    t.start(0, 0)
    for i in range(1, 12):
        t.sample(0, i * .05)
    assert t.reason == 'rotation time limit'


@pytest.mark.parametrize('target', [0, -1, 1.5, True])
def test_requires_measured_integer_calibration(target):
    with pytest.raises(ValueError):
        RotationTrial(target)
