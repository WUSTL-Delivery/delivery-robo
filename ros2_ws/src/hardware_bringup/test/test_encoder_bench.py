import pytest

from hardware_bringup.encoder_bench import settled, summarize, tick_delta


def test_rollover_both_directions():
    assert tick_delta(-2147483648, 2147483647) == 1
    assert tick_delta(2147483647, -2147483648) == -1


def test_forward_calibration_with_receive_stall():
    result = summarize([(0, 100), (.05, 150), (.1, 200), (.4, 300)], distance=2)
    assert result['net_ticks'] == 200
    assert result['metres_per_tick_candidate'] == .01
    assert result['gaps_over_150ms'] == 1
    assert result['max_receive_gap_ms'] == pytest.approx(300)


def test_reverse_and_backtracking_are_distinct():
    result = summarize([(0, 100), (1, 80), (2, 85), (3, 60)], turns=2)
    assert result['net_ticks'] == -40
    assert result['absolute_tick_travel'] == 50
    assert result['ticks_per_wheel_turn_candidate'] == 20


def test_stationary_no_divide_by_zero():
    assert summarize([(0, 7), (1, 7)], distance=1)['metres_per_tick_candidate'] is None


def test_settled_requires_continuous_fresh_unchanged_samples():
    samples = [(i / 20, 9) for i in range(25)]
    assert settled(samples, 1.21)
    assert not settled(samples, 2)
    assert not settled(samples[:10], .46)
    assert not settled([(0, 9), (1.2, 9)], 1.21)
    assert not settled(samples[:-1] + [(1.2, 10)], 1.21)


def test_reject_invalid_sample_times():
    with pytest.raises(ValueError):
        summarize([(0, 1), (0, 2)])
