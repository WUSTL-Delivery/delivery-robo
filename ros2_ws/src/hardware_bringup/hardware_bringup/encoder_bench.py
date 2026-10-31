"""Read-only wheel-count trials; no serial access and no command publishers."""
import argparse
import csv
import json
import math
from pathlib import Path
import select
import sys
import time


def tick_delta(current, previous):
    """Signed 32-bit Arduino counter difference, including rollover."""
    return (current - previous + 2**31) % 2**32 - 2**31


def summarize(samples, distance=None, turns=None):
    """Summarize the samples, given as (host monotonic seconds, cumulative count) pairs."""
    if len(samples) < 2:
        raise ValueError('At least two samples are required')
    gaps = [b[0] - a[0] for a, b in zip(samples, samples[1:])]
    if min(gaps) <= 0:
        raise ValueError('Sample times must increase')
    deltas = [tick_delta(b[1], a[1]) for a, b in zip(samples, samples[1:])]
    net = sum(deltas)
    duration = samples[-1][0] - samples[0][0]
    ordered = sorted(gaps)
    result = {
        'samples': len(samples), 'duration_s': duration,
        'net_ticks': net, 'absolute_tick_travel': sum(abs(d) for d in deltas),
        'positive_ticks': sum(d for d in deltas if d > 0),
        'negative_ticks': sum(d for d in deltas if d < 0),
        'mean_receive_hz': (len(samples) - 1) / duration,
        'p95_receive_gap_ms': ordered[math.ceil(.95 * len(ordered)) - 1] * 1000,
        'max_receive_gap_ms': max(gaps) * 1000,
        'gaps_over_150ms': sum(g > .15 for g in gaps),
        'max_abs_tick_step': max(abs(d) for d in deltas),
        'metres_per_tick_candidate': distance / net if distance and net else None,
        'ticks_per_wheel_turn_candidate': abs(net) / turns if turns and net else None,
        'timing_note': 'Host receipt gaps, NOT sensor latency. Counter resets invalidate a trial.',
    }
    return result


def settled(samples, now, window=1.0, freshness=.25):
    """Require fresh, continuous reports with unchanged count for a full second."""
    if not samples or now - samples[-1][0] > freshness:
        return False
    end = samples[-1][0]
    i = len(samples) - 1
    while i > 0 and samples[i][0] > end - window:
        i -= 1
    tail = samples[i:]
    return (end - tail[0][0] >= window
            and all(tick == tail[-1][1] for _, tick in tail)
            and all(b[0] - a[0] <= freshness for a, b in zip(tail, tail[1:])))


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('must be finite and positive')
    return number


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/wheel_ticks')
    parser.add_argument('--output', required=True, help='New directory for CSV and summary.json')
    measure = parser.add_mutually_exclusive_group()
    measure.add_argument('--distance', type=positive, help='Measured FORWARD travel in metres')
    measure.add_argument('--turns', type=positive, help='Measured wheel rotations in ONE direction')
    opts = parser.parse_args(args)
    if not sys.stdin.isatty():
        parser.error('Run interactively in a terminal (start / finish / quit commands)')
    # Imports are lazy so the analysis can be tested without ROS installed.
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Int64

    output = Path(opts.output)
    output.mkdir(parents=True, exist_ok=False)
    rclpy.init(args=[])
    node = rclpy.create_node('encoder_bench')
    samples = []
    trial_start = None
    with (output / 'samples.csv').open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['host_monotonic_s', 'ticks'])

        def receive(msg):
            sample = (time.monotonic(), msg.data)
            samples.append(sample)
            writer.writerow(sample)
            handle.flush()

        node.create_subscription(Int64, opts.topic, receive, qos_profile_sensor_data)
        print('Read-only recorder: no motor commands. Waiting for ' + opts.topic, flush=True)
        print('Hold wheel still for >1 s, type start then Enter. Move the measured amount,')
        print('hold still for >1 s, then type finish. Type quit to abort.', flush=True)
        next_status = 0.0
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.05)
                now = time.monotonic()
                if now >= next_status:
                    count = samples[-1][1] if samples else 'NO DATA'
                    print(f'count={count}  settled={settled(samples, now)}', flush=True)
                    next_status = now + 2
                if not select.select([sys.stdin], [], [], 0)[0]:
                    continue
                line = sys.stdin.readline()
                if not line:
                    break
                command = line.strip().lower()
                if command == 'quit':
                    break
                if command not in ('start', 'finish'):
                    print('Use start, finish, or quit.', flush=True)
                    continue
                if not settled(samples, now):
                    print('Not settled or no fresh data: hold still and retry.', flush=True)
                    continue
                if command == 'start':
                    if trial_start is not None:
                        print('Already started. Use finish or quit.', flush=True)
                        continue
                    trial_start = len(samples) - 1
                    print('START recorded. Move the measured amount now.', flush=True)
                elif trial_start is None:
                    print('Type start first.', flush=True)
                elif len(samples) - trial_start < 2:
                    print('Wait for more samples.', flush=True)
                else:
                    result = summarize(samples[trial_start:], opts.distance, opts.turns)
                    result.update(topic=opts.topic, measured_distance_m=opts.distance,
                                  measured_turns=opts.turns,
                                  start_csv_sample_index=trial_start,
                                  end_csv_sample_index=len(samples)-1)
                    (output / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
                    print(json.dumps(result, indent=2), flush=True)
                    print('Saved to ' + str(output), flush=True)
                    break
        except KeyboardInterrupt:
            print('Aborted; raw CSV retained, no completed trial summary.')
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
