"""Time one ``mppi_step`` on this machine (plan §7 risk 1: MPPI on a Pi CPU).

    ros2 run hardware_bringup bench_mppi [iterations]

Needs the workspace sourced so ``autonomy`` is importable. No ROS node is started. The
controller runs at 10 Hz, so a step must take well under 100 ms including the reference
lookup; if it does not, ``K`` in ``autonomy/utils/mppi_step.py`` (tangled from the Org
source) has to drop, which is the one unavoidable edit to delivery-autonomy.
"""
import statistics
import sys
import time


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    iterations = int(argv[0]) if argv else 30

    import jax
    import jax.numpy as jnp
    import numpy as np
    from autonomy.utils.dynamics import INPUT_DIMENSION, STATE_DIMENSION
    from autonomy.utils.mppi_step import mppi_step

    horizon, dt, v_target = 50, 0.1, 1.5
    x0 = np.zeros(STATE_DIMENSION)
    u_nominal = jnp.zeros((horizon, INPUT_DIMENSION))
    s = np.arange(horizon) * v_target * dt
    ref = jnp.array(np.stack([s, np.zeros(horizon), np.zeros(horizon)], axis=-1))
    key = jax.random.PRNGKey(0)

    print(f'jax {jax.__version__} backend {jax.default_backend()} devices {jax.devices()}')
    t0 = time.perf_counter()
    mppi_step(key=key, x0=x0, U_nominal=u_nominal, ref_traj=ref, v_target=v_target).block_until_ready()
    print(f'first call (JIT compile): {time.perf_counter() - t0:.1f} s')

    times_ms = []
    for _ in range(iterations):
        key, sub = jax.random.split(key)
        t0 = time.perf_counter()
        u = mppi_step(key=sub, x0=x0, U_nominal=u_nominal, ref_traj=ref, v_target=v_target)
        u.block_until_ready()
        times_ms.append((time.perf_counter() - t0) * 1e3)
        u_nominal = u

    mean, median, worst = statistics.mean(times_ms), statistics.median(times_ms), max(times_ms)
    print(f'{iterations} steps: mean {mean:.1f} ms, median {median:.1f} ms, max {worst:.1f} ms '
          f'(budget: 100 ms per control tick at 10 Hz)')
    if worst > 80.0:
        print('VERDICT: too slow for a reliable 10 Hz loop on this CPU; reduce K in mppi_step.py '
              '(tangled: edit docs/20260109142710-delivery.org and re-tangle)')
        return 1
    print('VERDICT: fits the 10 Hz budget')
    return 0


if __name__ == '__main__':
    sys.exit(main())
