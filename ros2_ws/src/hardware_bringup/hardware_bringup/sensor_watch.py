"""
Which obstacle sensors are publishing, and the obstacle guard's decision (TDM-52), without ROS.

The user's rule for a missing obstacle sensor: with one of lidar / depth camera missing the robot
keeps the same flow (plan, drive, stop at the goal) on the sensor that is left; with both missing
it stops driving autonomously, and the joystick still overrides.

``SensorWatch`` keeps the receive time of each input's newest message. Times are plain seconds
from one clock (the nodes pass ``time.monotonic()``, which an NTP step on the Pi cannot move), so
everything here is testable without ROS. ``ObstacleGuardLogic`` turns a watch into the lock
decision of ``obstacle_guard`` and one log line per change of the live inputs.
"""
import math


class SensorWatch:
    """The receive time of each named input's newest message; live = younger than ``timeout``."""

    def __init__(self, names, timeout):
        if not (math.isfinite(timeout) and timeout > 0.0):
            raise ValueError('timeout must be positive')
        self.timeout = float(timeout)
        self._last = dict.fromkeys(names)

    @property
    def names(self):
        """Return the inputs, in the order given."""
        return tuple(self._last)

    def seen(self, name, now):
        """Record a message from ``name`` received at ``now``."""
        self._last[name] = now

    def ever_seen(self):
        """Whether any input has published at all."""
        return any(t is not None for t in self._last.values())

    def live(self, now):
        """Return the inputs with a message at most ``timeout`` old, in order."""
        return tuple(n for n, t in self._last.items() if t is not None and now - t <= self.timeout)

    def silent(self, now):
        """Return the inputs that are not live, in order."""
        live = self.live(now)
        return tuple(n for n in self._last if n not in live)

    def describe(self, name, now):
        """Describe one input, e.g. ``/scan silent 2.3 s`` or ``/camera/scan never received``."""
        last = self._last[name]
        if last is None:
            return f'{name} never received'
        return f'{name} silent {now - last:.1f} s'

    def silent_summary(self, now):
        """Describe every silent input, comma separated; '' when all are live."""
        return ', '.join(self.describe(n, now) for n in self.silent(now))


class ObstacleGuardLogic:
    """
    The obstacle guard's lock: engaged while no input is live, unless disabled.

    ``update(now)`` returns ``(locked, log)``. ``log`` is ``(level, text)`` once per change of the
    live inputs (level 'info', 'warn' or 'error') and None otherwise. Before any input has been
    seen the lock is engaged: a guard that starts before its sensors blocks driving until one
    of them publishes.
    """

    def __init__(self, watch, enabled=True):
        self.watch = watch
        self.enabled = enabled
        self._state = None   # (locked, live inputs) at the last update

    def update(self, now):
        """Return ``(locked, log)`` for time ``now``."""
        live = self.watch.live(now)
        locked = self.enabled and not live
        state = (locked, live)
        if state == self._state:
            return locked, None
        was_locked = self._state is not None and self._state[0]
        self._state = state
        if not self.enabled:
            return locked, None
        names = self.watch.names
        if locked and not self.watch.ever_seen():
            return locked, ('warn', f'no scan yet from {" or ".join(names)}: autonomous driving '
                                    'blocked until one arrives')
        if locked:
            return locked, ('error', 'BOTH obstacle sensors missing ('
                            + self.watch.silent_summary(now) + '): autonomous driving BLOCKED '
                            '(navigation masked in twist_mux); the joystick still drives')
        released = '; lock released, autonomous driving allowed' if was_locked else ''
        silent = self.watch.silent(now)
        if silent:
            return locked, ('warn', f'obstacle sensor missing ({self.watch.silent_summary(now)}): '
                                    f'driving on {" and ".join(live)} only{released}')
        return locked, ('info', f'obstacle sensors live: {" and ".join(live)}{released}')
