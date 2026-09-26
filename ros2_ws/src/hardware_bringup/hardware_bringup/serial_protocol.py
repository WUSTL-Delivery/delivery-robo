"""Line protocol of ``firmware/motor-controller`` (57600 baud, ``\\r``-terminated).

Commands the bridge uses::

    m <pwm> <servo_deg>\\r   set drive PWM (-255..255, sign = direction) and servo angle (0..180)
    e\\r                     reply: the cumulative encoder count as a bare integer line

The firmware also prints debug chatter (``ddd Parsed - Vel: ...`` before every command and
``Parsed - Vel: ...`` after every ``m``) and ``X\\r`` on a parse error, so replies are found by
shape (an integer on a line of its own), never by position.
"""
import re

ENCODER_QUERY = b'e\r'
ERROR_RESPONSE = b'X'

_INT_LINE = re.compile(rb'^-?\d+$')
_LINE_END = re.compile(rb'\r\n|\r|\n')


def drive_command(pwm, servo_deg):
    """``m <pwm> <servo>`` with integer arguments; the firmware casts both to ``int`` anyway."""
    return f'm {int(round(pwm))} {int(round(servo_deg))}\r'.encode('ascii')


def parse_encoder_count(line):
    """Return the encoder count if ``line`` is a bare integer, else ``None``."""
    if _INT_LINE.match(line):
        return int(line)
    return None


def is_error_response(line):
    return line == ERROR_RESPONSE


class LineSplitter:
    """Incremental splitter tolerant of ``\\r\\n``, lone ``\\r`` (the error reply) and lone ``\\n``."""

    def __init__(self, max_buffer=4096):
        self._buf = bytearray()
        self.max_buffer = max_buffer

    def feed(self, data):
        """Append ``data``; return the complete, non-empty lines it produced (terminators stripped)."""
        self._buf.extend(data)
        parts = _LINE_END.split(bytes(self._buf))
        self._buf = bytearray(parts[-1])          # trailing partial line, possibly empty
        if len(self._buf) > self.max_buffer:      # garbage without a terminator: drop it
            self._buf.clear()
        return [p for p in parts[:-1] if p]
