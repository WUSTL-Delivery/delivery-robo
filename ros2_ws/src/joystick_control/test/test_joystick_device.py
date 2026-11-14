"""
TDM-18: joystick_node opens the motor board as /dev/motor, never a raw /dev/ttyUSBn.

/dev/motor is the board's udev symlink (deployment/udev/99-motor.rules). The RPLidar C1 is a
/dev/ttyUSBn too, so a /dev/ttyUSB0 default could write motor commands into the lidar. When
/dev/motor is missing the node says what to run and keeps retrying; it never opens anything
else. No serial port is opened here: serial.Serial and the existence check are faked. Needs
rclpy and pyserial (python3-serial, on the Pi); skipped without them.
"""
import types

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('serial')

from joystick_control import joystick_node  # noqa: E402, I100
from sensor_msgs.msg import Joy  # noqa: E402


class FakeSerial:
    opened = []

    def __init__(self, device, baud, timeout=None):
        FakeSerial.opened.append((device, baud))
        self.is_open = True
        self.written = []
        self.fail_writes = False

    def write(self, data):
        if self.fail_writes:
            raise OSError(5, 'Input/output error')
        self.written.append(data)

    def close(self):
        self.is_open = False


@pytest.fixture
def present(monkeypatch):
    """Fake serial.Serial and the existence check; returns the set of paths that exist."""
    paths = set()
    FakeSerial.opened = []
    monkeypatch.setattr(joystick_node.serial, 'Serial', FakeSerial)
    fake_os = types.SimpleNamespace(path=types.SimpleNamespace(exists=paths.__contains__))
    monkeypatch.setattr(joystick_node, 'os', fake_os)
    return paths


@pytest.fixture
def make_node():
    """Start rclpy with the given `-p name:=value` overrides and build a JoystickNode."""
    nodes = []

    def make(*overrides):
        args = ['--ros-args'] + [a for o in overrides for a in ('-p', o)] if overrides else None
        rclpy.init(args=args)
        nodes.append(joystick_node.JoystickNode())
        return nodes[-1]

    yield make
    for node in nodes:
        node.destroy_node()
    if nodes:
        rclpy.shutdown()


def test_default_device_is_the_motor_symlink():
    assert joystick_node.DEFAULT_DEVICE == '/dev/motor'


def test_node_opens_dev_motor_by_default(present, make_node):
    present.add('/dev/motor')
    node = make_node()
    assert node.device == '/dev/motor'
    assert FakeSerial.opened == [('/dev/motor', 57600)]


def test_missing_dev_motor_opens_nothing_then_recovers(present, make_node):
    node = make_node()
    assert node.ser is None
    assert FakeSerial.opened == []          # no fallback to some /dev/ttyUSBn
    node.open_serial()                      # what the 1 s retry timer calls
    assert FakeSerial.opened == []
    present.add('/dev/motor')               # board plugged in / rules installed
    node.open_serial()
    assert FakeSerial.opened == [('/dev/motor', 57600)]
    assert node.ser is not None


def test_write_failure_closes_the_port_for_the_retry(present, make_node):
    present.add('/dev/motor')
    node = make_node()
    node.stored_msg = Joy(axes=[0.0, 0.5, 0.0], buttons=[])
    node.timer_callback()
    assert node.ser.written == [b'm 55.00 45.00\r']
    node.ser.fail_writes = True
    node.timer_callback()
    assert node.ser is None
    node.open_serial()
    assert FakeSerial.opened == [('/dev/motor', 57600)] * 2


def test_device_parameter_still_overrides(present, make_node):
    present.add('/dev/serial/by-id/usb-test')
    make_node('device:=/dev/serial/by-id/usb-test')
    assert FakeSerial.opened == [('/dev/serial/by-id/usb-test', 57600)]


def test_hint_without_rules_says_what_to_run(tmp_path):
    hint = joystick_node.missing_port_hint('/dev/motor', rules=str(tmp_path / 'missing.rules'))
    assert 'sudo deployment/udev/install_udev.sh' in hint
    assert 'Not falling back to /dev/ttyUSB*' in hint


def test_hint_with_rules_installed_asks_for_the_board(tmp_path):
    rules = tmp_path / '99-motor.rules'
    rules.write_text('# installed\n')
    hint = joystick_node.missing_port_hint('/dev/motor', rules=str(rules))
    assert 'plugged in' in hint
    assert 'install_udev' not in hint


def test_hint_for_another_device_names_the_parameter():
    assert '`device` parameter' in joystick_node.missing_port_hint('/dev/ttyUSB7')


def test_rules_path_is_where_install_udev_puts_it():
    assert joystick_node.MOTOR_RULES == '/etc/udev/rules.d/99-motor.rules'
