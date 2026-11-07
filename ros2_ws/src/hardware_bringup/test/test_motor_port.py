"""
TDM-18: arduino_bridge opens the motor board as /dev/motor, never a raw /dev/ttyUSBn.

/dev/motor is the board's udev symlink (deployment/udev/99-motor.rules). The RPLidar C1 is a
/dev/ttyUSBn too; on the Oct 24 Pi test the bridge and sllidar_node fought over one tty. The
port comes from config/hardware.yaml (drive.launch.py's port:= overrides it), the node's own
default is the same, and a missing /dev/motor is reported with the command that fixes it.
No serial port is opened here: serial.Serial and the existence check are faked. Needs
launch_ros; the node tests also need rclpy and pyserial (python3-serial, on the Pi) and are
skipped without them.
"""
import importlib.util
import os
import time
import types

import pytest

pytest.importorskip('launch_ros')
yaml = pytest.importorskip('yaml')

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HARDWARE_YAML = os.path.join(PKG, 'config', 'hardware.yaml')


def _load_drive_launch():
    path = os.path.join(PKG, 'launch', 'drive.launch.py')
    spec = importlib.util.spec_from_file_location('tdm18_drive_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


drive = _load_drive_launch()


def _bridge_port(port_arg):
    """Return the port arduino_bridge gets from drive.launch.py with port:=port_arg."""
    from launch import LaunchContext
    from launch_ros.utilities import evaluate_parameters

    context = LaunchContext()
    context.launch_configurations.update({'hardware_params': HARDWARE_YAML, 'port': port_arg})
    (node,) = drive._bridge(context)
    assert node.node_executable == 'arduino_bridge'
    values = {}
    for entry in evaluate_parameters(context, node._Node__parameters):
        if isinstance(entry, dict):
            values.update(entry)
        else:
            with open(entry) as f:
                sections = yaml.safe_load(f)
            for name in ('/**', 'arduino_bridge'):
                values.update(sections.get(name, {}).get('ros__parameters', {}))
    return values['port']


def test_hardware_yaml_port_is_the_motor_symlink():
    with open(HARDWARE_YAML) as f:
        bridge = yaml.safe_load(f)['arduino_bridge']['ros__parameters']
    assert bridge['port'] == '/dev/motor'


def test_drive_launch_gives_the_bridge_dev_motor():
    assert _bridge_port('') == '/dev/motor'


def test_drive_launch_port_argument_still_overrides():
    assert _bridge_port('/dev/serial/by-id/usb-test') == '/dev/serial/by-id/usb-test'


@pytest.fixture
def arduino_bridge():
    pytest.importorskip('rclpy')
    pytest.importorskip('serial')
    from hardware_bringup import arduino_bridge
    return arduino_bridge


def test_node_default_is_the_motor_symlink(arduino_bridge):
    assert arduino_bridge.DEFAULT_PORT == '/dev/motor'


def test_hint_without_rules_says_what_to_run(arduino_bridge, tmp_path):
    hint = arduino_bridge.missing_port_hint('/dev/motor', rules=str(tmp_path / 'missing.rules'))
    assert 'sudo deployment/udev/install_udev.sh' in hint
    assert 'Not falling back to /dev/ttyUSB*' in hint


def test_hint_with_rules_installed_asks_for_the_board(arduino_bridge, tmp_path):
    rules = tmp_path / '99-motor.rules'
    rules.write_text('# installed\n')
    hint = arduino_bridge.missing_port_hint('/dev/motor', rules=str(rules))
    assert 'plugged in' in hint
    assert 'install_udev' not in hint


def test_hint_for_another_port_names_the_parameter(arduino_bridge):
    assert '`port` parameter' in arduino_bridge.missing_port_hint('/dev/ttyUSB7')


def test_missing_dev_motor_opens_nothing_then_recovers(arduino_bridge, monkeypatch):
    import rclpy

    opened = []

    class FakeSerial:
        def __init__(self, port, baud, **kwargs):
            opened.append((port, baud))

        def reset_input_buffer(self):
            pass

        def read(self, n):
            time.sleep(0.01)                    # the reader thread polls; don't spin
            return b''

        in_waiting = 0

        def write(self, data):
            pass

        def flush(self):
            pass

        def close(self):
            pass

    present = set()
    monkeypatch.setattr(arduino_bridge.serial, 'Serial', FakeSerial)
    fake_os = types.SimpleNamespace(path=types.SimpleNamespace(exists=present.__contains__))
    monkeypatch.setattr(arduino_bridge, 'os', fake_os)
    rclpy.init()
    node = arduino_bridge.ArduinoBridge()
    try:
        assert node._ser is None
        assert opened == []                     # no fallback to some /dev/ttyUSBn
        assert node._open_serial() is False     # what the 1 s reconnect timer calls
        present.add('/dev/motor')
        assert node._open_serial() is True
        assert opened == [('/dev/motor', 57600)]
    finally:
        node.shutdown_hardware()
        node.destroy_node()
        rclpy.shutdown()
