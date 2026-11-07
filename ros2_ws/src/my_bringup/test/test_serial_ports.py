"""
TDM-18: the motor board and the lidar are opened by their own udev symlinks.

Both are /dev/ttyUSBn, and plug order decides which is ttyUSB0, so every launch opens them as
/dev/motor (deployment/udev/99-motor.rules) and /dev/rplidar (99-rplidar.rules). The rules must
create exactly those links, match different USB vendors (so one tty never gets both), and both
be installed by install_udev.sh.
"""
import importlib.util
import os
from pathlib import Path
import re

import pytest

pytest.importorskip('launch_ros')
yaml = pytest.importorskip('yaml')

PKG = Path(__file__).resolve().parents[1]
REPO = PKG.parents[2]
UDEV = REPO / 'deployment' / 'udev'


def _load_master_launch():
    path = PKG / 'launch' / 'master_launch.py'
    spec = importlib.util.spec_from_file_location('tdm18_master_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _rules(name):
    """Each active rule line of a udev rules file as {key: value}, e.g. 'ATTRS{idVendor}'."""
    path = UDEV / name
    if not path.is_file():
        pytest.skip(f'{path} not found (not running from a delivery-robo checkout)')
    rules = []
    for line in path.read_text().splitlines():
        if line.strip() and not line.lstrip().startswith('#'):
            rules.append(dict(re.findall(r'([A-Z]+(?:\{\w+\})?)\s*[+:!=]?=\s*"([^"]*)"', line)))
    return rules


def test_teleop_joystick_node_opens_dev_motor():
    from launch import LaunchContext
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters

    description = _load_master_launch().generate_launch_description()
    (control,) = [e for e in description.entities
                  if isinstance(e, Node) and e.node_executable == 'joystick_node']
    params = {}
    for entry in evaluate_parameters(LaunchContext(), control._Node__parameters):
        assert isinstance(entry, dict), entry
        params.update(entry)
    assert params == {'device': '/dev/motor'}


def test_lidar_opens_dev_rplidar():
    with open(PKG / 'config' / 'lidar.yaml') as f:
        sllidar = yaml.safe_load(f)['sllidar_node']['ros__parameters']
    assert sllidar['serial_port'] == '/dev/rplidar'


def test_motor_rule_makes_dev_motor_for_the_ch340_only():
    (rule,) = _rules('99-motor.rules')
    assert rule['SUBSYSTEM'] == 'tty'
    assert rule['SYMLINK'] == 'motor'
    assert rule['ATTRS{idVendor}'] == '1a86'
    # every WCH id the kernel's ch341 driver binds (`modinfo ch341`); a vendor-only match would
    # also fire behind a WCH USB hub, for the lidar too
    assert set(rule['ATTRS{idProduct}'].split('|')) == {'7523', '7522', '5523'}


def test_rplidar_rule_makes_dev_rplidar_for_the_cp2102():
    (rule,) = _rules('99-rplidar.rules')
    assert rule['SYMLINK'] == 'rplidar'
    assert (rule['ATTRS{idVendor}'], rule['ATTRS{idProduct}']) == ('10c4', 'ea60')


def test_no_device_matches_both_rules():
    (motor,) = _rules('99-motor.rules')
    (lidar,) = _rules('99-rplidar.rules')
    assert motor['ATTRS{idVendor}'] != lidar['ATTRS{idVendor}']


def test_install_udev_installs_and_reports_both():
    script = UDEV / 'install_udev.sh'
    if not script.is_file():
        pytest.skip(f'{script} not found')
    text = script.read_text()
    (rules,) = re.findall(r'^RULES=\((.*)\)$', text, flags=re.M)
    assert {'99-motor.rules', '99-rplidar.rules'} <= set(rules.split())
    assert '-e /dev/motor' in text and '-e /dev/rplidar' in text
    assert os.access(script, os.X_OK)
