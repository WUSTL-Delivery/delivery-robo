"""TDM-34: launch/autonomy_real.launch.py gives delivery-autonomy's controller the robot's settings.

The controller gets config/hardware.yaml (its controller_server section) in both modes, and
use_costmap = the nav2 launch argument. TDM-41: its vehicle model is the robot's (hardware.yaml's
/** wheelbase and max_steer_rad, arduino_bridge's values) on the robot and the Gazebo car's with
sim:=true. Needs launch_ros importable (a sourced ROS environment); skipped otherwise.
"""
import importlib.util
import os
import pathlib

import pytest

pytest.importorskip('launch_ros')
yaml = pytest.importorskip('yaml')

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HARDWARE_YAML = os.path.join(PKG, 'config', 'hardware.yaml')

# delivery-autonomy dev's controller vehicle parameters (utils/dynamics.py VEHICLE_PARAMS) and
# their defaults, the sim car's (SIM_VEHICLE)
VEHICLE_DEFAULTS = {'wheelbase': 0.5, 'max_steer_rad': 0.6, 'steer_tau': 0.545, 'drive_tau': 0.01,
                    'max_accel': 5.0}
# Every parameter delivery-autonomy dev's controller declares (controller.py CONTROLLER_PARAMS,
# costmap_speed.py GOVERNOR_PARAMS, dynamics.py VEHICLE_PARAMS, the two topics). rclpy silently
# ignores a parameter a node never declares, so a misspelt key in hardware.yaml would otherwise go
# unnoticed.
CONTROLLER_PARAMETERS = {
    'max_speed', 'min_speed', 'xy_goal_tolerance', 'yaw_goal_tolerance',
    'use_costmap', 'slow_distance', 'stop_distance', 'clearance_lookahead', 'costmap_timeout',
    'crawl_speed', 'goal_decel', 'goal_creep_speed', 'state_topic', 'costmap_topic',
    *VEHICLE_DEFAULTS,
}
# The speeds MPPI samples within (controller.py MPPI_MIN_SPEED, MPPI_MAX_SPEED)
MPPI_MIN_SPEED, MPPI_MAX_SPEED = -0.5, 1.5


def _load_launch_module():
    path = os.path.join(PKG, 'launch', 'autonomy_real.launch.py')
    spec = importlib.util.spec_from_file_location('tdm34_autonomy_real_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


autonomy_real = _load_launch_module()


def _controller(tmp_path, sim, nav2):
    """Run the launch file's OpaqueFunction as `ros2 launch` would; return the controller Node and
    its parameters evaluated (a Path per parameter file, a dict per inline set)."""
    from launch import LaunchContext
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters

    context = LaunchContext()
    context.launch_configurations.update({
        'hardware_params': HARDWARE_YAML, 'gps_in_topic': '', 'ekf_cmd_vel_topic': '',
        'sim': sim, 'nav2': nav2, 'planner_cache_dir': str(tmp_path)})
    nodes = [a for a in autonomy_real._nodes(context) if isinstance(a, Node)]
    (controller,) = [n for n in nodes if n.node_executable == 'controller']
    return controller, evaluate_parameters(context, controller._Node__parameters), context


def _remappings(node, context):
    from launch.utilities import perform_substitutions

    return [(perform_substitutions(context, src), perform_substitutions(context, dst))
            for src, dst in node._Node__remappings]


@pytest.fixture
def controller_yaml():
    with open(HARDWARE_YAML) as f:
        return yaml.safe_load(f)['controller_server']['ros__parameters']


@pytest.mark.parametrize('sim', ['false', 'true'])
@pytest.mark.parametrize('nav2', ['false', 'true'])
def test_controller_gets_hardware_yaml_and_use_costmap_from_nav2(tmp_path, sim, nav2):
    _, params, _ = _controller(tmp_path, sim, nav2)
    files = [p for p in params if isinstance(p, pathlib.Path)]
    inline = {}
    for p in params:
        if isinstance(p, dict):
            inline.update(p)
    assert files == [pathlib.Path(HARDWARE_YAML)]
    # inline values come after the file, so they win
    assert isinstance(params[-1], dict)
    assert inline['use_costmap'] is (nav2 == 'true')
    assert inline['use_sim_time'] is (sim == 'true')
    # nothing in the inline set shadows a hardware.yaml controller value, except that the sim car
    # keeps its own vehicle
    vehicle = set(VEHICLE_DEFAULTS) if sim == 'true' else set()
    assert set(inline) == {'use_costmap', 'use_sim_time'} | vehicle


def test_nav2_defaults_to_false(monkeypatch):
    from launch.actions import DeclareLaunchArgument

    monkeypatch.setattr(autonomy_real, 'get_package_share_directory', lambda _: PKG)
    description = autonomy_real.generate_launch_description()
    args = {a.name: a for a in description.entities if isinstance(a, DeclareLaunchArgument)}
    assert 'nav2' in args
    assert args['nav2'].choices == ['true', 'false']
    default = ''.join(s.perform(None) for s in args['nav2'].default_value)
    assert default == 'false'


def test_controller_remapped_to_twist_mux_only_on_the_robot(tmp_path):
    robot, _, context = _controller(tmp_path, 'false', 'false')
    sim, _, _ = _controller(tmp_path, 'true', 'false')
    assert _remappings(robot, context) == [('/cmd_vel', '/cmd_vel_nav')]
    assert _remappings(sim, context) == []


def _effective(params, node):
    """The parameter values `node` ends up with from the evaluated parameter list: each file's /**
    section, then its section for the node (later wins, as in rcl), then each inline dict."""
    values = {}
    for entry in params:
        if isinstance(entry, dict):
            values.update(entry)
            continue
        with open(entry) as f:
            sections = yaml.safe_load(f)
        for name in ('/**', node):
            values.update(sections.get(name, {}).get('ros__parameters', {}))
    return values


def test_robot_controller_models_the_robot_the_bridge_drives(tmp_path):
    _, params, _ = _controller(tmp_path, 'false', 'true')
    controller = _effective(params, 'controller_server')
    bridge = _effective([pathlib.Path(HARDWARE_YAML)], 'arduino_bridge')
    # The controller converts its steering angle to a yaw rate with wheelbase and clips it at
    # max_steer_rad; the bridge converts back and clips with its own. Equal, the angle survives.
    for name in ('wheelbase', 'max_steer_rad'):
        assert controller[name] == bridge[name], name
    assert (controller['wheelbase'], controller['max_steer_rad']) == (0.36322, 0.4363)
    # The robot's lags are unmeasured: the controller keeps its defaults
    for name in ('steer_tau', 'drive_tau', 'max_accel'):
        assert name not in controller, name


@pytest.mark.parametrize('nav2', ['false', 'true'])
def test_sim_controller_models_the_gazebo_car(tmp_path, nav2):
    _, params, _ = _controller(tmp_path, 'true', nav2)
    controller = _effective(params, 'controller_server')
    assert {name: controller[name] for name in VEHICLE_DEFAULTS} == VEHICLE_DEFAULTS
    assert autonomy_real.SIM_VEHICLE == VEHICLE_DEFAULTS


def test_hardware_yaml_controller_keys_are_controller_parameters(controller_yaml):
    assert set(controller_yaml) <= CONTROLLER_PARAMETERS
    # the launch file owns use_costmap: a value here would be overridden anyway
    assert 'use_costmap' not in controller_yaml


def test_hardware_yaml_controller_values_are_valid(controller_yaml):
    assert 0.0 < controller_yaml['max_speed'] <= MPPI_MAX_SPEED
    assert MPPI_MIN_SPEED <= controller_yaml['min_speed'] <= 0.0
    # slow_distance (default 2.0 m) must stay above stop_distance
    assert 0.0 <= controller_yaml['stop_distance'] < controller_yaml.get('slow_distance', 2.0)
    for name, value in controller_yaml.items():
        assert isinstance(value, float), f'{name} must be a double (write 1.0, not 1): {value!r}'


def test_robot_never_reverses_and_squeezes_past_a_centred_obstacle(controller_yaml):
    # Nav2's Hybrid-A* paths are Dubins (no reversing), so neither does the follower
    assert controller_yaml['min_speed'] == 0.0
    # 0.35 m (the default) stops the robot for good at an obstacle centred in a 3 m corridor;
    # 0.2 m was approved for the robot on 2026-10-16 (TDM-34)
    assert controller_yaml['stop_distance'] == 0.2
