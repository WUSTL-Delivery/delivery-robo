"""Nav2 local planning: costmap + planner_server + local_planner (TDM-21, TDM-24).

    ros2 launch hardware_bringup nav2_local.launch.py                     # robot, sources:=fused
    ros2 launch hardware_bringup nav2_local.launch.py sources:=separate   # lidar + camera, both mark/clear
    ros2 launch hardware_bringup nav2_local.launch.py sources:=lidar scan_topic:=/scan
    ros2 launch hardware_bringup nav2_local.launch.py sim:=true           # sim time, lidar on /scan
    ros2 launch hardware_bringup nav2_local.launch.py sim:=true drive:=true   # ... and drive the sim
    ros2 launch hardware_bringup nav2_local.launch.py map:=/path/to/boundary.yaml
    ros2 launch hardware_bringup nav2_local.launch.py local_planner:=false    # costmap + planner only

Starts map_server (the boundary map on /map), planner_server (whose global costmap is the
costmap: static_layer from /map, obstacle_layer from the scans, inflation; frame odom, rolling
40 x 40 m around base_link), a lifecycle manager, and hardware_bringup's local_planner (goal on
/goal_pose or /nav/goal_node -> ComputePathToPose -> /nav/local_plan, and with drive:=true on to
delivery-autonomy's /follow_path). Output: /global_costmap/costmap, /nav/local_plan. Parameters
live in config/nav2_local.yaml; this file only wires the sources. local_planner's planner_id is
taken from planner_server's first planner_plugins entry, so the two cannot drift apart.
master_launch.py includes this file in mode:=autonomy with nav2:=true (drive:=true there).

Needs the TF the rest of the stack publishes: map -> odom (gps_datum_tf, or the sim's static
transform), odom -> base_link (delivery-autonomy state_estimation), base_link -> sensor frames
(robot_description).

sources:=
  fused     /scan_fused marks; /scan_filtered and /camera/scan only clear (scan_fusion's
            contract: obstacle endpoints only, NaN for missing). Default on the robot.
  separate  /scan_filtered and /camera/scan both mark and clear.
  lidar     one LaserScan (scan_topic) marks and clears. Default with sim:=true (/scan).

The params file is patched with PyYAML and written to $ROS_HOME/nav2_local_params_domain<N>.yaml
(default ~/.ros, N = ROS_DOMAIN_ID), because launch_ros dict parameters on planner_server never
reach its costmap sub-node (/global_costmap/global_costmap). Every launch on that domain rewrites
the same file, so nothing piles up; it is kept after shutdown (it is what the nodes started with).

Degrades instead of aborting a parent launch: no map found or the patched params file cannot be
written -> logs and starts nothing; Nav2 packages missing -> logs the apt command; nav2_planner
missing -> runs the standalone
nav2_costmap_2d node under the same name (/global_costmap/global_costmap, same parameters, same
topic), so the costmap can still be checked before the planner is installed (local_planner
then only waits for the planner action server); nav2_msgs / shared_types missing or a stale
build without the local_planner executable -> logs and skips local_planner only.
"""
import contextlib
import copy
import os
import tempfile

from ament_index_python.packages import (
    get_package_prefix, get_package_share_directory, PackageNotFoundError)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# obstacle_layer source name (a key in nav2_local.yaml) -> (marking, clearing), per sources:=
SOURCE_MODES = {
    'fused': {'fused': (True, False), 'lidar': (False, True), 'camera': (False, True)},
    'separate': {'lidar': (True, True), 'camera': (True, True)},
    'lidar': {'lidar': (True, True)},
}
STANDALONE_COSTMAP = 'global_costmap/global_costmap'
MANAGER = 'lifecycle_manager_nav2_local'


def _have(pkg):
    try:
        get_package_share_directory(pkg)
        return True
    except PackageNotFoundError:
        return False


def default_map():
    """share/autonomy/maps/boundary.yaml when the autonomy package is built, else ''."""
    try:
        path = os.path.join(get_package_share_directory('autonomy'), 'maps', 'boundary.yaml')
    except PackageNotFoundError:
        return ''
    return path if os.path.isfile(path) else ''


def _ros_params(tree):
    """Every ros__parameters dict in a params-file tree."""
    if isinstance(tree, dict):
        for key, value in tree.items():
            if key == 'ros__parameters' and isinstance(value, dict):
                yield value
            else:
                yield from _ros_params(value)


def patch_params(params, *, map_yaml, sources, scan_topic='', sim=False, planner=True):
    """Return a copy of nav2_local.yaml's tree with the launch arguments applied."""
    params = copy.deepcopy(params)
    if sources == 'auto':
        sources = 'lidar' if sim else 'fused'
    modes = SOURCE_MODES[sources]

    for section in _ros_params(params):
        section['use_sim_time'] = sim

    params['map_server']['ros__parameters']['yaml_filename'] = map_yaml

    obstacle = params['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']
    obstacle['observation_sources'] = ' '.join(modes)
    for name, (marking, clearing) in modes.items():
        obstacle[name]['marking'] = marking
        obstacle[name]['clearing'] = clearing
    lidar_topic = scan_topic or ('/scan' if sim else '')
    if lidar_topic:
        obstacle['lidar']['topic'] = lidar_topic

    manager = params[MANAGER]['ros__parameters']
    if planner:
        manager['node_names'] = ['map_server', 'planner_server']
    else:
        manager['node_names'] = ['map_server', STANDALONE_COSTMAP]
        manager['bond_timeout'] = 0.0   # the standalone costmap never opens a bond
    return params


def patched_params_path(environ=None):
    """Where the patched params file goes: one per user and ROS domain, rewritten by every launch.

    $ROS_HOME (default ~/.ros, where ROS keeps its logs) rather than a new mkstemp file per launch,
    which piled up in /tmp. Keyed by ROS_DOMAIN_ID because a second stack can only run beside this
    one on another domain (one domain cannot hold two /planner_server), e.g. a test next to a sim.
    """
    environ = os.environ if environ is None else environ
    home = os.path.expanduser(environ.get('ROS_HOME') or os.path.join('~', '.ros'))
    domain = environ.get('ROS_DOMAIN_ID', '').strip()
    return os.path.join(home, f'nav2_local_params_domain{domain if domain.isdigit() else 0}.yaml')


def write_params(params, path):
    """Dump params to path atomically: a temp file beside it, then a rename over it.

    A reader never sees half a file, and a failed write leaves the previous file and no stray
    temp file.
    """
    import yaml  # noqa: PLC0415 - see _launch_setup

    directory = os.path.dirname(path)
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix='.nav2_local_', suffix='.yaml', dir=directory)
    try:
        with os.fdopen(fd, 'w') as f:
            yaml.safe_dump(params, f, sort_keys=False)
        os.replace(tmp, path)
    except BaseException:
        with contextlib.suppress(OSError):
            os.unlink(tmp)
        raise


def local_planner_params(params, *, sim=False, drive=False):
    """local_planner's parameters: sim time, drive, and the planner_id planner_server serves."""
    planner = params.get('planner_server', {}).get('ros__parameters', {})
    plugins = planner.get('planner_plugins') or ['GridBased']   # Nav2's own default
    return {'use_sim_time': sim, 'drive': drive, 'planner_id': plugins[0]}


def _local_planner_missing():
    """Why local_planner cannot start here, or '' when it can."""
    missing = [p for p in ('nav2_msgs', 'shared_types') if not _have(p)]
    if missing:
        return (f'missing {", ".join(missing)} (shared_types comes with the delivery-autonomy '
                'packages; nav2_msgs from apt ros-jazzy-nav2-msgs)')
    try:
        exe = os.path.join(get_package_prefix('hardware_bringup'), 'lib', 'hardware_bringup',
                           'local_planner')
    except PackageNotFoundError:
        exe = ''
    if not os.path.exists(exe):
        return 'local_planner executable not installed (stale build? rebuild hardware_bringup)'
    return ''


def _role(source):
    return '+'.join(r for r, on in (('mark', source['marking']), ('clear', source['clearing'])) if on)


def _launch_setup(context):
    import yaml  # noqa: PLC0415 - python3-yaml; imported here so a broken install only fails this launch

    sim = LaunchConfiguration('sim').perform(context) == 'true'
    sources = LaunchConfiguration('sources').perform(context)
    scan_topic = LaunchConfiguration('scan_topic').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    map_yaml = LaunchConfiguration('map').perform(context) or default_map()

    missing = [p for p in ('nav2_map_server', 'nav2_costmap_2d', 'nav2_lifecycle_manager')
               if not _have(p)]
    if missing:
        return [LogInfo(msg=(
            f'[nav2_local] missing {", ".join(missing)}: sudo apt install '
            + ' '.join('ros-jazzy-' + p.replace('_', '-') for p in missing)
            + '; Nav2 NOT started'))]
    if not map_yaml:
        return [LogInfo(msg=('[nav2_local] no map: pass map:=<boundary.yaml> (the default '
                             'share/autonomy/maps/boundary.yaml needs the autonomy package built); '
                             'Nav2 NOT started'))]
    map_yaml = os.path.abspath(os.path.expanduser(map_yaml))
    if not os.path.isfile(map_yaml):
        return [LogInfo(msg=f'[nav2_local] map {map_yaml} does not exist; Nav2 NOT started')]

    planner = _have('nav2_planner')
    with open(params_file) as f:
        params = patch_params(yaml.safe_load(f), map_yaml=map_yaml, sources=sources,
                              scan_topic=scan_topic, sim=sim, planner=planner)
    patched = patched_params_path()
    try:
        write_params(params, patched)
    except OSError as ex:
        return [LogInfo(msg=f'[nav2_local] cannot write the patched params file {patched}: {ex}; '
                            'Nav2 NOT started (set ROS_HOME to a writable directory)')]

    obstacle = params['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']
    wiring = ', '.join(f"{obstacle[n]['topic']} ({_role(obstacle[n])})"
                       for n in obstacle['observation_sources'].split())
    actions = [LogInfo(msg=f'[nav2_local] map {map_yaml}; scans: {wiring}; params {patched}')]
    if sim and sources in ('fused', 'separate'):
        actions.append(LogInfo(msg=f'[nav2_local] WARNING: sources:={sources} with sim:=true, but the '
                                   'sim publishes only /scan; the costmap will go stale'))

    actions.append(Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[patched],
    ))
    if planner:
        actions.append(Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[patched],
        ))
    else:
        actions += [
            LogInfo(msg=('[nav2_local] nav2_planner not installed (sudo apt install '
                         'ros-jazzy-nav2-planner ros-jazzy-nav2-smac-planner): running the standalone '
                         'costmap as /global_costmap/global_costmap, no planning')),
            Node(
                package='nav2_costmap_2d',
                executable='nav2_costmap_2d',
                namespace='global_costmap',
                name='global_costmap',
                output='screen',
                parameters=[patched],
            ),
        ]
    actions.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name=MANAGER,
        output='screen',
        parameters=[patched],
    ))

    if LaunchConfiguration('local_planner').perform(context) == 'true':
        why_not = _local_planner_missing()
        if why_not:
            actions.append(LogInfo(msg=f'[nav2_local] {why_not}; local_planner NOT started'))
        else:
            drive = LaunchConfiguration('drive').perform(context) == 'true'
            actions.append(Node(
                package='hardware_bringup',
                executable='local_planner',
                name='local_planner',
                output='screen',
                respawn=True,          # a restart forgets the goal: it never resumes driving by itself
                respawn_delay=3.0,
                parameters=[local_planner_params(params, sim=sim, drive=drive)],
            ))
    return actions


def generate_launch_description():
    share = get_package_share_directory('hardware_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='',
            description='boundary map yaml for map_server; empty = share/autonomy/maps/boundary.yaml '
                        '(required when the autonomy package is not built)'),
        DeclareLaunchArgument(
            'sources', default_value='auto', choices=['auto', 'fused', 'separate', 'lidar'],
            description='obstacle_layer inputs: fused (/scan_fused marks, lidar + camera clear), '
                        'separate (lidar + camera mark and clear), lidar (one LaserScan); '
                        'auto = lidar with sim:=true, else fused'),
        DeclareLaunchArgument(
            'scan_topic', default_value='',
            description='lidar LaserScan topic; empty = /scan_filtered (/scan with sim:=true)'),
        DeclareLaunchArgument(
            'sim', default_value='false', choices=['true', 'false'],
            description='delivery-autonomy Gazebo sim: use_sim_time, lidar on /scan, sources auto -> lidar'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(share, 'config', 'nav2_local.yaml'),
            description='Nav2 parameters (map_server, planner_server, global_costmap, lifecycle manager)'),
        DeclareLaunchArgument(
            'local_planner', default_value='true', choices=['true', 'false'],
            description='run hardware_bringup local_planner (/goal_pose or /nav/goal_node -> '
                        'ComputePathToPose -> /nav/local_plan); false = costmap + planner_server only'),
        DeclareLaunchArgument(
            'drive', default_value='false', choices=['true', 'false'],
            description="local_planner also sends each path to delivery-autonomy's /follow_path (MPPI); "
                        'false = plan and publish /nav/local_plan only'),
        OpaqueFunction(function=_launch_setup),
    ])
