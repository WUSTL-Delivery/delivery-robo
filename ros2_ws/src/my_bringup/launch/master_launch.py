import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

# Selected by deployment/robot_config.yaml (via startup.sh), or by hand:
#   ros2 launch my_bringup master_launch.py mode:=teleop
#   ros2 launch my_bringup master_launch.py mode:=autonomous
#   ros2 launch my_bringup master_launch.py mode:=autonomy
#
# teleop     joystick straight to the Arduino (joy_node + joystick_control). Unchanged.
# autonomous sensors only (RTK GPS + NTRIP, IMU). Nothing drives.
# autonomy   sensors + hardware_bringup drive chain (joystick -> twist_mux -> arduino_bridge)
#            + the delivery-autonomy nodes (gps datum, EKF, planner, MPPI controller).
#            The joystick overrides the controller at any time while its enable button is held.
#
# sim:=true (autonomy only, laptop testing) swaps the sensors and the drive chain for the Gazebo
# sim in the delivery-autonomy submodule; the autonomy nodes are the same. Needs Gazebo, i.e.
# delivery-autonomy's `nix develop` shell. Never on the robot.

# NTRIP caster settings for the ublox_dgnss ntrip client. The committed values are the
# historical ones (they are in git history already). deployment/ntrip.local.yaml (gitignored,
# top-level "key: value" lines) overrides them key by key, so the credentials can be rotated
# on the robot without a commit. See deployment/README.md.
NTRIP_DEFAULTS = {
    'use_https': 'false',
    'host': '168.166.125.30',
    'port': '2101',
    'mountpoint': 'RTX_RTCM34',
    'username': '/WashUroboticsDelivery2026',
    'password': '$DeliveryWU197?',
}


def _repo_root():
    """robotics/ checkout: $REPO, else derived from this file (symlink-install), else ~/delivery-robo."""
    candidates = [os.environ.get('REPO')]
    here = os.path.dirname(os.path.realpath(__file__))
    candidates.append(os.path.abspath(os.path.join(here, '..', '..', '..', '..')))  # launch/ -> my_bringup -> src -> ros2_ws -> repo
    candidates.append(os.path.expanduser('~/delivery-robo'))
    for c in candidates:
        if c and os.path.isfile(os.path.join(c, 'deployment', 'robot_config.yaml')):
            return c
    return None


def _ntrip_settings():
    settings = dict(NTRIP_DEFAULTS)
    repo = _repo_root()
    path = os.path.join(repo, 'deployment', 'ntrip.local.yaml') if repo else None
    if path and os.path.isfile(path):
        try:
            import yaml
            with open(path) as f:
                override = yaml.safe_load(f) or {}
            for key in settings:
                if key in override and override[key] is not None:
                    settings[key] = str(override[key]).lower() if key == 'use_https' else str(override[key])
            print(f'[master_launch] NTRIP settings from {path}')
        except Exception as exc:  # noqa: BLE001 - a bad override must not brick the boot
            print(f'[master_launch] WARNING: could not read {path} ({exc}); using committed NTRIP defaults')
    else:
        print('[master_launch] NTRIP: no deployment/ntrip.local.yaml, using the committed defaults '
              '(rotate the caster password and put the new one in that file)')
    return settings


def generate_launch_description():
   mode = LaunchConfiguration('mode')
   mode_arg = DeclareLaunchArgument(
      'mode',
      default_value='teleop',
      choices=['teleop', 'autonomous', 'autonomy'],
      description='teleop = wired joystick control; autonomous = sensor stack only (GPS/NTRIP/IMU); '
                  'autonomy = sensors + drive bridge + delivery-autonomy with joystick override',
   )
   sim = LaunchConfiguration('sim')
   sim_arg = DeclareLaunchArgument(
      'sim',
      default_value='false',
      choices=['true', 'false'],
      description='autonomy mode only: the delivery-autonomy Gazebo sim replaces the sensors and the '
                  'drive chain (laptop testing inside its nix develop shell; never on the robot)',
   )
   robot_id = LaunchConfiguration('robot_id')
   api_url = LaunchConfiguration('api_url')
   robot_id_arg = DeclareLaunchArgument(
      'robot_id', default_value='robo-1',
      description='id shown on the robo-web dashboard',
   )
   api_url_arg = DeclareLaunchArgument(
      'api_url', default_value='https://robo-web-ebon.vercel.app',
      description='robo-web base URL; heartbeat POSTs to <api_url>/api/heartbeat',
   )
   is_teleop = IfCondition(PythonExpression(["'", mode, "' == 'teleop'"]))
   # sensors run in both non-teleop modes, except when the sim stands in for them
   is_sensors = IfCondition(PythonExpression(
      ["'", mode, "' == 'autonomous' or ('", mode, "' == 'autonomy' and '", sim, "' != 'true')"]))
   is_autonomy = IfCondition(PythonExpression(["'", mode, "' == 'autonomy'"]))
   is_autonomy_real = IfCondition(PythonExpression(["'", mode, "' == 'autonomy' and '", sim, "' != 'true'"]))
   is_autonomy_sim = IfCondition(PythonExpression(["'", mode, "' == 'autonomy' and '", sim, "' == 'true'"]))

   # --- heartbeat: both modes, reports status to robo-web ----------------
   # Guard: with --symlink-install this launch file is the *new* one even when
   # the build failed and the install is the *old* one. A Node whose executable
   # is missing aborts the whole launch, so only add it if it was built.
   try:
      from ament_index_python.packages import get_package_prefix
      _hb_exe = os.path.join(get_package_prefix('my_bringup'), 'lib', 'my_bringup', 'heartbeat_node')
      heartbeat_available = os.path.exists(_hb_exe)
   except Exception:  # noqa: BLE001
      heartbeat_available = False
   if not heartbeat_available:
      print('[master_launch] heartbeat_node executable not found (stale build?); skipping status heartbeat')

   heartbeat_node = Node(
      package='my_bringup',
      executable='heartbeat_node',
      name='heartbeat',
      respawn=True,
      respawn_delay=3.0,
      output='screen',
      parameters=[{'robot_id': robot_id, 'api_url': api_url, 'mode': mode}],
   )

   # --- teleop: wired joystick (verified working on the Pi, keep as-is) -----
   joy_node = Node(
      package='joy',
      executable='joy_node',
      name='joy_node',
      respawn=True,
      respawn_delay=3.0,
      output='screen',
      condition=is_teleop,
   )

   control_node = Node(
      package='joystick_control',
      executable='joystick_node',
      name='control_node',
      respawn=True,
      respawn_delay=3.0,
      output='screen',
      condition=is_teleop,
   )

   # --- sensors (autonomous + autonomy) ---------------------------------------
   # Resolved at load time, so guard it: teleop must still work on a Pi
   # where ublox_dgnss isn't built.
   try:
      ublox_dir = get_package_share_directory('ublox_dgnss')
   except PackageNotFoundError:
      ublox_dir = None
      print('[master_launch] ublox_dgnss not found; autonomous/autonomy modes have no GPS/NTRIP nodes')

   gps_main_node = IncludeLaunchDescription(
       PythonLaunchDescriptionSource(
           os.path.join(ublox_dir or '', 'launch', 'ublox_rover_hpposllh_navsatfix.launch.py')
       ),
       condition=is_sensors,
   )

   gps_ntrip_node = IncludeLaunchDescription(
       PythonLaunchDescriptionSource(
           os.path.join(ublox_dir or '', 'launch', 'ntrip_client.launch.py')
       ),
       launch_arguments=_ntrip_settings().items(),
       condition=is_sensors,
   )

   imu_node = Node(
       package='imu_package',
       executable='imu_publisher',
       name='imu_publisher',
       respawn=True,
       respawn_delay=3.0,
       output='screen',
       condition=is_sensors,
   )

   sensor_nodes = [imu_node]
   if ublox_dir is not None:
      sensor_nodes += [gps_main_node, gps_ntrip_node]

   # --- autonomy: drive chain + delivery-autonomy ------------------------------
   # Same guard idea: a missing package must degrade, not abort the boot.
   autonomy_nodes = []
   try:
      hw_dir = get_package_share_directory('hardware_bringup')
   except PackageNotFoundError:
      hw_dir = None
      print('[master_launch] hardware_bringup not found; autonomy mode has no drive chain or autonomy nodes')
   if hw_dir is not None:
      autonomy_nodes.append(IncludeLaunchDescription(
          PythonLaunchDescriptionSource(os.path.join(hw_dir, 'launch', 'drive.launch.py')),
          condition=is_autonomy_real,
      ))
      try:
         get_package_share_directory('autonomy')
         autonomy_nodes.append(IncludeLaunchDescription(
             PythonLaunchDescriptionSource(os.path.join(hw_dir, 'launch', 'autonomy_real.launch.py')),
             launch_arguments={'sim': sim}.items(),
             condition=is_autonomy,
         ))
      except PackageNotFoundError:
         print('[master_launch] autonomy package not found (submodule ros2_ws/src/delivery-autonomy not '
               'checked out or not built); autonomy mode is joystick-through-bridge only')

   # --- sim (autonomy + sim:=true): Gazebo in place of the sensors and the drive chain ----
   # run_simulator.sh (Gazebo, robot spawn, ros_gz_bridge) is not installed by the simulation
   # package, so it runs from the submodule checkout; simulation.launch.py adds the robot state
   # publisher, ground truth and RViz. HEADLESS=1 in the environment hides the Gazebo window.
   sim_nodes = []
   repo = _repo_root()
   sim_script = os.path.join(repo or '', 'ros2_ws', 'src', 'delivery-autonomy', 'src', 'simulation',
                             'scripts', 'run_simulator.sh')
   try:
      sim_dir = get_package_share_directory('simulation')
   except PackageNotFoundError:
      sim_dir = None
   if sim_dir is None or not os.path.isfile(sim_script):
      # the robot never builds the simulation package, so only complain when the sim was asked for
      sim_nodes.append(LogInfo(
         msg='[master_launch] WARNING: simulation package or run_simulator.sh not found; sim:=true has no '
             'sim (startup.sh builds the simulation package only with sim: true)',
         condition=is_autonomy_sim,
      ))
   else:
      sim_nodes += [
         ExecuteProcess(cmd=['bash', sim_script], name='gazebo', output='screen', condition=is_autonomy_sim),
         IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim_dir, 'launch', 'simulation.launch.py')),
            condition=is_autonomy_sim,
         ),
      ]

   return LaunchDescription([
        mode_arg,
        sim_arg,
        robot_id_arg,
        api_url_arg,
        *([heartbeat_node] if heartbeat_available else []),
        joy_node, 
        control_node, 
        *sensor_nodes,
        *autonomy_nodes,
        *sim_nodes,
    ])
