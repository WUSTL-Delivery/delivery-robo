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
#
# nav2:=true (autonomy only, opt-in until the Pi's CPU budget is measured) adds Nav2 local
# planning: hardware_bringup nav2_local.launch.py = map_server (boundary map) + planner_server
# (costmap from the map, EKF odom and scans; Smac Hybrid-A*) + lifecycle manager + local_planner,
# which drives through the MPPI's /follow_path. Goals on /goal_pose or /nav/goal_node. Needs
# the apt Nav2 packages (deployment/install_deps.sh); without them it logs and skips,
# e.g. in the Lyrical sim shell, where Nav2 runs from a separate Jazzy terminal instead
# (hardware_bringup/README.md, "Nav2 local planning"). nav2 also sets the MPPI controller's
# use_costmap, so pass nav2:=true there as well when the Jazzy terminal runs Nav2.
#
# A missing obstacle sensor (TDM-52, the user's rule): with one of lidar / depth camera missing
# the robot keeps planning and driving on the other; with both missing it stops driving
# autonomously and the joystick still overrides.
#   * nav2:=true with nav2_sources auto (robot) or fused also starts my_bringup scan_fusion
#     (/scan_filtered + /camera/scan -> /scan_fused, fixed_frame odom from the EKF; with sim:=true
#     the lidar is /scan). It publishes camera-only frames when the lidar is missing and nothing
#     when both are, so the costmap goes stale and planning stops only then.
#   * on the robot, hardware_bringup obstacle_guard (autonomy_real.launch.py) locks twist_mux's
#     navigation input while both scans are silent, also with nav2:=false. lidar:=false and
#     realsense:=false count as missing sensors. obstacle_guard:=false disables it for bench work.

# NTRIP caster settings for the ublox_dgnss ntrip client. The committed values are the
# historical ones (they are in git history already). deployment/ntrip.local.yaml (gitignored,
# top-level "key: value" lines) overrides them key by key, so the credentials can be rotated
# on the robot without a commit. See deployment/README.md.
#
# RTX_RTCM34 is a MoDOT network RTK (VRS) mountpoint: the caster accepts the login but streams
# no RTCM until the rover reports its position in a GGA sentence (sourcetable nmea flag = 1).
# send_gga makes the client take the position from /fix and send it on every (re)connect;
# without it the receiver never leaves a plain 3D fix (diff_soln false, no RXM-RTCM).
NTRIP_DEFAULTS = {
    'use_https': 'false',
    'host': '168.166.125.30',
    'port': '2101',
    'mountpoint': 'RTX_RTCM34',
    'username': '/WashUroboticsDelivery2026',
    'password': '$DeliveryWU197?',
    'send_gga': 'true',
    'fix_topic': '/fix',
}
NTRIP_BOOL_KEYS = ('use_https', 'send_gga')


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
                    settings[key] = str(override[key]).lower() if key in NTRIP_BOOL_KEYS else str(override[key])
            print(f'[master_launch] NTRIP settings from {path}')
        except Exception as exc:  # noqa: BLE001 - a bad override must not brick the boot
            print(f'[master_launch] WARNING: could not read {path} ({exc}); using committed NTRIP defaults')
    else:
        print('[master_launch] NTRIP: no deployment/ntrip.local.yaml, using the committed defaults '
              '(rotate the caster password and put the new one in that file)')
    return settings


def _obstacle_guard_arg():
    """Declare obstacle_guard, passed to autonomy_real.launch.py (TDM-52)."""
    return DeclareLaunchArgument(
        'obstacle_guard', default_value='true', choices=['true', 'false'],
        description='autonomy mode on the robot: block autonomous driving (twist_mux lock) while '
                    'neither the lidar nor the depth camera publishes; false = disabled, for '
                    'bench work only')


def _scan_fusion_actions(mode, sim, nav2, nav2_sources):
    """
    Start my_bringup scan_fusion when Nav2's costmap reads /scan_fused (TDM-52).

    That is mode:=autonomy with nav2:=true and nav2_sources fused, or auto on the robot (auto is
    lidar in the sim). Inputs /scan_filtered (/scan with sim:=true) and /camera/scan. Motion
    compensation in odom: state_estimation (autonomy_real.launch.py) publishes odom -> base_link,
    which the costmap (global_frame odom) needs as well. Guarded like heartbeat_node against a
    stale build without the executable.
    """
    is_fusion = IfCondition(PythonExpression(
        ["'", mode, "' == 'autonomy' and '", nav2, "' == 'true' and ('", nav2_sources,
         "' == 'fused' or ('", nav2_sources, "' == 'auto' and '", sim, "' != 'true'))"]))
    try:
        from ament_index_python.packages import get_package_prefix
        exe = os.path.join(get_package_prefix('my_bringup'), 'lib', 'my_bringup', 'scan_fusion')
        available = os.path.exists(exe)
    except Exception:  # noqa: BLE001
        available = False
    if not available:
        return [LogInfo(
            msg='[master_launch] WARNING: scan_fusion executable not found (stale build? rebuild '
                'my_bringup); no /scan_fused, so the fused costmap goes stale and nothing is '
                'planned',
            condition=is_fusion,
        )]
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('my_bringup'), 'launch', 'scan_fusion.launch.py')),
        launch_arguments={
            'lidar_topic': PythonExpression(
                ["'/scan' if '", sim, "' == 'true' else '/scan_filtered'"]),
            'fixed_frame': 'odom',
            'use_sim_time': sim,
        }.items(),
        condition=is_fusion,
    )]


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
   nav2 = LaunchConfiguration('nav2')
   nav2_arg = DeclareLaunchArgument(
      'nav2',
      default_value='false',
      choices=['true', 'false'],
      description='autonomy mode only: Nav2 local planning (boundary-map costmap + Hybrid-A* planner_server '
                  '+ local_planner driving the MPPI via /follow_path); goals on /goal_pose or /nav/goal_node',
   )
   nav2_sources = LaunchConfiguration('nav2_sources')
   nav2_sources_arg = DeclareLaunchArgument(
      'nav2_sources',
      default_value='auto',
      choices=['auto', 'fused', 'separate', 'lidar'],
      description='with nav2:=true: costmap scan inputs (nav2_local.launch.py sources:=); auto = lidar '
                  '(/scan) with sim:=true, else fused (/scan_fused; auto and fused start '
                  'my_bringup scan_fusion)',
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
   lidar = LaunchConfiguration('lidar')
   lidar_arg = DeclareLaunchArgument(
      'lidar', default_value='true', choices=['true', 'false'],
      description='every mode except sim: run the RPLidar C1 driver (/scan), hot-plug watchdog and /scan_filtered',
   )
   realsense = LaunchConfiguration('realsense')
   realsense_arg = DeclareLaunchArgument(
      'realsense', default_value='true', choices=['true', 'false'],
      description='every mode except sim: run the Intel RealSense depth camera driver (/camera/color, /camera/depth)',
   )
   is_teleop = IfCondition(PythonExpression(["'", mode, "' == 'teleop'"]))
   # sensors run in both non-teleop modes, except when the sim stands in for them
   is_sensors = IfCondition(PythonExpression(
      ["'", mode, "' == 'autonomous' or ('", mode, "' == 'autonomy' and '", sim, "' != 'true')"]))
   is_autonomy = IfCondition(PythonExpression(["'", mode, "' == 'autonomy'"]))
   is_autonomy_real = IfCondition(PythonExpression(["'", mode, "' == 'autonomy' and '", sim, "' != 'true'"]))
   is_autonomy_sim = IfCondition(PythonExpression(["'", mode, "' == 'autonomy' and '", sim, "' == 'true'"]))
   is_nav2 = IfCondition(PythonExpression(["'", mode, "' == 'autonomy' and '", nav2, "' == 'true'"]))
   # the robot's own model and lidar, i.e. everything except autonomy + sim (Gazebo provides both)
   not_sim = IfCondition(PythonExpression(["not ('", mode, "' == 'autonomy' and '", sim, "' == 'true')"]))

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

   # --- robot description (TF) ---------------------------------------------
   # Every mode: robot_state_publisher for base_link -> base_footprint /
   # lidar_link / imu_link / gps_link. Not with sim:=true, where simulation.launch.py
   # runs the sim model's own. Guarded like ublox so a Pi without
   # robot_description built still launches.
   try:
      _desc_dir = get_package_share_directory('robot_description')
      robot_description_actions = [IncludeLaunchDescription(
         PythonLaunchDescriptionSource(
            os.path.join(_desc_dir, 'launch', 'description.launch.py')),
         condition=not_sim,
      )]
   except PackageNotFoundError:
      robot_description_actions = []
      print('[master_launch] robot_description not found; no robot TF (lidar_link/imu_link)')
   # --- end robot description (TF) -----------------------------------------

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
      # motor board's udev symlink (deployment/udev/99-motor.rules); never a raw /dev/ttyUSBn
      parameters=[{'device': '/dev/motor'}],
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
             # nav2: the controller slows and stops for the costmap only when Nav2 publishes one
             launch_arguments={'sim': sim, 'nav2': nav2,
                               'obstacle_guard': LaunchConfiguration('obstacle_guard')}.items(),
             condition=is_autonomy,
         ))
      except PackageNotFoundError:
         print('[master_launch] autonomy package not found (submodule ros2_ws/src/delivery-autonomy not '
               'checked out or not built); autonomy mode is joystick-through-bridge only')

   # --- Nav2 local planning (autonomy + nav2:=true) ------------------------------
   # nav2_local.launch.py guards itself: missing Nav2 packages (the Lyrical sim shell has none),
   # no boundary map, or no shared_types each log and skip instead of aborting this launch. The
   # only guard needed here is for a stale hardware_bringup install without the file.
   # drive:=true: local_planner sends its paths to /follow_path, the MPPI's /cmd_vel is already
   # /cmd_vel_nav (autonomy_real.launch.py), so the joystick still overrides through twist_mux.
   nav2_actions = []
   _nav2_launch = os.path.join(hw_dir or '', 'launch', 'nav2_local.launch.py')
   if hw_dir is not None and os.path.isfile(_nav2_launch):
      nav2_actions.append(IncludeLaunchDescription(
         PythonLaunchDescriptionSource(_nav2_launch),
         launch_arguments={'sim': sim, 'sources': nav2_sources, 'drive': 'true'}.items(),
         condition=is_nav2,
      ))
   else:
      nav2_actions.append(LogInfo(
         msg='[master_launch] WARNING: hardware_bringup nav2_local.launch.py not installed (rebuild '
             'hardware_bringup); nav2:=true has no Nav2',
         condition=is_nav2,
      ))

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

   # --- lidar (RPLidar C1) ---------------------------------------------------
   # sllidar_ros2 driver on /dev/rplidar -> /scan (frame lidar_link); params in
   # config/lidar.yaml. Runs in BOTH modes (like the robot description) so the
   # Pi publishes /scan whenever the C1 is plugged in; `lidar:=false` disables
   # the driver, watchdog and filter. /scan + TF base_link->lidar_link is what a
   # Nav2 costmap obstacle layer consumes (see config/costmap_lidar.example.yaml).
   # Hot-plug: sllidar_node exits when the port is missing at startup, so
   # respawn keeps retrying every ~3 s until the lidar is plugged in. It does
   # NOT exit when unplugged mid-run (spins on read timeouts), so
   # lidar_watchdog SIGINTs it once /scan is silent for 5 s and respawn then
   # reopens the re-plugged device. Guarded like ublox_dgnss so launch still
   # works where the driver isn't built. Off with sim:=true: Gazebo publishes /scan.
   use_lidar = IfCondition(PythonExpression(
      ["'", lidar, "' == 'true' and not ('", mode, "' == 'autonomy' and '", sim, "' == 'true')"]))
   lidar_nodes = []
   lidar_params = os.path.join(get_package_share_directory('my_bringup'), 'config', 'lidar.yaml')
   try:
      get_package_share_directory('sllidar_ros2')
      lidar_available = True
   except PackageNotFoundError:
      lidar_available = False
      print('[master_launch] sllidar_ros2 not found; no lidar (/scan)')

   lidar_node = Node(
      package='sllidar_ros2',
      executable='sllidar_node',
      name='sllidar_node',
      respawn=True,
      respawn_delay=3.0,
      output='screen',
      parameters=[lidar_params],
      remappings=[('scan', '/scan')],
      condition=use_lidar,
   )

   # Same stale-build guard as heartbeat_node above.
   try:
      from ament_index_python.packages import get_package_prefix
      _wd_exe = os.path.join(get_package_prefix('my_bringup'), 'lib', 'my_bringup', 'lidar_watchdog')
      watchdog_available = os.path.exists(_wd_exe)
   except Exception:  # noqa: BLE001
      watchdog_available = False

   lidar_watchdog = Node(
      package='my_bringup',
      executable='lidar_watchdog',
      name='lidar_watchdog',
      respawn=True,
      respawn_delay=3.0,
      output='screen',
      parameters=[lidar_params],
      condition=use_lidar,
   )

   if lidar_available:
      lidar_nodes += [lidar_node]
      if watchdog_available:
         lidar_nodes += [lidar_watchdog]
      else:
         print('[master_launch] lidar_watchdog executable not found (stale build?); lidar will not recover from a mid-run unplug')

   # --- lidar self-hit filter: /scan -> /scan_filtered (TDM-16) -----------
   # Needs: sudo apt install ros-jazzy-laser-filters
   # Guarded so the launch still works where it isn't installed.
   # Follows the lidar: both modes, off with lidar:=false.
   try:
      get_package_share_directory('laser_filters')
      lidar_nodes.append(Node(
         package='laser_filters',
         executable='scan_to_scan_filter_chain',
         # No name=: it remaps EVERY node in the process, and the box filter
         # runs its own internal node, so both would be called
         # scan_to_scan_filter_chain (breaks param services). The default
         # name already matches the key in laser_filters.yaml.
         respawn=True,
         respawn_delay=3.0,
         output='screen',
         parameters=[os.path.join(get_package_share_directory('my_bringup'),
                                  'config', 'laser_filters.yaml')],
         remappings=[('scan', '/scan'), ('scan_filtered', '/scan_filtered')],
         condition=use_lidar,
      ))
   except PackageNotFoundError:
      print('[master_launch] laser_filters not found; no /scan_filtered '
            '(sudo apt install ros-jazzy-laser-filters)')

   # --- depth camera (Intel RealSense D4xx) ----------------------------------
   # Intel's realsense2_camera driver (apt ros-jazzy-realsense2-camera); params
   # in config/realsense.yaml. Namespace '' + name 'camera' gives
   # /camera/color/image_raw and /camera/depth/image_rect_raw (not the driver's
   # default /camera/camera/...). Both modes, off with realsense:=false.
   # Hot-plug is handled by the driver itself (wait_for_device_timeout /
   # reconnect_timeout); respawn only covers a crash. Guarded like sllidar_ros2
   # so launch still works where the driver isn't installed. Off with sim:=true
   # (the Gazebo robot has its own camera topics).
   use_realsense = IfCondition(PythonExpression(
      ["'", realsense, "' == 'true' and not ('", mode, "' == 'autonomy' and '", sim, "' == 'true')"]))
   realsense_nodes = []
   try:
      get_package_share_directory('realsense2_camera')
      realsense_nodes.append(Node(
         package='realsense2_camera',
         executable='realsense2_camera_node',
         namespace='',
         name='camera',
         respawn=True,
         respawn_delay=3.0,
         output='screen',
         emulate_tty=True,
         parameters=[os.path.join(get_package_share_directory('my_bringup'),
                                  'config', 'realsense.yaml')],
         condition=use_realsense,
      ))
   except PackageNotFoundError:
      print('[master_launch] realsense2_camera not found; no depth camera '
            '(sudo apt install ros-jazzy-realsense2-camera)')

   # depth -> /camera/obstacles (PointCloud2) + /camera/scan (LaserScan) for
   # the costmap, floor removed (my_bringup/depth_obstacles.py). Same
   # stale-build guard as heartbeat_node; it idles until depth + TF arrive.
   try:
      from ament_index_python.packages import get_package_prefix
      _do_exe = os.path.join(get_package_prefix('my_bringup'), 'lib', 'my_bringup', 'depth_obstacles')
      depth_obstacles_available = os.path.exists(_do_exe)
   except Exception:  # noqa: BLE001
      depth_obstacles_available = False
   if realsense_nodes and depth_obstacles_available:
      realsense_nodes.append(Node(
         package='my_bringup',
         executable='depth_obstacles',
         name='depth_obstacles',
         respawn=True,
         respawn_delay=3.0,
         output='screen',
         parameters=[os.path.join(get_package_share_directory('my_bringup'),
                                  'config', 'realsense.yaml')],
         condition=use_realsense,
      ))
   elif realsense_nodes:
      print('[master_launch] depth_obstacles executable not found (stale build?); '
            'no /camera/scan or /camera/obstacles')

   return LaunchDescription([
        mode_arg,
        sim_arg,
        nav2_arg,
        nav2_sources_arg,
        robot_id_arg,
        api_url_arg,
        lidar_arg,
        realsense_arg,
        _obstacle_guard_arg(),
        *([heartbeat_node] if heartbeat_available else []),
        *robot_description_actions,  # robot description (TF)
        joy_node, 
        control_node, 
        *sensor_nodes,
        *autonomy_nodes,
        *nav2_actions,
        *_scan_fusion_actions(mode, sim, nav2, nav2_sources),
        *sim_nodes,
        *lidar_nodes,
        *realsense_nodes,
    ])
