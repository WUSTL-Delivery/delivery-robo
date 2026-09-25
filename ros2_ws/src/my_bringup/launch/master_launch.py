import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

# Selected by deployment/robot_config.yaml (via startup.sh), or by hand:
#   ros2 launch my_bringup master_launch.py mode:=teleop
#   ros2 launch my_bringup master_launch.py mode:=autonomous

def generate_launch_description():
   mode = LaunchConfiguration('mode')
   mode_arg = DeclareLaunchArgument(
      'mode',
      default_value='teleop',
      choices=['teleop', 'autonomous'],
      description='teleop = wired joystick control; autonomous = sensor stack (GPS/NTRIP/IMU)',
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
      description='autonomous mode only: run the RPLidar C1 driver (+ hot-plug watchdog)',
   )
   is_teleop = IfCondition(PythonExpression(["'", mode, "' == 'teleop'"]))
   is_autonomous = IfCondition(PythonExpression(["'", mode, "' == 'autonomous'"]))

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
   # Both modes: robot_state_publisher for base_footprint -> base_link ->
   # lidar_link / imu_link / gps_link. Guarded like ublox so a Pi without
   # robot_description built still launches.
   try:
      _desc_dir = get_package_share_directory('robot_description')
      robot_description_actions = [IncludeLaunchDescription(
         PythonLaunchDescriptionSource(
            os.path.join(_desc_dir, 'launch', 'description.launch.py')),
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
      condition=is_teleop,
   )

   # --- autonomous: sensor stack ------------------------------------------
   # Resolved at load time, so guard it: teleop must still work on a Pi
   # where ublox_dgnss isn't built.
   try:
      ublox_dir = get_package_share_directory('ublox_dgnss')
   except PackageNotFoundError:
      ublox_dir = None
      print('[master_launch] ublox_dgnss not found; autonomous mode has no GPS/NTRIP nodes')

   gps_main_node = IncludeLaunchDescription(
       PythonLaunchDescriptionSource(
           os.path.join(ublox_dir or '', 'launch', 'ublox_rover_hpposllh_navsatfix.launch.py')
       ),
       condition=is_autonomous,
   )

   gps_ntrip_node = IncludeLaunchDescription(
       PythonLaunchDescriptionSource(
           os.path.join(ublox_dir or '', 'launch', 'ntrip_client.launch.py')
       ),
       launch_arguments={
           'use_https': 'false',
           'host':'168.166.125.30',
           'port': '2101',
           'mountpoint':'RTX_RTCM34',
           'username':'/WashUroboticsDelivery2026',
           'password':'$DeliveryWU197?'
       }.items(),
       condition=is_autonomous,
   )

   imu_node = Node(
       package='imu_package',
       executable='imu_publisher',
       name='imu_publisher',
       respawn=True,
       respawn_delay=3.0,
       output='screen',
       condition=is_autonomous,
   )

   autonomous_nodes = [imu_node]
   if ublox_dir is not None:
      autonomous_nodes += [gps_main_node, gps_ntrip_node]

   # --- lidar (RPLidar C1) ---------------------------------------------------
   # sllidar_ros2 driver on /dev/rplidar -> /scan (frame lidar_link); params in
   # config/lidar.yaml. Autonomous mode only, and `lidar:=false` disables it.
   # Hot-plug: sllidar_node exits when the port is missing at startup, so
   # respawn keeps retrying every ~3 s until the lidar is plugged in. It does
   # NOT exit when unplugged mid-run (spins on read timeouts), so
   # lidar_watchdog SIGINTs it once /scan is silent for 5 s and respawn then
   # reopens the re-plugged device. Guarded like ublox_dgnss so launch still
   # works where the driver isn't built.
   use_lidar = IfCondition(PythonExpression(
      ["'", mode, "' == 'autonomous' and '", lidar, "' == 'true'"]))
   lidar_params = os.path.join(get_package_share_directory('my_bringup'), 'config', 'lidar.yaml')
   try:
      get_package_share_directory('sllidar_ros2')
      lidar_available = True
   except PackageNotFoundError:
      lidar_available = False
      print('[master_launch] sllidar_ros2 not found; autonomous mode has no lidar (/scan)')

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
      autonomous_nodes += [lidar_node]
      if watchdog_available:
         autonomous_nodes += [lidar_watchdog]
      else:
         print('[master_launch] lidar_watchdog executable not found (stale build?); lidar will not recover from a mid-run unplug')

   # --- lidar self-hit filter: /scan -> /scan_filtered (TDM-16) -----------
   # Needs: sudo apt install ros-jazzy-laser-filters
   # Guarded so autonomous mode still launches where it isn't installed.
   # Follows the lidar: only when mode=autonomous and lidar:=true.
   try:
      get_package_share_directory('laser_filters')
      autonomous_nodes.append(Node(
         package='laser_filters',
         executable='scan_to_scan_filter_chain',
         name='scan_to_scan_filter_chain',
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

   return LaunchDescription([
        mode_arg,
        robot_id_arg,
        api_url_arg,
        lidar_arg,
        *([heartbeat_node] if heartbeat_available else []),
        *robot_description_actions,  # robot description (TF)
        joy_node, 
        control_node, 
        *autonomous_nodes
    ])
