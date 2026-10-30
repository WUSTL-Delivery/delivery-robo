"""delivery-autonomy on the real robot: gps_datum_tf + state_estimation + planner + controller.

Plan stages 2 and 4. Starts the three autonomy executables directly with hardware
parameters instead of autonomy.launch.py (whose map->odom is a fixed Gazebo pose), so
delivery-autonomy itself is never edited:

* state_estimation reads the gated GPS on /gps/fix, /imu/data, the bridge's /odom, and the
  command the bridge actually executed (/cmd_vel_executed) as its prediction input;
* the controller's hardcoded /cmd_vel publisher is remapped to /cmd_vel_nav for twist_mux;
* the controller takes its speeds and stop distance from hardware.yaml (controller_server), and
  use_costmap from nav2:= (master_launch passes its own nav2), so it slows and stops for
  obstacles in Nav2's costmap only when Nav2 runs. Without a costmap it would otherwise crawl at
  crawl_speed (0.3 m/s) waiting for one;
* global_planner runs with cwd = planner_cache_dir so its osmnx Overpass cache (./cache)
  survives reboots; robot.service sets no WorkingDirectory;
* obstacle_guard (TDM-52, robot only) locks twist_mux's navigation input while neither the lidar
  (/scan_filtered) nor the depth camera (/camera/scan) publishes: the user's rule is that the
  robot drives on with one of them missing and stops driving autonomously with both missing
  (the joystick still overrides). It runs here, beside the controller whose /cmd_vel_nav it
  gates, so whatever starts the controller on the robot also starts the guard; twist_mux's lock
  (config/twist_mux.yaml) blocks navigation whenever the guard is not running. lidar:=false or
  realsense:=false in master_launch do not change its inputs: a sensor switched off counts as
  missing, so both off means no autonomous driving. obstacle_guard:=false starts it disabled
  (never locks, logs an error every 30 s) for bench work without sensors. Not with sim:=true:
  the sim has no twist_mux.

Needs drive.launch.py (or the whole master_launch mode:=autonomy) for /odom and motion, and
the IMU + GPS drivers for sensor data.

sim:=true runs the same nodes against the delivery-autonomy Gazebo sim
(master_launch mode:=autonomy sim:=true): sim time, Gazebo's own /imu/data and /odom, the
controller straight on the sim's /cmd_vel (no twist_mux), and gps_datum_tf in its local_enu
mode. It gates Gazebo's /gps/fix onto /gps/fix_gated for state_estimation and publishes
map->odom from the first fix it forwards, as the Gazebo world position of that fix (the sim's
map frame is the world frame). The robot settles 0.37 m from its spawn request after the
drop. A fixed map->odom (autonomy.launch.py: the settled pose since delivery-autonomy TDM-39)
is off by however far the robot is from that pose at the EKF's first fix. Taking both from the
same fix removes that race. The controller gets hardware.yaml's controller_server values here
too, so the sim drives with the robot's speeds and stop distance. Not with the robot's vehicle,
though: hardware.yaml's /** wheelbase and max_steer_rad describe the real robot, and the sim
controller gets the Gazebo car's (SIM_VEHICLE) instead.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# delivery-autonomy src/simulation/worlds/washu.world <spherical_coordinates> (heading 0):
# latitude, longitude, elevation. gps_datum_tf inverts Gazebo's NavSat sensor with it.
SIM_WORLD_ORIGIN = (38.64817527, -90.30128152, 0.0)
SIM_GATED_GPS_TOPIC = '/gps/fix_gated'
# The Gazebo car the controller models with sim:=true: delivery-autonomy's SIM_VEHICLE (its
# controller defaults; robot.urdf.xacro <wheel_base> 0.50, steering joints +-0.6 rad).
# hardware.yaml's /** wheelbase and max_steer_rad are the real robot's and would otherwise reach
# the sim controller.
SIM_VEHICLE = {'wheelbase': 0.5, 'max_steer_rad': 0.6, 'steer_tau': 0.545, 'drive_tau': 0.01,
               'max_accel': 5.0}


def _nodes(context):
    hardware_yaml = LaunchConfiguration('hardware_params').perform(context)
    gps_in = LaunchConfiguration('gps_in_topic').perform(context)
    sim = LaunchConfiguration('sim').perform(context) == 'true'
    nav2 = LaunchConfiguration('nav2').perform(context) == 'true'
    ekf_cmd_vel = (LaunchConfiguration('ekf_cmd_vel_topic').perform(context)
                   or ('/cmd_vel' if sim else '/cmd_vel_executed'))
    cache_dir = os.path.expanduser(LaunchConfiguration('planner_cache_dir').perform(context))
    os.makedirs(cache_dir, exist_ok=True)
    real_time = {'use_sim_time': sim}

    if sim:
        # Not hardware.yaml (robot topics, UTM): gate the bridged /gps/fix and place odom at the
        # Gazebo world position of the EKF's first fix.
        ekf_gps = SIM_GATED_GPS_TOPIC
        datum_params = [{
            **real_time,
            'gps_in_topic': gps_in or '/gps/fix',
            'gps_out_topic': ekf_gps,
            'projection': 'local_enu',
            'enu_origin_lat': SIM_WORLD_ORIGIN[0],
            'enu_origin_lon': SIM_WORLD_ORIGIN[1],
            'enu_origin_alt': SIM_WORLD_ORIGIN[2],
        }]
    else:
        ekf_gps = '/gps/fix'          # hardware.yaml gps_datum_tf.gps_out_topic
        datum_params = [hardware_yaml]
        if gps_in:
            datum_params.append({'gps_in_topic': gps_in})
    datum = Node(
        package='hardware_bringup',
        executable='gps_datum_tf',
        name='gps_datum_tf',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=datum_params,
    )
    state_estimation = Node(
        package='autonomy',
        executable='state_estimation',
        name='state_estimation',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            **real_time,
            'imu_topic': '/imu/data',
            'odom_topic': '/odom',
            'gps_topic': ekf_gps,
            'cmd_vel_topic': ekf_cmd_vel,
        }],
    )
    global_planner = Node(
        package='autonomy',
        executable='global_planner',
        name='global_planning_server',
        output='screen',
        respawn=True,
        respawn_delay=10.0,          # it dies if Overpass is unreachable; give the network time
        cwd=cache_dir,
        parameters=[real_time],
    )
    # hardware.yaml's controller_server section (speeds, stop distance) in both modes, so the sim
    # drives with the robot's values. use_costmap follows nav2: without Nav2 there is no costmap,
    # and the controller would cap itself at crawl_speed waiting for one. The vehicle model is the
    # exception: the robot's comes from hardware.yaml's /**, the sim car keeps its own.
    controller = Node(
        package='autonomy',
        executable='controller',
        name='controller_server',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=[hardware_yaml,
                    {**real_time, 'use_costmap': nav2, **(SIM_VEHICLE if sim else {})}],
        remappings=[] if sim else [('/cmd_vel', '/cmd_vel_nav')],
    )
    nodes = [datum, state_estimation, global_planner, controller]
    if not sim:
        guard = LaunchConfiguration('obstacle_guard', default='true').perform(context) == 'true'
        nodes.append(Node(
            package='hardware_bringup',
            executable='obstacle_guard',
            name='obstacle_guard',
            output='screen',
            respawn=True,
            respawn_delay=1.0,       # navigation stays blocked (lock timeout) until it is back
            parameters=[{'enabled': guard}],
        ))
    return nodes


def generate_launch_description():
    share = get_package_share_directory('hardware_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware_params', default_value=os.path.join(share, 'config', 'hardware.yaml')),
        DeclareLaunchArgument(
            'gps_in_topic', default_value='',
            description='NavSatFix topic published by the GPS driver; overrides hardware.yaml when set '
                        "(empty with sim:=true = the Gazebo bridge's /gps/fix)"),
        DeclareLaunchArgument(
            'ekf_cmd_vel_topic', default_value='',
            description="state_estimation's prediction input; empty = /cmd_vel_executed (/cmd_vel with "
                        "sim:=true). /cmd_vel_out to use twist_mux's output instead"),
        DeclareLaunchArgument(
            'sim', default_value='false', choices=['true', 'false'],
            description='run against the delivery-autonomy Gazebo sim instead of the robot'),
        DeclareLaunchArgument(
            'nav2', default_value='false', choices=['true', 'false'],
            description="Nav2 runs and publishes /global_costmap/costmap (master_launch passes its own "
                        "nav2). Sets the controller's use_costmap: slow down and stop for costmap "
                        "obstacles. false = the controller ignores the costmap"),
        DeclareLaunchArgument(
            'planner_cache_dir', default_value='~/.cache/delivery_autonomy',
            description='working directory of global_planner, holds its osmnx ./cache'),
        DeclareLaunchArgument(
            'obstacle_guard', default_value='true', choices=['true', 'false'],
            description='robot only: block navigation in twist_mux while neither the lidar nor '
                        'the depth camera publishes; false = run it disabled (bench work)'),
        OpaqueFunction(function=_nodes),
    ])
