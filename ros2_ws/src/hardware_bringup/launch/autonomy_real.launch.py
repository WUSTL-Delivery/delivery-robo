"""delivery-autonomy on the real robot: gps_datum_tf + state_estimation + planner + controller.

Plan stages 2 and 4. Starts the three autonomy executables directly with hardware
parameters instead of autonomy.launch.py (whose map->odom is the Gazebo spawn pose), so
delivery-autonomy itself is never edited:

* state_estimation reads the gated GPS on /gps/fix, /imu/data, the bridge's /odom, and the
  command the bridge actually executed (/cmd_vel_executed) as its prediction input;
* the controller's hardcoded /cmd_vel publisher is remapped to /cmd_vel_nav for twist_mux;
* global_planner runs with cwd = planner_cache_dir so its osmnx Overpass cache (./cache)
  survives reboots; robot.service sets no WorkingDirectory.

Needs drive.launch.py (or the whole master_launch mode:=autonomy) for /odom and motion, and
the IMU + GPS drivers for sensor data.

sim:=true runs the same three autonomy nodes against the delivery-autonomy Gazebo sim
(master_launch mode:=autonomy sim:=true): sim time, Gazebo's own /gps/fix, /imu/data and /odom,
the controller straight on the sim's /cmd_vel (no twist_mux), and autonomy.launch.py's fixed
map->odom at the spawn pose instead of gps_datum_tf.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    hardware_yaml = LaunchConfiguration('hardware_params').perform(context)
    gps_in = LaunchConfiguration('gps_in_topic').perform(context)
    sim = LaunchConfiguration('sim').perform(context) == 'true'
    ekf_cmd_vel = (LaunchConfiguration('ekf_cmd_vel_topic').perform(context)
                   or ('/cmd_vel' if sim else '/cmd_vel_executed'))
    cache_dir = os.path.expanduser(LaunchConfiguration('planner_cache_dir').perform(context))
    os.makedirs(cache_dir, exist_ok=True)
    real_time = {'use_sim_time': sim}

    if sim:
        # autonomy.launch.py's calibration: the odom origin is the spawn pose in
        # delivery-autonomy's run_simulator.sh. Keep the two in sync.
        datum = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_broadcaster',
            arguments=['--x', '-95.0', '--y', '25.5', '--z', '2', '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'map', '--child-frame-id', 'odom'],
            parameters=[real_time],
        )
    else:
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
            'gps_topic': '/gps/fix',
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
    controller = Node(
        package='autonomy',
        executable='controller',
        name='controller_server',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=[real_time],
        remappings=[] if sim else [('/cmd_vel', '/cmd_vel_nav')],
    )
    return [datum, state_estimation, global_planner, controller]


def generate_launch_description():
    share = get_package_share_directory('hardware_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware_params', default_value=os.path.join(share, 'config', 'hardware.yaml')),
        DeclareLaunchArgument(
            'gps_in_topic', default_value='',
            description='NavSatFix topic published by the GPS driver; overrides hardware.yaml when set'),
        DeclareLaunchArgument(
            'ekf_cmd_vel_topic', default_value='',
            description="state_estimation's prediction input; empty = /cmd_vel_executed (/cmd_vel with "
                        "sim:=true). /cmd_vel_out to use twist_mux's output instead"),
        DeclareLaunchArgument(
            'sim', default_value='false', choices=['true', 'false'],
            description='run against the delivery-autonomy Gazebo sim instead of the robot'),
        DeclareLaunchArgument(
            'planner_cache_dir', default_value='~/.cache/delivery_autonomy',
            description='working directory of global_planner, holds its osmnx ./cache'),
        OpaqueFunction(function=_nodes),
    ])
