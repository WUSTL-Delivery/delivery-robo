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
    ekf_cmd_vel = LaunchConfiguration('ekf_cmd_vel_topic').perform(context)
    cache_dir = os.path.expanduser(LaunchConfiguration('planner_cache_dir').perform(context))
    os.makedirs(cache_dir, exist_ok=True)
    real_time = {'use_sim_time': False}

    datum_params = [hardware_yaml]
    if gps_in:
        datum_params.append({'gps_in_topic': gps_in})
    gps_datum_tf = Node(
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
        remappings=[('/cmd_vel', '/cmd_vel_nav')],
    )
    return [gps_datum_tf, state_estimation, global_planner, controller]


def generate_launch_description():
    share = get_package_share_directory('hardware_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware_params', default_value=os.path.join(share, 'config', 'hardware.yaml')),
        DeclareLaunchArgument(
            'gps_in_topic', default_value='',
            description='NavSatFix topic published by the GPS driver; overrides hardware.yaml when set'),
        DeclareLaunchArgument(
            'ekf_cmd_vel_topic', default_value='/cmd_vel_executed',
            description="state_estimation's prediction input; /cmd_vel_out to use twist_mux's output instead"),
        DeclareLaunchArgument(
            'planner_cache_dir', default_value='~/.cache/delivery_autonomy',
            description='working directory of global_planner, holds its osmnx ./cache'),
        OpaqueFunction(function=_nodes),
    ])
