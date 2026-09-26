"""Drive chain only: joystick -> twist_mux -> arduino_bridge -> Arduino.

Plan stage 1. Runs on its own for bench tests (wheels off the ground):

    ros2 launch hardware_bringup drive.launch.py [port:=/dev/serial/by-id/usb-...]
    ros2 topic pub -r 10 /cmd_vel_out geometry_msgs/msg/Twist '{linear: {x: 0.3}}'   # Ctrl-C -> watchdog stop
    ros2 topic echo /odom

master_launch.py includes it in mode:=autonomy together with autonomy_real.launch.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bridge(context):
    hardware_yaml = LaunchConfiguration('hardware_params').perform(context)
    port = LaunchConfiguration('port').perform(context)
    params = [hardware_yaml]
    if port:
        params.append({'port': port})      # an explicit port:= beats the YAML; empty keeps the YAML
    return [Node(
        package='hardware_bringup',
        executable='arduino_bridge',
        name='arduino_bridge',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=params,
    )]


def generate_launch_description():
    share = get_package_share_directory('hardware_bringup')
    hardware_yaml = LaunchConfiguration('hardware_params')
    twist_mux_yaml = LaunchConfiguration('twist_mux_params')

    args = [
        DeclareLaunchArgument(
            'hardware_params', default_value=os.path.join(share, 'config', 'hardware.yaml'),
            description='calibration + node parameters (config/hardware.yaml)'),
        DeclareLaunchArgument(
            'twist_mux_params', default_value=os.path.join(share, 'config', 'twist_mux.yaml')),
        DeclareLaunchArgument(
            'port', default_value='',
            description='Arduino serial port; overrides hardware.yaml when set'),
    ]

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        # autorepeat keeps /joy flowing while the stick is idle, which joy_to_cmdvel's
        # joystick-present check and twist_mux's 0.5 s timeout both rely on
        parameters=[{'deadzone': 0.1, 'autorepeat_rate': 20.0}],
    )
    joy_to_cmdvel = Node(
        package='hardware_bringup',
        executable='joy_to_cmdvel',
        name='joy_to_cmdvel',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=[hardware_yaml],
    )
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=[twist_mux_yaml],
    )

    return LaunchDescription(args + [joy_node, joy_to_cmdvel, twist_mux, OpaqueFunction(function=_bridge)])
