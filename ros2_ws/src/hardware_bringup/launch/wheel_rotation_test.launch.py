"""Standalone powered bench trial. No real joystick or autonomy launch."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def value(name, kind):
    return ParameterValue(LaunchConfiguration(name), value_type=kind)


def generate_launch_description():
    share = get_package_share_directory('hardware_bringup')
    hardware = LaunchConfiguration('hardware_params')
    return LaunchDescription([
        DeclareLaunchArgument('ticks_per_rotation',
                              description='Measured counts per WHEEL revolution'),
        DeclareLaunchArgument('port', description='Arduino /dev/serial/by-id/... path'),
        DeclareLaunchArgument('throttle', default_value='0.1'),
        DeclareLaunchArgument('encoder_direction', default_value='1'),
        DeclareLaunchArgument('max_run_seconds', default_value='10.0',
                              description='Trial stops after this long even if the count '
                                          'never reaches the target'),
        DeclareLaunchArgument('hardware_params',
                              default_value=os.path.join(share, 'config', 'hardware.yaml')),
        Node(package='hardware_bringup', executable='wheel_rotation_test', output='screen',
             parameters=[{'ticks_per_rotation': value('ticks_per_rotation', int),
                          'encoder_direction': value('encoder_direction', int),
                          'throttle': value('throttle', float),
                          'max_run_seconds': value('max_run_seconds', float)}]),
        # A private, explicitly bounded latch allows one Joy message to drive a trial.
        # All-zero Joy is a real zero command because enable-button gating is OFF
        # here only. No changes are made to the normal autonomy/manual configuration.
        Node(package='hardware_bringup', executable='joy_to_cmdvel', name='rotation_test_mapper',
             output='screen', parameters=[hardware, {
                 'joy_topic': '/encoder_test/joy', 'output_topic': '/encoder_test/cmd_vel_joy',
                 'axis_throttle': 1, 'axis_steer': 0,
                 'invert_throttle': False, 'invert_steer': False,
                 'max_speed': 1.5, 'require_enable_button': False,
                 'require_joystick': True, 'joy_timeout': value('max_run_seconds', float),
             }]),
        Node(package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
             parameters=[os.path.join(share, 'config', 'twist_mux.yaml')],
             remappings=[('cmd_vel_joy', '/encoder_test/cmd_vel_joy'),
                         ('cmd_vel_nav', '/encoder_test/unused_navigation'),
                         ('cmd_vel_out', '/encoder_test/cmd_vel_out')]),
        Node(package='hardware_bringup', executable='arduino_bridge', name='arduino_bridge',
             output='screen', parameters=[hardware, {
                 'port': value('port', str), 'cmd_vel_topic': '/encoder_test/cmd_vel_out',
                 'executed_topic': '/encoder_test/cmd_vel_executed',
                 'odom_topic': '/encoder_test/odom', 'ticks_topic': '/wheel_ticks',
             }],
             # hardware.yaml's arduino_bridge section beats the overrides above, so remap.
             remappings=[('/cmd_vel_out', '/encoder_test/cmd_vel_out'),
                         ('/cmd_vel_executed', '/encoder_test/cmd_vel_executed'),
                         ('/odom', '/encoder_test/odom')]),
    ])
