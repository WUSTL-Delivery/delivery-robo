"""Preview gamepad mapping on isolated topics; does not launch motor hardware."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device_id', default_value='0'),
        DeclareLaunchArgument('enable_button', default_value='4'),
        DeclareLaunchArgument('axis_throttle', default_value='1'),
        DeclareLaunchArgument('axis_steer', default_value='0'),
        Node(package='joy', executable='joy_node', name='encoder_test_joy',
             output='screen', parameters=[{
                 'device_id': ParameterValue(LaunchConfiguration('device_id'), value_type=int),
                 'autorepeat_rate': 20.0, 'deadzone': .1,
             }], remappings=[('joy', '/encoder_test/joy')]),
        Node(package='hardware_bringup', executable='joy_to_cmdvel',
             name='encoder_test_mapping', output='screen', parameters=[{
                 'joy_topic': '/encoder_test/joy',
                 'output_topic': '/encoder_test/cmd_vel_preview',
                 'max_speed': .2,
                 'require_enable_button': True, 'require_joystick': True,
                 'enable_button': ParameterValue(
                     LaunchConfiguration('enable_button'), value_type=int),
                 'axis_throttle': ParameterValue(
                     LaunchConfiguration('axis_throttle'), value_type=int),
                 'axis_steer': ParameterValue(LaunchConfiguration('axis_steer'), value_type=int),
             }]),
    ])
