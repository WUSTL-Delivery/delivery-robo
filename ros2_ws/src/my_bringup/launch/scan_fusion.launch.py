"""Run fusion next to master_launch.py: /scan_filtered + /camera/scan -> /scan_fused.

For a stationary check:
    ros2 launch my_bringup scan_fusion.launch.py
With odometry TF available:
    ros2 launch my_bringup scan_fusion.launch.py fixed_frame:=odom
For bags/simulation, also pass use_sim_time:=true and remap sensor topics.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Configure fusion without launching duplicate sensor drivers."""
    share = get_package_share_directory('my_bringup')
    defaults = {
        'params_file': os.path.join(share, 'config', 'scan_fusion.yaml'),
        'lidar_topic': '/scan_filtered',
        'camera_topic': '/camera/scan',
        'output_topic': '/scan_fused',
        'fixed_frame': '',
        'use_sim_time': 'false',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        Node(
            package='my_bringup', executable='scan_fusion', name='scan_fusion', output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                **{name: ParameterValue(LaunchConfiguration(name), value_type=str)
                   for name in ('lidar_topic', 'camera_topic', 'output_topic', 'fixed_frame')},
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
        ),
    ])
