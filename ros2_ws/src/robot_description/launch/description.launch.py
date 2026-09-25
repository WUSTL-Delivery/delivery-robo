"""robot_state_publisher for the delivery robot (TF for the sensor frames).

    ros2 launch robot_description description.launch.py

Publishes the fixed TF chain base_footprint -> base_link -> lidar_link /
imu_link / gps_link / camera_link from urdf/robot.urdf.xacro.

The xacro is processed at launch time, so editing the installed file (or the
source file with --symlink-install) and relaunching is enough to change a
sensor offset.

Degrades gracefully:
  * xacro missing (neither the python module nor the `xacro` executable):
    logs an error and starts nothing, instead of aborting the parent launch.
  * joint_state_publisher missing: robot_state_publisher still runs; the
    fixed sensor frames are published, but the wheel/steering joints have no
    TF until something publishes /joint_states.
"""
import os
import shutil

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _process_xacro(path):
    """Return the URDF string, or None if xacro is not available."""
    try:
        import xacro  # noqa: PLC0415 - optional dependency
        return xacro.process_file(path).toxml()
    except ImportError:
        pass
    exe = shutil.which('xacro')
    if exe:
        import subprocess  # noqa: PLC0415
        return subprocess.check_output([exe, path], text=True)
    return None


def _launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1')
    xacro_path = os.path.join(
        get_package_share_directory('robot_description'), 'urdf', 'robot.urdf.xacro')

    try:
        robot_description = _process_xacro(xacro_path)
    except Exception as e:  # noqa: BLE001 - a bad model must not kill the parent launch
        return [LogInfo(msg=f'[robot_description] xacro failed on {xacro_path}: {e}; '
                            'robot_state_publisher NOT started (no TF)')]
    if robot_description is None:
        return [LogInfo(msg='[robot_description] xacro not installed '
                            '(sudo apt install ros-jazzy-xacro); '
                            'robot_state_publisher NOT started (no TF)')]

    actions = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            respawn=True,
            respawn_delay=3.0,
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),
    ]

    # Zero joint states for the wheel/steering joints, only if installed.
    try:
        get_package_share_directory('joint_state_publisher')
        actions.append(Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ))
    except PackageNotFoundError:
        actions.append(LogInfo(msg='[robot_description] joint_state_publisher not installed; '
                                   'wheel/steering joints have no TF until /joint_states is '
                                   'published (sensor frames are fixed and unaffected)'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_launch_setup),
    ])
