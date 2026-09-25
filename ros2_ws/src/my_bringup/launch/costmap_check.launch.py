"""Nav2 costmap fed by the RPLidar, to check /scan is usable for costmaps.

Run it next to the robot stack (master_launch.py, or the boot service):

    ros2 launch my_bringup costmap_check.launch.py              # uses /scan_filtered
    ros2 launch my_bringup costmap_check.launch.py scan_topic:=/scan

Publishes nav_msgs/OccupancyGrid on /costmap/costmap (view it in RViz as a Map
display, Fixed Frame base_link). Robot-centred, no odometry needed; see
config/costmap_check.yaml. Needs:
    sudo apt install ros-jazzy-nav2-costmap-2d ros-jazzy-nav2-lifecycle-manager
"""
import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scan_topic = LaunchConfiguration('scan_topic')
    args = [DeclareLaunchArgument(
        'scan_topic', default_value='/scan_filtered',
        description='LaserScan topic the obstacle layer marks/clears from')]

    missing = []
    for pkg in ('nav2_costmap_2d', 'nav2_lifecycle_manager'):
        try:
            get_package_share_directory(pkg)
        except PackageNotFoundError:
            missing.append(pkg)
    if missing:
        return LaunchDescription(args + [LogInfo(msg=(
            f'[costmap_check] missing {", ".join(missing)}: sudo apt install '
            + ' '.join('ros-jazzy-' + p.replace('_', '-') for p in missing)))])

    params = os.path.join(get_package_share_directory('my_bringup'), 'config', 'costmap_check.yaml')
    return LaunchDescription(args + [
        # Standalone Costmap2DROS; it names itself /costmap/costmap.
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            output='screen',
            parameters=[params, {'obstacle_layer.scan.topic': scan_topic}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_check',
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['costmap/costmap']}],
        ),
    ])
