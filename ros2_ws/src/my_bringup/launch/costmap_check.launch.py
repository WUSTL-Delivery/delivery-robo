"""Nav2 costmap fed by the RPLidar and the RealSense, to check both are usable.

Run it next to the robot stack (master_launch.py, or the boot service):

    ros2 launch my_bringup costmap_check.launch.py              # lidar + camera
    ros2 launch my_bringup costmap_check.launch.py sources:=lidar
    ros2 launch my_bringup costmap_check.launch.py sources:=camera
    ros2 launch my_bringup costmap_check.launch.py scan_topic:=/scan

The lidar source reads scan_topic (/scan_filtered). The camera source reads
/camera/scan from depth_obstacles (floor removed), so obstacles below the
lidar plane show up too.

Publishes nav_msgs/OccupancyGrid on /costmap (view it in RViz as a Map
display, Fixed Frame base_link). Robot-centred, no odometry needed; see
config/costmap_check.yaml. Needs:
    sudo apt install ros-jazzy-nav2-costmap-2d ros-jazzy-nav2-lifecycle-manager
"""
import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    scan_topic = LaunchConfiguration('scan_topic')
    sources = LaunchConfiguration('sources')
    args = [
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan_filtered',
            description='lidar LaserScan topic the obstacle layer marks/clears from'),
        DeclareLaunchArgument(
            'sources', default_value='both', choices=['lidar', 'camera', 'both'],
            description='observation sources: lidar (scan), camera (/camera/scan) or both'),
    ]
    # costmap_check.yaml names the sources `scan` (lidar) and `camera`
    observation_sources = PythonExpression([
        "{'lidar': 'scan', 'camera': 'camera', 'both': 'scan camera'}['", sources, "']"])

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
        # Standalone Costmap2DROS; in Jazzy it names itself /costmap.
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            output='screen',
            parameters=[params, {'obstacle_layer.scan.topic': scan_topic,
                                 'obstacle_layer.observation_sources': observation_sources}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_check',
            output='screen',
            # bond_timeout 0: the standalone costmap never opens a bond, so the
            # default 4 s bond check aborts bringup (the costmap stays active).
            parameters=[{'autostart': True, 'node_names': ['costmap'], 'bond_timeout': 0.0}],
        ),
    ])
