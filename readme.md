# Delivery Robotics

Delivery MonoRepo for robotics

- `ros2_ws/` — the robot's ROS 2 Jazzy workspace. Install its dependencies with
  `deployment/install_deps.sh`; see `deployment/README.md`.
- `sim/` — the ros2_control / Gazebo workspace. Build it from `sim/` with plain `colcon build`;
  `sim/colcon.meta` adds the CMake flag the `serial` package needs.
