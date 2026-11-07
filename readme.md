# Delivery Robotics

Delivery MonoRepo for robotics

- `ros2_ws/` — the robot's ROS 2 Jazzy workspace. Install its dependencies with
  `deployment/install_deps.sh`; see `deployment/README.md`. The Gazebo simulation lives in the
  `delivery-autonomy` submodule (`master_launch.py mode:=autonomy sim:=true`).
- `docker/` — the same environment as a container: a dev container for laptops and the runtime
  image the Pi can run instead of the native install; see `docker/README.md`.
