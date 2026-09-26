#!/usr/bin/env bash
# One-time (and after requirement changes) install of what mode: autonomy needs on the Pi
# beyond ROS 2 Jazzy itself. Run as the `delivery` user:
#
#   ~/delivery-robo/deployment/install_autonomy_deps.sh
#
# Two layers:
#   1. apt   - ROS packages the launch files start (joy, twist_mux, tf2 tools) and pip itself;
#   2. pip   - the whole numeric stack from autonomy-requirements.txt. It is installed as one
#              consistent numpy-2 set on purpose: apt's pandas/shapely/pyproj are numpy-1 builds
#              and would not import next to the numpy 2 that current jax requires.
#
# PEP 668: Ubuntu marks the system Python "externally managed" and refuses plain pip. We
# install into the user's ~/.local with --user --break-system-packages: /usr/bin/python3, which
# the colcon-built ROS nodes run under, sees ~/.local, while apt's own files stay untouched.
# The alternative (a --system-site-packages venv activated before `colcon build`) bakes the venv
# interpreter into every node's shebang and breaks the moment someone builds without it, so it
# is not used here.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REQ="$HERE/autonomy-requirements.txt"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

echo "[deps] apt packages"
sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends \
  python3-pip python3-venv python3-requests \
  "ros-${ROS_DISTRO}-joy" "ros-${ROS_DISTRO}-twist-mux" "ros-${ROS_DISTRO}-tf2-ros" "ros-${ROS_DISTRO}-tf2-tools"

echo "[deps] pip packages from $REQ (user site, PEP 668 override)"
python3 -m pip install --user --break-system-packages --upgrade pip
python3 -m pip install --user --break-system-packages -r "$REQ"

echo "[deps] smoke test"
python3 - <<'PY'
import importlib, time
mods = ['numpy', 'scipy', 'pandas', 'shapely', 'networkx', 'pyproj', 'geopandas', 'osmnx', 'jax']
for m in mods:
    mod = importlib.import_module(m)
    print(f'  {m:10s} {getattr(mod, "__version__", "?")}')
import jax, jax.numpy as jnp
t0 = time.perf_counter()
print('  jax backend', jax.default_backend(), '- jit test:', float(jax.jit(lambda x: (x * 2).sum())(jnp.arange(4.0))),
      f'({time.perf_counter() - t0:.1f} s incl. compile)')
PY

cat <<'MSG'
[deps] done.
Next:
  1. rebuild:   cd ~/delivery-robo/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-ignore simulation
  2. time MPPI: source install/setup.bash && ros2 run hardware_bringup bench_mppi
  3. warm the planner's OSM cache once while online (it caches under ~/.cache/delivery_autonomy):
                ros2 launch hardware_bringup autonomy_real.launch.py   # Ctrl-C after "Path Planning Action Server is ready."
MSG
