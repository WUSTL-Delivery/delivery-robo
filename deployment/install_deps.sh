#!/usr/bin/env bash
# Installs everything ros2_ws needs beyond ROS 2 Jazzy. Run as `delivery` after dependency changes.
#
# pip goes first and is the only source of pip packages: the lock pins every version and hash.
# rosdep then installs the apt keys from every package.xml; it sees the pip keys in rosdep.yaml
# as already installed. Installs go to ~/.local (--user --break-system-packages, PEP 668) so the
# /usr/bin/python3 that ROS nodes run under sees them; a venv would leak into node shebangs.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends python3-pip python3-rosdep

python3 -m pip install --user --break-system-packages --require-hashes -r "$HERE/requirements.lock"

[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
echo "yaml file://$HERE/rosdep.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/50-delivery-robo.list >/dev/null
rosdep update --rosdistro "$ROS_DISTRO"
rosdep install --from-paths "$HERE/../ros2_ws/src" --ignore-src -y --rosdistro "$ROS_DISTRO"

python3 -c 'import jax, jax.numpy as jnp; print("jax", jax.__version__, jax.jit(lambda x: x * 2)(jnp.ones(2)))'
