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
# Bring every installed ROS package up to the current apt sync before rosdep adds new ones
# (TDM-46). apt installs a new package from the current sync without upgrading the ROS core under
# it, and the mix fails only at run time: nav2_msgs built against fastcdr 2.2.8 on top of fastcdr
# 2.2.5 killed planner_server (exit 127, "symbol lookup error") on its first costmap publish.
# Rebuild ros2_ws after this upgrades anything.
mapfile -t ros_pkgs < <(dpkg-query -W -f='${db:Status-Abbrev} ${Package}\n' "ros-$ROS_DISTRO-*" \
                        2>/dev/null | awk '$1 == "ii" {print $2}')
[ "${#ros_pkgs[@]}" -eq 0 ] || sudo apt-get install -y --only-upgrade "${ros_pkgs[@]}"
sudo apt-get install -y --no-install-recommends python3-pip python3-rosdep

python3 -m pip install --user --break-system-packages --require-hashes -r "$HERE/requirements.lock"

[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
echo "yaml file://$HERE/rosdep.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/50-delivery-robo.list >/dev/null
rosdep update --rosdistro "$ROS_DISTRO"
rosdep install --from-paths "$HERE/../ros2_ws/src" --ignore-src -y --rosdistro "$ROS_DISTRO"

python3 -c 'import jax, jax.numpy as jnp; print("jax", jax.__version__, jax.jit(lambda x: x * 2)(jnp.ones(2)))'
