#!/usr/bin/env bash
# Starts the robot stack from the runtime image, configured like startup.sh: robot_config.yaml,
# shadowed by robot_config.local.yaml, from deployment/ mounted at $REPO/deployment. No git pull or
# colcon build: the image already holds the built workspace. Arguments replace the launch, e.g.
# `docker compose run --rm robot ros2 topic list`.
set -euo pipefail
set +u  # the ROS setup files read unset variables
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
set -u

config="$REPO/deployment/robot_config.yaml"
[ -f "$REPO/deployment/robot_config.local.yaml" ] && config="$REPO/deployment/robot_config.local.yaml"
config="${ROBOT_CONFIG:-$config}"
cfg() { sed -n "s/^[[:space:]]*$1:[[:space:]]*\([^#]*\).*/\1/p" "$config" 2>/dev/null | head -n1 | xargs; }

domain=$(cfg ros_domain_id)
[ -n "$domain" ] && export ROS_DOMAIN_ID="$domain"
[ $# -gt 0 ] && exec "$@"

mode=$(cfg mode) nav2=$(cfg nav2) robot_id=$(cfg robot_id) api_url=$(cfg api_url)
echo "robot-entrypoint: $config mode=${mode:-teleop} nav2=${nav2:-false} image=$GIT_SHA"
exec ros2 launch my_bringup master_launch.py "mode:=${mode:-teleop}" sim:=false "nav2:=${nav2:-false}" \
  "robot_id:=${robot_id:-robo-1}" "api_url:=${api_url%/}"
