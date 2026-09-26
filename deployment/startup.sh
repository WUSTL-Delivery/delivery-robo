#!/usr/bin/env bash
# Boot script for the delivery robot Pi (user: delivery, host: raspi).
# On every start: fetch the repo, rebuild if anything changed, read
# robot_config.yaml, then launch master_launch.py in the configured mode
# (teleop / autonomous / autonomy). Designed to be run by systemd (see robot.service) but works
# fine by hand: ~/delivery-robo/deployment/startup.sh
#
# Failure behavior is deliberately "start anyway": no network -> skip the pull;
# build fails -> fall back to the last good install. The robot should come up
# in the field even when GitHub is unreachable.
# No `set -u`: ROS setup.bash files reference unset vars (AMENT_TRACE_SETUP_FILES)
# and would abort the script under nounset.
set -o pipefail

REPO="${REPO:-$HOME/delivery-robo}"
WS="$REPO/ros2_ws"
BRANCH="${BRANCH:-main}"
ROS_SETUP=/opt/ros/jazzy/setup.bash
# Committed config, optionally shadowed by an untracked per-Pi override.
CONFIG="$REPO/deployment/robot_config.yaml"
[ -f "$REPO/deployment/robot_config.local.yaml" ] && CONFIG="$REPO/deployment/robot_config.local.yaml"
CONFIG="${ROBOT_CONFIG:-$CONFIG}"

log() { echo "[startup] $*"; }

# Top-level "key: value" only; no yq/python dependency so boot can't fail on it.
cfg_get() { sed -n "s/^[[:space:]]*$1:[[:space:]]*\([^#]*\).*/\1/p" "$CONFIG" | head -n1 | xargs; }

# --- 0. Tell robo-web we're alive, before anything slow happens -------------
# Fire-and-forget, bounded by curl -m so an unreachable API costs <5 s and
# never blocks boot. The ROS heartbeat node takes over once the launch is up.
ROBOT_ID=""; API_URL=""
if [ -f "$CONFIG" ]; then
  ROBOT_ID=$(cfg_get robot_id)
  API_URL=$(cfg_get api_url)
fi
ROBOT_ID="${ROBOT_ID:-robo-1}"
API_URL="${API_URL%/}"
announce() {  # announce <state> <stage>
  [ -n "$API_URL" ] || return 0
  command -v curl >/dev/null 2>&1 || return 0
  curl -m 5 -s -X POST "$API_URL/api/heartbeat" -H 'content-type: application/json' \
    -d "{\"id\":\"$ROBOT_ID\",\"state\":\"$1\",\"stage\":\"$2\",\"hostname\":\"$(hostname)\",\"uptime_s\":$(cut -d' ' -f1 /proc/uptime 2>/dev/null || echo 0)}" \
    >/dev/null 2>&1 &
}
announce booting startup.sh

cd "$REPO" || { echo "[startup] repo missing at $REPO"; exit 1; }

# --- 1. Pull if we can reach the remote (bounded so boot never hangs) -------
REBUILD=0
if timeout 20 git fetch origin "$BRANCH" 2>&1; then
  LOCAL=$(git rev-parse HEAD)
  REMOTE=$(git rev-parse "origin/$BRANCH")
  if [ "$LOCAL" != "$REMOTE" ]; then
    # ff-only: refuses instead of clobbering local edits made on the Pi.
    if git merge --ff-only "origin/$BRANCH"; then
      log "updated $LOCAL -> $(git rev-parse --short HEAD), will rebuild"
      REBUILD=1
    else
      log "WARNING: local changes on the Pi diverge from origin/$BRANCH."
      log "WARNING: running the OLD code. Commit/stash on the Pi to resume updates."
    fi
  else
    log "already up to date at $(git rev-parse --short HEAD)"
  fi
else
  log "fetch failed (offline?) — starting with the existing build"
fi

# --- 1b. Submodules (delivery-autonomy lives at ros2_ws/src/delivery-autonomy) ----
# Bounded like the fetch. Offline with an empty submodule -> autonomy mode has no autonomy
# nodes (master_launch warns and skips them); teleop is unaffected.
SUBMODULE=ros2_ws/src/delivery-autonomy
if [ -f .gitmodules ]; then
  SUB_BEFORE=$(git -C "$SUBMODULE" rev-parse HEAD 2>/dev/null || echo none)
  # Only the paths registered in .gitmodules: the repo also carries gitlinks with no
  # .gitmodules entry (librealsense, rplidar_ros, ublox_dgnss, sim/src/serial), and a bare
  # `git submodule update` aborts on the first of those with "fatal: No url found for
  # submodule path ..." before cloning anything.
  mapfile -t SUBMODULE_PATHS < <(git config -f .gitmodules --get-regexp '^submodule\..*\.path$' | awk '{print $2}')
  # 180 s: the first init clones the repo (a few MB of meshes and history); later updates are instant.
  if [ "${#SUBMODULE_PATHS[@]}" -gt 0 ] && \
     timeout 180 git submodule update --init --recursive -- "${SUBMODULE_PATHS[@]}" 2>&1; then
    SUB_AFTER=$(git -C "$SUBMODULE" rev-parse HEAD 2>/dev/null || echo none)
    if [ "$SUB_BEFORE" != "$SUB_AFTER" ]; then
      log "submodule delivery-autonomy $SUB_BEFORE -> ${SUB_AFTER:0:7}, will rebuild"; REBUILD=1
    fi
  else
    log "WARNING: submodule update failed (offline?) — using whatever is checked out"
  fi
fi

# --- 2. Build if updated or never built ------------------------------------
# --packages-ignore simulation: the Gazebo package inside delivery-autonomy needs nothing
# the Pi lacks at build time, but it is useless there and slows the build.
[ -f "$WS/install/setup.bash" ] || REBUILD=1
if [ "$REBUILD" = 1 ]; then
  log "building ros2_ws..."
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  if (cd "$WS" && colcon build --symlink-install --packages-ignore simulation); then
    log "build ok"
  else
    log "WARNING: build FAILED — falling back to the previous install"
  fi
fi

# --- 3. Read the robot config ----------------------------------------------
MODE=""
if [ -f "$CONFIG" ]; then
  MODE=$(cfg_get mode)
  log "config $CONFIG"
else
  log "WARNING: config $CONFIG not found"
fi
MODE="${MODE:-teleop}"
case "$MODE" in
  teleop|autonomous|autonomy) ;;
  *) log "WARNING: unknown mode '$MODE', falling back to teleop"; MODE=teleop ;;
esac

# Optional: pin the DDS domain so the robot does not share domain 0 with everything else on
# the WiFi it happens to join. Unset in the config = leave ROS_DOMAIN_ID as the environment has it.
DOMAIN=""
[ -f "$CONFIG" ] && DOMAIN=$(cfg_get ros_domain_id)
if [ -n "$DOMAIN" ]; then
  export ROS_DOMAIN_ID="$DOMAIN"
  log "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
fi

# --- 4. Launch -------------------------------------------------------------
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1090
source "$WS/install/setup.bash"
announce booting launching
log "launching my_bringup master_launch.py mode:=$MODE robot_id:=$ROBOT_ID api_url:=$API_URL"
exec ros2 launch my_bringup master_launch.py "mode:=$MODE" "robot_id:=$ROBOT_ID" \
  ${API_URL:+"api_url:=$API_URL"}
