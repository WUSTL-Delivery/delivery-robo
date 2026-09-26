#!/usr/bin/env bash
# Boot script for the delivery robot Pi (user: delivery, host: raspi).
# On every start: fetch the repo, rebuild if anything changed, read
# robot_config.yaml, then launch master_launch.py in the configured mode
# (teleop / autonomous / autonomy). Designed to be run by systemd (see robot.service) but works
# fine by hand: ~/delivery-robo/deployment/startup.sh
#
# Laptop test with the Gazebo sim instead of the robot (config `mode: autonomy`, `sim: true`), from
# inside delivery-autonomy's `nix develop` so Gazebo and the autonomy Python deps are there:
#   ROS_SETUP= REPO=<clone> BRANCH=<branch> ROBOT_CONFIG=<sim config> <clone>/deployment/startup.sh
#
# Failure behavior is deliberately "start anyway": no network -> skip the pull;
# build fails -> fall back to the last good install. The robot should come up
# in the field even when GitHub is unreachable.
# No `set -u`: ROS setup.bash files reference unset vars (AMENT_TRACE_SETUP_FILES)
# and would abort the script under nounset.
set -o pipefail

log() { printf '[%(%Y-%m-%dT%H:%M:%S%z)T] [startup] %s\n' -1 "$*" >&2; }

# Reset one log file on each run, including stderr and the ROS process we exec below.
# tee also leaves output visible in the terminal / systemd journal.
LOG_DIR="${ROBOT_LOG_DIR:-$HOME/.local/state/delivery-robo}"
LOG_FILE="$LOG_DIR/latest-startup.log"
# Replace the previous version's symlink, leaving its archived log intact.
if mkdir -p -- "$LOG_DIR" && command -v tee >/dev/null 2>&1 &&
   { [ ! -L "$LOG_FILE" ] || rm -- "$LOG_FILE"; } &&
   (umask 077; : > "$LOG_FILE"); then
  exec > >(tee -a -- "$LOG_FILE") 2>&1
  log "log file: $LOG_FILE"
else
  log "WARNING: cannot enable file logging at $LOG_FILE; continuing with console output"
fi

STAGE=initialization
trap 'startup_status=$?; log "startup exited: stage=$STAGE exit=$startup_status elapsed=${SECONDS}s"' EXIT

stage() { STAGE="$1"; log "===== $STAGE ====="; }
run_logged() {
  local command_status command_text started_at=$SECONDS
  printf -v command_text '%q ' "$@"
  log "RUN $command_text"
  if "$@"; then command_status=0; else command_status=$?; fi
  log "DONE $command_text exit=$command_status elapsed=$((SECONDS - started_at))s"
  return "$command_status"
}

REPO="${REPO:-$HOME/delivery-robo}"
WS="$REPO/ros2_ws"
BRANCH="${BRANCH:-main}"
# ROS_SETUP= (empty) keeps the ROS already in the environment, e.g. delivery-autonomy's
# `nix develop` shell when testing with the sim on a laptop.
ROS_SETUP="${ROS_SETUP-/opt/ros/jazzy/setup.bash}"
# Committed config, optionally shadowed by an untracked per-Pi override.
CONFIG="$REPO/deployment/robot_config.yaml"
[ -f "$REPO/deployment/robot_config.local.yaml" ] && CONFIG="$REPO/deployment/robot_config.local.yaml"
CONFIG="${ROBOT_CONFIG:-$CONFIG}"

# shellcheck disable=SC1090
source_ros() {  # ROS_SETUP= (empty) keeps the ROS already in the environment
  if [ -z "$ROS_SETUP" ]; then
    log "ROS_SETUP is empty: using the ROS already in the environment (ROS_DISTRO=${ROS_DISTRO:-unset})"
    return 0
  fi
  log "sourcing $ROS_SETUP"
  source "$ROS_SETUP"
  log "source $ROS_SETUP exit=$?"
}
log "startup beginning: script=$0 pid=$$ user=$(id -un) host=$(hostname) bash=$BASH_VERSION"
log "working_directory=$PWD home=$HOME repo=$REPO workspace=$WS branch=$BRANCH"
log "config=$CONFIG ros_setup=$ROS_SETUP PATH=$PATH"

# Top-level "key: value" only; no yq/python dependency so boot can't fail on it.
cfg_get() { sed -n "s/^[[:space:]]*$1:[[:space:]]*\([^#]*\).*/\1/p" "$CONFIG" | head -n1 | xargs; }

# --- 0. Tell robo-web we're alive, before anything slow happens -------------
# Fire-and-forget, bounded by curl -m so an unreachable API costs <5 s and
# never blocks boot. The ROS heartbeat node takes over once the launch is up.
stage "initial config and heartbeat"
ROBOT_ID=""; API_URL=""
if [ -f "$CONFIG" ]; then
  ROBOT_ID=$(run_logged cfg_get robot_id)
  API_URL=$(run_logged cfg_get api_url)
else
  log "WARNING: config $CONFIG not found; using defaults"
fi
ROBOT_ID="${ROBOT_ID:-robo-1}"
API_URL="${API_URL%/}"
announce() {  # announce <state> <stage>
  [ -n "$API_URL" ] || { log "heartbeat skipped: api_url is empty"; return 0; }
  command -v curl >/dev/null 2>&1 || { log "heartbeat skipped: curl not installed"; return 0; }
  (
    log "heartbeat sending: state=$1 stage=$2"
    http_status=$(curl -m 5 -sS --fail -o /dev/null -w '%{http_code}' \
      -X POST "$API_URL/api/heartbeat" -H 'content-type: application/json' \
      -d "{\"id\":\"$ROBOT_ID\",\"state\":\"$1\",\"stage\":\"$2\",\"hostname\":\"$(hostname)\",\"uptime_s\":$(cut -d' ' -f1 /proc/uptime 2>/dev/null || echo 0)}")
    heartbeat_status=$?
    log "heartbeat finished: stage=$2 exit=$heartbeat_status http_status=$http_status"
  ) &
}
announce booting startup.sh

stage "repository state"
run_logged cd "$REPO" || { log "ERROR: repo missing or inaccessible at $REPO"; exit 1; }
run_logged git status --short --branch
run_logged git log -1 --format='%H %s'

# --- 1. Pull if we can reach the remote (bounded so boot never hangs) -------
stage "fetch and merge"
REBUILD=0
if run_logged timeout 20 git fetch origin "$BRANCH"; then
  LOCAL=$(run_logged git rev-parse HEAD)
  REMOTE=$(run_logged git rev-parse "origin/$BRANCH")
  log "revision comparison: local=$LOCAL remote=$REMOTE target=origin/$BRANCH"
  if [ "$LOCAL" != "$REMOTE" ]; then
    # ff-only: refuses instead of clobbering local edits made on the Pi.
    if run_logged git merge --ff-only "origin/$BRANCH"; then
      log "updated $LOCAL -> $(git rev-parse --short HEAD), will rebuild"
      REBUILD=1
    else
      merge_status=$?
      log "WARNING: merge failed (exit=$merge_status); see Git's error above. Running the OLD code."
      run_logged git status --short --branch
    fi
  else
    log "already up to date at $(git rev-parse --short HEAD)"
  fi
else
  fetch_status=$?
  if [ "$fetch_status" = 124 ]; then
    log "WARNING: fetch timed out after 20s"
  fi
  log "WARNING: fetch failed (exit=$fetch_status); see Git's error above — starting with the existing build; no fetch retry this run"
fi

# --- 1b. Sync submodules (delivery-autonomy, sllidar_ros2) -------------------
# Runs every boot, not only after a pull: a Pi that pulled before a submodule
# existed still has an empty directory. Bounded + non-fatal, like the fetch
# above. Only the paths registered in .gitmodules: the repo also carries
# gitlinks with no .gitmodules entry (librealsense, ublox_dgnss, sim/src/serial),
# and a bare `git submodule update` aborts on the first of those with
# "fatal: No url found for submodule path ..." before fetching anything.
# Offline with an empty delivery-autonomy -> autonomy mode has no autonomy nodes
# (master_launch warns and skips them); teleop is unaffected.
AUTONOMY_SUBMODULE=ros2_ws/src/delivery-autonomy
AUTONOMY_BEFORE=$(git -C "$REPO/$AUTONOMY_SUBMODULE" rev-parse HEAD 2>/dev/null || echo none)
if [ -f "$REPO/.gitmodules" ]; then
  mapfile -t SUBMODULE_PATHS < <(git config -f "$REPO/.gitmodules" --get-regexp '^submodule\..*\.path$' | awk '{print $2}')
  # 180 s: the first init clones delivery-autonomy (a few MB of meshes and history).
  if [ "${#SUBMODULE_PATHS[@]}" -gt 0 ] && \
     timeout 180 git submodule update --init --recursive -- "${SUBMODULE_PATHS[@]}" 2>&1; then
    log "submodules up to date"
  else
    log "WARNING: submodule update failed (offline?) — continuing with what's on disk"
  fi
fi
# A moved delivery-autonomy pin -> rebuild (its packages are not named after the submodule).
AUTONOMY_AFTER=$(git -C "$REPO/$AUTONOMY_SUBMODULE" rev-parse HEAD 2>/dev/null || echo none)
if [ "$AUTONOMY_BEFORE" != "$AUTONOMY_AFTER" ]; then
  log "submodule delivery-autonomy $AUTONOMY_BEFORE -> ${AUTONOMY_AFTER:0:7}, will rebuild"
  REBUILD=1
fi
# Newly initialized driver source that has never been built -> rebuild.
for pkg in sllidar_ros2; do
  if [ -f "$WS/src/$pkg/package.xml" ] && [ ! -d "$WS/install/$pkg" ]; then
    log "$pkg source present but not installed, will rebuild"
    REBUILD=1
  fi
done

# --- 2. Build if updated or never built ------------------------------------
stage "build"
# --packages-ignore simulation: the Gazebo package inside delivery-autonomy needs nothing
# the Pi lacks at build time, but it is useless there and slows the build. `sim: true`
# (laptop testing, see robot_config.yaml) builds it.
SIM=""
[ -f "$CONFIG" ] && SIM=$(run_logged cfg_get sim)
case "${SIM:-false}" in
  true) SIM=true; BUILD_IGNORE=()
        if [ ! -d "$WS/install/simulation" ]; then log "sim: true and simulation is not built, will rebuild"; REBUILD=1; fi ;;
  false) SIM=false; BUILD_IGNORE=(--packages-ignore simulation) ;;
  *) log "WARNING: sim must be true or false, got '$SIM'; using false"; SIM=false; BUILD_IGNORE=(--packages-ignore simulation) ;;
esac
if [ ! -f "$WS/install/setup.bash" ]; then
  log "rebuild required: $WS/install/setup.bash is missing"
  REBUILD=1
fi
if [ "$REBUILD" = 1 ]; then
  log "building ros2_ws..."
  source_ros
  if (run_logged cd "$WS" && run_logged colcon build --symlink-install "${BUILD_IGNORE[@]}"); then
    log "build ok"
  else
    build_status=$?
    log "WARNING: build FAILED (exit=$build_status) — falling back to the previous install"
  fi
else
  log "build skipped: no update applied and $WS/install/setup.bash exists"
fi

# --- 3. Read the robot config ----------------------------------------------
stage "launch config"
MODE=""
if [ -f "$CONFIG" ]; then
  MODE=$(run_logged cfg_get mode)
  log "config $CONFIG"
else
  log "WARNING: config $CONFIG not found"
fi
MODE="${MODE:-teleop}"
case "$MODE" in
  teleop|autonomous|autonomy) ;;
  *) log "WARNING: unknown mode '$MODE', falling back to teleop"; MODE=teleop ;;
esac
[ "$SIM" = true ] && [ "$MODE" != autonomy ] && log "WARNING: sim: true only applies to mode: autonomy; ignored in $MODE"

# Optional: pin the DDS domain so the robot does not share domain 0 with everything else on
# the WiFi it happens to join. Unset in the config = leave ROS_DOMAIN_ID as the environment has it.
DOMAIN=""
[ -f "$CONFIG" ] && DOMAIN=$(cfg_get ros_domain_id)
if [ -n "$DOMAIN" ]; then
  export ROS_DOMAIN_ID="$DOMAIN"
  log "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
fi

# --- 4. Launch -------------------------------------------------------------
stage "ROS environment"
source_ros
log "sourcing $WS/install/setup.bash"
# shellcheck disable=SC1090
source "$WS/install/setup.bash"
log "source $WS/install/setup.bash exit=$?"
run_logged command -v ros2
stage "ROS launch"
announce booting launching
log "launching my_bringup master_launch.py mode:=$MODE sim:=$SIM robot_id:=$ROBOT_ID api_url:=$API_URL"
log "handing off to ros2 after ${SECONDS}s; launch stdout/stderr continue in this log"
exec ros2 launch my_bringup master_launch.py "mode:=$MODE" "sim:=$SIM" "robot_id:=$ROBOT_ID" \
  ${API_URL:+"api_url:=$API_URL"}
