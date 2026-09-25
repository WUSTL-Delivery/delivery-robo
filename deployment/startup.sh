#!/usr/bin/env bash
# Boot script for the delivery robot Pi (user: delivery, host: raspi).
# On every start: fetch the repo, rebuild if anything changed, read
# robot_config.yaml, then launch master_launch.py in the configured mode
# (teleop / autonomous). Designed to be run by systemd (see robot.service) but works
# fine by hand: ~/delivery-robo/deployment/startup.sh
#
# Failure behavior is deliberately "start anyway": no network -> skip the pull;
# build fails -> fall back to the last good install. The robot should come up
# in the field even when GitHub is unreachable.
# No `set -u`: ROS setup.bash files reference unset vars (AMENT_TRACE_SETUP_FILES)
# and would abort the script under nounset.
set -o pipefail

log() { printf '[%(%Y-%m-%dT%H:%M:%S%z)T] [startup] %s\n' -1 "$*" >&2; }

# Keep each boot's output, including stderr and the ROS process we exec below.
# tee also leaves output visible in the terminal / systemd journal.
LOG_DIR="${ROBOT_LOG_DIR:-$HOME/.local/state/delivery-robo}"
printf -v LOG_NAME 'startup-%(%Y%m%dT%H%M%S%z)T-%s.log' -1 "$$"
LOG_FILE="$LOG_DIR/$LOG_NAME"
if mkdir -p -- "$LOG_DIR" && (umask 077; : >> "$LOG_FILE") && command -v tee >/dev/null 2>&1; then
  exec > >(tee -a -- "$LOG_FILE") 2>&1
  ln -sfn -- "$LOG_NAME" "$LOG_DIR/latest-startup.log" || log "WARNING: could not update latest-startup.log"
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
ROS_SETUP=/opt/ros/jazzy/setup.bash
# Committed config, optionally shadowed by an untracked per-Pi override.
CONFIG="$REPO/deployment/robot_config.yaml"
[ -f "$REPO/deployment/robot_config.local.yaml" ] && CONFIG="$REPO/deployment/robot_config.local.yaml"
CONFIG="${ROBOT_CONFIG:-$CONFIG}"

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

# --- 2. Build if updated or never built ------------------------------------
stage "build"
if [ ! -f "$WS/install/setup.bash" ]; then
  log "rebuild required: $WS/install/setup.bash is missing"
  REBUILD=1
fi
if [ "$REBUILD" = 1 ]; then
  log "building ros2_ws..."
  log "sourcing $ROS_SETUP for build"
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  log "source $ROS_SETUP exit=$?"
  if (run_logged cd "$WS" && run_logged colcon build --symlink-install); then
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
  teleop|autonomous) ;;
  *) log "WARNING: unknown mode '$MODE', falling back to teleop"; MODE=teleop ;;
esac

# --- 4. Launch -------------------------------------------------------------
stage "ROS environment"
log "sourcing $ROS_SETUP"
# shellcheck disable=SC1090
source "$ROS_SETUP"
log "source $ROS_SETUP exit=$?"
log "sourcing $WS/install/setup.bash"
# shellcheck disable=SC1090
source "$WS/install/setup.bash"
log "source $WS/install/setup.bash exit=$?"
run_logged command -v ros2
stage "ROS launch"
announce booting launching
log "launching my_bringup master_launch.py mode:=$MODE robot_id:=$ROBOT_ID api_url:=$API_URL"
log "handing off to ros2 after ${SECONDS}s; launch stdout/stderr continue in this log"
exec ros2 launch my_bringup master_launch.py "mode:=$MODE" "robot_id:=$ROBOT_ID" \
  ${API_URL:+"api_url:=$API_URL"}
