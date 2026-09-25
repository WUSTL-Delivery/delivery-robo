#!/usr/bin/env bash
# Install the RPLidar C1 udev rule (/dev/rplidar symlink) on the robot Pi.
# Idempotent: safe to re-run after editing 99-rplidar.rules (e.g. to pin
# the C1's serial). Must run as root:
#
#   sudo deployment/udev/install_udev.sh
#
# ROBOT_USER overrides the user added to dialout (default: delivery).
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "install_udev.sh: must run as root (use sudo)" >&2
  exit 1
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULE_SRC="$HERE/99-rplidar.rules"
RULE_DST="/etc/udev/rules.d/99-rplidar.rules"
ROBOT_USER="${ROBOT_USER:-delivery}"

[[ -f "$RULE_SRC" ]] || { echo "missing $RULE_SRC" >&2; exit 1; }

# 1. rule file (only rewritten when it changed)
if [[ -f "$RULE_DST" ]] && cmp -s "$RULE_SRC" "$RULE_DST"; then
  echo "rule already up to date: $RULE_DST"
else
  install -m 0644 -o root -g root "$RULE_SRC" "$RULE_DST"
  echo "installed $RULE_DST"
fi

# 2. reload + re-trigger tty devices so an already-plugged C1 gets the link
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty --action=add
udevadm settle || true

# 3. serial access for the robot user
if id "$ROBOT_USER" >/dev/null 2>&1; then
  if id -nG "$ROBOT_USER" | tr ' ' '\n' | grep -qx dialout; then
    echo "$ROBOT_USER already in dialout"
  else
    usermod -aG dialout "$ROBOT_USER"
    echo "added $ROBOT_USER to dialout (takes effect on next login/reboot)"
  fi
else
  echo "warning: user '$ROBOT_USER' not found; skipped dialout" >&2
fi

# 4. report
if [[ -e /dev/rplidar ]]; then
  echo "OK: $(ls -l /dev/rplidar)"
else
  echo "/dev/rplidar not present (is the C1 plugged in?)"
fi
