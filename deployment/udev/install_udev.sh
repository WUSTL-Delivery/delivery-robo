#!/usr/bin/env bash
# Install the robot's udev rules on the Pi (or a dev laptop):
#   99-rplidar.rules          RPLidar C1 -> /dev/rplidar symlink
#   99-realsense-libusb.rules Intel RealSense USB access for non-root users
# Idempotent: safe to re-run after editing a rule (e.g. to pin the C1's
# serial). Must run as root:
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
RULES=(99-rplidar.rules 99-realsense-libusb.rules)
ROBOT_USER="${ROBOT_USER:-delivery}"

# 1. rule files (each only rewritten when it changed)
for rule in "${RULES[@]}"; do
  src="$HERE/$rule"
  dst="/etc/udev/rules.d/$rule"
  [[ -f "$src" ]] || { echo "missing $src" >&2; exit 1; }
  if [[ -f "$dst" ]] && cmp -s "$src" "$dst"; then
    echo "rule already up to date: $dst"
  else
    install -m 0644 -o root -g root "$src" "$dst"
    echo "installed $dst"
  fi
done

# 2. reload + re-trigger so already-plugged devices pick the rules up:
#    tty for the C1's symlink, usb for the RealSense permissions
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty --action=add
udevadm trigger --subsystem-match=usb --action=add
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
if lsusb 2>/dev/null | grep -qiE 'ID (8086:0b|8086:0a|38e5:)'; then
  echo "OK: RealSense on USB: $(lsusb | grep -iE 'ID (8086:0b|8086:0a|38e5:)' | head -1)"
else
  echo "no RealSense on USB (is the camera plugged in?)"
fi
