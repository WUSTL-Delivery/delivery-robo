# Deployment — the robot Pi

The robot's compute is a Raspberry Pi, hostname `raspi`, user `delivery`,
running **ROS 2 Jazzy** installed natively at `/opt/ros/jazzy`. The repo is
cloned at `~/delivery-robo`.

## Connecting

```sh
ssh delivery@raspi.local     # mDNS; use the IP if .local doesn't resolve
```

(Ask a team member for the password / current WiFi. The Pi joins WiFi via
`wpa_supplicant.conf` in the home directory.)

## What runs at boot

`deployment/startup.sh`, run as a systemd service (`robot.service`). On every
start it:

1. `git fetch` + fast-forward to `origin/main` (20 s timeout — skipped when
   offline, refused if someone edited files directly on the Pi so local
   changes are never clobbered);
2. rebuilds `ros2_ws` with colcon **only if** the repo moved or there is no
   install yet (build failure falls back to the previous install);
3. reads `deployment/robot_config.yaml` and launches
   `ros2 launch my_bringup master_launch.py mode:=<mode>`.

So the normal deploy flow is: **merge to `main`, then reboot the robot** (or
`sudo systemctl restart robot.service`). No hand-building on the Pi.

## Robot config (`deployment/robot_config.yaml`)

```yaml
mode: teleop        # teleop | autonomous
robot_id: robo-1    # name shown on the robo-web dashboard
api_url: https://robo-web-ebon.vercel.app   # robo-web base URL (no trailing slash)
```

- `teleop` — wired joystick: `joy_node` + `joystick_control/joystick_node`
  (the setup verified working on the Pi).
- `autonomous` — sensor stack: RTK GPS + NTRIP (`ublox_dgnss`), IMU.

The committed file is the default for every Pi. To change the mode on one
robot without a git commit (a local edit to a tracked file would block the
boot-time `git pull`), copy it to `deployment/robot_config.local.yaml`
(gitignored) — that file wins when present. `ROBOT_CONFIG=/path/to.yaml`
overrides both. Unknown/missing mode falls back to `teleop`.

The same switch works by hand:
`ros2 launch my_bringup master_launch.py mode:=autonomous`.

- `robot_id` / `api_url` — identity and endpoint for the status heartbeat
  (see below). Fill in `api_url` with the Vercel URL of `../robo-web`.

Note: the ros2_control drive stack (`launch_real_robot.launch.py` in `sim/`)
is still launched manually.

## Status heartbeat (robo-web)

The robot reports itself to the `robo-web` dashboard (separate repo,
`../robo-web`, deployed on Vercel) so nobody has to hunt for IPs at a demo.
Two things send it:

1. **`startup.sh`** fires a fire-and-forget `curl -m 5` POST at the very top
   (`state: booting`, `stage: startup.sh`) and again right before the launch
   (`stage: launching`) — so the robot shows up on the dashboard seconds after
   power-on, long before ROS is up. Unreachable API = 5 s lost, nothing else.
2. **`my_bringup/heartbeat_node`** (`ros2_ws/src/my_bringup/my_bringup/heartbeat_node.py`),
   started by `master_launch.py` in every mode with `respawn=True`. Every
   `interval_s` (5 s) it POSTs a full status; network errors are logged
   (throttled to one warning / 30 s) and never crash the node.
   `state` is `booting` until the node has been up one interval and sees at
   least one other ROS node, then it becomes the mode (`teleop`/`autonomous`).

By hand:

```sh
ros2 run my_bringup heartbeat_node --ros-args -p api_url:=https://x.vercel.app -p robot_id:=robo-1
ros2 launch my_bringup master_launch.py mode:=teleop robot_id:=robo-1 api_url:=https://x.vercel.app
```

### API contract

- `POST <api_url>/api/heartbeat` — JSON body; the server upserts the robot by
  `id` and stamps `last_seen`. All fields except `id` are optional:

  ```json
  {
    "id": "robo-1",
    "state": "booting | teleop | autonomous",
    "mode": "teleop",
    "stage": "startup.sh | launching | ros",
    "hostname": "raspi",
    "ips": ["10.0.0.12"],
    "uptime_s": 83.4,
    "git_sha": "0356c3e",
    "ros_distro": "jazzy",
    "ros_nodes": ["joy_node", "control_node"],
    "ros_topics": ["/joy", "/cmd_vel"],
    "cpu_temp_c": 51.0,
    "load_avg": [0.5, 0.4, 0.3],
    "last_joy_age_s": 0.3,
    "last_cmd_vel": {"linear_x": 0.2, "angular_z": 0.0},
    "heartbeat_seq": 42,
    "ts": 1787943000.0
  }
  ```

- `POST <api_url>/api/status` — `{"id": "robo-1", "state": "dead"}`; used by the
  dashboard itself to mark a robot dead after 20 s without a heartbeat
  (Vercel has no free long-running timers, so the browser does it).
- `GET <api_url>/api/robots` — list of every known robot with its last
  payload and `last_seen`.

## Installing / re-wiring the service on the Pi

```sh
cd ~/delivery-robo && git pull
chmod +x deployment/startup.sh
sudo cp deployment/robot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now robot.service
```

Checking on it:

```sh
systemctl status robot.service          # running? enabled?
systemctl is-enabled robot.service      # "enabled" = starts on boot
journalctl -u robot.service -f          # live logs (launch output goes here)
```

Each run overwrites `~/.local/state/delivery-robo/latest-startup.log`, so the
file always contains output from the latest startup:

```sh
tail -F ~/.local/state/delivery-robo/latest-startup.log
cat ~/.local/state/delivery-robo/latest-startup.log   # share this when debugging
```

The log captures stdout and stderr from Git, colcon, ROS setup files, and the
ROS launch, while keeping output in the terminal and journal. Timestamped
stage markers and command exit codes show the repository state, fetch/merge
results (including fetch timeouts), build decisions, config parsing, setup
results, and heartbeat HTTP errors. Logging starts before fetching so failures
to locate the repo are recorded too. The script hands off to ROS with `exec`;
ROS output continues in the log, and systemd records its eventual exit status.

Set `ROBOT_LOG_DIR=/path/to/logs` to choose another directory (use a systemd
environment override when running as a service). If the log directory cannot
be written, startup prints a warning and continues with console/journal output.
Timestamped logs created by older versions are left in place; new runs reuse
`latest-startup.log` without creating additional files.

## History

The script that was found running on the Pi (uncommitted, ~/startup.sh)
was:

```bash
#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/delivery-robo/ros2_ws/install/setup.bash
ros2 launch my_bringup master_launch.py
# ros2 run joystick_control joystick_node &
#ros2 run joy joy_node &
```

`deployment/startup.sh` supersedes it (same launch, plus pull-and-rebuild).
When switching to the service, remove the old wiring for `~/startup.sh` so it
doesn't start twice.

## Pi setup notes (not yet automated)

Things the Pi needs that no script installs yet — verify when reimaging:

- pip: `adafruit-circuitpython-bno08x` (+ Blinka `board`/`busio`) for the IMU;
  `pyserial` for the encoder node
- apt/ROS: `ublox_dgnss` (+ ntrip client), `twist_mux`, `ros2_control` stack
- user groups: `dialout` (serial), `i2c`
- the `serial` package must be built with
  `colcon build --packages-select serial --cmake-args -DCMAKE_POSITION_INDEPENDENT_CODE=ON`
- `rplidar_sdk` is checked out at `~/rplidar_sdk` (lidar not yet wired into
  any launch)
- stray copies exist in the home dir (`~/relivery-robo-old`, a loose
  `~/build`/`~/install`) — do not source those by accident

## WiFi: hotspot first, wustl-guest fallback

`deployment/wifi/` sets wlan0 up with `wpa_supplicant@wlan0` (priority-ordered
SSIDs) + `systemd-networkd` DHCP. The hotspot has `priority=10`, `wustl-guest-2.0`
`priority=1` (lab wifi `wurc5` at 5), so the Pi joins the hotspot whenever it's in range and drops to
wustl-guest otherwise (and switches back when the hotspot returns). One-time
install on the Pi:

```sh
HOTSPOT_SSID='<ssid>' HOTSPOT_PSK='<password>' sudo -E deployment/wifi/install.sh
```

## RPLidar C1 (`/dev/rplidar` udev symlink)

The SLAMTEC RPLidar C1 (USB-C to USB into the Pi) is a CP2102 USB-serial
device (`10c4:ea60`, 460800 baud). `deployment/udev/99-rplidar.rules` gives
it a stable `/dev/rplidar` symlink, so the `sllidar_ros2` driver never has to
guess which `/dev/ttyUSBn` it got. The link appears on plug-in and vanishes
on unplug — no reboot needed.

Install (one-time per Pi, idempotent — re-run after editing the rule):

```sh
cd ~/delivery-robo
sudo deployment/udev/install_udev.sh
```

It copies the rule to `/etc/udev/rules.d/`, runs
`udevadm control --reload-rules` + `udevadm trigger`, and adds `delivery`
to `dialout` (effective after re-login/reboot; the rule also sets
`MODE=0666` so the port opens before that).

Verify:

```sh
ls -l /dev/rplidar            # -> lrwxrwxrwx ... /dev/rplidar -> ttyUSB0
udevadm info -q symlink -n /dev/ttyUSB0   # should list "rplidar"
# unplug the C1: /dev/rplidar must disappear
```

### Pinning to the C1's serial (do this if any other CP2102 is attached)

The default rule matches **any** CP2102. The IMU (BNO08x) is on I2C and the
u-blox GPS enumerates as `ttyACM*`, so neither collides — but the wheel
encoder's microcontroller shows up as `/dev/ttyUSB*`, and if its board uses a
CP2102 (common on ESP32 boards) it would also get `/dev/rplidar`. Check with
`lsusb` (look for more than one `10c4:ea60`). If so, pin the rule:

1. Plug in only the C1, then read its serial:
   ```sh
   udevadm info -a -n /dev/ttyUSB0 | grep -m1 'ATTRS{serial}'
   ```
2. In `deployment/udev/99-rplidar.rules`, comment out RULE A, uncomment
   RULE B and replace `REPLACE_WITH_C1_SERIAL` with that value.
3. `sudo deployment/udev/install_udev.sh`, plug the other board back in, and
   confirm `ls -l /dev/rplidar` still points at the C1's tty.

Note: the wheel-encoder node defaults to `port:=/dev/ttyUSB0`; with the C1
also plugged in, which device gets `ttyUSB0` depends on enumeration order,
so pass the encoder its own port (or give it its own udev symlink).

## Lidar -> costmap

`ros2_ws/src/my_bringup/config/costmap_lidar.example.yaml` is an **example, not
launched** Nav2 (Jazzy) params snippet for when costmap work starts (here or in
delivery-autonomy's MPPI stack). It defines `local_costmap` (global frame
`odom`) and `global_costmap` (global frame `map`), each with an
`obstacle_layer` + `inflation_layer` fed by one observation source `scan`:
`/scan_filtered` (LaserScan, `sensor_frame: lidar_link`), marking + clearing,
`obstacle_max_range: 8.0`, `raytrace_max_range: 10.0`,
`robot_base_frame: base_link`. The footprint is the sim chassis (0.8 x 0.4 m)
until TDM-13 confirms the real one. No launch file loads it — copy the
sections into the real Nav2 params file when wiring it up.

What the robot publishes for Nav2 (both modes, from `master_launch.py`, while
the C1 is plugged in; `lidar:=false` turns it off):

- `/scan`: raw `sensor_msgs/LaserScan` from `sllidar_node`, ~10 Hz, frame `lidar_link`
- `/scan_filtered`: the same scan with the robot body removed (laser_filters)
- TF `base_footprint -> base_link -> lidar_link` from `robot_state_publisher`

To check that Nav2 really builds a costmap from it (robot-centred, no odometry
needed), run next to the robot stack and view `/costmap` in RViz as a
Map display with Fixed Frame `base_link`:

```bash
sudo apt install ros-jazzy-nav2-costmap-2d ros-jazzy-nav2-lifecycle-manager   # once
ros2 launch my_bringup costmap_check.launch.py            # or scan_topic:=/scan
```
