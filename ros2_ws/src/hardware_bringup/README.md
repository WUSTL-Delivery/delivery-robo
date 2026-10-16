# hardware_bringup

Glue between the real robot and the `delivery-autonomy` stack (checked out as the submodule
`ros2_ws/src/delivery-autonomy`, pinned to our `dev` branch, see
[delivery-autonomy version](#delivery-autonomy-version); nothing here edits it). It implements
`plans/autonomy-hardware-integration.md`: the autonomy nodes run on the Pi, a joystick can
override them at any moment, and one node owns the Arduino's serial port.

```
 joy_node ─/joy─► joy_to_cmdvel ─/cmd_vel_joy (prio 100)─┐
                                                          ├─► twist_mux ─/cmd_vel_out─► arduino_bridge ══serial══► Arduino
 controller ─/cmd_vel→/cmd_vel_nav (prio 10)─────────────┘                                  │ │
                                                     /cmd_vel_executed ◄────────────────────┘ └─► /odom, /wheel_ticks
 ublox_dgnss ─/fix (best effort)─► gps_datum_tf ─/gps/fix (reliable)─► state_estimation ◄─ /imu/data ◄─ imu_node
                                        └─► TF map→odom (static)            └─► /state_estimation/odom, TF odom→base_link
```

| Node | What it does |
|---|---|
| `arduino_bridge` | Sole owner of the serial port. `/cmd_vel_out` → `m <pwm> <servo>`; polls `e` at 20 Hz → `/odom` (twist only) and `/wheel_ticks`; watchdog stops the motor 0.5 s after the last command; rejects NaN; publishes the command it really sent on `/cmd_vel_executed`. |
| `joy_to_cmdvel` | Joy → Twist on `/cmd_vel_joy` while the enable button is held. Same stick mapping and servo range as `joystick_node`. No joystick present → zeros at top priority (robot held stopped). |
| `twist_mux` | Upstream package. Joystick beats controller; a source drops out 0.5 s after it stops publishing. It never publishes zeros itself, which is why the bridge has the watchdog. |
| `gps_datum_tf` | Gates the GPS on `status` and finite values, converts best-effort → reliable QoS, forwards to the EKF once it subscribes, and publishes `map→odom` from the first forwarded fix (translation = UTM − planner origin, yaw = grid convergence ≈ +1.685°). |
| `bench_mppi` | Times `mppi_step` on this CPU. Run it before the first drive. |

Everything the EKF reads is what the plan's §2 table lists: `/imu/data` (angular_velocity.z),
`/gps/fix` (lat/lon), `/odom` (twist), and the executed command as its prediction input.

## Running

```sh
# whole robot, at boot: deployment/robot_config.yaml  ->  mode: autonomy
ros2 launch my_bringup master_launch.py mode:=autonomy

# pieces, for the bench
ros2 launch hardware_bringup drive.launch.py [port:=/dev/serial/by-id/usb-...]   # joystick + mux + bridge
ros2 launch hardware_bringup autonomy_real.launch.py [gps_in_topic:=/fix]        # datum + EKF + planner + controller

# a goal (node id from the planner's "Sample Node IDs for testing:" log line)
python3 ros2_ws/src/delivery-autonomy/src/autonomy/autonomy/behavior.py <node_id>
```

First-time setup on the Pi: `deployment/install_autonomy_deps.sh` (Python deps, `joy`,
`twist_mux`), then rebuild. `startup.sh` initialises the submodule and builds with
`--packages-ignore simulation`.

All parameters live in `config/hardware.yaml`. The three geometry numbers under `/**` are shared
by `arduino_bridge` and `joy_to_cmdvel` and must stay identical for both, which is why they are
declared once. Its `controller_server` section holds the robot's settings for the
delivery-autonomy MPPI controller, see [Controller parameters](#controller-parameters).

`autonomy_real.launch.py` arguments: `hardware_params` (default `config/hardware.yaml`),
`gps_in_topic`, `ekf_cmd_vel_topic`, `sim`, `planner_cache_dir`, and `nav2` (default `false`).
`nav2` sets the controller's `use_costmap`. master_launch passes its own `sim` and `nav2`.

## delivery-autonomy version

The submodule `ros2_ws/src/delivery-autonomy` is pinned to commit `861989d` of
delivery-autonomy's **`dev`** branch. That is our branch of the upstream code, made while its
author is away. It adds the controller parameters below, the slow-down and stop for costmap
obstacles, the taper into the path end, MPPI's costmap cost, the WGS84 GPS projection, and the
sim fixes. `.gitmodules` says `branch = dev` for documentation only: `git submodule update` checks
out the pinned commit, not the branch head. Move the pin with `git submodule update --remote`
or `git update-index --cacheinfo 160000,<sha>,ros2_ws/src/delivery-autonomy`.

**The Pi cannot fetch the pinned commit yet.** `dev` exists only in local checkouts until
someone pushes it to the submodule's remote (`sebtheiler/delivery-autonomy`). That push is
human ticket TDM-35, and it has not happened. Until then, `startup.sh`'s submodule update fails
on the Pi. That failure is not fatal: the Pi logs "submodule update failed" and keeps the
delivery-autonomy it already has (upstream `2b8924b`, the previous pin). That old controller
ignores every `controller_server` value in `hardware.yaml`. It drives at a fixed 1.5 m/s, does
not read the costmap, and can reverse. Before driving autonomously, check that
`git -C ros2_ws/src/delivery-autonomy rev-parse --short HEAD` on the Pi prints `861989d`.

## Controller parameters

`autonomy_real.launch.py` starts delivery-autonomy's controller (`controller_server`, the MPPI
path follower behind `/follow_path`) with `config/hardware.yaml` as its parameter file. It does
this with `sim:=true` too, so the sim drives with the robot's values. The file sets only the three
values that differ from the controller's defaults. `use_costmap` comes from the launch file's
`nav2` argument. All but the two topics can be changed at runtime
(`ros2 param set /controller_server ...`). The controller rejects a value outside the ranges below.

| Parameter | Robot | Default | Why |
|---|---|---|---|
| `max_speed` | **1.0** | 1.5 | m/s. Cruise speed, and the cap on every forward command. 1.5 is the joystick's full stick (`pwm_limit` / `pwm_per_mps`), and `pwm_per_mps` is still a guess on an open-loop drive, so nobody knows how fast 1.5 really is. Start lower. MPPI accepts (0, 1.5]. |
| `min_speed` | **0.0** | -0.5 | m/s. The robot never reverses. Nav2's Hybrid-A\* paths are Dubins (no reversing), and nothing behind the robot is checked. At 1.5 m/s with -0.5, the TDM-33 sim runs had 3-4 stray reverse commands per run (down to -0.13 m/s). In the TDM-34 sim at 1.0 m/s, neither value produced a reverse command, and 0.0 changed neither the goal accuracy nor the box behaviour (see "Sim recipe"). Range [-0.5, 0]. |
| `stop_distance` | **0.2** | 0.35 | m. The robot stops for an inscribed or lethal costmap cell that is near the path ahead: the path passes within this distance of it, and the robot gets closer to it. The limit falls linearly from `max_speed` at `slow_distance` to 0 at `stop_distance`, and anything under 0.05 m/s counts as a stop. So with these values the robot halts at 0.2 + 0.05 × 1.8 / 1.0 = 0.29 m from such a cell (0.26 m at 1.5 m/s). With 0.35, an obstacle centred in a 3 m corridor (two 1.2 m gaps) stops the robot until someone moves it. With 0.2 it either squeezes past or stops, depending on where the plan puts the robot in the gap (TDM-34 sim: stopped in 3 of 4 runs, passed once). The user approved 0.2 on 2026-10-16. |
| `use_costmap` | `nav2:=` | true | Slow down and stop for costmap obstacles near the path, and let MPPI pay for costmap cells. Set by the launch file: true only when Nav2 runs. Without a costmap the controller would cap itself at `crawl_speed`. |
| `slow_distance` | 2.0 | 2.0 | m. Full speed when the nearest obstacle near the path ahead is at least this far away, then linearly down to 0 at `stop_distance`. |
| `clearance_lookahead` | 3.0 | 3.0 | m of path ahead checked for obstacles. |
| `costmap_timeout` | 1.0 | 1.0 | s. An older costmap counts as none. Nav2 publishes at 4 Hz (`config/nav2_local.yaml`). |
| `crawl_speed` | 0.3 | 0.3 | m/s cap while `use_costmap` is on but the costmap is missing or stale. A stale costmap still stops the robot for what it shows. |
| `goal_decel` | 0.8 | 0.8 | m/s². Braking into the end of the path: speed ≤ √(2·decel·remaining). |
| `goal_creep_speed` | 0.15 | 0.15 | m/s. The taper's floor, so the robot still reaches the end. |
| `xy_goal_tolerance` | 0.25 | 0.25 | m (EKF). The path counts as done within this distance of its end... |
| `yaw_goal_tolerance` | 0.15 | 0.15 | rad. ...and this heading error. |
| `state_topic`, `costmap_topic` | defaults | `/state_estimation/odom`, `/global_costmap/costmap` | |

Not parameters yet: MPPI's model has the sim robot's geometry (wheelbase 0.5 m, steering limit
0.6 rad) in delivery-autonomy `utils/dynamics.py`. The robot's are 0.36322 m and 0.4363 rad (see
"Known limits").

## Nav2 local planning

Nav2 builds a costmap from three things: the boundary map (delivery-autonomy's
`autonomy/maps/boundary.yaml`, free within 1.5 m of every OSM walkway edge, occupied
elsewhere), the EKF's TF and the scans. It then plans a short local path to a goal with Smac
Hybrid-A\*. The delivery-autonomy MPPI (`/follow_path`) follows that path. Nav2's own controller
is not used.

```
 boundary.yaml ─► map_server ─/map─► static_layer ───┐
 scans (sources:=) ─────────────────► obstacle_layer ─┼─► inflation_layer = the costmap
 TF map→odom + EKF odom→base_link ───────────────────┘    (frame odom, rolling 40 x 40 m @ 0.1 m)
                                                                │
 /goal_pose (RViz) ──────┐               ComputePathToPose      ▼
 /nav/goal_node (Int64) ─┼─► local_planner ◄────────────► planner_server (Smac Hybrid-A*)
 /nav/cancel (Empty) ────┘     │  ▲
                               │  └── /compute_shortest_path (OSM route, for /nav/goal_node)
                               ├──► /nav/local_plan (Path in map, always)
                               └──► /follow_path (drive:=true) ─► controller (MPPI) ─► cmd_vel
```

| Node | What it does |
|---|---|
| `map_server` | Serves the boundary map on `/map` (frame `map`, transient local). |
| `planner_server` | Owns the costmap (`/global_costmap/global_costmap`; plugins static, obstacle, inflation) and answers `/compute_path_to_pose` with Smac Hybrid-A\* (`planner_id` `GridBased`, Dubins, no reversing). If `nav2_planner` is not installed, the launch file runs the standalone costmap under the same name instead, and nothing plans. |
| `lifecycle_manager_nav2_local` | Configures and activates the two nodes above (`autostart`). |
| `local_planner` | Turns a goal into a route. Twice a second it plans a path to a carrot ~12 m ahead on the route, transforms the path to `map`, publishes it on `/nav/local_plan` and, with `drive:=true`, sends it to `/follow_path`. When planning fails (a loop counts as a failure) it sends one empty path (the follower stops) and then tries carrots ±2 m and ±4 m along the route. Within 2 m of the goal, once the last path ends there, it stops replanning and lets the follower finish that path. |

The Nav2 parameters are in `config/nav2_local.yaml`. `launch/nav2_local.launch.py` patches that
file for its arguments. local_planner has no section there: it runs on its own defaults (see
[local_planner parameters](#local_planner-parameters)), apart from `drive`, `use_sim_time`, and
`planner_id`, which the launch file sets from `drive:=`, `sim:=` and planner_server's
`planner_plugins`.

### Running

```sh
# on the robot: the whole stack plus Nav2 (opt-in until the Pi's CPU budget is measured)
ros2 launch my_bringup master_launch.py mode:=autonomy nav2:=true [nav2_sources:=separate]

# Nav2 alone, next to a running stack
ros2 launch hardware_bringup nav2_local.launch.py                 # plan only: /nav/local_plan, nothing drives
ros2 launch hardware_bringup nav2_local.launch.py drive:=true     # ... and send each path to /follow_path
ros2 launch hardware_bringup nav2_local.launch.py local_planner:=false   # costmap + planner_server only
```

| Argument | Default | Meaning |
|---|---|---|
| `sources` | `auto` | Scan inputs to the obstacle layer, see below. `auto` = `lidar` with `sim:=true`, else `fused`. master_launch passes `nav2_sources`. |
| `sim` | `false` | Sim time everywhere, lidar on `/scan`. master_launch passes its own `sim`. |
| `drive` | `false` | local_planner also sends each path to `/follow_path`. master_launch always sets `true`. |
| `local_planner` | `true` | Start local_planner. |
| `map` | autonomy's `share/autonomy/maps/boundary.yaml` | Boundary map yaml. Required when the `autonomy` package is not built. |
| `scan_topic` | `/scan_filtered` (`/scan` in sim) | The lidar source's topic. |
| `params_file` | `config/nav2_local.yaml` | Nav2 parameters. |

`sources:=` modes:

| Mode | Marks obstacles | Only clears | Use |
|---|---|---|---|
| `fused` | `/scan_fused` | `/scan_filtered`, `/camera/scan` | Robot default. Needs `my_bringup scan_fusion` (branch `feat/lidar-depth-scan-fusion`, not merged yet). Until it is merged, use `separate`. |
| `separate` | `/scan_filtered`, `/camera/scan` | – | Robot, lidar and depth camera each mark and clear. |
| `lidar` | `scan_topic` | – | One LaserScan. The sim (`/scan`) or a robot without the camera. |

Every source has an `expected_update_rate`. If a sensor stops publishing, the costmap goes
stale and planning fails, so the robot stops. It does not plan on an old picture. The launch
file degrades instead of failing: if the Nav2 packages or the map are missing, it logs the
reason and starts nothing; if `nav2_msgs` or `shared_types` is missing, it skips only
local_planner. master_launch therefore runs as before in a shell without Nav2.

### Sending a goal

```sh
# a pose in map, here 5 m east of the sim's spawn point (RViz "2D Goal Pose" publishes the same)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: -90.0, y: 25.5}, orientation: {w: 1.0}}}'
# an OSM node id: route from /compute_shortest_path, then local paths along it
ros2 topic pub --once /nav/goal_node std_msgs/msg/Int64 '{data: 1282043935}'
# stop now (empty path to the follower) and forget the goal
ros2 topic pub --once /nav/cancel std_msgs/msg/Empty '{}'
```

A zero quaternion on `/goal_pose` means "no final heading". Watch `/nav/local_plan` (Path, frame
`map`) and `/global_costmap/costmap` (OccupancyGrid, frame `odom`) in RViz. With `drive:=false`
the plan is published but nothing moves.

### local_planner parameters

Every parameter the node declares, with its default. The launch file passes only `drive`,
`planner_id` and `use_sim_time`. To change any other, start the launch file with
`local_planner:=false` and run the node yourself:

```sh
ros2 run hardware_bringup local_planner --ros-args -p drive:=true -p rate_hz:=1.0 [-p use_sim_time:=true]
```

| Parameter | Default | Meaning |
|---|---|---|
| `rate_hz` | `2.0` | Replans per second. Read once at startup. Only one request is in flight, so a plan slower than one period skips ticks. 2.0 since TDM-36: in TDM-25's sim runs the robot passed a centred box 0.154 m away on average at 2 Hz, against 0.096 m at 1 Hz, and each plan takes 2-15 ms. Planner_server's `expected_planner_frequency` matches it. |
| `lookahead` | `12.0` | m. The carrot sits this far along the route ahead of the robot's progress, clamped to the goal. Keep it well inside the 40 x 40 m rolling costmap and no longer than Smac's `lookup_table_size`. |
| `carrot_offsets` | `[0.0, 2.0, -2.0, 4.0, -4.0]` | m along the route, added to `lookahead`. After n failures in a row the next carrot uses entry n (0-based, cycling); a success goes back to the first. Along the route because the boundary map is free only within 1.5 m of an OSM edge. An empty list means `[0.0]`. |
| `final_approach_distance` | `2.0` | m (TDM-28). Once the last path sent ends at the goal and the robot is this close to it, the node stops replanning, drops any request in flight and lets the follower finish that path. It replans again only if the robot gets 0.5 m further out. Closer than ~2 m the forward-only planner answers with 11-13 m loops. In this stretch a new obstacle no longer stops the robot. |
| `loop_margin` | `5.0` | m (TDM-28). A plan longer than it needs to be by more than this is a failure (reason "loop"). Needed = the straight line to its end, or the route's own arc to the carrot when that is longer. Only checked when the carrot's heading is within 90° of the robot's, so a U-turn to a goal behind the robot is kept. `<= 0` turns the check off. |
| `goal_tolerance` | `0.5` | m. The goal counts as reached (logged, then forgotten) once the robot is this close and the carrot is at the goal. It does not stop replanning: `final_approach_distance` does. The follower itself stops 0.25 m from the end of its path. |
| `drive` | `false` | Also send every path and every stop (empty path) to `follow_action`. Without it the node only publishes on `plan_topic`. Set by `drive:=`. |
| `planner_id` | `GridBased` | The planner_server plugin to ask (`ComputePathToPose` `planner_id`). Set from planner_server's first `planner_plugins` entry. |
| `planner_timeout` | `5.0` | s. A request with no answer after this long is cancelled and counts as a failure. |
| `route_spacing` | `0.5` | m. Routes (the OSM route, or the two-point route of a direct goal) are resampled at this spacing. |
| `progress_window` | `4.0` | m. The robot's progress is its projection onto the next `progress_window` metres of route past the last progress. It never moves backwards, and a route that passes near itself cannot make it jump ahead. |
| `map_frame` | `map` | Frame of routes, carrots, `plan_topic` and every path sent to the follower (which assumes `map` whatever the header says). |
| `robot_frame` | `base_link` | The robot pose is the latest `map_frame` → `robot_frame` transform. |
| `planner_action` | `/compute_path_to_pose` | Nav2 `ComputePathToPose` (planner_server). |
| `route_action` | `/compute_shortest_path` | delivery-autonomy `ComputePath`: the OSM route for a `goal_node_topic` goal. |
| `follow_action` | `/follow_path` | delivery-autonomy `FollowPath` (the MPPI controller). Used only with `drive`. |
| `plan_topic` | `/nav/local_plan` | Every accepted plan (Path in `map_frame`), and an empty Path on every stop. |
| `goal_topic` | `/goal_pose` | Direct goals (PoseStamped, any frame with a transform to `map_frame`). |
| `goal_node_topic` | `/nav/goal_node` | OSM node goals (Int64). |
| `cancel_topic` | `/nav/cancel` | Empty: stop now and forget the goal. |

### Sim recipe (Lyrical Gazebo + Jazzy Nav2)

The Nix Lyrical shell has no prebuilt Nav2, so Nav2 runs from apt Jazzy in a second terminal.
Both terminals talk over DDS on one domain. Use a **private `ROS_DOMAIN_ID`** in both
terminals: domain 0 has other processes on it, including a fake `/gps/fix` publisher.

```sh
# terminal 1: Lyrical sim + autonomy (delivery-autonomy's nix develop shell, see
# deployment/README.md "Testing with the sim"). nav2:=true starts no Nav2 here (the shell
# has none, so nav2_local.launch.py logs that and starts nothing), but it sets the
# controller's use_costmap, so the MPPI slows and stops for terminal 2's costmap
export ROS_DOMAIN_ID=57
ros2 launch my_bringup master_launch.py mode:=autonomy sim:=true nav2:=true

# terminal 2: apt Jazzy, with a workspace that has hardware_bringup plus delivery-autonomy's
# shared_types and autonomy built against Jazzy (autonomy installs the boundary map)
export ROS_DOMAIN_ID=57
source /opt/ros/jazzy/setup.bash && source <jazzy_ws>/install/setup.bash
ros2 launch hardware_bringup nav2_local.launch.py sim:=true drive:=true
```

Then send a goal from terminal 2 as shown above. Notes:

- The Jazzy and Lyrical `ros2` CLI daemons clash on one domain (XML-RPC "unknown tag"). Run
  `ros2 daemon stop` and use `--no-daemon`, or a small rclpy script.
- Topics and actions both work across the two distros: the Jazzy local_planner calls the
  Lyrical `/compute_shortest_path` and `/follow_path` directly, no relay (TDM-25).
- `map → odom` matches `/ground_truth/odom`. The robot settles ~0.37 m east of its spawn request
  after the z = 5 drop. With master_launch `sim:=true`, `autonomy_real.launch.py` runs
  `gps_datum_tf` with `projection:=local_enu`, which publishes `map → odom` from the same first
  fix the EKF takes, wherever the robot is then. delivery-autonomy's own `autonomy.launch.py`
  (since its TDM-39) publishes a fixed `map → odom` at the settled pose (−94.630, 25.504), which
  is right as long as state_estimation starts after the robot has landed.
- `ground_truth_node` stays up while the robot model is removed and re-created (a reset); since
  delivery-autonomy TDM-39 it skips the empty pose messages that used to kill it.
- The bridge publishes `/clock` at ~1 kHz, so every rclpy node with sim time spends 60-80 % of
  a core on it (local_planner 77 % even when idle). CPU numbers from the sim say nothing about
  the Pi.

What TDM-25 measured in this sim (node 1282043935, 190 m from spawn, 1.5 m/s, 12 runs):

- The robot always reaches the goal, 0.41-0.50 m from it by ground truth. The follower keeps
  1.5 m/s until it is within 0.25 m of the path end (by the EKF), then commands 0; the robot
  brakes ~0.4 m and stops 0.2-0.28 m past the end. On top of that, the EKF lags ~0.15 m while
  moving and is off by a steady 0.25 m at the goal.
- About 134 plans per run at 1 Hz (266 at 2 Hz). Each plan takes 2-15 ms. With the stock
  parameters (local_planner at 1 Hz, the default then), no planning failures on an empty route.
- A 0.6 m box in the middle of a 3 m corridor is always passed, but only 0.00-0.24 m away. The
  lidar marks only the box's near face, so plans swing back towards the centreline through the
  part of the box nobody has seen yet. A centred box also leaves only 1.2 m gaps: the most a
  0.63 m robot centred in one can keep is 0.27 m. A box 0.45 m off-centre is passed 0.36 m away.
- That is why local_planner now replans at 2 Hz (TDM-36): the mean clearance past a centred box
  was 0.154 m at 2 Hz against 0.096 m at 1 Hz. `footprint_padding: 0.1` did not help (0.099 m),
  added stop paths beside the boxes and turned the slope marks below into corridor-blocking
  walls (one 4 s stall), so it is not used.
- At the foot of the route's 2° slopes, the lidar sometimes marks the rising ground 7 m ahead
  as a line of obstacles across the corridor for about a second. At 1 Hz that never failed a
  plan; at 2 Hz it caused some NO_VALID_PATH stop paths, but the robot did not halt.

What TDM-34 measured with delivery-autonomy `dev` (861989d) and this file's
[controller parameters](#controller-parameters): `max_speed` 1.0, `min_speed` 0.0,
`stop_distance` 0.2, `use_costmap` true. Same route, local_planner at 2 Hz, footprint clearance
by ground truth. A box that stopped the robot was removed 10 s later, and the robot then drove
on.

| Case | Result | Goal error |
|---|---|---|
| No box | No slowdowns, mean 0.99 m/s, no reverse command | 0.161 m short |
| 0.6 m box, centred | Stopped 0.263 m from the box | 0.140 m |
| 0.6 m box, centred, repeat | Passed at 0.229 m, after a 0.5 s stop | 0.143 m |
| 0.6 m box, centred, `autonomy_real.launch.py sim:=true nav2:=true` | Stopped 0.236 m from the box | 0.148 m |
| 0.6 m box, 0.45 m off-centre | Passed at 0.444 m, no slowdown | 0.140 m |
| `min_speed` -0.5, no box | No reverse command | 0.131 m |
| `min_speed` -0.5, centred box | Stopped 0.205 m from the box | 0.135 m |

The robot never touched a box. Goal errors are 0.13-0.16 m, against 0.41-0.50 m before the
taper and the WGS84 fix. TDM-33, at 1.5 m/s with `min_speed` -0.5, got 0.147 m with no box,
0.219 m (pass) or 0.271 m (stop) at the centred box, and 0.443 m off-centre.

## Bench checks before the first drive (wheels off the ground)

1. **Serial.** `ros2 launch hardware_bringup drive.launch.py`. The bridge logs `Arduino ready
   (encoder count N)`. If it keeps saying `waiting for the Arduino reset` / `no encoder reply`,
   check the port (`ls -l /dev/serial/by-id/`) and that the motor-controller sketch is flashed.
2. **Command path.** `ros2 topic pub -r 10 /cmd_vel_out geometry_msgs/msg/Twist '{linear: {x: 0.3}}'`
   spins the motor forward and centres the servo; `'{linear: {x: 0.3}, angular: {z: 0.3}}'` turns
   the wheels left (CCW yaw). If they turn right, set `servo_deg_per_rad` negative. Publish
   continuously (`-r 10`), not `-1`: a single message is cancelled by the 0.5 s watchdog. Ctrl-C
   the publisher: the motor must stop within 0.5 s and the log says `no command for 0.50 s`.
3. **Odometry.** `ros2 topic echo /odom` while spinning a wheel by hand: `twist.linear.x` must be
   positive when the wheel turns forward. If negative, make `metres_per_tick` negative.
4. **Joystick.** `ros2 topic echo /joy`, press the intended deadman button and read its index
   into `enable_button` (4 is LB/L1 on Xbox and PlayStation pads under `joy_node`; the sim's 9
   is a stick click there). With the button held, the stick must move the servo and motor and
   the log must say `manual override ACTIVE`; released, `manual override released`.
5. **Override.** Start `autonomy_real.launch.py` too, send a goal, then hold the enable button and
   move the stick: `ros2 topic echo /cmd_vel_out` must follow the stick within 0.5 s, and revert
   to the controller 0.5 s after release.
6. **Sensors and frames.** `ros2 topic hz /imu/data` ≈ 50 Hz. `ros2 topic echo /fix --once`
   (raw) then `/gps/fix --once` (gated): `status.status` ≥ 0, 1 once RTK corrections flow.
   `ros2 run tf2_ros tf2_echo map odom` shows a yaw of about 1.685° and a translation of tens
   of metres at most; `tf2_echo map base_link` follows the robot as it is pushed.
7. **CPU.** `ros2 run hardware_bringup bench_mppi` must report a max well under 100 ms.

## Calibration (`config/hardware.yaml`)

Every value here was guessed from code that contradicts itself; measure each one.

| Parameter | How |
|---|---|
| `metres_per_tick` | Push the robot a tape-measured 10 m in a straight line. `ros2 topic echo /wheel_ticks` before and after; divide. Sign: positive if the count grew while going forward. |
| `servo_center_deg` | Front jacked up. Publish `/cmd_vel_out` (`-r 10`) with `x: 0.3` and `z: 0.0` while adjusting the value until the wheels point straight. The deployed `joystick_node` used 45; the firmware and the ros2_control HAL use 90. |
| `servo_deg_per_rad`, `max_steer_rad` | Command a known yaw rate at a known speed (δ = atan(ω·L/v)), read the road-wheel angle with a protractor, take the ratio servo-degrees/road-radian. `max_steer_rad` is the mechanical limit of the linkage. Keep `joy_to_cmdvel`'s copy identical (it is the shared `/**` value). |
| `pwm_per_mps`, `pwm_limit` | On the ground, publish (`-r 10`) `x:` 0.3, 0.6, 0.9, 1.2, 1.5 m/s in turn and read the steady `/odom` speed for each; fit a line through PWM (= x·pwm_per_mps) versus measured speed and update `pwm_per_mps`. `pwm_limit` caps the drive (110 = today's joystick cap; firmware accepts 255). The drive is open loop, so the fit is the speed control. |
| `wheelbase` | Tape measure, rear axle centre to front axle centre. The URDF says 0.36322, the HAL 0.30, the autonomy model 0.5 (the autonomy's own 0.5 stays: the bridge absorbs the difference, except in the tightest turns, see "Known limits"). |
| IMU covariances | `ros2 run imu_package imu_covariance_estimator` while driving, then paste into `imu_node.py`. The committed values were measured at rest. |

## Known limits

- **Datum consistency.** The EKF and `gps_datum_tf` must use the same first fix. The datum node
  forwards only once the EKF subscribes and re-derives the datum if every subscriber disappears,
  so start the stack, and restart it after a crash, with the robot **stationary**. A second
  `/gps/fix` subscriber (rosbag, `ros2 topic echo`) defeats the re-derivation; then restart the
  whole stack instead.
- **Scale.** `map→odom` is rigid. Since delivery-autonomy TDM-38 the EKF's `latlon2meters`
  projects fixes onto the WGS84 tangent plane at the datum, so what is left is the UTM point
  scale of the planner's map frame (1.00028, ~2.8 cm per 100 m from the datum) and the change of
  the grid convergence across campus (~1 cm over 300 m). Before TDM-38 its spherical radius
  added 0.1–0.3 %. In the sim, the EKF's error against ground truth at the end of the 190 m route
  fell from 0.246 m to 0.002 m.
- **Yaw rate on `/odom` is synthesised** from the commanded steering angle (one drive encoder, no
  steering feedback), so that channel partly confirms the EKF's own prediction. A steering angle
  sensor is the real fix.
- **MPPI.** K=3000 rollouts × T=50 at 10 Hz; `bench_mppi` decides whether the Pi can. `K` lives in
  tangled code (`docs/20260109142710-delivery.org` in delivery-autonomy).
- **MPPI's geometry is the sim robot's.** delivery-autonomy's dynamics model and `publish_command`
  use a wheelbase of 0.5 m and a steering limit of 0.6 rad. The robot has 0.36322 m and
  0.4363 rad. The controller sends speed and yaw rate (ω = v/0.5·tan δ), and `arduino_bridge`
  turns that back into a steering angle with the real wheelbase (δ = atan(ω·0.36322/v)), so the
  curvature MPPI asked for is what the robot gets. The exception is the tightest turns: MPPI
  thinks it can turn on a 0.73 m radius, the robot only on 0.78 m. Any model angle above
  0.570 rad is clipped to the robot's 0.4363 rad, and the robot then turns up to 6 % wider than
  MPPI predicted. Nav2 plans with a 1.0 m minimum radius, so this only happens in hard
  corrections. Both numbers are hardcoded in delivery-autonomy, not parameters.
- **EKF tuning** assumed the sim's noiseless GPS (`R_gps` = 3 cm) and its 100 Hz IMU. Both are
  tangled constants; expect to retune after the first logs.
- `require_joystick: true` means the robot will not move autonomously without a joystick
  connected. That is deliberate.
