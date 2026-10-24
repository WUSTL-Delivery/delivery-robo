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

First-time setup on the Pi: `deployment/install_deps.sh` (pinned Python deps, then rosdep for
`joy`, `twist_mux` and the rest of every `package.xml`), then rebuild. `startup.sh` initialises
the submodule and builds with `--packages-ignore simulation`.

All parameters live in `config/hardware.yaml`. The three geometry numbers under `/**` are shared
by `arduino_bridge` and `joy_to_cmdvel` and must stay identical for both, which is why they are
declared once. The controller reads two of them too, `wheelbase` and `max_steer_rad`, for its
vehicle model. Its `controller_server` section holds the robot's other settings for the
delivery-autonomy MPPI controller, see [Controller parameters](#controller-parameters).

`autonomy_real.launch.py` arguments: `hardware_params` (default `config/hardware.yaml`),
`gps_in_topic`, `ekf_cmd_vel_topic`, `sim`, `planner_cache_dir`, and `nav2` (default `false`).
`nav2` sets the controller's `use_costmap`. master_launch passes its own `sim` and `nav2`.

## delivery-autonomy version

The submodule `ros2_ws/src/delivery-autonomy` is pinned to commit `551467a` of
delivery-autonomy's **`dev`** branch. That is our branch of the upstream code, made while its
author is away. It adds the controller parameters below, the slow-down and stop for costmap
obstacles, the taper into the path end, MPPI's costmap cost, the robot's geometry as controller
parameters, the WGS84 GPS projection, and the
sim fixes. `.gitmodules` says `branch = dev` for documentation only: `git submodule update` checks
out the pinned commit, not the branch head. Move the pin with `git submodule update --remote`
or `git update-index --cacheinfo 160000,<sha>,ros2_ws/src/delivery-autonomy`.

`dev` is pushed to the submodule's remote (`sebtheiler/delivery-autonomy`, TDM-35), so the Pi
fetches the pin on its next online boot. If that submodule update fails (offline), the Pi logs
"submodule update failed" and keeps the delivery-autonomy it already has. A Pi that has not
booted online since the push still runs upstream `2b8924b`, the previous pin, whose controller
ignores every `controller_server` value in `hardware.yaml`: it drives at a fixed 1.5 m/s, does
not read the costmap, and can reverse. Before driving autonomously, check that
`git -C ros2_ws/src/delivery-autonomy rev-parse --short HEAD` on the Pi prints `551467a`.

## Controller parameters

`autonomy_real.launch.py` starts delivery-autonomy's controller (`controller_server`, the MPPI
path follower behind `/follow_path`) with `config/hardware.yaml` as its parameter file. It does
this with `sim:=true` too, so the sim drives with the robot's values, except for the vehicle model
(below). The `controller_server` section sets only the three values that differ from the
controller's defaults. The wheelbase and steering limit come from `/**`. `use_costmap` comes from
the launch file's `nav2` argument. All but the two topics can be changed at runtime
(`ros2 param set /controller_server ...`). The controller rejects a value outside the ranges below.

| Parameter | Robot | Default | Why |
|---|---|---|---|
| `max_speed` | **1.0** | 1.5 | m/s. Cruise speed, and the cap on every forward command. 1.5 is the joystick's full stick (`pwm_limit` / `pwm_per_mps`), and `pwm_per_mps` is still a guess on an open-loop drive, so nobody knows how fast 1.5 really is. Start lower. MPPI accepts (0, 1.5]. |
| `min_speed` | **0.0** | -0.5 | m/s. The robot never reverses. Nav2's Hybrid-A\* paths are Dubins (no reversing), and nothing behind the robot is checked. At 1.5 m/s with -0.5, the TDM-33 sim runs had 3-4 stray reverse commands per run (down to -0.13 m/s). In the TDM-34 sim at 1.0 m/s, neither value produced a reverse command, and 0.0 changed neither the goal accuracy nor the box behaviour (see "Sim recipe"). Range [-0.5, 0]. |
| `stop_distance` | **0.2** | 0.35 | m. The robot stops for an inscribed or lethal costmap cell that is near the path ahead: the path passes within this distance of it, and the robot gets closer to it. The limit falls linearly from `max_speed` at `slow_distance`, never below `obstacle_creep_speed`, and is 0 at `stop_distance`. So the robot creeps up to 0.2 m from such a cell at 0.15 m/s and stops there. Until TDM-40 anything under 0.05 m/s counted as a stop, which halted it at 0.2 + 0.05 × 1.8 / 1.0 = 0.29 m. Inside 0.2 m (TDM-42) it moves only on a path that leads straight away from the nearest such cell, and no faster than `obstacle_creep_speed`, so it no longer creeps along beside an obstacle. The limit goes by every such cell of the last second of costmaps. With 0.35, an obstacle centred in a 3 m corridor (two 1.2 m gaps) stops the robot until someone moves it. With 0.2 the robot got past such a box in 3 of 8 TDM-42 sim runs (0.189-0.215 m from it) and stopped 0.196-0.209 m from it in the other 5 (see "Sim recipe"). The user approved 0.2 on 2026-10-16. |
| `use_costmap` | `nav2:=` | true | Slow down and stop for costmap obstacles near the path, and let MPPI pay for costmap cells. Set by the launch file: true only when Nav2 runs. Without a costmap the controller would cap itself at `crawl_speed`. |
| `slow_distance` | 2.0 | 2.0 | m. Full speed when the nearest obstacle near the path ahead is at least this far away, then linearly down towards 0 at `stop_distance`, but not below `obstacle_creep_speed`. |
| `obstacle_creep_speed` | 0.15 | 0.15 | m/s. The obstacle limit's floor short of `stop_distance`. The robot keeps moving at least this fast until it is at `stop_distance`, then stops. Without a floor the limit would shrink with the distance left, and the robot would never quite get there. Must be positive. The limit never exceeds `max_speed`. |
| `clearance_lookahead` | 3.0 | 3.0 | m of path ahead checked for obstacles. |
| `costmap_timeout` | 1.0 | 1.0 | s. An older costmap counts as none. Nav2 publishes at 4 Hz (`config/nav2_local.yaml`). |
| `crawl_speed` | 0.3 | 0.3 | m/s cap while `use_costmap` is on but the costmap is missing or stale. A stale costmap still stops the robot for what it shows. |
| `goal_decel` | 0.8 | 0.8 | m/s². Braking into the end of the path: speed ≤ √(2·decel·remaining). |
| `goal_creep_speed` | 0.15 | 0.15 | m/s. The taper's floor, so the robot still reaches the end. |
| `xy_goal_tolerance` | 0.25 | 0.25 | m (EKF). The path counts as done within this distance of its end... |
| `yaw_goal_tolerance` | 0.15 | 0.15 | rad. ...and this heading error. |
| `wheelbase` | **0.36322** (`/**`) | 0.5 | m. MPPI's vehicle model, and the conversion of its steering angle into the yaw rate it sends. `arduino_bridge` converts back with the same `/**` value, so the robot steers the angle MPPI planned. The sim gets the Gazebo car's 0.5. |
| `max_steer_rad` | **0.4363** (`/**`) | 0.6 | rad. MPPI samples steering within ±this, and the command is clipped to it. It is the bridge's own limit, so MPPI plans only turns the robot can make: 0.78 m radius at the tightest, where the sim car's 0.6 rad meant 0.73 m. The sim gets 0.6. Range (0, π/2). |
| `steer_tau`, `drive_tau`, `max_accel` | defaults | 0.545, 0.01, 5.0 | s, s, m/s². The model's steering lag, speed lag and acceleration limit, fitted to the Gazebo car. Nobody has measured the robot's (see "Calibration"), so it keeps the sim's. Must be positive. |
| `state_topic`, `costmap_topic` | defaults | `/state_estimation/odom`, `/global_costmap/costmap` | |

The sim car is not the robot. With `sim:=true`, `autonomy_real.launch.py` gives the controller the
Gazebo car's vehicle model (`SIM_VEHICLE` there: 0.5 m, 0.6 rad and the default lags), or
`hardware.yaml`'s `/**` values would reach the sim controller too. Check with
`ros2 param get /controller_server wheelbase`: 0.36322 on the robot, 0.5 in the sim.

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

What TDM-40 measured once the speed limit stopped at `stop_distance` itself, with an
`obstacle_creep_speed` floor of 0.15 m/s (delivery-autonomy `27d1378`). Every run used
`autonomy_real.launch.py sim:=true nav2:=true` with this file's values, on the same route as
TDM-34, with the same box removal.

| Case | Result | Goal error |
|---|---|---|
| No box | No slowdowns, mean 0.99 m/s, no reverse command. One 0.4 s stop path from a Nav2 planning failure (the slope marks above) | 0.156 m |
| 0.6 m box, centred, run 1 | Crept at 0.15 m/s from ~0.4 m, stopped 0.191 m from the box | 0.132 m |
| run 2 | Stopped at 0.209 m, crept on 0.13 m in short restarts, stopped 0.187 m from the box | 0.145 m |
| run 3 | Stopped at 0.264 m, crept on 0.44 m beside the box, stopped 0.178 m from it (0.176 m while moving) | 0.134 m |
| run 4 | Stopped at 0.184 m, crept on 1 cm, stopped 0.182 m from the box (0.181 m while moving) | 0.128 m |
| run 5 | Stopped at 0.193 m, crept on 0.12 m in short restarts, stopped 0.178 m from the box (0.174 m while moving) | 0.150 m |
| run 6 | Stopped 0.195 m from the box | 0.141 m |
| 0.6 m box, 0.45 m off-centre | Passed at 0.455 m, no slowdown | 0.151 m |

The centred box stopped the robot every time. It came to rest 0.178-0.195 m from the box,
against 0.205-0.263 m in TDM-34. It never touched the box, and it drove on only after the box
was removed. The first stop came where the controller measured 0.19-0.20 m (0.18-0.26 m by
ground truth). In 4 runs a later path ran beside the box rather than towards it, so the box no
longer counted, and the robot crept on at 0.15 m/s until the next path stopped it again. Once
the robot is that close, Smac's paths from its pose pass 0.08-0.16 m from the box's inscribed
band, and none keeps `stop_distance`. So the robot waits until the box moves. With
`stop_distance` 0.2, getting through the gap would need paths that keep further from the box,
not a different speed limit.

What TDM-42 measured after two changes. `config/nav2_local.yaml` no longer smooths Smac's paths
(`smooth_path: false`). delivery-autonomy `f0298b0` lets a robot that is already inside
`stop_distance` move only on a path that leads straight away from the nearest obstacle cell, at
`obstacle_creep_speed` at most. Its speed limit also goes by every obstacle cell of the last
second of costmaps. Same launch, values, route and box removal as TDM-40. The sim controller
read `wheelbase` 0.5 (the Gazebo car's).

| Case | Result | Closest while moving | Goal error |
|---|---|---|---|
| No box | No slowdowns or stops, mean 0.99 m/s, no reverse command | – | 0.140 m |
| 0.6 m box, 0.45 m off-centre | Passed, no slowdown | 0.473 m | 0.133 m |
| 0.6 m box, centred, run 1 | Stopped at 0.217 m, crept 1 cm, stopped again (north gap) | 0.206 m | 0.147 m |
| run 2 | Stopped (north gap), 0.196 m at rest | 0.202 m | 0.127 m |
| run 3 | Stopped (north gap) | 0.197 m | 0.126 m |
| run 4 | Passed (south gap) after stops of 0.7 s and 1.5 s | 0.189 m | 0.147 m |
| run 5 | Passed (south gap) without stopping | 0.215 m | 0.151 m |
| run 6 | Stopped at 0.205 m, crept 7 mm, stopped again (north gap) | 0.198 m | 0.141 m |
| run 7 | Passed (south gap) after a 0.5 s stop | 0.199 m | 0.135 m |
| run 8 | Stopped (north gap) | 0.209 m | 0.149 m |

The robot never touched the box. Every stop was at the stop line, and the robot drove on as
soon as the box was gone. But it got through only 3 times in 8, and only just. Four runs kept
0.2 m while moving; the other four came 1-11 mm closer. The passes came closest on the turn back
behind the box, where the rear corner swings in. The controller measures from the robot's
centre and cannot see that corner. Three things decide it:

- Which gap Smac takes. The box stands 5° askew to the corridor. Through the south gap its
  widest point is the far corner, and all 3 passes went that way. Through the north gap it is
  the near corner. The robot was still swinging out when it got there, so all 5 stops were on
  that side.
- The swing comes late. Until the robot is off to one side, the lidar sees only the box's near
  face, so Smac plans round a thin wall. Its plans are 0.6-0.7 m off the centreline 1 m before
  the box, where the middle of the gap is 0.91 m, and they turn back right behind the face. By
  the time the side shows up the robot is about 1.5 m away, too close to finish swinging out
  with a 1 m turning radius.
- The margin is smaller than the costmap's noise. The gap leaves 0.07 m to spare on each side
  of a robot that keeps 0.2 m. The controller's distance to the box (robot centre to the nearest
  inscribed cell) differed from the footprint's real clearance by -0.11 to +0.17 m from one
  costmap update to the next, -0.02 to -0.07 m as a median per run.

Without the one-second memory (same Nav2 file, 4 centred runs) the robot passed once at
0.199 m and stopped at 0.224 m and 0.211 m. In the fourth run it crept from 0.186 m to 0.170 m,
because the cells of the box's corner flickered in and out of successive costmaps.

What each planner setting did, replayed offline. 64 costmaps recorded in the TDM-34 and TDM-40
runs (robot 1.5-6 m before the centred box) were replayed into a planner_server with this
file's Smac and inflation settings plus the one change. Each plan went to the carrot
local_planner had asked for and was scored against the real box:

| Setting | Plan's footprint clearance to the box, median | Plans keeping 0.2 m | Plan's offset from the centreline abreast of the box, median (box partly / mostly seen) |
|---|---|---|---|
| Before TDM-42 (smoothing on) | 0.068 m | 5 of 64 | 0.72 / 0.80 m |
| **`smooth_path: false`** (now) | 0.122 m | 12 of 64 | 0.79 / 0.87 m |
| `cost_penalty` 4.0 / 5.0 | 0.058 / 0.063 m | 5 / 5 | 0.72 / 0.79 m |
| `use_quadratic_cost_penalty` | 0.083 m | 7 | 0.73 / 0.80 m |
| `analytic_expansion_max_cost` 50 | 0.068 m (no plan changed) | 5 | 0.72 / 0.80 m |
| `cost_scaling_factor` 1.0 / 1.5 / 3.0 | 0.022 / 0.057 / 0.084 m | 2 / 4 / 7 | 0.66-0.73 / 0.71-0.79 m |
| `non_straight_penalty` 1.5 / `change_penalty` 0.5 | 0.069 / 0.051 m | 3 / 4 | 0.70 / 0.76-0.78 m |
| No smoothing plus `cost_penalty` 2.0 or 4.0, quadratic (with `cost_penalty` 3.0 or 5.0), `cost_scaling_factor` 1.5, 3.0 or 4.0, or 3.0 with quadratic or with `cost_penalty` 4.0 | 0.103-0.134 m | 8-15 | 0.78-0.82 / 0.84-0.89 m |

The clearances are low throughout because most of the replayed costmaps show only the box's
near face. Only turning off the smoothing made a real difference. The rest moved the plans by
1-3 cm. The two that helped a little (higher scaling, quadratic) also weaken the corridor
centring that TDM-29 tuned, so they were left out. Plans took 2-8 ms on the bench (20 ms at
most) and in the sim (66 ms at most; `max_planning_time` is 1.0 s).

Getting past a centred box reliably needs one of these: a costmap that treats the unseen space
behind a marked face as unknown, so that Smac keeps out of it until the side has been seen; a
speed limit that measures from the footprint instead of the centre, so it sees the corners and
needs no slack for them; or a smaller `stop_distance`, which is the user's call.

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
| `servo_deg_per_rad`, `max_steer_rad` | Command a known yaw rate at a known speed (δ = atan(ω·L/v)), read the road-wheel angle with a protractor, take the ratio servo-degrees/road-radian. `max_steer_rad` is the mechanical limit of the linkage. `joy_to_cmdvel` and the controller read the same shared `/**` value. |
| `pwm_per_mps`, `pwm_limit` | On the ground, publish (`-r 10`) `x:` 0.3, 0.6, 0.9, 1.2, 1.5 m/s in turn and read the steady `/odom` speed for each; fit a line through PWM (= x·pwm_per_mps) versus measured speed and update `pwm_per_mps`. `pwm_limit` caps the drive (110 = today's joystick cap; firmware accepts 255). The drive is open loop, so the fit is the speed control. |
| `wheelbase` | Tape measure, rear axle centre to front axle centre. The URDF says 0.36322, the HAL 0.30. The MPPI controller's model reads the same `/**` value, so one measurement fixes both. |
| `steer_tau`, `drive_tau`, `max_accel` (`controller_server`) | Unmeasured, the controller keeps the sim car's fit. On the ground, step `/cmd_vel_out` from 0 to 1.0 m/s straight ahead: the slope at the start of the `/odom` speed is `max_accel`, the time constant of its approach to the final speed `drive_tau`. Then at a steady 0.5 m/s step the steering and fit the time constant of the yaw rate on `/imu/data` for `steer_tau`. Not `/odom`'s yaw rate: it is computed from the command (see "Known limits"). Write the values into `controller_server`. |
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
- **MPPI's lags are the sim car's.** Since delivery-autonomy TDM-41 the controller models the
  robot's wheelbase and steering limit (`wheelbase`, `max_steer_rad` from `/**`). MPPI plans only
  turns the robot can make, and the bridge steers the angle MPPI asked for. Before, it planned
  with the sim car's 0.5 m and 0.6 rad. It thought it could turn on a 0.73 m radius where the
  robot needs 0.78 m, and any model angle above 0.570 rad came out up to 6.6 % wider than
  predicted. The steering lag (0.545 s), speed lag (0.01 s) and acceleration limit (5 m/s²) are
  still the Gazebo fit. The robot's servo and open-loop drive have not been measured (see
  "Calibration"). The EKF's model also stays the sim car's. That does no harm: it turns the
  executed yaw rate back into an angle with the same 0.5 m it predicts with, so it gets the
  curvature right.
- **EKF tuning** assumed the sim's noiseless GPS (`R_gps` = 3 cm) and its 100 Hz IMU. Both are
  tangled constants; expect to retune after the first logs.
- `require_joystick: true` means the robot will not move autonomously without a joystick
  connected. That is deliberate.
