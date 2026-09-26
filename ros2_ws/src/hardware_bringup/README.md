# hardware_bringup

Glue between the real robot and the `delivery-autonomy` stack (checked out as the submodule
`ros2_ws/src/delivery-autonomy`, never modified). It implements
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
declared once.

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
| `wheelbase` | Tape measure, rear axle centre to front axle centre. The URDF says 0.36322, the HAL 0.30, the autonomy model 0.5 (the autonomy's own 0.5 stays: the bridge absorbs the difference). |
| IMU covariances | `ros2 run imu_package imu_covariance_estimator` while driving, then paste into `imu_node.py`. The committed values were measured at rest. |

## Known limits

- **Datum consistency.** The EKF and `gps_datum_tf` must use the same first fix. The datum node
  forwards only once the EKF subscribes and re-derives the datum if every subscriber disappears,
  so start the stack, and restart it after a crash, with the robot **stationary**. A second
  `/gps/fix` subscriber (rosbag, `ros2 topic echo`) defeats the re-derivation; then restart the
  whole stack instead.
- **Scale.** `map→odom` is rigid. UTM point scale (1.00028) and the equirectangular radius error in
  the EKF's `latlon2meters` (0.1–0.3 %) leave up to ~0.3 m per 100 m from the datum. Keep test
  legs short and re-start near the goal area.
- **Yaw rate on `/odom` is synthesised** from the commanded steering angle (one drive encoder, no
  steering feedback), so that channel partly confirms the EKF's own prediction. A steering angle
  sensor is the real fix.
- **MPPI.** K=3000 rollouts × T=50 at 10 Hz; `bench_mppi` decides whether the Pi can. `K` lives in
  tangled code (`docs/20260109142710-delivery.org` in delivery-autonomy).
- **EKF tuning** assumed the sim's noiseless GPS (`R_gps` = 3 cm) and its 100 Hz IMU. Both are
  tangled constants; expect to retune after the first logs.
- `require_joystick: true` means the robot will not move autonomously without a joystick
  connected. That is deliberate.
