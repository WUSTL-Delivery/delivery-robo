# Encoder bench test and gamepad checkout

## Automatic one-revolution Joy test

`wheel_rotation_test` publishes one `sensor_msgs/Joy` with left-stick Y = 0.1
by default, watches `/wheel_ticks` (`std_msgs/Int64` from arduino_bridge), and
publishes all-zero Joy when the signed net count reaches a measured wheel revolution.
The drive command is published ONCE; stop messages repeat at 20 Hz and the node
stays stopped until explicitly restarted. It has no joystick subscription.
Status is published on `/encoder_test/rotation_status` as JSON in `std_msgs/String`.

The older `wheel_encoder_node` publishes **`/wheel_encoder/data`**, type
`std_msgs/msg/String` (unless launched in a namespace). To inspect it on the Pi:

```bash
ros2 topic list -t | grep -E 'wheel|encoder'
ros2 topic echo /wheel_encoder/data
```

The rotation node can read that format with ROS parameters
`-p encoder_topic:=/wheel_encoder/data -p encoder_message_type:=string`.
The default remains `/wheel_ticks`, type Int64, for the dedicated bridge launch.
Bare integer strings are accepted; debug text and `No response received` are not
counts and cause a running trial to stop. Do not run the old serial reader together
with arduino_bridge just to supply this topic. This alternative input does not
change the isolated Joy output or make the legacy drive node safe to co-launch.

Use the dedicated launch below, not the normal joystick/autonomy launch. It uses
private test command topics and no physical gamepad driver or autonomy nodes.
Its mapper accepts zero-button Joy as stop and holds the initial command for at
most 5 seconds. The normal mapper's enable-button behavior is unchanged.
If the test node dies, the mapper times out within 5 seconds; the bridge also has
its existing 0.5 s watchdog if commands from the mapper/mux cease. This is a
bounded one-shot latch, not a short heartbeat from the test node.

First measure ticks per WHEEL revolution with the hand-turned procedure below.
No default calibration is supplied. A motor shaft revolution is not necessarily
a wheel revolution. The default throttle is a normalized stick fraction, NOT a
measured speed. It may be insufficient to overcome motor friction; a stall is
reported as a time-limit fault rather than silently increasing throttle.

After building this package on the Pi, stop the master/service/other serial owners,
support the driven wheels clear of the ground, verify servo centre calibration,
and keep the physical motor stop accessible. This launch automatically starts
once fresh stationary encoder counts have arrived for a second and a Joy
subscriber exists. Do not launch it until the bench is ready.

```bash
sudo systemctl stop robot.service
source /opt/ros/jazzy/setup.bash
source ~/delivery-robo/ros2_ws/install/setup.bash
# Replace both placeholders with actual measured/configured values:
ros2 launch hardware_bringup wheel_rotation_test.launch.py \
  ticks_per_rotation:=YOUR_MEASURED_INTEGER \
  port:=/dev/serial/by-id/YOUR_ARDUINO_ID \
  throttle:=0.1 encoder_direction:=1
```

Set encoder_direction:=-1 if this commanded direction DECREASES the count.
It faults and requests stop on any opposite count movement, a count jump larger
than one rotation, no encoder report for 0.3 s, or no completed rotation within
5 s. Startup without a subscriber/settled encoder times out after 15 s. Counter
resets cannot always be distinguished from movement; don't reset/reconnect mid-test.
The earlier twist_mux missing-symbol issue must be fixed before powered use.

The threshold means **stop requested**, not proof the wheel physically stopped
at exactly 360 degrees. Message delay and inertia cause overshoot. Inspect the
wheel mark/recording and keep encoder_bench running separately to measure final
settled movement. The test intentionally does not add reverse braking.

Do not run the legacy wheel_encoder_node/joystick_node alongside the bridge:
all compete for the same serial port. This launch reads the encoder through the
existing bridge rather than introducing another serial reader.

These tools are a test harness, not a claim that the physical encoder or controller
has passed. Do not flash firmware or change calibration until measurements agree.

## What this separates

* Counts per wheel turn: mark a tire and chassis reference, take up drivetrain slack
  in the test direction, then measure 10 complete turns without reversing.
* Mechanical play: separately reverse slowly and measure how far the wheel moves
  before the count changes. Use a ruler at the tire or video with a visible mark.
  Encoder mounting determines whether play appears before or after the sensor;
  count data alone cannot measure backlash or tire slip.
* Delivery timing: host receipt intervals show stalls/bunching, not true sensor
  latency. `/wheel_ticks` is an unstamped Int64; firmware returns a bare count.
  True latency needs timestamped/sequence-numbered firmware samples (with clock
  synchronization) or an external timing reference. Do not label receive gaps latency.

The firmware maintains one signed cumulative counter (pins 2 and 3, quadrature
CHANGE interrupts), not separately reported left/right encoders. Identify the shaft
it measures before interpreting turns. Both wheels may need to turn together if
the sensor is upstream of a differential; spinning one free wheel can mislead.

## Build on the Pi after copying/checking out these changes

```bash
cd ~/delivery-robo/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select hardware_bringup
source install/setup.bash
```

## Hand-turned test: no gamepad or twist_mux required

Disable motor/steering power while keeping encoder and Arduino logic powered.
Stop the master launch (Ctrl+C) and `sudo systemctl stop robot.service`.
There must be only ONE serial owner: do not run wheel_encoder_node,
joystick_node, the legacy HAL, or a second arduino_bridge alongside this.
The bridge sends zero-drive/centering commands and opening serial resets the Arduino,
so motor-power isolation matters even when turning the wheel by hand.

Use the actual Arduino by-id path from `ls -l /dev/serial/by-id/`:

```bash
ros2 run hardware_bringup arduino_bridge --ros-args \
  --params-file "$(ros2 pkg prefix hardware_bringup)/share/hardware_bringup/config/hardware.yaml" \
  -p port:=/dev/serial/by-id/REPLACE_WITH_ARDUINO_ID \
  -p cmd_vel_topic:=/encoder_test/unused_command
```

In another sourced Pi terminal:

```bash
ros2 run hardware_bringup encoder_bench --turns 10 --output ~/encoder-tests/forward-01
```

1. Take up slack in the intended direction. Align the tire mark, hold still for at
   least one second, type `start` and press Enter. Wait for START confirmation.
2. Rotate exactly 10 turns in ONE direction, ending at the mark. Hold still for
   at least one second, type `finish`, press Enter. If not settled, retry.
3. Repeat three times forward, then three times backward in separate output
   directories (positive `--turns 10` for either direction). Take up slack before
   each start. Do not reset/reconnect the encoder during a trial.
4. Repeat at a different hand-turning speed. Similar ticks/turn but uneven speed
   estimates suggests timing; different ticks/turn suggests counting, mechanics,
   wiring, or inconsistent wheel movement. This is diagnostic evidence, not proof.

The tool saves every received count in `samples.csv` and the completed trial in
`summary.json`. Interrupted runs retain raw CSV but have no completed summary.
The CSV includes setup samples; summary indices identify the trial endpoints.
It never opens serial, publishes commands, or updates hardware.yaml.

For a stationary-noise trial omit `--turns`, start, leave untouched for 30 seconds,
then finish. Net ticks AND absolute tick travel should be zero; cancelling changes
can give zero net even when the count jitters.

For distance calibration, push straight FORWARD over a measured 5-10 m with the
robot carrying its normal load, after taking up slack:

```bash
ros2 run hardware_bringup encoder_bench --distance 5 --output ~/encoder-tests/push-01
```

Use exactly the entered distance. Candidate metres/tick = measured forward distance
/ signed net ticks. A negative result means forward travel decreases the count;
the bridge supports signed calibration. Repeat before changing `metres_per_tick`.
Loaded rolling distance is preferable to deriving this from an unloaded tire diameter.

## Statistics

* net_ticks: signed sum of successive counter differences, with 32-bit rollover.
* absolute_tick_travel: sum of absolute differences. Larger than abs(net_ticks)
  shows observed reversals/jitter; sub-poll reversals may be invisible.
* positive_ticks / negative_ticks: observed changes in each direction.
* mean_receive_hz, p95/max_receive_gap_ms: host message timing INCLUDING rests.
* gaps_over_150ms: count of unusually long gaps relative to nominal 20 Hz polling;
  this is a heuristic, not a certified pass threshold or missed-encoder-pulse count.
* max_abs_tick_step: helps spot resets/discontinuities; resets are not automatically
  distinguishable from real motion. Any reset invalidates calibration.
* calibration candidates: measured distance or turns divided into net count.
  Zero net ticks gives null, not a valid calibration.

Compare abs(ticks/turn) across trials and report min/max and percentage spread.
Choose acceptance tolerance based on required odometry accuracy and measurement
precision; don't quietly fit calibration to a single run.

## Handheld controller

Suggested bench candidate: wired Logitech F310, rear switch in X (XInput) mode,
directly connected to the Pi. This avoids Bluetooth pairing as a variable; it is
not yet verified on this robot. Try an existing USB gamepad before purchasing one.
ROS joy supports SDL-recognized devices, but `joy_node` button indices vary.
Use the same `joy_node` as the current drive launch so the mapping check transfers.

```bash
sudo apt install ros-jazzy-joy
ros2 run joy joy_enumerate_devices
ros2 launch hardware_bringup controller_check.launch.py device_id:=0
```

Use the ID actually listed. This launch starts only joy and the mapping node,
with `/encoder_test/joy` and `/encoder_test/cmd_vel_preview`; it does not launch
the Arduino bridge or mux. It can be used with motor hardware disconnected.

In other sourced terminals:

```bash
ros2 topic echo /encoder_test/joy
ros2 topic echo /encoder_test/cmd_vel_preview
```

Move one stick axis at a time; press one button at a time. Default candidates are
throttle axis 1, steer axis 0, enable button 4. Verify, don't assume labels match.
Change launch arguments `axis_throttle`, `axis_steer`, `enable_button` as needed.
Hold enable and move the stick: preview should change. Center it while holding
enable: preview should be zero. Release enable: output stops (not a zero message).
Unplug USB: after joy_timeout (1 s default), mapping publishes zeros.
Check direction signs before applying mapping to hardware.yaml.

Once verified, copy the measured mapping to hardware.yaml and rebuild. Powered
bench testing uses ONLY drive.launch.py, with autonomy/master launch stopped,
properly supported wheels, verified servo centre, and a tested physical stop.
Use a conservative max_speed/PWM limit chosen for the actual drivetrain;
the preview's 0.2 is only a software command scale, not calibrated physical speed.
In full autonomy, releasing enable allows autonomy to resume after the mux timeout;
the enable button is NOT a physical emergency stop. Do not bypass twist_mux to
work around its earlier missing-symbol error; powered tests depend on fixing it.

Sources: https://index.ros.org/p/joy/ and
https://www.logitechg.com/en-us/shop/p/f310-gamepad

## Known timing issues to investigate on the actual robot

The old wheel_encoder_node reads with a 0.5 s serial timeout and clears its input
buffer each poll. Use arduino_bridge for this test instead. The bridge polls at
20 Hz and drains its reader queue on the polling timer, so up to roughly one poll
period of additional publication delay can occur under normal scheduling.
Current source has nonblocking battery sampling, but an older flashed sketch may
still have the previous blocking battery code. Compare flashed firmware provenance;
do not assume repository source is what the Arduino is running.
