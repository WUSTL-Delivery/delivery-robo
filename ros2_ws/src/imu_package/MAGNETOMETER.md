# Read the BNO08x magnetometer over SSH (ROS 2 Jazzy)

The existing `imu_package imu_publisher` now enables the calibrated magnetometer
report and publishes `/imu/mag` (`sensor_msgs/msg/MagneticField`) at 10 Hz.
It still publishes `/imu/data` at 50 Hz. Both use the same I2C connection.
No motor command is published. Run the driver natively on the Pi; Docker on the
laptop is for software tests and cannot read the Pi's physical I2C bus.

## Install from GitHub

The implementation is on `feat/nav2-local-planner` in WUSTL-Delivery/delivery-robo.
Connect with `ssh delivery@delivery.local`. If you previously applied the bench
patch, preserve/reconcile that local change before switching/pulling; do not
discard unrelated work. From a clean checkout on the Pi:

```bash
cd ~/delivery-robo
git fetch origin
git switch feat/nav2-local-planner
git pull --ff-only origin feat/nav2-local-planner
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-select imu_package --symlink-install
source install/setup.bash
```

For Docker, rebuild the image instead of the native colcon step above.

## Start one driver

If the normal robot service owns the IMU, stop it while the robot is parked:

```bash
sudo systemctl stop robot.service
ros2 node list
```

Also stop any manually launched IMU process before starting this one. DDS domains
can hide other nodes; a separate domain does not give exclusive I2C ownership.
Do not run two IMU drivers against the same sensor.

In SSH terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source ~/delivery-robo/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=89
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 run imu_package imu_publisher --ros-args -p mag_rate_hz:=10.0
```

In SSH terminal 2, connected to the **same Pi**:

```bash
source /opt/ros/jazzy/setup.bash
source ~/delivery-robo/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=89
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 topic echo /imu/mag
```

Read `magnetic_field.x`, `.y`, `.z`. They are tesla: `0.000025` T is 25 microtesla.
Rotate the sensor/robot by hand and watch the components change. These are
sensor-frame field components, not a compass heading or a global-frame direction.
Motor currents and nearby metal may distort the readings. Covariance is zero
to signal **unknown**, rather than claiming measured accuracy. This change does
not add magnetometer fusion to the localization filter.

The header uses the ROS readout time; the driver does not expose the original
magnetometer report timestamp here. Its cached readings can repeat; topic rate
measures publication rate, not proof of distinct sensor samples.

Ctrl-C stops echo. Check publication frequency or save readings:

```bash
ros2 topic hz /imu/mag
# Ctrl-C before starting the next command:
ros2 bag record -o "$HOME/mag-$(date +%Y%m%d-%H%M%S)" /imu/mag /imu/data
```

Ctrl-C stops recording and closes the bag. Stop the test publisher before
restarting the normal stack. `sudo systemctl start robot.service` runs the
configured robot mode, so only do that when ready to restore normal operation.
The service's update step may refuse to fast-forward while this local patch is
present; commit/merge the driver change through the team's normal workflow later.

## Record CSV for autonomy development

Publication is enabled by default. CSV recording is opt-in with `mag_csv_path`:

```bash
ros2 run imu_package imu_publisher --ros-args \
  -p mag_csv_path:="$HOME/magnetometer/reading-$(date +%Y%m%d-%H%M%S).csv" \
  -p mag_record_limit:=36000
```

Run this instead of, not alongside, the earlier driver command. Each row contains
`stamp_sec,stamp_nanosec,frame_id,mag_x_t,mag_y_t,mag_z_t` from the published
message. The vector is calibrated sensor data in tesla; covariance remains unknown.
Invalid samples are neither published nor recorded. Rows are flushed after each
write and the file closes at shutdown. An existing filename is refused rather
than overwritten. The default cap of 36,000 rows is roughly one hour at 10 Hz;
after it is reached, publication continues. Disk write errors stop recording and
log an error while publication continues. This is a bench recorder: synchronous
file writes can delay callbacks on slow storage, so leave recording disabled for
timing-sensitive autonomy runs and use a separate rosbag recorder as needed.

For persistent recordings in Docker, stop the previous test driver, then mount
a Pi folder into the new container (all commands run on the Pi):

```bash
sudo docker stop imu-test  # if the previous test container is still running
cd ~/delivery-robo
mkdir -p "$HOME/magnetometer"
sudo docker compose run -d --build --no-deps --name imu-record \
  -e ROS_DOMAIN_ID=89 \
  -v /proc/device-tree:/proc/device-tree:ro \
  -v "$HOME/magnetometer:/recordings" \
  --entrypoint /bin/bash robot -lc \
  'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && exec ros2 run imu_package imu_publisher --ros-args -p mag_csv_path:=/recordings/mag-$(date +%Y%m%d-%H%M%S).csv'
sudo docker logs --tail 80 imu-record
```

Files survive container removal in `~/magnetometer` on the Pi. For later sessions
use a new container name or remove the stopped `imu-record` container first.
To download CSVs, run on Windows:

```powershell
scp "delivery@delivery.local:~/magnetometer/*.csv" .
```

Future autonomy nodes can subscribe to `/imu/mag` using `sensor_msgs/MagneticField`.
Verify mounting/TF, magnetic interference, calibration and covariance before using
these readings as a heading observation. No new EKF fusion is enabled by this change.

## Troubleshooting

- Check `ls -l /dev/i2c-*` and `id`: I2C must be enabled and your user must have
  access. Use your Pi's existing I2C setup; no wiring changes are needed if
  `/imu/data` already works.
- Missing `adafruit_bno08x`, `board`, `busio`, or `lgpio`: the repository's
  `deployment/install_deps.sh` installs the hardware dependencies. Run it from
  `~/delivery-robo`, then rebuild/source the package. Do not run the node with
  `sudo` as a workaround for a different Python environment.
- No `/imu/mag`: confirm both terminals use domain 89 and the rebuilt workspace,
  check the publisher's log, and use `ros2 topic info /imu/mag --verbose`.
- Disable the extra report with `-p publish_magnetometer:=false` if you need the
  previous IMU-only behavior. `frame_id` defaults to `imu_link`; keep its axes
  consistent with the physical sensor mounting and TF.

References: [Adafruit example and microtesla units](https://docs.circuitpython.org/projects/bno08x/en/latest/examples.html),
[ROS MagneticField definition](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/MagneticField.msg).
