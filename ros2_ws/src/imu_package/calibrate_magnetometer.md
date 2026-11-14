# Magnetometer calibration (`calibrate_magnetometer`)

The BNO08x calibrates its own magnetometer. This tool runs that built-in routine
interactively and saves the result to the sensor's flash, so it boots calibrated
instead of re-learning after every power-on.

## Run it

1. Stop `imu_publisher` (only one program can use the I2C IMU at a time).
2. Build and run:
   ```bash
   colcon build --packages-select imu_package
   source install/setup.bash
   ros2 run imu_package calibrate_magnetometer
   ```
3. Slowly rotate the IMU through as many orientations as you can: figure-8s, then
   roll, pitch and yaw each through at least 180 degrees.
4. When "Mag accuracy" has held at 3 (High) for 5 seconds, the calibration is
   saved and the tool exits. Restart `imu_publisher`.

If it doesn't reach High within 3 minutes it exits without saving.

## Tips

- Calibrate with the IMU mounted on the robot, rotating the whole robot if you
  can. The motors, battery and frame distort the magnetic field, so a calibration
  done on the bare board won't hold once it is installed.
- Stay away from steel desks, rebar floors and laptops while calibrating.
- The autonomy EKF currently uses only the gyro's yaw rate, not the IMU's
  orientation, so this calibration matters once heading from the IMU is used.
