"""Calibrate the BNO08x magnetometer and save the result to the sensor's flash.

Stop imu_publisher first (only one program can use the I2C IMU), then run:

    ros2 run imu_package calibrate_magnetometer

While it runs, slowly rotate the IMU (ideally the whole robot, so the motors and
battery are included) through as many orientations as you can: figure-8s, then
roll, pitch and yaw each through 180+ degrees. Watch the "Mag accuracy" value
climb to 3 (High). Once it has stayed at High for a few seconds, the calibration
is saved so the sensor starts out calibrated on the next boot.
"""
import sys
import time

import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_GAME_ROTATION_VECTOR,
    BNO_REPORT_MAGNETOMETER,
    REPORT_ACCURACY_STATUS,
)
from adafruit_bno08x.i2c import BNO08X_I2C

HOLD_SECONDS = 5.0   # how long accuracy must stay at High before saving
TIMEOUT_SECONDS = 180.0


def main():
    i2c = busio.I2C(board.SCL, board.SDA)
    bno = BNO08X_I2C(i2c)

    bno.begin_calibration()
    bno.enable_feature(BNO_REPORT_MAGNETOMETER)
    bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)

    print('Rotate the IMU slowly through all orientations (figure-8s, roll, pitch, yaw).')
    start = time.monotonic()
    good_since = None

    while True:
        time.sleep(0.2)
        mx, my, mz = bno.magnetic          # microtesla
        status = bno.calibration_status    # 0 unreliable, 1 low, 2 medium, 3 high
        print(f'Mag: x={mx:7.2f} y={my:7.2f} z={mz:7.2f} uT | '
              f'Mag accuracy: {status} ({REPORT_ACCURACY_STATUS[status]})')

        if status == 3:
            good_since = good_since or time.monotonic()
            if time.monotonic() - good_since >= HOLD_SECONDS:
                break
        else:
            good_since = None

        if time.monotonic() - start > TIMEOUT_SECONDS:
            print('Timed out without reaching High accuracy. Nothing saved; move away from '
                  'metal/magnets and try again with larger, slower rotations.')
            sys.exit(1)

    bno.save_calibration_data()
    print('Calibration saved to the BNO08x. Restart imu_publisher.')


if __name__ == '__main__':
    main()
