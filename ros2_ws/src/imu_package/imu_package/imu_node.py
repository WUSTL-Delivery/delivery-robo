import csv
import math
from pathlib import Path

from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField


def open_sensor():
    """Open the Pi I2C device only when starting the hardware node."""
    import board
    import busio
    return BNO08X_I2C(busio.I2C(board.SCL, board.SDA))


def magnetic_message(values_ut, stamp, frame_id):
    """Convert the BNO08x calibrated microtesla vector to a ROS tesla vector."""
    if values_ut is None or len(values_ut) != 3:
        raise ValueError('No three-axis magnetometer sample available')
    if not all(math.isfinite(value) for value in values_ut):
        raise ValueError('Non-finite magnetometer sample')
    msg = MagneticField()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z = (
        float(value) * 1e-6 for value in values_ut)
    # All zeros means covariance unknown, not a measured zero uncertainty.
    msg.magnetic_field_covariance = [0.0] * 9
    return msg


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        # 50 Hz for the autonomy EKF (was a fixed 10 Hz).
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_magnetometer', True)
        self.declare_parameter('mag_rate_hz', 10.0)
        self.declare_parameter('mag_csv_path', '')
        self.declare_parameter('mag_record_limit', 36000)
        self.mag_file = None
        self.mag_rows = 0
        self.mag_record_limit = self.get_parameter('mag_record_limit').value
        if self.mag_record_limit <= 0:
            raise ValueError('mag_record_limit must be positive')
        csv_path = self.get_parameter('mag_csv_path').value
        if csv_path and not self.get_parameter('publish_magnetometer').value:
            raise ValueError('Recording requires publish_magnetometer:=true')
        rate_hz = float(self.get_parameter('rate_hz').value)
        mag_rate_hz = float(self.get_parameter('mag_rate_hz').value)
        for name, value in (('rate_hz', rate_hz), ('mag_rate_hz', mag_rate_hz)):
            if not math.isfinite(value) or not 0 < value <= 100:
                raise ValueError(f'{name} must be finite and in (0, 100]')
        self.frame_id = self.get_parameter('frame_id').value
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)

        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)
        self.get_logger().info(f'IMU node publishing imu/data at {rate_hz:g} Hz')

        self.bno = open_sensor()

        # Ask the sensor for reports at the publish rate (microseconds). The library only
        # exposes the interval from adafruit-circuitpython-bno08x 1.3.0; older versions are
        # fixed at 50 ms, so the values read faster than 20 Hz repeat the previous sample.
        report_interval_us = int(round(1e6 / rate_hz))
        for feature in (BNO_REPORT_ACCELEROMETER, BNO_REPORT_ROTATION_VECTOR,
                        BNO_REPORT_GYROSCOPE):
            try:
                self.bno.enable_feature(feature, report_interval_us)
            except TypeError:
                self.bno.enable_feature(feature)
                self.get_logger().warning(
                    'adafruit_bno08x < 1.3.0: sensor reports stay at 20 Hz regardless of rate_hz '
                    '(pip install -U adafruit-circuitpython-bno08x)')

        if self.get_parameter('publish_magnetometer').value:
            self.mag_publisher = self.create_publisher(MagneticField, 'imu/mag', 10)
            try:
                self.bno.enable_feature(BNO_REPORT_MAGNETOMETER, int(round(1e6 / mag_rate_hz)))
            except TypeError:
                self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
            self.mag_timer = self.create_timer(1.0 / mag_rate_hz, self.magnetic_callback)
            self.get_logger().info(f'Publishing imu/mag in tesla at {mag_rate_hz:g} Hz')
            if csv_path:
                path = Path(csv_path).expanduser()
                path.parent.mkdir(parents=True, exist_ok=True)
                # Exclusive creation prevents accidental overwrite of an earlier experiment.
                self.mag_file = path.open('x', newline='', encoding='utf-8')
                self.mag_writer = csv.writer(self.mag_file)
                self.mag_writer.writerow(['stamp_sec', 'stamp_nanosec', 'frame_id',
                                          'mag_x_t', 'mag_y_t', 'mag_z_t'])
                self.mag_file.flush()
                self.get_logger().info(
                    f'Recording magnetometer to {path} (limit {self.mag_record_limit} samples)')

    def magnetic_callback(self):
        """Publish magnetic field without stopping IMU publication on a bad mag read."""
        try:
            values = self.bno.magnetic
            msg = magnetic_message(values, self.get_clock().now().to_msg(), self.frame_id)
        except (OSError, RuntimeError, ValueError) as exc:
            self.get_logger().warning(f'Magnetometer sample unavailable: {exc}',
                                      throttle_duration_sec=5.0)
            return
        self.mag_publisher.publish(msg)
        self.record_magnetic(msg)

    def record_magnetic(self, msg):
        """Record published samples, limiting disk use without stopping the ROS topic."""
        if self.mag_file is None:
            return
        try:
            self.mag_writer.writerow([
                msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id,
                msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])
            self.mag_file.flush()
            self.mag_rows += 1
            if self.mag_rows >= self.mag_record_limit:
                self.get_logger().info('Magnetometer recording limit reached; topic continues.')
                self.close_recording()
        except OSError as exc:
            self.get_logger().error(f'Magnetometer recording stopped: {exc}; topic continues.')
            self.close_recording()

    def close_recording(self):
        """Close the CSV even after a disk write failure."""
        stream, self.mag_file = self.mag_file, None
        if stream is not None:
            try:
                stream.close()
            except OSError as exc:
                self.get_logger().error(f'Could not close magnetometer CSV: {exc}')

    def destroy_node(self):
        """Flush and close the recording when shutting down the node."""
        self.close_recording()
        return super().destroy_node()

    def timer_callback(self):
        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
        accel_x, accel_y, accel_z = self.bno.acceleration
        gyro_x, gyro_y, gyro_z = self.bno.gyro

        msg.orientation.x = quat_i
        msg.orientation.y = quat_j
        msg.orientation.z = quat_k
        msg.orientation.w = quat_real

        msg.linear_acceleration.x = accel_x
        msg.linear_acceleration.y = accel_y
        msg.linear_acceleration.z = accel_z

        msg.angular_velocity.x = gyro_x
        msg.angular_velocity.y = gyro_y
        msg.angular_velocity.z = gyro_z

        msg.orientation_covariance = [
            1.1912989969130138e-08, 7.458051230937461e-11, -7.714913773216416e-10,
            7.458051230937461e-11, 4.578213663520167e-09, -6.144561660125438e-10,
            -7.714913773216416e-10, -6.144561660125438e-10, 7.026383185562511e-08]
        msg.angular_velocity_covariance = [
            1.5105170688371545e-06, -1.1745603636778752e-08, -1.9045815937753488e-07,
            -1.1745603636778752e-08, 4.2638258674014977e-07, -2.2517018702222813e-08,
            -1.9045815937753488e-07, -2.2517018702222813e-08, 1.0818464270587167e-06]
        msg.linear_acceleration_covariance = [
            3.525993954008387e-06, 1.2365802041646368e-08, 8.610262162331126e-08,
            1.2365802041646368e-08, 2.0331439523472155e-05, -1.1739878728426825e-06,
            8.610262162331126e-08, -1.1739878728426825e-06, 0.000130014958651195]

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
