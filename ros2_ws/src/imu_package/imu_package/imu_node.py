import board
import busio
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR, BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE
from adafruit_bno08x.i2c import BNO08X_I2C
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        # 50 Hz for the autonomy EKF (was a fixed 10 Hz).
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('frame_id', 'imu_link')
        rate_hz = float(self.get_parameter('rate_hz').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)

        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)
        self.get_logger().info(f"IMU node publishing imu/data at {rate_hz:g} Hz")

        i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = BNO08X_I2C(i2c)

        # Ask the sensor for reports at the publish rate (microseconds). The library only
        # exposes the interval from adafruit-circuitpython-bno08x 1.3.0; older versions are
        # fixed at 50 ms, so the values read faster than 20 Hz repeat the previous sample.
        report_interval_us = int(round(1e6 / rate_hz))
        for feature in (BNO_REPORT_ACCELEROMETER, BNO_REPORT_ROTATION_VECTOR, BNO_REPORT_GYROSCOPE):
            try:
                self.bno.enable_feature(feature, report_interval_us)
            except TypeError:
                self.bno.enable_feature(feature)
                self.get_logger().warning(
                    'adafruit_bno08x < 1.3.0: sensor reports stay at 20 Hz regardless of rate_hz '
                    '(pip install -U adafruit-circuitpython-bno08x)')


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

        msg.orientation_covariance = [1.1912989969130138e-08, 7.458051230937461e-11, -7.714913773216416e-10, 7.458051230937461e-11, 4.578213663520167e-09, -6.144561660125438e-10, -7.714913773216416e-10, -6.144561660125438e-10, 7.026383185562511e-08]
        msg.angular_velocity_covariance = [1.5105170688371545e-06, -1.1745603636778752e-08, -1.9045815937753488e-07, -1.1745603636778752e-08, 4.2638258674014977e-07, -2.2517018702222813e-08, -1.9045815937753488e-07, -2.2517018702222813e-08, 1.0818464270587167e-06]
        msg.linear_acceleration_covariance = [3.525993954008387e-06, 1.2365802041646368e-08, 8.610262162331126e-08, 1.2365802041646368e-08, 2.0331439523472155e-05, -1.1739878728426825e-06, 8.610262162331126e-08, -1.1739878728426825e-06, 0.000130014958651195]

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
