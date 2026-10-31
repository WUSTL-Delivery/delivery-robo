import argparse
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Convert quaternion to roll, pitch, yaw in radians."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw], dtype=np.float64)


class ImuCovarianceEstimator(Node):
    def __init__(self, topic: str, warmup_samples: int, target_samples: int, min_variance: float) -> None:
        super().__init__('imu_covariance_estimator')
        self.topic = topic
        self.warmup_samples = warmup_samples
        self.target_samples = target_samples
        self.min_variance = min_variance

        self.messages_seen = 0
        self.orientation_samples: List[np.ndarray] = []
        self.angular_velocity_samples: List[np.ndarray] = []
        self.linear_accel_samples: List[np.ndarray] = []
        self.finished = False

        qos = QoSProfile(depth=200)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.subscription = self.create_subscription(
            Imu,
            self.topic,
            self.imu_callback,
            qos,
        )
        self.get_logger().info(
            f"Listening on '{self.topic}'. "
            f'Warmup: {self.warmup_samples} samples, capture: {self.target_samples} samples.'
        )

    def imu_callback(self, msg: Imu) -> None:
        if self.finished:
            return

        self.messages_seen += 1
        if self.messages_seen <= self.warmup_samples:
            if self.messages_seen % max(1, self.warmup_samples // 5) == 0:
                self.get_logger().info(f'Warmup {self.messages_seen}/{self.warmup_samples}...')
            return

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        qnorm = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if qnorm > 1e-9:
            self.orientation_samples.append(quaternion_to_rpy(qx / qnorm, qy / qnorm, qz / qnorm, qw / qnorm))

        self.angular_velocity_samples.append(
            np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z], dtype=np.float64)
        )
        self.linear_accel_samples.append(
            np.array(
                [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
                dtype=np.float64,
            )
        )

        captured = len(self.linear_accel_samples)
        if captured % max(1, self.target_samples // 10) == 0:
            self.get_logger().info(f'Captured {captured}/{self.target_samples} samples...')

        if captured >= self.target_samples:
            self.finished = True
            self.report_and_shutdown()

    def _covariance_from_samples(self, values: List[np.ndarray], unwrap_angles: bool = False) -> np.ndarray:
        if len(values) < 2:
            raise RuntimeError('Need at least 2 samples to compute covariance.')

        data = np.vstack(values)
        if unwrap_angles:
            data = np.unwrap(data, axis=0)

        cov = np.cov(data, rowvar=False, ddof=1)

        # Keep diagonal non-zero to prevent downstream EKF singularities.
        for i in range(3):
            if cov[i, i] < self.min_variance:
                cov[i, i] = self.min_variance

        return cov

    @staticmethod
    def _flatten_row_major(cov: np.ndarray) -> List[float]:
        return [float(v) for v in cov.reshape(-1)]

    def report_and_shutdown(self) -> None:
        try:
            orientation_cov = self._covariance_from_samples(self.orientation_samples, unwrap_angles=True)
            angular_cov = self._covariance_from_samples(self.angular_velocity_samples)
            linear_cov = self._covariance_from_samples(self.linear_accel_samples)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            rclpy.shutdown()
            return

        orientation_flat = self._flatten_row_major(orientation_cov)
        angular_flat = self._flatten_row_major(angular_cov)
        linear_flat = self._flatten_row_major(linear_cov)

        self.get_logger().info('')
        self.get_logger().info('=== Estimated IMU Covariances ===')
        self.get_logger().info(f'Collected messages: {self.messages_seen}')
        self.get_logger().info(f'Used samples: {len(self.linear_accel_samples)}')
        self.get_logger().info(f'orientation_covariance: {orientation_flat}')
        self.get_logger().info(f'angular_velocity_covariance: {angular_flat}')
        self.get_logger().info(f'linear_acceleration_covariance: {linear_flat}')
        self.get_logger().info('Paste these arrays into your IMU publisher node.')

        with open('imu_covariance_results.txt', 'w') as file:
            file.write(f'orientation_covariance: {orientation_flat}')
            file.write(f'angular_velocity_covariance: {angular_flat}')
            file.write(f'linear_acceleration_covariance: {linear_flat}')
        print('\n# YAML-style block for copy/paste')
        print(f'orientation_covariance: {orientation_flat}')
        print(f'angular_velocity_covariance: {angular_flat}')
        print(f'linear_acceleration_covariance: {linear_flat}')

        rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Estimate sensor_msgs/Imu covariance matrices from a live IMU topic.'
    )
    parser.add_argument('--topic', default='imu/data', help='IMU topic name (default: imu/data)')
    parser.add_argument(
        '--warmup-samples',
        type=int,
        default=200,
        help='Number of initial samples to ignore while sensor settles',
    )
    parser.add_argument(
        '--samples',
        type=int,
        default=2000,
        help='Number of samples used to estimate covariance',
    )
    parser.add_argument(
        '--min-variance',
        type=float,
        default=1e-9,
        help='Minimum diagonal variance floor',
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.samples < 2:
        raise ValueError('--samples must be at least 2')
    if args.warmup_samples < 0:
        raise ValueError('--warmup-samples must be non-negative')

    rclpy.init()
    node = ImuCovarianceEstimator(
        topic=args.topic,
        warmup_samples=args.warmup_samples,
        target_samples=args.samples,
        min_variance=args.min_variance,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
