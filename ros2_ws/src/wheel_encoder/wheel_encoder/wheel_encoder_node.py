import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

# The motor-controller board's udev symlink (deployment/udev/99-motor.rules). Never a raw
# /dev/ttyUSBn: the RPLidar C1 is one too, and plug order decides which is ttyUSB0.
DEFAULT_PORT = '/dev/motor'


class WheelEncoderNode(Node):
    def __init__(self):
        super().__init__('wheel_encoder_node')
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baud_rate', 57600)
        self.declare_parameter('timer_period', 0.1)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
        period = self.get_parameter('timer_period').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(String, 'wheel_encoder/data', 10)
        self.ser = None
        if not os.path.exists(port):
            hint = ('the motor-controller board is unplugged, or its udev rule is not installed: '
                    'run `sudo deployment/udev/install_udev.sh` in the delivery-robo checkout'
                    if port == DEFAULT_PORT else 'check the `port` parameter')
            raise SystemExit(f'wheel_encoder_node: {port} does not exist ({hint})')
        self.ser = serial.Serial(port, baud, timeout=0.5)
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().info(f'Wheel encoder node started on {port} at {baud} baud')

    def timer_callback(self):
        self.ser.reset_input_buffer()
        self.ser.write(b'e\r')
        raw_line = self.ser.readline()
        if raw_line:
            line = raw_line.decode('utf-8', errors='replace').strip()
            try:
                line = str(float(line) * 6 / 5)
            except ValueError:
                pass
            msg = String()
            msg.data = line
            self.publisher_.publish(msg)
        else:
            msg = String()
            msg.data = "No response received"
            self.publisher_.publish(msg)

    def destroy_node(self):
        if self.ser is not None:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelEncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
