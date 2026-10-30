import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import time

# The motor-controller board's udev symlink (deployment/udev/99-motor.rules). Never a raw
# /dev/ttyUSBn: the RPLidar C1 is one too, and plug order decides which is ttyUSB0.
DEFAULT_DEVICE = '/dev/motor'
MOTOR_RULES = '/etc/udev/rules.d/99-motor.rules'
UDEV_INSTALL = 'sudo deployment/udev/install_udev.sh'


def missing_port_hint(device, rules=MOTOR_RULES):
    """Say why ``device`` is missing and what to do; never suggests a raw /dev/ttyUSB*."""
    if device != DEFAULT_DEVICE:
        return f'{device} does not exist (check the `device` parameter)'
    if not os.path.exists(rules):
        return (f'{device} does not exist because its udev rule is not installed ({rules} '
                f'missing). Run `{UDEV_INSTALL}` in the delivery-robo checkout, see '
                f'deployment/README.md. Not falling back to /dev/ttyUSB*: that can be the lidar')
    return f'{device} does not exist: is the motor-controller board plugged in?'


class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        
        # Declare parameters
        self.declare_parameter('device', DEFAULT_DEVICE)
        self.declare_parameter('baud_rate', 57600)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('verbose', False)
        
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value

        self.stored_msg = None

        
        # Constants
        self.MAX_PWM = 110.0
        self.MAX_STEER = 25.0
        self.STEER_ORIGIN = 45.0
        
        # Initialize Serial; while it is closed (board unplugged, /dev/motor missing, write
        # failure) retry every second, so teleop comes back without a restart
        self.ser = None
        self.open_serial()
        self.reopen_timer = self.create_timer(1.0, self.open_serial)

        # Subscription
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)

        self.timer = self.create_timer(
            0.1, self.timer_callback
        )

        self.get_logger().info("Joystick control node (Python) started.")

    def open_serial(self):
        if self.ser is not None:
            return
        if not os.path.exists(self.device):
            self.get_logger().error(
                f'{missing_port_hint(self.device)}; retrying every second',
                throttle_duration_sec=10.0)
            return
        try:
            self.ser = serial.Serial(self.device, self.baud_rate, timeout=self.timeout)
            self.get_logger().info(f'Opened serial port: {self.device} at {self.baud_rate}')
        except Exception as e:
            self.get_logger().error(
                f'Failed to open serial port {self.device}: {e}; retrying every second',
                throttle_duration_sec=10.0)
            self.ser = None

    def joy_callback(self, msg):
        self.stored_msg = msg

    def timer_callback(self):
        # Example axes mapping from C++ code:
        # steer = axes[2] (left stick X in their code)
        # throttle = axes[1] (left stick Y)
        
        # Checking if axes are available
        if self.stored_msg is not None:
            msg = self.stored_msg
            if len(msg.axes) < 3:
                return

            steer = msg.axes[0]
            throttle = msg.axes[1]

            # Original C++ node also reads a button, though it doesn't use it.
            # Button 9 is typically "Start" or "Options" on many controllers, but it's labeled "A" in the comment.
            if len(msg.buttons) > 9:
                a_button = msg.buttons[9]

            pwm = throttle * self.MAX_PWM
            servo_angle = self.STEER_ORIGIN + steer * self.MAX_STEER
        
            if self.verbose:
                self.get_logger().info(f"Throttle: {throttle:.2f} | Steer: {steer:.2f}")
                self.get_logger().info(f"PWM: {pwm:.2f} | Angle: {servo_angle:.2f}")

            if self.ser and self.ser.is_open:
                command = f"m {pwm:.2f} {servo_angle:.2f}\r"
                try:
                    self.ser.write(command.encode())
                except Exception as e:
                    self.get_logger().error(f'Error writing to serial: {e}; reopening')
                    ser, self.ser = self.ser, None
                    try:
                        ser.close()
                    except Exception:
                        pass

def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
