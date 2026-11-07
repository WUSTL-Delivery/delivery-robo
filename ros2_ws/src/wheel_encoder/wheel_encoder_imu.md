# Wheel Encoder Node — Documentation

## Overview

This document explains the `wheel_encoder` ROS2 package: what it does, why it exists, how the software is structured, and how data flows from the physical hardware all the way to other ROS2 nodes. It is intended for anyone who is new to the project or wants to understand the full picture without diving directly into code.

---

## Background and Motivation

### What is a Wheel Encoder?

A wheel encoder is a sensor that measures how much a wheel has rotated. By counting rotations (or fractions of rotations), the robot can estimate how far it has traveled. This is one of the most fundamental sensors in mobile robotics — it provides **odometry**, meaning the robot's best guess of its own position and velocity based purely on wheel movement.

### Why Serial Communication?

The wheel encoder sensor on this robot is not read directly by the Raspberry Pi or main compute unit. Instead, a **microcontroller** (e.g., an Arduino or similar) is physically wired to the encoder hardware and handles the low-level counting. The microcontroller communicates with the main computer over a **USB serial connection**.

This is a common design pattern in robotics:
- Microcontrollers are better suited for hard real-time tasks like counting encoder pulses accurately without missing any.
- The main compute unit (running ROS2) handles higher-level logic, planning, and coordination.
- The two communicate over serial (UART over USB).

### What Replaced What?

The original file in this package was incorrectly using I2C to read a BNO085 IMU (Inertial Measurement Unit) — a completely different sensor that measures orientation and acceleration, not wheel rotation. That code did not belong in the `wheel_encoder` package and was replaced with the correct serial-based wheel encoder implementation.

---

## Package Structure

```
ros2_ws/src/wheel_encoder/
├── package.xml                          # ROS2 package metadata and dependencies
├── setup.py                             # Python package setup, registers the ROS2 node entry point
├── setup.cfg                            # ament build configuration
├── resource/
│   └── wheel_encoder                    # Marker file required by ament_python
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── wheel_encoder/
    ├── __init__.py                      # Makes this directory a Python package
    └── wheel_encoder_node.py            # The actual ROS2 node implementation
```

### Key Files Explained

**`package.xml`**
Declares this as a ROS2 package. The critical dependency here is:
```xml
<depend>python3-serial</depend>
```
This tells `rosdep` (ROS's dependency manager) to install the `pyserial` Python library, which is what allows Python code to communicate over a serial port. Without this, the node cannot open the serial connection to the microcontroller.

**`setup.py`**
Registers the node as a runnable command-line executable within ROS2:
```python
'console_scripts': [
    'wheel_encoder_node = wheel_encoder.wheel_encoder_node:main',
],
```
This means after building the workspace with `colcon build`, you can run the node with:
```bash
ros2 run wheel_encoder wheel_encoder_node
```

---

## The Node: `wheel_encoder_node.py`

### Imports

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
```

- **`rclpy`** — The ROS2 Python client library. Provides all the ROS2 machinery: initialization, spinning, shutting down.
- **`Node`** — The base class for all ROS2 nodes. Every node inherits from this.
- **`std_msgs/String`** — A standard ROS2 message type that simply wraps a Python string. Used here because the microcontroller sends back a text-based response.
- **`serial`** — From the `pyserial` library. Provides the `Serial` class that opens and manages a serial port connection.

---

### Node Initialization (`__init__`)

```python
super().__init__('wheel_encoder_node')
self.declare_parameter('port', DEFAULT_PORT)   # '/dev/motor'
self.declare_parameter('baud_rate', 57600)
self.declare_parameter('timer_period', 0.1)
```

The node is named `wheel_encoder_node` in the ROS2 graph. Three **ROS2 parameters** are declared with default values:

| Parameter | Default | Meaning |
|---|---|---|
| `port` | `/dev/motor` | The motor-controller board's udev symlink (`deployment/udev/99-motor.rules`). Not a raw `/dev/ttyUSB*`: the lidar is one too |
| `baud_rate` | `57600` | Serial communication speed in bits per second |
| `timer_period` | `0.1` | How often (in seconds) the node polls the encoder — 10 Hz |

Parameters allow these values to be changed at launch time without modifying code:
```bash
ros2 run wheel_encoder wheel_encoder_node --ros-args -p port:=/dev/ttyUSB1 -p baud_rate:=115200
```

```python
port = self.get_parameter('port').get_parameter_value().string_value
baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
period = self.get_parameter('timer_period').get_parameter_value().double_value
```

The declared parameters are then read back and stored in local variables for use during setup.

```python
self.publisher_ = self.create_publisher(String, 'wheel_encoder/data', 10)
```

Creates a ROS2 **publisher** on the topic `wheel_encoder/data`. Any other node in the system can subscribe to this topic to receive encoder data. The `10` is the queue depth — how many messages can be buffered if a subscriber is slow to process them.

```python
self.ser = serial.Serial(port, baud, timeout=0.5)
```

Opens the serial port. The `timeout=0.5` means that if the microcontroller does not respond within 0.5 seconds, `readline()` will return whatever partial data it has (or an empty string) rather than blocking forever. This prevents the node from hanging.

```python
self.timer = self.create_timer(period, self.timer_callback)
```

Creates a ROS2 timer that calls `timer_callback` every `period` seconds (default: every 0.1 seconds = 10 times per second).

---

### Timer Callback (`timer_callback`)

This is the heart of the node — it runs 10 times per second.

```python
def timer_callback(self):
    self.ser.write(b'e\n')
    line = self.ser.readline().decode('utf-8', errors='replace').strip()
    msg = String()
    msg.data = line
    self.publisher_.publish(msg)
```

**Step-by-step breakdown:**

1. **`self.ser.write(b'e\n')`**
   Sends the two-byte sequence `e` followed by a newline character (`\n`) over the serial port to the microcontroller. The `b` prefix means this is a raw bytes literal, not a Python string — serial ports operate on raw bytes, not Unicode text.

   The `'e'` command is a protocol agreed upon with the microcontroller firmware. When the microcontroller receives `e\n`, it knows to respond with the current encoder reading.

2. **`self.ser.readline()`**
   Waits for the microcontroller to send back a line of text (terminated by `\n`). Returns raw bytes.

3. **`.decode('utf-8', errors='replace')`**
   Converts the raw bytes to a Python string using UTF-8 encoding. The `errors='replace'` means if any byte cannot be decoded (e.g., corrupted data), it is replaced with the `?` character instead of crashing.

4. **`.strip()`**
   Removes any leading/trailing whitespace, including the trailing `\n` that the microcontroller appended.

5. **`msg = String(); msg.data = line`**
   Creates a ROS2 `std_msgs/String` message and sets its `data` field to the decoded encoder reading.

6. **`self.publisher_.publish(msg)`**
   Publishes the message to the `wheel_encoder/data` topic. Any subscribing node will immediately receive it.

---

### Shutdown (`destroy_node`)

```python
def destroy_node(self):
    self.ser.close()
    super().destroy_node()
```

When the node is shut down (e.g., via `Ctrl+C`), `destroy_node` is called. This explicitly closes the serial port before the process exits. This is important because:
- Leaving a serial port open can cause issues the next time you try to connect.
- On Linux, the device file (`/dev/ttyUSB0`) remains "in use" until properly released.

---

### `main` Function

```python
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
```

This is the entry point for the node:

1. **`rclpy.init()`** — Initializes the ROS2 Python runtime. Must be called before creating any nodes.
2. **`WheelEncoderNode()`** — Creates an instance of the node, which runs `__init__`, opens the serial port, and sets up the timer and publisher.
3. **`rclpy.spin(node)`** — Hands control to the ROS2 executor, which continuously processes callbacks (including the timer callback) until shutdown.
4. **`KeyboardInterrupt`** — Catches `Ctrl+C` gracefully so the `finally` block runs.
5. **`node.destroy_node()`** — Closes the serial port and cleans up the node.
6. **`rclpy.shutdown()`** — Shuts down the ROS2 Python runtime cleanly.

---

## Full Data Flow

```
Physical Wheel
     |
     | (mechanical rotation)
     v
Encoder Sensor (hardware)
     |
     | (pulse signals)
     v
Microcontroller (Arduino / similar)
     |
     | (USB cable, serial at 57600 baud)
     v
/dev/ttyUSB0  (Linux device file)
     |
     | pyserial reads/writes
     v
WheelEncoderNode (ROS2, Python)
  - Every 0.1s: sends 'e\n' to microcontroller
  - Reads back response line
  - Publishes as std_msgs/String
     |
     | ROS2 topic: wheel_encoder/data
     v
Any subscribing ROS2 node
(e.g., odometry calculator, navigation stack)
```

---

## How to Build and Run

### Build the workspace

```bash
cd ~/delivery-robo/ros2_ws
colcon build --packages-select wheel_encoder
source install/setup.bash
```

### Run the node

```bash
ros2 run wheel_encoder wheel_encoder_node
```

### Run with custom parameters

```bash
ros2 run wheel_encoder wheel_encoder_node --ros-args \
  -p port:=/dev/ttyUSB1 \
  -p baud_rate:=115200 \
  -p timer_period:=0.05
```

### Verify it is publishing

In a separate terminal (after sourcing the workspace):
```bash
ros2 topic echo /wheel_encoder/data
```

You should see `std_msgs/String` messages appearing at ~10 Hz with the encoder readings from the microcontroller.

### Check the node is running

```bash
ros2 node list
ros2 node info /wheel_encoder_node
```

---

## Common Issues

| Problem | Likely Cause | Fix |
|---|---|---|
| `wheel_encoder_node: /dev/motor does not exist (...)` | Board unplugged, or the Pi's udev rules were never installed | Plug the board in; otherwise run `sudo deployment/udev/install_udev.sh` once (deployment/README.md, "Motor-controller board") |
| `serial.SerialException: [Errno 13] Permission denied: '/dev/ttyUSB0'` | User does not have permission to access the serial port | Run `sudo usermod -aG dialout $USER` and log out/in |
| Topic publishes empty strings | Microcontroller not responding to `e\n` | Check microcontroller firmware to confirm it handles the `e` command |
| Garbage characters in published data | Baud rate mismatch | Confirm the microcontroller firmware baud rate and set `baud_rate` parameter to match |

---

## Dependencies Summary

| Dependency | Where Declared | Purpose |
|---|---|---|
| `rclpy` | implicit (ROS2 Python standard) | ROS2 Python client library |
| `std_msgs` | implicit (ROS2 standard messages) | Provides the `String` message type |
| `python3-serial` | `package.xml` | `pyserial` — serial port communication |
| `setuptools` | `setup.py` | Python package build tooling |
