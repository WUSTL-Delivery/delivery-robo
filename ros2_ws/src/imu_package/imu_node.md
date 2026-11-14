# IMU Publisher Node — Documentation

For the current magnetometer topic, SSH setup, recording commands, and units,
see [MAGNETOMETER.md](MAGNETOMETER.md). The driver now publishes `/imu/mag`
at 10 Hz alongside `/imu/data` at 50 Hz; older examples below show the original
10 Hz IMU-only implementation.

## Overview

This document explains the `imu_package` IMU publisher node: what it does, why it exists, how it is structured, and how data flows from the physical sensor into ROS2. It is intended for anyone who wants to understand how the robot's inertial measurements are exposed to the rest of the system without reading the code first.

---

## Background and Motivation

### What is an IMU?

An IMU, or Inertial Measurement Unit, is a sensor that measures the robot's motion and orientation. On this robot, the IMU provides:

- **Orientation** in quaternion form
- **Linear acceleration**
- **Angular velocity**

These measurements are used by other ROS2 nodes for localization, filtering, and state estimation.

### Why This Node Exists

The IMU hardware is a BNO08X sensor connected over **I2C**. The ROS2 node is responsible for:

- Initializing the sensor
- Reading its measurements on a fixed schedule
- Packaging the readings into a standard ROS2 message
- Publishing them on a topic that other nodes can subscribe to

This keeps the sensor-specific code isolated from the rest of the robot software.

---

## Package Structure

```
ros2_ws/src/imu_package/
├── package.xml                         # ROS2 package metadata and dependencies
├── setup.py                            # Python package setup and entry points
├── resource/
│   └── imu_package                     # Marker file required by ament_python
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── imu_package/
    ├── __init__.py                     # Makes this directory a Python package
    ├── imu_node.py                     # IMU publisher node implementation
    └── imu_covariance_estimator.py     # Utility node for estimating covariance values
```

### Key Files Explained

**`package.xml`**
Declares the ROS2 package and its runtime dependencies.

The important dependencies for the IMU publisher are:
- `rclpy` for ROS2 Python node support
- `sensor_msgs` for the `Imu` message type

**`setup.py`**
Registers the node as a runnable ROS2 executable:

```python
'console_scripts': [
    'imu_publisher = imu_package.imu_node:main',
    'imu_covariance_estimator = imu_package.imu_covariance_estimator:main',
],
```

This means the IMU publisher can be launched with:

```bash
ros2 run imu_package imu_publisher
```

---

## The Node: `imu_node.py`

### Imports

```python
import board
import busio
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR, BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE
from adafruit_bno08x.i2c import BNO08X_I2C
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
```

- **`board`** and **`busio`** come from the CircuitPython hardware stack and provide access to the Raspberry Pi's I2C pins.
- **`adafruit_bno08x`** is the sensor driver library for the BNO08X family.
- **`rclpy`** and **`Node`** provide the ROS2 runtime and node base class.
- **`sensor_msgs.msg.Imu`** is the standard ROS2 message used to publish IMU data.

---

### Node Initialization (`__init__`)

```python
super().__init__('imu_node')
self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
```

The node is named `imu_node` in the ROS2 graph and publishes `sensor_msgs/Imu` messages on the `imu/data` topic. The queue depth of `10` allows a small buffer if downstream nodes are temporarily slow.

```python
self.timer = self.create_timer(0.1, self.timer_callback)
```

A timer is created so the sensor is sampled every 0.1 seconds, which is 10 Hz.

```python
self.get_logger().info("IMU node has started publishing!!!!!!!")
```

This logs a startup message so it is obvious in the terminal that the node came up successfully.

```python
REPORT_INTERVAL = 100000
i2c = busio.I2C(board.SCL, board.SDA)
self.bno = BNO08X_I2C(i2c)
```

The node opens the I2C bus and constructs the BNO08X sensor object. `REPORT_INTERVAL` is currently defined but not used.

```python
self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
```

The node enables the sensor reports it needs:

- Accelerometer data
- Rotation vector data
- Gyroscope data

---

### Timer Callback (`timer_callback`)

The timer callback runs every 0.1 seconds and publishes one IMU message.

```python
def timer_callback(self):
    msg = Imu()
```

A fresh `sensor_msgs/Imu` message is created.

```python
msg.header.stamp = self.get_clock().now().to_msg()
msg.header.frame_id = 'imu_link'
```

The message is timestamped and assigned the `imu_link` frame. That frame ID is important for downstream consumers that interpret the sensor orientation in the robot's TF tree.

```python
quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
accel_x, accel_y, accel_z = self.bno.acceleration
gyro_x, gyro_y, gyro_z = self.bno.gyro
```

The node reads the latest quaternion, linear acceleration, and angular velocity measurements from the sensor.

```python
msg.orientation.x = quat_i
msg.orientation.y = quat_j
msg.orientation.z = quat_k
msg.orientation.w = quat_real
```

The quaternion values are copied into the standard ROS2 orientation field.

```python
msg.linear_acceleration.x = accel_x
msg.linear_acceleration.y = accel_y
msg.linear_acceleration.z = accel_z

msg.angular_velocity.x = gyro_x
msg.angular_velocity.y = gyro_y
msg.angular_velocity.z = gyro_z
```

The linear acceleration and angular velocity fields are filled in from the sensor readings.

```python
msg.orientation_covariance = [...]
msg.angular_velocity_covariance = [...]
msg.linear_acceleration_covariance = [...]
```

The node publishes fixed covariance matrices for each measurement group. These values tell downstream filters how much trust to place in the IMU data. They are especially important for state estimators and sensor fusion nodes.

```python
self.publisher_.publish(msg)
```

The completed message is published to `imu/data`.

---

### Shutdown and Entry Point

```python
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
```

This is the standard ROS2 Python entry point:

1. Initialize ROS2
2. Create the node
3. Spin until interrupted
4. Destroy the node
5. Shut down ROS2 cleanly

The current implementation does not override `destroy_node`, so there is no custom sensor cleanup beyond the default node teardown.

---

## Full Data Flow

```
Physical Motion of Robot
     |
     | (rotation, acceleration, vibration)
     v
BNO08X IMU Sensor
     |
     | (I2C bus)
     v
imu_node (ROS2, Python)
  - Reads quaternion, acceleration, and gyro data
  - Adds timestamp and frame ID
  - Publishes sensor_msgs/Imu
     |
     | ROS2 topic: imu/data
     v
Any subscribing ROS2 node
(e.g., localization, filtering, logging, covariance estimation)
```

---

## How to Build and Run

### Build the workspace

```bash
cd ~/delivery-robo/ros2_ws
colcon build --packages-select imu_package
source install/setup.bash
```

### Run the IMU publisher

```bash
ros2 run imu_package imu_publisher
```

### Verify it is publishing

In a separate terminal after sourcing the workspace:

```bash
ros2 topic echo /imu/data
```

You should see `sensor_msgs/Imu` messages at about 10 Hz.

### Check the node is running

```bash
ros2 node list
ros2 node info /imu_node
```

---

## Common Issues

| Problem | Likely Cause | Fix |
|---|---|---|
| Node fails to start with I2C errors | Sensor is not connected or I2C is not enabled | Check wiring and confirm I2C is enabled on the hardware |
| `ModuleNotFoundError` for Adafruit libraries | Python dependencies are missing | Install the BNO08X and CircuitPython support packages used by the node |
| Published orientation looks wrong | IMU mounting orientation does not match `imu_link` | Verify the sensor frame and TF tree conventions |
| Downstream filters behave poorly | Covariance values are not representative | Re-estimate the covariance values or tune the fixed covariance arrays |

---

## Dependencies Summary

| Dependency | Where Declared | Purpose |
|---|---|---|
| `rclpy` | `package.xml` | ROS2 Python client library |
| `sensor_msgs` | `package.xml` | Provides the `Imu` message type |
| `adafruit_bno08x` | Python runtime dependency | Driver for the BNO08X IMU |
| `board` / `busio` | Python runtime dependency | Hardware access for I2C pins |
| `numpy` | `setup.py` | Used by the covariance estimator utility |
