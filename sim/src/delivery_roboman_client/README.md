# Delivery Roboman client

`delivery_roboman_client` connects an order server to ROS 2 over a WebSocket.
It announces the robot to the server, publishes assigned orders for other ROS
nodes to consume, and reports that the robot is ready when a ROS node signals
delivery completion. Navigation to the pickup location is still a TODO.

The implementation is in
[`delivery_roboman_client/robot_client.py`](delivery_roboman_client/robot_client.py).
This package lives in the `sim` workspace and must be started manually; none
of the repository's launch files starts it. The
[robo-web status heartbeat](../../../deployment/README.md#status-heartbeat-robo-web)
is a separate HTTP integration in `ros2_ws` with its own configuration.

## Build and run

Use a ROS 2 environment with `colcon`, `rclpy`, `std_msgs`, and `geometry_msgs`.
The client also needs the Python package `websocket-client` (imported as
`websocket`), which is listed in [`setup.py`](setup.py). Install it in the Python
environment used by ROS before starting the node; the import check below
should succeed.

From the repository root, using the ROS 2 Jazzy installation described in the
[deployment guide](../../../deployment/README.md):

```bash
source /opt/ros/jazzy/setup.bash
python3 -c 'import rclpy, websocket; from std_msgs.msg import String; from geometry_msgs.msg import PoseStamped'
cd sim
colcon build --packages-select delivery_roboman_client --symlink-install
source install/setup.bash
ros2 run delivery_roboman_client robot_client --ros-args \
  -p robot_id:=robot_001 \
  -p ws_url:=ws://localhost:8080/ws
```

An external order server must be listening at `ws_url`; this repository does
not include that server. Replace `localhost` with the server's address when
it runs on another machine.

| ROS parameter | Default | Purpose |
| --- | --- | --- |
| `robot_id` | `robot_001` | Identity sent to the server and used to filter assignments. |
| `ws_url` | `ws://localhost:8080/ws` | WebSocket endpoint to connect to. |

These parameters are read during node construction. Pass overrides at startup;
editing `deployment/robot_config.yaml` does not configure this client.

## Message flow

```mermaid
sequenceDiagram
    participant Server as Order server
    participant Client as delivery_roboman_client
    participant Worker as ROS delivery worker (to be implemented)
    Client->>Server: WebSocket connects; status = online
    Server->>Client: Order assignment JSON
    Client->>Worker: robot/current_order (JSON inside String.data)
    Worker->>Client: robot/delivery_complete (JSON inside String.data)
    Client->>Server: status = ready
```

The delivery worker in the diagram represents a consumer to implement; this
repository does not contain a subscriber that acts on `robot/current_order`.

### Client to server: status updates

Outgoing messages use this envelope:

```json
{
  "type": "update",
  "payload": {
    "status": "online",
    "robot_id": "robot_001"
  }
}
```

| Event | Local status | WebSocket update |
| --- | --- | --- |
| Node constructed | `offline` | None. |
| Connection opens, including a reconnect | `online` | `online`. |
| Assignment accepted for this robot | `delivery` | None; only the ROS order is published. |
| Completion callback receives a JSON object while connected | `ready` | `ready`. |
| Connection closes | `offline` | None. |
| Shutdown while connected | `shutdown` | Attempts `shutdown` before closing the socket. |

Status sends are skipped while disconnected and are not queued for later.
Send failures are logged; a successful send is not an application-level
acknowledgement from the server. Updates contain no order ID or location.

### Server to client: order assignments

There is currently a field-name mismatch: `on_message()` checks for `RobotID`
and `OrderID`, but `handle_order_match()` reads `robot_id` and `order_id` from
the same top-level object. To exercise the existing implementation with an
order ID, a server message must include both spellings:

```json
{
  "RobotID": "robot_001",
  "OrderID": "order_123",
  "robot_id": "robot_001",
  "order_id": "order_123"
}
```

This is a workaround for the current client, not a verified server schema.
The capitalized fields are checked only for presence; the lowercase fields
provide the actual values. A message with only lowercase fields never reaches
the handler. A message with only capitalized fields is rejected because the
handler reads a missing `robot_id` as `None`. Assignments with a different
lowercase `robot_id` are also rejected.

An accepted assignment sets the local status to `delivery` and publishes a
`std_msgs/msg/String` on `robot/current_order`. Its `data` contains JSON such as:

```json
{
  "order_id": "order_123",
  "robot_id": "robot_001",
  "timestamp": 1787943000.0
}
```

`timestamp` comes from the client's `time.time()` at publication, in seconds
since the Unix epoch. Extra assignment fields are not forwarded. The client
does not validate a missing `order_id`, suppress duplicate assignments, or
track an active order for later completion checks.

### ROS topics and completion

All three topics use `std_msgs/msg/String`, with JSON inside `data`. Their
names are relative to the node's namespace; the examples below assume the
default namespace. Publishers and the subscription use queue depth 10.

| Topic | Direction relative to client | Current behavior |
| --- | --- | --- |
| `robot/current_order` | Publishes | One message for each accepted assignment. |
| `robot/delivery_complete` | Subscribes | Parses a JSON object, logs its `order_id`, then attempts a `ready` update. |
| `robot/status` | Publisher created | No messages during normal operation: the timer that would publish them is commented out. |

For a manual check with a test server, source ROS and `sim/install/setup.bash`
in each additional terminal. Start an order observer before sending the
assignment example from the server:

```bash
ros2 topic echo /robot/current_order std_msgs/msg/String
```

After observing the assignment, simulate completion:

```bash
ros2 topic pub --once /robot/delivery_complete std_msgs/msg/String \
  '{data: "{\"order_id\": \"order_123\"}"}'
```

The server should receive the status envelope above with `status` set to
`ready`. The completion callback does not check whether that order was
assigned to this robot; even an empty JSON object attempts the same update.
If disconnected, the update is skipped and the local status is unchanged.

## Connection behavior and troubleshooting

The WebSocket runs in a daemon thread while the main thread spins ROS. The
close callback sets the client offline, waits five seconds, and starts a new
connection attempt. Each successful connection announces `online` again;
there is no restoration of the previous delivery state or replay of missed
messages. The close callback has no shutdown guard, so it can also attempt to
reconnect after an intentional close while the process is still alive.

| Symptom | Check |
| --- | --- |
| `No module named 'websocket'` | Make `websocket-client` available to the Python interpreter running the ROS executable. |
| `ros2 run` cannot find the package or executable | Build this package in `sim`, source `sim/install/setup.bash`, and use executable name `robot_client`. |
| Connected but no order is published | Check both sets of assignment field names and that lowercase `robot_id` matches the parameter. |
| `/robot/status` exists but `ros2 topic echo` receives nothing | The status timer is disabled in the current source. |
| Server never sees the initial `ready` state | Connection sends `online`; automatic promotion to `ready` is commented out. Currently only a completion callback sends `ready`. |
| Order is published but the robot does not move | This client does not send navigation goals; a delivery worker must consume the order topic. |
