# Docker environment

One image definition (`docker/Dockerfile`) for both laptops and the Pi:

- **Pinned:** Ubuntu 24.04 by digest; every ROS package from one dated snapshot of the ROS apt
  repo (`ROS_SNAPSHOT`), so the ROS core and the packages rosdep adds always come from the same
  sync; pip packages from `deployment/requirements.lock` with hashes.
- **Stages:** `deps` (ROS + every dependency the `package.xml` files declare), `dev` (laptops),
  `runtime` (the Pi: the built workspace, started from `deployment/robot_config.yaml`).

The build copies `ros2_ws/src`, so check the submodules out first:

```sh
git submodule update --init
```

## Developing on a laptop

```sh
docker compose build dev    # Linux: HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose build dev
docker compose up -d dev
docker compose exec dev bash
# inside: the repo is at ~/delivery-robo
cd ros2_ws && colcon build --packages-ignore simulation && colcon test
```

On Linux the container user needs your UID/GID so the files it writes into the checkout are yours;
macOS file sharing maps them to your user anyway, so the defaults (1000:1000) are fine there.

VS Code: "Dev Containers: Reopen in Container" uses `.devcontainer/devcontainer.json` (the same
`dev` service). The Gazebo sim and RViz are not in the image; they stay in delivery-autonomy's
`nix develop` shell (`deployment/README.md`, "Testing with the sim").

## Running on the Pi

The Pi pulls a published image; it does not build. Pin the tag you run in `.env` next to
`compose.yaml` (gitignored), e.g. `ROBOT_TAG=5cdef1e`, then:

```sh
docker compose pull robot
docker compose up -d robot            # mode/nav2/robot_id/api_url from deployment/robot_config.yaml
docker compose logs -f robot
docker compose run --rm robot ros2 topic list
```

`deployment/robot_config.local.yaml` and `deployment/ntrip.local.yaml` work as with
`startup.sh`; `sim:` is ignored (always false). The container uses the host network (DDS
discovery with laptops), runs privileged with `/dev` mounted (hot-plugged lidar, motor board,
RealSense, u-blox, I2C, joystick), and restarts unless stopped. The udev rules
(`deployment/udev/`) still have to be installed on the host. Do not run it together with
`robot.service`: both open the same devices.

## Publishing an image

Multi-arch (`linux/arm64` for the Pi, `linux/amd64` for laptops and CI), tagged with the commit:

```sh
docker login ghcr.io
docker buildx build -f docker/Dockerfile --target runtime --platform linux/arm64,linux/amd64 \
  --build-arg GIT_SHA=$(git rev-parse --short HEAD) \
  -t ghcr.io/wustl-delivery/delivery-robo:$(git rev-parse --short HEAD) --push .
```

## Bumping the pins

- **ROS packages:** set `ROS_SNAPSHOT` to a newer date from http://snapshots.ros.org/jazzy/ and
  rebuild; all ROS packages move together.
- **Ubuntu:** update `UBUNTU_DIGEST` from `docker buildx imagetools inspect ubuntu:noble`.
- **Snapshot signing key:** `docker/ros-snapshots.asc` (ROS Snapshot builder,
  `4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA`) expires 2027-06-01; refresh it from
  keyserver.ubuntu.com before then.
- **pip packages:** edit `deployment/requirements.txt` and regenerate the lock (command in its
  header).
