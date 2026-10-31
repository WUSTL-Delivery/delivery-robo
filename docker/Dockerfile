# syntax=docker/dockerfile:1
# Reproducible ROS 2 Jazzy environment for ros2_ws (docker/README.md).
#
# Ubuntu is pinned by digest and every ROS package comes from one dated snapshot, so the ROS core
# and everything rosdep adds always come from the same apt sync (a mixed install is TDM-46's
# crash). pip packages come from deployment/requirements.lock. Targets: dev (laptops), runtime (Pi).
ARG UBUNTU_DIGEST=sha256:fe81560990fae86dc4743202b2cb4d3df98d0bcf949173cecef15ce1127ef852

FROM ubuntu:noble@${UBUNTU_DIGEST} AS base
ARG ROS_SNAPSHOT=2026-10-05
ENV ROS_DISTRO=jazzy LANG=C.UTF-8 LC_ALL=C.UTF-8 DEBIAN_FRONTEND=noninteractive
COPY docker/ros-snapshots.asc /usr/share/keyrings/ros-snapshots.asc
RUN echo "deb [signed-by=/usr/share/keyrings/ros-snapshots.asc]" \
         "http://snapshots.ros.org/jazzy/${ROS_SNAPSHOT}/ubuntu noble main" \
         > /etc/apt/sources.list.d/ros-snapshot.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-base ros-dev-tools build-essential python3-pip git \
 && rm -rf /var/lib/apt/lists/*

# Only the package.xml files, so the dependency layers below rebuild only when dependencies change.
FROM base AS manifests
COPY ros2_ws/src /src
RUN mkdir /manifests && cd /src && find . -name package.xml -exec cp --parents {} /manifests \;

# Same order as deployment/install_deps.sh: the hash-pinned pip set first, then rosdep, which sees
# the pip keys in rosdep.yaml as installed. PIP_BREAK_SYSTEM_PACKAGES stays unset so rosdep refuses
# (PEP 668) rather than pip-installing an unpinned version if a pip key were ever missing.
# --ignore-installed: pip installs into /usr/local, ahead of apt's copies on sys.path, without
# removing them (the Pi gets the same layering from --user).
FROM base AS deps
COPY deployment/requirements.lock deployment/rosdep.yaml /deps/
RUN pip install --no-cache-dir --break-system-packages --ignore-installed --require-hashes \
      -r /deps/requirements.lock
COPY --from=manifests /manifests /deps/src
RUN rosdep init \
 && echo "yaml file:///deps/rosdep.yaml" > /etc/ros/rosdep/sources.list.d/50-delivery-robo.list \
 && rosdep update --rosdistro jazzy \
 && apt-get update \
 && rosdep install --from-paths /deps/src --ignore-src -y --rosdistro jazzy \
 && rm -rf /var/lib/apt/lists/*

# Laptops: the repo is bind-mounted at /home/dev/delivery-robo; build and test inside.
FROM deps AS dev
ARG UID=1000 GID=1000
RUN userdel -r ubuntu \
 && groupadd -g "$GID" dev && useradd -m -u "$UID" -g "$GID" -s /bin/bash dev \
 && apt-get update && apt-get install -y --no-install-recommends sudo \
 && rm -rf /var/lib/apt/lists/* \
 && echo "dev ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/dev \
 && echo "source /opt/ros/jazzy/setup.bash" >> /home/dev/.bashrc
USER dev
WORKDIR /home/dev/delivery-robo

# The Pi: the built workspace, started from deployment/robot_config.yaml (robot-entrypoint.sh).
FROM deps AS runtime
COPY ros2_ws/src /ws/src
RUN . /opt/ros/jazzy/setup.sh && cd /ws && colcon build --packages-ignore simulation \
 && rm -rf build log
COPY docker/robot-entrypoint.sh /usr/local/bin/robot-entrypoint.sh
ARG GIT_SHA=unknown
ENV REPO=/repo GIT_SHA=${GIT_SHA}
ENTRYPOINT ["/usr/local/bin/robot-entrypoint.sh"]
