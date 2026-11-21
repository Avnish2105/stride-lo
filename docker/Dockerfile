# syntax=docker/dockerfile:1
# STRIDE LiDAR odometry: build / test / dev image.
#
# Multi-arch (amd64 laptops and CI, arm64 Jetson Orin): based on the official multi-arch
# ros:jazzy-ros-base, no -march=native anywhere.
#
# Targets
#   deps  toolchain + ROS deps + pinned underlay (/opt/stride_underlay):
#         GTSAM 4.3.0, Livox-SDK2 (/usr/local), livox_ros_driver2. Sophus in /usr/local.
#   ci    deps, default user root (CI runners mount the checkout with their own uid).
#   dev   ci + Gazebo Harmonic (ros_gz), rviz2, debugging tools, non-root user `dev`.
#
# Build through scripts/dc (sets the uid/gid and the same-path workspace mount):
#   scripts/dc build dev
ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}-ros-base AS deps
ARG ROS_DISTRO
ARG DEBIAN_FRONTEND=noninteractive
# GTSAM peaks at ~1.5 GB per compile job: keep this low on small machines.
ARG BUILD_JOBS=3
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake ninja-build ccache git curl ca-certificates pkg-config \
        clang clang-tidy clang-format gdb \
        python3-colcon-common-extensions python3-vcstool python3-pip \
        python3-numpy python3-scipy python3-matplotlib python3-yaml \
        libeigen3-dev libfmt-dev libgtest-dev libgmock-dev libyaml-cpp-dev libboost-dev \
        libapr1-dev libaprutil1-dev libpcl-dev \
        ros-${ROS_DISTRO}-rclcpp-lifecycle ros-${ROS_DISTRO}-rclcpp-components \
        ros-${ROS_DISTRO}-lifecycle-msgs ros-${ROS_DISTRO}-diagnostic-updater \
        ros-${ROS_DISTRO}-diagnostic-msgs ros-${ROS_DISTRO}-rosbag2-cpp \
        ros-${ROS_DISTRO}-rosbag2-storage-mcap ros-${ROS_DISTRO}-rosbag2-py \
        ros-${ROS_DISTRO}-rosbag2-transport ros-${ROS_DISTRO}-ros2bag \
        ros-${ROS_DISTRO}-urdf ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-tf2-eigen \
        ros-${ROS_DISTRO}-pcl-conversions ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-ament-cmake-gtest \
        ros-${ROS_DISTRO}-launch-testing-ament-cmake \
        ros-${ROS_DISTRO}-rosidl-default-generators \
    && rm -rf /var/lib/apt/lists/*

# Shallow fetch of one pinned commit (GitHub serves reachable SHAs).
COPY docker/fetch_commit.sh /usr/local/bin/fetch_commit.sh

# Sophus (header-only SO(3)/SE(3)); not in the Ubuntu/ROS apt repos we rely on.
ARG SOPHUS_COMMIT=de0f8d3d92bf776271e16de56d1803940ebccab9
# tag 1.22.10
RUN fetch_commit.sh https://github.com/strasdat/Sophus.git ${SOPHUS_COMMIT} /tmp/sophus \
    && cmake -S /tmp/sophus -B /tmp/sophus/build -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF -DBUILD_PYTHON_BINDINGS=OFF \
    && cmake --install /tmp/sophus/build && rm -rf /tmp/sophus

# Livox-SDK2 (needed only to build livox_ros_driver2; STRIDE itself uses just its messages).
ARG LIVOX_SDK2_COMMIT=c0796f04c143143899c87a773d9f6b7136453c0b
RUN fetch_commit.sh https://github.com/Livox-SDK/Livox-SDK2.git ${LIVOX_SDK2_COMMIT} /tmp/livox_sdk2 \
    && cmake -S /tmp/livox_sdk2 -B /tmp/livox_sdk2/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/livox_sdk2/build -j${BUILD_JOBS} \
    && cmake --install /tmp/livox_sdk2/build && ldconfig && rm -rf /tmp/livox_sdk2

# Underlay: GTSAM 4.3.0 (same commit and core flags as the GPS-denied workspace) and the
# Livox ROS driver (CustomMsg). gtsam_unstable is not needed by STRIDE and stays off.
ARG GTSAM_COMMIT=71a25ca36c084cbad1f872e812d6d97fbadfdb05
# tag 4.3.0
ARG LIVOX_DRIVER_COMMIT=21445540f0d100dc86a7e6df312dd70bbdb4afdf
RUN fetch_commit.sh https://github.com/borglab/gtsam.git ${GTSAM_COMMIT} /opt/stride_underlay/src/gtsam \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd /opt/stride_underlay \
    && MAKEFLAGS="-j${BUILD_JOBS}" colcon build --merge-install --event-handlers console_cohesion+ \
         --packages-select gtsam \
         --cmake-args -DCMAKE_BUILD_TYPE=Release \
           -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
           -DGTSAM_BUILD_TIMING_ALWAYS=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
           -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON \
           -DGTSAM_USE_QUATERNIONS=ON -DGTSAM_ALLOW_DEPRECATED_SINCE_V43=OFF \
           -DGTSAM_WITH_TBB=OFF -DGTSAM_BUILD_PYTHON=OFF -DGTSAM_INSTALL_MATLAB_TOOLBOX=OFF \
           -DGTSAM_BUILD_UNSTABLE=OFF \
    && rm -rf /opt/stride_underlay/build /opt/stride_underlay/log /opt/stride_underlay/src/gtsam

# The driver's CMake needs DISTRO_ROS (its build.sh passes it) to pick the rosidl API of Jazzy.
RUN fetch_commit.sh https://github.com/Livox-SDK/livox_ros_driver2.git ${LIVOX_DRIVER_COMMIT} \
         /opt/stride_underlay/src/livox_ros_driver2 \
    && cp /opt/stride_underlay/src/livox_ros_driver2/package_ROS2.xml \
          /opt/stride_underlay/src/livox_ros_driver2/package.xml \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source /opt/stride_underlay/install/setup.bash \
    && cd /opt/stride_underlay \
    && MAKEFLAGS="-j${BUILD_JOBS}" colcon build --merge-install --event-handlers console_cohesion+ \
         --packages-select livox_ros_driver2 \
         --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
           -DROS_EDITION=ROS2 -DDISTRO_ROS=${ROS_DISTRO} \
    && rm -rf /opt/stride_underlay/build /opt/stride_underlay/log /opt/stride_underlay/src

COPY docker/entrypoint.sh docker/stride_env.sh /usr/local/bin/
ENV STRIDE_UNDERLAY=/opt/stride_underlay/install
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]

FROM deps AS ci
# CI runs as root (or --user) with the checkout mounted; nothing else to add.

FROM ci AS dev
ARG ROS_DISTRO
ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-rviz2 \
        sudo less vim-tiny bash-completion htop valgrind \
    && rm -rf /var/lib/apt/lists/*
# Ubuntu 24.04 images ship a user `ubuntu` with uid 1000: replace it with `dev`, so files
# created in the bind-mounted workspace belong to the host user (no root-owned build/).
RUN if id ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
    && (getent group ${USER_GID} >/dev/null || groupadd -g ${USER_GID} ${USERNAME}) \
    && useradd -m -s /bin/bash -u ${USER_UID} -g ${USER_GID} ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && echo 'source /usr/local/bin/stride_env.sh' >> /home/${USERNAME}/.bashrc
USER ${USERNAME}
