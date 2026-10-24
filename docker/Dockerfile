# Rover stack image: ROS 2 Jazzy (Ubuntu 24.04) with both workspaces built.
#
# Built for the Jetson Orin Nano on JetPack 7.2 (L4T r39, linux/arm64). JetPack
# 7.2 moves the Orin family onto the arm64-SBSA CUDA stack, so NVIDIA's standard
# multi-arch CUDA images run on it, with the host's GPU driver injected by the
# NVIDIA Container Toolkit (`runtime: nvidia` in compose.yaml). The same base
# builds on an x86 GPU machine for the off-board half.
#
#   docker build -f docker/Dockerfile -t rover:jazzy .         # from the repo root
#
# Keep the base's CUDA version at or below the host's (JetPack 7.2.1 ships CUDA
# 13.2.2; check /usr/local/cuda/version.json on the host).
# For a slim CPU-only image (no CUDA; also runs on a JetPack 6 host):
#   --build-arg BASE_IMAGE=ros:jazzy-ros-base
# ROS 2 Jazzy is installed from apt below when the base doesn't already have it.
ARG BASE_IMAGE=nvidia/cuda:13.2.1-runtime-ubuntu24.04
FROM ${BASE_IMAGE}

ARG ROS_DISTRO=jazzy
# Livox-SDK2 release and the livox_ros_driver2 commit pinned by the git submodule.
ARG LIVOX_SDK2_REF=v1.2.5
ARG LIVOX_DRIVER_REF=4a1def929e5b59c7a8122d19fce6efba581ce9f7
ARG DEPTHAI_VERSION=3.11.0
# ros-infrastructure/ros-apt-source release that installs the ROS apt repo + key.
ARG ROS_APT_SOURCE_VERSION=1.3.0

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    ROS_DISTRO_NAME=${ROS_DISTRO} \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    PYTHONUNBUFFERED=1 \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ── ROS 2 Jazzy (only if the base image lacks it, e.g. the CUDA base) ────────
RUN if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
            ca-certificates curl software-properties-common && \
        add-apt-repository -y universe && \
        curl -fsSL -o /tmp/ros2-apt-source.deb \
            "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb" && \
        dpkg -i /tmp/ros2-apt-source.deb && rm /tmp/ros2-apt-source.deb && \
        apt-get update && apt-get install -y --no-install-recommends \
            ros-${ROS_DISTRO}-ros-base python3-rosdep python3-colcon-common-extensions && \
        rosdep init && rm -rf /var/lib/apt/lists/*; \
    fi

# ── System + ROS dependencies ─────────────────────────────────────────────────
# Onboard: sensors, EKF, SLAM, Nav2 controller, VESC. Off-board: Nav2 global
# side (navigation2 covers both). Tools: the launch scripts use ping and lsusb.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git \
        python3-pip python3-colcon-common-extensions python3-rosdep \
        python3-serial python3-numpy \
        iputils-ping usbutils iproute2 \
        libusb-1.0-0 udev \
        libapr1-dev libpcl-dev \
        ros-${ROS_DISTRO}-robot-localization \
        ros-${ROS_DISTRO}-slam-toolbox \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-pointcloud-to-laserscan \
        ros-${ROS_DISTRO}-pcl-conversions \
        ros-${ROS_DISTRO}-pcl-ros \
        ros-${ROS_DISTRO}-ackermann-msgs \
        ros-${ROS_DISTRO}-joy \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-tf2-tools \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-serial-driver \
        ros-${ROS_DISTRO}-rosbag2 \
    && rm -rf /var/lib/apt/lists/*

# ── DepthAI v3 (OAK-D Pro) ────────────────────────────────────────────────────
# Ubuntu 24.04's system Python is PEP 668 "externally managed";
# PIP_BREAK_SYSTEM_PACKAGES (above) lets pip install next to the apt packages.
# numpy<2: Jazzy's cv_bridge is built against Ubuntu 24.04's NumPy 1.26.
RUN python3 -m pip install --no-cache-dir "depthai==${DEPTHAI_VERSION}" "numpy<2"

# ── Livox-SDK2 (not in apt) ───────────────────────────────────────────────────
# Some SDK headers use uint*_t without including <cstdint>, which GCC 13
# (Ubuntu 24.04) no longer pulls in transitively; force-include it.
RUN git clone --depth 1 --branch "${LIVOX_SDK2_REF}" https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 && \
    cmake -S /tmp/Livox-SDK2 -B /tmp/Livox-SDK2/build -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_FLAGS="-include cstdint" && \
    cmake --build /tmp/Livox-SDK2/build -j"$(nproc)" && \
    cmake --install /tmp/Livox-SDK2/build && \
    ldconfig && rm -rf /tmp/Livox-SDK2

# ── Source ────────────────────────────────────────────────────────────────────
WORKDIR /opt/rover
COPY . /opt/rover

# The build context excludes .git, so an uninitialised submodule arrives empty.
# Fetch the pinned driver commit in that case.
RUN LIVOX=ros2_ws/src/livox_ros_driver2; \
    if [ ! -f "$LIVOX/package_ROS2.xml" ]; then \
        rm -rf "$LIVOX" && \
        git clone https://github.com/Livox-SDK/livox_ros_driver2.git "$LIVOX" && \
        git -C "$LIVOX" checkout "${LIVOX_DRIVER_REF}" && rm -rf "$LIVOX/.git"; \
    fi

# ── Remaining rosdep keys, then build both workspaces ────────────────────────
# rosdep only scans the packages build.sh builds, not the whole UCSD hub.
RUN cp ros2_ws/src/livox_ros_driver2/package_ROS2.xml ros2_ws/src/livox_ros_driver2/package.xml && \
    apt-get update && (rosdep update --rosdistro "${ROS_DISTRO}" || true) && \
    rosdep install -r -y --ignore-src --rosdistro "${ROS_DISTRO}" \
        --skip-keys "depthai livox_sdk" \
        --from-paths \
            ros2_ws/src/robot_slam ros2_ws/src/livox_bringup ros2_ws/src/livox_ros_driver2 \
            ros2_ws/src/oakd_publisher ros2_ws/src/keyframe_bridge ros2_ws/src/vesc \
            ros2_ws/src/ucsd_robocar_hub2/ucsd_robocar_control2_pkg \
            ros2_ws_offboard/src \
    ; rm -rf /var/lib/apt/lists/*

# --no-symlink copies everything into install/, so build/ can be dropped to
# shrink the image. For live-editing, see the dev service in docker/compose.yaml.
RUN scripts/build.sh onboard --no-symlink && scripts/build.sh offboard --no-symlink && \
    rm -rf ros2_ws/build ros2_ws/log ros2_ws_offboard/build ros2_ws_offboard/log

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
