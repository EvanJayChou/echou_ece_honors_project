# Rover stack image: ROS 2 Humble (Ubuntu 22.04) with both workspaces built.
#
# Built for the Jetson Orin Nano (linux/arm64, JetPack 6 host), but the default
# base is multi-arch, so the same file builds on an x86 GPU machine for the
# off-board half.
#
#   docker build -f docker/Dockerfile -t rover:humble .        # from the repo root
#
# The stack doesn't use CUDA, so the default base is the slim official ROS image.
# For CUDA in the container later (e.g. on-robot 3DGS), switch to an L4T base:
#   --build-arg BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.4.0
# ROS 2 Humble is installed from apt below when the base doesn't already have it.
ARG BASE_IMAGE=ros:humble-ros-base
FROM ${BASE_IMAGE}

ARG ROS_DISTRO=humble
# Livox-SDK2 release and the livox_ros_driver2 commit pinned by the git submodule.
ARG LIVOX_SDK2_REF=v1.2.5
ARG LIVOX_DRIVER_REF=4a1def929e5b59c7a8122d19fce6efba581ce9f7
ARG DEPTHAI_VERSION=3.11.0

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    ROS_DISTRO_NAME=${ROS_DISTRO} \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    PYTHONUNBUFFERED=1

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ── ROS 2 Humble (only if the base image lacks it, e.g. l4t-jetpack) ─────────
RUN if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
            curl gnupg2 lsb-release software-properties-common && \
        add-apt-repository -y universe && \
        curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /usr/share/keyrings/ros-archive-keyring.gpg && \
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
            > /etc/apt/sources.list.d/ros2.list && \
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
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-tf2-tools \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-serial-driver \
        ros-${ROS_DISTRO}-rosbag2 \
    && rm -rf /var/lib/apt/lists/*

# ── DepthAI v3 (OAK-D Pro) ────────────────────────────────────────────────────
# numpy<2: Humble's cv_bridge is built against NumPy 1.x.
RUN python3 -m pip install --no-cache-dir "depthai==${DEPTHAI_VERSION}" "numpy<2"

# ── Livox-SDK2 (not in apt) ───────────────────────────────────────────────────
RUN git clone --depth 1 --branch "${LIVOX_SDK2_REF}" https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 && \
    cmake -S /tmp/Livox-SDK2 -B /tmp/Livox-SDK2/build -DCMAKE_BUILD_TYPE=Release && \
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
