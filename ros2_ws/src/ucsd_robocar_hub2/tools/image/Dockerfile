ARG BASE_IMAGE=ros:jazzy-ros-base-noble

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Getting Base image (ubuntu 22.04)
# --------------------------------------------------------------------------------------------------------------------------------------------------
FROM $BASE_IMAGE AS ucsd_robocar2_base

ARG ROS_DISTRO
ARG ROS_SOURCE

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Setting environment variable
# --------------------------------------------------------------------------------------------------------------------------------------------------
ENV ROS_DISTRO=${ROS_DISTRO}


####################################################################################################################################################
# Install required packages/dependencies for ros
####################################################################################################################################################

WORKDIR /

# --------------------------------------------------------------------------------------------------------------------------------------------------
# intel debian packages
# --------------------------------------------------------------------------------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    build-essential \
    libgtk-3-dev \
    usbutils

# --------------------------------------------------------------------------------------------------------------------------------------------------
# install ros2 packages
# --------------------------------------------------------------------------------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-desktop \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-realsense2-camera-msgs \
    ros-$ROS_DISTRO-librealsense2 \ 
    ros-$ROS_DISTRO-message-filters \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-sick-scan-xd \
    ros-$ROS_DISTRO-teleop-twist-joy \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-joy-teleop \
    ros-$ROS_DISTRO-rviz-default-plugins \
    ros-$ROS_DISTRO-rviz-rendering \
    ros-$ROS_DISTRO-rosbag2 \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-ackermann-msgs \
    ros-$ROS_DISTRO-serial-driver \
    ros-$ROS_DISTRO-depthai-ros \
    && rm -rf /var/lib/apt/lists/*

# Not sure if these are needed...
#    ros-$ROS_DISTRO-rviz2 \
#    ros-$ROS_DISTRO-rviz-common \
#    ros-$ROS_DISTRO-realsense2-camera-description \

# --------------------------------------------------------------------------------------------------------------------------------------------------
# install useful packages
# --------------------------------------------------------------------------------------------------------------------------------------------------
WORKDIR /
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-pip \
    python3-dev \
    nano \
    vim \
    net-tools \
    iputils-ping \
    x11-apps \
    nautilus \
    firefox \
    git-all \
    cheese \
    jstest-gtk \
    joystick \
    gedit \
    gedit-plugin-multi-edit \
    gedit-plugins \
    python3-tk

RUN apt-get update && \
    pip3 install \
    pandas \
    control \
    --break-system-packages

FROM ucsd_robocar2_base AS ucsd_robocar2_dep

####################################################################################################################################################
# Create ros2 and sensor2 workspaces and directory paths for all sensor and actuator drivers
####################################################################################################################################################

WORKDIR /

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Camera inventory
# --------------------------------------------------------------------------------------------------------------------------------------------------
# None

# --------------------------------------------------------------------------------------------------------------------------------------------------
# IMU inventory
# --------------------------------------------------------------------------------------------------------------------------------------------------
# None

# --------------------------------------------------------------------------------------------------------------------------------------------------
# GPS inventory
# --------------------------------------------------------------------------------------------------------------------------------------------------
# None

# --------------------------------------------------------------------------------------------------------------------------------------------------
# ros2 and sensor2 ws
# --------------------------------------------------------------------------------------------------------------------------------------------------
RUN mkdir -p /home/projects/ros2_ws/src
RUN mkdir -p /home/projects/sensor2_ws/src

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Lidar inventory
# --------------------------------------------------------------------------------------------------------------------------------------------------
RUN mkdir -p /home/projects/sensor2_ws/src/lidars/bpearl/src
RUN mkdir -p /home/projects/sensor2_ws/src/lidars/rplidar/src
RUN mkdir -p /home/projects/sensor2_ws/src/lidars/livox/src
RUN mkdir -p /home/projects/sensor2_ws/src/lidars/sicktim/src
RUN mkdir -p /home/projects/sensor2_ws/src/lidars/ld06/ros2/src

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Get RS Bpearl --ROS2
# TODO: FIX CONFIG FILES TO COMPILE WITH COLCON
# --------------------------------------------------------------------------------------------------------------------------------------------------
# WORKDIR /home/projects/sensor2_ws/src/lidars/bpearl/src
# RUN git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git && \
#     cd rslidar_sdk && \
#     git submodule init && \
#     git submodule update
#WORKDIR /home/projects/sensor2_ws/lidars/rslidar
#RUN [\
#    "/bin/bash", \
#    "-c", \
#    "source /opt/ros/$ROS_DISTRO/setup.bash && \
#    colcon build && \
#    source install/setup.bash"\
#    ]

# # --------------------------------------------------------------------------------------------------------------------------------------------------
# # Get RPLidar --ROS2
# # --------------------------------------------------------------------------------------------------------------------------------------------------
# WORKDIR /home/projects/sensor2_ws/src/lidars/rplidar/src
# RUN git clone https://github.com/CreedyNZ/rplidar_ros2.git
# WORKDIR /home/projects/sensor2_ws/src/lidars/rplidar
# RUN [\
#     "/bin/bash", \
#     "-c", \
#     "source /opt/ros/$ROS_DISTRO/setup.bash && \
#     colcon build && \
#     source install/setup.bash"\
#     ]

# # --------------------------------------------------------------------------------------------------------------------------------------------------
# # Get livox --ROS2
# # --------------------------------------------------------------------------------------------------------------------------------------------------
# WORKDIR /home/projects/sensor2_ws/src/lidars/livox/src
# RUN git clone https://github.com/Livox-SDK/livox_ros2_driver.git
# WORKDIR /home/projects/sensor2_ws/src/lidars/livox
# RUN [\
#     "/bin/bash", \
#     "-c", \
#     "source /opt/ros/$ROS_DISTRO/setup.bash && \
#     colcon build && \
#     source install/setup.bash"\
#     ]

# # --------------------------------------------------------------------------------------------------------------------------------------------------
# # Get SickTim5xx --ROS2
# # --------------------------------------------------------------------------------------------------------------------------------------------------
# WORKDIR /home/projects/sensor2_ws/src/lidars/sicktim/src
# RUN git clone -b master https://github.com/SICKAG/sick_scan_xd.git
# WORKDIR /home/projects/sensor2_ws/src/lidars/sicktim
# RUN [\
#     "/bin/bash", \
#     "-c", \
#     "source /opt/ros/$ROS_DISTRO/setup.bash && \
#     colcon build && \
#     source install/setup.bash"\
#     ]

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Get LD06 lidar --ROS2
# --------------------------------------------------------------------------------------------------------------------------------------------------
WORKDIR /home/projects/sensor2_ws/src/lidars/ld06/ros2/src
RUN git clone https://github.com/linorobot/ldlidar.git
WORKDIR /home/projects/sensor2_ws/src/lidars/ld06/ros2
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep update && rosdep install --from-path src --ignore-src -y && \
    colcon build && \
    source install/setup.bash"\
    ]

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Camera inventory
# --------------------------------------------------------------------------------------------------------------------------------------------------
RUN mkdir -p /home/projects/sensor2_ws/src/cameras/zed/src
RUN mkdir -p /home/projects/sensor2_ws/src/cameras/oakd/src

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Get OAK-D --ROS2
# --------------------------------------------------------------------------------------------------------------------------------------------------
WORKDIR /home/projects/sensor2_ws/src/cameras/oakd/src
RUN git clone https://github.com/Triton-AI/multi_cam_oak_lite.git
RUN pip3 install \
    depthai==3.1.0 \
    --break-system-packages
WORKDIR /home/projects/sensor2_ws/src/cameras/oakd
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep update && rosdep install --from-path src --ignore-src -y && \
    colcon build && \
    source install/setup.bash" \
    ]



# --------------------------------------------------------------------------------------------------------------------------------------------------
# IMU inventory
# --------------------------------------------------------------------------------------------------------------------------------------------------

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Get OpenLog Artemis DEV-16832 --ROS2
# --------------------------------------------------------------------------------------------------------------------------------------------------
RUN mkdir -p /home/projects/sensor2_ws/src/imu/artemis_openlog/src

RUN apt-get update
RUN pip3 install --ignore-installed \
        transforms3d \
        vpython \
    --break-system-packages

# RUN pip install wxPython --verbose --break-system-packages

WORKDIR /home/projects/sensor2_ws/src/imu/artemis_openlog/src
RUN git clone https://github.com/sisaha9/razor_imu_9dof.git
WORKDIR /home/projects/sensor2_ws/src/imu/artemis_openlog
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep update && rosdep install --from-path src --ignore-src -y && \
    colcon build && \
    source install/setup.bash"\
    ]

# --------------------------------------------------------------------------------------------------------------------------------------------------
# GPS inventory
# --------------------------------------------------------------------------------------------------------------------------------------------------
RUN mkdir -p /home/projects/sensor2_ws/src/gps/ublox/src

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Get Ublox GPS --ROS2
# --------------------------------------------------------------------------------------------------------------------------------------------------
WORKDIR /home/projects/sensor2_ws/src/gps/ublox/src
RUN git clone -b ros2 https://github.com/KumarRobotics/ublox.git
WORKDIR /home/projects/sensor2_ws/src/gps/ublox
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep update && rosdep install --from-path src --ignore-src -y && \
    colcon build && \
    source install/setup.bash"\
    ]

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Actuator inventory (both ROS1 and ROS2 compatiable)
# --------------------------------------------------------------------------------------------------------------------------------------------------

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Get VESC drivers --ROS2
# --------------------------------------------------------------------------------------------------------------------------------------------------
WORKDIR /home/projects/sensor2_ws/src/vesc/src/
RUN git clone --branch humble https://github.com/Triton-AI/vesc-robocar.git
RUN git clone --branch foxy-devel https://github.com/f1tenth/ackermann_mux.git
RUN git clone --branch humble-devel https://github.com/f1tenth/teleop_tools.git
WORKDIR /home/projects/sensor2_ws/src/vesc
RUN apt-get update && apt-get install \
    ros-$ROS_DISTRO-asio-cmake-module
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep update && rosdep install --from-path src --ignore-src -y && \
    colcon build && \
    source install/setup.bash"\
    ]


# --------------------------------------------------------------------------------------------------------------------------------------------------
# Get PyVesc (both ROS1 and ROS2 compatiable)
# --------------------------------------------------------------------------------------------------------------------------------------------------
WORKDIR /  
RUN apt-get update
RUN pip3 install git+https://github.com/LiamBindle/PyVESC \
    --break-system-packages

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Get adafruit servokit (both ROS1 and ROS2 compatiable)
# --------------------------------------------------------------------------------------------------------------------------------------------------
RUN apt-get update
RUN pip3 install \
        adafruit-circuitpython-pca9685 \
        adafruit-circuitpython-servokit \
        Jetson.GPIO \
    --break-system-packages && \
    groupadd -f -r gpio && \
    usermod -a -G gpio root

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Get ROSBoard
# --------------------------------------------------------------------------------------------------------------------------------------------------
WORKDIR /
RUN apt-get update
RUN pip install tornado --break-system-packages
WORKDIR /home/projects/rosboard_ws/src
RUN git clone https://github.com/dheera/rosboard.git
WORKDIR /home/projects/rosboard_ws
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep update && rosdep install --from-path src --ignore-src -y && \
    colcon build && \
    source install/setup.bash"\
    ]

# --------------------------------------------------------------------------------------------------------------------------------------------------
# simulator ws 
# --------------------------------------------------------------------------------------------------------------------------------------------------

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Get f1tenth simulator --ROS2
# --------------------------------------------------------------------------------------------------------------------------------------------------

### gym==0.19.0 is required but not supported by python 3.12
### skip for now and hope for future compatibility

# RUN mkdir -p /home/projects/sim2_ws/src
# WORKDIR /home/projects/sim2_ws/src
# RUN git clone https://github.com/f1tenth/f1tenth_gym
# RUN git clone https://github.com/f1tenth/f1tenth_gym_ros.git

# WORKDIR /home/projects/sim2_ws/src/f1tenth_gym
# RUN pip3 install -e . --break-system-packages

# WORKDIR /home/projects/sim2_ws


# RUN [\
#     "/bin/bash", \
#     "-c", \
#     "source /opt/ros/$ROS_DISTRO/setup.bash && \
#     rosdep update && rosdep install --from-path src --ignore-src -y && \
#     colcon build && \
#     source install/setup.bash"\
#     ]

RUN apt-get update && \
    pip3 install \
    numpy==1.26.4 \
    --break-system-packages

FROM ucsd_robocar2_dep AS ucsd_robocar2


####################################################################################################################################################
# Setting up UCSD Robocar Framework
####################################################################################################################################################

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Get ucsd_robocar_hub2
# --------------------------------------------------------------------------------------------------------------------------------------------------
WORKDIR /home/projects/ros2_ws/src
WORKDIR /home/projects/ros2_ws/src
RUN git clone https://github.com/UCSD-ECEMAE-148/ucsd_robocar_hub2.git && \
    cd ucsd_robocar_hub2 && \
    git submodule init && \
    git submodule update --remote --merge
WORKDIR /home/projects/ros2_ws
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build && \
    source install/setup.bash"\
    ]

# --------------------------------------------------------------------------------------------------------------------------------------------------
# TODO: ADD CUDA enabled openCV see: https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.opencv
# --------------------------------------------------------------------------------------------------------------------------------------------------


# --------------------------------------------------------------------------------------------------------------------------------------------------
# Add custom functions to bashrc
# --------------------------------------------------------------------------------------------------------------------------------------------------
WORKDIR /home/scripts/
COPY tools/image/bashrc/bashrc_docker.sh ./bashrc_docker.sh
RUN ["/bin/bash", "-c", "echo '. /home/scripts/bashrc_docker.sh' >> /root/.bashrc"]

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Place user in projects
# --------------------------------------------------------------------------------------------------------------------------------------------------
WORKDIR /home/projects/
