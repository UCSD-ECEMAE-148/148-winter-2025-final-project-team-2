# This is an auto generated Dockerfile for ros:desktop
# generated from docker_images_ros2/create_ros_image.Dockerfile.em

#
#
####################################################################################################################################################
# Setup environment for supporting ROS1-Noetic and ROS2-Foxy
####################################################################################################################################################
#
#

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Getting Base image (ubuntu 20.04)
# --------------------------------------------------------------------------------------------------------------------------------------------------
FROM ros:foxy-ros-base-focal

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Setup sources.list
# --------------------------------------------------------------------------------------------------------------------------------------------------
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Setting environment variable
# --------------------------------------------------------------------------------------------------------------------------------------------------
ENV ROS1_DISTRO noetic
ENV ROS2_DISTRO foxy

#
#
####################################################################################################################################################
# Install required packages/dependencies for ros1 and ros2
####################################################################################################################################################
#
#

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
    libgtk-3-dev

# --------------------------------------------------------------------------------------------------------------------------------------------------
# install ros2 packages
# --------------------------------------------------------------------------------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-desktop=0.9.2-1* \
    ros-foxy-ros1-bridge \
    ros-foxy-realsense2-camera \
    ros-foxy-realsense2-camera-msgs \
    ros-foxy-librealsense2 \ 
    ros-foxy-message-filters \
    ros-foxy-image-transport \
    ros-foxy-sick-scan2 \
    ros-foxy-teleop-twist-joy \
    ros-foxy-joy \
    ros-foxy-joy-teleop \
    ros-foxy-rviz-default-plugins \
    ros-foxy-rviz-rendering \
    ros-foxy-ros2bag \
    ros-foxy-rosbag2-converter-default-plugins \
    ros-foxy-rosbag2-storage-default-plugins \
    ros-foxy-robot-localization \
    ros-foxy-slam-toolbox \
    ros-foxy-ackermann-msgs \
    ros-foxy-serial-driver \
    ros-foxy-depthai-ros \
    && rm -rf /var/lib/apt/lists/*

# Not sure if these are needed...
#    ros-foxy-rviz2 \
#    ros-foxy-rviz-common \
#    ros-foxy-realsense2-camera-description \

# --------------------------------------------------------------------------------------------------------------------------------------------------
# install ros1 packages
# --------------------------------------------------------------------------------------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
        ros-noetic-rosbash \
        ros-noetic-navigation \
        ros-noetic-sick-scan \ 
        ros-noetic-hector-slam \
        ros-noetic-scan-tools \
        ros-noetic-razor-imu-9dof \
        ros-noetic-driver-base \
        ros-noetic-map-server \
        ros-noetic-teleop-twist-joy \
        ros-noetic-joy \
        ros-noetic-joy-teleop \
        ros-noetic-realsense2-camera \
        ros-noetic-realsense2-description && \
    rm -rf /var/lib/apt/lists/*

# --------------------------------------------------------------------------------------------------------------------------------------------------
# install useful packages
# --------------------------------------------------------------------------------------------------------------------------------------------------
WORKDIR /
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-pip \
    python3-dev \
    nano \
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
    control

#
#
####################################################################################################################################################
# Create ros1, ros2, sensor1 and sensor2 workspaces and directory paths for all sensor and actuator drivers
####################################################################################################################################################
#
#

WORKDIR /

# --------------------------------------------------------------------------------------------------------------------------------------------------
# ros1 and Sensor1 ws
# --------------------------------------------------------------------------------------------------------------------------------------------------
RUN mkdir -p /home/projects/ros1_ws/src
RUN mkdir -p /home/projects/sensor1_ws/src

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - 
# Lidar inventory
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - 
RUN mkdir -p /home/projects/sensor1_ws/src/lidars/rplidar/src
RUN mkdir -p /home/projects/sensor1_ws/src/lidars/ld06/src

# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
# Get RPLidar --ROS1
# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
WORKDIR /home/projects/sensor1_ws/src/lidars/rplidar/src
RUN git clone https://github.com/Slamtec/rplidar_ros.git
WORKDIR /home/projects/sensor1_ws/src/lidars/rplidar
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/noetic/setup.bash && \
    rosdep update && rosdep install --from-path src --ignore-src -y && \
    catkin_make && \
    source devel/setup.bash"\
    ]

# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
# Get LD06 lidar --ROS1
# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
WORKDIR /home/projects/sensor1_ws/src/lidars/ld06/src
RUN git clone https://github.com/AlessioMorale/ld06_lidar.git
WORKDIR /home/projects/sensor1_ws/src/lidars/ld06
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/noetic/setup.bash && \
    rosdep update && rosdep install --from-path src --ignore-src -y && \
    catkin_make && \
    source devel/setup.bash"\
    ]

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - 
# Camera inventory
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - 
# None

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - 
# IMU inventory
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - 
# None

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - 
# GPS inventory
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - 
# None

# --------------------------------------------------------------------------------------------------------------------------------------------------
# ros2 and sensor2 ws
# --------------------------------------------------------------------------------------------------------------------------------------------------
RUN mkdir -p /home/projects/ros2_ws/src
RUN mkdir -p /home/projects/sensor2_ws/src

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - 
# Lidar inventory
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - 
RUN mkdir -p /home/projects/sensor2_ws/src/lidars/bpearl/src
RUN mkdir -p /home/projects/sensor2_ws/src/lidars/rplidar/src
RUN mkdir -p /home/projects/sensor2_ws/src/lidars/livox/src
RUN mkdir -p /home/projects/sensor2_ws/src/lidars/sicktim/src
RUN mkdir -p /home/projects/sensor2_ws/src/lidars/ld06/ros2/src

# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
# Get RS Bpearl --ROS2
# TODO: FIX CONFIG FILES TO COMPILE WITH COLCON
# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
WORKDIR /home/projects/sensor2_ws/src/lidars/bpearl/src
RUN git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git && \
    cd rslidar_sdk && \
    git submodule init && \
    git submodule update
#WORKDIR /home/projects/sensor2_ws/lidars/rslidar
#RUN [\
#    "/bin/bash", \
#    "-c", \
#    "source /opt/ros/foxy/setup.bash && \
#    colcon build && \
#    source install/setup.bash"\
#    ]

# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
# Get RPLidar --ROS2
# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
WORKDIR /home/projects/sensor2_ws/src/lidars/rplidar/src
RUN git clone https://github.com/CreedyNZ/rplidar_ros2.git
WORKDIR /home/projects/sensor2_ws/src/lidars/rplidar
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/foxy/setup.bash && \
    colcon build && \
    source install/setup.bash"\
    ]

# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
# Get livox --ROS2
# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
WORKDIR /home/projects/sensor2_ws/src/lidars/livox/src
RUN git clone https://github.com/Livox-SDK/livox_ros2_driver.git
WORKDIR /home/projects/sensor2_ws/src/lidars/livox
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/foxy/setup.bash && \
    colcon build && \
    source install/setup.bash"\
    ]

# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
# Get SickTim5xx --ROS2
# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
WORKDIR /home/projects/sensor2_ws/src/lidars/sicktim/src
RUN git clone https://github.com/SICKAG/sick_scan2.git
WORKDIR /home/projects/sensor2_ws/src/lidars/sicktim
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/foxy/setup.bash && \
    colcon build && \
    source install/setup.bash"\
    ]

# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
# Get LD06 lidar --ROS2
# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
WORKDIR /home/projects/sensor2_ws/src/lidars/ld06/ros2/src
RUN git clone https://github.com/linorobot/ldlidar.git
WORKDIR /home/projects/sensor2_ws/src/lidars/ld06/ros2
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/foxy/setup.bash && \
    rosdep update && rosdep install --from-path src --ignore-src -y && \
    colcon build && \
    source install/setup.bash"\
    ]

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - 
# Camera inventory
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - 
RUN mkdir -p /home/projects/sensor2_ws/src/cameras/zed/src
RUN mkdir -p /home/projects/sensor2_ws/src/cameras/oakd/src

# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
# Get OAK-D --ROS2
# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
WORKDIR /home/projects/sensor2_ws/src/cameras/oakd/src
RUN git clone https://github.com/Triton-AI/multi_cam_oak_lite.git
WORKDIR /home/projects/sensor2_ws/src/cameras/oakd
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/foxy/setup.bash && \
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules && \
    udevadm control --reload-rules && udevadm trigger && \
    rosdep update && rosdep install --from-path src --ignore-src -y && \
    colcon build && \
    source install/setup.bash" \
    ]


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - 
# IMU inventory
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - 

# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
# Get OpenLog Artemis DEV-16832 --ROS2
# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
RUN mkdir -p /home/projects/sensor2_ws/src/imu/artemis_openlog/src

RUN apt-get update
RUN pip3 install \
        transforms3d \
        vpython \
        wxPython

WORKDIR /home/projects/sensor2_ws/src/imu/artemis_openlog/src
RUN git clone https://github.com/sisaha9/razor_imu_9dof.git
WORKDIR /home/projects/sensor2_ws/src/imu/artemis_openlog
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/foxy/setup.bash && \
    rosdep update && rosdep install --from-path src --ignore-src -y && \
    colcon build && \
    source install/setup.bash"\
    ]

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - 
# GPS inventory
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - 
RUN mkdir -p /home/projects/sensor2_ws/src/gps/ublox/src

# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
# Get Ublox GPS --ROS2
# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
WORKDIR /home/projects/sensor2_ws/src/gps/ublox/src
RUN git clone -b foxy-devel https://github.com/KumarRobotics/ublox.git
WORKDIR /home/projects/sensor2_ws/src/gps/ublox
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/foxy/setup.bash && \
    rosdep update && rosdep install --from-path src --ignore-src -y && \
    colcon build && \
    source install/setup.bash"\
    ]

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - 
# Actuator inventory (both ROS1 and ROS2 compatiable)
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - 

# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
# Get VESC drivers --ROS2
# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
WORKDIR /home/projects/sensor2_ws/src/vesc/src/
RUN git clone --branch ros2 https://github.com/f1tenth/vesc.git
RUN git clone --branch foxy-devel https://github.com/f1tenth/ackermann_mux.git
RUN git clone --branch foxy-devel https://github.com/f1tenth/teleop_tools.git
WORKDIR /home/projects/sensor2_ws/src/vesc
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/foxy/setup.bash && \
    rosdep update && rosdep install --from-path src --ignore-src -y && \
    colcon build && \
    source install/setup.bash"\
    ]


# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
# Get PyVesc (both ROS1 and ROS2 compatiable)
# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
WORKDIR /  
RUN apt-get update
RUN pip3 install git+https://github.com/LiamBindle/PyVESC
    
# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
# Get adafruit servokit (both ROS1 and ROS2 compatiable)
# -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
RUN apt-get update
RUN pip3 install \
        adafruit-circuitpython-pca9685 \
        adafruit-circuitpython-servokit \
        Jetson.GPIO && \
    groupadd -f -r gpio && \
    usermod -a -G gpio root

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - 
# Get ROSBoard
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - 
WORKDIR /
RUN apt-get update
RUN pip install tornado
WORKDIR /home/projects/rosboard_ws/src
RUN git clone https://github.com/dheera/rosboard.git
WORKDIR /home/projects/rosboard_ws
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/foxy/setup.bash && \
    rosdep update && rosdep install --from-path src --ignore-src -y && \
    colcon build && \
    source install/setup.bash"\
    ]

# http://localhost:8888


# --------------------------------------------------------------------------------------------------------------------------------------------------
# simulator ws
# --------------------------------------------------------------------------------------------------------------------------------------------------

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - 
# Get f1tenth simulator --ROS2
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - - 

RUN mkdir -p /home/projects/sim2_ws/src
WORKDIR /home/projects/sim2_ws/src
RUN git clone https://github.com/f1tenth/f1tenth_gym
RUN git clone https://github.com/f1tenth/f1tenth_gym_ros.git

WORKDIR /home/projects/sim2_ws/src/f1tenth_gym
RUN pip3 install -e gym/
WORKDIR /home/projects/sim2_ws


RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/foxy/setup.bash && \
    rosdep update && rosdep install --from-path src --ignore-src -y && \
    colcon build && \
    source install/setup.bash"\
    ]

#
#
####################################################################################################################################################
# Setting up UCSD Robocar Framework
####################################################################################################################################################
#
#

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Get ucsd_robocar_hub1
# --------------------------------------------------------------------------------------------------------------------------------------------------
WORKDIR /home/projects/ros1_ws/src
RUN git clone https://gitlab.com/ucsd_robocar/ucsd_robocar_hub1.git
    cd ucsd_robocar_hub1 && \
    git submodule init && \
    git submodule update --remote --merge
WORKDIR /home/projects/ros1_ws
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/noetic/setup.bash && \
    catkin_make && \
    source devel/setup.bash"\
    ]

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Get ucsd_robocar_hub2
# --------------------------------------------------------------------------------------------------------------------------------------------------
WORKDIR /home/projects/ros2_ws/src
WORKDIR /home/projects/ros2_ws/src
RUN git clone https://gitlab.com/ucsd_robocar2/ucsd_robocar_hub2.git && \
    cd ucsd_robocar_hub2 && \
    git submodule init && \
    git submodule update --remote --merge
WORKDIR /home/projects/ros2_ws
RUN [\
    "/bin/bash", \
    "-c", \
    "source /opt/ros/foxy/setup.bash && \
    colcon build && \
    source install/setup.bash"\
    ]

# --------------------------------------------------------------------------------------------------------------------------------------------------
# TODO: ADD CUDA enabled openCV see: https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.opencv
# --------------------------------------------------------------------------------------------------------------------------------------------------

# --------------------------------------------------------------------------------------------------------------------------------------------------
# Place user in projects
# --------------------------------------------------------------------------------------------------------------------------------------------------
WORKDIR /home/projects/
