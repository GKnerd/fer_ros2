#    Copyright 2025 Proximity Robotics & Automation GmbH
#    Modifications copyright 2026 Georgios Katranis
#
#    This Dockerfile is derived from internal Docker tooling developed at
#    Proximity Robotics & Automation GmbH. It has been adapted for the
#    Franka Emika Robot (FER) platform: dependency set, user-creation block,
#    CycloneDDS configuration, libfranka and pinned external packages built
#    from source, colcon build stage, and a single image for x86_64 and
#    aarch64 have been modified. The
#    original layering pattern is retained.
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

##############################################################################
##                                 Base Image                               ##
##############################################################################
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base
ENV PROJECT_WS=ros2_ws
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

##############################################################################
##                                 Global Dependecies                       ##
##############################################################################


RUN apt-get update && apt-get upgrade -y \
    && apt-get install --no-install-recommends -y \
    apt-utils \
    aptitude \
    sudo \
    nano \
    curl \
    wget \
    tmux \
    build-essential \
    software-properties-common \
    cmake \
    git \
    gcc \
    libpoco-dev \
    libfmt-dev  \
    libeigen3-dev \
    python3-pip \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

##############################################################################
##                                 Create User                              ##
##############################################################################
ARG USER=fer_ros2
ARG UID=1000
ARG GID=$UID
ARG ROS_DOMAIN_ID=0
ARG VIDEO_GID=44
ENV USER=${USER}
ENV UID=${UID}
ENV GID=${GID}
ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID}

# Remove any existing user holding the target UID (e.g. Ubuntu Noble ships
# an `ubuntu` user at UID 1000) and recreate it under ${USER}. Hardware
# groups (video, dialout, plugdev, realtime) enable GUI/USB/serial access
# and SCHED_FIFO scheduling for ros2_control on the real robot later on.
RUN existing_user="$(getent passwd | awk -F: -v target_uid=${UID} '$3 == target_uid { print $1 }')" && \
    if [ -n "$existing_user" ]; then \
        userdel --remove "$existing_user"; \
    fi && \
    if ! getent group "$GID" >/dev/null 2>&1; then \
        groupadd -g "$GID" "$USER"; \
    fi && \
    useradd -m -u "$UID" -g "$GID" --shell "$(which bash)" "$USER" -G sudo && \
    groupadd realtime && \
    groupmod -g ${VIDEO_GID} video && \
    usermod -aG video "$USER" && \
    usermod -aG dialout "$USER" && \
    usermod -aG realtime "$USER" && \
    usermod -aG plugdev "$USER"

RUN echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudogrp && \
    chmod 0440 /etc/sudoers.d/sudogrp

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc
RUN echo "source /home/$USER/deps_ws/install/setup.bash" >> /etc/bash.bashrc
RUN echo "source /home/$USER/$PROJECT_WS/install/setup.bash" >> /etc/bash.bashrc
RUN echo "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" >> /etc/bash.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /etc/bash.bashrc

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///home/${USER}/env/cyclone_dds.xml

# Create base directories
USER ${USER}
RUN mkdir -p /home/${USER}/${PROJECT_WS}/src
RUN mkdir -p /home/${USER}/deps
RUN mkdir -p /home/${USER}/deps_ws/src
##############################################################################
##                                 Dependencies                             ##
##############################################################################
USER root

RUN apt-get update && \
    apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt* \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-behaviortree-cpp \
    ros-${ROS_DISTRO}-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

USER ${USER}

##############################################################################
##                                 User Dependecies                         ##
##############################################################################
ARG LIBFRANKA_VERSION=0.9.2
WORKDIR /home/${USER}/deps
RUN git clone --recursive --branch ${LIBFRANKA_VERSION} https://github.com/frankarobotics/libfranka.git && \
    cd libfranka && \
    cmake -S . \
    -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_TESTS=OFF \
    && cmake --build build -j"$(nproc)"

##############################################################################
##                                 External ROS Packages                    ##
##############################################################################
# Pinned upstream packages, built once into ~/deps_ws as an underlay of
# ros2_ws. Each ARG sits right before its step --> change of ARG rebuilds from that step on.

WORKDIR /home/${USER}/deps_ws
ARG FRANKA_DESCRIPTION_COMMIT=72baf5bf4e88eaec27f0eb61be1b20a001abf2ab
# PR #122 (issue #121): fixes a thread crash on ROS 2 Jazzy, not yet in the
# 0.3.0 release.
ARG BT_ROS2_COMMIT=6c6aa078ee7bc52fec98984bed4964556abf5beb
RUN git clone https://github.com/frankarobotics/franka_description.git src/franka_description && \
    git -C src/franka_description checkout ${FRANKA_DESCRIPTION_COMMIT} && \
    git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git src/BehaviorTree.ROS2 && \
    git -C src/BehaviorTree.ROS2 checkout ${BT_ROS2_COMMIT}

RUN sudo apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --merge-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF

# MuJoCo simulation backend, only with --build-arg WITH_MUJOCO=1.
ARG WITH_MUJOCO=0
ARG MUJOCO_VENDOR_COMMIT=26187d69dd239adc45c121af780d57590d189686
ARG MUJOCO_ROS2_CONTROL_COMMIT=375a5db49d2c92fcfdd8f88c1376c4fb3eee3a70
RUN if [ "$WITH_MUJOCO" = "1" ]; then \
        git clone https://github.com/pal-robotics/mujoco_vendor.git src/mujoco_vendor && \
        git -C src/mujoco_vendor checkout ${MUJOCO_VENDOR_COMMIT} && \
        git clone https://github.com/ros-controls/mujoco_ros2_control.git src/mujoco_ros2_control && \
        git -C src/mujoco_ros2_control checkout ${MUJOCO_ROS2_CONTROL_COMMIT} && \
        sudo apt-get update && \
        rosdep install --from-paths src --ignore-src -r -y && \
        . /opt/ros/$ROS_DISTRO/setup.sh && \
        colcon build --merge-install --packages-skip-build-finished \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF; \
    else \
        echo "WITH_MUJOCO=${WITH_MUJOCO}, skipping the MuJoCo packages"; \
    fi

##############################################################################
##                                 Build ROS                                ##
##############################################################################
WORKDIR /home/${USER}/${PROJECT_WS}
COPY --chown=${USER}:${USER} ${PROJECT_WS}/src /home/$USER/${PROJECT_WS}/src

# The external packages come from ~/deps_ws; keep rosdep from installing
# their apt releases.
RUN sudo apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y \
    --skip-keys "franka_description behaviortree_ros2 mujoco_vendor mujoco_ros2_control"

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    . /home/${USER}/deps_ws/install/setup.sh && \
    colcon build --symlink-install \
    --merge-install \
    --cmake-clean-cache \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DBUILD_TESTING=OFF \
    -DFranka_DIR:PATH=/home/${USER}/deps/libfranka/build

RUN echo "source /home/$USER/ros2_ws/install/setup.bash" >> /home/$USER/.bashrc

RUN sudo sed --in-place --expression \
    '$isource "/home/$USER/${PROJECT_WS}/install/setup.bash"' \
    /ros_entrypoint.sh

CMD ["/bin/bash"]

