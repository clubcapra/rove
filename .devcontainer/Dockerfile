FROM ros:humble-ros-base

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=rove
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# ENV variables
ENV ROS_DISTRO=humble
ENV GZ_VERSION=harmonic

WORKDIR /workspace/$USERNAME

RUN groupadd --gid $USER_GID $USERNAME || true \
    && groupadd -g 107 input \
    && useradd -s /bin/bash --uid $USER_UID -g $USER_GID -G 107 -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo tesseract-ocr \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Give user serial device permission
RUN usermod -a -G dialout $USERNAME

# Update all packages
RUN apt update && apt upgrade -y

# Install Git
RUN apt install -y git

# Install necessary packages for Gazebo Garden and its dependencies
RUN apt-get update && apt-get install -y \
    software-properties-common \
    wget \
    lsb-release \
    gnupg \
    curl

# Install Gazebo
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt update -y 

RUN apt install -y ros-humble-ros-gzharmonic

# Change user
RUN chown $USER_UID:$USER_GID /workspace/$USERNAME

USER $USERNAME

# Copy to preload the ros packages
COPY ./ /workspace/$USERNAME/

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

COPY --chown=$USERNAME ./ /workspace/$USERNAME

# Add needed packages
RUN vcs import src < rove.repos


# Rosdep update
RUN rosdep update

# Install ROS dependencies
RUN rosdep install --from-paths src --ignore-src --rosdistro humble -y
