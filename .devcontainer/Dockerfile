FROM ghcr.io/clubcapra/gazebo:harmonic

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=rove
ARG USER_UID=1000
ARG USER_GID=$USER_UID

WORKDIR /workspace/$USERNAME

RUN groupadd --gid $USER_GID $USERNAME \
    && groupadd -g 107 input \
    && useradd -s /bin/bash --uid $USER_UID -g $USER_GID -G 107 -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Update all packages
RUN apt update && apt upgrade -y

# Install Git
RUN apt install -y git

# Install ROS dependencies
RUN apt install -y ros-humble-xacro ros-humble-ros-gz-bridge ros-humble-rviz2 ros-humble-slam-toolbox ros-humble-robot-localization
# RUN apt install -y ros-humble-joy teleop-twist-joy usbutils
# apt-get -y install ros-humble-velodyne-gazebo-plugins
# Rosdep update
RUN rosdep update

# Change user
USER $USERNAME

# Rosdep update
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src --rosdistro humble -y

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /gazebo/gazebo_ws/install/setup.bash" >> ~/.bashrc
