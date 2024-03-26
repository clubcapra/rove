FROM ghcr.io/clubcapra/gazebo:harmonic

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=rove
ARG USER_UID=1000
ARG USER_GID=$USER_UID

WORKDIR /workspace/$USERNAME

RUN groupadd --gid $USER_GID $USERNAME || true \
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

# Change user
USER $USERNAME

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /gazebo/gazebo_ws/install/setup.bash" >> ~/.bashrc

# Install ROS dependencies
RUN rosdep update
# Copy to preload the ros packages
COPY ./ /workspace/$USERNAME/

RUN rosdep install --from-paths src --ignore-src --rosdistro humble -y

