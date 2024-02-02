FROM ghcr.io/clubcapra/gazebo:harmonic



# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=rove
ARG USER_UID=1000
ARG USER_GID=$USER_UID

WORKDIR /workspace/$USERNAME

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Update all packages
RUN apt update && apt upgrade -y

# Install Git
RUN apt install -y git

# Copy to preload the ros packages
COPY ./ /workspace/$USERNAME/

# Change user
USER $USERNAME

# Rosdep update
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src --rosdistro humble -y

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /gazebo/gazebo_ws/install/setup.bash" >> ~/.bashrc



