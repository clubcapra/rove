# Start from ROS Humble base image
FROM ros:humble-ros-base

# Set environment variables
ENV ROS_DISTRO=humble

# COPY rove_description for pre-installation
COPY . /root/workspace/src/rove_description

# Build the rove_description ROS packages
RUN /bin/bash -c '. /opt/ros/${ROS_DISTRO}/setup.bash; \
    cd ~/workspace; \
    colcon build --symlink-install --packages-select rove_description --merge-install'

# Create entrypoint
RUN echo '#!/bin/bash \n\
    source /opt/ros/${ROS_DISTRO}/setup.bash \n\
    source /root/workspace/install/setup.bash \n\
    exec "$@"' > /ros_entrypoint.sh

# Set entrypoint and default command
ENTRYPOINT ["/ros_entrypoint.sh"]
# TO BE change to start ros2 command
CMD ["ign", "gazebo", "--headless-rendering"]
