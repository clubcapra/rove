# Installing Gazebo
Gazebo allows us to simulate Rove in a virtual environment.

## What's up
The default version of gazebo for ROS Humble is Gazebo Fortress. However, Gazebo Fortress has (or at least had) issues with the tracks. This is why we need Gazebo Harmonic. According to [this documentation](https://web.archive.org/web/20250421124813/https://gazebosim.org/docs/latest/ros_installation/), Gazebo Harmonic is compatible with ROS Humble but need some work to get along.

## Removing Gazebo
In case of fire, use this (removes gazebo and all installed dependencies):
```sh
sudo apt remove '^.*libgz.*$' '^.*gzharmonic.*$' '^.*gz-.*$' '^.*gazebo.*$' 
```

## Installation

Install dependencies:
```sh
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
```

Install Gazebo Harmonic:
```sh
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

Additionally, install these to ensure successfull build:
```sh
sudo apt install ros-humble-ros-gzharmonic-sim ros-humble-controller-manager 
```

To build gz_ros2_control, you need to add this to your `.bashrc`:
```sh
export GZ_VERSION=harmonic
```
And don't forget to source or restart your terminal after changing `.bashrc`.
```sh
source ~/.bashrc
```

## References
- https://web.archive.org/web/20250422015505/https://gazebosim.org/docs/harmonic/install_ubuntu/#binary-installation-on-ubuntu
