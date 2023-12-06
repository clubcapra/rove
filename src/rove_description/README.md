# Rove Description

This package contains the URDF description of the rover and everything related to the simulation of the robot.

# Problem

Due to the fact that Ubuntu 22.04 ships with DART 6.12, but Gazebo requires DART 6.13 for the tracks simulation, it is necessary to install DART from source to meet the version requirement. Compiling DART, Gazebo, and gz-ros from source can be time-consuming, so a Dockerfile has been created to streamline the process. This Dockerfile enables anyone to run the simulator without going through the lengthy compilation steps themselves. For those who need a user interface, there are two options: either follow the steps outlined in the Dockerfile to perform a local installation or utilize an X server to forward the simulator's display from the Docker container to the host machine.

More details on the issue can be found here: https://github.com/gazebosim/gz-sim/issues/1662


# Installation

## VM 

```bash
ign gazebo --render-engine ogre
```

## WSL
    
```bash
export LIBGL_ALWAYS_INDIRECT=0 export LIBGL_ALWAYS_SOFTWARE=1
ign gazebo --headless-rendering
```

## Docker

A cloud image will be available in the next version of the software, but currently, you need to build it. This process can take from 30 min to 1h.

### Build the image

```bash
docker compose build
```

### Run the image

```bash
docker compose up
```


# Usage

Once everything has been installed and compiled, if you don't use docker, you can run the simulator with the following command:

```bash
ros2 launch rove_description sim.launch.py
```
