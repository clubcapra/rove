# Rove

Rove is a robot developed by the Capra team at ÉTS. Utilizing ROS2 Humble, Rove is designed for advanced applications in search and rescue robotics.

## Work in a docker container (Preferred)

Working in a dev container will allow you to have the same environment as the CI and make sure that your code will work on another computer. It will also allow you to easily switch package version and test things without breaking your computer.

### Windows docker installation

1. Install [Docker Desktop](https://www.docker.com/products/docker-desktop)
2. Install [Visual Studio Code](https://code.visualstudio.com/)
3. Install Xserver (We recommend [VcXsrv](https://sourceforge.net/projects/vcxsrv/))
4. Install the [Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension in VSCode
5. Clone and open the repository
6. Click on the green button in the bottom left corner of VSCode and select "Remote-Containers: Reopen in Container" or use the command palette to do it.
7. Wait for the container to build
8. Start Xserver with the ```-nowgl``` option (double click on the shortcut to open it if you use VcXsrv)

### Linux docker installation

Same as the windows installation, step 3 and 8 can be skipped.

Replace the DISPLAY environment variable in the .env file
```bash
echo DISPLAY=$DISPLAY
```

To be able to use the controller node, the user need read/write permissions on the inputs.
Example:
```bash
cat /dev/input/event0
```

**Suggestion:** Configure docker to be able to run as non-root user https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user

## Native/WSL installation (Ubuntu 22.04 LTS, other distros not supported)

To install ROS2 and vcs natively: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html 

Note: At the step `sudo apt install ros-humble-desktop` do `sudo apt install ros-humble-desktop-full` instead.

```bash
git clone https://github.com/gazebosim/ros_gz.git -b humble # Install gazebo locally
git clone https://github.com/clubcapra/rove.git
vcs import src < rove.repos
colcon build --symlink-install
source install/setup.bash
```

## Running Rove in simulation

IF YOU ARE RUNNING IN WSL: do this command
```bash
export LIBGL_ALWAYS_INDIRECT=0 export LIBGL_ALWAYS_SOFTWARE=1
```
Do these commands to run the gazebo simulation with physics enabled
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch rove_bringup sim.launch.py
```
OR Do these commands to only run the rviz simulation (with joints control)
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch rove_description launch.py
```

## Running the controller with a usb cable

```bash
source install/setup.bash
ros2 launch rove_bringup rove_controller_usb.launch.py
```

## Running the controller with bluetooth

```bash
source install/setup.bash
ros2 launch rove_bringup rove_controller_bluetooth.launch.py
```

## Launch VectorNav node
```bash
ros2 launch rove_bringup vectornav.launch.py
```

## Adding New Packages

To add a package for Rove, create it using the ROS2 command ([Creating Your First ROS2 Package](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)). Name it starting with `rove_` to ensure Git tracking. For non-Rove specific packages, create a separate repository and add it to `rove.repos`.

## Updating Packages

Update Rove-specific packages directly in this repository. For Capra-related or external packages, update their references in `rove.repos`. Change the Git branch in `rove.repos` as needed and apply updates with `vcs import src < rove.repos`.

## Package structure

**Bringup:**
This essential package is responsible for initializing the rover in real-world settings. It activates all necessary subsidiary packages tailored for specific operational scenarios.

**Description:**
This package is specifically designed for detailing the rover's structure in URDF (Unified Robot Description Format).

**Gazebo:**
This package facilitates the simulation of the rover within the Gazebo environment, providing a virtual testing ground.

**Hardware:**
Dedicated to initializing all sensors and actuators on the rover, this package is pivotal for operational readiness. It is specifically designed for use with the actual hardware and is not suitable for development machines.

**Navigation:**
This is a wrapper package that integrates navigation functionalities, primarily based on the nav2 framework.

**NLP (Natural Language Processing):**
This package manages the deployment of the NLP server and chatbot, facilitating advanced communication and processing capabilities.

**SLAM (Simultaneous Localization and Mapping):**
This package serves as a comprehensive wrapper for SLAM operations, incorporating tools like slam_toolbox, rtab map, and sensor filters for effective environment mapping and rover localization.

## Docker architecture

It's possible to run the entire project into multiple docker containers. Each container can be run independently and are built using the following structure :

![Docker structure](https://www.plantuml.com/plantuml/svg/VP71Ri8m38RlUOgezwvZq8vnc3XmsLDKJkgLGEj4JbfjU_gr0QgWJHoGVjl_xtouUn-0mz1tyc3r6Ldwm8CE0wCGmOGEPNOTVFJGeZoWGsgGj46V2S6e0r0xszgZvYTZ2zqDIeDZA5huGMLtH-3Uaj6P12zlHPfawulfjpiElUemRz2VWtNHFhNhQ_qWCSbSWSSbCXUfdyOB6uscCL0O3w3ZWgzjLLURUS5BVLbMA_rPhak4jNfhLXLiomq0gjNhymfK2TigFdB2u2tLbWs9Ux1n_WEZjXJ0479qJmtihEkHGhrC_iGOl5F8_9qx4sFip0Fx1I6V4HAa922IPsMUlzyTCz5njdoNcuZT_y55jg3A2NLsBbUNOb6nxSnTy8G-M98HEXhIGoOwdINvFL8pz9tu1G00 "Docker structure")

# Documentation

To update the UML diagram, create a new encoded link on the PlantUML website. Copy the existing UML, make your changes, and then update the Markdown file with the new link.
