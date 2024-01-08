# Rove

Rove is a robot developed by the Capra team at ÉTS. Utilizing ROS2 Humble, Rove is designed for advanced applications in search and rescue robotics.

## Work in a docker container (Preferred)

Working in a dev container will allow you to have the same environnement as the CI and make sure that your code will work on another computer. It will also allow you to easily switch package version and test thing without breaking your computer.

### Windows installation

1. Install [Docker Desktop](https://www.docker.com/products/docker-desktop)
2. Install [Visual Studio Code](https://code.visualstudio.com/)
3. Install Xserver (We recommend [VcXsrv](https://sourceforge.net/projects/vcxsrv/))
4. Install the [Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension in VSCode
5. Clone and open the repository
6. Click on the green button in the bottom left corner of VSCode and select "Remote-Containers: Reopen in Container" or use the command palette to do it.
7. Wait for the container to build
8. Start Xserver with the ```-nowgl``` option (double click on the shortcut to open it if you use VcXsrv)

### Linux installation

To be tested, but it should be similar to the Windows installation without the Xserver part.

## Native installation (Ubuntu 22.04 LTS, other distros not supported)

```bash
git clone https://github.com/clubcapra/rove.git
vcs import src < rove.repos
colcon build --symlink-install
source install/setup.bash
```

## Running Rove in simulation

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch rove_description sim.launch.py
```

**Note:** To move the robot, you need to use gazebo as there is no controller yet


## Adding New Packages

To add a package for Rove, create it using the ROS2 command ([Creating Your First ROS2 Package](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)). Name it starting with `rove_` to ensure Git tracking. For non-Rove specific packages, create a separate repository and add it to `rove.repos`.

## Updating Packages

Update Rove-specific packages directly in this repository. For Capra-related or external packages, update their references in `rove.repos`. Change the Git branch in `rove.repos` as needed and apply updates with `vcs import src < rove.repos`.

## Docker architecture

It's possible to run the entire project into multiple docker containers. Each container can be run independantly and are built using the following structure :

![Docker structure](https://www.plantuml.com/plantuml/svg/VP6zaeGm2CTxdo9Zxrn_nSqMsznJt63aTZERZqmWFlz573UU1ON27twWm8qO2jVW1tRiqOptP5zOp7U01vexPemBHkkGnc4eQ1dYOyDAee_sV3vhc3rE2zABKnuDa6dXCsbzdIta0erVyMS6Gi4sH-5SP2o_O964xbBhZKzONIfISGY5Nnsv58NUNOM5oYccu53mjr8go8NgWOylTAdKC9F0pHgjDRDWpIfKz5MePWS5orWivlT_TjcA4fbf-jfljRr4dMxHNSdiMxn4-x8kYNwYmv4eC_qBo9JdW0nqQTMtUycSvxbXN6hmVm00 "Docker structure")

# Documentation

To update the UML diagram, create a new encoded link on the PlantUML website. Copy the existing UML, make your changes, and then update the markdown file with the new link.
