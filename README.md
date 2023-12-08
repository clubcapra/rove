# Rove

Rove is a robot developed by the Capra team at ÉTS. Utilizing ROS2 Humble, Rove is designed for advanced applications in search and rescue robotics.

## Native installation

```bash
git clone https://github.com/clubcapra/rove.git
vcs import src < rove.repos
colcon build
```

## Work in a docker container

You need to have docker installed and running on your computer. If you use Windows, you will need Xserver to display the GUI. You will also need to copy the sample.env file to .env and change the values to match your system. (Default should work for most setup)

## Adding New Packages

To add a package for Rove, create it using the ROS2 command ([Creating Your First ROS2 Package](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)). Name it starting with `rove_` to ensure Git tracking. For non-Rove specific packages, create a separate repository and add it to `rove.repos`.

## Updating Packages

Update Rove-specific packages directly in this repository. For Capra-related or external packages, update their references in `rove.repos`. Change the Git branch in `rove.repos` as needed and apply updates with `vcs import src < rove.repos`.

## Docker architecture

It's possible to run the entire project into multiple docker containers. Each container can be run independantly and are built using the following structure :

![Docker structure](https://www.plantuml.com/plantuml/svg/VP6zaeGm2CTxdo9Zxrn_nSqMsznJt63aTZERZqmWFlz573UU1ON27twWm8qO2jVW1tRiqOptP5zOp7U01vexPemBHkkGnc4eQ1dYOyDAee_sV3vhc3rE2zABKnuDa6dXCsbzdIta0erVyMS6Gi4sH-5SP2o_O964xbBhZKzONIfISGY5Nnsv58NUNOM5oYccu53mjr8go8NgWOylTAdKC9F0pHgjDRDWpIfKz5MePWS5orWivlT_TjcA4fbf-jfljRr4dMxHNSdiMxn4-x8kYNwYmv4eC_qBo9JdW0nqQTMtUycSvxbXN6hmVm00 "Docker structure")

# Documentation

To update the UML diagram, create a new encoded link on the PlantUML website. Copy the existing UML, make your changes, and then update the markdown file with the new link.
