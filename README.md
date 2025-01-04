# Rove

Rove is a robot developed by the Capra team at ÉTS, designed for advanced applications in search and rescue robotics using ROS2 Humble.

## Table of Contents
- [Rove](#rove)
  - [Table of Contents](#table-of-contents)
  - [Installation Options](#installation-options)
    - [Docker Container (Recommended)](#docker-container-recommended)
      - [Windows Setup](#windows-setup)
      - [Linux Setup](#linux-setup)
    - [Native/WSL Installation](#nativewsl-installation)
  - [Running Rove](#running-rove)
    - [Simulation](#simulation)
    - [Real-World launch](#real-world-launch)
    - [Controller Setup](#controller-setup)
    - [Additional Components (Need hardware installation)](#additional-components-need-hardware-installation)
      - [VectorNav](#vectornav)
      - [Robotiq Gripper (Need hardware installation)](#robotiq-gripper-need-hardware-installation)
    - [Foxglove Interface](#foxglove-interface)
  - [Development Guide](#development-guide)
    - [Each time you push to the repository, you need to ensure that black formatting is applied to all new files.](#each-time-you-push-to-the-repository-you-need-to-ensure-that-black-formatting-is-applied-to-all-new-files)
    - [Adding New Packages](#adding-new-packages)
    - [Updating Packages](#updating-packages)
    - [Package Documentation](#package-documentation)
  - [Architecture](#architecture)
    - [Package Structure](#package-structure)
    - [Docker Architecture](#docker-architecture)

## Installation Options

### Docker Container (Recommended)

Using a dev container ensures consistent environments across development and CI/CD pipelines.

#### Windows Setup
1. Install required software:
   - [Docker Desktop](https://www.docker.com/products/docker-desktop)
   - [Visual Studio Code](https://code.visualstudio.com/)
   - [VcXsrv](https://sourceforge.net/projects/vcxsrv/) (Xserver)
   - VSCode [Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension

2. Setup steps:
   - Clone and open the repository
   - Click the green button in VSCode's bottom left corner
   - Select "Remote-Containers: Reopen in Container"
   - Start Xserver with the **-nowgl** option

#### Linux Setup
Follow the same steps as Windows, excluding Xserver installation. Additionally:

1. Update DISPLAY environment variable in .env:
   ```bash
   echo DISPLAY=$DISPLAY
   ```

2. Configure controller node permissions:
   ```bash
   cat /dev/input/event0
   ```

**Note:** Consider [configuring Docker for non-root usage](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)

### Native/WSL Installation

Requires Ubuntu 22.04 LTS.

1. Install prerequisites:
   - [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) (use `ros-humble-desktop-full`)
   - [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/#binary-installation-on-ubuntu)

2. Setup workspace:
   ```bash
   git clone https://github.com/clubcapra/rove.git
   cd rove
   vcs import src < rove.repos
   echo "export GZ_VERSION=harmonic" >> ~/.bashrc && source ~/.bashrc
   sudo rosdep init
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash
   ```

3. (Optional) Hardware package installation:
   ```bash
   vcs import src < rove_hw.repos
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash
   ```

## Running Rove

### Simulation
For WSL users:
```bash
export LIBGL_ALWAYS_INDIRECT=0 export LIBGL_ALWAYS_SOFTWARE=1
```

Launch simulation:
```bash
ros2 launch rove_bringup sim.launch.py
```

You can also launch the simulation with Ovis using the following command:
```bash
ros2 launch rove_bringup sim.launch.py with_ovis:=true
```

### Real-World launch

You can launch the real-world robot using the following command:
```bash
ros2 launch rove_bringup real.launch.py
```

### Controller Setup
USB connection:
```bash
source install/setup.bash
ros2 launch rove_bringup rove_controller_usb.launch.py
```

Bluetooth connection:
```bash
source install/setup.bash
ros2 launch rove_bringup rove_controller_bluetooth.launch.py
```

### Additional Components (Need hardware installation)

#### VectorNav
```bash
ros2 launch rove_bringup vectornav.launch.py
```

#### Robotiq Gripper (Need hardware installation)
```bash
# Launch controller
ros2 launch robotiq_description robotiq_control.launch.py

# Control commands
# Close gripper
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 1, max_effort: 1.0}}"

# Open gripper
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0, max_effort: 1.0}}"

# View in RViz
ros2 launch robotiq_description view_gripper.launch.py
```

### Foxglove Interface

1. Install as service:
   ```bash
   ./utils/install.sh
   ```

2. Or launch manually:
   ```bash
   ros2 launch rove_launch_handler launch_handler.py
   ```

Configuration file location: `utils/ui/capra_ui.json`

## Development Guide

### Each time you push to the repository, you need to ensure that black formatting is applied to all new files.
```bash
black .
```

### Adding New Packages
1. Create using ROS2 package creation commands
2. Name Rove-specific packages with `rove_` prefix
3. External packages should be added to `rove.repos`

### Updating Packages
- Rove packages: Update directly in repository
- External packages: Update references in `rove.repos`
- Apply updates: `vcs import src < rove.repos`

## Architecture

### Package Structure

- **Bringup**: Initializes rover in real-world settings
- **Description**: Contains URDF robot description
- **Gazebo**: Handles simulation environment
- **Navigation**: Integrates nav2 framework
- **SLAM**: Handles mapping and localization

### Docker Architecture
![Docker structure](https://www.plantuml.com/plantuml/svg/VP71Ri8m38RlUOgezwvZq8vnc3XmsLDKJkgLGEj4JbfjU_gr0QgWJHoGVjl_xtouUn-0mz1tyc3r6Ldwm8CE0wCGmOGEPNOTVFJGeZoWGsgGj46V2S6e0r0xszgZvYTZ2zqDIeDZA5huGMLtH-3Uaj6P12zlHPfawulfjpiElUemRz2VWtNHFhNhQ_qWCSbSWSSbCXUfdyOB6uscCL0O3w3ZWgzjLLURUS5BVLbMA_rPhak4jNfhLXLiomq0gjNhymfK2TigFdB2u2tLbWs9Ux1n_WEZjXJ0479qJmtihEkHGhrC_iGOl5F8_9qx4sFip0Fx1I6V4HAa922IPsMUlzyTCz5njdoNcuZT_y55jg3A2NLsBbUNOb6nxSnTy8G-M98HEXhIGoOwdINvFL8pz9tu1G00)

## Package Documentation
- [Rove Bringup](src/rove_bringup/README.md)
- [Rove Description](src/rove_description/README.md)
- [Rove Launch Handler](src/rove_launch_handler/README.md)
- [Rove Navigation](src/rove_navigation/README.md)
- [Rove SLAM](src/rove_slam/README.md)
- [Rove Template](src/rove_template/README.md)
- [Rove Zed](src/rove_zed/README.md)