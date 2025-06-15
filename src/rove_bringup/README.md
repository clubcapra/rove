# Rove Bringup

This package contains launch files and configurations to bring up the Rove robot in various environments.

## Launch Files

- **sim.launch.py**: Launches the simulation environment, including the robot and necessary sensors.
- **real.launch.py**: Launches the real-world robot with all required nodes and configurations.
- **test.launch.py**: Launches the test environment for testing various functionalities.
- **rove_controller_usb.launch.py**: Launches the USB controller for manual control of the robot.
- **rove_controller_bluetooth.launch.py**: Launches the Bluetooth controller for wireless control of the robot.
- **vectornav.launch.py**: Launches the VectorNav sensor for orientation and navigation.

## Nodes

- **rove_controller_node**: Handles the robot's control logic, including receiving commands and controlling actuators.