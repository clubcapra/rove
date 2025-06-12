#!/usr/bin/env bash
sudo mkdir -p /var/log/ros2
sudo chmod 775 /var/log/ros2
sudo chown rove:rove /var/log/ros2
sudo chmod +x ./src/rove_launch_handler/scripts/update_service.sh
./src/rove_launch_handler/scripts/update_service.sh
