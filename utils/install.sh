#!/usr/bin/env bash
sudo mkdir -p /media/SSD/stable/log/ros2
sudo chmod 775 /media/SSD/stable/log/ros2
sudo chown rove:rove /media/SSD/stable/log/ros2
sudo chmod +x ./src/rove_launch_handler/scripts/update_service.sh
./src/rove_launch_handler/scripts/update_service.sh
