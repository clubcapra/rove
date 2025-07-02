#!/usr/bin/bash

script_dir="$(dirname "$(readlink -f "$0")")"

sudo mkdir -p /media/SSD/stable/log/ros2
sudo chmod 775 /media/SSD/stable/log/ros2
sudo chown rove:rove /media/SSD/stable/log/ros2
sudo chmod +x $script_dir/../src/rove_launch_handler/scripts/update_service.sh
sudo cp $script_dir/99-vn300.rules /etc/udev/rules.d/99-vn300.rules
sudo cp $script_dir/99-gripper.rules /etc/udev/rules.d/99-gripper.rules
$script_dir/../src/rove_launch_handler/scripts/update_service.sh
