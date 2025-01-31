#!/usr/bin/env bash
sudo mkdir -p /var/log/ros2
sudo chmod 775 /var/log/ros2
sudo cp ./src/rove_launch_handler/scripts/launch_script.service /lib/systemd/system/launch_script.service
sudo systemctl daemon-reload
sudo systemctl enable launch_script.service
sudo systemctl start launch_script.service

# setup udevrules
echo SUBSYSTEM=='"video4linux"', ATTR{index}=='"0"', ATTRS{idVendor}=='"2e1a"', SYMLINK+='"insta"' | sudo tee /etc/udev/rules.d/99-insta.rules
sudo udevadm trigger
