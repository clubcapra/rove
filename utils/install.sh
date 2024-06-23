#!/usr/bin/env bash

sudo cp ./src/rove_launch_handler/scripts/launch_script.sh /usr/bin/launch_script.sh
sudo cp ./src/rove_launch_handler/scripts/launch_script.service /lib/systemd/system/launch_script.service
sudo systemctl enable launch_script.service