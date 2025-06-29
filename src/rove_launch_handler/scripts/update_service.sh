#!/bin/bash

script_dir="$(dirname "$(readlink -f "$0")")"


sudo systemctl disable --now launch_script.service
sudo systemctl disable --now launch_script_monitor.service
sudo cp $script_dir/launch_script.service /etc/systemd/system/launch_script.service
sudo cp $script_dir/launch_script_monitor.service /etc/systemd/system/launch_script_monitor.service
sudo systemctl daemon-reload
sudo systemctl enable --now launch_script.service
sudo systemctl enable --now launch_script_monitor.service
