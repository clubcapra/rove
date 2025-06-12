#!/bin/bash

script_dir="$(dirname "$(readlink -f "$0")")"

systemctl disable --now --user launch_script.service
# sudo cp $script_dir/launch_script.service /etc/systemd/user/launch_script.service
# systemctl start --user launch_script.service

sudo systemctl disable --now launch_script.service
sudo cp $script_dir/launch_script.service /etc/systemd/system/launch_script.service
sudo systemctl daemon-reload
sudo systemctl enable --now launch_script.service
