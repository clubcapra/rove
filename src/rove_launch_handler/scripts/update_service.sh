#!/bin/bash

script_dir="$(dirname "$(readlink -f "$0")")"

systemctl stop --user launch_script.service
sudo cp $script_dir/launch_script.service /etc/systemd/user/launch_script.service
systemctl start --user launch_script.service
