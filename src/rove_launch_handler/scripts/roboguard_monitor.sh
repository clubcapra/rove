#!/usr/bin/bash

CONNECTED=0

function check_topics() {
    ros2 topic list | grep -q '/RoboGuard'
    if [[ $? -eq 1 ]]; then
        echo 0
    else
        echo 1
    fi
}

while [[ 1 ]] ; do
    RES=$(check_topics)

    if [[ $CONNECTED -eq 0 ]]; then
        
        if [[ $RES -eq 1 ]]; then
            CONNECTED=1
            echo Restarting
            sudo systemctl restart launch_script.service
        fi
    else
        if [[ $RES -eq 0 ]]; then
            CONNECTED=0
            echo RoboGuard disconnected
        fi
    fi
    sleep 5

done