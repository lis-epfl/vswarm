#!/usr/bin/env zsh
# Blocks until the ROS master becomes available, checking availability every second

SLEEP_TIME=1  # How long to sleep in between checking for ROS

echo "Waiting for ROS to become available"
until rostopic list > /dev/null 2>&1; do
    sleep "$SLEEP_TIME"
done
echo "ROS available"