#!/usr/bin/env zsh
# Blocks until the ROS node becomes available, checking availability every second

SLEEP_TIME=1  # How long to sleep in between checking for ROS

echo "Waiting for node: $1"
while true; do
    success=$(rosnode ping -c 1 "$1" 2>&1 | grep "xmlrpc")
    if [[ -n "$success" ]] ; then
        break
    fi
done
echo "Node available: $1"