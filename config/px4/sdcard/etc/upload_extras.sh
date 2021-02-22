#!/usr/bin/env zsh

set -euo pipefail

DEST_DIR="fs/microsd/etc"

echo "Uploading extras.txt to ${DEST_DIR}."
rosrun mavros mavftp -n "$ROS_NAMESPACE"/mavros "cd"
rosrun mavros mavftp -n "$ROS_NAMESPACE"/mavros "cd" "fs/microsd/etc"
rosrun mavros mavftp -n "$ROS_NAMESPACE"/mavros "upload" "extras.txt"
rosrun mavros mavftp -n "$ROS_NAMESPACE"/mavros "cd"
echo "Done."
