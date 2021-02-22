#!/usr/bin/env zsh
# Blocks until the desired interface is available

if [ "$1" = "" ] || [ $# -gt 1 ]; then
    echo "Usage: $(basename "$0") gcs_host"
    exit -1
fi

INTERFACE="$1"
SLEEP_TIME=1  # How long to sleep in between checks

echo "Waiting for interface to become available: ${INTERFACE}"
until [[ "$(cat /sys/class/net/${INTERFACE}/operstate)" == "up" ]]; do
    sleep "$SLEEP_TIME"
done
echo "Interface available: ${INTERFACE}"