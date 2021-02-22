#!/usr/bin/env zsh
# Blocks until the ROS master becomes available, checking availability every second

if [ "$1" = "" ] || [ $# -gt 1 ]; then
    echo "Usage: $(basename "$0") gcs_host"
    exit -1
fi

GCS_IP="$1"
PING_DEADLINE=1  # Ping deadline in seconds
PING_COUNT=1  # How man pings to send
SLEEP_TIME=1  # How long to sleep in between pings

echo "Pinging ground control station (${GCS_IP})"
until ping -c "$PING_COUNT" -n -w "$PING_SLEEP" "$GCS_IP" &> /dev/null; do
    sleep "$SLEEP_TIME"
done
echo "Successfully pinged ground control station (${GCS_IP})"