#!/usr/bin/env bash

# Environment variables have to be preserved in this script!
# Run as: sudo -E ./wifi_reconnect.sh

set -e

# Check if we have superuser privileges
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root!"
   exit 1
fi

# Check if access point variables are set
if [ -z "$WIFI_SSID" ]; then
    echo 'Environment variable $WIFI_SSID must be set.'
    exit 2
fi

if [ -z "$WIFI_PASS" ]; then
    echo 'Environment variable $WIFI_PASS must be set.'
    exit 3
fi

# Reconnect to wireless access point
WIFI_UUID=$(nmcli c | grep "$WIFI_SSID" | awk '{print $2}')
if [[ "$WIFI_UUID" != "" ]]; then
    nmcli c delete uuid "$WIFI_UUID"
fi
nmcli d wifi connect "$WIFI_SSID" password "$WIFI_PASS"
