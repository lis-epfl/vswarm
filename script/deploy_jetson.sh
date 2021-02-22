#!/usr/bin/env bash

set -e

function print_usage() {
    echo "Usage: $0 DRONE_ID"
}

# Check if we have superuser privileges
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root!"
   exit 1
fi

# Check for DRONE_ID command line argument
if [[ "$#" -ne 1 ]]; then
    print_usage
    exit 2
fi
DRONE_ID="$1"

# Set hostname
sed -i -E "s/jetson-[0-9]/jetson-${DRONE_ID}/" /etc/hostname
sed -i -E "s/127.0.1.1 jetson-[0-9]/127.0.1.1 jetson-${DRONE_ID}/" /etc/hosts