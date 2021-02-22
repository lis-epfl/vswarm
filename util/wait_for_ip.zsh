#!/usr/bin/env zsh
# This script waits until a connection to the default gateway can be established

until ip route show | grep -q 'default'; do
    sleep 1
done