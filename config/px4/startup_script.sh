#!/bin/sh

. px4-alias.sh

# Start uORB messaging bus
uorb start

# Load paramters from file if they exist
if [ -f eeprom/parameters ]; then
	param load
fi

param set SYS_MC_EST_GROUP 2  # ekf2 (recommended)

# Multi-vehicle setup
param set MAV_SYS_ID $((px4_instance + 1))  # MAVLink system ID
simulator_tcp_port=$((4560 + px4_instance))
udp_offboard_port_local=$((14580 + px4_instance))
udp_offboard_port_remote=$((15540 + px4_instance))
udp_gcs_port_local=$((14570 + px4_instance))

param set BAT_N_CELLS 3  # 3S battery
