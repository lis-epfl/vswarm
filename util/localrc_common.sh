#!/usr/bin/env zsh

# IMPORTANT: this script requires DRONE_ID environment variable to be set!
# export DRONE_ID=1

# PATH
export PATH="$PATH:$HOME/.local/bin"

# ROS
source /opt/ros/melodic/setup.zsh
source ~/catkin_ws/devel/setup.zsh
export PYTHONPATH="$HOME/catkin_ws/devel/lib/python3/dist-packages:$PYTHONPATH"

# Multimaster ROS
export ROS_MASTER_URI='http://localhost:11311'
export ROS_IP="192.168.100.20${DRONE_ID}"  # wlan: *.201, eth: *.101
export ROS_NAMESPACE="drone_${DRONE_ID}"

# MAVROS
# Connection URL reference: https://github.com/mavlink/mavros/blob/master/mavros/README.md#connection-url
export MAVROS_FCU_URL='/dev/ttyTHS2:921600'
# Permanent broadcast fills up the TX queue too quickly!
export MAVROS_GCS_URL='udp-b://@'  # broadcast
# export MAVROS_GCS_URL='udp://@localhost'  # localhost
export MAVROS_TGT_SYSTEM="$DRONE_ID"

# PX4
export NUM_AGENTS=3

# Football field Chavannes (10 into the field)
export PX4_HOME_LAT=46.5259443
export PX4_HOME_LON=6.5698368
export PX4_HOME_ALT=450.0

# Domaine de Sien (5m into field)
# export PX4_HOME_LAT=46.5336256
# export PX4_HOME_LON=6.5248574
# export PX4_HOME_ALT=501.381296119

# Rooftop EPFL MED
# export PX4_HOME_LAT=46.5197445
# export PX4_HOME_LON=6.5671712
# export PX4_HOME_ALT=466.8

# Python virtualenvwrapper
source /usr/local/bin/virtualenvwrapper.sh

# Set Jetson fan PWM to value from 0 to 255
# Command: echo "X" | sudo tee "$JETSON_FAN_PWM"
export JETSON_FAN_PWM='/sys/devices/pwm-fan/target_pwm'

# tmuxp
export DISABLE_AUTO_TITLE='true'

# Wi-Fi
export WIFI_SSID='amplifi'
export WIFI_PASS='correct-horse-battery-staple'  #  Change me https://xkcd.com/936/ ;)

# Spinnaker config for FLIR
export SPINNAKER_CAM_IMU_TIMESHIFT=0.007908459002226685  # in seconds

export DETECTION_CONFIG_PATH="${HOME}/pytorch_models/yolov3_tiny/yolov3-tiny.cfg"
export DETECTION_CHECKPOINT_PATH="${HOME}/pytorch_models/yolov3_tiny/yolov3-tiny.pt"

# Aliases
alias fd="fdfind"

# MAVROS aliases
alias mavcmd="rosrun mavros mavcmd -n $ROS_NAMESPACE/mavros"
alias mavftp="rosrun mavros mavftp -n $ROS_NAMESPACE/mavros"
alias mavparam="rosrun mavros mavparam -n $ROS_NAMESPACE/mavros"
alias mavsafety="rosrun mavros mavsafety -n $ROS_NAMESPACE/mavros"
alias mavsetp="rosrun mavros mavsetp -n $ROS_NAMESPACE/mavros"
alias mavsys="rosrun mavros mavsys -n $ROS_NAMESPACE/mavros"
alias mavwp="rosrun mavros mavwp -n $ROS_NAMESPACE/mavros"

alias reboot_pixhawk='rosservice call mavros/cmd/command "{broadcast: false, command: 246, param1: 1}"'

# MAVLINK Shell
alias mavlink_shell="${HOME}/Developer/phd/px4/Tools/mavlink_shell.py 0.0.0.0:14550"

# Rsync
function rsync_code_from_gcs() {
    if [ "$#" -ne 1 ]; then
        echo "Usage: $0 <gcs>"
        return
    fi
    export gcs="$1"
    export src="$HOME/Sync/Developer/phd/"
    export dst="$HOME/Developer/phd/"

    rsync -ahvz --info=progress2 -e ssh "$USER"@"$gcs":"$src" "$dst"  --exclude={'.git','*.pt','*.weights','*.onnx','haptic_drone','px4','runs','*.png','*.jpg','__pycache__','*.pyc'}
}

function rsync_code_to_drones() {
    if [ "$#" -lt 1 ]; then
        echo "Usage: $0 <drone-1> ... <drone-N>"
        return
    fi
    export src="$HOME/Sync/Developer/phd/"
    export dst="$HOME/Developer/phd/"
    for host in "$@"; do
        rsync -ahvz --info=progress2 -e ssh "$src" "$USER"@"$host":"$dst" --exclude={'.git','*.pt','*.weights','*.onnx','haptic_drone','px4','runs','*.png','*.jpg','__pycache__','*.pyc'}
    done
}