#!/usr/bin/env zsh

SESSION_NAME="vswarm_flocking"
USER='fabian'

# Source ROS environment (/opt/ and ~/catkin_ws/) via user .zshrc
source "/home/$USER/.zshrc"

case "$1" in
    start)
        tmuxp load -d "$SESSION_NAME"
        ;;
    stop)
        tmux kill-session -t vswarm
        ;;
    restart|reload|force-reload)
        stop
        start
        ;;
    *)
        echo "Usage: $0 {start|stop|reload}"
        exit 1
        ;;
esac

exit 0