#!/usr/bin/env zsh

set -x

target_session=$1
shift
window_name=$1
shift

if ! tmux has-session -t "$target_session" 2> /dev/null; then
    tmux new-session -s "$target_session" -d
fi

tmux new-window -t "$target_session" -n "$window_name" "$@"

set +x