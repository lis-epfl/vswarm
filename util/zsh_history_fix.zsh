#!/usr/bin/env zsh

# Fixes a corrupt Zsh history file (~/.zsh_history) after reboot

mv ~/.zsh_history ~/.zsh_history_corrupt
strings ~/.zsh_history_corrupt > ~/.zsh_history
fc -R ~/.zsh_history
rm ~/.zsh_history_corrupt