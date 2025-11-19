#!/usr/bin/env zsh
set -euo pipefail

# If no tmux session exists, exit
if ! tmux list-sessions >/dev/null 2>&1; then
  echo "No tmux sessions found, nothing to stop."
else
  # Send Ctrl-C to every pane in every session
  tmux list-panes -a -F "#{session_name}:#{window_index}.#{pane_index}" | while read -r target; do
      echo "Sending Ctrl-C to $target"
      tmux send-keys -t "$target" C-c
  done

  # Give nodes a moment to exit cleanly
  sleep 1

  # Kill the tmux server (all sessions/windows/panes)
  tmux kill-server
fi

# Stop docker container (same name)
docker rm -f diff_vio >/dev/null 2>&1 || true
