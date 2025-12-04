#!/usr/bin/env zsh
set -euo pipefail

# Stored sudo password for automated commands (plaintext as requested)
SUDO_PASSWORD="${SUDO_PASSWORD:-nv}"

# Optional: session name (default: swarm)
SESSION="swarm"

# Optional: seeker img_pub_intervals override (empty = launch default)
IMG_PUB_INTERVALS=""

# Optional: use_image_transport override (empty = launch default)
USE_IMAGE_TRANSPORT=""

usage() {
  cat <<'EOF'
Usage: ./scripts/run.zsh [options] [SESSION [IMG_PUB_INTERVALS]]
Options:
  -s, --session NAME              tmux session name (default: swarm)
  -i, --img-pub-intervals VALUE   seeker img_pub_intervals override
  -t, --use-image-transport BOOL  seeker use_image_transport (true/false)
  -h, --help                      show this help
EOF
}

POSITIONAL_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    -s|--session)
      [[ $# -ge 2 ]] || { echo "Argument required for $1" >&2; exit 1; }
      SESSION="$2"
      shift 2
      ;;
    --session=*)
      SESSION="${1#*=}"
      shift
      ;;
    -i|--img-pub-intervals)
      [[ $# -ge 2 ]] || { echo "Argument required for $1" >&2; exit 1; }
      IMG_PUB_INTERVALS="$2"
      shift 2
      ;;
    --img-pub-intervals=*)
      IMG_PUB_INTERVALS="${1#*=}"
      shift
      ;;
    -t|--use-image-transport)
      [[ $# -ge 2 ]] || { echo "Argument required for $1" >&2; exit 1; }
      USE_IMAGE_TRANSPORT="$2"
      shift 2
      ;;
    --use-image-transport=*)
      USE_IMAGE_TRANSPORT="${1#*=}"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      break
      ;;
    *)
      POSITIONAL_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ ${#POSITIONAL_ARGS[@]} -gt 0 ]]; then
  SESSION="${POSITIONAL_ARGS[0]}"
fi
if [[ ${#POSITIONAL_ARGS[@]} -gt 1 ]]; then
  IMG_PUB_INTERVALS="${POSITIONAL_ARGS[1]}"
fi

# Delay between tmux commands
DELAY=2

# Require tmux
if ! command -v tmux >/dev/null 2>&1; then
  echo "Please install tmux: sudo apt-get install -y tmux"
  exit 1
fi

# Wrapper to run commands with sudo if needed; supports SUDO_PASSWORD env var
run_as_root() {
  if [[ $EUID -eq 0 ]]; then
    "$@"
  elif [[ -n "${SUDO_PASSWORD:-}" ]]; then
    printf '%s\n' "$SUDO_PASSWORD" | sudo -S "$@"
  else
    sudo "$@"
  fi
}

# Ensure Jetson runs at max power/clocks before launching anything
set_power_mode_max() {
  if command -v nvpmodel >/dev/null 2>&1; then
    echo "Setting Jetson power mode to MAX (nvpmodel -m 0)..."
    run_as_root nvpmodel -m 0
  else
    echo "nvpmodel not found; skipping power mode change" >&2
  fi
}

start_jetson_clocks() {
  if command -v jetson_clocks >/dev/null 2>&1; then
    echo "Enabling jetson_clocks..."
    run_as_root jetson_clocks
  else
    echo "jetson_clocks not found; skipping" >&2
  fi
}

set_power_mode_max
start_jetson_clocks

# Helper: run a command in a specific pane with a short pause
run() {
  local pane="$1"; shift
  local cmd="$*"
  echo "[tmux:$SESSION:$pane] $cmd"
  tmux send-keys -t "$SESSION:$pane" "$cmd" C-m
  sleep "$DELAY"
}

SEEKER_EXTRA_ARGS=""
if [[ -n "$IMG_PUB_INTERVALS" ]]; then
  SEEKER_EXTRA_ARGS+=" img_pub_intervals:=$IMG_PUB_INTERVALS"
fi
if [[ -n "$USE_IMAGE_TRANSPORT" ]]; then
  SEEKER_EXTRA_ARGS+=" use_image_transport:=$USE_IMAGE_TRANSPORT"
fi

# Kill existing session with the same name for a clean start
if tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux kill-session -t "$SESSION"
fi

#######################
# Window 0: main (5 panes)
#######################

tmux new-session -d -s "$SESSION" -n main      # 0.0
tmux split-window -h -t "$SESSION:0"           # 0.0 | 0.1
tmux split-window -v -t "$SESSION:0.0"         # 0.0 / 0.2
tmux split-window -v -t "$SESSION:0.1"         # 0.1 / 0.3
tmux split-window -v -t "$SESSION:0.3"         # 0.3 / 0.4
tmux select-layout -t "$SESSION:0" tiled       # arrange as a tidy 5-pane grid

# 1) top-left: start mavros
run 0.0 'roslaunch mavros px4.launch'

# 2) top-right: configure mavlink streams + motion capture
run 0.1 'rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0'   # ATTITUDE_QUATERNION
run 0.1 'rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0'  # HIGHRES_IMU
run 0.1 'rosrun mavros mavcmd long 511 83 5000 0 0 0 0 0'   # ATTITUDE_TARGET
run 0.1 'roslaunch vrpn_client_ros sample.launch server:=10.1.1.198'

# 3) bottom-left: mocap + IMU fusion
run 0.2 'roslaunch ekf nokov.launch'

# 4) bottom-middle: PX4 controller
run 0.3 'roslaunch px4ctrl run_ctrl.launch'

# 5) bottom-right: quad camera
run 0.4 "roslaunch seeker 1seeker_nodelet.launch${SEEKER_EXTRA_ARGS}"

#######################
# Window 1: VIO / observers / control
#######################

tmux new-window -t "$SESSION:1" -n second   # 1.0
tmux split-window -v -t "$SESSION:1"        # 1.0 / 1.1
tmux split-window -h -t "$SESSION:1.1"      # 1.1 | 1.2
tmux select-layout -t "$SESSION:1" tiled

# 1) top-left: restart container then run VIO
run 1.0 'docker rm -f diff_vio >/dev/null 2>&1 || true
docker run -it --rm \
  --net=host \
  --name diff_vio \
  -e ROS_IP=$(hostname -I | awk "{print \$1}") \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -v /home/nv/Wss/swarm_deploy/docker/docker_diff_vio:/root/ros_install/share \
  diff-robot:localization_v1.6 \
  bash -lc "source /root/ros_install/setup.bash && roslaunch fusion_odometry vio_rectify_stereo_seeker_v2.launch"'

# 2) top-right: other drone observer (uncomment if needed)
# run 1.1 'rosrun drone_observer drone_detector.py'

# 3) bottom: self control (uncomment if needed)
# run 1.2 'roslaunch drone_control drone_control.launch'

# Focus pane and attach
tmux select-window -t "$SESSION:0"
tmux select-pane   -t "$SESSION:0.0"
tmux attach -t "$SESSION"
