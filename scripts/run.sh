#!/usr/bin/env bash
set -e

# 可选参数：SESSION 名；不填默认 swarm
SESSION="${1:-swarm}"
DELAY=2   # 顺序启动的间隔秒数，想改就改这里

# 需要 tmux；没装就提示
if ! command -v tmux >/dev/null 2>&1; then
  echo "请先安装 tmux：sudo apt-get install -y tmux"
  exit 1
fi

# 若已存在同名会话，干净重来
tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

# 窗口0：五宫格（左上、右上、左下、中下、右下）
tmux new-session -d -s "$SESSION" -n main        # 0.0 (左上)
tmux split-window -v -t "$SESSION:0.0"           # 0.1 (右上)
tmux split-window -h -t "$SESSION:0.0"           # 0.2 (左下)
tmux split-window -h -t "$SESSION:0.2"           # 0.3 (中下)
tmux split-window -h -t "$SESSION:0.2"           # 0.4 (右下)

# 1) 左上：起 mavros
tmux send-keys -t "$SESSION:0.0" 'roslaunch mavros px4.launch & sleep 1' C-m & sleep "$DELAY"

# 2) 右上：配置飞控参数，起动捕节点
tmux send-keys -t "$SESSION:0.1" 'rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 0.5' C-m    # ATTITUDE_QUATERNION
tmux send-keys -t "$SESSION:0.1" 'rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 0.5' C-m   # HIGHRES_IMU
tmux send-keys -t "$SESSION:0.1" 'rosrun mavros mavcmd long 511 83 5000 0 0 0 0 0 & sleep 0.5' C-m    # ATTITUDE_TARGET

tmux send-keys -t "$SESSION:0.1" 'roslaunch vrpn_client_ros sample.launch server:=10.1.1.198' C-m & sleep "$DELAY"

# 3) 左下：起 动捕+imu 里程计融合节点
tmux send-keys -t "$SESSION:0.2" 'roslaunch ekf nokov.launch' C-m & sleep "$DELAY"

# 4) 中下：起 px4 控制节点
tmux send-keys -t "$SESSION:0.3" 'roslaunch px4ctrl run_ctrl.launch' C-m & sleep "$DELAY"

# 5) 右下：起四目相机
tmux send-keys -t "$SESSION:0.4" 'roslaunch seeker 3undistort_nodelet.launch' C-m & sleep "$DELAY"

# 窗口1：三宫格（左上、右上、下）
tmux new-window -t "$SESSION:1" -n second   # 1.0
tmux split-window -v -t "$SESSION:1.0"      # 1.1
tmux split-window -h -t "$SESSION:1.0"      # 1.2

# 1) 左上：若已存在同名容器，先删；再起容器并跑 VIO
tmux send-keys -t "$SESSION:1.0" \
'docker rm -f diff_vio >/dev/null 2>&1 || true
docker run -it --rm \
  --net=host \
  --name diff_vio \
  -e ROS_IP=$(hostname -I | awk "{print \$1}") \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -v /home/nv/SWARM-Physical/docker/docker_diff_vio:/root/ros_install/share \
  diff-robot:localization \
  bash -lc "source /root/ros_install/setup.bash && roslaunch fusion_odometry vio_rectify_stereo_seeker_v2.launch"' C-m & sleep "$DELAY"

# 2) 右上：起其他无人机观测节点
# tmux send-keys -t "$SESSION:1.1" 'rosrun drone_observer drone_detector.py' C-m & sleep "$DELAY"

# 3) 下：起自身控制节点
# tmux send-keys -t "$SESSION:1.2" 'roslaunch drone_control drone_control.launch' C-m & sleep "$DELAY"

# 回到窗口0左上并进入会话
tmux select-window -t "$SESSION:0"
tmux select-pane -t "$SESSION:0.0"
tmux attach -t "$SESSION"
