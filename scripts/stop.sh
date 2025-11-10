SESSION=swarm   # 按需改

# 给 session 下所有 pane 发送 Ctrl-C
tmux list-panes -t "$SESSION" -a -F "#{session_name}:#{window_index}.#{pane_index}" | while read target; do
    tmux send-keys -t "$target" C-c
done

# 等待一秒
sleep 1

# 关闭 docker 容器
docker rm -f diff_vio >/dev/null 2>&1 || true

# 直接杀掉整个 session（所有窗口+pane 一起关掉）
tmux kill-session -t "$SESSION"
