#!/usr/bin/env bash
set -e

# ===== 可根据需要修改这两项 =====
NEW_ENV="${1:-$HOME/Wss/swarm_deploy/env_swarm}"
SYS_PY="/usr/bin/python3"
PY_MINOR="3.8"
# ================================

# 1. 从系统环境获取 tensorrt 的安装目录
SYS_TRT="$($SYS_PY - << 'EOF'
import os, inspect
import tensorrt
print(os.path.dirname(inspect.getfile(tensorrt)))
EOF
)"

# 2. 计算虚拟环境的 site-packages 目录
NEW_SITE="$NEW_ENV/lib/python${PY_MINOR}/site-packages"

mkdir -p "$NEW_SITE"
cd "$NEW_SITE"

# 3. 清理已有的同名包（如果有）
rm -rf tensorrt

# 4. 软链接系统的 tensorrt 包
ln -s "$SYS_TRT" tensorrt

# 5. 尝试同步 tensorrt-*.dist-info（如果存在）
SYS_TRT_DIR="$(dirname "$SYS_TRT")"
TRT_INFO="$(ls "$SYS_TRT_DIR" | grep -E 'tensorrt-.*dist-info' | head -n1 || true)"

if [ -n "$TRT_INFO" ]; then
    rm -rf "$NEW_SITE/$TRT_INFO"
    ln -s "$SYS_TRT_DIR/$TRT_INFO" "$NEW_SITE/"
fi

# 6. 在新虚拟环境里简单测试一下
"$NEW_ENV/bin/python" - << 'EOF'
import tensorrt as trt
print("TensorRT version in venv:", trt.__version__)
EOF