#!/usr/bin/env zsh
set -e

# ===== Adjust these if needed =====
NEW_ENV="${1:-$HOME/Wss/swarm_deploy/env_swarm}"
SYS_PY="/usr/bin/python3"
PY_MINOR="3.8"
# ================================

# 1) Locate system TensorRT installation
SYS_TRT="$($SYS_PY - << 'EOF'
import os, inspect
import tensorrt
print(os.path.dirname(inspect.getfile(tensorrt)))
EOF
)"

# 2) Compute site-packages path in the target venv
NEW_SITE="$NEW_ENV/lib/python${PY_MINOR}/site-packages"

mkdir -p "$NEW_SITE"
cd "$NEW_SITE"

# 3) Remove any existing TensorRT package
rm -rf tensorrt

# 4) Symlink TensorRT from the system install
ln -s "$SYS_TRT" tensorrt

# 5) Optionally link tensorrt-*.dist-info if present
SYS_TRT_DIR="$(dirname "$SYS_TRT")"
TRT_INFO="$(ls "$SYS_TRT_DIR" | grep -E 'tensorrt-.*dist-info' | head -n1 || true)"

if [ -n "$TRT_INFO" ]; then
    rm -rf "$NEW_SITE/$TRT_INFO"
    ln -s "$SYS_TRT_DIR/$TRT_INFO" "$NEW_SITE/"
fi

# 6) Smoke test inside the venv
"$NEW_ENV/bin/python" - << 'EOF'
import tensorrt as trt
print("TensorRT version in venv:", trt.__version__)
EOF
