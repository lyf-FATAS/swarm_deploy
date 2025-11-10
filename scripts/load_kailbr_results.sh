#!/usr/bin/env bash
# 用法: load_kailbr_results.sh [args...]
# 说明: 始终使用默认的 workdir 与 script，只把传入的参数传递给 python 脚本

set -euo pipefail

# 脚本自身所在目录
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# ===== 默认值 =====
WORKDIR="${SCRIPT_DIR}/../src/seeker1/config/seeker_omni_depth"
SCRIPT="${SCRIPT_DIR}/../src/seeker1/script/3_get_undistort_kalibr_info.py"
# 用当前终端环境的 python（可用环境变量 PYTHON 覆盖）
PY="${PYTHON:-$(command -v python || command -v python3 || true)}"
# =================

if [[ -z "${PY}" ]]; then
  echo "[ERROR] 未找到可用的 python，可设置环境变量 PYTHON 指定解释器" >&2
  exit 1
fi

# 校验
if [[ ! -d "$WORKDIR" ]]; then
  echo "[ERROR] 工作目录不存在：$WORKDIR" >&2
  exit 1
fi
SCRIPT_ABS="$(realpath "$SCRIPT")"
if [[ ! -f "$SCRIPT_ABS" ]]; then
  echo "[ERROR] 找不到脚本：$SCRIPT_ABS" >&2
  exit 1
fi

# 在指定目录下执行（保持当前终端环境）
cd "$WORKDIR"
exec "$PY" "$SCRIPT_ABS" "$@"
