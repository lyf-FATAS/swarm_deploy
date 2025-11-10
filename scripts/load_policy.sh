#!/usr/bin/env bash
# ============================================
# load_policy.sh
# 将任意 .pt/.pth 转为 ONNX + TensorRT engine，
# 结果统一输出到 <repo_root>/src/drone_control/model/
# 依赖：export_pt_to_trt.py（优先在 scripts/ 下找，退而求其次 scripts/policy/）
# 额外：把输入的 pt/pth 也一并复制到目标目录（同名可用 --force 覆盖）
# ============================================

set -Eeuo pipefail

# ---------- 推断仓库根与默认目标目录 ----------
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$(realpath "${SCRIPT_DIR}/..")"
DEST_DIR_DEFAULT="${REPO_ROOT}/src/drone_control/model"

# ---------- 默认配置（可在命令行覆盖） ----------
DEST_DIR="${DEST_DIR_DEFAULT}"
DRONE_KEY="drone_0"
ACTIVATION="elu"

# 动态/固定 batch（默认固定为 1）
MIN_BATCH=1
OPT_BATCH=1
MAX_BATCH=1

# 构建选项
FP16=0          # 设为 1 开启 --fp16
INT8=0          # 设为 1 开启 --int8（优先于 FP16）
WORKSPACE=4096  # MiB
INFER_DIMS=1    # 设为 0 时必须提供 --in-dim/--out-dim

# 进阶：显式维度（仅当 INFER_DIMS=0 时使用）
IN_DIM=""
OUT_DIM=""
HIDDEN=""       # 例如："1024,1024,1024,512,512,512,256,256,256"

# 其他
TRTEXEC=""      # 指定 trtexec 路径，留空则自动查找
PYTHON_BIN=""   # 指定 python 解释器，留空用当前 python
FORCE=1         # 1 表示覆盖已有文件
SUFFIX_MODE="always"  # auto|always|off，推荐 always 便于区分导出

usage() {
  cat <<EOF
用法：
  $(basename "$0") [选项] <ckpt_path.pt|.pth>

示例：
  # 自动推断维度 + FP16 + 动态 batch(1/8/32) + 追加后缀，输出到 ../src/drone_control/model/
  $(basename "$0") /path/to/xxx.pt --fp16

  # 固定 batch=1（部署常用）+ FP16
  $(basename "$0") /path/to/xxx.pt --fp16 --min-batch 1 --opt-batch 1 --max-batch 1

常用选项：
  --dest DIR              目标目录（默认 ${DEST_DIR_DEFAULT}）
  --drone-key KEY         子模型键（默认 ${DRONE_KEY}）
  --activation NAME       激活函数（默认 ${ACTIVATION}；relu/elu/gelu/tanh/sigmoid）
  --min-batch N           默认 ${MIN_BATCH}
  --opt-batch N           默认 ${OPT_BATCH}
  --max-batch N           默认 ${MAX_BATCH}
  --fp16                  启用 FP16
  --int8                  启用 INT8（优先于 FP16）
  --workspace MiB         默认 ${WORKSPACE}
  --no-infer              不自动推断维度（则需 --in-dim/--out-dim）
  --in-dim N              显式输入维度
  --out-dim N             显式输出维度
  --hidden "a,b,c"        隐藏层配置
  --trtexec PATH          指定 trtexec
  --python PATH           指定 python
  --force                 覆盖已有文件（亦用于覆盖同名 pt/pth 拷贝）
  --suffix-mode MODE      auto|always|off（默认 ${SUFFIX_MODE}）
  -h|--help               查看帮助
EOF
}

# ---------- 解析参数 ----------
CKPT_PATH=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --dest) DEST_DIR="${2}"; shift 2 ;;
    --drone-key) DRONE_KEY="${2}"; shift 2 ;;
    --activation) ACTIVATION="${2}"; shift 2 ;;
    --min-batch) MIN_BATCH="${2}"; shift 2 ;;
    --opt-batch) OPT_BATCH="${2}"; shift 2 ;;
    --max-batch) MAX_BATCH="${2}"; shift 2 ;;
    --fp16) FP16=1; shift ;;
    --int8) INT8=1; shift ;;
    --workspace) WORKSPACE="${2}"; shift 2 ;;
    --no-infer) INFER_DIMS=0; shift ;;
    --in-dim) IN_DIM="${2}"; shift 2 ;;
    --out-dim) OUT_DIM="${2}"; shift 2 ;;
    --hidden) HIDDEN="${2}"; shift 2 ;;
    --trtexec) TRTEXEC="${2}"; shift 2 ;;
    --python) PYTHON_BIN="${2}"; shift 2 ;;
    --force) FORCE=1; shift ;;
    --suffix-mode) SUFFIX_MODE="${2}"; shift 2 ;;
    -*)
      echo "[ERROR] 未知参数：$1"; echo; usage; exit 2 ;;
    *)
      CKPT_PATH="$1"; shift ;;
  esac
done

if [[ -z "${CKPT_PATH}" ]]; then
  echo "[ERROR] 缺少 ckpt 路径"; echo; usage; exit 2
fi

if [[ "${INFER_DIMS}" -eq 0 ]] && { [[ -z "${IN_DIM}" ]] || [[ -z "${OUT_DIM}" ]] ; }; then
  echo "[ERROR] --no-infer 时必须提供 --in-dim 与 --out-dim"
  exit 2
fi

# ---------- 定位导出脚本 ----------
EXPORT_PY_CANDIDATES=(
  "${SCRIPT_DIR}/export_pt_to_trt.py"
  "${SCRIPT_DIR}/policy/export_pt_to_trt.py"
)
EXPORT_PY=""
for p in "${EXPORT_PY_CANDIDATES[@]}"; do
  if [[ -f "$p" ]]; then EXPORT_PY="$p"; break; fi
done
if [[ -z "${EXPORT_PY}" ]]; then
  echo "[ERROR] 未找到 export_pt_to_trt.py（在 scripts/ 或 scripts/policy/ 下都没找到）"
  exit 1
fi

# ---------- 目标目录/文件名 ----------
mkdir -p "${DEST_DIR}"

BASENAME="$(basename -- "${CKPT_PATH}")"
STEM="${BASENAME%.*}"

ONNX_PATH="${DEST_DIR}/${STEM}.onnx"
ENGINE_PATH="${DEST_DIR}/${STEM}.engine"

# ---------- 选择 Python ----------
if [[ -z "${PYTHON_BIN}" ]]; then
  PYTHON_BIN="$(command -v python3 || command -v python || true)"
fi
if [[ -z "${PYTHON_BIN}" ]]; then
  echo "[ERROR] 未找到可用的 python 解释器，请用 --python 指定"
  exit 1
fi

# ---------- 组装命令 ----------
CMD=( "${PYTHON_BIN}" "${EXPORT_PY}" "${CKPT_PATH}"
      --onnx "${ONNX_PATH}" --engine "${ENGINE_PATH}"
      --suffix-mode "${SUFFIX_MODE}"
      --drone-key "${DRONE_KEY}"
      --activation "${ACTIVATION}"
      --workspace "${WORKSPACE}"
      --min-batch "${MIN_BATCH}" --opt-batch "${OPT_BATCH}" --max-batch "${MAX_BATCH}"
      --weights-only
)

# 推断 or 显式维度
if [[ "${INFER_DIMS}" -eq 1 ]]; then
  CMD+=( --infer-dims )
else
  CMD+=( --in-dim "${IN_DIM}" --out-dim "${OUT_DIM}" )
  if [[ -n "${HIDDEN}" ]]; then
    CMD+=( --hidden "${HIDDEN}" )
  fi
fi

# 精度
if [[ "${INT8}" -eq 1 ]]; then
  CMD+=( --int8 )
elif [[ "${FP16}" -eq 1 ]]; then
  CMD+=( --fp16 )
fi

# trtexec/force
if [[ -n "${TRTEXEC}" ]]; then
  CMD+=( --trtexec "${TRTEXEC}" )
fi
if [[ "${FORCE}" -eq 1 ]]; then
  CMD+=( --force )
fi

# ---------- 执行导出 ----------
echo "→ 输出目录: ${DEST_DIR}"
echo "→ 调用: "
printf '   %q ' "${CMD[@]}"; echo
echo
"${CMD[@]}"

# ---------- 复制原始 ckpt 到目标目录 ----------
PT_TARGET="${DEST_DIR}/${BASENAME}"
if [[ "${FORCE}" -eq 1 ]]; then
  cp -f -- "${CKPT_PATH}" "${PT_TARGET}"
else
  # 若不覆盖，目标已存在则给提示
  if [[ -e "${PT_TARGET}" ]]; then
    echo "[INFO] 目标已存在，未覆盖：${PT_TARGET}（使用 --force 可覆盖）"
  else
    cp -- "${CKPT_PATH}" "${PT_TARGET}"
  fi
fi

echo
echo "✅ 完成。请在 ${DEST_DIR} 下查看："
echo " - $(basename -- "${PT_TARGET}")"
echo " - （ONNX 与 Engine 文件名会因 --suffix-mode 追加后缀）"
