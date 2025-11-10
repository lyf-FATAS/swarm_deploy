#!/home/nv/SWARM-Physical/env_swarm/bin/python
# -*- coding: utf-8 -*-
"""
@file    export_pt_to_trt.py
@brief   一键将策略 ckpt（.pt/.pth）导出为 ONNX 与 TensorRT Engine（.engine）
@details
1) 先调用本仓库的 `scripts/policy/export_pt_to_onnx.py` 生成 ONNX；
2) 再调用 `trtexec` 生成 TensorRT 引擎文件（支持动态 batch 配置、FP16/INT8 选项）。
3) 自动为导出文件添加后缀以区分精度/批大小，规则可配置：--suffix-mode auto|always|off
"""

import argparse
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Optional

import torch  # 仅用于从 ckpt 中推断 in_dim（如需）；不会用到 CUDA


# ============================================================
# 工具函数
# ============================================================
def which(program: str) -> Optional[str]:
    """在 PATH 中查找可执行文件。"""
    return shutil.which(program)


def require_file(p: Path, name: str):
    if not p.exists():
        raise FileNotFoundError(f"[ERROR] {name} 不存在：{p}")


def run(cmd: list, cwd: Optional[Path] = None):
    print("→", " ".join(map(str, cmd)))
    proc = subprocess.run(cmd, cwd=cwd, stdout=sys.stdout, stderr=sys.stderr)
    if proc.returncode != 0:
        raise RuntimeError(f"[ERROR] 命令执行失败（返回码 {proc.returncode}）")


def safe_torch_load(path: Path):
    """只在 CPU 上读取，优先 weights_only=True，失败回退。"""
    try:
        return torch.load(path, map_location="cpu", weights_only=True)
    except TypeError:
        return torch.load(path, map_location="cpu")


def infer_in_dim_from_ckpt(ckpt_path: Path, drone_key: Optional[str], policy_key: str) -> int:
    """
    从权重粗略推断输入维度 in_dim：
    - 优先取 net_container.0.weight 的 in_features；
    - 否则用 policy_layer.weight 的 in_features。
    """
    raw = safe_torch_load(ckpt_path)
    root = raw
    if isinstance(raw, dict) and drone_key and drone_key in raw:
        root = raw[drone_key]
    # 取 policy 分支或直接 state_dict
    if isinstance(root, dict) and policy_key in root and isinstance(root[policy_key], dict):
        sd = root[policy_key]
    elif isinstance(root, dict):
        sd = root
    else:
        raise RuntimeError("无法从 ckpt 中解析 state_dict。")

    # 剥离 module.
    if sd and all(isinstance(k, str) and k.startswith("module.") for k in sd.keys()):
        sd = {k[len("module."):]: v for k, v in sd.items()}

    # 优先 net_container.0.weight
    w0 = sd.get("net_container.0.weight")
    if isinstance(w0, torch.Tensor) and w0.ndim == 2:
        return int(w0.shape[1])

    # 其次 policy_layer.weight
    pw = sd.get("policy_layer.weight")
    if isinstance(pw, torch.Tensor) and pw.ndim == 2:
        return int(pw.shape[1])

    raise RuntimeError("无法推断 in_dim：缺少 net_container.0.weight / policy_layer.weight")


def build_suffix(fp16: bool, int8: bool, bmin: int, bopt: int, bmax: int) -> str:
    """构造用于区分精度/批大小的后缀，如：'_fp16_b1-8-32' 或 '_int8_b1'。"""
    # 精度优先级：INT8 > FP16 > FP32
    prec = "int8" if int8 else ("fp16" if fp16 else "fp32")
    if bmin == bopt == bmax:
        btag = f"b{bmin}"
    else:
        btag = f"b{bmin}-{bopt}-{bmax}"
    return f"_{prec}_{btag}"


def apply_suffix(path: Path, suffix: str) -> Path:
    """在文件扩展名前插入后缀。"""
    return path.with_name(path.stem + suffix + path.suffix)


# ============================================================
# 参数
# ============================================================
def build_argparser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="一键导出策略 ckpt 为 ONNX 与 TensorRT Engine"
    )
    # 输入/输出
    ap.add_argument("ckpt", help="输入 ckpt 路径（.pt/.pth）")
    ap.add_argument("--onnx", default=None, help="输出 ONNX 路径（默认与 ckpt 同名 .onnx）")
    ap.add_argument("--engine", default=None, help="输出 TensorRT 引擎路径（默认与 ckpt 同名 .engine）")

    # 文件名后缀行为
    ap.add_argument(
        "--suffix-mode", choices=["auto", "always", "off"], default="auto",
        help=(
            "导出文件名后缀模式："
            "auto=仅在使用默认文件名时添加后缀；"
            "always=无论是否显式指定输出路径都添加后缀；"
            "off=不添加后缀。默认 auto。"
        ),
    )

    # 调用 export_pt_to_onnx.py 的参数透传（核心）
    ap.add_argument("--drone-key", default="drone_0", help="子模型键（默认 drone_0）")
    ap.add_argument("--policy-key", default="policy", help="policy 分支键名（默认 policy）")
    ap.add_argument("--activation", default="elu", choices=["relu", "elu", "gelu", "tanh", "sigmoid"])
    ap.add_argument("--hidden", default=None, help='隐藏层，如 "1024,1024,512"（未指定且 --infer-dims 关闭时，需结合 --in-dim/--out-dim）')
    ap.add_argument("--in-dim", type=int, default=None, help="输入维度（未开 --infer-dims 时必须提供）")
    ap.add_argument("--out-dim", type=int, default=None, help="输出维度（未开 --infer-dims 时必须提供）")
    ap.add_argument("--infer-dims", action="store_true", help="从权重自动推断 in/out/hidden（推荐开启）")
    ap.add_argument("--weights-only", action="store_true", help="加载 ckpt 仅取权重（torch>=2.0）")
    ap.add_argument("--verbose-onnx", action="store_true", help="ONNX 导出脚本的 --verbose")

    # trtexec 配置
    ap.add_argument("--trtexec", default=None, help="trtexec 可执行路径；默认自动从 PATH 或 /usr/src/tensorrt/bin/trtexec 寻找")
    ap.add_argument("--workspace", type=int, default=4096, help="工作空间（MiB），默认 4096")
    ap.add_argument("--fp16", action="store_true", help="启用 FP16 构建")
    ap.add_argument("--int8", action="store_true", help="启用 INT8 构建（需要额外校准流程，本脚本不含校准器）")

    # 动态 batch 形状（仅 batch 维动态，特征维 = in_dim）
    ap.add_argument("--min-batch", type=int, default=1, help="最小 batch")
    ap.add_argument("--opt-batch", type=int, default=8, help="优化 batch")
    ap.add_argument("--max-batch", type=int, default=32, help="最大 batch")

    # 其他
    ap.add_argument("--force", action="store_true", help="若目标文件已存在则覆盖")
    ap.add_argument("--python", default=sys.executable, help="调用 onnx 导出脚本所用 Python 解释器")
    return ap


# ============================================================
# 主流程
# ============================================================
def main():
    ap = build_argparser()
    args = ap.parse_args()

    ckpt_path = Path(args.ckpt).expanduser().resolve()
    require_file(ckpt_path, "ckpt")

    # 输出路径默认与 ckpt 同名
    user_set_onnx = args.onnx is not None
    user_set_engine = args.engine is not None
    onnx_path = Path(args.onnx).expanduser().resolve() if user_set_onnx else ckpt_path.with_suffix(".onnx")
    engine_path = Path(args.engine).expanduser().resolve() if user_set_engine else ckpt_path.with_suffix(".engine")

    # 精度冲突提示：INT8 优先
    if args.int8 and args.fp16:
        print("[INFO] 同时指定了 --int8 与 --fp16，将以 INT8 为准。")

    # 构造后缀
    suffix = build_suffix(fp16=args.fp16 and not args.int8, int8=args.int8,
                          bmin=args.min_batch, bopt=args.opt_batch, bmax=args.max_batch)

    # 应用后缀策略
    def maybe_suffix(path: Path, is_user_set: bool) -> Path:
        if args.suffix_mode == "off":
            return path
        if args.suffix_mode == "auto" and is_user_set:
            return path  # 用户显式给了路径，则不改名
        return apply_suffix(path, suffix)

    onnx_path = maybe_suffix(onnx_path, user_set_onnx)
    engine_path = maybe_suffix(engine_path, user_set_engine)

    # 目标存在时处理
    if onnx_path.exists() and not args.force:
        print(f"[INFO] 已存在 ONNX：{onnx_path}（使用 --force 可覆盖）")
    if engine_path.exists() and not args.force:
        print(f"[INFO] 已存在 Engine：{engine_path}（使用 --force 可覆盖）")

    # ---------------- 1) 调用 ONNX 导出脚本 ----------------
    onnx_script = Path(__file__).parent / "export_pt_to_onnx.py"
    # 兼容从仓库根目录执行
    if not onnx_script.exists():
        onnx_script = Path("scripts/policy/export_pt_to_onnx.py")
    require_file(onnx_script, "export_pt_to_onnx.py")

    onnx_cmd = [
        args.python, str(onnx_script),
        str(ckpt_path), str(onnx_path),
        "--drone-key", args.drone_key,
        "--policy-key", args.policy_key,
        "--activation", args.activation,
    ]

    if args.infer_dims:
        onnx_cmd.append("--infer-dims")
    else:
        if args.in_dim is None or args.out_dim is None:
            raise SystemExit("[ERROR] 未开启 --infer-dims 时，必须提供 --in-dim 与 --out-dim")
        onnx_cmd += ["--in-dim", str(args.in_dim), "--out-dim", str(args.out_dim)]
        if args.hidden:
            onnx_cmd += ["--hidden", args.hidden]

    if args.weights_only:
        onnx_cmd.append("--weights-only")
    if args.verbose_onnx:
        onnx_cmd.append("--verbose")

    # 若需要覆盖，先删旧 onnx
    if onnx_path.exists() and args.force:
        onnx_path.unlink()

    print("\n==== [1/2] 导出 ONNX ====")
    run(onnx_cmd)
    require_file(onnx_path, "导出的 ONNX")

    # ---------------- 2) 调用 trtexec 生成 Engine ----------------
    # 找 trtexec
    trtexec = args.trtexec or which("trtexec") or "/usr/src/tensorrt/bin/trtexec"
    if not Path(trtexec).exists():
        raise FileNotFoundError(
            f"[ERROR] 未找到 trtexec，请通过 --trtexec 指定，或确保其在 PATH 中，"
            f"或安装在 /usr/src/tensorrt/bin/trtexec"
        )

    # 需要 in_dim 用来拼 shapes；若用户没给且开启了 infer-dims，我们再从 ckpt 推断一次
    in_dim = args.in_dim
    if in_dim is None:
        try:
            in_dim = infer_in_dim_from_ckpt(ckpt_path, args.drone_key, args.policy_key)
        except Exception as e:
            raise SystemExit(f"[ERROR] 推断 in_dim 失败，请指定 --in-dim 或开启 --infer-dims：{e}")

    # 形状字符串（仅 batch 动态）
    min_shapes = f"STATES:{args.min_batch}x{in_dim}"
    opt_shapes = f"STATES:{args.opt_batch}x{in_dim}"
    max_shapes = f"STATES:{args.max_batch}x{in_dim}"

    trt_cmd = [
        trtexec,
        f"--onnx={onnx_path.as_posix()}",
        f"--saveEngine={engine_path.as_posix()}",
        f"--minShapes={min_shapes}",
        f"--optShapes={opt_shapes}",
        f"--maxShapes={max_shapes}",
        f"--workspace={args.workspace}",
        "--noDataTransfers",    # 构建时不跑推理数据（加快）
        "--verbose"             # 构建日志更详细（可按需移除）
    ]
    if args.int8:
        trt_cmd.append("--int8")
    elif args.fp16:
        trt_cmd.append("--fp16")

    # 若需要覆盖，先删旧 engine
    if engine_path.exists() and args.force:
        engine_path.unlink()

    print("\n==== [2/2] 构建 TensorRT Engine ====")
    run(trt_cmd)
    require_file(engine_path, "生成的 Engine")

    print(f"\n✅ 完成！\nONNX  : {onnx_path}\nEngine: {engine_path}")


if __name__ == "__main__":
    main()
