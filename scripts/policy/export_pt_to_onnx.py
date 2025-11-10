#!/home/nv/SWARM-Physical/env_swarm/bin/python
# -*- coding: utf-8 -*-
"""
@file    export_pt_to_onnx.py
@brief   从 PyTorch 检查点 (.pt/.pth) 导出 ONNX（仅导出 policy 分支的 MLP）
@details
- 适配多种 ckpt 结构：{drone_k:{policy:state_dict}}, {policy:state_dict}, 直接 state_dict, 以及 {state_dict:...}
- 自动剥离 "module." 前缀；可仅保留 net_container.* 与 policy_layer.*
- 支持从权重**自动推断** INPUT_DIM/OUTPUT_DIM/隐藏层维度（--infer-dims）
- 激活函数可选：relu/elu/gelu/tanh/sigmoid
- 导出 ONNX：输入名 "STATES"，输出名 "ACTIONS"，batch 维动态，opset=17
"""

import argparse
from typing import List, Optional, Tuple
from pathlib import Path
import sys

import torch
import torch.nn as nn


# ============================================================
# 命令行参数
# ============================================================
def parse_hidden(s: str) -> List[int]:
    """
    将 "1024,1024,512" 或 "[1024, 1024, 512]" 解析为 [1024,1024,512]。
    允许空串/[] 表示无隐藏层（线性直连）。
    """
    if s is None:
        return []
    s = s.strip()
    if not s or s == "[]":
        return []
    s = s.strip("[]").replace(" ", "")
    return [int(x) for x in s.split(",") if x]


def build_argparser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="从 PyTorch ckpt（.pt/.pth）导出 policy MLP 到 ONNX"
    )
    ap.add_argument("input", help="输入 ckpt 路径：.pt/.pth")
    ap.add_argument("output", help="输出 ONNX 路径：.onnx")
    ap.add_argument("--drone-key", default="drone_0",
                    help="子模型键（drone_0/drone_1/...）；若顶层无该键则自动回退到顶层")
    ap.add_argument("--policy-key", default="policy",
                    help="policy 分支在字典中的键名（默认 policy）")
    ap.add_argument("--activation", default="elu",
                    choices=["relu", "elu", "gelu", "tanh", "sigmoid"],
                    help="隐藏层激活函数")
    ap.add_argument("--hidden", type=parse_hidden,
                    default=parse_hidden("1024,1024,1024,512,512,512,256,256,256"),
                    help='隐藏层维度列表，例如 "1024,1024,512" 或 "[1024,1024,512]"')
    ap.add_argument("--in-dim", type=int, default=None,
                    help="输入维度（默认自动推断；若关闭 --infer-dims 则必须提供）")
    ap.add_argument("--out-dim", type=int, default=None,
                    help="输出维度（默认自动推断；若关闭 --infer-dims 则必须提供）")
    ap.add_argument("--infer-dims", action="store_true",
                    help="从权重自动推断 in/out/hidden 维度（优先于 --in-dim/--out-dim/--hidden）")
    ap.add_argument("--weights-only", action="store_true",
                    help="仅加载权重（torch>=2.0更省内存；旧版将自动回退普通加载）")
    ap.add_argument("--device", default="cpu", choices=["cpu"],
                    help="加载设备（固定 cpu，避免 CUDA 依赖）")
    ap.add_argument("--verbose", action="store_true",
                    help="打印更详细的结构/维度信息")
    return ap


# ============================================================
# 基础模块
# ============================================================
def act(name: str) -> nn.Module:
    name = name.lower()
    return {
        "relu": nn.ReLU(),
        "elu": nn.ELU(),
        "gelu": nn.GELU(),
        "tanh": nn.Tanh(),
        "sigmoid": nn.Sigmoid(),
    }[name]


class MLP(nn.Module):
    """
    与权重命名对齐的 MLP：
    - 隐藏层容器命名为 net_container（顺序：Linear, Act, Linear, Act, ...）
    - 输出层命名为 policy_layer
    """
    def __init__(self, in_dim: int, hidden: List[int], out_dim: int, activation: str = "elu"):
        super().__init__()
        layers: List[nn.Module] = []
        prev = in_dim
        a = act(activation)
        for h in hidden:
            layers += [nn.Linear(prev, h), a]
            prev = h
        self.net_container = nn.Sequential(*layers)
        self.policy_layer = nn.Linear(prev, out_dim)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.net_container(x)
        x = self.policy_layer(x)
        return x


# ============================================================
# 权重加载与解析
# ============================================================
def safe_torch_load(path: Path, device: str = "cpu", weights_only: bool = False):
    """
    兼容 torch 版本的安全加载。优先尝试 weights_only=True，失败回退。
    """
    try:
        if weights_only:
            return torch.load(path, map_location=device, weights_only=True)
        return torch.load(path, map_location=device)
    except TypeError:
        return torch.load(path, map_location=device)


def strip_prefix(state: dict, prefix: str) -> dict:
    """剥离键前缀，例如 'module.'。"""
    plen = len(prefix)
    return {k[plen:]: v for k, v in state.items() if k.startswith(prefix)}


def looks_like_statedict(d: object) -> bool:
    """
    粗判是否是 state_dict：键是 str 且值大多为 Tensor。
    """
    if not isinstance(d, dict):
        return False
    if not d:
        return False
    t = sum(isinstance(v, torch.Tensor) for v in d.values())
    return t >= max(1, int(0.5 * len(d)))  # 至少有一半是 Tensor


def pick_policy_state(raw: dict,
                      drone_key: Optional[str] = "drone_0",
                      policy_key: str = "policy",
                      verbose: bool = False) -> dict:
    """
    从多种 ckpt 结构中取出 policy 的 state_dict。
    支持：
      A) { "drone_0": { "policy": { ... } } , ... }
      B) { "policy": { ... } }
      C) 直接就是 { param_name: Tensor, ... }
      D) { "state_dict": { ... } } / { "model_state": { ... } }
    """
    root = raw

    # A) 指定了 drone_key 且存在
    if isinstance(raw, dict) and drone_key in raw:
        root = raw[drone_key]
        if verbose:
            print(f"[INFO] 使用子模型 '{drone_key}' 作为根。")

    # B) 根下存在 policy_key
    if isinstance(root, dict) and policy_key in root and looks_like_statedict(root[policy_key]):
        if verbose:
            print(f"[INFO] 发现 '{policy_key}' 分支，直接使用。")
        return root[policy_key]

    # D) 常见的 state_dict 包裹
    for k in ("state_dict", "model_state", "model", "weights"):
        if isinstance(root, dict) and k in root and looks_like_statedict(root[k]):
            if verbose:
                print(f"[INFO] 在键 '{k}' 下找到 state_dict。")
            return root[k]

    # C) 顶层即 state_dict
    if looks_like_statedict(root):
        if verbose:
            print("[INFO] 顶层看起来就是 state_dict。")
        return root

    # 无法识别
    raise RuntimeError("未能在 ckpt 中找到可用的 policy state_dict。"
                       "可尝试调整 --drone-key/--policy-key 或用 inspect 脚本确认结构。")


def filter_policy_keys(sd: dict, verbose: bool = False) -> dict:
    """
    仅保留 net_container.* 与 policy_layer.*；剥离可选的 "module." 前缀。
    """
    # 剥 "module."
    if sd and all(k.startswith("module.") for k in sd.keys()):
        if verbose:
            print("[INFO] 剥离 'module.' 前缀。")
        sd = strip_prefix(sd, "module.")

    # 仅保留需要的键
    keep = {k: v for k, v in sd.items()
            if k.startswith("net_container.") or k.startswith("policy_layer.")}
    if verbose:
        print(f"[INFO] 过滤后保留 {len(keep)}/{len(sd)} 个参数键。")
    return keep


# ============================================================
# 从权重推断维度（可选）
# ============================================================
def infer_dims_from_state(sd: dict, verbose: bool = False) -> Tuple[int, List[int], int]:
    """
    根据权重键推断：
      - 输入维度：来自 net_container.0.weight 的 in_features
      - 隐藏层列表：按 net_container.<idx>.weight 的 out_features 顺序排列
      - 输出维度：来自 policy_layer.weight 的 out_features
    """
    # 找到所有隐藏层 Linear 的权重
    hidden_pairs = []
    for k, v in sd.items():
        # 形如 'net_container.0.weight' / 'net_container.2.weight' ...
        if k.startswith("net_container.") and k.endswith(".weight") and v.ndim == 2:
            parts = k.split(".")
            try:
                idx = int(parts[1])
            except (ValueError, IndexError):
                continue
            # v: (out_features, in_features)
            out_f = int(v.shape[0])
            in_f = int(v.shape[1])
            hidden_pairs.append((idx, out_f, in_f))

    hidden_pairs.sort(key=lambda x: x[0])
    hidden: List[int] = [p[1] for p in hidden_pairs]

    # 输入维：取第一个 Linear 的 in_features（若不存在则报错）
    if hidden_pairs:
        in_dim = hidden_pairs[0][2]
    else:
        # 没有隐藏层：从 policy_layer.weight 推断
        pw = sd.get("policy_layer.weight", None)
        if pw is None or pw.ndim != 2:
            raise RuntimeError("无法从权重推断输入维度：缺少 net_container.*.weight 或 policy_layer.weight")
        # 若无隐藏层，policy_layer 的输入即 in_dim
        in_dim = int(pw.shape[1])

    # 输出维：来自 policy_layer.weight 的 out_features
    pw = sd.get("policy_layer.weight", None)
    if pw is None or pw.ndim != 2:
        raise RuntimeError("缺少 policy_layer.weight，无法推断输出维度")
    out_dim = int(pw.shape[0])

    if verbose:
        print(f"[INFO] 推断维度：in_dim={in_dim}, hidden={hidden}, out_dim={out_dim}")

    return in_dim, hidden, out_dim


# ============================================================
# 主流程
# ============================================================
def main():
    ap = build_argparser()
    args = ap.parse_args()

    ckpt_path = Path(args.input)
    onnx_path = Path(args.output)

    if not ckpt_path.exists():
        print(f"[ERROR] 输入 ckpt 不存在：{ckpt_path}")
        sys.exit(1)

    # 1) 加载与抽取 policy state_dict
    raw = safe_torch_load(ckpt_path, device=args.device, weights_only=args.weights_only)
    try:
        policy_sd = pick_policy_state(raw, drone_key=args.drone_key,
                                      policy_key=args.policy_key, verbose=args.verbose)
    except Exception as e:
        print(f"[ERROR] 解析 ckpt 失败：{e}")
        sys.exit(1)

    # 2) 过滤键，并剥离 module. 等前缀
    state = filter_policy_keys(policy_sd, verbose=args.verbose)

    if not state:
        print("[ERROR] 过滤后的 state_dict 为空。请确认 ckpt 内含有 net_container.* / policy_layer.*。")
        sys.exit(1)

    # 3) 维度配置：自动推断或使用用户提供
    if args.infer_dims:
        in_dim, hidden, out_dim = infer_dims_from_state(state, verbose=args.verbose)
    else:
        if args.in_dim is None or args.out_dim is None:
            print("[ERROR] 未开启 --infer-dims 时，必须显式提供 --in-dim 与 --out-dim。")
            sys.exit(1)
        in_dim = int(args.in_dim)
        out_dim = int(args.out_dim)
        hidden = list(args.hidden or [])

    if args.verbose:
        print(f"[INFO] 最终维度：in_dim={in_dim}, hidden={hidden}, out_dim={out_dim}")
        # 简要打印关键权重形状
        for k in ("net_container.0.weight", "policy_layer.weight"):
            if k in state and isinstance(state[k], torch.Tensor):
                print(f"[INFO] {k}: {tuple(state[k].shape)} {state[k].dtype}")

    # 4) 构建模型并加载权重
    model = MLP(in_dim, hidden, out_dim, activation=args.activation).eval()
    missing, unexpected = model.load_state_dict(state, strict=False)

    if missing or unexpected:
        print("⚠️  state_dict 键不完全匹配（信息性提示）")
        if missing:
            print("   Missing   :", missing)
        if unexpected:
            print("   Unexpected:", unexpected)

    # 5) 导出 ONNX
    dummy = torch.randn(1, in_dim)
    try:
        torch.onnx.export(
            model, dummy, onnx_path.as_posix(),
            input_names=["STATES"], output_names=["ACTIONS"],
            dynamic_axes={"STATES": {0: "batch"}, "ACTIONS": {0: "batch"}},
            opset_version=17, do_constant_folding=True, verbose=False
        )
    except Exception as e:
        print(f"[ERROR] 导出 ONNX 失败：{e}")
        sys.exit(1)

    print(f"✅ 导出成功 -> {onnx_path}")


if __name__ == "__main__":
    main()
