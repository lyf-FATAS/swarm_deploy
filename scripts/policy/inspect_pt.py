#!/home/nv/SWARM-Physical/env_swarm/bin/python
# -*- coding: utf-8 -*-
"""
@file    inspect_pt.py
@brief   批量/单个 .pt/.pth 文件结构打印工具
@details 支持从命令行传入单个文件或包含多个 ckpt 的目录；递归打印字典/列表结构，
         对 Tensor 显示维度与 dtype；支持关键词过滤（命中剪枝/命中即全展开）、限制递归深度与每层元素数。
         过滤为不区分大小写；当某层键名命中 --pattern 时，其子树全部显示。
"""

import argparse
import sys
from pathlib import Path
from typing import Optional
import torch

# =============================
# 参数解析
# =============================
ap = argparse.ArgumentParser(
    description="递归打印 PyTorch ckpt（.pt/.pth）内部结构（支持文件或目录）"
)
ap.add_argument("input", help="输入路径：单个 .pt/.pth 文件，或包含这些文件的文件夹")
ap.add_argument("--device", default="cpu", choices=["cpu"],  # 固定到 cpu，避免 cuda 依赖
                help="加载设备（默认 cpu）")
ap.add_argument("--weights-only", action="store_true",
                help="仅加载权重（torch>=2.0 支持），更快更省内存")
ap.add_argument("--pattern", nargs="*", default=None,
                help="仅打印键名包含这些子串的条目（可多项，任一匹配即通过；不区分大小写）")
ap.add_argument("--max-depth", type=int, default=None,
                help="限制递归最大深度（0 只打印顶层类型）")
ap.add_argument("--max-items", type=int, default=None,
                help="每一层最多打印的元素数量（字典 key 或列表项）")
ap.add_argument("--show-scalars", action="store_true",
                help="对于非 Tensor 的标量（int/float/str/bool 等）直接打印其值")
args = ap.parse_args()

# =============================
# 过滤 / 剪枝辅助
# =============================
def match_pattern(key: str) -> bool:
    """若未指定 pattern，则全部通过；否则任意关键词命中即通过（不区分大小写）。"""
    if not args.pattern:
        return True
    k = str(key).lower()
    return any(p.lower() in k for p in args.pattern)

def subtree_has_match(obj) -> bool:
    """
    当指定了 --pattern 时，判断该对象子树是否存在“命中键”。
    - dict: 任一 key 命中 或 任一 value 的子树命中 -> True
    - list/tuple: 任一元素子树命中 -> True
    - 其它（Tensor/标量等）: False（是否展示由父层键名匹配决定）
    未指定 --pattern 时，恒为 True（不剪枝）。
    """
    if not args.pattern:
        return True

    if isinstance(obj, dict):
        if any(match_pattern(k) for k in obj.keys()):
            return True
        return any(subtree_has_match(v) for v in obj.values())

    if isinstance(obj, (list, tuple)):
        return any(subtree_has_match(v) for v in obj)

    return False

# =============================
# 加载
# =============================
def safe_torch_load(path: Path):
    """兼容不同 torch 版本的加载：优先 weights_only=True，失败则回退。"""
    try:
        use_weights_only = getattr(args, "weights_only", False)  # argparse 生成的是 weights_only
        if use_weights_only:
            return torch.load(path, map_location=args.device, weights_only=True)
        else:
            return torch.load(path, map_location=args.device)
    except TypeError:
        # 老版本 torch 无 weights_only 参数；统一回退普通加载
        return torch.load(path, map_location=args.device)

# =============================
# 打印
# =============================
def print_nested(obj, level: int = 0, key_name: Optional[str] = None, force: bool = False):
    """
    递归打印任意层级对象：
    - dict: 打印 key -> 类型/张量信息
    - list/tuple: 打印索引 -> 类型/张量信息
    - Tensor: 打印形状和 dtype
    - 常见标量: 可选打印值（--show-scalars）

    过滤策略：
    - 若 force=True（某祖先键命中），则该子树全部打印（无剪枝）。
    - 否则：仅当“键命中或子树命中”时才打印/下钻。
    """
    # 深度限制（根为 0；子内容默认层级 +2 以增强可读性）
    if args.max_depth is not None and level > args.max_depth:
        return

    # 若未强制展开，则按命中剪枝
    if not force and args.pattern:
        key_hit = (key_name is not None and match_pattern(key_name))
        tree_hit = key_hit or subtree_has_match(obj)
        if not tree_hit:
            return
        # 当前键命中，则从这一层开始对子树强制展开
        force = force or key_hit

    indent = "    " * level

    # Tensor
    if isinstance(obj, torch.Tensor):
        name = key_name if key_name is not None else "<tensor>"
        print(f"{indent}{name}: Tensor {tuple(obj.shape)} {obj.dtype}")
        return

    # 标量
    scalar_types = (int, float, bool, str, bytes, type(None))
    if isinstance(obj, scalar_types):
        if key_name is None:
            print(f"{indent}{type(obj).__name__}")
        else:
            if args.show_scalars:
                val = obj
                if isinstance(val, (str, bytes)) and len(val) > 200:
                    val = (val[:200] + "...") if isinstance(val, str) else (val[:200] + b"...")
                print(f"{indent}{key_name}: {type(obj).__name__} = {val}")
            else:
                print(f"{indent}{key_name}: {type(obj).__name__}")
        return

    # dict
    if isinstance(obj, dict):
        header = f"{key_name}: dict" if key_name is not None else "dict"
        print(f"{indent}{header}")

        # 先过滤，再限流；若 force 则不过滤
        if force or not args.pattern:
            items = list(obj.items())
        else:
            items = [(k, v) for (k, v) in obj.items()
                     if match_pattern(k) or subtree_has_match(v)]

        printed = 0
        for k, v in items:
            if args.max_items is not None and printed >= args.max_items:
                break

            if isinstance(v, torch.Tensor):
                print(f"{indent}    {k}: Tensor {tuple(v.shape)} {v.dtype}")
                printed += 1
                continue

            # 容器：直接把键名传给递归，避免重复“dict”标题
            if isinstance(v, (dict, list, tuple)):
                child_force = force or (args.pattern and match_pattern(k))
                print_nested(v, level + 1, k, child_force)
                printed += 1
                continue

            # 其它对象/标量（非容器）：
            if isinstance(v, (int, float, bool, str, bytes, type(None))):
                if args.show_scalars and not isinstance(v, (dict, list, tuple, torch.Tensor)):
                    val = v
                    if isinstance(val, (str, bytes)) and len(val) > 200:
                        val = (val[:200] + "...") if isinstance(val, str) else (val[:200] + b"...")
                    print(f"{indent}    {k}: {type(v).__name__} = {val}")
                else:
                    print(f"{indent}    {k}: {type(v).__name__}")
            else:
                print(f"{indent}    {k}: {type(v).__name__}")
            printed += 1
        return

    # list / tuple
    if isinstance(obj, (list, tuple)):
        header = f"{key_name}: {type(obj).__name__}" if key_name is not None else type(obj).__name__
        print(f"{indent}{header}")

        printed = 0
        for i, v in enumerate(obj):
            if not force and args.pattern and not subtree_has_match(v):
                continue
            if args.max_items is not None and printed >= args.max_items:
                break

            # Tensor
            if isinstance(v, torch.Tensor):
                print(f"{indent}    [{i}]: Tensor {tuple(v.shape)} {v.dtype}")
                printed += 1
                continue

            # 容器：把索引当作“键名”传递，避免重复标题
            if isinstance(v, (dict, list, tuple)):
                print_nested(v, level + 1, f"[{i}]", force)
                printed += 1
                continue

            # 标量 / 其它对象
            if isinstance(v, (int, float, bool, str, bytes, type(None))):
                if args.show_scalars:
                    val = v
                    if isinstance(val, (str, bytes)) and len(val) > 200:
                        val = (val[:200] + "...") if isinstance(val, str) else (val[:200] + b"...")
                    print(f"{indent}    [{i}]: {type(v).__name__} = {val}")
                else:
                    print(f"{indent}    [{i}]: {type(v).__name__}")
            else:
                print(f"{indent}    [{i}]: {type(v).__name__}")
            printed += 1
        return

    # 其它对象：只打印类型名
    if key_name is None:
        print(f"{indent}{type(obj).__name__}")
    else:
        print(f"{indent}{key_name}: {type(obj).__name__}")

def handle_one_file(path: Path):
    print("=" * 80)
    print(f"加载文件: {path}")
    try:
        ckpt = safe_torch_load(path)
    except Exception as e:
        print(f"[ERROR] 加载失败：{e}")
        return

    # 若设置了 pattern 但整个树都不命中，提示一下
    if args.pattern and not subtree_has_match(ckpt):
        print("顶层类型:", type(ckpt).__name__)
        print(f"[INFO] 未找到命中键：{', '.join(args.pattern)}")
        return

    print(f"顶层类型: {type(ckpt).__name__}")
    print_nested(ckpt, level=0, key_name=None, force=False)

# =============================
# 主流程：文件/目录两用
# =============================
in_path = Path(args.input)
if not in_path.exists():
    print(f"[ERROR] 输入路径不存在：{in_path}")
    sys.exit(1)

if in_path.is_file():
    if in_path.suffix.lower() not in (".pt", ".pth"):
        print(f"[WARN] 非 .pt/.pth 文件：{in_path.name}，仍尝试加载…")
    handle_one_file(in_path)
else:
    # 目录：遍历所有 .pt/.pth
    files = sorted([p for p in in_path.rglob("*") if p.suffix.lower() in (".pt", ".pth")])
    if not files:
        print(f"[WARN] 目录下未找到 .pt/.pth 文件：{in_path}")
    for f in files:
        handle_one_file(f)
