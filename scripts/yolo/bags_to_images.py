#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import sys
from pathlib import Path

# 第三方依赖
import cv2
import numpy as np

# ROS 依赖
try:
    import rosbag
except Exception:
    print("未找到 rosbag 模块，请确保安装了 ROS 对应的 python 绑定（如 noetic: python3-rosbag）。")
    raise

try:
    from cv_bridge import CvBridge
except Exception:
    print("未找到 cv_bridge，请安装对应发行版或保证在 ROS 环境下运行。")
    raise


# ========== 工具函数 ==========

def sanitize_topic(topic: str) -> str:
    """将 /a/b/c 变成 a_b_c，避免目录名含 /"""
    return topic.strip("/").replace("/", "_") or "root"

def ensure_dir(p: Path):
    p.mkdir(parents=True, exist_ok=True)

def is_image_msg(msg) -> bool:
    return getattr(msg, "_type", "") == "sensor_msgs/Image"

def is_compressed_image_msg(msg) -> bool:
    return getattr(msg, "_type", "") == "sensor_msgs/CompressedImage"

def msg_to_cv2(bridge: CvBridge, msg, topic_hint: str):
    """将 ROS Image/CompressedImage 转为可保存的 numpy 图像（用 _type 判断，兼容动态类）"""
    if is_image_msg(msg):
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    elif is_compressed_image_msg(msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
        if img is None:
            raise ValueError("cv2.imdecode 失败，CompressedImage 数据可能损坏")
    else:
        raise TypeError("不支持的消息类型: {} / {}".format(type(msg), getattr(msg, "_type", "")))

    # 处理浮点图（如 32FC1 深度图）
    if img.dtype in (np.float32, np.float64):
        enc = getattr(msg, "encoding", "")
        if ("depth" in topic_hint.lower()) or (is_image_msg(msg) and isinstance(enc, str) and "32fc" in enc.lower()):
            # 深度图：米 -> 毫米，uint16
            img = np.nan_to_num(img, nan=0.0, posinf=0.0, neginf=0.0)
            img = (img * 1000.0).clip(0, 65535).astype(np.uint16)
        else:
            # 其他浮点：归一化到 0..255
            valid = np.isfinite(img)
            if np.any(valid):
                mn = float(np.nanmin(img[valid])); mx = float(np.nanmax(img[valid]))
                if mx > mn:
                    img = ((img - mn) / (mx - mn) * 255.0).clip(0, 255).astype(np.uint8)
                else:
                    img = np.zeros_like(img, dtype=np.uint8)
            else:
                img = np.zeros_like(img, dtype=np.uint8)

    return img

def detect_image_topics(bag: rosbag.Bag):
    """从 bag 中自动找出图像话题（raw 与 compressed）"""
    info = bag.get_type_and_topic_info()
    image_topics = []
    for t, tinfo in info.topics.items():
        if tinfo.msg_type in ("sensor_msgs/Image", "sensor_msgs/CompressedImage"):
            image_topics.append((t, tinfo.msg_type))
    return image_topics

def save_image(img, out_path: Path, fmt: str, jpg_quality: int):
    if fmt == "jpg":
        cv2.imwrite(str(out_path.with_suffix(".jpg")), img, [int(cv2.IMWRITE_JPEG_QUALITY), int(jpg_quality)])
    elif fmt == "tiff":
        cv2.imwrite(str(out_path.with_suffix(".tiff")), img)
    else:
        cv2.imwrite(str(out_path.with_suffix(".png")), img)


# ========== 主处理 ==========

def process_bag(bag_path: Path, out_root: Path, suffix: str, topics_cli, all_topics: bool, fmt: str, jpg_quality: int):
    bag_stem = bag_path.stem  # 不含 .bag
    bag_out_dir = out_root / f"{bag_stem}{suffix}"
    ensure_dir(bag_out_dir)

    print(f"\n=== 处理 {bag_path.name} ===")
    bridge = CvBridge()

    with rosbag.Bag(str(bag_path), "r") as bag:
        # 决定导出的话题
        if topics_cli:
            chosen = []
            type_map = {t: ti.msg_type for t, ti in bag.get_type_and_topic_info().topics.items()}
            for t in topics_cli:
                if t not in type_map:
                    print(f"  警告：{t} 不在 bag 中，跳过")
                elif type_map[t] not in ("sensor_msgs/Image", "sensor_msgs/CompressedImage"):
                    print(f"  警告：{t} 不是图像类型（{type_map[t]}），跳过")
                else:
                    chosen.append((t, type_map[t]))
        else:
            chosen = detect_image_topics(bag) if all_topics else []
            if not chosen:
                print("  未指定 --topics 且未开启 --all-topics，或未检测到图像话题，跳过该 bag")
                return

        # 每个话题单独保存
        for topic, msg_type in chosen:
            topic_dir = bag_out_dir / sanitize_topic(topic)
            ensure_dir(topic_dir)

            count = 0
            for _, msg, t in bag.read_messages(topics=[topic]):
                try:
                    img = msg_to_cv2(bridge, msg, topic)
                except Exception as e:
                    print(f"  [{topic}] 第 {count} 张转换失败：{e}")
                    continue

                # 用时间戳命名：<secs>_<nsecs>
                fname = f"{int(t.secs)}_{int(t.nsecs):09d}"
                out_path = topic_dir / fname
                save_image(img, out_path, fmt, jpg_quality)
                count += 1

                if count % 100 == 0:
                    print(f"  [{topic}] 已保存 {count} 张…")
            print(f"  [{topic}] 完成，共 {count} 张，输出目录：{topic_dir}")

def main():
    ap = argparse.ArgumentParser(description="批量将一个文件夹下的 ROS bag 导出为图片序列")
    ap.add_argument("input", help="输入路径：包含 .bag 的文件夹，或单个 .bag 文件")
    ap.add_argument("--out", help="输出根目录（默认与 bag 同级）", default=None)
    ap.add_argument("--suffix", help="每个 bag 的输出文件夹后缀（默认 _images）", default="_images")
    ap.add_argument("--topics", nargs="+", help="要导出的图像话题（可多个）。若不指定且不加 --all-topics，则跳过。")
    ap.add_argument("--all-topics", action="store_true", help="自动检测并导出 bag 内所有图像话题")
    ap.add_argument("--fmt", choices=["png", "jpg", "tiff"], default="png", help="保存格式（默认 png）")
    ap.add_argument("--jpg-quality", type=int, default=95, help="JPG 质量（默认 95）")
    args = ap.parse_args()

    in_path = Path(args.input)
    if not in_path.exists():
        print(f"输入路径不存在：{in_path}")
        sys.exit(1)

    # 解析输入：单个 bag 或文件夹
    if in_path.is_dir():
        bag_files = sorted(in_path.glob("*.bag"))
        if not bag_files:
            print(f"目录中未找到 .bag：{in_path}")
            sys.exit(1)
        out_root = Path(args.out) if args.out else in_path
    else:
        if in_path.suffix != ".bag":
            print("输入若为文件，必须是 .bag")
            sys.exit(1)
        bag_files = [in_path]
        out_root = Path(args.out) if args.out else in_path.parent

    # 依次处理
    for bag in bag_files:
        process_bag(
            bag_path=bag,
            out_root=out_root,
            suffix=args.suffix,
            topics_cli=args.topics,
            all_topics=args.all_topics,
            fmt=args.fmt,
            jpg_quality=args.jpg_quality,
        )

    print("\n全部完成 ✅")

if __name__ == "__main__":
    main()
