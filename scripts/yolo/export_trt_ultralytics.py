from ultralytics import YOLO

model = YOLO("../model/yolo11.pt")
model.export(
    format="engine",
    device=0,               # GPU索引
    half=True,              # FP16加速
    imgsz=(640, 640),       # 输入分辨率
    dynamic=True,
    batch=4,
    # dynamic=False,          # 固定输入尺寸
    # simplify=True,          # 简化ONNX图（默认启用）
    nms=True                # 嵌入NMS后处理
)