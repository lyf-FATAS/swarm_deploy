#!/home/nv/SWARM-Physical/env_swarm/bin/python

import rospy
import cv2
import math
import time
import threading
import queue
import numpy as np
# import torch
import message_filters
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

class YoloBatchDetector:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('yolo_batch_detector', anonymous=True)

        # 加载YOLO模型（支持.pt/.engine等格式）
        self.model_path = rospy.get_param('~model_path', '/home/nv/Wss/swarm_deploy/src/state_estimation/drone_observer/model/yolo11_8_17_b4_fp16.engine')
        self.device = rospy.get_param('~device', 'cuda')  # 'cuda'或'cpu'
        self.model = YOLO(self.model_path, task='detect')

        # Letterbox配置（目标尺寸，默认640x640）
        self.target_size = rospy.get_param('~target_size', (640, 640))  # (width, height)
        self.letterbox_color = rospy.get_param('~letterbox_color', (114, 114, 114))  # 填充色（BGR）

        # CV Bridge初始化（ROS图像与OpenCV转换）
        self.bridge = CvBridge()
        
        # 位置还原比例参数
        self.cam_ratio = 320.0 * 0.21 # Z = (f*L)/s
        # 相机在 body 系下位姿
        R_cam0_body = np.array([[ 0,  0,  1],
                                     [-1,  0,  0],
                                     [ 0, -1,  0]])
        t_cam0_body = np.array([ 0.065, -0.065,  0])
        
        R_cam1_body = np.array([[-1,  0,  0],
                                     [ 0,  0, -1],
                                     [ 0, -1,  0]])
        t_cam1_body = np.array([-0.065, -0.065,  0])
        
        R_cam2_body = np.array([[ 0,  0, -1],
                                     [ 1,  0,  0],
                                     [ 0, -1,  0]])
        t_cam2_body = np.array([-0.065,  0.065,  0])
        
        R_cam3_body = np.array([[ 1,  0,  0],
                                     [ 0,  0,  1],
                                     [ 0, -1,  0]])
        t_cam3_body = np.array([ 0.065,  0.065,  0])
        self.cam_body = [
            (R_cam0_body, t_cam0_body),
            (R_cam1_body, t_cam1_body),
            (R_cam2_body, t_cam2_body),
            (R_cam3_body, t_cam3_body),
        ]
        
        # 性能统计
        self.alpha = 0.9  # 平滑系数
        self.time_last_frame = time.time()
        self.time_callback = 0
        self.time_inference = 0
        self.fps = 0
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.5
        self.line_thickness = 1
        self.text_color = (0, 255, 0)
        self.bg_color = (30, 30, 30)
        
        # 置信度阈值
        self.conf_threshold = rospy.get_param('~conf_threshold', 0.5)

        # 可视化窗口（可选）
        self.debug_display = rospy.get_param('~debug_display', True)
        if self.debug_display:
            self.other_pos_queue = queue.Queue(maxsize=20)  # 用于存放结果
            self.results_queue = queue.Queue(maxsize=1)     # 用于存放结果
            self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback) # 创建定时器，每50毫秒调用一次处理函数
            self.marker_pub = rospy.Publisher("/other_drones", MarkerArray, queue_size=10)
        
        # 发布话题列表
        self.detected_topics = rospy.get_param('~detected_image_topics', [
            '/yolo/front/right/detected_image',
            '/yolo/right/right/detected_image',
            '/yolo/back/right/detected_image',
            '/yolo/left/right/detected_image'
        ])
        self.publishers = [rospy.Publisher(t, Image, queue_size=10) for t in self.detected_topics]
        self.pub_obs_cloud = rospy.Publisher("/drone_controller/other_drones_obs", PointCloud, queue_size=10)  # 新增：给其他节点

        # 多路摄像头话题
        self.image_topics = rospy.get_param('~image_topics', [
            '/front/right/image_raw',
            '/right/right/image_raw',
            '/back/right/image_raw',
            '/left/right/image_raw'
        ])
        # 同步订阅（近似时间同步）
        subs = [message_filters.Subscriber(topic, Image) for topic in self.image_topics]
        self.ts = message_filters.ApproximateTimeSynchronizer(subs, queue_size=1, slop=0.02)
        self.ts.registerCallback(self.batch_callback)

        rospy.loginfo("YOLO Batch Detector Initialized for topics: %s", self.image_topics)

    def letterbox(self, img, target_size=(640, 640), color=(114, 114, 114)):
        h, w = img.shape[:2]
        tw, th = target_size
        scale = min(tw / w, th / h)
        nw, nh = int(w * scale), int(h * scale)
        resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
        pad_w, pad_h = (tw - nw) // 2, (th - nh) // 2
        padded = cv2.copyMakeBorder(resized, pad_h, pad_h, pad_w, pad_w, cv2.BORDER_CONSTANT, value=color)
        return padded, scale, (pad_w, pad_h)
    
    def to_xyz_array(self, new_points):
        """
        new_points: list[np.ndarray]，每个元素形如 (3,) 或更长
        return: (M,3) float32，剔除非数
        """
        # 尽量一次性堆叠；形状不齐再退化拼接
        try:
            arr = np.stack(new_points, axis=0)
        except ValueError:
            arr = np.vstack([np.asarray(p).ravel() for p in new_points])

        arr = arr.astype(np.float32, copy=False)   # 与 Point32 匹配
        arr = arr[:, :3]                           # 只取 xyz
        mask = np.isfinite(arr).all(axis=1)        # 过滤 NaN/Inf
        return arr[mask]

    def batch_callback(self, *msgs):
        t0 = time.time()
        self.fps = (1-self.alpha)*(1/(t0 - self.time_last_frame)) + self.alpha * self.fps
        self.time_last_frame = t0
        # 转换并Letterbox每张图像
        raw_images, padded_images, scales, pads, msgs_header = [], [], [], [], []
        for msg in msgs:
            try:
                img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except CvBridgeError as e:
                rospy.logerr(f"CV Bridge Error: {e}")
                return
            raw_images.append(img)
            p_img, scale, pad = self.letterbox(img, self.target_size, self.letterbox_color)
            padded_images.append(p_img)
            scales.append(scale)
            pads.append(pad)
            msgs_header.append(msg.header)

        # # 构建 batch tensor，并转换为 float
        # batch_np = np.stack(padded_images, axis=0)             # (4, H, W, C)
        # batch_tensor = torch.from_numpy(batch_np).permute(0,3,1,2).to(self.device)
        # batch_tensor = batch_tensor.float() / 255.0            # 转为 [0,1] 区间浮点

        # 推理
        t1 = time.time()
        results = self.model.predict(
            source=padded_images,
            conf=self.conf_threshold,
            device=0 if self.device == 'cuda' else 'cpu',
            batch = 4,
            verbose=False,
        )
        self.time_inference = (1-self.alpha)*((time.time() - t1) * 1000) + self.alpha * self.time_inference
        
        # 目标位置解算
        new_points = []
        pos_wrt_cam = np.zeros(3)
        # print(f"\n\n================== new msg ====================\n")
        for idx, (raw, scale, pad, res) in enumerate(zip(raw_images, scales, pads, results)):
            for box in res.boxes:
                x1_lb, y1_lb, x2_lb, y2_lb = box.xyxy[0].cpu().numpy()
                # 类1（drone light）
                if int(box.cls[0].cpu().numpy()) == 1:
                    x1 = int(max(0, (x1_lb - pad[0]) / scale))
                    y1 = int(max(0, (y1_lb - pad[1]) / scale))
                    x2 = int(min(raw.shape[1] - 1, (x2_lb - pad[0]) / scale))
                    y2 = int(min(raw.shape[0] - 1, (y2_lb - pad[1]) / scale))
                    length = math.hypot((x2 - x1), (y2 - y1))
                    pos_wrt_cam[2] = self.cam_ratio / length
                    pos_wrt_cam[0] = ((x1 + x2)/2.0 - 320.0) / 320.0 * pos_wrt_cam[2]
                    pos_wrt_cam[1] = ((y1 + y2)/2.0 - 240.0) / 320.0 * pos_wrt_cam[2]
                    pos_wrt_body = self.cam_body[idx][0] @ pos_wrt_cam + self.cam_body[idx][1]
                    new_points.append(pos_wrt_body)
                    self.other_pos_queue.put(pos_wrt_body)
                    # print(f"[{idx}]x={pos_wrt_cam[0]:.2f}, y={pos_wrt_cam[1]:.2f}, z={pos_wrt_cam[2]:.2f}\n")
                    # print(f"[{idx}]x={pos_wrt_body[0]:.2f}, y={pos_wrt_body[1]:.2f}, z={pos_wrt_body[2]:.2f}\n")

        if self.debug_display:
            # 将结果放入队列供可视化线程使用
            self.results_queue.put((raw_images, scales, pads, results, msgs_header))
        
        cloud = PointCloud()
        cloud.header.frame_id = "base_link"
        cloud.header.stamp = msgs[0].header.stamp
        cloud.points = [Point32(x=x, y=y, z=z) for (x, y, z) in new_points]
        # 如需附带置信度/ID，可用 cloud.channels 追加 ChannelFloat32
        self.pub_obs_cloud.publish(cloud)

        self.time_callback = (1-self.alpha)*((time.time() - t0) * 1000) + self.alpha * self.time_callback
        
    def timer_callback(self, event):
        # 1.可视化结果发布
        try:
            # 获取推理结果
            raw_images, scales, pads, results, msg_header = self.results_queue.get(timeout=1)
            
            for idx, (raw, scale, pad, res) in enumerate(zip(raw_images, scales, pads, results)):
                # 绘制检测结果
                drawn = raw.copy()
                for box in res.boxes:
                    x1_lb, y1_lb, x2_lb, y2_lb = box.xyxy[0].cpu().numpy()
                    conf, cls_id = float(box.conf[0].cpu().numpy()), int(box.cls[0].cpu().numpy())
                    name = self.model.names[cls_id]

                    x1 = int(max(0, (x1_lb - pad[0]) / scale))
                    y1 = int(max(0, (y1_lb - pad[1]) / scale))
                    x2 = int(min(raw.shape[1] - 1, (x2_lb - pad[0]) / scale))
                    y2 = int(min(raw.shape[0] - 1, (y2_lb - pad[1]) / scale))

                    cv2.rectangle(drawn, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(drawn, f"{name}: {conf:.2f}", (x1, y1 - 5),
                                self.font, self.font_scale, self.text_color, self.line_thickness)

                # 绘制面板
                self.draw_stats_panel(drawn)
                
                # 发布检测后的图像
                try:
                    msg_out = self.bridge.cv2_to_imgmsg(drawn, 'bgr8')
                    msg_out.header = msg_header[idx]
                    self.publishers[idx].publish(msg_out)
                except CvBridgeError as e:
                    rospy.logerr(f"CV Bridge Publish Error: {e}")
        except queue.Empty:
            pass  # 如果队列为空，继续等待结果
        
        # 2.marker 发布
        # 1) 尝试一次性“掏空”队列（非阻塞）
        new_points = []
        while True:
            try:
                p = self.other_pos_queue.get_nowait()  # 期望是 np.array([x, y, z], dtype=float32/64)
                new_points.append(np.asarray(p).astype(float).ravel()[:3])
            except queue.Empty:
                break

        if not new_points:
            return

        # 2) 构造并发布 MarkerArray
        now = rospy.Time.now()
        ma = MarkerArray()

        # 3) 新点：每个点一个小球
        for i, p in enumerate(new_points):
            m = Marker()
            m.header.frame_id = "base_link"
            m.header.stamp = now
            m.ns = "other_drones"
            m.id = i                      # 同一批内唯一；依靠 lifetime 每次替换
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(p[0])
            m.pose.position.y = float(p[1])
            m.pose.position.z = float(p[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.21
            m.scale.z = 0.08
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 1.0, 1.0
            m.lifetime = rospy.Duration(0.08)  # 0.08s 后自动消失（避免旧点残留）
            ma.markers.append(m)
            
        self.marker_pub.publish(ma)

    def draw_stats_panel(self, image):
        # 统计面板
        ph, pw = 90, 180
        sx, sy = 10, 10
        overlay = image.copy()
        cv2.rectangle(overlay, (sx, sy), (sx + pw, sy + ph), self.bg_color, -1)
        cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)
        cv2.rectangle(image, (sx, sy), (sx + pw, sy + ph), (100,100,100), 1)
        cv2.putText(image, "YOLOv11 Stats", (sx + 10, sy + 20),
                    self.font, self.font_scale, (0,200,255), self.line_thickness)
        cv2.line(image, (sx, sy+25), (sx + pw -10, sy+25), (80,80,80), 1)
        cv2.putText(image, f"FPS: {self.fps:.1f}", (sx+10, sy+45),
                    self.font, self.font_scale, self.text_color, self.line_thickness)
        cv2.putText(image, f"Inf: {self.time_inference:.1f}ms", (sx+10, sy+65),
                    self.font, self.font_scale, self.text_color, self.line_thickness)
        cv2.putText(image, f"Total: {self.time_callback:.1f}ms", (sx+10, sy+85),
                    self.font, self.font_scale, self.text_color, self.line_thickness)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = YoloBatchDetector()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()