"""下视摄像头红色火源检测。见 docs/superpowers/specs/2026-07-16-fire-patrol-design.md
"覆盖巡逻路径"/"APPROACH"一节。参照 circle_pole/Lcode/pole_vision.py 的拆分模式
(纯函数+后台线程类)，但只识别单色(红)、返回2维质心像素偏移(dx_px, dy_px)——
下视摄像头需要同时对准x/y两个方向，跟前置摄像头单轴atan方位角不同。
"""
import os
import threading
import time
from collections import deque
from typing import Optional, Tuple

import cv2
import numpy as np

from Lcode.Logger import logger

CAMERA_FRAME_WIDTH = 1920
CAMERA_FRAME_HEIGHT = 1080

# 火源面积范围：灯罩高度不超过10cm、俯视为近似圆形光斑。上限用于排除大面积
# 反光/其他红色物体误触发(见设计文档审查发现1"误触发风险不可逆")，具体像素
# 数值需现场标定(取决于飞行高度/摄像头视场角)，这里给经验初始值。
MIN_FIRE_AREA_PX = 200
MAX_FIRE_AREA_PX = 50000

# 红色HSV阈值，色相环绕0/180两段取并集。经验初始值，现场需按实际LED光源标定
# (参照circle_pole真机标定红杆子的经验：偏暗/偏亮光源饱和度差异很大)。
HSV_RANGES_RED = [((0, 100, 100), (10, 255, 255)), ((170, 100, 100), (180, 255, 255))]


def detect_fire(frame_bgr, min_area: int = MIN_FIRE_AREA_PX,
                 max_area: int = MAX_FIRE_AREA_PX) -> Optional[Tuple[float, float]]:
    """在BGR图像里找面积在[min_area, max_area]范围内的最大红色连通域，
    返回(dx_px, dy_px) = 质心像素坐标 - 画面中心，没找到时返回None。"""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    height, width = frame_bgr.shape[:2]

    mask = None
    for lower, upper in HSV_RANGES_RED:
        m = cv2.inRange(hsv, np.array(lower), np.array(upper))
        mask = m if mask is None else (mask | m)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_area = 0
    best_cx, best_cy = None, None
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area or area > max_area:
            continue
        if area > best_area:
            moments = cv2.moments(c)
            if moments["m00"] == 0:
                continue
            best_area = area
            best_cx = moments["m10"] / moments["m00"]
            best_cy = moments["m01"] / moments["m00"]

    if best_cx is None:
        return None
    return best_cx - width / 2.0, best_cy - height / 2.0


class SmoothedFireDetector:
    """对detect_fire()逐帧结果做滑动平均，减少单帧噪声导致APPROACH阶段误修正
    (见设计文档"独立的、更保守的伺服增益"一节)。任意一帧丢失目标(None)时清空
    窗口重新积累，不能用陈旧偏移量继续参与平均。"""

    def __init__(self, window: int = 5):
        self.window = window
        self._buf = deque(maxlen=window)

    def update(self, detection: Optional[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
        if detection is None:
            self._buf.clear()
            return None
        self._buf.append(detection)
        if len(self._buf) < self.window:
            return None
        avg_dx = sum(d[0] for d in self._buf) / len(self._buf)
        avg_dy = sum(d[1] for d in self._buf) / len(self._buf)
        return avg_dx, avg_dy


class FireVision:
    """后台线程持续拉下视摄像头帧+红色检测+滑动平均，主循环每tick只读`latest()`
    共享的最新结果，不阻塞30ms主循环(风格与Serial_fc.listen_fc()/pole_vision.PoleVision一致)。
    摄像头打不开时start()返回False，latest()永远返回全None——PATROL态"检测到火情"
    条件永远不满足，等同于纯巡逻场景，不会crash也不会阻塞主循环(见设计文档审查
    发现2"检测线程健康检查缺失"，此处只保证不crash，心跳超时告警留待真机测试阶段)。
    """

    def __init__(self, device: str = "/dev/video0", smooth_window: int = 5):
        self.device = device
        self._lock = threading.Lock()
        self._latest = {"dx_px": None, "dy_px": None, "t": 0.0}
        self._running = False
        self._cap = None
        self._smoother = SmoothedFireDetector(window=smooth_window)

    def start(self) -> bool:
        self._cap = cv2.VideoCapture(self.device)
        if not self._cap.isOpened():
            logger.error(f"下视摄像头打不开({self.device})，火情检测禁用")
            self._cap = None
            return False
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_FRAME_WIDTH)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_FRAME_HEIGHT)
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()
        return True

    def stop(self):
        self._running = False

    def latest(self) -> dict:
        with self._lock:
            return dict(self._latest)

    def _loop(self):
        try:
            while self._running:
                ok, frame = self._cap.read()
                if not ok:
                    time.sleep(0.05)
                    continue
                raw = detect_fire(frame)
                smoothed = self._smoother.update(raw)
                with self._lock:
                    if smoothed is None:
                        self._latest = {"dx_px": None, "dy_px": None, "t": time.time()}
                    else:
                        self._latest = {"dx_px": smoothed[0], "dy_px": smoothed[1], "t": time.time()}
        finally:
            if self._cap is not None:
                self._cap.release()
