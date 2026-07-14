"""前置摄像头红/绿杆塔颜色检测 — 纯函数部分，不依赖硬件，方便独立单元测试。
后台线程封装(PoleVision类)见本文件下半部分，见2026-07-14设计文档"视觉子系统"一节。
"""
import math
import os
import threading
import time

import cv2
import numpy as np

from Lcode.Logger import logger

CAMERA_FOCAL_PX = 1100.0  # 已标定焦距，见 drone_control/tools/camera_test_20260713/
CAMERA_FRAME_WIDTH = 1920
MIN_CONTOUR_AREA_PX = 200

# HSV阈值。红色注意色相环绕0/180两段，取并集。
# 2026-07-14现场标定：红色初始值(S_min=120)完全没识别到实际赛题杆子——真杆子
# 是偏暗的砖红/暗红色，不是鲜艳正红，实测干净区域(排除杆身反光高光条纹后)
# S集中在41~98(中位69)、V集中在94~183(中位133)，比初始假设的鲜艳红低得多。
# 调整为S_min=35/V_min=80，真机验证过(照片见camera_test_20260714)：能正确框住
# 杆子本体(最大连通域约12000px)，明显大于背景干扰物(<4000px)。
# 绿色同一天标定：第一次远距离采样(~5m)测到的是几乎无饱和度的灰绿色(S中位
# 14~24)，跟这次调近距离(~2m)后重新采样的结果(S集中26~89，中位54)差异巨大——
# 说明这颗摄像头下绿色的可辨识饱和度对距离/曝光很敏感，远距离时颜色信号可能
# 弱到不可用，需要后续在APPROACHING实际接近距离下复测确认。当前阈值按近距离
# 样本调，真机验证过：最大连通域约16000px，明显大于最大干扰物(<5000px)。
# 2026-07-14真机测试暴露绿色阈值(hue上限90)偏松：现场只摆红杆子测试时，红杆子
# 被正确识别(red连通域13094px)，但背景(地面防潮布/柜子一带，低饱和度灰色)被
# 误判成更大的绿色连通域(22309px, H集中80~95/S集中23~37/V集中137~172)，area
# 比较逻辑选中了假绿色，实际真机复现"红杆子被识别成绿色目标"。把hue上限从90
# 收紧到75、S_min从25提到30，排除掉这块灰背景(H 80~95已在新范围外，S多数样本
# 在23~37也被新S_min过滤掉大半)。**这次调整只用背景负样本验证过，没有用真实
# 绿色杆子正样本复核**，下次真机测试前必须先拿绿色杆子单独现场测一次确认still能
# 正确识别，不能想当然。
HSV_RANGES = {
    "red": [((0, 35, 80), (10, 255, 255)), ((170, 35, 80), (180, 255, 255))],
    "green": [((40, 30, 60), (75, 255, 255))],
}


def detect_target(frame_bgr, colors=("red", "green"), hsv_ranges=None,
                   min_area=MIN_CONTOUR_AREA_PX):
    """在BGR图像里找`colors`范围内面积最大的连通域，返回(dx_px, color)。

    dx_px = 目标质心像素x - 画面中心x，没找到任何满足面积阈值的目标时返回(None, None)。
    colors参数用于颜色锁定(APPROACHING阶段只传锁定的那一个颜色，忽略画面里出现
    的另一色，见2026-07-14设计文档"颜色锁定"一节)。
    """
    ranges = hsv_ranges or HSV_RANGES
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    width = frame_bgr.shape[1]

    best_area = 0
    best_color = None
    best_cx = None

    for color in colors:
        mask = None
        for lower, upper in ranges[color]:
            m = cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask = m if mask is None else (mask | m)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area >= min_area and area > best_area:
            moments = cv2.moments(c)
            if moments["m00"] == 0:
                continue
            best_area = area
            best_color = color
            best_cx = moments["m10"] / moments["m00"]

    if best_color is None:
        return None, None
    return best_cx - width / 2.0, best_color


def azimuth_from_dx(dx_px, focal_px=CAMERA_FOCAL_PX):
    """像素偏移换算成方位角(弧度)，见2026-07-13设计文档"阶段2视觉辅助方案"一节的公式。"""
    return math.atan(dx_px / focal_px)


class PoleVision:
    """后台线程持续拉前置摄像头帧+HSV检测，主循环每tick只读`latest()`共享的最新
    结果，不阻塞30ms主循环通信实时性(见2026-07-14设计文档"视觉子系统"一节)。

    摄像头打不开时`start()`返回False、不起线程，`latest()`永远返回全None——
    PATROL态的"雷达+视觉双确认"触发条件因此永远不满足，等同于阶段1纯雷达场景，
    不会抛异常也不会阻塞主循环(2026-07-14审查记录的已知风险3：视觉系统整体故障
    时任务会一直卡在PATROL直到超时，此处只保证不crash，卡死风险本身按之前讨论
    "先记着，等真机测试暴露出来再处理"，不在本次范围内解决)。
    """

    def __init__(self, device="/dev/video0"):
        self.device = device
        self._lock = threading.Lock()
        self._latest = {"dx_px": None, "color": None, "t": 0.0}
        self._locked_color = None
        self._running = False
        self._cap = None
        # 2026-07-14排查"识别到非预期颜色"问题时新增：设置DRONE_VISION_DEBUG_DIR后，
        # 每次颜色从无到有/切换时保存一帧现场画面，方便事后核对镜头里实际拍到的是什么。
        self._debug_dir = os.getenv("DRONE_VISION_DEBUG_DIR")
        self._last_saved_color = None

    def start(self):
        self._cap = cv2.VideoCapture(self.device)
        if not self._cap.isOpened():
            logger.error(f"前置摄像头打不开({self.device})，视觉子系统禁用")
            self._cap = None
            return False
        # 2026-07-14真机测试发现：这颗USB摄像头默认给出320x240帧，而
        # CAMERA_FOCAL_PX(1100)是按1920宽度标定的焦距——不显式设置分辨率，
        # detect_target()算出来的dx_px是按实际帧宽(320)算的，但azimuth_from_dx
        # 拿去除的焦距却是按1920宽度标定的，两者对不上会让方位角完全算错。
        # MJPG格式+1920x1080是这颗摄像头支持的最高分辨率，也是标定时用的分辨率。
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_FRAME_WIDTH)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()
        return True

    def stop(self):
        self._running = False

    def set_locked_color(self, color):
        """color为None时匹配红/绿两色(PATROL搜索阶段)，传具体颜色时只匹配该颜色
        (APPROACHING阶段颜色锁定，见2026-07-14设计文档"颜色锁定"一节)。"""
        with self._lock:
            self._locked_color = color

    def latest(self):
        with self._lock:
            return dict(self._latest)

    def _loop(self):
        try:
            while self._running:
                ok, frame = self._cap.read()
                if not ok:
                    time.sleep(0.05)
                    continue
                with self._lock:
                    locked = self._locked_color
                colors = (locked,) if locked else ("red", "green")
                dx_px, color = detect_target(frame, colors=colors)
                if self._debug_dir and color is not None and color != self._last_saved_color:
                    self._save_debug_frame(frame, color)
                self._last_saved_color = color
                with self._lock:
                    self._latest = {"dx_px": dx_px, "color": color, "t": time.time()}
        finally:
            self._cap.release()

    def _save_debug_frame(self, frame, color):
        os.makedirs(self._debug_dir, exist_ok=True)
        path = os.path.join(self._debug_dir, f"{time.strftime('%Y%m%d_%H%M%S')}_{color}.jpg")
        cv2.imwrite(path, frame)
