"""前置摄像头红/绿杆塔颜色检测 — 纯函数部分，不依赖硬件，方便独立单元测试。
后台线程封装(PoleVision类)见本文件下半部分，见2026-07-14设计文档"视觉子系统"一节。
"""
import math

import cv2
import numpy as np

CAMERA_FOCAL_PX = 1100.0  # 已标定焦距，见 drone_control/tools/camera_test_20260713/
CAMERA_FRAME_WIDTH = 1920
MIN_CONTOUR_AREA_PX = 200

# HSV阈值为经验初始值，真机测试计划第1步(原地悬停+颜色识别)会现场标定调整，
# 见2026-07-14设计文档"视觉子系统"一节。红色注意色相环绕0/180两段，取并集。
HSV_RANGES = {
    "red": [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (180, 255, 255))],
    "green": [((40, 80, 50), (85, 255, 255))],
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
