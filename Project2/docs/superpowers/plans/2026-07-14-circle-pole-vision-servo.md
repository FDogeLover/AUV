# 绕障飞行器(D题) 阶段2：前置摄像头颜色识别 + 视觉伺服接近 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在`drone_control/circle_pole/`现有阶段1单杆环绕基础上，接入前置摄像头红/绿颜色识别，新增`APPROACHING`状态实现"雷达定x接近距离+视觉定y横向对准"的闭环接近飞行，用颜色替代坐标容差做"是否已环绕过"去重，环绕方向按红=顺时针/绿=逆时针决定，支持双杆全流程（`PATROL→APPROACHING→CIRCLING→退回基准线→PATROL/TO_LANDING→LANDING`）。

**Architecture:** 新增纯函数+后台线程模块`Lcode/pole_vision.py`（HSV颜色检测独立于飞控循环运行，主循环只读最新结果）。`Mission_GPT.py`新增`APPROACHING`/`RETREAT`两个`nav_mode`，`PATROL`态的触发逻辑从"雷达单独确认即环绕"改为"雷达+视觉+颜色去重三条件持续满足才触发接近"，环绕完成后不再直接判定颜色数量，而是先退回`x=0`基准线再决定继续巡航还是转向降落。降落路径从硬编码常量改成`landing_router.txt`航点文件，`router.txt`改名`patrol_router.txt`。

**Tech Stack:** Python 3, pytest, OpenCV(`opencv-python`，新增依赖), 现有`Lcode.Lradar.PoleTracker`/`Lcode.Lpid.PID`/`Lcode.circle_planner`

设计依据：
- [docs/superpowers/specs/2026-07-13-circle-pole-design.md](../specs/2026-07-13-circle-pole-design.md)（阶段1，已实现+真机验证）
- [docs/superpowers/specs/2026-07-14-circle-pole-vision-servo-design.md](../specs/2026-07-14-circle-pole-vision-servo-design.md)（本计划的设计来源，含真机测试前必须复核的2处赛题指标偏差——环绕半径/巡航高度，本计划不涉及）

**范围声明**：本计划不改`POLE_CIRCLE_RADIUS_M`（环绕半径）也不改`patrol_router.txt`巡航高度——这两处是已知的赛题指标偏差，spec文档已标注"真机验证阶段2链路稳定后需要专项测试上调/比赛前必须复核"，属于参数调优，不是本次实现的一部分。

---

### Task 1: `Lcode/pole_vision.py` 颜色检测纯函数（TDD）

**Files:**
- Create: `drone_control/circle_pole/Lcode/pole_vision.py`
- Test: `drone_control/circle_pole/test_pole_vision.py`
- Modify: `drone_control/circle_pole/requirements.txt`

- [ ] **Step 1: 添加 opencv-python 依赖**

```
numpy>=1.21.0
simple_pid>=0.7.0
pyserial>=3.5
opencv-python>=4.5.0
```

Run: `pip install opencv-python` （本机开发环境需要装这个包才能跑本Task后续测试；ubuntu-pi真机部署环境也要装）

- [ ] **Step 2: 写失败测试（此时`pole_vision`模块还不存在）**

```python
"""pole_vision 颜色检测纯函数单元测试。

运行:
    cd drone_control/circle_pole && python -m pytest test_pole_vision.py -v
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

import numpy as np
import pytest

from Lcode.pole_vision import detect_target, azimuth_from_dx, CAMERA_FOCAL_PX


def _blank_frame(width=1920, height=1080):
    return np.zeros((height, width, 3), dtype=np.uint8)


def _draw_rect_bgr(frame, color_bgr, x_center, half_w=60, half_h=200):
    y_center = frame.shape[0] // 2
    frame[y_center - half_h:y_center + half_h,
          x_center - half_w:x_center + half_w] = color_bgr
    return frame


class TestDetectTarget:
    def test_no_target_returns_none_none(self):
        frame = _blank_frame()
        dx_px, color = detect_target(frame)
        assert dx_px is None
        assert color is None

    def test_red_rectangle_detected_as_red(self):
        frame = _blank_frame()
        # OpenCV红色HSV在色相环两端(0附近和180附近)，用纯红BGR(0,0,255)覆盖两段其中一段
        _draw_rect_bgr(frame, (0, 0, 255), x_center=960)
        dx_px, color = detect_target(frame)
        assert color == "red"
        assert dx_px == pytest.approx(0.0, abs=5)

    def test_green_rectangle_detected_as_green(self):
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 255, 0), x_center=960)
        dx_px, color = detect_target(frame)
        assert color == "green"

    def test_dx_px_positive_when_target_right_of_center(self):
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 0, 255), x_center=1200)  # 画面中心960，目标在右侧
        dx_px, color = detect_target(frame)
        assert color == "red"
        assert dx_px > 0

    def test_dx_px_negative_when_target_left_of_center(self):
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 0, 255), x_center=700)
        dx_px, color = detect_target(frame)
        assert color == "red"
        assert dx_px < 0

    def test_colors_filter_ignores_unlisted_color(self):
        """颜色锁定用：只传锁定的颜色列表，画面里出现另一色也应该忽略。"""
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 255, 0), x_center=960)  # 只画绿色
        dx_px, color = detect_target(frame, colors=("red",))  # 只找红色
        assert dx_px is None
        assert color is None

    def test_area_below_min_threshold_ignored(self):
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 0, 255), x_center=960, half_w=2, half_h=2)  # 4x4像素，远小于阈值
        dx_px, color = detect_target(frame)
        assert dx_px is None
        assert color is None


class TestAzimuthFromDx:
    def test_zero_dx_is_zero_azimuth(self):
        assert azimuth_from_dx(0.0) == pytest.approx(0.0)

    def test_positive_dx_gives_positive_azimuth(self):
        assert azimuth_from_dx(500.0) > 0

    def test_negative_dx_gives_negative_azimuth(self):
        assert azimuth_from_dx(-500.0) < 0

    def test_matches_atan_formula_with_default_focal(self):
        dx = 300.0
        expected = math.atan(dx / CAMERA_FOCAL_PX)
        assert azimuth_from_dx(dx) == pytest.approx(expected)
```

- [ ] **Step 3: 运行测试确认失败**

Run: `cd "D:/项目与工具/Python项目/Project2/Project2/drone_control/circle_pole" && python -m pytest test_pole_vision.py -v`
Expected: FAIL（`ModuleNotFoundError: No module named 'Lcode.pole_vision'`）

- [ ] **Step 4: 实现纯函数**

```python
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
```

- [ ] **Step 5: 运行测试确认通过**

Run: `cd "D:/项目与工具/Python项目/Project2/Project2/drone_control/circle_pole" && python -m pytest test_pole_vision.py -v`
Expected: 全部PASS

- [ ] **Step 6: Commit**

```bash
cd "D:/项目与工具/Python项目/Project2/Project2"
git add drone_control/circle_pole/Lcode/pole_vision.py drone_control/circle_pole/test_pole_vision.py drone_control/circle_pole/requirements.txt
git commit -m "feat(circle_pole): 前置摄像头HSV颜色检测纯函数(detect_target/azimuth_from_dx)"
```

---

### Task 2: `PoleVision` 后台线程类（TDD）

**Files:**
- Modify: `drone_control/circle_pole/Lcode/pole_vision.py`
- Modify: `drone_control/circle_pole/test_pole_vision.py`

- [ ] **Step 1: 写失败测试（用假的cv2.VideoCapture，不需要真摄像头）**

追加到 `test_pole_vision.py` 末尾：

```python
import time

from Lcode.pole_vision import PoleVision


class _FakeCapOpenFails:
    def isOpened(self):
        return False


class _FakeCapOneFrame:
    """只在第一次read()返回一帧红色图像，之后一直返回失败，供测试用。"""

    def __init__(self, frame):
        self._frame = frame
        self._served = False

    def isOpened(self):
        return True

    def read(self):
        if not self._served:
            self._served = True
            return True, self._frame
        return False, None


class TestPoleVisionStartFailure:
    def test_start_returns_false_when_camera_cannot_open(self, monkeypatch):
        monkeypatch.setattr("Lcode.pole_vision.cv2.VideoCapture",
                             lambda *_a, **_k: _FakeCapOpenFails())
        pv = PoleVision()
        assert pv.start() is False

    def test_latest_before_any_frame_is_all_none(self):
        pv = PoleVision()
        latest = pv.latest()
        assert latest["dx_px"] is None
        assert latest["color"] is None
        assert latest["t"] == 0.0


class TestPoleVisionLockedColor:
    def test_set_locked_color_updates_state_and_is_readable(self):
        pv = PoleVision()
        assert pv._locked_color is None
        pv.set_locked_color("red")
        assert pv._locked_color == "red"
        pv.set_locked_color(None)
        assert pv._locked_color is None


class TestPoleVisionBackgroundLoop:
    def test_loop_publishes_detection_from_captured_frame(self, monkeypatch):
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 0, 255), x_center=960)
        fake_cap = _FakeCapOneFrame(frame)
        monkeypatch.setattr("Lcode.pole_vision.cv2.VideoCapture", lambda *_a, **_k: fake_cap)

        pv = PoleVision()
        assert pv.start() is True
        # 后台线程读一帧需要一点时间，轮询等待而不是固定sleep
        deadline = time.time() + 2.0
        while pv.latest()["color"] is None and time.time() < deadline:
            time.sleep(0.02)
        pv.stop()

        latest = pv.latest()
        assert latest["color"] == "red"
        assert latest["dx_px"] == pytest.approx(0.0, abs=5)
        assert latest["t"] > 0.0
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd "D:/项目与工具/Python项目/Project2/Project2/drone_control/circle_pole" && python -m pytest test_pole_vision.py -v`
Expected: FAIL（`ImportError: cannot import name 'PoleVision'`）

- [ ] **Step 3: 实现 `PoleVision` 类**

追加到 `Lcode/pole_vision.py` 末尾：

```python
import threading
import time

from Lcode.Logger import logger


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

    def start(self):
        self._cap = cv2.VideoCapture(self.device)
        if not self._cap.isOpened():
            logger.error(f"前置摄像头打不开({self.device})，视觉子系统禁用")
            self._cap = None
            return False
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
        while self._running:
            ok, frame = self._cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            with self._lock:
                locked = self._locked_color
            colors = (locked,) if locked else ("red", "green")
            dx_px, color = detect_target(frame, colors=colors)
            with self._lock:
                self._latest = {"dx_px": dx_px, "color": color, "t": time.time()}
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd "D:/项目与工具/Python项目/Project2/Project2/drone_control/circle_pole" && python -m pytest test_pole_vision.py -v`
Expected: 全部PASS

- [ ] **Step 5: Commit**

```bash
cd "D:/项目与工具/Python项目/Project2/Project2"
git add drone_control/circle_pole/Lcode/pole_vision.py drone_control/circle_pole/test_pole_vision.py
git commit -m "feat(circle_pole): PoleVision后台线程类，主循环非阻塞读取颜色检测结果"
```

---

### Task 3: 航点文件改名 + 新增降落路径文件

**Files:**
- Rename: `drone_control/circle_pole/router.txt` → `drone_control/circle_pole/patrol_router.txt`
- Create: `drone_control/circle_pole/landing_router.txt`

- [ ] **Step 1: 用git mv改名，保留历史**

```bash
cd "D:/项目与工具/Python项目/Project2/Project2"
git mv drone_control/circle_pole/router.txt drone_control/circle_pole/patrol_router.txt
```

- [ ] **Step 2: 新建降落路径文件（占位值，沿用原`LANDING_POINT`常量坐标，现场量出实际降落标识位置后必须修改，见文件内注释）**

```
# 两根杆塔都绕完后的返航降落路径。占位值，沿用阶段1的LANDING_POINT常量坐标，
# 现场量出实际降落标识位置后必须修改。格式跟patrol_router.txt一致：x,y,z逐行，
# 走完最后一个航点后转LANDING。
2.0,0.0,1.2
```

- [ ] **Step 3: Commit**

```bash
cd "D:/项目与工具/Python项目/Project2/Project2"
git add drone_control/circle_pole/patrol_router.txt drone_control/circle_pole/landing_router.txt
git commit -m "feat(circle_pole): router.txt改名patrol_router.txt，新增landing_router.txt返航路径"
```

（此时`Mission_GPT.py`还硬编码读`router.txt`，会在Task 4修复；这一步先改文件本身，保持改名的git历史独立可追溯）

---

### Task 4: `load_waypoints()` 参数化 + `__init__` 接入视觉子系统

**Files:**
- Modify: `drone_control/circle_pole/Mission_GPT.py`

- [ ] **Step 1: `load_waypoints` 加 `path` 参数**

找到 `load_waypoints` 方法（原第205-233行），整体替换为：

```python
    def load_waypoints(self, path='patrol_router.txt'):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                waypoints = []
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#') and not line.startswith('x'):
                        parts = line.split(',')
                        if len(parts) >= 3:
                            try:
                                x = float(parts[0].strip())
                                y = float(parts[1].strip())
                                z = float(parts[2].strip())
                                waypoints.append([x, y, z])
                            except ValueError:
                                logger.warning(f"无效航点: {line}")
                if waypoints:
                    logger.info(f"从{path}加载 {len(waypoints)} 个航点")
                    return waypoints
        except FileNotFoundError:
            logger.warning(f"{path} 不存在，使用默认航点")
        except Exception as e:
            logger.warning(f"读取 {path} 失败: {e}，使用默认航点")

        default = [[0.0, 0.0, put_height/100],
                   [0.5, 0.0, put_height/100],
                   [0.5, 0.5, put_height/100],
                   [0.0, 0.5, put_height/100]]
        return default
```

- [ ] **Step 2: `__init__` 签名加 `pole_vision_obj` 参数，加视觉/去重/接近状态字段**

找到 `__init__` 签名（原第141-143行）：

```python
    def __init__(self, re_fc: List[int], se_fc: List[int],
                 realsense_obj: Optional[t265_class] = None,
                 serial_fc_ref=None, radar_obj=None):
```

替换为：

```python
    def __init__(self, re_fc: List[int], se_fc: List[int],
                 realsense_obj: Optional[t265_class] = None,
                 serial_fc_ref=None, radar_obj=None, pole_vision_obj=None):
```

找到 `self.targets = self.load_waypoints()`（原第168行），替换为：

```python
        self.targets = self.load_waypoints('patrol_router.txt')
```

找到环绕状态机字段初始化块（原194-203行，`# 环绕状态机(阶段1单杆/阶段2双杆共用)` 开始到 `_detour_checked_index` 结束），整体替换为：

```python
        # 环绕状态机(阶段1单杆/阶段2双杆共用)
        self.nav_mode = "PATROL"  # PATROL / APPROACHING / CIRCLING / RETREAT / TO_LANDING
        self.circled_poles = []   # 已完成环绕的杆塔 [(x,y,color), ...]，坐标供绕行检查用
        self.circled_colors = set()  # 已环绕颜色集合，去重判断依据(替代坐标容差)，见2026-07-14设计文档
        self._circle_pole_center = None  # 当前正在环绕的杆塔世界坐标(冻结快照)，Task 5会
                                          # 整体改名成_approach_pole_center(APPROACHING阶段
                                          # 也要用同一个字段)，这里先保持阶段1原名，避免和
                                          # 还没改名的_exclude_active_circle_target_only等
                                          # 方法产生中间断裂状态(AttributeError)
        self._approach_color = None        # 当前APPROACHING/CIRCLING锁定颜色
        self._approach_lost_since = None   # APPROACHING视觉结果开始陈旧的时间点(见POLE_VISION_STALE_S)
        self._trigger_candidate = None       # PATROL态正在累计确认时长的(x,y,color)候选目标
        self._trigger_candidate_since = None
        self._patrol_saved_targets = None
        self._patrol_saved_index = 0
        self._cruise_z = self.targets[0][2] if self.targets else put_height / 100
        self.pole_total = TOTAL_POLES
        self._detour_checked_index = -1  # 上次检查过绕行需求的target_index，避免同一个
                                          # 目标每tick重复计算/重复插入绕行航点

        # 视觉子系统(可选)
        self.pole_vision = pole_vision_obj
        self.approach_y_pid = PID(0, 0, p=APPROACH_Y_KP, i=APPROACH_Y_KI, d=APPROACH_Y_KD)
```

- [ ] **Step 3: 常量区新增视觉/接近相关常量**

找到常量区末尾（原100-102行，`LANDING_POINT`/`POLE_DETOUR_SAFETY_RADIUS_M`/`POLE_DETOUR_MARGIN_M`）。**先不要删除`LANDING_POINT`这一行**——`_on_circle_complete`到Task 10才会改成不再用它，`test_circle_state_machine.py`顶部现在还在`from Mission_GPT import ... LANDING_POINT`，这里如果提前删掉常量会让整个测试文件从本Task开始collection error（导入失败，不是某几个用例失败），后面几个Task"确认其余用例仍PASS"的验证步骤会因此没法做。`LANDING_POINT`常量和它的最后一个引用点（`_on_circle_complete`）、以及测试文件里的import，三处放到Task 10原子性地一起删除。在`POLE_DETOUR_MARGIN_M`那一行后追加：

```python
POLE_COLOR_DIRECTION = {"red": "cw", "green": "ccw"}  # 赛题固定映射，见2026-07-14设计文档

# ---------- 阶段2：前置摄像头颜色识别 + 视觉伺服接近 ----------
POLE_VISION_STALE_S = 0.5     # 视觉检测结果超过此值未更新，APPROACHING视为目标丢失
POLE_TRIGGER_CONFIRM_S = 0.3  # PATROL→APPROACHING触发条件(雷达确认+视觉确认+颜色未去重)
                                # 需要持续满足这么久才真正触发，同一个计时器也兼做颜色
                                # 锁定防抖，避免单帧误判/边界抖动被当真，见2026-07-14设计
                                # 文档"触发滞回""颜色锁定"两节
POLE_VISION_Y_SIGN = 1        # 未标定！真机标定前只是假设值，标定结果可能是+1也可能是-1，
                                # 标定前APPROACHING阶段y方向视觉伺服可能是反的(同POLE_YAW_SIGN)
APPROACH_Y_KP = 30.0           # 视觉伺服y轴PID增益，误差量是方位角(弧度，量级约±0.7rad)，
                                # 不是x_pid/y_pid那种米制误差，不能沿用0.82那组增益。
                                # 初始值未经真机标定，见2026-07-14设计文档测试计划步骤1
APPROACH_Y_KI = 0.0
APPROACH_Y_KD = 2.0
APPROACH_Y_VEL_MAX = 35        # 视觉伺服y轴输出限幅(cm/s量级，跟x_pid/y_pid的40上限同量级)
APPROACH_X_SPEED_FAR = 25      # 雷达距离 > APPROACH_X_SWITCH_DIST_M 时的接近速度
APPROACH_X_SPEED_NEAR = 12     # 雷达距离 <= APPROACH_X_SWITCH_DIST_M 时降速微调
APPROACH_X_SWITCH_DIST_M = 0.8
APPROACH_CIRCLE_TRIGGER_DIST_M = POLE_CIRCLE_RADIUS_M  # APPROACHING→CIRCLING触发距离，
                                # 直接复用环绕半径(2026-07-14讨论决定，不额外加余量常量)
CAMERA_DEVICE = os.getenv("DRONE_CAMERA_DEVICE", "/dev/video0")
```

- [ ] **Step 4: 顶部导入 `pole_vision`**

找到 `from Lcode.circle_planner import generate_circle_waypoints, compute_detour_waypoint`（原第20行），下面新增一行：

```python
from Lcode.pole_vision import azimuth_from_dx
```

- [ ] **Step 5: 运行现有测试套件确认没有回归（本Task只是新增字段/参数/常量，纯加法，不改变任何现有行为）**

Run: `cd "D:/项目与工具/Python项目/Project2/Project2/drone_control/circle_pole" && python -m pytest -v`
Expected: 全部PASS，无收集错误——`_circle_pole_center`字段名本Task特意保持不变（见`__init__`那段代码的注释），`LANDING_POINT`常量也还在，`circled_poles`格式也还没改，所以不应该有任何测试因为本Task的改动而失败

- [ ] **Step 6: Commit**

```bash
cd "D:/项目与工具/Python项目/Project2/Project2"
git add drone_control/circle_pole/Mission_GPT.py
git commit -m "feat(circle_pole): load_waypoints参数化，接入视觉子系统常量与初始化字段"
```

---

### Task 5: CIRCLING 阶段悬停避让整体关闭

**Files:**
- Modify: `drone_control/circle_pole/Mission_GPT.py`
- Modify: `drone_control/circle_pole/test_mission_pole_integration.py`
- Modify: `drone_control/circle_pole/test_circle_state_machine.py`

- [ ] **Step 1: 先改测试，把"CIRCLING态下另一根杆触发悬停"相关的3个用例改成用PATROL态验证同样的悬停/滞回行为（CIRCLING已经不再检查其他杆子，这几个场景的安全网移到了PATROL/APPROACHING阶段）**

在 `test_mission_pole_integration.py` 里，把 `test_navigate_hovers_for_different_uncircled_pole_while_circling`、`test_navigate_keeps_hovering_within_resume_hysteresis_band`、`test_navigate_resumes_once_pole_dist_exceeds_resume_threshold` 三个测试方法体里的：

```python
        m._circle_pole_center = (5.0, 5.0)  # 正在环绕的目标，离这次的杆子很远
        m.nav_mode = "CIRCLING"
        m.targets = [[5.0, 4.3, 1.0]]
```

统一替换为：

```python
        m._approach_pole_center = (5.0, 5.0)  # 正在接近的目标，离这次的杆子很远
        m.nav_mode = "TO_LANDING"
        m.targets = [[5.0, 4.3, 1.0]]
```

**不要改用`nav_mode = "PATROL"`**（这是2026-07-14计划自审时发现的一处真实bug，记录下来避免下次犯同样的错）：`navigate()`里`PATROL`态现在还在用未经Task 7改造的`_find_new_pole`+`_start_circling`直接触发逻辑——只要雷达确认到一个未环绕过的杆塔，不管`_approach_pole_center`是什么，会立即触发环绕并`return`，根本不会执行到悬停避让判断那段代码。所以测试摆一个"离飞机0.3m的确认杆塔"时，PATROL态不会走到悬停判断，而是直接被旧逻辑吞掉触发环绕，`_pole_hovering`永远是False——不是因为排除逻辑生效，是悬停判断代码根本没执行到。`TO_LANDING`态没有这个问题(`navigate()`里`_find_new_pole`触发只挂在`nav_mode == "PATROL"`分支下)，本Task之后也不会有任何后续Task给`TO_LANDING`加类似的"提前return"分支(Task 8只给`APPROACHING`加，不影响`TO_LANDING`)，所以选它作为验证悬停避让通用逻辑的稳定测试场景。`m.targets`这行**保留不删**（跟原CIRCLING版本一致，`TO_LANDING`态不会自动加载`patrol_router.txt`，需要手动给一个非空列表避免`target_index>=len(targets)`分支意外触发)。

同时把这三个方法的说明注释和docstring里"CIRCLING态"字样改成"非CIRCLING阶段"，例如 `test_navigate_hovers_for_different_uncircled_pole_while_circling` 改名为 `test_navigate_hovers_for_different_uncircled_pole_outside_circling`，docstring改为：

```python
    def test_navigate_hovers_for_different_uncircled_pole_outside_circling(self):
        """悬停避让唯一还有效的场景：非CIRCLING阶段(PATROL/APPROACHING/TO_LANDING/
        RETREAT)下遇到一根不是当前接近目标、也没环绕过的杆塔(阶段2双杆场景的安全
        网)。CIRCLING阶段整体关闭悬停避让(2026-07-14设计文档"CIRCLING阶段悬停
        避让整体关闭"一节)，不再是这里验证的场景。这里用TO_LANDING态验证(而不是
        PATROL)，因为PATROL态目前还有未经Task 7改造的_find_new_pole+_start_circling
        直接触发逻辑，会在悬停判断代码执行前就把确认到的杆塔吞掉触发环绕，见Task 5
        Step 1对应说明。"""
```

其余两个测试同理改名去掉"circling"字样、docstring改成"非CIRCLING阶段"语境（内容不用改，只是场景从"CIRCLING态排除自己环绕目标"变成"TO_LANDING态排除自己正在接近的目标"，验证的仍然是同一套`_exclude_active_circle_target_only`/滞回逻辑）。

在 `test_circle_state_machine.py` 里，`TestCirclingHoverExclusion.test_other_confirmed_pole_still_triggers_hover_during_circling` 整个删除（CIRCLING已经不再检查任何杆子，这个场景在Task 5 Step 1里已经用PATROL态覆盖了），改成一条新断言：

```python
class TestCirclingHoverExclusion:
    def test_own_circling_target_does_not_trigger_hover(self):
        m = _make_mission(radar_obj=object())
        m._approach_pole_center = (0.5, 0.3)
        m.nav_mode = "CIRCLING"
        m.targets = [[0.5, -0.2, 1.2], [1.0, 0.3, 1.2]]
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])  # 就是环绕目标本身

        m.set_speed = lambda *a, **k: None
        m.navigate([0.45, 0.28, 1.2], 0.0)

        assert m._pole_hovering is False

    def test_circling_ignores_any_other_pole_regardless_of_distance(self):
        """2026-07-14决定：CIRCLING阶段悬停避让整体关闭，不只排除当前目标。哪怕
        另一根完全不相关的杆子就在飞机旁边(0.1m，远小于悬停阈值)，环绕过程中也
        不应该被打断——赛题最小间距150cm+环绕半径0.7m下，理论安全边际只有5cm，
        被已知定位噪声完全吞掉，环绕中途悬停打断本身比不检测风险更高。"""
        m = _make_mission(radar_obj=object())
        m._approach_pole_center = (5.0, 5.0)  # 正在环绕的目标，离这次的杆子很远
        m.nav_mode = "CIRCLING"
        m.targets = [[5.0, 4.3, 1.2], [5.0, 5.7, 1.2]]
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.1, 0.0)])  # 另一根杆子，离飞机只有0.1m

        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert m._pole_hovering is False
```

（`_circle_pole_center` 全部改名成 `_approach_pole_center`，跟Task 6会做的字段改名保持一致；这一步先改测试，实现代码在Step 2里跟着改）

**同时顺手把测试文件里其余两处遗留的`_circle_pole_center`引用也机械改名**（Step 3的sed命令只处理`Mission_GPT.py`，不会碰测试文件，这两处不手动改会在Step 4持续报`AttributeError`）：`test_circle_state_machine.py`里`TestPatrolTriggersCircling.test_confirmed_new_pole_switches_to_circling`（`assert m._circle_pole_center == pytest.approx((0.5, 0.3))`）和`TestCirclingHoverExclusion.test_own_circling_target_still_excluded_despite_yaw_drift_offset`（`m._circle_pole_center = (0.5, 0.3)`），两处都只是字段名替换，不改断言逻辑/测试意图。

- [ ] **Step 2: 运行测试确认失败（字段还叫`_circle_pole_center`，且CIRCLING悬停还没关）**

Run: `cd "D:/项目与工具/Python项目/Project2/Project2/drone_control/circle_pole" && python -m pytest test_mission_pole_integration.py test_circle_state_machine.py -v`
Expected: 多个FAIL（`AttributeError: no attribute '_approach_pole_center'`，以及`test_circling_ignores_any_other_pole_regardless_of_distance`因为悬停还没关而失败）。**注意**：还有两个Step 1没有提到、但同样会FAIL的用例——`test_circle_state_machine.py`里`TestPatrolTriggersCircling.test_confirmed_new_pole_switches_to_circling`（断言里读`m._circle_pole_center`）和`TestCirclingHoverExclusion.test_own_circling_target_still_excluded_despite_yaw_drift_offset`（同样设置`m._circle_pole_center = ...`），这两个测试Step 1没要求重写，但引用了即将被sed批量改名的字段，也会失败。这不是需要额外解决的新问题——纯属字段名不同步导致的`AttributeError`，跟其他因为改名而失败的用例是同一类原因，Step 3实现的时候顺手把这两处的`_circle_pole_center`也机械改成`_approach_pole_center`（纯改名，不改断言逻辑）即可一起修好，不用单独处理。

- [ ] **Step 3: 实现——字段改名 + CIRCLING悬停避让整体关闭**

找到悬停避让判断块（原第463-474行，从 `# 悬停避让距离判断` 注释开始到 `pole_hover = pole_dist is not None and pole_dist < POLE_DANGER_DIST_M` 结束），整体替换为：

```python
            # 悬停避让距离判断：CIRCLING阶段整体关闭(2026-07-14决定，不只排除当前
            # 目标)。赛题最小杆塔间距150cm、环绕半径0.7m下，环绕到最靠近另一根杆
            # 的点最坏情况只有80cm，仅比0.75m悬停阈值高5cm，这个理论安全边际被
            # 已知定位噪声(confirmed_poles()漂移0.4~1m量级)完全吞掉，环绕过程中
            # 途悬停打断本身(近距离转弯突然停住)比不检测风险更高。PATROL/
            # APPROACHING/TO_LANDING/RETREAT阶段不受影响，仍用现有悬停避让+
            # _exclude_circle_target排除机制。
            if self.nav_mode != "CIRCLING":
                hover_check_poles = self._exclude_circle_target(confirmed)
                pole_dist = nearest_confirmed_pole_dist(hover_check_poles, pos[0], pos[1])
                if self._pole_hovering:
                    pole_hover = pole_dist is not None and pole_dist < POLE_RESUME_DIST_M
                else:
                    pole_hover = pole_dist is not None and pole_dist < POLE_DANGER_DIST_M
```

把整个类里所有 `self._circle_pole_center` 引用（含`__init__`里的初始化、`_exclude_active_circle_target_only`方法体等全部出现的地方）统一改名为 `self._approach_pole_center`——APPROACHING阶段也要用同一个字段记录当前目标，不再是CIRCLING专属（`grep -n "_circle_pole_center" Mission_GPT.py` 确认改完后没有残留）：

```bash
cd "D:/项目与工具/Python项目/Project2/Project2/drone_control/circle_pole"
sed -i 's/_circle_pole_center/_approach_pole_center/g' Mission_GPT.py
grep -n "_circle_pole_center" Mission_GPT.py  # 应该没有输出
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd "D:/项目与工具/Python项目/Project2/Project2/drone_control/circle_pole" && python -m pytest test_mission_pole_integration.py test_circle_state_machine.py -v`
Expected: 除了`TestCircleCompletion`类里的`test_single_pole_mission_switches_to_to_landing_when_circle_done`/`test_multi_pole_mission_resumes_patrol_when_more_poles_remain`两个用例FAIL（它们还在手动设置已经改名的旧字段`m._circle_pole_center = ...`，现在这只是个不生效的孤立属性，真正的`self._approach_pole_center`是`None`，`_on_circle_complete`会解包None报错——这两个用例整个逻辑到Task 10会被重写掉，不用现在修），其余全部PASS，无收集错误。重点确认`TestCirclingHoverExclusion`全部、`TestMissionPoleHover`里改名的3个用例都PASS

- [ ] **Step 5: Commit**

```bash
cd "D:/项目与工具/Python项目/Project2/Project2"
git add drone_control/circle_pole/Mission_GPT.py drone_control/circle_pole/test_mission_pole_integration.py drone_control/circle_pole/test_circle_state_machine.py
git commit -m "fix(circle_pole): CIRCLING阶段悬停避让整体关闭，_circle_pole_center改名_approach_pole_center"
```

---

### Task 6: `circled_poles` 改三元组 + 颜色去重替代坐标容差

**Files:**
- Modify: `drone_control/circle_pole/Mission_GPT.py`
- Modify: `drone_control/circle_pole/test_circle_state_machine.py`
- Modify: `drone_control/circle_pole/test_mission_pole_integration.py`

- [ ] **Step 1: 先改测试——`circled_poles`断言改三元组，新增颜色去重测试**

在 `test_mission_pole_integration.py`，`test_navigate_does_not_hover_when_already_circled_pole_confirmed_nearby` 和 `test_navigate_does_not_hover_when_already_circled_pole_position_has_drifted` 两处：

```python
        m.circled_poles = [(0.3, 0.0)]  # 已环绕过
```

改为：

```python
        m.circled_poles = [(0.3, 0.0, "red")]  # 已环绕过
```

同理 `m.circled_poles = [(-0.91, -0.28)]` 改为 `m.circled_poles = [(-0.91, -0.28, "red")]`。

在 `test_circle_state_machine.py`：
- `test_inserts_detour_when_already_circled_pole_blocks_direct_path` 和 `test_does_not_recheck_same_target_index_repeatedly` 里 `m.circled_poles = [(1.0, 0.0)]` 改为 `m.circled_poles = [(1.0, 0.0, "red")]`
- `test_single_pole_mission_switches_to_to_landing_when_circle_done` 和 `test_multi_pole_mission_resumes_patrol_when_more_poles_remain` 属于CIRCLING完成流程，会在Task 10整体重写（现在会因为改用`_approach_color`/RETREAT而全部失效），本Task先不动它们

`TestPatrolTriggersCircling` 类整体删除（PATROL现在触发`APPROACHING`不是直接`CIRCLING`，测试内容移到Task 7的`test_approaching_state.py`），只保留一个颜色去重的新测试类：

```python
class TestColorDedup:
    def test_already_circled_color_is_not_in_circled_colors_initially(self):
        m = _make_mission(radar_obj=object())
        assert m.circled_colors == set()

    def test_color_already_circled_true_after_recorded(self):
        m = _make_mission(radar_obj=object())
        m.circled_colors.add("red")
        assert m._color_already_circled("red") is True
        assert m._color_already_circled("green") is False
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd "D:/项目与工具/Python项目/Project2/Project2/drone_control/circle_pole" && python -m pytest test_circle_state_machine.py test_mission_pole_integration.py -v`
Expected: 新增的`TestColorDedup`两个用例FAIL(`AttributeError: no attribute '_color_already_circled'`)；其余因三元组已在Step1同步改好，应该继续PASS（`TestPatrolTriggersCircling`已删除，不再产生FAIL）

- [ ] **Step 3: 修`_already_circled`的3元组解包 + 实现`_color_already_circled`**

`circled_poles`现在存的是`(x, y, color)`三元组（Step 1已经把测试改成这样seed），但`_already_circled`还是按二元组解包(`for cx, cy in self.circled_poles`)，不修会在任何调用它的测试里抛`ValueError: too many values to unpack`。找到 `_already_circled` 方法（原第685-694行）：

```python
    def _already_circled(self, x, y):
        # ...(原有注释不变)...
        tolerance = POLE_CIRCLE_RADIUS_M + POLE_CIRCLE_EXCLUDE_MARGIN_M
        return any(math.hypot(x - cx, y - cy) <= tolerance
                   for cx, cy in self.circled_poles)
```

把最后一行的解包改成三元组（颜色用`_`丢弃，这个方法只关心坐标）：

```python
        return any(math.hypot(x - cx, y - cy) <= tolerance
                   for cx, cy, _color in self.circled_poles)
```

在 `_already_circled` 方法后面，新增方法：

```python
    def _color_already_circled(self, color):
        """颜色去重判断——替代坐标容差成为"是否已环绕过"的依据(2026-07-14设计
        文档"去重机制"一节)，只用于PATROL→APPROACHING触发；`_already_circled`
        (坐标容差)继续保留给悬停避让/绕行排除使用，两者服务不同目的，见设计文档。
        """
        return color in self.circled_colors
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd "D:/项目与工具/Python项目/Project2/Project2/drone_control/circle_pole" && python -m pytest test_circle_state_machine.py test_mission_pole_integration.py -v`
Expected: `TestColorDedup`两个用例PASS；`test_single_pole_mission_switches_to_to_landing_when_circle_done`/`test_multi_pole_mission_resumes_patrol_when_more_poles_remain`继续FAIL（Task 5引入的已知问题，见Task 5 Step 4说明，留给Task 10重写）；`test_to_landing_arrival_transitions_to_land_state`不依赖`circled_poles`/`_approach_pole_center`，应该继续PASS；其余全部PASS

- [ ] **Step 5: Commit**

```bash
cd "D:/项目与工具/Python项目/Project2/Project2"
git add drone_control/circle_pole/Mission_GPT.py drone_control/circle_pole/test_circle_state_machine.py drone_control/circle_pole/test_mission_pole_integration.py
git commit -m "feat(circle_pole): circled_poles改三元组含颜色，新增_color_already_circled去重判断"
```

---

### Task 7: PATROL→APPROACHING 触发滞回逻辑（TDD）

**Files:**
- Modify: `drone_control/circle_pole/Mission_GPT.py`
- Create: `drone_control/circle_pole/test_approaching_state.py`

**关键设计回顾**（见2026-07-14设计文档）：雷达确认新杆塔 + 视觉当前看到未去重颜色的目标，两者同时满足且**持续`POLE_TRIGGER_CONFIRM_S`(0.3s)** 才真正触发，避免边界抖动/单帧误判。

- [ ] **Step 1: 写失败测试**

```python
"""PATROL→APPROACHING触发滞回逻辑 + APPROACHING状态控制律单元测试。

运行:
    cd drone_control/circle_pole && python -m pytest test_approaching_state.py -v
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from Mission_GPT import mission, POLE_TRIGGER_CONFIRM_S


def _make_mission(radar_obj=None, pole_vision_obj=None):
    re_fc = [0] * 14
    se_fc = [0] * 11
    return mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None,
                   radar_obj=radar_obj, pole_vision_obj=pole_vision_obj)


class _FakeVision:
    def __init__(self, dx_px=0.0, color="red", fresh=True):
        self._dx_px = dx_px
        self._color = color
        self._fresh = fresh
        self.locked_color = None

    def latest(self):
        t = time.time() if self._fresh else 0.0
        return {"dx_px": self._dx_px, "color": self._color, "t": t}

    def set_locked_color(self, color):
        self.locked_color = color


class TestPatrolTriggerRequiresBothRadarAndVision(object):
    def test_radar_only_does_not_trigger(self):
        m = _make_mission(radar_obj=object(), pole_vision_obj=None)
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m.nav_mode == "PATROL"

    def test_radar_and_vision_together_do_not_trigger_before_confirm_window(self):
        """条件刚满足的第一帧不应该立刻触发，需要持续POLE_TRIGGER_CONFIRM_S。"""
        m = _make_mission(radar_obj=object(), pole_vision_obj=_FakeVision(color="red"))
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m.nav_mode == "PATROL"
        assert m._trigger_candidate == (0.5, 0.3, "red")

    def test_radar_and_vision_together_trigger_approaching_after_confirm_window(self):
        m = _make_mission(radar_obj=object(), pole_vision_obj=_FakeVision(color="red"))
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)  # 第一帧：只是记下候选，不触发
        assert m.nav_mode == "PATROL"

        m._trigger_candidate_since = time.time() - POLE_TRIGGER_CONFIRM_S - 0.01
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])
        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert m.nav_mode == "APPROACHING"
        assert m._approach_pole_center == pytest.approx((0.5, 0.3))
        assert m._approach_color == "red"

    def test_already_circled_color_does_not_trigger(self):
        m = _make_mission(radar_obj=object(), pole_vision_obj=_FakeVision(color="red"))
        m.circled_colors.add("red")
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m.nav_mode == "PATROL"
        assert m._trigger_candidate is None

    def test_stale_vision_does_not_trigger(self):
        m = _make_mission(radar_obj=object(), pole_vision_obj=_FakeVision(color="red", fresh=False))
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m.nav_mode == "PATROL"

    def test_color_change_resets_confirm_window(self):
        """候选颜色中途变化(比如视觉误判抖动)要重新计时，不能沿用旧计时器——
        这也是颜色锁定防抖机制的一部分，见2026-07-14设计文档"颜色锁定"一节。"""
        vision = _FakeVision(color="red")
        m = _make_mission(radar_obj=object(), pole_vision_obj=vision)
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)
        first_since = m._trigger_candidate_since

        vision._color = "green"
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])
        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert m._trigger_candidate == (0.5, 0.3, "green")
        assert m._trigger_candidate_since > first_since
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd "D:/项目与工具/Python项目/Project2/Project2/drone_control/circle_pole" && python -m pytest test_approaching_state.py -v`
Expected: FAIL（PATROL态还是走旧的`_find_new_pole`直接`_start_circling`逻辑）

- [ ] **Step 3: 实现——替换PATROL触发逻辑**

找到 `navigate()` 里PATROL触发的旧代码块（原第447-452行）：

```python
            # PATROL态：发现一个未环绕过的确认杆塔，立即切到CIRCLING
            if self.nav_mode == "PATROL":
                new_pole = self._find_new_pole(confirmed)
                if new_pole is not None:
                    self._start_circling(new_pole, pos)
                    return
```

替换为：

```python
            # PATROL态：雷达+视觉+颜色未去重三条件持续满足POLE_TRIGGER_CONFIRM_S
            # 才触发APPROACHING(2026-07-14设计文档"触发滞回"一节)
            if self.nav_mode == "PATROL":
                if self._update_trigger_candidate(confirmed):
                    return
```

在 `_find_new_pole` 方法（原第696-699行）后面新增两个方法：

```python
    def _update_trigger_candidate(self, confirmed):
        """返回True表示本次调用触发了APPROACHING(调用方应该直接return，跳过
        本tick剩余的悬停避让/日志逻辑，语义上跟原来_start_circling后直接return
        一致)。"""
        new_pole = self._find_new_pole(confirmed)
        vision = self.pole_vision.latest() if self.pole_vision is not None else None
        vision_color = vision["color"] if vision else None
        vision_fresh = (vision is not None and vision["t"] > 0
                         and time.time() - vision["t"] < POLE_VISION_STALE_S)

        candidate = None
        if (new_pole is not None and vision_fresh and vision_color is not None
                and not self._color_already_circled(vision_color)):
            candidate = (new_pole["x"], new_pole["y"], vision_color)

        if candidate is None:
            self._trigger_candidate = None
            self._trigger_candidate_since = None
            return False

        if self._trigger_candidate is None or self._trigger_candidate[2] != candidate[2]:
            self._trigger_candidate = candidate
            self._trigger_candidate_since = time.time()
            return False

        if time.time() - self._trigger_candidate_since >= POLE_TRIGGER_CONFIRM_S:
            self._start_approaching(*candidate)
            self._trigger_candidate = None
            self._trigger_candidate_since = None
            return True
        return False

    def _start_approaching(self, x, y, color):
        self._patrol_saved_targets = self.targets
        self._patrol_saved_index = self.target_index
        self._approach_pole_center = (x, y)
        self._approach_color = color
        self._approach_lost_since = None
        if self.pole_vision is not None:
            self.pole_vision.set_locked_color(color)
        self.nav_mode = "APPROACHING"
        logger.warning(f"雷达+视觉确认{color}杆塔({x:.2f},{y:.2f})，开始视觉接近")
```

删除旧的 `_start_circling` 方法（原第745-760行，`def _start_circling(self, pole, pos):` 整个方法体）——被 `_start_approaching` + Task 8 的 `_start_circling_from_approach` 取代，`POLE_CIRCLE_DIRECTION` 常量也一并删除（不再被引用，改用`POLE_COLOR_DIRECTION`映射，已在Task 4加入）。

- [ ] **Step 4: 运行测试确认通过**

Run: `cd "D:/项目与工具/Python项目/Project2/Project2/drone_control/circle_pole" && python -m pytest test_approaching_state.py -v`
Expected: 全部PASS

- [ ] **Step 5: 跑整个套件确认没有引入新回归**

Run: `cd "D:/项目与工具/Python项目/Project2/Project2/drone_control/circle_pole" && python -m pytest -v`
Expected: 除了`test_single_pole_mission_switches_to_to_landing_when_circle_done`/`test_multi_pole_mission_resumes_patrol_when_more_poles_remain`两个已知问题（Task 5引入，留给Task 10重写），其余全部PASS，没有新增的意外失败

- [ ] **Step 6: Commit**

```bash
cd "D:/项目与工具/Python项目/Project2/Project2"
git add drone_control/circle_pole/Mission_GPT.py drone_control/circle_pole/test_approaching_state.py
git commit -m "feat(circle_pole): PATROL到APPROACHING触发滞回逻辑(雷达+视觉+颜色去重+持续确认)"
```

---

### Task 8: `APPROACHING` 状态控制律（TDD）

**Files:**
- Modify: `drone_control/circle_pole/Mission_GPT.py`
- Modify: `drone_control/circle_pole/test_approaching_state.py`

**控制律回顾**（见设计文档）：x方向按雷达距离两档限速接近目标（`APPROACH_X_SWITCH_DIST_M`分界），y方向完全由视觉伺服PID居中；视觉结果陈旧超过`POLE_VISION_STALE_S`视为目标丢失，退回PATROL恢复`target_index`；雷达距离降到`APPROACH_CIRCLE_TRIGGER_DIST_M`触发CIRCLING（本Task先只做到"判定该转CIRCLING"，实际生成环绕航点在Task 9）。

- [ ] **Step 1: 追加失败测试到 `test_approaching_state.py`**

```python
class TestApproachingControlLaw:
    def _start(self, pos, pole=(2.0, 0.0), color="red", dx_px=0.0):
        vision = _FakeVision(dx_px=dx_px, color=color)
        m = _make_mission(radar_obj=object(), pole_vision_obj=vision)
        m._approach_pole_center = pole
        m._approach_color = color
        m.nav_mode = "APPROACHING"
        m.set_speed = lambda *a, **k: None
        m.navigate(pos, 0.0)
        return m

    def test_far_distance_uses_fast_speed(self):
        calls = []
        m = self._start(pos=[0.0, 0.0, 1.2], pole=(2.0, 0.0))
        m.set_speed = lambda x, y, yaw, z: calls.append(x)
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert calls[-1] == pytest.approx(APPROACH_X_SPEED_FAR)

    def test_near_distance_uses_slow_speed(self):
        """dist要选在(APPROACH_CIRCLE_TRIGGER_DIST_M, APPROACH_X_SWITCH_DIST_M]
        区间内(当前值下是(0.7, 0.8]m)，才是"降速但还没触发环绕"的近距离区——
        选0.5m这种比环绕触发距离(0.7m)还近的值会在到达这里之前就已经转CIRCLING，
        2026-07-14实现时发现的真实test-data bug，不是巧合。"""
        calls = []
        m = self._start(pos=[1.25, 0.0, 1.2], pole=(2.0, 0.0))  # dist=0.75m
        m.set_speed = lambda x, y, yaw, z: calls.append(x)
        m.navigate([1.25, 0.0, 1.2], 0.0)
        assert calls[-1] == pytest.approx(APPROACH_X_SPEED_NEAR)

    def test_approach_speed_direction_toward_pole(self):
        """杆塔在飞机后方(x更小)时，接近速度应该是负的(往回飞)。"""
        calls = []
        m = self._start(pos=[3.0, 0.0, 1.2], pole=(2.0, 0.0))
        m.set_speed = lambda x, y, yaw, z: calls.append(x)
        m.navigate([3.0, 0.0, 1.2], 0.0)
        assert calls[-1] < 0

    def test_vision_dx_drives_y_speed_via_pid(self):
        calls = []
        m = self._start(pos=[0.0, 0.0, 1.2], pole=(2.0, 0.0), dx_px=500.0)
        m.set_speed = lambda x, y, yaw, z: calls.append(y)
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert calls[-1] != 0

    def test_target_lost_beyond_timeout_retreats_to_patrol(self):
        stale_vision = _FakeVision(color="red", fresh=False)
        m = _make_mission(radar_obj=object(), pole_vision_obj=stale_vision)
        m._patrol_saved_targets = [[0.0, 0.0, 1.2], [1.0, 0.0, 1.2]]
        m._patrol_saved_index = 1
        m._approach_pole_center = (2.0, 0.0)
        m._approach_color = "red"
        m.nav_mode = "APPROACHING"
        m.set_speed = lambda *a, **k: None

        m.navigate([0.0, 0.0, 1.2], 0.0)  # 第一次陈旧：只是开始计时，不立刻退回
        assert m.nav_mode == "APPROACHING"

        m._approach_lost_since = time.time() - POLE_VISION_STALE_S - 0.01
        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert m.nav_mode == "PATROL"
        assert m.targets == [[0.0, 0.0, 1.2], [1.0, 0.0, 1.2]]
        assert m.target_index == 1
        assert m._approach_pole_center is None
        assert stale_vision.locked_color is None

    def test_reaching_trigger_distance_switches_to_circling(self):
        """dist选0.6m(明显小于0.7m触发阈值)而不是卡在边界值0.7m——2.7-2.0在浮点
        下算出来是0.7000000000000002，`<=0.7`会因为浮点误差判为False，边界相等
        测试不该用浮点减法凑出来的"看起来相等"，2026-07-14实现时发现的真实
        test-data bug。这里只需要验证"进入触发距离内会转CIRCLING"，不需要卡边界。"""
        m = self._start(pos=[2.1, 0.0, 1.2], pole=(2.7, 0.0))  # dist=0.6m，明显在触发阈值内
        m.set_speed = lambda *a, **k: None
        m.navigate([2.1, 0.0, 1.2], 0.0)
        assert m.nav_mode == "CIRCLING"
```

在文件顶部import行补充：

```python
from Mission_GPT import (
    mission, POLE_TRIGGER_CONFIRM_S, POLE_VISION_STALE_S,
    APPROACH_X_SPEED_FAR, APPROACH_X_SPEED_NEAR,
)
```

（替换掉原来只导入`mission, POLE_TRIGGER_CONFIRM_S`的那一行）

- [ ] **Step 2: 运行测试确认失败**

Run: `cd "D:/项目与工具/Python项目/Project2/Project2/drone_control/circle_pole" && python -m pytest test_approaching_state.py -v`
Expected: `TestApproachingControlLaw`全部FAIL（`nav_mode == "APPROACHING"`时`navigate()`还是走原来的航点逻辑，`self.targets`是空的会走"全部航点完成"分支）

- [ ] **Step 3: 实现 `_approaching_step`，接入`navigate()`入口**

在 `navigate()` 方法最开头（`def navigate(self, pos, yaw):` 之后，`# 全部航点耗尽...`注释之前）插入分支：

```python
    def navigate(self, pos, yaw):
        if self.nav_mode == "APPROACHING":
            self._approaching_step(pos, yaw)
            return

        # 全部航点耗尽：按当前nav_mode分支处理(环绕完成/到达降落点/巡航耗尽兜底)
```

在 `_start_approaching` 方法后面新增：

```python
    def _approaching_step(self, pos, yaw):
        cx, cy = self._approach_pole_center
        dist = math.hypot(pos[0] - cx, pos[1] - cy)

        vision = self.pole_vision.latest() if self.pole_vision is not None else None
        vision_fresh = (vision is not None and vision["t"] > 0
                         and time.time() - vision["t"] < POLE_VISION_STALE_S)

        if not vision_fresh:
            if self._approach_lost_since is None:
                self._approach_lost_since = time.time()
            elif time.time() - self._approach_lost_since >= POLE_VISION_STALE_S:
                logger.warning("APPROACHING视觉目标丢失，退回PATROL")
                self._abort_approaching()
                return
        else:
            self._approach_lost_since = None

        if dist <= APPROACH_CIRCLE_TRIGGER_DIST_M:
            self._start_circling_from_approach(pos)
            return

        x_speed = APPROACH_X_SPEED_FAR if dist > APPROACH_X_SWITCH_DIST_M else APPROACH_X_SPEED_NEAR
        vx = int(x_speed if cx >= pos[0] else -x_speed)

        vy = 0
        if vision_fresh and vision["dx_px"] is not None:
            azimuth = azimuth_from_dx(vision["dx_px"])
            self.approach_y_pid.set_target(0.0)
            vy_raw = self.approach_y_pid.get_pid(azimuth * POLE_VISION_Y_SIGN)
            vy = int(self.limit(vy_raw, APPROACH_Y_VEL_MAX))

        self._step_ramp_z(int(self._cruise_z * 100))
        self.set_speed(vx, vy, 0, int(self._ramp_z_cm))

    def _abort_approaching(self):
        if self.pole_vision is not None:
            self.pole_vision.set_locked_color(None)
        self.targets = self._patrol_saved_targets
        self.target_index = self._patrol_saved_index
        self.last_target_index = -1
        self._approach_pole_center = None
        self._approach_color = None
        self._approach_lost_since = None
        self.nav_mode = "PATROL"
```

（`_start_circling_from_approach` 这里先加一个最小占位实现，让本Task测试能通过，Task 9会替换成真正生成环绕航点的完整版本）：

```python
    def _start_circling_from_approach(self, pos):
        self.nav_mode = "CIRCLING"
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd "D:/项目与工具/Python项目/Project2/Project2/drone_control/circle_pole" && python -m pytest test_approaching_state.py -v`
Expected: 全部PASS

- [ ] **Step 5: Commit**

```bash
cd "D:/项目与工具/Python项目/Project2/Project2"
git add drone_control/circle_pole/Mission_GPT.py drone_control/circle_pole/test_approaching_state.py
git commit -m "feat(circle_pole): APPROACHING状态控制律(雷达x两档限速+视觉y伺服+目标丢失回退)"
```

---

### Task 9: `_start_circling_from_approach` 颜色→方向映射（TDD）

**Files:**
- Modify: `drone_control/circle_pole/Mission_GPT.py`
- Modify: `drone_control/circle_pole/test_approaching_state.py`

- [ ] **Step 1: 追加失败测试**

```python
from Lcode.circle_planner import generate_circle_waypoints  # noqa: E402  (文件顶部已有sys.path.insert)


class TestStartCirclingFromApproach:
    def test_red_pole_circles_clockwise(self):
        m = _make_mission(radar_obj=object())
        m._approach_pole_center = (2.0, 0.0)
        m._approach_color = "red"
        m._cruise_z = 1.2

        m._start_circling_from_approach([1.3, 0.0, 1.2])

        assert m.nav_mode == "CIRCLING"
        expected = generate_circle_waypoints(2.0, 0.0, 1.3, 0.0, radius=0.7,
                                              n_points=6, direction="cw", z=1.2)
        assert m.targets == expected
        assert m.target_index == 0

    def test_green_pole_circles_counterclockwise(self):
        m = _make_mission(radar_obj=object())
        m._approach_pole_center = (2.0, 0.0)
        m._approach_color = "green"
        m._cruise_z = 1.2

        m._start_circling_from_approach([1.3, 0.0, 1.2])

        expected = generate_circle_waypoints(2.0, 0.0, 1.3, 0.0, radius=0.7,
                                              n_points=6, direction="ccw", z=1.2)
        assert m.targets == expected
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd "D:/项目与工具/Python项目/Project2/Project2/drone_control/circle_pole" && python -m pytest test_approaching_state.py -v`
Expected: FAIL（Task 8的占位实现只设了`nav_mode`，没生成航点）

- [ ] **Step 3: 实现真正的 `_start_circling_from_approach`**

替换Task 8里的占位实现：

```python
    def _start_circling_from_approach(self, pos):
        cx, cy = self._approach_pole_center
        direction = POLE_COLOR_DIRECTION[self._approach_color]
        waypoints = generate_circle_waypoints(
            cx, cy, pos[0], pos[1],
            radius=POLE_CIRCLE_RADIUS_M, n_points=POLE_CIRCLE_N_POINTS,
            direction=direction, z=self._cruise_z,
        )
        self.targets = waypoints
        self.target_index = 0
        self.last_target_index = -1
        self.nav_mode = "CIRCLING"
        logger.warning(
            f"进入环绕：{self._approach_color}杆塔({cx:.2f},{cy:.2f})，"
            f"方向{direction}，{len(waypoints)}个航点"
        )
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd "D:/项目与工具/Project2/Project2/drone_control/circle_pole" && python -m pytest test_approaching_state.py -v`
Expected: 全部PASS

- [ ] **Step 5: Commit**

```bash
cd "D:/项目与工具/Python项目/Project2/Project2"
git add drone_control/circle_pole/Mission_GPT.py drone_control/circle_pole/test_approaching_state.py
git commit -m "feat(circle_pole): APPROACHING到CIRCLING衔接，颜色决定环绕方向(红cw/绿ccw)"
```

---

### Task 10: 环绕完成退回基准线（`RETREAT`）+ 降落路径改用 `landing_router.txt`

**Files:**
- Modify: `drone_control/circle_pole/Mission_GPT.py`
- Modify: `drone_control/circle_pole/test_circle_state_machine.py`

- [ ] **Step 1: 重写 `TestCircleCompletion` 类**

替换 `test_circle_state_machine.py` 里整个 `TestCircleCompletion` 类：

```python
class TestCircleCompletion:
    def test_circle_done_retreats_to_x_zero_baseline_first(self):
        m = _make_mission(radar_obj=object(), pole_total=1)
        m.nav_mode = "CIRCLING"
        m._approach_pole_center = (0.5, 0.3)
        m._approach_color = "red"
        m.targets = [[0.5, 0.8, 1.2]]
        m.target_index = 1  # 已飞完最后一个环绕航点

        m.navigate([0.5, 0.8, 1.2], 0.0)

        assert m.nav_mode == "RETREAT"
        assert m.circled_poles == [(0.5, 0.3, "red")]
        assert m.circled_colors == {"red"}
        assert m.targets == [[0.0, 0.8, m._cruise_z]]
        assert m.target_index == 0
        assert m._approach_pole_center is None

    def test_single_color_mission_switches_to_to_landing_after_retreat(self):
        m = _make_mission(radar_obj=object(), pole_total=1)
        m.nav_mode = "RETREAT"
        m.circled_colors = {"red"}
        m.targets = [[0.0, 0.8, 1.2]]
        m.target_index = 1  # 已到达基准线

        m.navigate([0.0, 0.8, 1.2], 0.0)

        assert m.nav_mode == "TO_LANDING"
        assert m.target_index == 0
        assert len(m.targets) >= 1  # 从landing_router.txt加载

    def test_multi_color_mission_resumes_patrol_after_retreat(self):
        m = _make_mission(radar_obj=object(), pole_total=2)
        patrol_targets = [[0.0, 0.0, 1.2], [1.0, 0.0, 1.2], [2.0, 0.0, 1.2]]
        m._patrol_saved_targets = patrol_targets
        m._patrol_saved_index = 1
        m.nav_mode = "RETREAT"
        m.circled_colors = {"red"}  # 只绕完一种颜色，还差一种
        m.targets = [[0.0, 0.8, 1.2]]
        m.target_index = 1

        m.navigate([0.0, 0.8, 1.2], 0.0)

        assert m.nav_mode == "PATROL"
        assert m.targets == patrol_targets
        assert m.target_index == 1

    def test_to_landing_arrival_transitions_to_land_state(self):
        m = _make_mission(radar_obj=None)
        m.nav_mode = "TO_LANDING"
        m.targets = [[2.0, 0.0, 1.2]]
        m.target_index = 1  # 已到达唯一的降落航点

        m.navigate([2.0, 0.0, 1.2], 0.0)

        assert m.state == "LAND"
```

同时删除文件顶部 `from Mission_GPT import mission, POLE_CIRCLE_N_POINTS, LANDING_POINT` 里的 `LANDING_POINT`（已不存在），改为：

```python
from Mission_GPT import mission, POLE_CIRCLE_N_POINTS
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd "D:/项目与工具/Python项目/Project2/Project2/drone_control/circle_pole" && python -m pytest test_circle_state_machine.py -v`
Expected: `TestCircleCompletion`全部FAIL（`navigate()`里`_on_circle_complete`还是老逻辑）

- [ ] **Step 3: 实现——`_on_circle_complete`改退回基准线，新增`_on_retreat_complete`**

找到 `navigate()` 里航点耗尽的分支判断（原第414-423行）：

```python
        if self.target_index >= len(self.targets):
            if self.nav_mode == "CIRCLING":
                self._on_circle_complete()
            elif self.nav_mode == "TO_LANDING":
                logger.info("到达降落点")
                self.state = "LAND"
            else:
                logger.info("全部航点完成")
                self.state = "LAND"
            return
```

替换为：

```python
        if self.target_index >= len(self.targets):
            if self.nav_mode == "CIRCLING":
                self._on_circle_complete(pos)
            elif self.nav_mode == "RETREAT":
                self._on_retreat_complete()
            elif self.nav_mode == "TO_LANDING":
                logger.info("到达降落点")
                self.state = "LAND"
            else:
                logger.info("全部航点完成")
                self.state = "LAND"
            return
```

找到 `_on_circle_complete` 方法（原第762-776行），整体替换为：

```python
    def _on_circle_complete(self, pos):
        cx, cy = self._approach_pole_center
        color = self._approach_color
        logger.info(f"{color}杆塔({cx:.2f},{cy:.2f})环绕完成")
        self.circled_poles.append((cx, cy, color))
        self.circled_colors.add(color)
        self._approach_pole_center = None
        self._approach_color = None
        if self.pole_vision is not None:
            self.pole_vision.set_locked_color(None)
        # 先退回x=0基准线，再决定继续巡航还是转向降落(见2026-07-14设计文档)
        self.targets = [[0.0, pos[1], self._cruise_z]]
        self.target_index = 0
        self.last_target_index = -1
        self.nav_mode = "RETREAT"

    def _on_retreat_complete(self):
        if len(self.circled_colors) >= self.pole_total:
            logger.info(f"已绕完全部{self.pole_total}种颜色，前往降落点")
            self.targets = self.load_waypoints('landing_router.txt')
            self.target_index = 0
            self.nav_mode = "TO_LANDING"
        else:
            self.targets = self._patrol_saved_targets
            self.target_index = self._patrol_saved_index
            self.nav_mode = "PATROL"
        self.last_target_index = -1
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd "D:/项目与工具/Python项目/Project2/Project2/drone_control/circle_pole" && python -m pytest test_circle_state_machine.py -v`
Expected: 全部PASS

- [ ] **Step 5: Commit**

```bash
cd "D:/项目与工具/Python项目/Project2/Project2"
git add drone_control/circle_pole/Mission_GPT.py drone_control/circle_pole/test_circle_state_machine.py
git commit -m "feat(circle_pole): 环绕完成先退回x=0基准线(RETREAT)，降落路径改用landing_router.txt"
```

---

### Task 11: 全量回归 + `main.py` 接入 `PoleVision`

**Files:**
- Modify: `drone_control/circle_pole/main.py`

- [ ] **Step 1: 查看 `main.py` 现有雷达/mission初始化方式**

Run: `grep -n "radar_obj\|PoleTracker\|mission(" "D:/项目与工具/Python项目/Project2/Project2/drone_control/circle_pole/main.py"`

（根据输出确认`radar_obj`是怎么创建并传给`mission()`的，`pole_vision_obj`按同样风格创建——可选依赖，创建失败/环境变量未设置时传`None`，不阻塞主流程，这一步要先看实际代码再改，不能凭空套模板）

- [ ] **Step 2: 参照雷达对象的创建/传参模式，新增 `PoleVision` 初始化并传给 `mission()`**

在雷达对象创建代码附近，新增（`device`用`Mission_GPT.CAMERA_DEVICE`常量，创建失败调用`logger.error`不中断主流程，模式与`radar_obj`初始化一致）：

```python
from Lcode.pole_vision import PoleVision
from Mission_GPT import CAMERA_DEVICE

pole_vision_obj = PoleVision(device=CAMERA_DEVICE)
if not pole_vision_obj.start():
    pole_vision_obj = None  # 摄像头打不开，视觉子系统禁用，PATROL永远不会触发APPROACHING
```

把 `pole_vision_obj` 传给 `mission(...)` 构造调用的 `radar_obj=...` 参数后面，新增 `pole_vision_obj=pole_vision_obj`。

- [ ] **Step 3: 全量回归测试**

Run: `cd "D:/项目与工具/Python项目/Project2/Project2/drone_control/circle_pole" && python -m pytest -v`
Expected: 全部PASS，无收集错误，无跳过

- [ ] **Step 4: Commit**

```bash
cd "D:/项目与工具/Python项目/Project2/Project2"
git add drone_control/circle_pole/main.py
git commit -m "feat(circle_pole): main.py接入PoleVision，摄像头打不开时优雅降级为None"
```

---

## 本计划完成后仍未处理的事项（记录，不在本计划范围）

- **真机HSV阈值标定 + 视觉伺服PID调参**：`HSV_RANGES`/`APPROACH_Y_KP/KI/KD`都是未经真机验证的初始值，需要按spec文档"测试计划"5步顺序现场调试
- **`POLE_CIRCLE_RADIUS_M`/`patrol_router.txt`高度**：两处已知赛题指标偏差，spec文档已标注比赛前必须复核，本计划不改
- **2026-07-14审查记录的4条风险**（yaw耦合、T265丢失兜底、摄像头单点故障、雷达视觉目标错配）：已存入项目memory，用户决定"先记着，等真机测试暴露出来再处理"，不在本计划范围
- **`landing_router.txt`真实坐标**：现在是占位值(沿用旧`LANDING_POINT`常量)，需要现场量出实际降落标识位置后修改
