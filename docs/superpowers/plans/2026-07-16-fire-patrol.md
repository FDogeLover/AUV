# fire_patrol Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新建 `drone_control/fire_patrol/`（基于 `drone_control/basic/`），实现2023电赛G题"空地协同智能消防系统"无人机侧：6x5格心弓字形全覆盖巡逻、下视摄像头红色火源检测（单次触发）、视觉伺服悬停对准、警示LED/抛投机构占位接口、向消防车广播位置/火情坐标的透明UART协议。

**Architecture:** 复用`basic/`现有的`IDLE→TAKEOFF→NAVIGATE→LAND→END`状态机骨架和`navigate()`到达确认机制，参照`circle_pole/Mission_GPT.py`已验证过的`nav_mode`子状态模式（`state`保持`"NAVIGATE"`不变，`nav_mode`在`PATROL/APPROACH/CONFIRM_WARN/HOVER_DROP`间切换）。视觉检测模块参照`circle_pole/Lcode/pole_vision.py`的后台线程+纯函数拆分模式改造成下视摄像头单色(红)2维质心检测。通信新增`Lcode/Lground.py`（独立于飞控串口`Serial_fc`），参照`original/Lcode/Lprotocol.py`的`Serial_dmz`透明UART模式但改造帧格式为坐标+帧类型。

**Tech Stack:** Python 3, OpenCV(`cv2`)+numpy（复用`circle_pole`已验证的HSV检测管线）, pyserial, simple_pid, pytest

---

## 设计文档

`docs/superpowers/specs/2026-07-16-fire-patrol-design.md`（已提交，含设计审查发现的欠缺项）

## 文件结构

```
drone_control/fire_patrol/
├── main.py                      # 复制自 basic/main.py，接入 FireVision + Serial_ground
├── t265.py                      # 复制自 basic/t265.py，不改
├── router.txt                   # 由 coverage_path.py 生成的6x5格心弓字形航点
├── Lcode/
│   ├── __init__.py              # 复制自 basic/Lcode/__init__.py
│   ├── Logger.py                # 复制自 basic/Lcode/Logger.py
│   ├── global_variable.py       # 复制自 basic/Lcode/global_variable.py
│   ├── Lpid.py                  # 复制自 basic/Lcode/Lpid.py，不改
│   ├── Lprotocol.py             # 复制自 basic/Lcode/Lprotocol.py（仅Serial_fc），不改
│   ├── coverage_path.py         # 新增：6x5格心弓字形航点生成纯函数 + router.txt写入脚本
│   ├── fire_vision.py           # 新增：下视摄像头红色火源检测（纯函数+后台线程类）
│   ├── actuators.py             # 新增：warn_led()/drop_bag() 占位接口
│   └── Lground.py               # 新增：无人机→消防车透明UART广播（位置心跳帧+火情帧）
├── Mission_GPT.py                # 复制自 basic/Mission_GPT.py 后扩展：nav_mode状态机
└── test_*.py                     # 各模块单元测试
```

---

### Task 1: 脚手架 — 复制 basic/ 作为 fire_patrol/ 基础

**Files:**
- Create: `drone_control/fire_patrol/main.py`
- Create: `drone_control/fire_patrol/t265.py`
- Create: `drone_control/fire_patrol/Lcode/__init__.py`
- Create: `drone_control/fire_patrol/Lcode/Logger.py`
- Create: `drone_control/fire_patrol/Lcode/global_variable.py`
- Create: `drone_control/fire_patrol/Lcode/Lpid.py`
- Create: `drone_control/fire_patrol/Lcode/Lprotocol.py`
- Create: `drone_control/fire_patrol/Mission_GPT.py`

- [ ] **Step 1: 复制文件**

```bash
mkdir -p drone_control/fire_patrol/Lcode
cp drone_control/basic/main.py drone_control/fire_patrol/main.py
cp drone_control/basic/t265.py drone_control/fire_patrol/t265.py
cp drone_control/basic/Lcode/__init__.py drone_control/fire_patrol/Lcode/__init__.py
cp drone_control/basic/Lcode/Logger.py drone_control/fire_patrol/Lcode/Logger.py
cp drone_control/basic/Lcode/global_variable.py drone_control/fire_patrol/Lcode/global_variable.py
cp drone_control/basic/Lcode/Lpid.py drone_control/fire_patrol/Lcode/Lpid.py
cp drone_control/basic/Lcode/Lprotocol.py drone_control/fire_patrol/Lcode/Lprotocol.py
cp drone_control/basic/Mission_GPT.py drone_control/fire_patrol/Mission_GPT.py
```

- [ ] **Step 2: 改 main.py 顶部文档字符串，标注这是 fire_patrol 版本**

在 `drone_control/fire_patrol/main.py` 开头的 docstring 里把 `"基本飞行 — 主入口"` 改成：

```python
"""
G题空地协同智能消防系统 — 无人机侧主入口 (fire_patrol)

运行:
  python main.py

依赖:
  pyrealsense2 (或使用模拟 fallback)
  pyserial
  simple_pid
  numpy
  opencv-python (下视摄像头火情检测)

启动顺序:
  1. 创建 T265 实例
  2. 打开飞控串口 → 启动监听 + 发送线程
  3. 打开消防车广播串口(Serial_ground)
  4. 启动下视摄像头火情检测(FireVision)
  5. 创建任务 → 启动状态机
  6. 保持主线程存活
"""
```

- [ ] **Step 3: 验证目录结构**

```bash
ls drone_control/fire_patrol/Lcode/
```

Expected: `__init__.py Logger.py global_variable.py Lpid.py Lprotocol.py`

- [ ] **Step 4: Commit**

```bash
git add drone_control/fire_patrol/
git commit -m "fire_patrol: 脚手架，基于basic/复制骨架文件"
```

---

### Task 2: 覆盖巡逻航点生成

**Files:**
- Create: `drone_control/fire_patrol/Lcode/coverage_path.py`
- Test: `drone_control/fire_patrol/test_coverage_path.py`
- Create (generated output): `drone_control/fire_patrol/router.txt`

- [ ] **Step 1: 写失败测试**

```python
# drone_control/fire_patrol/test_coverage_path.py
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from Lcode.coverage_path import generate_boustrophedon_waypoints, CRUISE_Z_M


class TestGenerateBoustrophedonWaypoints:
    def test_returns_10_turn_waypoints(self):
        wps = generate_boustrophedon_waypoints()
        assert len(wps) == 10

    def test_first_waypoint_is_row0_start_at_origin(self):
        wps = generate_boustrophedon_waypoints()
        assert wps[0] == [0.0, 0.0, CRUISE_Z_M]

    def test_row0_end_at_x4_y0(self):
        wps = generate_boustrophedon_waypoints()
        assert wps[1] == [4.0, 0.0, CRUISE_Z_M]

    def test_lanes_alternate_direction_boustrophedon(self):
        wps = generate_boustrophedon_waypoints()
        # row1: 4.0,0.8 -> 0.0,0.8 (反向)
        assert wps[2] == [4.0, 0.8, CRUISE_Z_M]
        assert wps[3] == [0.0, 0.8, CRUISE_Z_M]

    def test_last_waypoint_is_row4_end_at_x4_y3_2(self):
        wps = generate_boustrophedon_waypoints()
        assert wps[9] == [4.0, 3.2, CRUISE_Z_M]

    def test_row_y_spacing_is_0_8m(self):
        wps = generate_boustrophedon_waypoints()
        ys = [wps[i][1] for i in (0, 2, 4, 6, 8)]
        assert ys == [0.0, 0.8, 1.6, 2.4, 3.2]

    def test_num_rows_and_cols_are_configurable(self):
        wps = generate_boustrophedon_waypoints(num_cols=3, num_rows=2, cell_size_m=1.0)
        # 2行3列 -> col跨度(3-1)*1.0=2.0m, row跨度(2-1)*1.0=1.0m, 2行*2转弯=4个航点
        assert len(wps) == 4
        assert wps[0] == [0.0, 0.0, CRUISE_Z_M]
        assert wps[1] == [2.0, 0.0, CRUISE_Z_M]
        assert wps[3] == [0.0, 1.0, CRUISE_Z_M]
```

- [ ] **Step 2: 运行测试确认失败**

```bash
cd drone_control/fire_patrol && python -m pytest test_coverage_path.py -v
```

Expected: FAIL，`ModuleNotFoundError: No module named 'Lcode.coverage_path'`

- [ ] **Step 3: 实现 coverage_path.py**

```python
# drone_control/fire_patrol/Lcode/coverage_path.py
"""6列x5行格心弓字形(boustrophedon)全覆盖巡逻航点生成。
见 docs/superpowers/specs/2026-07-16-fire-patrol-design.md "覆盖巡逻路径"一节。

赛题40dm x 48dm巡防区域按8dm x 8dm划分成6列x5行=30个格心，本地坐标系原点
(0,0,0)取T265上电点，起降点物理上放在row0/col0格心，格心间距0.8m。
只在每行两端的格心转弯，行内连续飞行(不逐格停留)。
"""
from typing import List

NUM_COLS = 6
NUM_ROWS = 5
CELL_SIZE_M = 0.8
CRUISE_Z_M = 1.8  # 对应赛题18dm巡航高度要求


def generate_boustrophedon_waypoints(num_cols: int = NUM_COLS,
                                      num_rows: int = NUM_ROWS,
                                      cell_size_m: float = CELL_SIZE_M,
                                      cruise_z_m: float = CRUISE_Z_M) -> List[List[float]]:
    """生成弓字形转弯点列表，每行2个航点(起点+终点)，共 num_rows*2 个。"""
    x_max = (num_cols - 1) * cell_size_m
    waypoints = []
    for row in range(num_rows):
        y = row * cell_size_m
        if row % 2 == 0:
            x_start, x_end = 0.0, x_max
        else:
            x_start, x_end = x_max, 0.0
        waypoints.append([x_start, y, cruise_z_m])
        waypoints.append([x_end, y, cruise_z_m])
    return waypoints


def save_router_txt(path: str, waypoints: List[List[float]]) -> None:
    """写入 basic/Mission_GPT.py `load_waypoints()` 兼容的 x,y,z 逐行格式，
    末尾追加返回原点(0,0,0)降落航点。"""
    with open(path, "w") as f:
        f.write("# x,y,z (meters) — fire_patrol 6x5格心弓字形全覆盖巡逻航点\n")
        f.write("# 自动生成，见 Lcode/coverage_path.py generate_boustrophedon_waypoints()\n")
        for x, y, z in waypoints:
            f.write(f"{x:.2f},{y:.2f},{z:.2f}\n")
        f.write("0.00,0.00,0.00\n")  # 返航降落点 = 本地原点


if __name__ == "__main__":
    import os
    wps = generate_boustrophedon_waypoints()
    out_path = os.path.join(os.path.dirname(__file__), "..", "router.txt")
    save_router_txt(out_path, wps)
    print(f"写入 {len(wps)}+1 个航点到 {out_path}")
```

- [ ] **Step 4: 运行测试确认通过**

```bash
cd drone_control/fire_patrol && python -m pytest test_coverage_path.py -v
```

Expected: 9 passed

- [ ] **Step 5: 生成 router.txt**

```bash
cd drone_control/fire_patrol && python Lcode/coverage_path.py
cat router.txt
```

Expected: 11行数据(10个巡逻转弯点+1个降落点)，第一行是`0.00,0.00,1.80`，最后一行是`0.00,0.00,0.00`

- [ ] **Step 6: Commit**

```bash
git add drone_control/fire_patrol/Lcode/coverage_path.py drone_control/fire_patrol/test_coverage_path.py drone_control/fire_patrol/router.txt
git commit -m "fire_patrol: 6x5格心弓字形覆盖巡逻航点生成"
```

---

### Task 3: 下视摄像头火情检测

**Files:**
- Create: `drone_control/fire_patrol/Lcode/fire_vision.py`
- Test: `drone_control/fire_patrol/test_fire_vision.py`

参照`circle_pole/Lcode/pole_vision.py`的拆分模式（纯函数+后台线程类），但改为：单色(红)、返回2维质心偏移(dx_px, dy_px)而非前置摄像头的单轴dx_px、按面积范围过滤（火源是俯视看到的近似圆形光斑，不是细高的杆子，不用高宽比形状过滤）、检测结果做滑动平均平滑。

- [ ] **Step 1: 写失败测试**

```python
# drone_control/fire_patrol/test_fire_vision.py
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

import numpy as np
import pytest

from Lcode.fire_vision import detect_fire, SmoothedFireDetector, MIN_FIRE_AREA_PX, MAX_FIRE_AREA_PX


def _blank_frame(width=1920, height=1080):
    return np.zeros((height, width, 3), dtype=np.uint8)


def _draw_circle_bgr(frame, color_bgr, cx, cy, radius=40):
    cv2_circle(frame, color_bgr, cx, cy, radius)
    return frame


def cv2_circle(frame, color_bgr, cx, cy, radius):
    import cv2
    cv2.circle(frame, (cx, cy), radius, color_bgr, -1)


class TestDetectFire:
    def test_no_target_returns_none(self):
        frame = _blank_frame()
        result = detect_fire(frame)
        assert result is None

    def test_red_circle_detected_with_centered_offset_near_zero(self):
        frame = _blank_frame()
        _draw_circle_bgr(frame, (0, 0, 255), cx=960, cy=540, radius=40)
        dx_px, dy_px = detect_fire(frame)
        assert dx_px == pytest.approx(0.0, abs=5)
        assert dy_px == pytest.approx(0.0, abs=5)

    def test_offset_right_and_below_center_is_positive(self):
        frame = _blank_frame()
        _draw_circle_bgr(frame, (0, 0, 255), cx=1200, cy=700, radius=40)
        dx_px, dy_px = detect_fire(frame)
        assert dx_px > 0
        assert dy_px > 0

    def test_offset_left_and_above_center_is_negative(self):
        frame = _blank_frame()
        _draw_circle_bgr(frame, (0, 0, 255), cx=700, cy=300, radius=40)
        dx_px, dy_px = detect_fire(frame)
        assert dx_px < 0
        assert dy_px < 0

    def test_area_below_min_threshold_ignored(self):
        frame = _blank_frame()
        _draw_circle_bgr(frame, (0, 0, 255), cx=960, cy=540, radius=3)  # 面积远小于MIN_FIRE_AREA_PX
        assert detect_fire(frame) is None

    def test_area_above_max_threshold_ignored(self):
        """面积过大(比如整面墙反光)不当作火源，见设计文档审查发现的"误触发风险"应对。"""
        frame = _blank_frame()
        _draw_circle_bgr(frame, (0, 0, 255), cx=960, cy=540, radius=500)  # 远超MAX_FIRE_AREA_PX
        assert detect_fire(frame) is None


class TestSmoothedFireDetector:
    def test_no_detection_returns_none(self):
        det = SmoothedFireDetector(window=3)
        assert det.update(None) is None

    def test_single_frame_not_enough_returns_none(self):
        det = SmoothedFireDetector(window=3)
        assert det.update((10.0, 5.0)) is None

    def test_window_full_returns_average(self):
        det = SmoothedFireDetector(window=3)
        det.update((10.0, 0.0))
        det.update((20.0, 0.0))
        result = det.update((30.0, 0.0))
        assert result == pytest.approx((20.0, 0.0))

    def test_single_none_frame_resets_window(self):
        """检测中途丢失一帧应该重新积累，不能用陈旧帧掺进平均——避免真实火源移出
        画面后仍用旧偏移量继续修正(见设计文档APPROACH阶段防抖要求)。"""
        det = SmoothedFireDetector(window=3)
        det.update((10.0, 0.0))
        det.update((20.0, 0.0))
        det.update(None)  # 丢失一帧，窗口应清空
        assert det.update((5.0, 0.0)) is None  # 只有1帧，还不够
```

- [ ] **Step 2: 运行测试确认失败**

```bash
cd drone_control/fire_patrol && python -m pytest test_fire_vision.py -v
```

Expected: FAIL，`ModuleNotFoundError: No module named 'Lcode.fire_vision'`

- [ ] **Step 3: 实现 fire_vision.py**

```python
# drone_control/fire_patrol/Lcode/fire_vision.py
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
```

- [ ] **Step 4: 运行测试确认通过**

```bash
cd drone_control/fire_patrol && python -m pytest test_fire_vision.py -v
```

Expected: 10 passed

- [ ] **Step 5: Commit**

```bash
git add drone_control/fire_patrol/Lcode/fire_vision.py drone_control/fire_patrol/test_fire_vision.py
git commit -m "fire_patrol: 下视摄像头红色火源检测(2维质心偏移+滑动平均)"
```

---

### Task 4: 警示LED / 抛投机构占位接口

**Files:**
- Create: `drone_control/fire_patrol/Lcode/actuators.py`
- Test: `drone_control/fire_patrol/test_actuators.py`

- [ ] **Step 1: 写失败测试**

```python
# drone_control/fire_patrol/test_actuators.py
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from Lcode.actuators import warn_led, drop_bag


class TestWarnLed:
    def test_returns_true_placeholder(self, caplog):
        result = warn_led()
        assert result is True


class TestDropBag:
    def test_returns_true_placeholder(self, caplog):
        result = drop_bag()
        assert result is True
```

- [ ] **Step 2: 运行测试确认失败**

```bash
cd drone_control/fire_patrol && python -m pytest test_actuators.py -v
```

Expected: FAIL，`ModuleNotFoundError: No module named 'Lcode.actuators'`

- [ ] **Step 3: 实现 actuators.py**

```python
# drone_control/fire_patrol/Lcode/actuators.py
"""警示LED / 抛投机构占位接口。硬件均未就绪(见设计文档"占位接口"一节)，
GPIO映射留TBD——先打日志占位，硬件到位后在这两个函数内部接GPIO调用，
调用方(Mission_GPT.py)不用改动。"""
from Lcode.Logger import logger


def warn_led() -> bool:
    """点亮警示LED，示警识别到火情。硬件未就绪，先打日志占位。"""
    logger.info("[占位] warn_led(): 警示LED点亮（硬件未接入）")
    return True


def drop_bag() -> bool:
    """触发抛投机构释放灭火包。硬件未就绪，先打日志占位。"""
    logger.info("[占位] drop_bag(): 抛投灭火包（硬件未接入）")
    return True
```

- [ ] **Step 4: 运行测试确认通过**

```bash
cd drone_control/fire_patrol && python -m pytest test_actuators.py -v
```

Expected: 2 passed

- [ ] **Step 5: Commit**

```bash
git add drone_control/fire_patrol/Lcode/actuators.py drone_control/fire_patrol/test_actuators.py
git commit -m "fire_patrol: 警示LED/抛投机构占位接口"
```

---

### Task 5: 无人机→消防车通信协议

**Files:**
- Create: `drone_control/fire_patrol/Lcode/Lground.py`
- Test: `drone_control/fire_patrol/test_Lground.py`

参照`original/Lcode/Lprotocol.py`的`Serial_dmz`透明UART模式（`AA...FF`帧、独立发送线程），但帧内容改为坐标+帧类型：帧类型0=心跳位置帧(1Hz)，帧类型1=火情帧(一次性)。坐标用cm整数(int16，两字节，与设计文档"通信协议"一节一致)。

- [ ] **Step 1: 写失败测试**

```python
# drone_control/fire_patrol/test_Lground.py
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from Lcode.Lground import build_position_frame, build_fire_frame, FRAME_TYPE_POSITION, FRAME_TYPE_FIRE


class TestBuildPositionFrame:
    def test_frame_starts_with_AA_and_ends_with_FF(self):
        frame = build_position_frame(x_cm=150, y_cm=-30)
        assert frame[0] == 0xAA
        assert frame[-1] == 0xFF

    def test_frame_type_byte_is_position(self):
        frame = build_position_frame(x_cm=150, y_cm=-30)
        assert frame[1] == FRAME_TYPE_POSITION

    def test_negative_coordinate_roundtrips_as_signed_int16(self):
        frame = build_position_frame(x_cm=-100, y_cm=200)
        x_bytes = frame[2:4]
        y_bytes = frame[4:6]
        x = int.from_bytes(x_bytes, byteorder="little", signed=True)
        y = int.from_bytes(y_bytes, byteorder="little", signed=True)
        assert x == -100
        assert y == 200


class TestBuildFireFrame:
    def test_frame_type_byte_is_fire(self):
        frame = build_fire_frame(x_cm=80, y_cm=120)
        assert frame[1] == FRAME_TYPE_FIRE

    def test_coordinates_encoded_correctly(self):
        frame = build_fire_frame(x_cm=80, y_cm=120)
        x = int.from_bytes(frame[2:4], byteorder="little", signed=True)
        y = int.from_bytes(frame[4:6], byteorder="little", signed=True)
        assert x == 80
        assert y == 120
```

- [ ] **Step 2: 运行测试确认失败**

```bash
cd drone_control/fire_patrol && python -m pytest test_Lground.py -v
```

Expected: FAIL，`ModuleNotFoundError: No module named 'Lcode.Lground'`

- [ ] **Step 3: 实现 Lground.py**

```python
# drone_control/fire_patrol/Lcode/Lground.py
"""无人机→消防车透明UART广播。见 docs/superpowers/specs/2026-07-16-fire-patrol-design.md
"通信协议"一节。参照 original/Lcode/Lprotocol.py 的 Serial_dmz 透明UART模式
(AA...FF帧、独立发送线程)，但帧内容从cls/cnt改为坐标+帧类型：
  帧类型0=心跳位置帧(1Hz，满足基本要求(3)"每秒1次位置坐标")
  帧类型1=火情帧(一次性，HOVER_DROP阶段触发)
坐标单位cm(int16小端有符号)，比dm(赛题原始单位)精度更高，避免消防车侧计算/
显示巡逻航迹曲线(基本要求(4))出现"阶梯感"(见设计文档通信协议一节)。
"""
import threading
import time

import serial

from Lcode.Logger import logger

FRAME_TYPE_POSITION = 0x00
FRAME_TYPE_FIRE = 0x01


def _build_frame(frame_type: int, x_cm: int, y_cm: int) -> bytes:
    x_bytes = int(x_cm).to_bytes(2, byteorder="little", signed=True)
    y_bytes = int(y_cm).to_bytes(2, byteorder="little", signed=True)
    return bytes([0xAA, frame_type]) + x_bytes + y_bytes + bytes([0xFF])


def build_position_frame(x_cm: int, y_cm: int) -> bytes:
    return _build_frame(FRAME_TYPE_POSITION, x_cm, y_cm)


def build_fire_frame(x_cm: int, y_cm: int) -> bytes:
    return _build_frame(FRAME_TYPE_FIRE, x_cm, y_cm)


class Serial_ground(object):
    """透明UART链路，无人机→消防车单向广播，不需要接收(消防车侧不在本仓库范围)。"""

    def __init__(self, port: str, baudrate: int = 115200):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.05)

    def send_position(self, x_cm: int, y_cm: int) -> None:
        self.ser.write(build_position_frame(x_cm, y_cm))

    def send_fire(self, x_cm: int, y_cm: int) -> None:
        self.ser.write(build_fire_frame(x_cm, y_cm))

    def close(self) -> None:
        if self.ser.is_open:
            self.ser.close()
            logger.info("消防车广播串口已关闭")


def start_position_heartbeat(serial_ground: Serial_ground, get_position_cm, hz: float = 1.0) -> threading.Thread:
    """启动1Hz心跳位置广播后台线程(基本要求(3))。get_position_cm是无参可调用对象，
    返回(x_cm, y_cm)。"""
    interval = 1.0 / hz

    def _loop():
        while True:
            x_cm, y_cm = get_position_cm()
            serial_ground.send_position(x_cm, y_cm)
            time.sleep(interval)

    t = threading.Thread(target=_loop, daemon=True)
    t.start()
    return t
```

- [ ] **Step 4: 运行测试确认通过**

```bash
cd drone_control/fire_patrol && python -m pytest test_Lground.py -v
```

Expected: 5 passed

- [ ] **Step 5: Commit**

```bash
git add drone_control/fire_patrol/Lcode/Lground.py drone_control/fire_patrol/test_Lground.py
git commit -m "fire_patrol: 无人机->消防车透明UART广播协议(位置心跳帧/火情帧)"
```

---

### Task 6: Mission_GPT 状态机扩展 — PATROL/APPROACH/CONFIRM_WARN/HOVER_DROP

**Files:**
- Modify: `drone_control/fire_patrol/Mission_GPT.py`
- Test: `drone_control/fire_patrol/test_fire_mission_state_machine.py`

这是核心任务。在 `basic/Mission_GPT.py`（已复制到 `fire_patrol/Mission_GPT.py`）基础上新增字段与逻辑，`state`字段维持`"NAVIGATE"`，新增`nav_mode`子状态驱动检测/接近/示警/悬停抛投分支（参照`circle_pole/Mission_GPT.py`已验证的`nav_mode`模式）。

- [ ] **Step 1: 写失败测试（状态转换的纯逻辑部分，不依赖真实摄像头/串口）**

```python
# drone_control/fire_patrol/test_fire_mission_state_machine.py
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from Mission_GPT import (
    mission,
    APPROACH_CENTERED_DIST_M,
    HOVER_DROP_DURATION_S,
)


class _FakeRealsense:
    def __init__(self, pos=(0.0, 0.0, 1.8), yaw=0.0, velocity=(0.0, 0.0, 0.0), confidence=3):
        self._pos = pos
        self._yaw = yaw
        self._velocity = velocity
        self._confidence = confidence
        self._running = True

    def start(self):
        return True

    def autoset(self):
        pass

    def get_tracking_confidence(self):
        return self._confidence

    def get_position(self):
        return self._pos

    def get_orientation(self):
        return (0.0, 0.0, self._yaw)

    def get_velocity(self):
        return self._velocity

    def is_running(self):
        return self._running

    def stop(self):
        self._running = False


def _make_mission(tmp_path):
    re_fc = [0] * 14
    se_fc = [0] * 11
    router = tmp_path / "router.txt"
    router.write_text("0.0,0.0,1.8\n4.0,0.0,1.8\n0.0,0.0,0.0\n")
    old_cwd = os.getcwd()
    os.chdir(tmp_path)
    try:
        m = mission(re_fc, se_fc, realsense_obj=_FakeRealsense())
    finally:
        os.chdir(old_cwd)
    m.t265_ok = True
    m.nav_mode = "PATROL"
    return m


class TestFireTriggerLatch:
    def test_fire_triggered_defaults_false(self, tmp_path):
        m = _make_mission(tmp_path)
        assert m.fire_triggered is False

    def test_maybe_trigger_approach_switches_mode_once(self, tmp_path):
        m = _make_mission(tmp_path)
        m.saved_target_index_before_fire = None
        triggered = m.maybe_trigger_approach(detection=(50.0, 30.0))
        assert triggered is True
        assert m.nav_mode == "APPROACH"
        assert m.fire_triggered is True

    def test_second_detection_does_not_retrigger(self, tmp_path):
        m = _make_mission(tmp_path)
        m.maybe_trigger_approach(detection=(50.0, 30.0))
        m.nav_mode = "PATROL"  # 模拟已经处理完一次火情后回到PATROL
        triggered = m.maybe_trigger_approach(detection=(10.0, 10.0))
        assert triggered is False
        assert m.nav_mode == "PATROL"  # 不会被重新触发进APPROACH

    def test_no_detection_does_not_trigger(self, tmp_path):
        m = _make_mission(tmp_path)
        triggered = m.maybe_trigger_approach(detection=None)
        assert triggered is False
        assert m.fire_triggered is False


class TestApproachCentering:
    def test_pixel_offset_within_deadband_counts_as_centered(self, tmp_path):
        m = _make_mission(tmp_path)
        # 像素偏移换算成的水平距离 < APPROACH_CENTERED_DIST_M 才算居中
        assert m.is_approach_centered(dx_px=2, dy_px=2) is True

    def test_large_pixel_offset_not_centered(self, tmp_path):
        m = _make_mission(tmp_path)
        assert m.is_approach_centered(dx_px=800, dy_px=800) is False


class TestHoverDropDuration:
    def test_hover_drop_duration_is_independent_constant(self):
        """见设计文档：HOVER_DROP_DURATION_S是赛题写死的3秒，不能跟navigate()
        到达确认用的arrival_hold_s混用。"""
        assert HOVER_DROP_DURATION_S == 3.0


class TestResumeAfterHoverDrop:
    def test_resume_continues_from_saved_index_not_reset(self, tmp_path):
        m = _make_mission(tmp_path)
        m.target_index = 3
        m.maybe_trigger_approach(detection=(50.0, 30.0))  # 保存target_index=3
        m.finish_hover_drop_and_resume()
        assert m.nav_mode == "PATROL"
        assert m.target_index == 3  # 恢复到触发时保存的索引，不重置为0
```

- [ ] **Step 2: 运行测试确认失败**

```bash
cd drone_control/fire_patrol && python -m pytest test_fire_mission_state_machine.py -v
```

Expected: FAIL，`ImportError: cannot import name 'APPROACH_CENTERED_DIST_M'`

- [ ] **Step 3: 修改 Mission_GPT.py — 新增常量**

在文件顶部常量区（`arrival_hold_s`等定义之后）加入：

```python
# ---------- fire_patrol 新增常量 ----------
FIRE_VISION_STALE_S = 0.5       # 火情检测结果超过此值未更新，视为摄像头/线程故障，不阻断飞行
APPROACH_DEADBAND_PX = 20       # 像素偏移小于此值不修正，防止中心附近来回抖
APPROACH_GAIN = 0.15            # APPROACH独立的小增益，明显小于navigate()跨格移动用的PID增益
APPROACH_MAX_STEP_CMPS = 8      # APPROACH阶段单次修正的速度上限(cm/s)，远小于navigate()的40，避免大幅晃动
APPROACH_TIMEOUT_S = 10.0       # 视觉伺服对准超时兜底，超时不强求完全居中，直接进入CONFIRM_WARN
APPROACH_CENTERED_DIST_M = 0.3  # 像素偏移换算的水平距离小于此值才算"对准"，对应赛题
                                  # 发挥部分(1)"接近火源水平距离<=5dm"，留量到3dm量级
HOVER_DROP_ALTITUDE_CM = 10.0 * 10  # 悬停抛投高度=10dm=100cm
HOVER_DROP_DURATION_S = 3.0     # 赛题写死的固定悬停时长，与arrival_hold_s(navigate()到达确认用)
                                  # 完全独立，不能混用——见设计文档"HOVER_DROP"一节
PIXEL_TO_METER_AT_CRUISE = 0.0015  # 像素偏移->水平距离粗略换算系数，现场需按实际摄像头
                                     # FOV/高度标定，这里给占位初始值
```

- [ ] **Step 4: 修改 Mission_GPT.py — `__init__`新增字段**

在`self.emergency_stop = False`之后加入：

```python
        # fire_patrol: 火情检测/接近/悬停抛投状态
        self.nav_mode = "PATROL"  # PATROL / APPROACH / CONFIRM_WARN / HOVER_DROP
        self.fire_triggered = False  # 全程只响应一次火情检测（见设计文档）
        self.saved_target_index_before_fire = None
        self._approach_start_time = None
        self.fire_vision = None  # main.py 注入 FireVision 实例
        self.serial_ground = None  # main.py 注入 Serial_ground 实例
```

- [ ] **Step 5: 修改 Mission_GPT.py — 新增触发/居中/续飞方法**

在`_on_arrival`方法之后插入：

```python
    # ================= fire_patrol: 火情检测触发 =================
    def maybe_trigger_approach(self, detection):
        """detection为None或已经触发过(fire_triggered=True)时不触发。
        触发时保存当前target_index用于HOVER_DROP完成后续飞。"""
        if detection is None or self.fire_triggered:
            return False
        self.saved_target_index_before_fire = self.target_index
        self.fire_triggered = True
        self.nav_mode = "APPROACH"
        self._approach_start_time = time.time()
        logger.info(f"检测到火情，悬停进入APPROACH（保存航点索引{self.target_index}）")
        return True

    def is_approach_centered(self, dx_px, dy_px):
        """像素偏移换算成水平距离，小于APPROACH_CENTERED_DIST_M才算对准正下方。"""
        dist_m = math.hypot(dx_px, dy_px) * PIXEL_TO_METER_AT_CRUISE
        return dist_m < APPROACH_CENTERED_DIST_M

    def approach_timed_out(self):
        if self._approach_start_time is None:
            return False
        return time.time() - self._approach_start_time >= APPROACH_TIMEOUT_S

    def finish_hover_drop_and_resume(self):
        """HOVER_DROP完成后恢复PATROL，从保存的target_index继续（不重置为0，
        不退回触发点），见设计文档"续飞逻辑"。"""
        self.target_index = self.saved_target_index_before_fire
        self.nav_mode = "PATROL"
        self._approach_start_time = None
```

- [ ] **Step 6: 运行测试确认通过**

```bash
cd drone_control/fire_patrol && python -m pytest test_fire_mission_state_machine.py -v
```

Expected: 8 passed

- [ ] **Step 7: 修改 Mission_GPT.py — `navigate()`接入检测触发与三个新分支**

把`navigate()`方法开头（`if self.target_index >= len(self.targets):`之前）加入PATROL态检测触发：

```python
    def navigate(self, pos, yaw):
        if self.nav_mode == "APPROACH":
            self._do_approach()
            return
        if self.nav_mode == "CONFIRM_WARN":
            self._do_confirm_warn()
            return
        if self.nav_mode == "HOVER_DROP":
            self._do_hover_drop(pos)
            return

        # PATROL态：持续检查火情检测结果，触发APPROACH
        if self.nav_mode == "PATROL" and self.fire_vision is not None:
            latest = self.fire_vision.latest()
            now = time.time()
            if (latest.get("dx_px") is not None
                    and now - latest.get("t", 0) < FIRE_VISION_STALE_S):
                if self.maybe_trigger_approach((latest["dx_px"], latest["dy_px"])):
                    return

        if self.target_index >= len(self.targets):
```

（原有`navigate()`函数体从`logger.info("全部航点完成")`开始的其余部分不变，因为只有`nav_mode=="PATROL"`才会走到这里——`APPROACH/CONFIRM_WARN/HOVER_DROP`都在函数顶部提前return了）

- [ ] **Step 8: 修改 Mission_GPT.py — 新增三个分支方法实现**

在`finish_hover_drop_and_resume`之后追加：

```python
    def _do_approach(self):
        """悬停对准正下方：独立小增益+死区+符号预验证的伺服修正，超时兜底进CONFIRM_WARN。
        见设计文档"APPROACH（视觉伺服对准正下方）"一节。"""
        if self.fire_vision is None:
            self.nav_mode = "CONFIRM_WARN"
            return
        latest = self.fire_vision.latest()
        dx_px, dy_px = latest.get("dx_px"), latest.get("dy_px")

        if dx_px is not None and dy_px is not None:
            if self.is_approach_centered(dx_px, dy_px):
                logger.info("APPROACH: 已对准正下方")
                self.nav_mode = "CONFIRM_WARN"
                return
            # 死区：像素偏移小于阈值不修正
            vx = 0 if abs(dx_px) < APPROACH_DEADBAND_PX else self.limit(
                dx_px * APPROACH_GAIN, APPROACH_MAX_STEP_CMPS)
            vy = 0 if abs(dy_px) < APPROACH_DEADBAND_PX else self.limit(
                dy_px * APPROACH_GAIN, APPROACH_MAX_STEP_CMPS)
            # 符号约定：dx_px>0(目标在画面右侧/x正方向)应产生正的vx指令使机体
            # 向x正方向移动以让目标居中——实现后必须先地面台架验证这个符号，
            # 防止正反馈导致偏差越修越大(见设计文档APPROACH一节)
            self.set_speed(int(vx), int(vy), 0, int(self._ramp_z_cm))
        else:
            self.set_speed(0, 0, 0, int(self._ramp_z_cm))

        if self.approach_timed_out():
            logger.warning("APPROACH: 对准超时，按当前位置继续")
            self.nav_mode = "CONFIRM_WARN"

    def _do_confirm_warn(self):
        """点亮警示LED后立即进入HOVER_DROP。"""
        from Lcode.actuators import warn_led
        warn_led()
        self.nav_mode = "HOVER_DROP"
        self._hover_drop_start_time = None

    def _do_hover_drop(self, pos):
        """降到10dm悬停HOVER_DROP_DURATION_S后抛投+广播坐标，见设计文档"HOVER_DROP"一节。"""
        self._step_ramp_z(HOVER_DROP_ALTITUDE_CM)
        self.set_speed(0, 0, 0, int(self._ramp_z_cm))

        if abs(self._ramp_z_cm - HOVER_DROP_ALTITUDE_CM) > 2.0:
            return  # 还在降高度途中，不开始计时

        if self._hover_drop_start_time is None:
            self._hover_drop_start_time = time.time()
            logger.info(f"HOVER_DROP: 到达{HOVER_DROP_ALTITUDE_CM:.0f}cm，悬停{HOVER_DROP_DURATION_S:.0f}s")
            return

        if time.time() - self._hover_drop_start_time < HOVER_DROP_DURATION_S:
            return

        from Lcode.actuators import drop_bag
        drop_bag()
        if self.serial_ground is not None:
            self.serial_ground.send_fire(int(pos[0] * 100), int(pos[1] * 100))
        logger.info("HOVER_DROP: 抛投+坐标广播完成，恢复PATROL续飞")
        self.finish_hover_drop_and_resume()
```

同时在`__init__`里`self._approach_start_time = None`之后加一行：

```python
        self._hover_drop_start_time = None
```

- [ ] **Step 9: 运行全部测试确认仍然通过**

```bash
cd drone_control/fire_patrol && python -m pytest test_fire_mission_state_machine.py test_coverage_path.py test_fire_vision.py test_actuators.py test_Lground.py -v
```

Expected: 全部通过，无回归

- [ ] **Step 10: Commit**

```bash
git add drone_control/fire_patrol/Mission_GPT.py drone_control/fire_patrol/test_fire_mission_state_machine.py
git commit -m "fire_patrol: PATROL/APPROACH/CONFIRM_WARN/HOVER_DROP状态机"
```

---

### Task 7: 接入 main.py

**Files:**
- Modify: `drone_control/fire_patrol/main.py`

- [ ] **Step 1: 修改 main.py 导入与初始化**

在`import Lcode.Lprotocol`之后加入：

```python
from Lcode.fire_vision import FireVision
from Lcode.Lground import Serial_ground, start_position_heartbeat
```

在`# 3. 创建任务`之前加入摄像头与消防车串口初始化：

```python
    # 2b. 下视摄像头火情检测
    fire_vision = FireVision(device=os.getenv("DRONE_FIRE_CAMERA", "/dev/video0"))
    fire_vision.start()  # 打不开时返回False，latest()永远全None，不阻断飞行(见设计文档)

    # 2c. 消防车广播串口
    serial_ground = Serial_ground(os.getenv("DRONE_GROUND_PORT", "/dev/ttyS7"))
```

修改`mission1 = mission(...)`那一行，改成：

```python
    # 3. 创建任务
    mission1 = mission(re_fc, se_fc, realsense, serial_fc)
    mission1.fire_vision = fire_vision
    mission1.serial_ground = serial_ground

    # 3b. 1Hz位置心跳广播（基本要求(3)）
    start_position_heartbeat(
        serial_ground,
        get_position_cm=lambda: (
            int(realsense.get_position()[0] * 100),
            int(realsense.get_position()[1] * 100),
        ),
    )
```

- [ ] **Step 2: 验证语法**

```bash
cd drone_control/fire_patrol && python -c "import ast; ast.parse(open('main.py').read())"
```

Expected: 无输出（无语法错误）

- [ ] **Step 3: Commit**

```bash
git add drone_control/fire_patrol/main.py
git commit -m "fire_patrol: main.py接入FireVision+消防车广播串口"
```

---

### Task 8: 全量测试 + 自查

- [ ] **Step 1: 跑全部单元测试**

```bash
cd drone_control/fire_patrol && python -m pytest -v
```

Expected: 全部通过

- [ ] **Step 2: 对照设计文档逐条检查覆盖**

对照`docs/superpowers/specs/2026-07-16-fire-patrol-design.md`逐节确认：覆盖巡逻路径(Task 2)、状态机(Task 6)、通信协议(Task 5)、占位接口(Task 4)、参数继承(未改`arrival_hold_s`等基础参数，沿用`basic`原值，Task 1复制未修改)——均有对应任务落实。

- [ ] **Step 3: Commit（如有自查修复）**

```bash
git add -A
git commit -m "fire_patrol: 自查修复"
```

---

## 遗留事项（不在本计划范围内，需真机测试阶段处理）

- HSV红色阈值(`HSV_RANGES_RED`)、面积阈值(`MIN_FIRE_AREA_PX`/`MAX_FIRE_AREA_PX`)、像素-距离换算系数(`PIXEL_TO_METER_AT_CRUISE`)均为占位经验值，需现场用实际LED光源标定
- `APPROACH`阶段符号约定（dx_px/dy_px正负对应的机体运动方向）实现后必须先地面台架验证，严禁直接上机测试
- 360s时间预算未做实测验证（见设计文档审查发现5）
- GPIO映射（`warn_led()`/`drop_bag()`硬件接入）留待机构就绪后再实现
- `DRONE_FIRE_CAMERA`/`DRONE_GROUND_PORT`环境变量的实际设备路径需现场确认
