# PoleTracker 接入 Mission_GPT 导航流程 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把已经重构好的 `PoleTracker`（`drone_control/basic_radar/Lcode/Lradar.py`）接入 `Mission_GPT.py` 的 `navigate()` 主循环，实现最小可用的避障反应——检测到确认的杆子且距离过近就悬停，不做绕行。雷达通过环境变量可选启用，默认关闭不影响现有不接雷达的测试。

**Architecture:** `main.py` 按 `DRONE_RADAR_ENABLED` 环境变量可选创建 `Serial_radar` 实例传给 `mission()`；`mission.__init__` 据此创建内部 `PoleTracker` 实例；`navigate()` 每30ms循环里按0.5秒节流轮询雷达，用一个独立可测的纯函数 `nearest_confirmed_pole_dist()` 判断是否需要悬停，命中就 `set_speed(0,0,0,ramp)` 并跳过本帧其余逻辑。

**Tech Stack:** Python 3.14，`pytest`（本项目 basic_radar 目录下已有先例，见 `test_pole_tracker.py`）。

**背景 spec：** `docs/superpowers/specs/2026-07-08-pole-tracker-navigation-integration-design.md`

---

## 开发环境准备

跟上一轮 `PoleTracker` 重构一样，本机开发环境需要：

```bash
cd drone_control/basic_radar && pip install pytest pyserial
```

（如果上一轮任务已经装过，这步可以跳过。）

---

### Task 1: `main.py` — 雷达可选创建

**Files:**
- Modify: `drone_control/basic_radar/main.py`

- [ ] **Step 1: 修改 `main.py`**

在文件顶部 import 区域（第19-25行附近）新增：

```python
from Lcode.Lradar import Serial_radar
```

在 `main()` 函数里，"2. 飞控串口"这一步之后、"3. 创建任务"之前（原第46-53行之间），插入：

```python
    # 2.5 雷达(可选，DRONE_RADAR_ENABLED=1 才启用，默认关闭不影响不接雷达的测试)
    radar = None
    if os.getenv("DRONE_RADAR_ENABLED", "0") == "1":
        radar_port = os.getenv("DRONE_RADAR_PORT", "/dev/ttyUSB0")
        radar_baud = int(os.getenv("DRONE_RADAR_BAUD", "460800"))
        radar = Serial_radar(radar_port, radar_baud)
        radar.port_open()
        radar.listen_start()
        logger.info(f"雷达避障已启用，端口={radar_port}，波特率={radar_baud}")
```

然后把原来的：

```python
    mission1 = mission(re_fc, se_fc, realsense, serial_fc)
```

改成：

```python
    mission1 = mission(re_fc, se_fc, realsense, serial_fc, radar_obj=radar)
```

- [ ] **Step 2: 验证不破坏现有导入**

`main.py` 这一步的改动本身不需要单元测试（这个文件是纯入口脚本，项目里一直没有对它做自动化测试），但要确认改完之后 `import` 不报错、语法正确：

```bash
cd drone_control/basic_radar && python -c "import ast; ast.parse(open('main.py', encoding='utf-8').read())"
```

预期：无输出，无异常（说明语法正确）。真正的运行时验证（`DRONE_RADAR_ENABLED=0` 默认路径不受影响）留到 Task 2 完成、`mission.__init__` 接受 `radar_obj` 参数之后一起做——现在如果直接 `python main.py` 会因为 `mission.__init__` 还不认识 `radar_obj` 参数而报 `TypeError`，这是预期的，Task 2 会修好。

- [ ] **Step 3: 提交**

```bash
git add drone_control/basic_radar/main.py
git commit -m "basic_radar: main.py按DRONE_RADAR_ENABLED环境变量可选创建雷达实例"
```

---

### Task 2: `Mission_GPT.py` — 悬停避障核心逻辑

**Files:**
- Modify: `drone_control/basic_radar/Mission_GPT.py`
- Test: `drone_control/basic_radar/test_mission_pole_integration.py`（新建）

- [ ] **Step 1: 写失败的测试**

创建 `drone_control/basic_radar/test_mission_pole_integration.py`：

```python
"""PoleTracker接入Mission_GPT导航流程的单元测试。

运行（先确保已 pip install pytest pyserial）：
    cd drone_control/basic_radar && python -m pytest test_mission_pole_integration.py -v
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from Mission_GPT import mission, nearest_confirmed_pole_dist, POLE_DANGER_DIST_M


# ════════════════════ nearest_confirmed_pole_dist ════════════════════

class TestNearestConfirmedPoleDist:
    def test_empty_list_returns_none(self):
        assert nearest_confirmed_pole_dist([], 0.0, 0.0) is None

    def test_single_pole_returns_its_distance(self):
        poles = [{"x": 0.3, "y": 0.0, "hits": 3}]
        assert nearest_confirmed_pole_dist(poles, 0.0, 0.0) == pytest.approx(0.3)

    def test_multiple_poles_returns_nearest(self):
        poles = [
            {"x": 5.0, "y": 0.0, "hits": 3},
            {"x": 0.4, "y": 0.3, "hits": 3},  # 距(0,0) = 0.5
        ]
        assert nearest_confirmed_pole_dist(poles, 0.0, 0.0) == pytest.approx(0.5)


# ════════════════════ mission 悬停/恢复行为 ════════════════════

def _make_mission(radar_obj=None):
    re_fc = [0] * 14
    se_fc = [0] * 11
    return mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None, radar_obj=radar_obj)


class TestMissionPoleHover:
    def test_no_radar_means_no_pole_tracker_and_no_hover(self):
        m = _make_mission(radar_obj=None)
        assert m.pole_tracker is None
        m.navigate([0.0, 0.0, 1.0], 0.0)  # 不应该抛异常
        assert m._pole_hovering is False

    def test_navigate_hovers_when_pole_confirmed_nearby(self):
        m = _make_mission(radar_obj=object())  # 哨兵对象，本测试跳过真实轮询，不会被调用
        m._last_pole_poll_time = time.time()  # 跳过本帧的雷达轮询(节流)，直接摆好历史数据
        for _ in range(3):
            m.pole_tracker._history.append([(0.3, 0.0)])  # 世界坐标(0.3,0)，离(0,0)只有0.3m

        sent = []
        m.set_speed = lambda x, y, yaw, z: sent.append((x, y, yaw, z))

        m.navigate([0.0, 0.0, 1.0], 0.0)

        assert m._pole_hovering is True
        assert sent == [(0, 0, 0, int(m._ramp_z_cm))]

    def test_navigate_does_not_hover_when_pole_far_away(self):
        m = _make_mission(radar_obj=object())
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(5.0, 5.0)])  # 远超 POLE_DANGER_DIST_M

        m.navigate([0.0, 0.0, 1.0], 0.0)

        assert m._pole_hovering is False

    def test_navigate_resumes_after_pole_no_longer_confirmed(self):
        m = _make_mission(radar_obj=object())
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.3, 0.0)])
        m.navigate([0.0, 0.0, 1.0], 0.0)
        assert m._pole_hovering is True

        m.pole_tracker.reset()
        m._last_pole_poll_time = time.time()
        m.navigate([0.0, 0.0, 1.0], 0.0)
        assert m._pole_hovering is False
```

- [ ] **Step 2: 运行测试，确认失败**

```bash
cd drone_control/basic_radar && python -m pytest test_mission_pole_integration.py -v
```

预期：`ImportError`（`nearest_confirmed_pole_dist`/`POLE_DANGER_DIST_M` 还不存在，`mission.__init__` 还不接受 `radar_obj` 参数）。

- [ ] **Step 3: 实现**

在 `drone_control/basic_radar/Mission_GPT.py` 顶部 import 区域（第16-19行附近）新增：

```python
from Lcode.Lradar import PoleTracker
```

在常量块末尾（第41行 `ARRIVAL_VEL_WINDOW` 注释之后）新增：

```python
POLE_POLL_INTERVAL_S = 0.5   # PoleTracker轮询间隔，跟07-07真机测试/回放验证用的节奏一致
POLE_DANGER_DIST_M = 0.6     # 确认的杆子距飞机当前位置小于此值就悬停(初步经验值，待真机调优)
POLE_YAW_SIGN = 1            # 未标定！CLAUDE.md已知问题13——真机/台架标定前只是假设值，
                              # 标定结果可能是+1也可能是-1，标定前这个避障功能的世界坐标可能是错的
```

在文件末尾（`class mission` 外面，任意顶层位置，建议紧跟在常量块后面、`class mission:` 定义之前）新增模块级纯函数：

```python
def nearest_confirmed_pole_dist(confirmed_poles, x, y):
    """confirmed_poles: PoleTracker.confirmed_poles()的返回值(list of {'x','y','hits'})。
    返回离(x,y)最近的确认杆子的距离(m)；没有杆子返回None。"""
    if not confirmed_poles:
        return None
    return min(math.hypot(p["x"] - x, p["y"] - y) for p in confirmed_poles)
```

修改 `mission.__init__` 签名（第46-48行）：

```python
    def __init__(self, re_fc: List[int], se_fc: List[int],
                 realsense_obj: Optional[t265_class] = None,
                 serial_fc_ref=None, radar_obj=None):
```

在 `__init__` 方法体末尾（第82-83行，飞行数据日志字段之后）新增：

```python
        # 雷达避障(可选)
        self.radar = radar_obj
        self.pole_tracker = PoleTracker(yaw_sign=POLE_YAW_SIGN) if radar_obj is not None else None
        self._last_pole_poll_time = 0.0
        self._pole_hovering = False  # 只在悬停状态切换时打日志，不是每帧刷屏
```

在 `navigate()` 方法里，`target = self.targets[self.target_index]` 和 `target_z = int(target[2] * 100)` 这两行（第295-296行）之后、`confidence = ...`（第298行）之前，插入：

```python
        # 雷达避障：检测到确认的杆子且距离过近就悬停，不绕行
        pole_hover = False
        pole_dist = None
        if self.pole_tracker is not None:
            now = time.time()
            if now - self._last_pole_poll_time >= POLE_POLL_INTERVAL_S:
                self._last_pole_poll_time = now
                self.pole_tracker.update(self.radar, pos[0], pos[1], yaw)
            pole_dist = nearest_confirmed_pole_dist(self.pole_tracker.confirmed_poles(), pos[0], pos[1])
            if pole_dist is not None and pole_dist < POLE_DANGER_DIST_M:
                pole_hover = True

        if pole_hover:
            if not self._pole_hovering:
                logger.warning(f"检测到杆子距离{pole_dist:.2f}m，悬停等待")
                self._pole_hovering = True
            self.set_speed(0, 0, 0, int(self._ramp_z_cm))
            return
        elif self._pole_hovering:
            logger.info("杆子确认已消失，恢复导航")
            self._pole_hovering = False
```

最后，在飞行日志写入的字典里（第384-398行的 `json.dumps({...})`）加一个字段，紧跟在 `"of_status": [...]，` 之后：

```python
                    "pole_hover": self._pole_hovering,
```

- [ ] **Step 4: 运行测试，确认通过**

```bash
cd drone_control/basic_radar && python -m pytest test_mission_pole_integration.py -v
```

预期：7 个测试全部 PASS（3 个 `nearest_confirmed_pole_dist` + 4 个 `mission` 悬停/恢复）。

同时重跑上一轮的 `test_pole_tracker.py`，确认没有被这次改动影响：

```bash
python -m pytest test_pole_tracker.py test_mission_pole_integration.py -v
```

预期：15 个测试全部 PASS（8 + 7）。

- [ ] **Step 5: 提交**

```bash
git add drone_control/basic_radar/Mission_GPT.py drone_control/basic_radar/test_mission_pole_integration.py
git commit -m "basic_radar: navigate()接入PoleTracker，检测到近距离杆子就悬停"
```

---

### Task 3: 桌面 DRY_RUN 冒烟检查

**Files:** 无代码改动，只运行验证

- [ ] **Step 1: 确认默认关闭路径完全不受影响**

```bash
cd drone_control/basic_radar
DRONE_DRY_RUN=1 DRONE_RADAR_ENABLED=0 timeout 5 python main.py
```

（Windows PowerShell 没有 `timeout` 命令的话用 `Start-Process`/`Ctrl+C` 手动中断，或者直接跑几秒后 `Ctrl+C`。）

预期：程序正常启动到"等待T265/等待起飞"的阶段（没有真实飞控/T265硬件的话会走"未连接"的确认流程，属于预期行为，不是这次改动引入的新问题），日志里**不应该**出现"雷达避障已启用"这行——确认 `DRONE_RADAR_ENABLED=0`（默认）时雷达代码路径完全没有被触发。

**如果没有真实飞控硬件连着，这一步会在等待串口/T265的地方卡住或报连接错误**——这是预期的（`main.py` 本来就需要真实硬件才能完整跑起来），只需要确认：a) 程序能正常导入、启动到"等待硬件"这一步，没有因为这次改动新增的代码（`radar_obj` 参数、`PoleTracker` 相关逻辑）本身报错；b) 没有雷达启用的日志。确认这两点就够了，不需要真的跑完整个飞行流程。

- [ ] **Step 2: 记录结果**

如果发现任何因为这次改动引入的报错（不是硬件缺失导致的），回到 Task 2 修复。如果一切正常，这一步不需要提交任何代码，口头/对话记录确认即可。

---

## 完成后不做的事（跟 spec 一致）

- 不做绕行/路径重规划
- 不做 `yaw_sign` 真机标定
- 不加超时强制恢复导航的机制
- 不修改 `original/` 全功能版的 `Mission_GPT.py`
