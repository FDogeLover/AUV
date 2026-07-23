# 绕障飞行器(D题) 阶段1：单杆环绕 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在新目录`drone_control/circle_pole/`实现"巡航→雷达检测杆塔→环绕飞行360°→飞到预设降落点降落"的最小可用状态机，先用2m量级单根杆子场景真机验证机制，阶段2(双杆全场)在此基础上只改配置。

**Architecture:** 以`drone_control/basic_radar/`为模板复制出`circle_pole/`。新增纯函数模块`Lcode/circle_planner.py`生成环绕航点；`Mission_GPT.py`在现有`navigate()`点到点导航基础上加一层`nav_mode`子状态机(`PATROL`/`CIRCLING`/`TO_LANDING`)，检测到未环绕的确认杆塔时把`self.targets`临时切换成环绕航点，环绕完成后切回巡航或转向降落点，全程复用现有到达确认(`arrival_hold_s`)、悬停避让(`POLE_DANGER_DIST_M`)机制不重写。

**Tech Stack:** Python 3, pytest, 现有`Lcode.Lradar.PoleTracker`/`Lcode.Lpid.PID`

设计依据：[docs/superpowers/specs/2026-07-13-circle-pole-design.md](../specs/2026-07-13-circle-pole-design.md)

---

### Task 1: 创建 `circle_pole/` 目录

**Files:**
- Create: `drone_control/circle_pole/`（从`drone_control/basic_radar/`复制）

- [ ] **Step 1: 复制目录，排除 `__pycache__`**

```bash
cd "D:/项目与工具/Python项目/Project2/drone_control"
cp -r basic_radar circle_pole
find circle_pole -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null
```

- [ ] **Step 2: 修正测试文件docstring里的路径引用（复制自basic_radar，指令还写着旧路径）**

```bash
cd "D:/项目与工具/Python项目/Project2/drone_control/circle_pole"
grep -rl "basic_radar" --include="*.py" . | xargs sed -i 's/basic_radar/circle_pole/g'
```

- [ ] **Step 3: 验证复制完整、旧测试套件在新目录下依然全绿（回归基线）**

Run: `cd "D:/项目与工具/Python项目/Project2/drone_control/circle_pole" && python -m pytest -v`
Expected: 全部现有测试（`test_pole_tracker.py`/`test_mission_pole_integration.py`/`test_arrival_confirm.py`等）PASS，无收集错误

- [ ] **Step 4: Commit**

```bash
cd "D:/项目与工具/Python项目/Project2"
git add drone_control/circle_pole
git commit -m "feat: 从basic_radar复制circle_pole作为绕障飞行器(D题)新版本起点"
```

---

### Task 2: 环绕航点生成器 `circle_planner.py`（TDD）

**Files:**
- Create: `drone_control/circle_pole/Lcode/circle_planner.py`
- Test: `drone_control/circle_pole/test_circle_planner.py`

- [ ] **Step 1: 写测试文件（此时`circle_planner`模块还不存在，测试会失败）**

```python
"""circle_planner.generate_circle_waypoints() 单元测试。

运行:
    cd drone_control/circle_pole && python -m pytest test_circle_planner.py -v
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from Lcode.circle_planner import generate_circle_waypoints


class TestGenerateCircleWaypoints:
    def test_point_count_is_n_points_plus_closing_point(self):
        pts = generate_circle_waypoints(0.0, 0.0, 1.0, 0.0, radius=0.5, n_points=6)
        assert len(pts) == 7

    def test_last_point_closes_loop_back_to_first(self):
        pts = generate_circle_waypoints(0.0, 0.0, 1.0, 0.0, radius=0.5, n_points=6)
        assert pts[-1] == pts[0]

    def test_all_points_at_exact_radius_from_center(self):
        pts = generate_circle_waypoints(1.0, -0.5, 2.0, -0.5, radius=0.5, n_points=6)
        for x, y, _z in pts:
            d = math.hypot(x - 1.0, y - (-0.5))
            assert d == pytest.approx(0.5, abs=1e-9)

    def test_first_point_is_nearest_point_on_circle_to_current_position(self):
        # 飞机在圆心正右方3m处，最近的圆上点应该也在正右方(圆心+半径,0)
        pts = generate_circle_waypoints(0.0, 0.0, 3.0, 0.0, radius=0.5, n_points=6)
        assert pts[0][0] == pytest.approx(0.5, abs=1e-9)
        assert pts[0][1] == pytest.approx(0.0, abs=1e-9)

    def test_cw_direction_decreases_angle_top_view(self):
        pts = generate_circle_waypoints(0.0, 0.0, 0.5, 0.0, radius=0.5, n_points=4, direction="cw")
        # 从(0.5,0)开始，顺时针(顶视)下一个点应转到(0,-0.5)附近(角度0°->-90°)
        assert pts[1][0] == pytest.approx(0.0, abs=1e-6)
        assert pts[1][1] == pytest.approx(-0.5, abs=1e-6)

    def test_ccw_direction_increases_angle_top_view(self):
        pts = generate_circle_waypoints(0.0, 0.0, 0.5, 0.0, radius=0.5, n_points=4, direction="ccw")
        assert pts[1][0] == pytest.approx(0.0, abs=1e-6)
        assert pts[1][1] == pytest.approx(0.5, abs=1e-6)

    def test_z_passed_through_to_all_points(self):
        pts = generate_circle_waypoints(0.0, 0.0, 1.0, 0.0, radius=0.5, n_points=6, z=1.35)
        assert all(p[2] == 1.35 for p in pts)

    def test_current_position_at_center_falls_back_to_angle_zero(self):
        pts = generate_circle_waypoints(0.0, 0.0, 0.0, 0.0, radius=0.5, n_points=6)
        assert pts[0][0] == pytest.approx(0.5, abs=1e-9)
        assert pts[0][1] == pytest.approx(0.0, abs=1e-9)

    def test_rejects_too_few_points(self):
        with pytest.raises(ValueError):
            generate_circle_waypoints(0.0, 0.0, 1.0, 0.0, n_points=2)

    def test_rejects_invalid_direction(self):
        with pytest.raises(ValueError):
            generate_circle_waypoints(0.0, 0.0, 1.0, 0.0, direction="sideways")
```

- [ ] **Step 2: 运行测试，确认失败(ModuleNotFoundError)**

Run: `cd "D:/项目与工具/Python项目/Project2/drone_control/circle_pole" && python -m pytest test_circle_planner.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'Lcode.circle_planner'`

- [ ] **Step 3: 实现 `Lcode/circle_planner.py`**

```python
"""环绕飞行航点生成器 — 纯函数，不依赖飞控/雷达对象，方便独立单元测试。"""
import math


def generate_circle_waypoints(center_x, center_y, cur_x, cur_y,
                               radius=0.5, n_points=6, direction="cw", z=1.5):
    """生成围绕(center_x, center_y)半径radius的环绕航点列表(世界系[x,y,z])。

    起始点取"当前位置->圆心连线"与圆的交点(离当前位置最近的圆上一点)，避免
    先横穿到圆上任意一点。direction="cw"(顺时针,顶视)/"ccw"(逆时针,顶视)。
    返回n_points+1个点：绕满一圈(n_points个等分点)后再重复第一个点，确保
    闭合>=360度(否则n_points个点首尾不重合，实际只转了(n_points-1)/n_points圈)。
    """
    if n_points < 3:
        raise ValueError("n_points must be >= 3")
    if direction not in ("cw", "ccw"):
        raise ValueError("direction must be 'cw' or 'ccw'")

    dx = cur_x - center_x
    dy = cur_y - center_y
    dist = math.hypot(dx, dy)
    start_angle = math.atan2(dy, dx) if dist > 1e-6 else 0.0

    sign = -1.0 if direction == "cw" else 1.0
    step = sign * (2 * math.pi / n_points)

    points = []
    for i in range(n_points):
        angle = start_angle + step * i
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        points.append([x, y, z])
    points.append(list(points[0]))
    return points
```

- [ ] **Step 4: 运行测试，确认全部通过**

Run: `cd "D:/项目与工具/Python项目/Project2/drone_control/circle_pole" && python -m pytest test_circle_planner.py -v`
Expected: 10 passed

- [ ] **Step 5: Commit**

```bash
cd "D:/项目与工具/Python项目/Project2"
git add drone_control/circle_pole/Lcode/circle_planner.py drone_control/circle_pole/test_circle_planner.py
git commit -m "feat(circle_pole): 添加环绕航点生成器circle_planner.py"
```

---

### Task 3: 阶段1巡航航线 `router.txt`

**Files:**
- Modify: `drone_control/circle_pole/router.txt`

- [ ] **Step 1: 替换成阶段1直线巡航航点**

内容（沿x轴走2m，y=0，巡航高度1.2m；杆塔真机测试时放在这条线附近、y方向偏移0.2~0.4m处，确保在雷达1.2m探测范围内能被扫到）：

```
# 阶段1：2m直线巡航，单杆验证用。杆塔实测摆放建议：路径旁y偏移0.2~0.4m处，
# 避免直接摆在航线正中间（悬停避让会在0.75m处触发，验证的是能不能先悬停
# 再切换到环绕，而不是一路顺畅冲上去）。
0.0,0.0,1.2
0.5,0.0,1.2
1.0,0.0,1.2
1.5,0.0,1.2
2.0,0.0,1.2
```

- [ ] **Step 2: Commit**

```bash
cd "D:/项目与工具/Python项目/Project2"
git add drone_control/circle_pole/router.txt
git commit -m "feat(circle_pole): 阶段1直线巡航航点(2m单杆验证用)"
```

---

### Task 4: `Mission_GPT.py` 状态机改造（TDD）

**Files:**
- Modify: `drone_control/circle_pole/Mission_GPT.py`
- Test: `drone_control/circle_pole/test_circle_state_machine.py`

- [ ] **Step 1: 写状态机测试文件（此时`nav_mode`等属性还不存在，测试会失败）**

```python
"""PATROL/CIRCLING/TO_LANDING状态机单元测试。

运行:
    cd drone_control/circle_pole && python -m pytest test_circle_state_machine.py -v
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from Mission_GPT import mission, POLE_CIRCLE_N_POINTS, LANDING_POINT


def _make_mission(radar_obj=None, pole_total=1):
    re_fc = [0] * 14
    se_fc = [0] * 11
    m = mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None, radar_obj=radar_obj)
    m.pole_total = pole_total
    return m


class TestPatrolTriggersCircling:
    def test_confirmed_new_pole_switches_to_circling(self):
        m = _make_mission(radar_obj=object(), pole_total=1)
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])

        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert m.nav_mode == "CIRCLING"
        assert m._circle_pole_center == pytest.approx((0.5, 0.3))
        assert len(m.targets) == POLE_CIRCLE_N_POINTS + 1
        assert m.target_index == 0

    def test_already_circled_pole_does_not_retrigger(self):
        m = _make_mission(radar_obj=object(), pole_total=2)
        m.circled_poles = [(0.5, 0.3)]
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])

        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert m.nav_mode == "PATROL"


class TestCirclingHoverExclusion:
    def test_own_circling_target_does_not_trigger_hover(self):
        m = _make_mission(radar_obj=object())
        m._circle_pole_center = (0.5, 0.3)
        m.nav_mode = "CIRCLING"
        m.targets = [[0.5, -0.2, 1.2], [1.0, 0.3, 1.2]]
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])  # 就是环绕目标本身

        m.set_speed = lambda *a, **k: None
        m.navigate([0.45, 0.28, 1.2], 0.0)

        assert m._pole_hovering is False

    def test_other_confirmed_pole_still_triggers_hover_during_circling(self):
        m = _make_mission(radar_obj=object())
        m._circle_pole_center = (0.5, 0.3)
        m.nav_mode = "CIRCLING"
        m.targets = [[0.5, -0.2, 1.2], [1.0, 0.3, 1.2]]
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.3, 0.0)])  # 另一根杆子，离(0,0)只有0.3m

        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert m._pole_hovering is True


class TestCircleCompletion:
    def test_single_pole_mission_switches_to_to_landing_when_circle_done(self):
        m = _make_mission(radar_obj=object(), pole_total=1)
        m.nav_mode = "CIRCLING"
        m._circle_pole_center = (0.5, 0.3)
        m.targets = [[0.5, 0.8, 1.2]]
        m.target_index = 1  # 已飞完最后一个环绕航点

        m.navigate([0.5, 0.8, 1.2], 0.0)

        assert m.nav_mode == "TO_LANDING"
        assert m.circled_poles == [(0.5, 0.3)]
        assert m.targets == [[LANDING_POINT[0], LANDING_POINT[1], m._cruise_z]]
        assert m.target_index == 0

    def test_multi_pole_mission_resumes_patrol_when_more_poles_remain(self):
        m = _make_mission(radar_obj=object(), pole_total=2)
        patrol_targets = [[0.0, 0.0, 1.2], [1.0, 0.0, 1.2], [2.0, 0.0, 1.2]]
        m._patrol_saved_targets = patrol_targets
        m._patrol_saved_index = 1
        m.nav_mode = "CIRCLING"
        m._circle_pole_center = (0.5, 0.3)
        m.targets = [[0.5, 0.8, 1.2]]
        m.target_index = 1

        m.navigate([0.5, 0.8, 1.2], 0.0)

        assert m.nav_mode == "PATROL"
        assert m.circled_poles == [(0.5, 0.3)]
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

- [ ] **Step 2: 运行测试，确认失败**

Run: `cd "D:/项目与工具/Python项目/Project2/drone_control/circle_pole" && python -m pytest test_circle_state_machine.py -v`
Expected: FAIL — `ImportError: cannot import name 'POLE_CIRCLE_N_POINTS'`（常量还不存在）

- [ ] **Step 3: 在`Mission_GPT.py`顶部导入新增`generate_circle_waypoints`，并添加环绕相关常量**

**注意：Step 3~6按顺序修改同一个文件，每步都会让文件变长，后面步骤不要依赖之前步骤给出的行号，一律用下面给出的原有代码文本(锚点)去定位插入位置。**

找到这一行（原有代码，紧跟在`from Lcode.Lradar import PoleTracker`之后）：

```python
from t265 import t265_class
```

在它前面插入一行（即让`generate_circle_waypoints`的import跟其他`Lcode`模块import放在一起）：

```python
from Lcode.circle_planner import generate_circle_waypoints
```

找到这段原有代码（`POLE_YAW_SIGN`常量定义，注释结尾是"标定结果可能是+1也可能是-1，标定前这个避障功能的世界坐标可能是错的"）：

```python
POLE_YAW_SIGN = 1            # 未标定！CLAUDE.md已知问题13——真机/台架标定前只是假设值，
                              # 标定结果可能是+1也可能是-1，标定前这个避障功能的世界坐标可能是错的
```

在这段代码**之后**添加：

```python
POLE_CIRCLE_RADIUS_M = 0.5   # 环绕半径，对应赛题50cm距离要求
POLE_CIRCLE_N_POINTS = 6     # 环绕航点数(60°一个)。弦长0.5m未超出已验证安全范围
                              # (已知问题15唯一站得住的结论是"扰动随步长单调增大"，无硬上限，
                              # 且问题21大范围大步长测试精度反而更好)
POLE_CIRCLE_DIRECTION = "cw" # 固定顺时针(顶视)，颜色识别接入后改为按红/绿判断
POLE_WORLD_MATCH_EPS_M = 0.2 # "同一根杆子"世界坐标匹配容差，跟PoleTracker.world_eps_m默认值一致
TOTAL_POLES = int(os.getenv("DRONE_POLE_TOTAL", "1"))  # 阶段1=1(默认)；阶段2设DRONE_POLE_TOTAL=2
LANDING_POINT = (2.0, 0.0)   # 降落点世界坐标占位值 — 现场量出实际降落标识位置后必须修改
```

- [ ] **Step 4: 在`mission.__init__`里添加新状态属性**

找到这一行原有代码（`__init__`里雷达避障相关属性的最后一行）：

```python
        self._hover_start_time = None  # 悬停开始时间，用于POLE_HOVER_TIMEOUT_S超时判断
```

在它之后添加：

```python
        # 环绕状态机(阶段1单杆/阶段2双杆共用)
        self.nav_mode = "PATROL"  # PATROL / CIRCLING / TO_LANDING
        self.circled_poles = []   # 已完成环绕的杆塔世界坐标 [(x,y), ...]
        self._circle_pole_center = None  # 当前正在环绕的杆塔世界坐标，从悬停避让判断中排除
        self._patrol_saved_targets = None
        self._patrol_saved_index = 0
        self._cruise_z = self.targets[0][2] if self.targets else put_height / 100
        self.pole_total = TOTAL_POLES
```

- [ ] **Step 5: 添加三个新的辅助方法**

找到这一行原有代码（`# ================= 到达处理 =================` 分节注释，`_on_arrival`方法定义前）：

```python
    # ================= 到达处理 =================
    def _on_arrival(self, target):
```

在 `# ================= 到达处理 =================` 这行**之前**插入：

```python
    # ================= 环绕状态机辅助方法 =================
    def _already_circled(self, x, y):
        return any(math.hypot(x - cx, y - cy) <= POLE_WORLD_MATCH_EPS_M
                   for cx, cy in self.circled_poles)

    def _find_new_pole(self, confirmed):
        for p in confirmed:
            if not self._already_circled(p["x"], p["y"]):
                return p
        return None

    def _exclude_circle_target(self, confirmed):
        if self._circle_pole_center is None:
            return confirmed
        cx, cy = self._circle_pole_center
        return [p for p in confirmed
                if math.hypot(p["x"] - cx, p["y"] - cy) > POLE_WORLD_MATCH_EPS_M]

    def _start_circling(self, pole, pos):
        self._patrol_saved_targets = self.targets
        self._patrol_saved_index = self.target_index
        self._circle_pole_center = (pole["x"], pole["y"])
        waypoints = generate_circle_waypoints(
            pole["x"], pole["y"], pos[0], pos[1],
            radius=POLE_CIRCLE_RADIUS_M, n_points=POLE_CIRCLE_N_POINTS,
            direction=POLE_CIRCLE_DIRECTION, z=self._cruise_z,
        )
        self.targets = waypoints
        self.target_index = 0
        self.last_target_index = -1
        self.nav_mode = "CIRCLING"
        logger.warning(
            f"检测到杆塔({pole['x']:.2f},{pole['y']:.2f})，开始环绕飞行，{len(waypoints)}个航点"
        )

    def _on_circle_complete(self):
        cx, cy = self._circle_pole_center
        logger.info(f"杆塔({cx:.2f},{cy:.2f})环绕完成")
        self.circled_poles.append((cx, cy))
        self._circle_pole_center = None
        if len(self.circled_poles) >= self.pole_total:
            logger.info(f"已绕完全部{self.pole_total}根杆塔，前往降落点")
            self.targets = [[LANDING_POINT[0], LANDING_POINT[1], self._cruise_z]]
            self.target_index = 0
            self.nav_mode = "TO_LANDING"
        else:
            self.targets = self._patrol_saved_targets
            self.target_index = self._patrol_saved_index
            self.nav_mode = "PATROL"
        self.last_target_index = -1
```

- [ ] **Step 6: 改造`navigate()`方法的开头部分**

找到这一段原有代码，从`def navigate(self, pos, yaw):`开始，到`pole_hover = pole_dist is not None and pole_dist < POLE_DANGER_DIST_M`结束（即原有的航点耗尽检查+雷达轮询+悬停判断这一整段，注意`if self.state == "TAKEOFF":`分支调用的是同一个`navigate`方法名，不要跟`def navigate`本身混淆——只替换这一个方法定义的开头部分）：

```python
    def navigate(self, pos, yaw):
        if self.target_index >= len(self.targets):
            logger.info("全部航点完成")
            self.state = "LAND"
            return

        target = self.targets[self.target_index]
        target_z = int(target[2] * 100)

        # 雷达避障：检测到确认的杆子且距离过近就悬停，不绕行。
        # 触发(POLE_DANGER_DIST_M)和恢复(POLE_RESUME_DIST_M)用两个不同阈值(滞回)，
        # 避免距离刚好卡在阈值附近抖动时悬停状态反复触发/取消。
        pole_hover = False
        pole_dist = None
        confirmed_poles_list = []  # 2026-07-09新增：记录全部确认杆子(不只是最近的一个)，
                                    # 用于验证多障碍物场景下是否真的同时跟踪了多个目标
        if self.pole_tracker is not None:
            now = time.time()
            if now - self._last_pole_poll_time >= POLE_POLL_INTERVAL_S:
                self._last_pole_poll_time = now
                self.pole_tracker.update(self.radar, pos[0], pos[1], yaw)
            confirmed = self.pole_tracker.confirmed_poles()
            confirmed_poles_list = [
                {"x": round(p["x"], 3), "y": round(p["y"], 3), "hits": p["hits"],
                 "dist": round(math.hypot(p["x"] - pos[0], p["y"] - pos[1]), 3)}
                for p in confirmed
            ]
            pole_dist = nearest_confirmed_pole_dist(confirmed, pos[0], pos[1])
            if self._pole_hovering:
                pole_hover = pole_dist is not None and pole_dist < POLE_RESUME_DIST_M
            else:
                pole_hover = pole_dist is not None and pole_dist < POLE_DANGER_DIST_M
```

替换为：

```python
    def navigate(self, pos, yaw):
        # 全部航点耗尽：按当前nav_mode分支处理(环绕完成/到达降落点/巡航耗尽兜底)
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

        target = self.targets[self.target_index]
        target_z = int(target[2] * 100)

        # 雷达避障：检测到确认的杆子且距离过近就悬停，不绕行(除非正在主动环绕它)。
        # 触发(POLE_DANGER_DIST_M)和恢复(POLE_RESUME_DIST_M)用两个不同阈值(滞回)，
        # 避免距离刚好卡在阈值附近抖动时悬停状态反复触发/取消。
        pole_hover = False
        pole_dist = None
        confirmed_poles_list = []  # 2026-07-09新增：记录全部确认杆子(不只是最近的一个)，
                                    # 用于验证多障碍物场景下是否真的同时跟踪了多个目标
        if self.pole_tracker is not None:
            now = time.time()
            if now - self._last_pole_poll_time >= POLE_POLL_INTERVAL_S:
                self._last_pole_poll_time = now
                self.pole_tracker.update(self.radar, pos[0], pos[1], yaw)
            confirmed = self.pole_tracker.confirmed_poles()
            confirmed_poles_list = [
                {"x": round(p["x"], 3), "y": round(p["y"], 3), "hits": p["hits"],
                 "dist": round(math.hypot(p["x"] - pos[0], p["y"] - pos[1]), 3)}
                for p in confirmed
            ]

            # PATROL态：发现一个未环绕过的确认杆塔，立即切到CIRCLING
            if self.nav_mode == "PATROL":
                new_pole = self._find_new_pole(confirmed)
                if new_pole is not None:
                    self._start_circling(new_pole, pos)
                    return

            # 悬停避让距离判断：排除当前正在主动环绕的目标(否则环绕航点一进0.75m
            # 就会被悬停逻辑拦下来，跟环绕意图矛盾)
            hover_check_poles = self._exclude_circle_target(confirmed)
            pole_dist = nearest_confirmed_pole_dist(hover_check_poles, pos[0], pos[1])
            if self._pole_hovering:
                pole_hover = pole_dist is not None and pole_dist < POLE_RESUME_DIST_M
            else:
                pole_hover = pole_dist is not None and pole_dist < POLE_DANGER_DIST_M
```

（后续`if pole_hover:`开始的代码块保持不变，不需要改动。）

- [ ] **Step 7: 运行测试，确认全部通过（新测试+旧回归测试）**

Run: `cd "D:/项目与工具/Python项目/Project2/drone_control/circle_pole" && python -m pytest -v`
Expected: 全部PASS，包括`test_circle_state_machine.py`的7个新测试和`test_mission_pole_integration.py`等既有测试

- [ ] **Step 8: Commit**

```bash
cd "D:/项目与工具/Python项目/Project2"
git add drone_control/circle_pole/Mission_GPT.py drone_control/circle_pole/test_circle_state_machine.py
git commit -m "feat(circle_pole): 状态机新增PATROL/CIRCLING/TO_LANDING，检测到杆塔自动环绕后前往降落点"
```

---

### Task 5: 同步到 ubuntu-pi、真机测试准备

**Files:** 无代码改动，操作步骤

- [ ] **Step 1: 本地清理`__pycache__`后整体scp到pi（新目录，非更新既有目录，例外走`scp -r`）**

```bash
find "D:/项目与工具/Python项目/Project2/drone_control/circle_pole" -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null
scp -r "D:/项目与工具/Python项目/Project2/drone_control/circle_pole" root@192.168.137.125:/home/sunrise/Desktop/FJJ/
ssh root@192.168.137.125 "chown -R sunrise:sunrise /home/sunrise/Desktop/FJJ/circle_pole"
```

- [ ] **Step 2: pi上跑一遍pytest做纯软件回归确认（不接飞控/雷达，仅验证代码在pi的Python环境下能跑）**

```bash
ssh root@192.168.137.125 "cd /home/sunrise/Desktop/FJJ/circle_pole && python3 -m pytest -v"
```

Expected: 全部PASS

- [ ] **Step 3: pi上`FJJ/.git`本地commit（不push，独立历史）**

```bash
ssh root@192.168.137.125 "cd /home/sunrise/Desktop/FJJ && git add circle_pole && git commit -m 'feat: 新增circle_pole(绕障飞行器D题阶段1单杆环绕)'"
```

- [ ] **Step 4: 真机测试前置检查清单（人工确认，不由Claude自动执行）**

在开阔空间（非网兜内小空间，避免墙角误判见设计文档"已知限制"）摆放：
- 一根杆塔，放在巡航直线（y=0）旁y偏移0.2~0.4m处，确保在1.2m雷达探测范围内
- `LANDING_POINT`当前占位值`(2.0, 0.0)`——如果实际测试场地终点不是这个坐标，先改常量再测
- 按照[[feedback_flight_test_safety_confirmation]]的约定，真实解锁/起飞前需要单独重新确认安全条件，这一步不在本实现计划自动执行范围内，需要用户当场确认后才能启动`main.py`

---

## Self-Review Notes

- **Spec覆盖**：设计文档的PATROL/CIRCLING/TO_LANDING状态机、悬停避让排除逻辑、`circle_planner.py`接口、阶段1直线巡航、降落点占位常量均已对应到Task 2-4的具体代码。颜色识别/LED/蜂鸣器/视觉降落点按设计明确排除，未纳入本计划。
- **占位符检查**：`LANDING_POINT`是有意的占位值（设计文档明确说明"需现场量出实际位置后修改"），已在Task 5 Step 4提醒；代码里没有`TBD`/`TODO`空实现。
- **类型一致性**：`generate_circle_waypoints`签名`(center_x, center_y, cur_x, cur_y, radius, n_points, direction, z)`在Task 2实现和Task 4的`_start_circling`调用处一致；`nav_mode`取值`"PATROL"/"CIRCLING"/"TO_LANDING"`在测试和实现里统一。
- **阶段2范围**：本计划只做阶段1（单杆），阶段2（双杆全场S形巡航）按设计文档只需改`router.txt`+`DRONE_POLE_TOTAL=2`环境变量，阶段1通过真机验证后另开一轮小计划处理，不在本计划内展开。
