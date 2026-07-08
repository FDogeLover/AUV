# PoleTracker 世界系重构 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 `PoleTracker`（`drone_control/basic_radar/Lcode/Lradar.py`）的多帧候选匹配从"机体系角度/距离容差"改成"世界系位置聚类"，修复飞机接近非正前方目标时方位角摆动（实测19°）导致匹配失效的问题，并用真实飞行轨迹+合成雷达数据的回放脚本验证修复有效。

**Architecture:** 新增两个纯函数 `body_to_world_xy()`/`world_to_body_angle_dist()`（互为逆变换）到 `Lradar.py`；`PoleTracker.update()` 增加位姿参数，历史窗口存世界系坐标；`confirmed_poles()` 匹配判据从机体角度/距离容差换成世界系欧氏距离。另建 `drone_control/tools/replay_pole_tracker.py`，用已同步到本机的真实飞行日志（`t` /`pos`/`t265_yaw_deg` 字段）驱动一个按实测命中率合成雷达候选的模拟器，新旧两版匹配逻辑并排跑一遍，输出对比报告。

**Tech Stack:** Python 3.14（Windows 开发机运行测试/回放脚本，无需真实硬件），`pytest`（新增开发依赖，仅用于跑本计划新增的单元测试，不是运行时依赖）。

**背景 spec：** `docs/superpowers/specs/2026-07-08-pole-tracker-world-frame-design.md`

---

## 开发环境准备

在 Windows 开发机上，本计划新增的测试用 `pytest`，且 `Lcode/Lradar.py` 顶部 `import serial`，开发机默认没装这两个包（用真实硬件时在 `basic_radar/requirements.txt` 里已经列了 `pyserial`，但只测逻辑不需要真的接雷达）。跑任何一个任务前先执行一次：

```bash
cd "drone_control/basic_radar" && pip install pytest pyserial
```

---

### Task 1: 新增机体系↔世界系互逆变换函数

**Files:**
- Modify: `drone_control/basic_radar/Lcode/Lradar.py`
- Test: `drone_control/basic_radar/test_pole_tracker.py`

- [ ] **Step 1: 写失败的测试**

创建 `drone_control/basic_radar/test_pole_tracker.py`：

```python
"""PoleTracker 世界系重构单元测试。

运行（先确保已 pip install pytest pyserial）：
    cd drone_control/basic_radar && python -m pytest test_pole_tracker.py -v
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from Lcode.Lradar import (
    radar_angle_to_body_xy,
    body_to_world_xy,
    world_to_body_angle_dist,
)


# ════════════════════ body_to_world_xy / world_to_body_angle_dist ════════════════════

class TestBodyWorldTransform:
    def test_zero_yaw_identity_shift(self):
        # yaw=0 时，世界系就是机体系平移了飞机当前位置
        bx, by = radar_angle_to_body_xy(0, 1000)  # 机体正前方1m
        wx, wy = body_to_world_xy(2.0, -1.0, 0.0, bx, by, yaw_sign=1)
        assert wx == pytest.approx(3.0, abs=1e-6)
        assert wy == pytest.approx(-1.0, abs=1e-6)

    def test_yaw_90deg_sign_positive(self):
        bx, by = radar_angle_to_body_xy(0, 1000)  # bx=1.0, by=0.0
        wx, wy = body_to_world_xy(0.0, 0.0, math.pi / 2, bx, by, yaw_sign=1)
        assert wx == pytest.approx(0.0, abs=1e-6)
        assert wy == pytest.approx(1.0, abs=1e-6)

    def test_yaw_90deg_sign_negative(self):
        # yaw_sign 翻转应该翻转旋转方向，这是给以后真机标定用的开关
        bx, by = radar_angle_to_body_xy(0, 1000)
        wx, wy = body_to_world_xy(0.0, 0.0, math.pi / 2, bx, by, yaw_sign=-1)
        assert wx == pytest.approx(0.0, abs=1e-6)
        assert wy == pytest.approx(-1.0, abs=1e-6)

    def test_round_trip_recovers_original_angle_distance(self):
        # world_to_body_angle_dist 是 body_to_world_xy 的逆变换：
        # 任意机体系候选点转到世界系、再转回来，应该拿回原始角度/距离
        angle_deg, distance_mm = 30.0, 800.0
        x_m, y_m, yaw_rad, sign = 2.0, -1.5, math.radians(40), 1
        bx, by = radar_angle_to_body_xy(angle_deg, distance_mm)
        wx, wy = body_to_world_xy(x_m, y_m, yaw_rad, bx, by, yaw_sign=sign)
        angle2, dist2 = world_to_body_angle_dist(wx, wy, x_m, y_m, yaw_rad, yaw_sign=sign)
        assert angle2 == pytest.approx(angle_deg, abs=1e-4)
        assert dist2 == pytest.approx(distance_mm, abs=1e-4)

    def test_round_trip_with_negative_sign(self):
        angle_deg, distance_mm = 200.0, 650.0
        x_m, y_m, yaw_rad, sign = -0.6, 0.07, math.radians(3.0), -1
        bx, by = radar_angle_to_body_xy(angle_deg, distance_mm)
        wx, wy = body_to_world_xy(x_m, y_m, yaw_rad, bx, by, yaw_sign=sign)
        angle2, dist2 = world_to_body_angle_dist(wx, wy, x_m, y_m, yaw_rad, yaw_sign=sign)
        assert angle2 == pytest.approx(angle_deg, abs=1e-4)
        assert dist2 == pytest.approx(distance_mm, abs=1e-4)
```

- [ ] **Step 2: 运行测试，确认失败**

```bash
cd drone_control/basic_radar && python -m pytest test_pole_tracker.py -v
```

预期：`ImportError: cannot import name 'body_to_world_xy'`（这两个函数还不存在）。

- [ ] **Step 3: 实现最小代码**

在 `drone_control/basic_radar/Lcode/Lradar.py` 里，紧跟在 `radar_angle_to_body_xy()` 函数定义之后（第34行之后）插入：

```python
def body_to_world_xy(x_m, y_m, yaw_rad, bx_m, by_m, yaw_sign=1):
    """机体系坐标(bx_m, by_m) -> 世界系坐标(x_m, y_m 为飞机当前世界系位置)。

    yaw_rad 约定：跟 t265.py 的 get_orientation()[2] / pose_data[5] 同一个量，
    符号尚未标定（t265.py 内部经过轴重映射+取反+欧拉角提取，不是标准数学CCW正角度）。
    yaw_sign 用于以后真机标定时切换旋转方向，本次实现先固定不管调用方传几都能跑，
    具体该用 +1 还是 -1 留给下次真机/台架测试确定。
    """
    yaw = yaw_sign * yaw_rad
    cy, sy = math.cos(yaw), math.sin(yaw)
    wx = x_m + bx_m * cy - by_m * sy
    wy = y_m + bx_m * sy + by_m * cy
    return wx, wy


def world_to_body_angle_dist(world_x, world_y, x_m, y_m, yaw_rad, yaw_sign=1):
    """body_to_world_xy 的逆变换：已知一个世界系点和飞机当前位姿，反推该点在机体系
    的雷达(角度,距离mm)读数——只在离线合成测试数据时用，不是雷达驱动本身需要的功能。
    """
    dx = world_x - x_m
    dy = world_y - y_m
    yaw = yaw_sign * yaw_rad
    cy, sy = math.cos(yaw), math.sin(yaw)
    bx = dx * cy + dy * sy
    by = -dx * sy + dy * cy
    angle_deg = math.degrees(math.atan2(-by, bx)) % 360.0
    distance_mm = math.hypot(bx, by) * 1000.0
    return angle_deg, distance_mm
```

- [ ] **Step 4: 运行测试，确认通过**

```bash
cd drone_control/basic_radar && python -m pytest test_pole_tracker.py -v
```

预期：5 个测试全部 PASS。

- [ ] **Step 5: 提交**

```bash
git add drone_control/basic_radar/Lcode/Lradar.py drone_control/basic_radar/test_pole_tracker.py
git commit -m "basic_radar: 新增机体系/世界系互逆坐标变换函数(PoleTracker世界系重构第一步)"
```

---

### Task 2: `PoleTracker` 改用世界系位置匹配

**Files:**
- Modify: `drone_control/basic_radar/Lcode/Lradar.py:217-301`（`PoleTracker` 类）
- Test: `drone_control/basic_radar/test_pole_tracker.py`（追加）

- [ ] **Step 1: 写失败的测试**

在 `test_pole_tracker.py` 末尾追加：

```python
# ════════════════════ PoleTracker 世界系匹配 ════════════════════

class FakeRadar:
    """模拟 Serial_radar.get_scan()，只提供 PoleTracker.update() 需要的接口。"""
    def __init__(self):
        self._scan = {}

    def set_single_point(self, angle_deg, distance_mm, intensity=100):
        self._scan = {round(angle_deg) % 360: (distance_mm, intensity)}

    def set_empty(self):
        self._scan = {}

    def get_scan(self):
        return dict(self._scan)


class TestPoleTrackerWorldFrame:
    def test_confirms_static_pole_despite_yaw_rotation(self):
        from Lcode.Lradar import PoleTracker

        pole_world = (1.0, 0.0)
        tracker = PoleTracker(window=6, min_hits=3, world_eps_m=0.2)
        radar = FakeRadar()

        # 飞机原地不动，但朝向在几次轮询之间明显变化
        for yaw_deg in (0.0, 15.0, -10.0, 20.0):
            yaw_rad = math.radians(yaw_deg)
            angle, dist_mm = world_to_body_angle_dist(
                pole_world[0], pole_world[1], 0.0, 0.0, yaw_rad, yaw_sign=1
            )
            radar.set_single_point(angle, dist_mm)
            tracker.update(radar, 0.0, 0.0, yaw_rad)

        confirmed = tracker.confirmed_poles()
        assert len(confirmed) == 1
        assert confirmed[0]["x"] == pytest.approx(pole_world[0], abs=0.05)
        assert confirmed[0]["y"] == pytest.approx(pole_world[1], abs=0.05)
        assert confirmed[0]["hits"] >= 3

    def test_confirms_static_pole_despite_off_axis_approach(self):
        """复现2026-07-07真机失败场景的简化版：飞机从(0,0)飞向(-0.6,0.065)，
        杆子在(-1.0,0.0)，不严格在正前方，方位角会随之摆动。"""
        from Lcode.Lradar import PoleTracker

        pole_world = (-1.0, 0.0)
        waypoints = [(0.0, 0.0), (-0.2, 0.02), (-0.4, 0.045), (-0.6, 0.065)]
        yaw_rad = math.radians(2.0)  # 全程yaw基本不变，跟真实数据一致

        angles = []
        for x, y in waypoints:
            angle, _ = world_to_body_angle_dist(pole_world[0], pole_world[1], x, y, yaw_rad, yaw_sign=1)
            angles.append(angle)
        # 先确认这组坐标真的复现了"方位角明显摆动"这个前提，摆动应该超过旧版4°容差
        assert max(angles) - min(angles) > 10.0

        tracker = PoleTracker(window=6, min_hits=3, world_eps_m=0.2)
        radar = FakeRadar()
        for x, y in waypoints:
            angle, dist_mm = world_to_body_angle_dist(pole_world[0], pole_world[1], x, y, yaw_rad, yaw_sign=1)
            radar.set_single_point(angle, dist_mm)
            tracker.update(radar, x, y, yaw_rad)

        confirmed = tracker.confirmed_poles()
        assert len(confirmed) == 1
        assert confirmed[0]["x"] == pytest.approx(pole_world[0], abs=0.05)
        assert confirmed[0]["y"] == pytest.approx(pole_world[1], abs=0.05)

    def test_non_repeating_noise_not_confirmed(self):
        from Lcode.Lradar import PoleTracker

        tracker = PoleTracker(window=6, min_hits=3, world_eps_m=0.2)
        radar = FakeRadar()
        # 4次轮询，每次一个互相离得很远(>world_eps_m)的候选点，模拟不重复出现的噪声
        noise_angles_dists = [(10, 900), (120, 700), (250, 850), (300, 600)]
        for angle, dist_mm in noise_angles_dists:
            radar.set_single_point(angle, dist_mm)
            tracker.update(radar, 0.0, 0.0, 0.0)

        assert tracker.confirmed_poles() == []
```

- [ ] **Step 2: 运行测试，确认失败**

```bash
cd drone_control/basic_radar && python -m pytest test_pole_tracker.py -v
```

预期：新增的3个测试 FAIL（`update()` 目前签名是 `update(self, radar)`，不接受位姿参数，会报 `TypeError`）。

- [ ] **Step 3: 实现**

修改 `drone_control/basic_radar/Lcode/Lradar.py` 里的 `PoleTracker` 类（第217-301行），整体替换为：

```python
class PoleTracker(object):
    """细杆子(绕障目标)探测器 — 空间聚类 + 多帧时间持续性组合，世界系匹配。

    背景(2026-07-07台架测试结论)：细杆子单帧扫描通常只产生1个孤立点，跟真正的噪声在
    空间特征上无法区分，`Serial_radar.get_obstacles()`(min_samples>=2起)会把它和噪声一起
    过滤掉。真正能把两者分开的是时间维度——噪声不会重复出现在同一角度附近，杆子会。

    2026-07-07真机验证发现：早期版本用机体系角度/距离容差匹配历史候选，飞机接近一个不严格
    在正前方的目标时，视线方位角会随飞机位置摆动(实测19°)，远超固定容差，导致匹配失败——
    这是纯几何效应，跟传感器/算法实现无关。本版本改成把每次轮询的候选点转换到世界系坐标
    (需要调用方传入飞机当前位置+朝向)，历史匹配比较世界系距离而不是机体系角度/距离，静止的
    杆子在世界系里位置不变，不受飞机自身运动影响。

    用法：只关心 max_range_mm 以内的目标(默认1.2m，留一点余量，实际工作距离约1m)，导航/
    搜寻循环里周期性调用 update(radar, x_m, y_m, yaw_rad)，累计到 min_hits 次命中就能从
    confirmed_poles() 里读到确认的杆子(世界系坐标)。命中率参考2026-07-07实测：70cm~90%，
    1m~70%，1.55m~10%——在1m以内工作，min_hits=3 通常几次调用内就能确认。
    """
    def __init__(self, max_range_mm=1200,
                 window=6, min_hits=3, min_intensity=60,
                 cluster_eps_m=0.15, cluster_min_samples=3,
                 world_eps_m=0.2, yaw_sign=1):
        self.max_range_mm = max_range_mm
        self.min_hits = min_hits
        self.min_intensity = min_intensity
        self.cluster_eps_m = cluster_eps_m
        self.cluster_min_samples = cluster_min_samples
        self.world_eps_m = world_eps_m
        self.yaw_sign = yaw_sign
        self._history = deque(maxlen=window)  # 每项: 本次poll识别到的候选点世界坐标列表 [(wx,wy), ...]

    def update(self, radar, x_m, y_m, yaw_rad):
        """拉一次雷达当前scan，剔除"够格当大障碍物"的聚类，剩下的孤立/小聚类点转换成
        世界系坐标记入历史窗口。x_m/y_m/yaw_rad 是飞机当前位姿(同 Mission_GPT 里
        pos[0]/pos[1]/yaw，来自 t265 的世界系位置+朝向)。返回本次识别到的候选点世界坐标列表。
        """
        scan = radar.get_scan()
        points = []
        for angle in sorted(scan.keys()):
            distance_mm, intensity = scan[angle]
            if distance_mm <= 0 or distance_mm > self.max_range_mm or intensity < self.min_intensity:
                continue
            points.append((angle, distance_mm))

        all_clusters = _cluster_points(points, eps_m=self.cluster_eps_m, min_samples=1)
        candidates = []
        for cluster in all_clusters:
            if len(cluster) >= self.cluster_min_samples:
                continue  # 大障碍物，交给 get_obstacles() 处理，不算细目标候选
            angle, distance_mm = min(cluster, key=lambda p: p[1])
            bx, by = radar_angle_to_body_xy(angle, distance_mm)
            wx, wy = body_to_world_xy(x_m, y_m, yaw_rad, bx, by, yaw_sign=self.yaw_sign)
            candidates.append((wx, wy))

        self._history.append(candidates)
        return candidates

    def confirmed_poles(self):
        """在滑动窗口内、世界系距离在 world_eps_m 以内重复出现次数 >= min_hits 的候选，
        判定为确认的杆子。返回 [{'x','y','hits'}, ...]，按距离原点由近到远排序。"""
        all_candidates = [c for frame in self._history for c in frame]
        n = len(all_candidates)
        used = [False] * n
        confirmed = []
        for i in range(n):
            if used[i]:
                continue
            x1, y1 = all_candidates[i]
            group = [(x1, y1)]
            used[i] = True
            for j in range(i + 1, n):
                if used[j]:
                    continue
                x2, y2 = all_candidates[j]
                if math.hypot(x2 - x1, y2 - y1) <= self.world_eps_m:
                    group.append((x2, y2))
                    used[j] = True
            if len(group) >= self.min_hits:
                avg_x = sum(x for x, _ in group) / len(group)
                avg_y = sum(y for _, y in group) / len(group)
                confirmed.append({"x": avg_x, "y": avg_y, "hits": len(group)})
        confirmed.sort(key=lambda o: math.hypot(o["x"], o["y"]))
        return confirmed

    def reset(self):
        self._history.clear()
```

- [ ] **Step 4: 运行测试，确认通过**

```bash
cd drone_control/basic_radar && python -m pytest test_pole_tracker.py -v
```

预期：全部 8 个测试 PASS。

- [ ] **Step 5: 提交**

```bash
git add drone_control/basic_radar/Lcode/Lradar.py drone_control/basic_radar/test_pole_tracker.py
git commit -m "basic_radar: PoleTracker匹配逻辑从机体系角度容差改为世界系位置聚类"
```

---

### Task 3: 回放脚本 — 命中率插值 + 合成候选生成器

**Files:**
- Create: `drone_control/tools/replay_pole_tracker.py`
- Test: `drone_control/tools/test_replay_pole_tracker.py`

- [ ] **Step 1: 写失败的测试**

创建 `drone_control/tools/test_replay_pole_tracker.py`：

```python
"""replay_pole_tracker.py 里合成数据生成逻辑的单元测试。

运行：cd drone_control/tools && python -m pytest test_replay_pole_tracker.py -v
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from replay_pole_tracker import hit_probability


class TestHitProbability:
    def test_known_table_points(self):
        assert hit_probability(0.70) == pytest.approx(0.90, abs=1e-6)
        assert hit_probability(1.00) == pytest.approx(0.70, abs=1e-6)
        assert hit_probability(1.55) == pytest.approx(0.10, abs=1e-6)

    def test_linear_interpolation_midpoint(self):
        # 0.70~1.00m 之间线性插值，0.85m 是中点
        assert hit_probability(0.85) == pytest.approx(0.80, abs=1e-6)

    def test_clamped_outside_known_range(self):
        assert hit_probability(0.30) == pytest.approx(0.90, abs=1e-6)  # 比0.70近，钳位到0.70的值
        assert hit_probability(2.50) == pytest.approx(0.10, abs=1e-6)  # 比1.55远，钳位到1.55的值
```

- [ ] **Step 2: 运行测试，确认失败**

```bash
cd drone_control/tools && python -m pytest test_replay_pole_tracker.py -v
```

预期：`ModuleNotFoundError: No module named 'replay_pole_tracker'`。

- [ ] **Step 3: 实现**

创建 `drone_control/tools/replay_pole_tracker.py`：

```python
"""离线回放：用2026-07-07真实杆子测试飞行的T265轨迹(pos/yaw)，按实测命中率合成雷达候选点，
对比旧版(机体系角度/距离容差)和新版(世界系位置聚类) PoleTracker 匹配逻辑能否确认出杆子。

背景见 docs/superpowers/specs/2026-07-08-pole-tracker-world-frame-design.md。
不接真实雷达/飞控，纯离线分析工具。

运行：
    cd drone_control/basic_radar && pip install pyserial   # Lradar.py 顶部 import serial 需要
    cd drone_control/tools && python replay_pole_tracker.py
"""
import json
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "basic_radar"))

from Lcode.Lradar import PoleTracker, world_to_body_angle_dist  # noqa: E402

FLIGHT_LOG = os.path.join(
    os.path.dirname(__file__),
    "test_data_20260707",
    "flight_data_pole_tracker_real_flight_20260707.jsonl.bak",
)
POLE_WORLD = (-1.0, 0.0)  # 起飞点180°方向1m处，见问题13笔记
YAW_SIGN = 1  # 尚未标定，合成数据生成和新版tracker用同一个假设值，见spec"该脚本不验证的内容"

# 2026-07-07台架实测命中率(distance_m, hit_probability)，线性插值，范围外钳位到端点值
HIT_RATE_TABLE = [(0.70, 0.90), (1.00, 0.70), (1.55, 0.10)]

ANGLE_NOISE_SIGMA_DEG = 3.0   # 模拟"混合像素"命中角度跳动
DIST_NOISE_SIGMA_MM = 20.0    # 参考台架测距误差量级(~2cm)


def hit_probability(distance_m):
    if distance_m <= HIT_RATE_TABLE[0][0]:
        return HIT_RATE_TABLE[0][1]
    if distance_m >= HIT_RATE_TABLE[-1][0]:
        return HIT_RATE_TABLE[-1][1]
    for (d0, p0), (d1, p1) in zip(HIT_RATE_TABLE, HIT_RATE_TABLE[1:]):
        if d0 <= distance_m <= d1:
            frac = (distance_m - d0) / (d1 - d0)
            return p0 + frac * (p1 - p0)
    return HIT_RATE_TABLE[-1][1]


def load_trajectory(path):
    """读取jsonl，返回 [(t, x, y, yaw_rad), ...]，只保留含 pos 字段的记录。"""
    traj = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            if "pos" not in rec:
                continue
            x, y = rec["pos"][0], rec["pos"][1]
            yaw_deg = rec.get("t265_yaw_deg", 0.0)
            traj.append((rec["t"], x, y, math.radians(yaw_deg)))
    return traj


def synthesize_candidate(x, y, yaw_rad, rng):
    """按当前位姿和已知杆子世界坐标，掷骰子决定这一帧是否产生命中，命中则返回
    带噪声的机体系候选 (angle_deg, distance_mm)，未命中返回 None。"""
    true_angle, true_dist_mm = world_to_body_angle_dist(
        POLE_WORLD[0], POLE_WORLD[1], x, y, yaw_rad, yaw_sign=YAW_SIGN
    )
    p_hit = hit_probability(true_dist_mm / 1000.0)
    if rng.random() >= p_hit:
        return None
    angle = (true_angle + rng.gauss(0, ANGLE_NOISE_SIGMA_DEG)) % 360.0
    dist_mm = max(1.0, true_dist_mm + rng.gauss(0, DIST_NOISE_SIGMA_MM))
    return angle, dist_mm


class _FrameRadar:
    """喂给 PoleTracker.update() 的一次性假雷达：只有当前这一帧的候选点。"""
    def __init__(self, candidate):
        self._scan = {}
        if candidate is not None:
            angle, dist_mm = candidate
            self._scan[round(angle) % 360] = (dist_mm, 100)

    def get_scan(self):
        return dict(self._scan)


class OldPoleTrackerSim:
    """2026-07-07台架验证版本的匹配逻辑(机体系角度/距离容差)，只为跟新版对比用，
    不是生产代码，复现自重构前的 PoleTracker.confirmed_poles()。"""
    def __init__(self, window=6, min_hits=3, angle_tol_deg=4, dist_tol_mm=150):
        self.min_hits = min_hits
        self.angle_tol_deg = angle_tol_deg
        self.dist_tol_mm = dist_tol_mm
        from collections import deque
        self._history = deque(maxlen=window)

    def update(self, candidate):
        self._history.append([candidate] if candidate is not None else [])

    def confirmed(self):
        all_candidates = [c for frame in self._history for c in frame]
        n = len(all_candidates)
        used = [False] * n
        for i in range(n):
            if used[i]:
                continue
            a1, d1 = all_candidates[i]
            group = [(a1, d1)]
            used[i] = True
            for j in range(i + 1, n):
                if used[j]:
                    continue
                a2, d2 = all_candidates[j]
                if abs(a2 - a1) <= self.angle_tol_deg and abs(d2 - d1) <= self.dist_tol_mm:
                    group.append((a2, d2))
                    used[j] = True
            if len(group) >= self.min_hits:
                return True, len(group)
        return False, 0


def run_replay(seed=42):
    rng = random.Random(seed)
    traj = load_trajectory(FLIGHT_LOG)

    old_tracker = OldPoleTrackerSim()
    new_tracker = PoleTracker(yaw_sign=YAW_SIGN)

    old_confirmed_at = None
    new_confirmed_at = None
    new_confirmed_result = None

    for idx, (t, x, y, yaw_rad) in enumerate(traj):
        candidate = synthesize_candidate(x, y, yaw_rad, rng)

        old_tracker.update(candidate)
        if old_confirmed_at is None:
            ok, hits = old_tracker.confirmed()
            if ok:
                old_confirmed_at = idx

        new_tracker.update(_FrameRadar(candidate), x, y, yaw_rad)
        if new_confirmed_at is None:
            confirmed = new_tracker.confirmed_poles()
            if confirmed:
                new_confirmed_at = idx
                new_confirmed_result = confirmed[0]

    print(f"轨迹总帧数(含pos): {len(traj)}")
    print(f"已知杆子世界坐标(假设): {POLE_WORLD}")
    print("-" * 60)
    print(f"旧版(机体系角度/距离容差): "
          f"{'第'+str(old_confirmed_at)+'帧确认' if old_confirmed_at is not None else '全程未确认'}")
    print(f"新版(世界系位置聚类): ", end="")
    if new_confirmed_at is not None:
        err_x = new_confirmed_result["x"] - POLE_WORLD[0]
        err_y = new_confirmed_result["y"] - POLE_WORLD[1]
        err_m = math.hypot(err_x, err_y)
        print(f"第{new_confirmed_at}帧确认，坐标=({new_confirmed_result['x']:.3f}, "
              f"{new_confirmed_result['y']:.3f})，误差={err_m*100:.1f}cm")
    else:
        print("全程未确认")


if __name__ == "__main__":
    run_replay()
```

- [ ] **Step 4: 运行测试，确认通过**

```bash
cd drone_control/tools && python -m pytest test_replay_pole_tracker.py -v
```

预期：3 个测试 PASS。

- [ ] **Step 5: 提交**

```bash
git add drone_control/tools/replay_pole_tracker.py drone_control/tools/test_replay_pole_tracker.py
git commit -m "tools: 新增PoleTracker回放验证脚本(命中率插值+合成候选生成器)"
```

---

### Task 4: 跑回放脚本，人工核对结果

**Files:**
- 无代码改动，只运行 Task 3 产出的脚本并核对输出

- [ ] **Step 1: 运行回放**

```bash
cd drone_control/basic_radar && pip install pyserial
cd ../tools && python replay_pole_tracker.py
```

- [ ] **Step 2: 核对输出**

预期看到类似（具体帧号/误差数值会因随机噪声略有不同，但两版结论方向应该一致）：

```
轨迹总帧数(含pos): 262
已知杆子世界坐标(假设): (-1.0, 0.0)
------------------------------------------------------------
旧版(机体系角度/距离容差): 全程未确认
新版(世界系位置聚类): 第N帧确认，坐标=(-1.0xx, 0.0xx)，误差=X.Xcm
```

**如果新版也未能确认**：检查 `ANGLE_NOISE_SIGMA_DEG`/`DIST_NOISE_SIGMA_MM`/`world_eps_m`(默认0.2m) 是不是相对彼此太紧——世界系误差主要来自角度噪声乘以距离（3°噪声在1m距离上对应约5cm横向误差，加上2cm距离噪声，0.2m阈值应该有富余），如果确实测出全程未确认，如实记录下来，不要放宽阈值凑出"预期"结果。

**如果旧版反而也确认了**：说明这次合成的轨迹/命中率参数没有充分复现19°摆动场景，检查 `POLE_WORLD` 和真实轨迹终点(-0.6, ~0.07)算出来的方位角摆动范围，跟问题13笔记描述的19°是否吻合(可以临时加一行 print 摆动范围)。

- [ ] **Step 3: 记录结论到 spec 文档**

在 `docs/superpowers/specs/2026-07-08-pole-tracker-world-frame-design.md` 末尾追加一节"回放验证结果"，写实际跑出来的数字（帧号、坐标误差、是否符合预期），不管结果是否符合预期都如实记录。

- [ ] **Step 4: 提交**

```bash
git add docs/superpowers/specs/2026-07-08-pole-tracker-world-frame-design.md
git commit -m "docs: 记录PoleTracker世界系重构回放验证结果"
```

---

## 完成后不做的事（跟 spec 一致）

- 不把 `PoleTracker` 接入 `Mission_GPT.py` 实际飞行/避障流程
- 不做 `yaw_sign` 的真机标定（下次真机/台架测试再做）
- 不改 `Serial_radar.get_obstacles()`
- 改动只在本机仓库，按现有约定改完 Python 文件后需要 scp 同步到 `ubuntu-pi`（`drone_control/basic_radar/Lcode/Lradar.py`；`tools/` 下的回放脚本是纯离线分析工具，不需要同步到板子）
