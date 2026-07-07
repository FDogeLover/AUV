# PoleTracker 世界系重构设计

## 背景

`drone_control/basic_radar/Lcode/Lradar.py` 里的 `PoleTracker`（2026-07-07 新增，台架验证有效）用滑动窗口 + 机体系角度/距离容差（`angle_tol_deg=4°`, `dist_tol_mm=150`）来判断"多帧里反复出现在同一位置的孤立候选点"是否是一根真实杆子。

2026-07-07 真机飞行验证（`router.txt` 目标 `-0.6,0,1.0`，杆子摆在起飞点180°方向1m处）发现该算法未能确认出真实存在的杆子。反算飞行轨迹（起飞点方位角≈180° → 飞到 X=-0.57/Y=0.065 时方位角≈161°）得出：飞机接近一个不严格在正前方延长线上的目标时，视线方位角会摆动，这次实测摆动了约19°，远超 `angle_tol_deg=4°` 的固定容差。这是纯几何效应（飞机对目标的相对方位角必然随飞机自身位置移动而变化），跟传感器质量或算法实现细节无关，简单调大容差治标不治本（会严重牺牲抗噪声能力）。

修复方向：候选点的历史持续性匹配不应该在"机体系角度"上做比较，而应该转换到世界系坐标（利用飞机自身位置+朝向）再比较——静止的杆子在世界系里位置不变，不管飞机怎么移动/转向。

## 范围

**只重构 `PoleTracker` 算法本身**，不涉及把它接入 `Mission_GPT.py` 的实际导航/避障决策流程（那是独立的后续设计）。当前代码库里没有任何地方调用 `PoleTracker`，重构接口不需要考虑向后兼容。

## 设计

### 1. `PoleTracker` 接口变更

```python
class PoleTracker:
    def __init__(self, max_range_mm=1200,
                 window=6, min_hits=3, min_intensity=60,
                 cluster_eps_m=0.15, cluster_min_samples=3,
                 world_eps_m=0.2, yaw_sign=1):
        ...

    def update(self, radar, x_m, y_m, yaw_rad):
        """x_m, y_m: 飞机当前世界系位置（同 Mission_GPT 里 pos[0]/pos[1]，来自 t265.get_position()）。
        yaw_rad: 飞机当前朝向（同 t265.get_orientation()[2] / pose_data[5]）。

        注意：yaw_rad 的符号约定尚未标定（t265.py 内部经过多层轴重映射+取反+欧拉角提取，
        不是标准数学 CCW 正角度），本次实现只留 yaw_sign 参数做正负切换，实际该用哪个符号
        需要下次真机/台架测试用已知方位目标标定（参照 debug_coordinate.py 当年标定 T265
        坐标系的方法：固定目标、转动机体、看哪个符号让算出来的世界坐标基本不动）。
        """
        ...
```

不再支持不传位姿的旧调用方式。

### 2. 机体系 → 世界系转换

```python
wx = x_m + yaw_sign * (bx * cos(yaw_rad) - by * sin(yaw_rad))
wy = y_m + yaw_sign * (bx * sin(yaw_rad) + by * cos(yaw_rad))
```

其中 `(bx, by) = radar_angle_to_body_xy(angle, distance)`（沿用现有函数，机体系0°=+X机体方向）。`update()` 内部照旧先做"聚类过滤大障碍物、留下孤立候选"这一步（不变），拿到候选点后立刻做这个坐标转换，历史窗口（`self._history`，`deque(maxlen=window)`）存的是世界系 `(wx, wy)` 而不是机体系 `(angle, distance)`。

### 3. `confirmed_poles()` 匹配逻辑

沿用现有的"两两比较、凑够 `min_hits` 个归为一组"逻辑（`O(n²)`，`used[]` 标记法不变），只把分组判据从

```python
abs(a2 - a1) <= angle_tol_deg and abs(d2 - d1) <= dist_tol_mm
```

换成

```python
math.hypot(wx2 - wx1, wy2 - wy1) <= world_eps_m
```

确认后的簇取平均世界坐标，返回字段：

```python
{"x": avg_wx, "y": avg_wy, "hits": len(group)}
```

（不再需要 `angle_deg`/`distance_mm` 字段本身用于确认逻辑，但可以从平均世界坐标反算等效角度/距离供日志展示——保留字段名，值改成由平均世界坐标反推。）

### 4. 离线回放验证脚本

新建 `drone_control/tools/replay_pole_tracker.py`。

**输入**：`drone_control/tools/test_data_20260707/flight_data_pole_tracker_real_flight_20260707.jsonl.bak`（2026-07-07 真实杆子测试飞行的完整日志，含每帧真实 `pos`/`t265_yaw_deg`；已从 `ubuntu-pi` 同步到本机，原文件是板载 `basic/flight_data.jsonl`，测试时雷达轮询是独立进程，没有把点云写进这份日志）。

**已知杆子世界坐标**（假设值，来自问题13笔记"起飞点180°方向1m处"）：`(-1.0, 0.0)`。

**每帧合成候选点**：
1. 用该帧真实 `pos` 算出飞机到杆子的真实距离 `d_true`。
2. 按 2026-07-07 台架命中率实测（70cm→90%，1m→70%，1.55m→10%，线性插值，超出范围外推裁剪到[0,1]）算出命中概率，掷骰子决定这帧是否产生候选。
3. 命中则生成机体系候选点：真实机体角度（由真实pos/yaw和杆子世界坐标反算）+ 角度高斯噪声(σ≈3°，模拟"混合像素"命中角度跳动)，真实距离 + 距离高斯噪声(σ≈2cm，参考台架测距误差量级)。
4. 未命中不产生候选（不额外模拟幽灵噪声点——现有聚类阶段已经会产生偶发孤立噪声候选，这次不重复引入）。

**双跑对比**：同一份合成候选流，分别喂给：
- **旧版逻辑**（原始机体系角度/距离容差匹配，直接用第3步生成时的机体角度/距离，不做任何世界系转换）
- **新版 `PoleTracker`**（本次重构，用该帧真实 `pos`/`yaw` 做世界系转换）

**输出**：文本报告，包含每一版本是否 confirm、confirm 发生在第几帧/第几秒、confirm 时的坐标（旧版无坐标概念，只报告是否confirm；新版报告世界坐标误差 vs 已知杆子位置 `(-1.0, 0.0)`）。

**验证目标**：旧版在这条真实轨迹上应该复现"未能confirm"（或confirm很晚/很不稳定），新版应该能稳定confirm且坐标误差在合理范围（几厘米到十几厘米级，跟T265本身漂移+雷达测距误差量级相当）。

**该脚本不验证的内容**：`yaw_sign` 符号是否匹配物理现实（见上文接口设计的说明，符号标定留到下次真机/台架测试）。

## 不在本次范围内

- 不把 `PoleTracker` 接入 `Mission_GPT.py` 实际飞行/避障流程
- 不做 `yaw_sign` 的实测标定
- 不修改 `Serial_radar.get_obstacles()`（大障碍物聚类）的逻辑
