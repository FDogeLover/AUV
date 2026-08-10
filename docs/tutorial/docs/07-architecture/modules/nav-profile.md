# 导航策略 NavigationProfile

## :memo: 本节介绍

`Lcode/navigation_profile.py` 定义航点到达判定策略，决定"什么时候算到了"。通过环境变量切换 `precision`（精确）和 `cruise`（巡航）两种模式。

<div class="learning-goals" markdown="1">

### :trophy: 学习目标


1. 理解 `precision` 和 `cruise` 两种到达判定策略的区别
2. 知道首尾航点自动降级为 precision 的原因
3. 能使用 `NavigationProfileConfig.from_env()` 从环境变量构建配置
4. 理解 `waypoint_mode()` 和 `cruise_timeout_s()` 的调用时机

</div>

---

## 两种模式

### precision（精确模式，默认）

要求滑动窗口内 **60%** 帧同时满足：

- XY 距离 ≤ 0.15m
- Z 距离 ≤ 0.20m
- T265 速度均值 ≤ 0.05 m/s

窗口大小：15 帧（约 0.45s）

### cruise（巡航模式）

进入 `cruise_radius_m` 半径内 + 连续 `cruise_confirm_cycles` 帧确认即视为到达。含进度超时检测（`cruise_timeout_base_s` + 按距离动态扩展）。

!!! info "首尾航点自动降级"
    无论全局模式设为什么，首 `precision_head` 个和尾 `precision_tail` 个航点自动使用 `precision` 模式，保证起飞和降落精度。

---

## API 参考

### NavigationProfileConfig

<div class="api-class-card">
  <div class="api-class-name">@dataclass NavigationProfileConfig</div>
  <div class="api-class-desc">不可变配置对象，构造时校验所有参数合法性</div>
</div>

<div class="api-method">
  <div class="api-method-sig">
    <span class="type-hint">NavigationProfileConfig</span>.<span class="param-name">from_env</span>(<span class="param-name">environ</span>: Mapping = None) <span class="api-returns">→ NavigationProfileConfig</span>
  </div>
  <div class="api-method-desc">从环境变量构建配置。environ 为 None 时读取 os.environ。</div>
</div>

| 字段 | 类型 | 默认值 | 环境变量 | 约束范围 |
|------|------|--------|---------|---------|
| `profile` | `str` | `"precision"` | `DRONE_NAV_PROFILE` | `precision` / `cruise` |
| `precision_head` | `int` | `1` | `DRONE_CRUISE_PRECISION_HEAD` | [1, 100] |
| `precision_tail` | `int` | `1` | `DRONE_CRUISE_PRECISION_TAIL` | [1, 100] |
| `cruise_radius_m` | `float` | `0.15` | `DRONE_CRUISE_RADIUS_M` | [0.05, 1.0] |
| `cruise_confirm_cycles` | `int` | `3` | `DRONE_CRUISE_CONFIRM_CYCLES` | [2, 20] |
| `cruise_require_z` | `bool` | `False` | `DRONE_CRUISE_REQUIRE_Z` | — |
| `cruise_timeout_base_s` | `float` | `25.0` | `DRONE_CRUISE_TIMEOUT_S` | [5.0, 300.0] |
| `cruise_min_progress_mps` | `float` | `0.20` | `DRONE_CRUISE_MIN_PROGRESS_MPS` | [0.05, 0.4] |
| `cruise_timeout_margin_s` | `float` | `5.0` | `DRONE_CRUISE_TIMEOUT_MARGIN_S` | [0.0, 60.0] |

### 方法

<div class="api-section-label">航点模式决策</div>

<div class="api-method">
  <div class="api-method-sig">
    .<span class="param-name">waypoint_mode</span>(<span class="param-name">target_index</span>: int, <span class="param-name">target_count</span>: int) <span class="api-returns">→ str</span>
  </div>
  <div class="api-method-desc">根据航点位置决定使用哪种到达策略。返回 `"precision"` 或 `"cruise"`。</div>
</div>

| 参数 | 类型 | 说明 |
|------|------|------|
| `target_index` | `int` | 当前航点索引（0-based） |
| `target_count` | `int` | 航点总数 |

<span class="api-returns">返回</span> `str` — `"precision"` 或 `"cruise"`

**决策逻辑：**

```
if profile == "precision":      → 全部航点用 precision
if target_index < precision_head: → precision（首段）
if target_index >= count - precision_tail: → precision（尾段）
else: → cruise
```

<div class="api-section-label">超时计算</div>

<div class="api-method">
  <div class="api-method-sig">
    .<span class="param-name">cruise_timeout_s</span>(<span class="param-name">segment_distance_m</span>: float) <span class="api-returns">→ float</span>
  </div>
  <div class="api-method-desc">根据航段距离动态计算超时。距离越长，允许时间越多。</div>
</div>

| 参数 | 类型 | 说明 |
|------|------|------|
| `segment_distance_m` | `float` | 当前航段距离（米） |

<span class="api-returns">返回</span> `float` — 超时秒数 = max(cruise_timeout_base_s, margin + distance / min_progress_mps)

---

## 使用示例

```python
from Lcode.navigation_profile import NavigationProfileConfig

config = NavigationProfileConfig.from_env()

# 5 个航点
waypoints = [(0,0,1), (-0.6,0,1), (-0.6,0,0.2), (0.6,0,1), (0,0,0.2)]

for i, wp in enumerate(waypoints):
    mode = config.waypoint_mode(i, len(waypoints))
    print(f"航点 {i}: {mode}")
    # 输出: 0=precision, 1=cruise, 2=cruise, 3=cruise, 4=precision

    # 计算超时
    dist = euclidean(prev_wp, wp)
    timeout = config.cruise_timeout_s(dist)
    print(f"  超时: {timeout:.1f}s")
```

## 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `DRONE_NAV_PROFILE` | `precision` | 全局到达策略 |
| `DRONE_CRUISE_RADIUS_M` | `0.15` | cruise 到达半径 (m) |
| `DRONE_CRUISE_CONFIRM_CYCLES` | `3` | cruise 确认帧数 |
| `DRONE_CRUISE_TIMEOUT_S` | `25.0` | cruise 超时基准 (s) |
| `DRONE_CRUISE_MIN_PROGRESS_MPS` | `0.20` | 最小进度速度 (m/s) |
| `DRONE_ARRIVAL_HOLD_S` | `1.5` | 到达后悬停时间 (s) |

---

## :material-help: 常见问题

??? question "cruise 模式为什么要有超时？"
    cruise 模式只看半径不看速度。如果无人机被卡住（风太大、PID 输出不足），它会停在航点附近但永远到不了精确位置。超时检测会在 `cruise_timeout_s` 秒后强制判定到达，避免任务卡死。

??? question "cruise_require_z=False 意味着不看高度？"
    是的。cruise 模式默认只看 XY 平面距离。`cruise_require_z=True` 会额外要求 Z 距离也 ≤ radius。一般不需要，因为 Z 方向由激光高度计独立控制。

??? question "precision 模式的 60% 窗口怎么理解？"
    15 帧窗口中有 9 帧（60%）同时满足 XY/Z/速度三个条件才算到达。这比单帧判定更鲁棒，能过滤 T265 偶发跳变。窗口太短容易误判，太长导致到达响应慢。

---

← [航向保持 HeadingHold](heading-hold.md) | [T265接口 →](t265.md)
