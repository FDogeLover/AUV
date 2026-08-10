# 航向保持 HeadingHold

## :memo: 本节介绍

`Lcode/heading_hold.py` 是独立的外环控制器，起飞时锁定初始 Yaw 角，飞行中持续修正航向漂移。只输出整数 Yaw 角速度指令（°/s），内层角速度闭环由凌霄 IMU 负责。

<div class="learning-goals" markdown="1">

### :trophy: 学习目标


1. 理解为什么需要航向保持外环（T265 不直接控制航向）
2. 知道 `HeadingHoldConfig` 的关键参数和约束
3. 能说出 `arm()` / `update()` / `disarm()` 的调用时机
4. 理解 runaway 检测和 fault 锁存的逻辑

</div>

---

## 为什么需要航向保持

T265 提供位置但不直接控制航向。无人机飞行中由于风力、PID 不对称等原因，Yaw 角会慢慢偏转。航向保持外环通过输出 Yaw 角速度指令修正这个偏转：

```
T265 Yaw → [HeadingHoldController] → yaw_rate_cmd → Lprotocol → 飞控内环
```

## 参数与修复历史

| 参数 | 当前值 | 约束范围 | 修复历史 |
|------|--------|---------|---------|
| `kp` | 0.5 | (0, 1.0] | 原值 0.25，修正不足 |
| `deadband_deg` | 1.5° | [0.5, 5.0] | 小于死区的误差不修正 |
| `max_rate_dps` | 3°/s | [1, 3] | 原值 1°/s，修正太慢 |
| `fault_error_deg` | 8.0° | > deadband | 超过即锁定故障 |
| `runaway_growth_deg` | 15° | > 0 | 原值 3°，太小导致误触（#45） |

!!! success "已修复（#45）"
    2026-07-24 参数调优修复了 runaway 误触问题。原值 `runaway_growth_deg=3°` 太小，正常转弯就被误判为失控。调大到 15° 后真机验证通过。

---

## API 参考

### HeadingHoldConfig

<div class="api-class-card">
  <div class="api-class-name">@dataclass HeadingHoldConfig</div>
  <div class="api-class-desc">不可变配置对象，构造时校验参数合法性</div>
</div>

<div class="api-method">
  <div class="api-method-sig">
    <span class="type-hint">HeadingHoldConfig</span>.<span class="param-name">from_env</span>(<span class="param-name">environ</span>: Mapping = None) <span class="api-returns">→ HeadingHoldConfig</span>
  </div>
  <div class="api-method-desc">从环境变量构建配置。environ 为 None 时读取 os.environ。</div>
</div>

| 字段 | 类型 | 默认值 | 环境变量 | 约束 |
|------|------|--------|---------|------|
| `enabled` | `bool` | `True` | `DRONE_HEADING_HOLD` (0/1) | — |
| `kp` | `float` | `0.5` | `DRONE_HEADING_HOLD_KP` | (0, 1.0] |
| `deadband_deg` | `float` | `1.5` | `DRONE_HEADING_HOLD_DEADBAND_DEG` | [0.5, 5.0] |
| `max_rate_dps` | `int` | `3` | `DRONE_HEADING_HOLD_MAX_DPS` | [1, 3] |
| `fault_error_deg` | `float` | `8.0` | — | > deadband_deg |
| `runaway_window_s` | `float` | `1.0` | — | > 0 |
| `runaway_growth_deg` | `float` | `15.0` | — | > 0 |

!!! warning "from_env 默认值与 dataclass 默认值不同"
    `from_env()` 中 `kp` 默认读 `"0.25"`，`max_rate_dps` 默认读 `"1"`——这是 #45 修复前的旧值，保留在 `from_env()` 中未更新。Mission_GPT 使用 `HeadingHoldConfig.from_env()` 构造配置（不是无参构造），所以如果不设置 `DRONE_HEADING_HOLD_KP` 和 `DRONE_HEADING_HOLD_MAX_DPS` 环境变量，实际生效的是旧值 `kp=0.25`、`max_rate_dps=1`，而非 dataclass 默认值 0.5 和 3。

    **建议**：在板端环境变量中显式设置 `DRONE_HEADING_HOLD_KP=0.5` 和 `DRONE_HEADING_HOLD_MAX_DPS=3` 以确保使用修复后的参数。

### HeadingHoldStatus

<div class="api-class-card">
  <div class="api-class-name">@dataclass HeadingHoldStatus</div>
  <div class="api-class-desc">每次 update/arm 的返回值，包含指令和诊断信息</div>
</div>

| 字段 | 类型 | 说明 |
|------|------|------|
| `command_dps` | `int` | Yaw 角速度指令（°/s），故障或未激活时为 0 |
| `enabled` | `bool` | 配置是否启用 |
| `armed` | `bool` | 是否已激活（arm 成功） |
| `target_deg` | `Optional[float]` | 锁定的目标航向角（°） |
| `current_deg` | `Optional[float]` | 当前航向角（°） |
| `error_deg` | `Optional[float]` | 航向误差（目标 - 当前），归一化到 [-180, 180) |
| `degraded_reason` | `Optional[str]` | 降级原因（disabled/not_armed/low_confidence） |
| `fault_reason` | `Optional[str]` | 故障原因（锁定后非 None） |

### HeadingHoldController

<div class="api-class-card">
  <div class="api-class-name">class HeadingHoldController</div>
  <div class="api-class-desc">航向保持控制器主体，管理 arm/disarm/update 生命周期</div>
</div>

<div class="api-method">
  <div class="api-method-sig">
    .<span class="param-name">arm</span>(<span class="param-name">current_yaw_rad</span>: float, <span class="param-name">now</span>: float) <span class="api-returns">→ HeadingHoldStatus</span>
  </div>
  <div class="api-method-desc">锁定当前 Yaw 为目标航向。在 TAKEOFF 阶段 T265 置信度达标后调用一次。重复调用是幂等的。</div>
</div>

<div class="api-method">
  <div class="api-method-sig">
    .<span class="param-name">update</span>(<span class="param-name">current_yaw_rad</span>: float, <span class="param-name">confidence</span>: int, <span class="param-name">now</span>: float) <span class="api-returns">→ HeadingHoldStatus</span>
  </div>
  <div class="api-method-desc">每个控制周期调用，计算 Yaw 角速度指令。confidence < 2 时降级输出 0。检测到 fault 或 runaway 时锁定故障。</div>
</div>

| 参数 | 类型 | 说明 |
|------|------|------|
| `current_yaw_rad` | `float` | T265 当前 Yaw（弧度） |
| `confidence` | `int` | T265 跟踪置信度（0-3） |
| `now` | `float` | 当前 monotonic 时间戳，用于 runaway 窗口检测 |

<div class="api-method">
  <div class="api-method-sig">
    .<span class="param-name">disarm</span>(<span class="param-name">reason</span>: str) <span class="api-returns">→ None</span>
  </div>
  <div class="api-method-desc">解除激活状态，后续 update 输出 0。在 LAND 或紧急停止时调用。</div>
</div>

<div class="api-method">
  <div class="api-method-sig">
    .<span class="param-name">reset_for_new_mission</span>() <span class="api-returns">→ None</span>
  </div>
  <div class="api-method-desc">完全重置：解除激活、清空目标、清空故障和降级标记、重置 runaway 窗口。新任务前调用。</div>
</div>

---

## Runaway 检测逻辑

```
update() 每周期:
  1. 误差 > fault_error_deg (8°) → 立即锁定 fault
  2. 比例控制未饱和 (|cmd| < max_rate) → 重置 runaway 窗口
  3. 比例控制已饱和 (|cmd| == max_rate):
     a. 误差方向变化 → 重置窗口，记录起始误差
     b. 窗口未满 (runaway_window_s=1.0s) → 继续等
     c. 窗口内误差增长 ≥ runaway_growth_deg (15°) → 锁定 fault
```

!!! info "为什么 runway_growth 从 3° 调到 15°"
    原值 3° 太灵敏——无人机正常转弯时，Yaw 误差在 1 秒内增长 3° 是正常的。这导致 `#45` runaway 误触，航向保持被锁定后输出 0，无人机失控旋转。15° 给了足够的容忍空间。

---

## 使用示例

```python
from Lcode.heading_hold import HeadingHoldConfig, HeadingHoldController

config = HeadingHoldConfig()  # 使用默认参数
controller = HeadingHoldController(config)

# TAKEOFF 阶段：T265 置信度达标后锁定航向
yaw_rad = t265.get_orientation()[2]
status = controller.arm(yaw_rad, time.monotonic())

# NAVIGATE 阶段：每周期更新
while navigating:
    yaw_rad = t265.get_orientation()[2]
    conf = t265.get_tracking_confidence()
    status = controller.update(yaw_rad, conf, time.monotonic())
    if status.fault_reason:
        logger.error(f"航向故障: {status.fault_reason}")
        # 安全处理...
    yaw_rate_cmd = status.command_dps  # 写入 comlist 发给飞控

# LAND 阶段
controller.disarm("landing")
```

## 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `DRONE_HEADING_HOLD` | `1` | `0` = 关闭航向保持，输出零 Yaw 指令 |
| `DRONE_HEADING_HOLD_KP` | — | 自定义 P 增益（覆盖默认 0.5） |

---

## :material-help: 常见问题

??? question "fault 锁定后能自动恢复吗？"
    不能。一旦 `fault_reason` 被设置，后续所有 `update()` 都返回 0 指令，直到调用 `reset_for_new_mission()` 或 `disarm()`。这是安全设计——航向失控后不应自动恢复，应降落检查。

??? question "deadband_deg=1.5° 会不会导致航向慢慢漂走？"
    1.5° 的死区意味着小角度偏差不修正。但实际飞行中风力导致的漂移通常 > 1.5°，会被正常修正。1° 以内的偏差对航线飞行精度影响可忽略。

??? question "DRONE_HEADING_HOLD=0 关闭后，Yaw 指令是什么？"
    输出恒定 0°/s。飞控内环会维持当前 Yaw 角速度为零，但不会有外环修正航向漂移。仅在调试时使用。

---

← [PID控制器 Lpid](lpid.md) | [导航策略 NavigationProfile →](nav-profile.md)
