# PID控制器 Lpid

## :memo: 本节介绍

`Lcode/Lpid.py` 封装了位置控制 PID，将目标位置与 T265 当前位姿的误差转换为速度指令，经 Lprotocol 发送给飞控。

<div class="learning-goals" markdown="1">

### :trophy: 学习目标


1. 知道 `PID` 类如何包装 `simple_pid` 库
2. 理解 `type=0`（XY位置环）和 `type=1`（Yaw角速度环）的区别
3. 记住当前调定的 PID 参数值
4. 知道调参时应该先短距离测试

</div>

---

## 控制环路

```
目标位置 → [位置PID] → 速度指令 → Lprotocol → 飞控 → 电机
                  ↑
              T265当前位姿
```

Mission_GPT 在 30ms 控制周期内调用 `get_pid()`，将输出速度写入 `comlist`，由 Lprotocol 的指令发送线程以 50Hz 频率发给飞控。

## 当前参数

| 控制环 | Kp | Ki | Kd | 输出限幅 |
|--------|-----|-----|-----|---------|
| XY位置环 (`type=0`) | 0.7 | 0.002 | 0.05 | ±40 cm/s |
| Yaw角速度环 (`type=1`) | 1.5 | 0.0 | 0.3 | ±30 °/s |

!!! tip "调参须知"
    这些参数已经过 T-016 真机测试验证。修改后务必先短距离测试，不要直接长航线飞行。详见 [PID调参指南](../../11-advanced/pid-tuning.md)。

---

## API 参考

<div class="api-class-card">
  <div class="api-class-name">class PID</div>
  <div class="api-class-desc">位置/航向 PID 控制器，封装 simple_pid 库</div>
</div>

### 构造方法

<div class="api-method">
  <div class="api-method-sig">
    <span class="type-hint">PID</span>(<span class="param-name">type</span>: int = 0, <span class="param-name">target</span>: float = 0, <span class="param-name">p</span>: float = None, <span class="param-name">i</span>: float = None, <span class="param-name">d</span>: float = None)
  </div>
  <div class="api-method-desc">根据 type 选择 XY 位置环或 Yaw 角速度环的默认参数。如显式传入 p/i/d 则覆盖默认值。</div>
</div>

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `type` | `int` | `0` | `0` = XY位置环，`1` = Yaw角速度环 |
| `target` | `float` | `0` | 初始目标值（setpoint） |
| `p` | `float` | `None` | 比例增益，None 时用类型默认值 |
| `i` | `float` | `None` | 积分增益 |
| `d` | `float` | `None` | 微分增益 |

### 控制方法

<div class="api-section-label">设定目标</div>

<div class="api-method">
  <div class="api-method-sig">
    .<span class="param-name">set_target</span>(<span class="param-name">target</span>: float) <span class="api-returns">→ None</span>
  </div>
  <div class="api-method-desc">更新 PID 设定值。Mission_GPT 在航点切换时调用此方法设置新的目标位置。</div>
</div>

<div class="api-section-label">计算输出</div>

<div class="api-method">
  <div class="api-method-sig">
    .<span class="param-name">get_pid</span>(<span class="param-name">current</span>: float) <span class="api-returns">→ float</span>
  </div>
  <div class="api-method-desc">输入当前测量值，返回 PID 控制输出。内部会更新 simple_pid 的积分项。输出已按构造时的限幅截断。</div>
</div>

| 参数 | 类型 | 说明 |
|------|------|------|
| `current` | `float` | 当前测量值（T265 位置或 Yaw 角） |

<span class="api-returns">返回</span> `float` — 速度指令（cm/s 或 °/s），已限幅

<div class="api-section-label">重置</div>

<div class="api-method">
  <div class="api-method-sig">
    .<span class="param-name">reset</span>() <span class="api-returns">→ None</span>
  </div>
  <div class="api-method-desc">清零积分项和内部状态。在状态切换（如 TAKEOFF → NAVIGATE）时调用，防止积分饱和。</div>
</div>

---

## 使用示例

```python
from Lcode.Lpid import PID

# XY 位置环：目标 x=1.0m
pid_x = PID(type=0, target=1.0)
pid_y = PID(type=0, target=0.0)
pid_z = PID(type=0, target=1.5)

# 30ms 控制周期
while navigating:
    pos = t265.get_position()  # [x, y, z]
    vx = pid_x.get_pid(pos[0])
    vy = pid_y.get_pid(pos[1])
    vz = pid_z.get_pid(pos[2])
    # 写入 comlist → Lprotocol 发送给飞控

# 切换航点
pid_x.set_target(new_x)
pid_x.reset()  # 清零旧积分
```

## 内部默认参数

| 参数 | XY位置环 | Yaw角速度环 |
|------|---------|-----------|
| `xyp` / `yawp` | 0.7 | 1.5 |
| `xyi` / `yawi` | 0.002 | 0.0 |
| `xyd` / `yawd` | 0.05 | 0.3 |
| `xylimit` / `yawlimit` | 40 | 30 |

---

## :material-help: 常见问题

??? question "为什么 Ki 这么小（0.002）？"
    无人机悬停时位置误差很小，积分项如果太大会导致超调和振荡。0.002 是经过多次实飞调定的经验值——足够消除稳态偏差，又不会在航点切换时造成积分饱和。

??? question "type=0 和 type=1 能混用吗？"
    可以但没必要。每个 PID 实例只能是一种类型，构造时固定。Mission_GPT 会创建独立的 X/Y/Z 三个位置环实例和一个 Yaw 角速度环实例，互不干扰。

??? question "reset() 应该在什么时候调用？"
    在每次航点切换、状态机阶段转换（TAKEOFF→NAVIGATE、NAVIGATE→LAND）时调用。如果不 reset，上一个目标的积分残留会叠加到新目标上，导致初始过冲。

---

← [串口协议 Lprotocol](lprotocol.md) | [航向保持 HeadingHold →](heading-hold.md)
