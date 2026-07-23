# 无人机稳定性重构设计文档

**日期**：2026-06-13  
**涉及模块**：`ANO_LX_FC_倾角保护版`（飞控固件）、`drone_control`（Python 上位机）  
**目标**：提升飞行稳定性，实现航点间动态高度变化

---

## 1. 背景与问题

当前系统存在三个稳定性问题：

| 问题 | 现象 | 根因 |
|---|---|---|
| Z 轴高度切换 | 航点间高度跳变，飞控超调 | Pi 直接跳变 target_z；固件 PID 无积分分离 |
| XY 轴漂移振荡 | 靠近目标点时来回抖动，到达慢 | XY PID D 项为 0，无阻尼 |
| 起飞偏航漂移 | 起飞过程小幅旋转 | takeoff() 仅 sleep(1s)，偏航无闭环 |

---

## 2. 改动边界

通信协议帧格式**不变**，改动严格限定在以下文件：

```
飞控端（C）
└── ANO_LX_FC_倾角保护版/Mycode/my_protocol.c
    └── height_set()  — 唯一改动点

Python端
├── drone_control/Mission_GPT.py
│   ├── takeoff()     — 改为闭环起飞 + 偏航稳定
│   ├── navigate()    — 加高度平滑 ramp
│   └── __init__()    — 增加 _ramp_z_cm 状态变量
└── drone_control/Lcode/Lpid.py
    └── XY PID 参数   — 加 D 项，参数可外部配置
```

`User_Task.c`、`Ano_Scheduler.c`、`Lprotocol.py` 均**不改动**。

---

## 3. 飞控端设计：`height_set()` 重写

### 3.1 问题

当前实现为增量式 PID，存在：
- 无倾斜补偿：激光斜距未转为垂直高度，水平移动时高度波动
- 无积分分离：大偏差时积分饱和导致超调
- 增量式结构：误差累积，切换目标高度时动态响应差

### 3.2 新实现：位置式 PID + 增强

```
输入：  current_h (cm, 激光测距)，target_h (cm, Pi 下发)
输出：  vel_z ∈ [-30, +30]
```

**三项增强**（代码基于已有注释中的骨架）：

**① 倾斜补偿**
```c
float rol_deg = fc_att.st_data.rol_x100 / 100.0f;
float pit_deg = fc_att.st_data.pit_x100 / 100.0f;
float tilt_deg = my_sqrt(rol_deg*rol_deg + pit_deg*pit_deg);
if (tilt_deg > 45.0f) tilt_deg = 45.0f;
float tilt_rad = tilt_deg * 0.0174533f;
height = (u32)((float)height * my_cos(tilt_rad));
```

**② 积分分离**
```c
static s16 height_integral = 0;
s16 i_term;
if (err > 200 || err < -200) {
    i_term = 0;
    height_integral = 0;  // 误差过大时清积分防饱和
} else {
    height_integral += err;
    if (height_integral >  100) height_integral =  100;
    if (height_integral < -100) height_integral = -100;
    i_term = (s16)(Ki * height_integral);
}
```

**③ 位置式输出**
```c
output = Kp * err + i_term + Kd * (err - err_last);
err_last = err;
if (output >  30) output =  30;
if (output < -30) output = -30;
```

**初始参数**：`Kp=0.8, Ki=0.05, Kd=0.2`（较原值降低 Ki，避免积分超调）

**编码约束**：文件为 GB2312 编码，不修改现有中文注释，新增代码注释使用英文。

---

## 4. Python 端设计

### 4.1 闭环起飞 + 偏航稳定（`takeoff()`）

**问题**：当前 `takeoff()` 仅 `sleep(1s)`，无高度反馈，偏航无控制。

**新流程**：

```
1. 发送 task_sta=1（触发飞控解锁 + 起飞）
2. 循环（每 30ms）：
   a. 读取 serial_fc_ref._last_laser_height_cm
   b. 偏航 PID：vyaw = yaw_pid.get_pid(realsense.get_orientation()[2])
               → 写入 se_fc[6]（偏航速度指令）
   c. Z 目标固定为 targets[0][2] * 100（第一个航点高度）
3. 高度连续 10 帧在目标 ±10cm 内 → 切换 state = "NAVIGATE"
4. 超时 15s → 强制切换（记录 warning）
```

**边界条件**：
- 激光高度有效性：`laser_h > 0.05`（5cm），与 `Lprotocol.py` 现有逻辑一致
- `yaw_pid` 复用已有实例，目标 `0`（保持起始方向）
- 偏航输出限幅 `[-30, +30]`（`Lpid.py` 已有 `yawlimit=30`）

### 4.2 高度平滑 Ramp（`navigate()`）

**问题**：`navigate()` 直接用 `target[2]*100` 作为 Z 指令，航点间跳变。

**新方案**：在 `mission.__init__()` 增加 `self._ramp_z_cm = 0.0`，每帧步进：

```python
RAMP_STEP = 1.5  # cm per frame, ~50 cm/s at 30ms cycle

# 每帧在 navigate() 中执行：
if self._ramp_z_cm < target_z - RAMP_STEP:
    self._ramp_z_cm += RAMP_STEP
elif self._ramp_z_cm > target_z + RAMP_STEP:
    self._ramp_z_cm -= RAMP_STEP
else:
    self._ramp_z_cm = target_z

self.set_speed(vx, vy, -vyaw, int(self._ramp_z_cm))
```

**关键约束**：
- 切换航点时 `_ramp_z_cm` **不重置**，保持高度连续
- 进入 `NAVIGATE` 状态时（takeoff 成功后），将 `_ramp_z_cm` 初始化为第一个航点的目标高度（`targets[0][2] * 100`），避免从 0 爬升的跳变

### 4.3 XY PID 调优（`Lpid.py`）

| 参数 | 当前 | 新值 | 原因 |
|---|---|---|---|
| `xyp` | 0.7 | 0.7 | 保持 |
| `xyi` | 0.002 | 0.002 | 保持 |
| `xyd` | 0.00 | **0.05** | 加入阻尼，抑制靠近目标时振荡 |

同时将参数提升为**构造时可传入**（`PID(type, target, p=None, i=None, d=None)`），保持默认值行为不变，方便飞行测试中调参。

---

## 5. Git 分支策略

```bash
git checkout -b refactor/stability-v1
```

- 所有改动在此分支进行
- `main` 分支保留当前可运行版本
- 改动完成、测试通过后再合并

---

## 6. 验证标准

| 项目 | 通过标准 |
|---|---|
| 飞控编译 | Keil 无新增 Error/Warning |
| 高度切换 | 航点间高度变化无明显超调（肉眼观察振荡 < 2 次） |
| 起飞偏航 | 起飞后偏航偏差 < 10° |
| XY 到达 | 到达阈值内（0.15m）稳定时间 < 3s |
| 急停保护 | 倾角保护、串口超时保护仍正常触发 |
