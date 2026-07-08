# PoleTracker 接入 Mission_GPT 导航流程设计

## 背景

`PoleTracker`（`drone_control/basic_radar/Lcode/Lradar.py`）已经完成世界系重构（见 [2026-07-08-pole-tracker-world-frame-design.md](2026-07-08-pole-tracker-world-frame-design.md)），回放验证证明匹配逻辑在真实飞行轨迹上比旧版更稳定。但目前代码库里没有任何地方实际调用 `PoleTracker`——雷达+杆子确认信号完全没接入实际飞行流程。

本次设计把 `PoleTracker` 接入 `drone_control/basic_radar/Mission_GPT.py` 的 `navigate()` 主循环，实现最小可用的避障反应：检测到确认的杆子且距离过近就悬停，不做绕行/重规划。

## 范围

**只做"检测+悬停"**，不做绕行、不做路径重规划。这是刻意选择的最小安全机制——`yaw_sign`（雷达世界系坐标的旋转符号）还没有真机标定，绕行这类需要精确坐标的动作现在做没有意义，先把"能不能可靠地在障碍物前停下来"这一步做扎实。

## 设计

### 1. 雷达/PoleTracker 的创建 —— 可选，环境变量控制

跟现有 `DRONE_DRY_RUN` 的可选模式风格一致，新增 `DRONE_RADAR_ENABLED`（默认 `"0"`，不开启）。`main.py`：

```python
from Lcode.Lradar import Serial_radar

port = os.getenv("DRONE_RADAR_PORT", "/dev/ttyUSB0")
baud = int(os.getenv("DRONE_RADAR_BAUD", "460800"))
radar = None
if os.getenv("DRONE_RADAR_ENABLED", "0") == "1":
    radar = Serial_radar(port, baud)
    radar.port_open()
    radar.listen_start()
    logger.info(f"雷达避障已启用，端口={port}，波特率={baud}")

mission1 = mission(re_fc, se_fc, realsense, serial_fc, radar_obj=radar)
```

`DRONE_RADAR_PORT`/`DRONE_RADAR_BAUD` 复用 `radar_bench_test.py` 已有的环境变量名和默认值，保持一致。不接雷达的现有测试（T265矫正、速度门槛验证等）不受影响，因为默认关闭。

### 2. `Mission_GPT.py` 新增常量

```python
POLE_POLL_INTERVAL_S = 0.5   # PoleTracker轮询间隔，跟07-07真机测试/回放验证用的节奏一致
POLE_DANGER_DIST_M = 0.6     # 确认的杆子距飞机当前位置小于此值就悬停(初步经验值，待真机调优)
POLE_YAW_SIGN = 1            # 未标定！CLAUDE.md已知问题13——真机/台架标定前只是假设值，
                              # 标定结果可能是+1也可能是-1，标定前这个避障功能的世界坐标可能是错的
```

### 3. `mission.__init__` 新增参数

```python
def __init__(self, re_fc, se_fc, realsense_obj=None, serial_fc_ref=None, radar_obj=None):
    ...
    self.radar = radar_obj
    self.pole_tracker = PoleTracker(yaw_sign=POLE_YAW_SIGN) if radar_obj is not None else None
    self._last_pole_poll_time = 0.0
    self._pole_hovering = False  # 用于只在状态切换时打一次日志，不是每帧刷屏
```

`from Lcode.Lradar import PoleTracker` 加到文件顶部 import。

### 4. 可测试的核心判定逻辑

拆成模块级纯函数，不依赖雷达/串口，方便单元测试：

```python
def nearest_confirmed_pole_dist(confirmed_poles, x, y):
    """confirmed_poles: PoleTracker.confirmed_poles()的返回值(list of {'x','y','hits'})。
    返回离(x,y)最近的确认杆子的距离(m)；没有杆子返回None。"""
    if not confirmed_poles:
        return None
    return min(math.hypot(p["x"] - x, p["y"] - y) for p in confirmed_poles)
```

### 5. `navigate()` 集成点

在 `navigate()` 开头、`target_index` 越界检查之后，`confidence == 0` 悬停分支之前，插入雷达轮询+悬停判断：

```python
pole_hover = False
if self.pole_tracker is not None:
    now = time.time()
    if now - self._last_pole_poll_time >= POLE_POLL_INTERVAL_S:
        self._last_pole_poll_time = now
        self.pole_tracker.update(self.radar, pos[0], pos[1], yaw)
    dist = nearest_confirmed_pole_dist(self.pole_tracker.confirmed_poles(), pos[0], pos[1])
    if dist is not None and dist < POLE_DANGER_DIST_M:
        pole_hover = True

if pole_hover:
    if not self._pole_hovering:
        logger.warning(f"检测到杆子距离{dist:.2f}m，悬停等待")
        self._pole_hovering = True
    self.set_speed(0, 0, 0, int(self._ramp_z_cm))
    return
elif self._pole_hovering:
    logger.info("杆子确认已消失，恢复导航")
    self._pole_hovering = False
```

`update()` 按 `POLE_POLL_INTERVAL_S` 节流（30ms主循环里不是每帧都轮询雷达），但 `confirmed_poles()` 每帧都查（便宜，纯内存计算，没有IO）。悬停判断放在到达检测/日志之前，命中就直接 `return`，不执行本帧剩余的PID/到达判断/日志逻辑。

### 6. 恢复语义

一旦悬停，飞机停住不动。真实存在的杆子会持续被雷达确认（`confirmed_poles()` 不会因为飞机静止就自动变空），需要人工遥控接管处理——这是刻意的保守设计，不设超时强制恢复。只有误报（噪声偶然凑够`min_hits`确认、之后不再重复出现）会让 `confirmed_poles()` 真正变空，从而自动恢复导航。（已跟用户确认过这个语义，见对话记录。）

### 7. 飞行日志新增字段

`navigate()` 现有的 `_log_file.write(json.dumps({...}))` 里加一个字段：

```python
"pole_hover": self._pole_hovering,
```

方便事后复盘"这段时间是不是在避障悬停"。

## 测试

- `nearest_confirmed_pole_dist()` 是纯函数，直接单元测试：空列表返回None、单个/多个杆子返回最近距离、边界值（距离刚好等于阈值）。
- `pole_hover` 状态切换逻辑（进入/离开悬停各打一次日志、`set_speed`被正确调用）用一个简化的 `mission` 实例 + fake `PoleTracker`（不需要真雷达）测试，验证：a) 无雷达(`radar_obj=None`)时完全不受影响；b) 雷达轮询按间隔节流；c) 悬停时`set_speed(0,0,0,...)`被调用且跳过PID；d) 悬停解除后日志/状态正确复位。

## 不在本次范围内

- 不做绕行/路径重规划（yaw_sign 未标定，绕行没有意义）
- 不做 `yaw_sign` 真机标定（独立任务，需要真机操作）
- 不加超时强制恢复导航的机制（刻意保守，真实障碍物应该一直等人工处理）
- 不修改 `original/` 全功能版的 `Mission_GPT.py`（`PoleTracker`/雷达是 `basic_radar/` 特有的模块，`original/` 没有这些依赖）
