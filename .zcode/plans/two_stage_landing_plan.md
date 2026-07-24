# 计划：两级降落可靠性改进（修订版）

## 问题描述 & 目标

**问题**：当前 `land()` 依赖 `OneKey_Land()` CMD 通道触发飞控内部降落序列，该通道可能被其他指令撞车导致降落指令被静默丢弃（`known_issues.md` 问题7），且近地时 T265 位置反馈漂移可能导致落地偏移。

**目标**：用两级下降替代单一 `land()`，使每次降落都可靠贴地后锁桨。

## 方案选择

| 方案 | 优点 | 缺点/风险 |
|------|------|----------|
| **方案A：两级下降（选中）** | ①最后一段水平开环，不依赖 T265 位置反馈，不受近地漂移影响 ② 垂直速度可控（-15~-20 cm/s） ③ 不经过 CMD 通道，不跟 `dt.wait_ck` 冲突 | 需新增 `DESCEND` 状态，改动状态机 |
| 方案B：纯固件侧修复 OneKey_Land | 已有多次修复但仍偶发不可靠，根本的 CMD 撞车问题在固件侧不可彻底解决 | 依赖 IMU 内部降落序列，不可控 |

**选择方案A，理由**：从飞控固件源码可知，`navigate()` 走的实时控制帧（`dt.fun[0x41]`）与 IMU CMD 通道完全独立，不存在撞车。最后一段水平开环（不依赖 T265 反馈）是消除近地漂移风险的最直接手段。

## Qoder 审查采纳情况

| ID | 等级 | 问题 | 处理 |
|----|------|------|------|
| H1 | 🔴 | `set_speed()` 的 `z` 是绝对高度，不是速度 | ✅ 改用 ramp 递减高度设定值 |
| H2 | 🔴 | `laser_height_valid()` 过滤 <5cm，锁桨条件永远无法满足 | ✅ DESCEND 阶段用独立的贴地检测逻辑 |
| H3 | 🔴 | `FC_Lock()` 仍走 CMD 通道受 `dt.wait_ck` 阻塞 | ✅ LAND 阶段循环重试，且固件有近地强制锁定的 PWM 直写兜底 |
| M1 | 🟡 | 超时转 END 会关串口，与"保持悬停等人工介入"矛盾 | ✅ 新增 `HOVER_WAIT` 状态替代 END |
| M2 | 🟡 | DESCEND 入口缺水平到位确认 | ✅ 增加"最后航点且已到达确认"前置条件 |
| M3 | 🟡 | 固件近地锁定可能先于 Python 触发 | ✅ DESCEND 循环正确处理意外 `unlock_sta==0` |

## 改动范围

### 影响文件

| 文件 | 改动说明 |
|------|----------|
| `drone_control/basic/Mission_GPT.py` | 新增 `DESCEND`、`HOVER_WAIT` 状态及处理逻辑；修改状态机流转；`land()` 改为循环发 `FC_Lock`；调整贴地检测逻辑 |
| `router.txt`（文档说明） | 最后一个航点的 Z 设 0 表示"降落到此点后启动垂降" |

### 状态机变更

```
当前: IDLE → TAKEOFF → NAVIGATE → LAND → END
                                   ↑ OneKey_Land CMD (不可靠)

变更后: IDLE → TAKEOFF → NAVIGATE → DESCEND → LAND → END
                  位置闭环+T265   水平开环垂降   FC_Lock  正常结束
                                              ↕ 超时
                                           HOVER_WAIT (等人工介入)
```

### 新增常量

```python
DESCEND_SAFE_HEIGHT_CM = 20       # 安全分界线高度（实测可靠）
DESCEND_RAMP_STEP = 0.45           # ramp 步长 cm/30ms ≈ 15 cm/s
DESCEND_LOCK_HEIGHT_CM = 4         # 锁桨触发高度阈值（比静态6cm留余量）
DESCEND_CONFIRM_COUNT = 10         # 锁桨高度去抖帧数
DESCEND_TIMEOUT_S = 20.0            # 垂降最长等待时间
LAND_LOCK_RETRY_INTERVAL = 0.5     # FC_Lock 重试间隔（秒）
```

### 关键修正：H1 — 用 ramp 替代直接速度

`descend()` 不再用 `set_speed(0,0,0,-15)`，而是**缓慢递减高度设定值**到 0：

```python
def descend(self, pos):
    """两级下降第二阶段：水平开环，ramp 递减高度到 0"""
    
    # 不再用 set_speed(0,0,0,-15) —— 那是绝对高度，不是速度
    # 改用 ramp 缓慢递减高度设定值
    target_z = 0  # 目标高度 0
    
    # 与 navigate() 同样的 ramp 机制，但步长固定为 ~15 cm/s
    if self._ramp_z_cm > DESCEND_RAMP_STEP:
        self._ramp_z_cm -= DESCEND_RAMP_STEP
    else:
        self._ramp_z_cm = 0
    
    # 水平开环，只维持航向
    self.set_speed(0, 0, yaw_cmd, int(self._ramp_z_cm))
```

### 关键修正：H2 — 独立的贴地检测

`descend()` 中不再用 `laser_height_valid()`（其下限 5cm 会过滤掉贴地读数），改用专门的贴地检测：

```python
def _is_near_ground(self, laser_cm):
    """DESCEND 专用的贴地检测，放宽有效性下限。
    已知静态贴地激光读数约 6cm。"""
    if laser_cm <= 0:
        return False  # 传感器失效
    if laser_cm > 50:
        return False  # 错误码
    return laser_cm < DESCEND_LOCK_HEIGHT_CM  # < 4cm
```

### 关键修正：H3 — FC_Lock 循环重试

`land()` 不再只发一次 `se_fc[7]=101`，而是循环重试直到确认：

```python
def land(self):
    """降落锁定：循环发 FC_Lock + 等待双确认，不再依赖 OneKey_Land。"""
    t_start = time.time()
    unlock_confirm_count = 0
    
    while True:
        # 循环发 FC_Lock 指令（解决 CMD 通道可能被占用的问题）
        with lock:
            self.se_fc[7] = 101
        
        # 持续清零速度指令
        self.set_speed(0, 0, 0, 0)
        
        # 读确认状态
        with lock:
            unlock_sta = self.re_fc[5]
            motor_pwm_mask = self.serial_fc_ref.debug_data.get("motor_pwm_mask")
        
        # 双确认去抖
        motor_pwm_ok = motor_pwm_mask is None or motor_pwm_mask == 0
        if unlock_sta == 0 and motor_pwm_ok:
            unlock_confirm_count += 1
            if unlock_confirm_count >= LAND_UNLOCK_CONFIRM_COUNT:
                logger.info("降落确认：已上锁")
                self.state = "END"
                return
        else:
            unlock_confirm_count = 0
        
        # 超时 → 转 HOVER_WAIT 等人工介入
        if time.time() - t_start >= LAND_CONFIRM_TIMEOUT_S:
            logger.warning("降落锁定超时，转 HOVER_WAIT 等待人工介入")
            self.state = "HOVER_WAIT"
            return
        
        time.sleep(0.03)
```

### 关键修正：M1 — 新增 HOVER_WAIT 状态

```python
def hover_wait(self):
    """超时后的悬停等待状态：保持串口在线，让固件维持悬停。
    不主动锁桨，等待人工遥控器介入。"""
    self.set_speed(0, 0, 0, self._ramp_z_cm)  # 维持当前高度
    # 不退出，不关串口 - 让固件能持续收到 T265 速度参考
```

### 关键修正：M2 — DESCEND 入口加水平到位判断

```python
# 在 navigate() 中：
if (self.target_index == len(self.targets) - 1 
    and self.arrival_confirmed_time is not None  # 已确认到达
    and pos[2] <= DESCEND_SAFE_HEIGHT_CM / 100):
    self.state = "DESCEND"
    return
```

### 关键修正：M3 — 处理固件抢先锁定

```python
def descend(self, pos):
    """..."""
    # 检查固件是否已经自动锁了（近地强制锁定）
    with lock:
        unlock_sta = self.re_fc[5]
    if unlock_sta == 0:
        logger.info("固件已抢先锁定，直接转 LAND 确认")
        self.state = "LAND"
        return
    
    # ... 正常 descend 逻辑
```

## 状态机完整调度

```python
# loop() 中：
if self.state == "TAKEOFF":
    self.takeoff()
elif self.state == "NAVIGATE":
    self.navigate(pos, yaw)
elif self.state == "DESCEND":
    self.descend(pos)
elif self.state == "LAND":
    self.land()
elif self.state == "HOVER_WAIT":
    self.hover_wait()  # 不推进状态，一直悬停等人工
elif self.state == "END":
    self.stop_all()
```

## 飞行日志增强

`DESCEND` 和 `HOVER_WAIT` 状态都要记录：

```python
log_fields = {
    "state": self.state,
    "pos": [x, y, laser_h],
    "ramp_z_cm": self._ramp_z_cm,
    "laser_raw_cm": laser_cm,        # 原始激光值，不过滤
    "descend_confirm": confirm_count,
    "elapsed_s": elapsed,
    "unlock_sta": unlock_sta,
    "t265_confidence": confidence,
}
```

## 风险点

| 风险 | 概率 | 影响 | 应对 |
|------|------|------|------|
| 激光静态贴地读数~6cm，4cm 阈值可能永远达不到 | 中 | 一直处于 DESCEND 无法转入 LAND | 超时 20s → HOVER_WAIT 等人工介入；或根据实测调整阈值 |
| 激光在 <5cm 盲区读数跳变 | 中 | 误触发贴地确认 | 10 帧去抖（~300ms） |
| 地面效应导致下降率变慢 | 低 | 垂降时间长 | 20s 超时足够 |
| `FC_Lock` CMD 通道持续被占用 | 低 | LAND 状态下一直无法确认 | 固件近地强制锁定会在 1 秒后直接清零 PWM 作为兜底 |
| PID 积分在 20cm 切换时有残余速度 | 低 | 进入 DESCEND 瞬间高度跳变 | DESCEND 入口重置 ramp 到当前实际高度 |

## 验证方式

### 桌面测试
- 运行现有 pytest 确保回归通过
- 手动 Mock 测试 DESCEND → LAND → HOVER_WAIT 状态流转

### 台架测试（拆桨）
- 手持测试：手动将激光传感器靠近地面，观察日志中状态机切换
- 验证 `se_fc[7]=101` 循环发送后 `unlock_sta` 响应

### 真机验证
- **低风险先导**：router.txt 写单航点 `(0, 0, 0.30)`，观察高于安全线的 navigate 下降
- **核心验证**：router.txt `(0, 0, 0)`，观察：
  - 20cm 切换到 DESCEND
  - DESCEND 阶段水平位置稳定
  - 贴地后锁桨成功
- **重复验证**：至少连续 3 次成功

## 回退方案

1. **代码回退**：`git checkout -- drone_control/basic/Mission_GPT.py`
2. **运行时回退**：遥控器随时切回纯手动
3. **router.txt 配置**：最后航点 Z > 0.20 则不触发 DESCEND，退回到旧流程
