# 计划：视觉伺服精准降落模块（visual_servo_landing）v3

## 问题描述 & 目标

**问题**：当前 competition_2026 降落依赖 T265 坐标定位，落点精度受 T265 漂移影响，
无法保证激光笔落在目标中心 15cm 半径内。

**目标**：飞机到达目标航点上方后，切换为视觉伺服模式，用 Cyber Camera 朝下识别
地面黑色实心方块，通过 IBVS 将飞机对中至方块正上方，对中后交还 Mission_GPT 执行降落。

---

## 架构分析（基于 Mission_GPT.py 实地验证）

| 项目 | 实际情况 |
|------|----------|
| `waypoint_actions` | 纯元数据，navigate() 不等待，ACTION_COMPLETED 由计时器触发 |
| `set_speed(vx, vy, yaw, z)` | 有锁，写 se_fc 共享数组，100Hz 串口线程消费 |
| 主循环 | `loop()` 30ms tick，daemon 线程，调用 navigate()/land() 等 |
| ActionHandler 模型 | fire-and-forget，不支持阻塞等待，不适合长时闭环控制 |

**结论**：视觉伺服**不能**作为 ActionHandler，必须在 Mission_GPT 的 30ms tick 内原生执行。

---

## 方案选择

**选择方案：新增 `VISUAL_SERVO` 状态（原生 Mission_GPT 状态机）**

| 方案 | 优点 | 缺点 |
|------|------|------|
| **A. 新增 Mission_GPT 状态（推荐）** | 零线程争用；直接调 set_speed()；天然接入安全兜底 | 需修改 Mission_GPT.py（非 Lcode/） |
| B. ActionHandler 线程 + set_speed() 注入 | 无需改 Mission_GPT.py | navigate() 与 handler 线程竞争 set_speed()，虽有锁但逻辑相互覆盖 |
| C. 延长 waypoint_hold_s 作为视觉伺服窗口 | 零侵入 Mission_GPT | hold 计时不感知视觉状态，完成时机无法精确控制 |

**方案 A 理由**：
- `loop()` 每 30ms 调 navigate() 或 `VISUAL_SERVO` 分支，单线程顺序执行，无竞态
- set_speed() 在 Mission_GPT 自己的线程中调用，与100Hz 发送线程共享 se_fc 的锁已覆盖
- 视觉伺服失败 → 直接转 `LAND` 状态（现有安全降落逻辑），无需额外回退设计

---

## 状态机扩展

```
IDLE → TAKEOFF → NAVIGATE → VISUAL_SERVO → LAND → END
                     │                        ↑
                     └── (其他动作) → LAND ──┘

VISUAL_SERVO 内部：
  SEARCHING ──(5s 超时)──────────────────────────────► LAND（安全兜底）
      │
      └──(检测到方块)──► CENTERING ──(15s 超时)────────► LAND
                             │
                             └──(连续 K 帧 err < 5cm)──► LAND
```

**进入条件**：`navigate()` 确认到达航点 + `waypoint_actions[i] == "visual_servo_land"`
**退出条件**：对中成功 / 任意超时 / emergency_stop → 均转 `LAND`（Mission_GPT 现有安全降落）

---

## 改动范围

### 新增文件（不含 Lcode/）
- `drone_control/competition_2026/vision/__init__.py`
- `drone_control/competition_2026/vision/square_detector.py` — OpenCV 黑色方块检测
- `drone_control/competition_2026/vision/servo_controller.py` — tick-based IBVS 控制器
- `drone_control/competition_2026/vision/test_square_detector.py`
- `drone_control/competition_2026/vision/test_servo_controller.py`

### 修改文件
- `drone_control/competition_2026/Mission_GPT.py`
  - 构造函数增加可选参数 `video_src=None`
  - 增加 `"VISUAL_SERVO"` 状态分支至 `loop()`
  - 新增 `_visual_servo_tick(pos)` 方法（调用 VisualServoController）
  - navigate() 到达确认后检查 action，若为 `"visual_servo_land"` 转 VISUAL_SERVO 状态

### 不修改
- `Lcode/` 目录下所有文件（含 action_executor.py）

---

## 关键模块设计

### servo_controller.py — tick-based IBVS

```python
class VisualServoController:
    """
    无状态/纯 tick 接口，由 Mission_GPT 每 30ms 调用一次。
    不持有任何飞控引用，只返回速度修正量。
    """

    def tick(self, frame, altitude_m: float) -> ServoTick:
        """
        Parameters
        ----------
        frame    : np.ndarray BGR 帧，None 表示本轮无帧
        altitude_m : 当前飞行高度（米）

        Returns
        -------
        ServoTick(vx, vy, state, done, failed, reason)
        """

@dataclass
class ServoTick:
    vx_cm_s: float = 0.0   # 前后速度修正（cm/s），由 Mission_GPT 写入 set_speed()
    vy_cm_s: float = 0.0   # 左右速度修正
    state: str = "SEARCHING"
    done: bool = False      # 对中成功，可以降落
    failed: bool = False    # 超时/异常，建议直接降落（坐标兜底）
    reason: str = ""
```

### Mission_GPT.py — `_visual_servo_tick(pos)` 方法

```python
def _visual_servo_tick(self, pos):
    """每30ms由 loop() 调用一次。"""
    altitude_m = pos[2]

    # 读帧（非阻塞，最多等 5ms）
    frame = None
    if self._video_src is not None:
        frame_obj = self._video_src.read_frame(timeout_s=0.005)
        if frame_obj:
            frame = frame_obj.payload

    tick = self._vs_ctrl.tick(frame, altitude_m)

    if tick.done or tick.failed:
        reason = "centered" if tick.done else tick.reason
        logger.info("[VS] → LAND  reason=%s", reason)
        self._emit_waypoint_event(ACTION_COMPLETED if tick.done else ACTION_FAILED)
        self.state = "LAND"
        return

    # 应用速度修正（替换 navigate() 本轮的 PID 输出）
    with lock:
        current_z = self.se_fc[5]   # 保持当前高度设定不变
    self.set_speed(tick.vx_cm_s, tick.vy_cm_s, 0, current_z)
```

---

## 线程安全分析

| 操作 | 调用线程 | 安全性 |
|------|----------|--------|
| `set_speed()` | Mission_GPT loop 线程（单一） | ✅ 已有 lock |
| `se_fc` 读写 | Mission_GPT loop 线程（写） + 串口发送线程（读） | ✅ lock 已覆盖 |
| `video_src.read_frame()` | Mission_GPT loop 线程 | ✅ timeout=5ms，不阻塞主循环 |
| `VisualServoController.tick()` | Mission_GPT loop 线程（单一） | ✅ 无共享状态 |

`VISUAL_SERVO` 状态期间 navigate() **不被调用**，消除与 navigate() 的 set_speed() 竞态。

---

## 回退方案（明确）

| 触发 | 行为 |
|------|------|
| `tick.failed=True`（超时/检测异常） | emit ACTION_FAILED → `self.state = "LAND"` → 以当前 T265 坐标降落 |
| `emergency_stop=True` | loop() 现有逻辑：stop_all() → 飞控紧急降落 |
| `video_src=None`（未配置摄像头） | `_visual_servo_tick()` 中 `frame=None`，tick 返回 SEARCHING，最终超时 → LAND |

ACTION_FAILED 后是 `self.state = "LAND"`，Mission_GPT 的 `land()` 直接执行，
与 Mission_GPT 原有安全机制完全一致，无需额外回退设计。

---

## 风险点

| 风险 | 缓解措施 |
|------|----------|
| 视觉伺服期间 navigate() 的 PID 被绕过 | `VISUAL_SERVO` 状态不调用 navigate()，不存在争用 |
| `read_frame()` 阻塞影响30ms 节拍 | timeout=5ms，超时返回 None，loop 继续 |
| 检测丢失（方块离 FOV） | 连续丢帧 → SEARCHING 超时 → tick.failed → LAND |
| focal_length_px 未标定 | 启动时 sanity check：已知高度下方块像素尺寸验证 |
| 旋翼下洗吹移方块标志物 | altitude < 0.3m 时 tick 返回 done=True，停止修正 |
| video_src 资源与 AirborneVideoManager 冲突 | 使用独立 VideoSource 实例（UVC 相机支持多实例读取） |

---

## 验证方式

**离线单元测试：**
- `test_square_detector.py`：合成图像验证检测
- `test_servo_controller.py`：mock 帧序列，验证 tick 状态转换、超时、done/failed

**桌面集成测试（无飞机）：**
- mock Mission_GPT，构造测试环境，每30ms调用一次`_visual_servo_tick()`
- 验证对中收敛后 `state == "LAND"`
- 验证遮挡相机→超时→`state == "LAND"`

**真机分阶段验证：**
- 阶段1：台架验证 read_frame() 不阻塞30ms 节拍（loop 时序日志）
- 阶段2：0.8m 悬停，观察 vx/vy 修正量符号和量级（不启用实际控制）
- 阶段3：接入完整控制，1m 悬停自动对中，记录收敛时间和落点偏差
