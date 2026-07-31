# 任务二动态降落设计与实现

2026-07-31 电赛D题任务二（陆空协同动态降落）代码完成，19个单元测试通过。

- **关键发现**：方案A——T265坐标驱动+偏差变换，视觉不进控制闭环
- **影响范围**：`drone_control/competition_2026_d/task2_*.py`（5个新文件）
- **后续动作**：任务一实飞通过后进入任务二实飞验证（T-019）

## 控制架构决策

用户明确要求：**视觉不用于控制，仅用于辅助判定**。控制完全由 T265 坐标驱动。

| 维度 | 决策 | 理由 |
|------|------|------|
| 控制坐标 | 无人机 T265 vs 小车 T265（经偏差变换） | 视觉不可靠，T265 精度更高 |
| 偏差维度 | 仅水平 XY | Z 方向由激光高度计单独控制 |
| 偏差标定 | 开机实飞标定 | 两 T265 开机点固定，起飞悬停期间收集样本 |
| 小车广播 | 复用任务一通信，含坐标+速度 | 不额外增加协议 |
| 速度前馈 | 小车广播速度直接做前馈 | bumpless transfer 用当前速度初始化 FormationController |
| 视觉职责 | 不进闭环，仅门限判定 | 投放对中、下降前确认、触地辅助 |

## 任务流程（C点切换）

```
任务一 phase（复用）           C点切换                任务二 phase（动态降落）
─────────────                ─────                  ──────────────────────
路径跟随+视觉对中     SYNC_TARGET_AT_C        ACTIVATE_TRACKER（热身）
WorldDeckHeightCtrl  ──────────────→          PlatformTracker 初始化
                     要求: offset_ready       DynamicLandingController 运行
                     + car_position          水平: 点位控制悬停（不用vz）
                                             ↓ landing_gate_passed
                                             DYNAMIC_LANDING
                                             PlatformTracker → FormationController（水平）
                                             DynamicLandingController（vz 累加高度）
                                             ↓ deck_ride_complete
                                             RETAKEOFF → RETURN_H → LAND_H
```

**C点切换的 bumpless transfer**：`FormationController.reset(timestamp, current_velocity)` 用任务一当前速度初始化，避免阶跃。

## 文件结构

| 文件 | 职责 |
|------|------|
| `task2_mission.py` | Task2MissionDirector 状态机，C点前复用任务一，C点后接动态降落 |
| `task2_flight.py` | 飞行适配层，C点后激活 PlatformTracker + FormationController + DynamicLandingController |
| `task2_runtime.py` | OffsetCalibrator（开机标定 offset_HA）+ LaserContactDetector（激光触地判定） |
| `task2_start.py` | Task2StartGate，继承 Task1StartGate，接受 task_mode=2 |
| `task2_telemetry.py` | UAV_STATE/UAV_EVENT 广播，DYNAMIC_LANDING 内部状态细分 |
| `test_task2_mission.py` | 19个状态机转换单元测试 |

## 遗留项（T-020）

1. **roll/pitch 接入**：`task2_flight.py` 的 `_run_landing` 中 `roll_deg`/`pitch_deg` 当前用 0.0 占位。用途：`dynamic_landing.py` 的 `_touchdown_gate()` 触地安全判定（倾角 > 8° 不判触地）。当前少一道安全冗余，非阻塞。

2. **offset 标定实飞验证**：`OffsetCalibrator` 需验证两 T265 漂移是否在 8cm 以内。如果漂移超限，可考虑加入视觉低频校准补偿（可选优化）。

## 协议依赖

- **CAR_STATE (0x10) 扩展格式（13B）**：含 `vx_mm_s`/`vy_mm_s` 世界速度字段，任务二速度前馈依赖。2026-07-31 通信联调验证通过。
- **CAR_POSITION (0x13)**：小车世界位置，经 offset_HA 变换后喂给 PlatformTracker。
