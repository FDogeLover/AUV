# 通信诊断脚本与联调结果

2026-07-31 通信联调验证通过，CAR_STATE 扩展格式（含 vx/vy）正常。

- **关键发现**：小车端 CAR_STATE 扩展格式（13B）已就位，任务二速度前馈 OK
- **影响范围**：`drone_control/competition_2026_d/comm_diagnostic.py`
- **后续动作**：任务一实飞时复用通信链路

## 诊断脚本

**位置**：`drone_control/competition_2026_d/comm_diagnostic.py`
**运行**：`cd /home/sunrise/Desktop/FJJ && python -u -m competition_2026_d.comm_diagnostic`

**功能**：
- 监听 `/dev/bt_serial` @ 115200 baud（不依赖飞控/T265/视觉）
- 每秒广播 UAV_READY，让小车知道无人机在线
- 自动回 ACK（CAR_START / CAR_STATE / CAR_POSITION）
- 实时解析所有帧并显示内容：
  - **CAR_START**：task_mode, session_id, config_hash
  - **CAR_STATE**：标注基础格式(9B)还是扩展格式(13B)，显示 segment/speed/heading/flags，**特别标注 vx/vy 是否存在**
  - **CAR_POSITION**：x, y, pose_age, flags
- 收到 CAR_START 后自动模拟任务一飞行（UAV_STATE/UAV_EVENT）
- Ctrl+C 退出时打印总结：每种帧的接收计数、平均频率、CAR_STATE 基础/扩展比例、任务二速度前馈是否就绪（OK/MISSING）

## 联调结果（2026-07-31）

| 帧类型 | 状态 | 详情 |
|--------|------|------|
| CAR_START | ✅ 正常 | task_mode=1, session=539414529, config_hash=0x2026D001 |
| CAR_POSITION | ✅ 正常 | x=1.500m, y=2.000m, ~7Hz, pose_age 0-31ms, flags 全有效 |
| CAR_STATE | ✅ 扩展格式(13B) | seg=UNKNOWN, speed=0, vx=0, vy=0（小车静止，值全0正常） |
| 任务二速度前馈 | ✅ OK | vx/vy 字段存在 |

## 遗留观察

1. **蓝牙丢帧严重**：seq 跳变明显（如 seq 1→2→9→...→20→27→35→59）。ACK + 重试机制能补偿关键帧（CAR_START）。

2. **CAR_STATE 频率约 3.2Hz**：比 CAR_POSITION（7Hz）低。FormationController 的 `max_estimate_age_s=0.20`（5Hz），3.2Hz 勾强够用但偏紧。小车移动时如果频率再降，可能会触发 `stale_estimate` 刹车。

3. **小车坐标恒为 (1.500, 2.000)**：小车静止，预期行为。移动后坐标应该变化，需移动时再验证 vx/vy 非零值。

## 调试历史

- **第一次诊断**：CAR_START 和 CAR_POSITION 正常，但 CAR_STATE 载荷长度为 11 字节（应为 9 或 13），解析失败。原因：小车端打包格式错误。
- **第二次诊断**：CAR_STATE 载荷长度仍为 11 字节。
- **第三次诊断**：小车端修复后，CAR_STATE 为 13 字节扩展格式，vx/vy 字段存在，任务二速度前馈 OK。
