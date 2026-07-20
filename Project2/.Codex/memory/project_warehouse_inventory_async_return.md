# 立体货架盘点异步扫码、返航与降落实测结论

## 2026-07-20 架构改造

盘点扫码已从阻塞式调用改为独立 worker + 主飞行线程持续定点控制：

- `SCAN` 状态继续运行 XY PID、高度 ramp 和航向保持；
- 扫码 worker 只读相机帧并发布不可变结果，不直接执行飞行、激光、存储或状态机副作用；
- generation 三重检查隔离过期结果；
- 扫码失败由主线程消费后生成安全返航路线；
- 动态换路同步更新 `inventory_route`、`coordinator.route`、`targets`、索引、到达窗口和 PID；
- `scan_tick()` 异常会进入 `LAND`，不再静默杀死主循环线程。

## 2026-07-20 A1 真机证据：不是“视觉失败所以没有返航”

A1 单货位测试状态日志：

```text
14:18:41 进入 VERIFY_QR
14:18:49 qr_timeout
14:18:49 VERIFY_QR → FAULT → RETURN
```

飞行日志随即把目标从 A1 替换为返航点：

```text
起始位置约 (-1.7609, 0.0623, 1.43)
返航目标   (-2.65,   0.0625, 1.40)
最终位置约 (-2.6369, 0.0703, 1.40)
目标距离   0.0152 m
```

因此下列链路均已发生：扫码 worker 超时 → 结果发布 → 主线程轮询/消费 →
状态机进入 RETURN → 动态返航路线安装 → 无人机飞完第一返航航段。

**后续路线未继续的直接原因不是二维码解码失败**：第一返航航点虽然已经接近到
1.52 cm，但 precision 到达条件还包含速度窗口和停留确认，最终以 `timeout` 结束；
`Mission_GPT._advance_waypoint()` 对 return-purpose timeout 的策略是立即切到 `LAND`，
不会推进第二返航点。随后 LAND 未正常结束，约1分钟后用户中断。返航到达策略与
LAND完成/确认是两个独立于视觉解码的问题，不能归因成单一视觉故障。

## 返航规划修复

扫码点高度来自主循环传入的 `pos[2]`；正常情况下 T265 三维位置读出后，Z 会被
飞控回传的有效激光高度覆盖。激光高度在1.40m附近的厘米级波动曾生成当前位置XY
重合的原地爬升首航点。现已按5cm容差处理：接近巡航高度时直接规划水平返航；
低层1.00m货位仍先爬升至1.40m。

A1 高度1.39m或1.40m时安全返航均为：

```text
(-2.65, 0.05, 1.40)
(-2.65, 3.50, 1.40)
(-2.50, 3.50, 1.40)  LAND_APPROACH
```

## 2026-07-20 后续修复（待真机验证）

- 返航中间点强制使用cruise到达模式，复用15cm半径和连续周期确认，并始终要求Z合格；最终LAND_APPROACH保持precision。
- timeout近距离推进只允许中间点且confidence>=2、XY<=15cm、Z误差<20cm；其他情况及末点timeout原地LAND。
- `return_timeout_near`仍经过`InventoryFlightMission._advance_waypoint()`和coordinator，coordinator返回ADVANCE后索引只推进一次；返回LAND时索引不变。
- waypoint事件新增`navigation_purpose`。
- LAND新增结构化`land_start`、周期诊断、`land_exit(confirmed/python_timeout)`与`land_wait_manual(firmware_timeout_gaveup)`；周期字段包括激光高度有效性、确认计数、PWM新鲜度、gaveup和实际命令。
- 未改变unlock+pwm双条件确认，也未改变固件gaveup后保持通信、永久等待人工介入的安全语义。
- 桌面全量测试：184 passed, 1 skipped；Qoder计划与实现两轮审查通过。

## 下一步（按优先级）

1. 修复返航中间点到达策略：位置已很近但速度确认超时时应继续下一返航点；只有
   距离仍明显过大时才原地 LAND。优先考虑返航中间点使用 cruise 语义，末点保持
   precision。
2. 单独诊断 LAND：记录一键降落是否发送、`unlock_sta`、`motor_pwm_mask`、
   `land_timeout_gaveup` 和实际激光高度轨迹，解释为何进入LAND后未结束。
3. 改善二维码：ROI内 pyzbar 失败后增加一次有界 OpenCV `detectAndDecode` 回退，
   不能恢复曾耗时约17秒的全帧几何搜索。
4. 增加逐帧诊断，区分 `decode_none`、未知内容、激光点不在码内、consensus等待和
   accepted；目前最终只有 `qr_timeout`，无法判断卡在哪一层。
5. 使用真实飞行图离线验证后再调 ROI、确认帧数和货位坐标。
