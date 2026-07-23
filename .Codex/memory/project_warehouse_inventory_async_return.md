# 立体货架盘点异步扫码、返航与降落实测结论

## 2026-07-20 架构改造

盘点扫码已从阻塞式调用改为独立 worker + 主飞行线程持续定点控制：

- `SCAN` 状态继续运行 XY PID、高度 ramp 和航向保持；
- 扫码 worker 只读相机帧并发布不可变结果，不直接执行飞行、激光、存储或状态机副作用；
- generation 三重检查隔离过期结果；
- 扫码失败由主线程消费后生成安全返航路线；
- 动态换路同步更新 `inventory_route`、`coordinator.route`、`targets`、索引、到达窗口和 PID；
- `scan_tick()` 异常会进入 `LAND`，不再静默杀死主循环线程。

## 返航修复历程

| 轮次 | 问题 | 修复 |
|------|------|------|
| 1 | 扫码阻塞主控制 | 异步worker+SCAN定点控制 |
| 2 | 扫码失败无安全路径 | `plan_safe_return()` |
| 3 | `coordinator.route`未同步 | `replace_inventory_navigation_route()`同步路线 |
| 4 | 激光高度抖动制造冗余首航点 | 5cm高度容差 |
| 5 | precision速度窗口卡住返航点 | 中间点cruise到达 |
| 6 | RETURN→TRANSIT非法转移 | 保持RETURN不转TRANSIT |
| 7 | 飞行线程异常静默死亡 | loop顶层traceback+清零XY+LAND |
| 8 | LAND诊断不明确 | land_start/land_exit/land_wait_manual事件 |

## 2026-07-20 实飞最终验证

三次A1实飞（高度1.25m）均完整走通返航链路：

```text
qr_timeout → FAULT → RETURN
→ 首返航点(-2.65, ...) cruise_arrival
→ 第二返航点(-2.65, 3.50, ...) cruise_arrival
→ LAND_APPROACH(-2.50, 3.50, ...) precision_arrival
→ LAND → 降落确认 ✓
```

其中一次（低电量）A1 qr_timeout后heading fault触发紧急LAND，其余两次返航全流程正常。

## LAND诊断字段

每次降落周期日志包含：`land_elapsed_s`、`laser_height_m/valid`、`unlock_confirm_count`、`motor_pwm_mask/age_s/ok`、`land_timeout_gaveup`、`task_command`、`z_command_cm`、`yaw_cmd_sent`

## 当前状态

- ✅ 返航安全链路已验证通过
- ✅ QR解码在1.25m高度通过，共识门槛已降为window=3/required=2
- ✅ 二维码解码统计已加入ScanResult.decode_stats
- ⏸ 视觉伺服因板端性能限制+飞行抖动暂不启用，激光待物理调正
- 🟡 A面完整6点飞行待复飞（上次因共识门槛过高未过A2~A6）

## 后续

1. 物理调正激光
2. A面6点复飞验证降低后的共识门槛
3. 逐货位微调扫码坐标（如果需要）
4. K230作为后续赛题升级路径

## 2026-07-22 收尾更新（替代上述业务策略）

早期“任一扫码失败立即安全返航”已在完整盘点阶段改为分类处理：

- 货位内容级失败（无 QR、解码失败、`qr_timeout`、`qr_duplicate`）返回
  `ADVANCE`，不入库并继续完整航线；
- 激光、飞控、T265/定位等硬件或飞行安全故障仍保留 RETURN/LAND 保护；
- 2026-07-22 首轮完整路线曾因 D2 `qr_duplicate` 走“空返航路线→降落”，
  修正后本机 `230 passed, 1 skipped`，板端定向 `33 passed`；
- 修正版连续两轮完成 40 航点并正常降落，最新一轮 `24/24`。

这不否定早期返航链路的安全验证价值；返航链路仍是硬故障回退，只是不再用于
可容忍的单货位内容失败。本文前面的“当前状态/后续”是 7 月 20 日历史记录，以本节为准。
