# 2026-07-22 立体货架盘点赛题收尾总结

## 结论

`drone_control/warehouse_inventory` 已阶段验收。最终版 A→B→C→D 完整路线
连续两轮飞完 40 航点，均正常返航、降落和上锁。最新一轮识别 24/24，
`missing_slots=[]`、`complete=true`，无 `flight_loop_exception`。

## 最终路线

- 起飞：`(0.00, 0.00, 1.25)`
- A（0°）：A1,A2,A3,A6,A5,A4，`Y=0.05`
- A→B：上端 `X=-2.80`绕行
- B（180°）：B6,B5,B4,B1,B2,B3，`Y=1.65`
- B→C：直接过渡到 `Y=2.05`
- C（0°）：C1,C2,C3,C6,C5,C4
- C→D：上端 `X=-2.80`绕行
- D（180°）：D6,D5,D4,D1,D2,D3，最终 `Y=3.45`
- 降落：`(-2.50, 3.50, 1.25) → (-2.50, 3.50, 0.20)`
- 上/下层扫码高度：`1.25/0.85 m`；全路线约 `21.15 m`

## 最终运行配置

```bash
DRONE_WAREHOUSE_MISSION_READY=1 \
DRONE_QR_DECODE_PROFILE=raw \
DRONE_QR_FOV_PRECHECK=0 \
DRONE_ASYNC_QR_SCAN=1 \
DRONE_HEADING_HOLD=1 \
DRONE_HEADING_SOURCE=t265 \
DRONE_HEADING_HOLD_MAX_DPS=3 \
DRONE_GROUND_MODE=off \
DRONE_STATE_DEBUG_LOG=1 \
DRONE_VISION_DEBUG_CAPTURE=1 \
python3 -u main.py
```

`DRONE_WAREHOUSE_MISSION_READY=1` 只是进入已验证的实飞入口，不代替每次的现场
安全确认、物理按键、T265/飞控检查和红灯 5 秒警示。

## 关键收尾修正

1. QR 实飞改为 raw-only，关闭快速 FOV 预检。
2. T265 航向误差符号修正，正常上限 3°/s；最终降落末段误差约 2°。
3. 最短完整路线固定使用两次上端绕行，D 面从 `Y=3.55` 调为 `3.45 m`。
4. 无 QR、解码失败、扫码超时和重复 QR 都跳过当前货位继续全程；激光、
   定位、飞控等硬件/安全故障仍保留降落回退。

## 验证与数据

- 本机模块全量：`230 passed, 1 skipped`
- 板端本次修正定向：`33 passed`
- 本机：`drone_control/tools/data_archive/test_data_20260722/warehouse_full_abcd_continue_on_miss_v2_*`
- 板端：`/home/sunrise/Desktop/FJJ/test_data/warehouse_inventory_20260722/`
- 视觉归档：248 张，约 19 MB

两轮使用相同的 `tee` 文件名，第二轮控制台日志覆盖第一轮；两轮完成性由现场
观察确认，归档的完整控制台和 24/24 结果来自最新一轮。`flight_data.jsonl`、
`inventory_state.jsonl` 为追加文件，已保留归档时的完整副本。

## 已知边界

- 视觉伺服未采用；板端解码延迟与飞行抖动下收益不足。
- 地面站仍是训练广播模式，不等待 ACK。
- 本赛题的工程验收不表示可取消通用降落、T265/USB 和电机上锁相关的长期
  安全注意事项。
