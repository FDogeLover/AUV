# 长路径航点到达超时分析

2026-07-26 T-016 测试中发现长路径(2m 矩形)远角航点均 timeout 跳点

- **关键发现**：`arrival_timeout_max=5.0s + 1.5s hold = 6.5s` 对 2m 移动距离不够用。飞行器在 2m 距离上消耗大部分时间窗口用于移动，剩余稳定时间不足以完成 15cm 阈值内的到达确认（还需同时满足速度 <0.05m/s 和 15 帧滑动窗口确认）。若 confidence≥3 阈值收紧到 10cm 进一步加剧。
- **影响范围**：`drone_control/basic/Mission_GPT.py` 到达判定逻辑（`arrival_timeout_max`、`posthreshold_xy`、`arrival_confirm_need`）
- **后续动作**：短距/常规飞行不受限，暂不调参。若后续有长距飞行需求，考虑将 `arrival_timeout_max` 从 5.0s 增加到 8.0~10.0s

## 时间分配估算

| 阶段 | 耗时 | 说明 |
|------|------|------|
| 移动 2m | ~3-4s | 巡航速度 ~0.5m/s |
| 减速稳定 | ~1-2s | 到达目标点附近后减速 |
| 阈值确认 | ~1.5s | 15 帧 + 速度窗口 + hold |
| **总计** | **~6-7s** | 刚好超过 6.5s 阈值 |

## 分析工具 Bug 备注

原 `flight_log_analyzer.py` 计算 XY 偏差时误用 `sqrt(pos[0]²+pos[1]²)`（到原点距离）而非 `sqrt((pos[0]-tx)²+(pos[1]-ty)²)`（到目标点距离），导致 timeout 航点的偏差数据误导。已用 `tools/analyze_t016.py` 修正。
