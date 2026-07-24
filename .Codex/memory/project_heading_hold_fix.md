---
name: project-heading-hold-fix
description: 航向保持 runaway_growth_deg 阈值调优，修复正常 T265 漂移误触
metadata: 
  node_type: memory
  type: project
  originSessionId: sess_444a2a84-dc30-4300-83d4-a618a8ba533e
---

# 航向保持参数调优（2026-07-24）

## 问题

HeadingHoldController 默认参数导致 T265 正常 yaw 漂移(~0.3°/s)误触 `runaway_detected()` 锁存：
- `kp=0.25` + 死区 1.5° + `max_rate_dps=1` → 修正力不足，误差缓慢累积
- `runaway_growth_deg=3.0` → 1 秒内误差增长 ≥3° 就触发永久锁存
- 锁存后 `yaw_cmd_sent=0`，航向完全开环

## 修复

| 参数 | 旧值 | 新值 | 说明 |
|------|:----:|:----:|------|
| kp | 0.25 | **0.5** | 修正力翻倍（上限从 0.5 升到 1.0） |
| max_rate_dps | 1 | **3** | 最大修正角速度 3 倍（上限从 3 升到...仍是 3）|
| runaway_growth_deg | 3.0 | **15.0** | 大幅放宽，正常漂移不会触发 |

## 关键逻辑

`_runaway_detected()` 仅在**指令已达 `max_rate_dps` 上限**时才激活——即控制器已经输出最大修正力但误差仍在增长时才判定为失控。不是误差缓慢累积就触发。

## 真机验证

- 2026-07-24 矩形路径 + 复合下降：三次飞行 `heading_fault_reason` 均为 null
- 全程 Yaw 漂移仅 +1.5°（20 秒飞行），此前每次飞行都触发故障锁存
- 1m 矩形路径各角到达精度 1~9cm

## 环境变量

| 变量 | 默认 | 范围 |
|------|:----:|:----:|
| DRONE_HEADING_HOLD | 1 | 0/1 |
| DRONE_HEADING_HOLD_KP | 0.5 | (0, 1.0] |
| DRONE_HEADING_HOLD_DEADBAND_DEG | 1.5 | [0.5, 5.0] |
| DRONE_HEADING_HOLD_MAX_DPS | 3 | [1, 3] |

`runaway_growth_deg` 当前未暴露为环境变量，需改源码。
