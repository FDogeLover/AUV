# 空地协同（D题）

## 概述

参考 `competition_2026_d/`。通过 DCP v1 二进制协议与小车蓝牙通信，实现双T265坐标校准、动态伴飞、移动平台降落、舵机投放。

## 系统架构

```
Cyber Camera --VS1观测--> Pi/RDK --串口--> STM32飞控
Pi <--蓝牙(/dev/bt_serial)--> 小车
无人机/小车 --> 地面站（只读）
```

## DCP v1 通信协议

0xAA头 + 版本 + 类型 + CRC16 的二进制协议。CAR_STATE扩展为13字节（含vx/vy速度前馈）。

详细文档：`docs/competition_2026_d/communication_protocol.md`

## 关键模块

| 文件 | 职责 |
|------|------|
| `air_ground_link.py` | 空地通信链路（DCP协议） |
| `formation_controller.py` | 编队控制器（动态伴飞） |
| `platform_tracker.py` | 平台跟踪 |
| `dynamic_landing.py` | 动态降落 |
| `payload_servo.py` | 舵机控制（投放） |
| `car_t265.py` | 双T265坐标校准 |

## 任务一 vs 任务二

| 任务 | 内容 | 状态 |
|------|------|------|
| 任务一 | 静态航线 + 投放 | ✅ 已实飞通过 |
| 任务二 | 动态降落 + 复飞 | ✅ 已实飞通过 |

---

← [视觉伺服](visual-servo.md) | [事件总线架构 →](event-bus.md)
