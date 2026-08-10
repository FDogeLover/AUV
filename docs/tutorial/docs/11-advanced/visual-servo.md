# 视觉伺服精准降落

## 概述

参考 `competition_2026/vision/` 和 `CyberCamera/boards/cybercam/`。核桃派运行黑色方块检测，UART发送目标偏移给Pi，IBVS控制器闭环控制无人机对准降落点。

## 架构

```
Cyber Camera (核桃派)
├── CSI摄像头 → 检测黑色方块
└── UART 115200 → 上位机
        ↓
Pi (RDK X5)
├── cyber_cam_reader.py → 读取检测结果
├── servo_controller.py → IBVS控制器
└── → Lprotocol → 飞控 → 电机修正
```

## 关键文件

| 文件 | 位置 | 职责 |
|------|------|------|
| `servo_controller.py` | `competition_2026/vision/` | tick-based IBVS控制器 |
| `cyber_cam_reader.py` | `competition_2026/vision/` | UART读取CyberCAM检测结果 |
| `square_detector.py` | `competition_2026/vision/` | 方块检测 |
| `detector.py` | `CyberCamera/boards/cybercam/` | 核桃派端检测算法 |
| `protocol.py` | `CyberCamera/boards/cybercam/` | 串口协议编码 |

## 计划文档

详细设计见 `.zcode/plans/visual_servo_landing_plan.md`（含3版迭代）。

!!! info "当前状态"
    已完成设计+实现+Qoder审查，等Cyber Camera到货部署。

---

[空地协同（D题）→](air-ground.md)
