---
title: 无人机项目 Claude Code 多 Agent 配置设计
date: 2026-05-19
status: approved
---

# 无人机项目 Claude Code 多 Agent 配置设计

## 1. 概述

为无人机工程项目（`Project2`）配置 Claude Code 多 Agent 体系，使开发者在编写飞控固件、Python 控制端、K230 视觉代码时能切换到对应的专家 Agent，提升开发效率。

## 2. 项目结构

项目根目录：`D:\项目与工具\Python项目\Project2`

```
├── .claude/
│   ├── agents/
│   │   ├── fc-firmware.md          ← 飞控固件 Agent（自动发现）
│   │   ├── drone-control.md        ← 控制端 Agent（自动发现）
│   │   └── vision.md               ← 视觉 Agent（自动发现）
│   ├── skills/
│   │   └── drone-tools/
│   │       └── SKILL.md            ← 项目专用 Skill
│   └── settings.json               ← 项目级别设置
├── CLAUDE.md                       ← enabledSkills + 工程规范
├── drone_control/                  ← 已有，不变
├── ANO_LX_FC_T265代替光流/        ← 已有，不变
├── ANO_LX_FC_倾角保护版/          ← 已有，不变
├── k230/                           ← 已有，不变
└── CodeWiki/                       ← 已有，不变
```

## 3. Agent 定义

### 3.1 fc-firmware（飞控固件）

- **专注领域**：飞控固件开发
- **技术栈**：C, Keil MDK, STM32F407, MSP432P401, TM4C123
- **擅长**：裸机嵌入式、实时控制、PWM/RC/ADC/UART 驱动、定时器中断、传感器融合
- **可用工具**：Read, Grep, Glob, Edit, Write
- **行为准则**：关注代码实时性、ROM/RAM 占用、外设驱动正确性

### 3.2 drone-control（Python 控制端）

- **专注领域**：上位机控制与通信
- **技术栈**：Python, pyrealsense2, pyserial, 多线程
- **擅长**：串口通信协议、T265 定位、PID 控制、任务状态机、覆盖规划
- **可用工具**：Read, Grep, Glob, Edit, Write
- **行为准则**：关注系统稳定性、异常处理、串口协议一致性

### 3.3 vision（视觉处理）

- **专注领域**：AI 视觉处理
- **技术栈**：Python, K230, YOLOv8, OpenCV
- **擅长**：目标检测模型部署、数据集采集、推理优化
- **可用工具**：Read, Grep, Glob, Edit, Write
- **行为准则**：关注检测精度、推理速度、模型部署流程

## 4. 项目 Skill

### drone-tools

提供在 Claude Code 中快速执行的项目命令：

| 命令 | 说明 |
|------|------|
| `pnpm dev` | 启动 drone_control 主程序 |
| `pnpm lint` | ruff 检查 Python 代码 |
| `pnpm test` | 运行测试脚本 |
| `pnpm build:fc` | 提示：使用 Keil 编译飞控固件 |
| `pnpm flash:fc` | 提示：使用 JLink 烧录固件 |

项目约定：
- Python 代码：ruff 规范
- C 固件代码：保持现有 ANON 命名规范
- 提交信息：Conventional Commits 格式
- 协议文档：参考 `CodeWiki/` 和 `地面站通信协议.md`

## 5. CLAUDE.md

启用 Skills：
- `superpowers:brainstorming`
- `superpowers:writing-plans`
- `drone-tools`

禁用 Skills：
- `loop`
- `claude-api`

## 6. 不变内容

- 所有现有代码目录不动
- `drone_control/.claude/settings.local.json` 保留
- `.gitignore` 保持不变