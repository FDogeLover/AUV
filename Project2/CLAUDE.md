---
enabledSkills:
  - superpowers:brainstorming
  - superpowers:writing-plans
  - drone-tools
disabledSkills:
  - loop
  - claude-api
---

# Project2 - 无人机工程

无人机工程项目，包含飞控固件、Python 控制端、K230 视觉处理。

## 项目结构

```
├── ANO_LX_FC_T265代替光流/       ← 飞控固件（C/Keil, T265代替光流版）
├── ANO_LX_FC_倾角保护版/         ← 飞控固件（C/Keil, 倾角保护版）
├── drone_control/                 ← Python 上位机控制端
├── k230/                          ← K230 AI 视觉处理
└── CodeWiki/                      ← 架构与接口文档
```

## 使用方式

```bash
# 进入项目目录启动
cd D:\项目与工具\Python项目\Project2\Project2
claude

# 或指定 Agent
claude --agent fc-firmware
claude --agent drone-control
claude --agent vision
```

## 切换 Agent

在会话中输入 `/agent` 查看可用角色，选择切换即可。