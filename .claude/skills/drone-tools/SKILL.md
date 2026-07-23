---
name: drone-tools
description: 无人机工程的项目命令与约定
---

# Drone Tools

## 项目约定
- Python 代码使用 **ruff** 规范
- 飞控 C 代码保持现有 ANON 命名规范（ANO_/Drv_/User_ 前缀）
- 串口通信协议参考 `CodeWiki/` 和 `地面站通信协议.md`
- 提交信息使用 Conventional Commits 格式

## 本地命令
```bash
pnpm dev          # 启动 drone_control/original/main.py（全功能版；2026-07-07 起从根目录整理进 original/ 子目录）
pnpm lint         # ruff 检查 Python 代码
pnpm test         # 运行测试脚本
pnpm build:fc     # 使用 Keil 编译飞控固件（手动操作）
pnpm flash:fc     # 使用 JLink 烧录固件（手动操作）
pnpm docs         # 查阅 CodeWiki 文档
```

## 切换 Agent
```bash
# 在 Claude Code 会话中
/agent            # 查看可用 Agent（fc-firmware / drone-control / vision）
```

## 相关文档
- [架构概览](CodeWiki/01_Architecture.md)
- [模块说明](CodeWiki/02_Modules.md)
- [关键流程与 API](CodeWiki/03_Key_Flows_and_APIs.md)
- [依赖说明](CodeWiki/04_Dependencies.md)
- [构建与运行](CodeWiki/05_Build_and_Run.md)
- [地面站通信协议](drone_control/original/地面站通信协议.md)