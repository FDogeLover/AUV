# Project2 无人机工程 — Agent 快速入门

> **给第一次进入本项目的 Agent 读**，5 分钟掌握全局。读完这篇你就知道该去哪找什么。

---

## 这是什么项目

无人机工程项目，包含三层：

| 层 | 技术栈 | 作用 |
|---|--------|------|
| **飞控固件** | C/Keil (STM32F407) + 凌霄IMU | 姿态自稳、电机控制、传感器融合 |
| **上位机 (Python)** | Python 3 + T265 + 光流 | 状态机控制、视觉定位、导航 |
| **任务调度** | Mission_GPT.py 状态机 | 起飞 → 导航 → 降落全流程 |

---

## 文档体系导航

```
┌─────────────────────────────────────────────────────┐
│  docs/AGENTS_GUIDE.md          ← 你现在在这里       │
│  (首次进入先读这篇，指导你去哪找什么)                 │
└─────────────────────────────────────────────────────┘
          │
          ▼ 接下来按优先级读:
```

| 优先级 | 文件 | 内容 |
|--------|------|------|
| ⭐⭐⭐ | `.Codex/memory/MEMORY.md` | **50 个详细记忆文件索引**（项目结构、工程决策、实战教训） |
| ⭐⭐⭐ | `docs/known_issues.md` | **46 条已知问题详情**（完整时间线、数据、根因） |
| ⭐⭐ | `docs/TODO.md` | 待办事务 + 重要性×紧急性评分 |
| ⭐⭐ | `docs/guides/` | 操作指南（IMU参数理解、导航配置等） |
| ⭐ | `.zcode/plans/` | 功能计划文档（两级降落等） |

### 各 Agent 配置说明

| 文件 | 作用 |
|------|------|
| `.claude/CLAUDE.md` | ZCode Agent：编码规范 + SSH 操作 + 已知问题速查 |
| `.Codex/CLAUDE.md` | Codex Agent：交互偏好 + Qoder 协作流程 |
| 两者都在开头指向本指南 | 所以无论哪个 Agent 启动，最终都会读到这里 |

---

## 最近重要变更（2026-07-24）

| 变更 | 说明 | 文档位置 |
|------|------|---------|
| 两级降落 DESCEND/HOVER_WAIT | 替代不稳定的 OneKey_Land CMD 通道，真机验证通过 | `.Codex/memory/project_two_stage_landing.md` |
| 航向保持参数调优 | kp=0.5, max_rate=3, runaway_growth=15° 修复误触 | `.Codex/memory/project_heading_hold_fix.md` |
| 飞行日志分析工具 | `tools/flight_log_analyzer.py` 一键分析 flight_data.jsonl | `.Codex/memory/project_flight_tools_workflow.md` |
| 数据归档流程 | `tools/pull_flight_log.sh` 拉取→归档→清空板子 | `.Codex/memory/project_flight_tools_workflow.md` |
| 历史数据迁移 | 板子 416MB 测试数据全部拉到本地 `data_archive/` | — |
| 文档体系精简 | CLAUDE.md 去重瘦身，docs/ 和 memory/ 为唯一事实源 | 本文件就是结果 |

---

## 关键约束（必须遵守）

### 飞控固件编辑
- **绝对禁止**用 Read/Edit 工具直接编辑 `.c/.h` 文件（GB2312 编码）
- 必须用项目根目录的 `edit_firmware.py` 脚本
- 详细规范见 `.Codex/memory/project_board_git_corruption_recovery.md`

### SSH 操作（板子 ubuntu-pi）
| 级别 | 操作 | 规则 |
|------|------|------|
| 放开 | `ls`/`cat`/`ps`/`git log`/`df` | 随意 |
| 限路径 | `cp`/`scp`/`mkdir` 在 `~/Desktop/FJJ/` 下 | 执行前说明 |
| 需确认 | `rm -rf`/`git reset --hard` | 展示命令后等同意 |
| **禁止** | `systemctl`/`reboot`/`apt`/`kill` 非自己进程/改网络配置 | 停下确认 |

完整规范见 `.claude/CLAUDE.md`「远程设备操作规范」。

### 文件同步约定
- 本机 → 板子：逐文件 `scp`，不用 `scp -r`（避免 `__pycache__`）
- 板子 FJJ/.git 换行符 LF/CRLF 混用，scp 后先核对再 commit
- root 操作后必须 `chown -R sunrise:sunrise`

### 数据归档
- **数据只存在本地**，板子不留历史副本
- 飞完执行：`./tools/pull_flight_log.sh`
- 板子 `flight_data.jsonl` 会被自动清空

---

## 故障排查入口

| 症状 | 先去读 |
|------|--------|
| 降落异常 / 锁桨失败 | `docs/known_issues.md #7, #22, #46` |
| 航向偏转 / yaw 漂移 | `docs/known_issues.md #45`、`.Codex/memory/project_heading_hold_fix.md` |
| 高度控制异常 | `docs/known_issues.md #6, #26` |
| T265 检测不到 | `docs/known_issues.md #1` |
| 测试数据在哪 | `drone_control/tools/data_archive/test_data_YYYYMMDD/` |
| 代码编译 / 固件编辑 | `.Codex/memory/project_board_git_corruption_recovery.md` |
| 板子连不上 SSH | `.Codex/memory/project_ubuntu_pi_dynamic_ip.md` |

## 可用技能

| 技能 | 命令 | 用途 |
|------|------|------|
| 文档管理 | `/doc check/map/sync/todo` | 维护文档一致性、验证引用、结构图、TODO 核验 |
| 会话总结 | `/session-summary save` | 保存/读取会话进度 |
| Qoder 协作 | `/qoder-workflow` | 新功能/重构的完整设计→审查→实现流程 |
