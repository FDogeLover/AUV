---
enabledSkills:
  - superpowers:brainstorming
  - superpowers:writing-plans
  - drone-tools
disabledSkills:
  - loop
  - claude-api
---

## 交互偏好

- **不使用交互式提问组件**（AskUserQuestion 工具）。需要用户提供信息时，直接在对话中以文字写出问题，等用户打字回复。

# Project2 - 无人机工程

> **新 Agent 入口 → `docs/AGENTS_GUIDE.md`**（5 分钟掌握全局）
>
> 2026-07-24 精简：详细记忆在 `memory/` 目录中，本文件只保留关键配置。

## 编码规范（关键约束！）

### 飞控固件 .c/.h 文件 — GB2312/GBK 编码

**绝对禁止**用 Read/Edit 工具直接编辑 `.c/.h` 文件！必须用 `edit_firmware.py` 脚本。

详情见 `memory/project_board_git_corruption_recovery.md`。

## ZCode × Qoder 协作规范

### 触发阈值

| 场景 | 是否走完整流程 |
|------|--------------|
| 参数调整、注释、重命名、单函数小改动 | ❌ 直接改 |
| bug 修复（已有测试覆盖） | ❌ 直接改 |
| **新增功能**（新状态、新传感器、新赛题模块） | ✅ 完整流程 |
| **大规模重构**（跨 3+ 文件或改变接口/状态机结构） | ✅ 完整流程 |
| 安全关键路径改动（land/QR解码/通信协议） | ✅ 完整流程（即使改动小） |

### 完整协作流程

```
Step 1  ZCode 撰写结构化计划文档 → .zcode/plans/
Step 2  Qoder Plan Review（第一轮审查）→ [高风险]/[中风险]/[低风险]/[通过]
Step 3  决策检查点 — 用户决定高风险处理方式
Step 4  按确认方案实施代码改动
Step 5  Qoder Implementation Review（第二轮审查）
Step 6  最终结论
```

**审查轮次上限：2轮**。超出说明方案本身需重新设计。

### 差异化 Prompt

| 改动类型 | 方式 |
|---------|------|
| 状态机 | `/qoder-statemachine` |
| 安全关键 | `/qoder-review` |
| 上板前 | `/qoder-preflight` |

Qoder CLI：`C:\Users\FJJ\.qoder-cn\bin\qoderclicn\qoderclicn.exe`

## 索引

```
.claude/CLAUDE.md                            ← 完整操作规范（SSH/编码/已知问题速查）
memory/                                      ← 48个详细记忆文件
|-- project_basic_framework_architecture.md  ← 项目架构
|-- project_board_git_corruption_recovery.md ← 固件编码 & 编辑规范
|-- feedback_*.md                            ← 实践教训
docs/known_issues.md                         ← 已知问题完整详情（40条）
docs/guides/imu_parameters_and_fusion_architecture.md ← IMU参数理解
.zcode/plans/two_stage_landing_plan.md       ← 两级降落计划
```
