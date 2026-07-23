# Session Summary Skill — 设计文档

**日期：** 2026-07-04
**状态：** 已批准
**实现方式：** 纯 Prompt 模板（方案 A）

---

## 1. 概述

创建一个名为 `session-summary` 的 ZCode Skill，用于：

1. **会话总结** — 在会话结束时，将当前工作状态整理为结构化的 Markdown 总结文件
2. **任务交接** — 在新会话开始时，读取最近的总结，快速恢复上下文并获取下一步建议

---

## 2. 基本信息

| 字段 | 值 |
|------|-----|
| Skill 名称 | `session-summary` |
| 存放路径 | `.agents/skills/session-summary/SKILL.md` |
| 输出目录 | `docs/session-summaries/` |
| 文件名格式 | `YYYY-MM-DD-session-summary.md` |
| 触发方式 | `/session-summary` 手动调用 |

---

## 3. 触发模式

Skill 通过第一个参数区分三种模式：

| 命令 | 行为 |
|------|------|
| `/session-summary save` | 保存模式：扫描当前状态，生成总结文件 |
| `/session-summary` | 读取模式（默认）：显示最近的一份总结 |
| `/session-summary --list` | 列出所有历史总结文件 |

---

## 4. 保存模式工作流程

1. 确认当前 git 状态（分支、有无未提交改动）
2. 获取自上次总结以来的 git log（对比上一个总结文件的日期）
3. 扫描本次会话涉及的改动文件（已修改/新增/未跟踪）
4. 询问用户"想对本次总结加什么备注？"（可选）
5. 按模板生成 Markdown 文件 → `docs/session-summaries/YYYY-MM-DD-session-summary.md`
6. 输出文件路径，提示用户查看

> 注意：不自动 git add/commit，留给用户自己决定。

---

## 5. 读取模式工作流程

- `/session-summary`（无参数）
  - 找到 `docs/session-summaries/` 中最近的一份总结
  - 打印摘要 + 完整的"未完成/待办"和"下一步建议"板块
  - 末尾提示：`输入 /session-summary save 来保存本次会话`

- `/session-summary --list`
  - 列出 `docs/session-summaries/` 下所有总结文件（按时间倒序）
  - 显示文件名 + 第一行摘要

---

## 6. 总结模板

```markdown
# 会话总结 — {日期}

**分支：** `{branch}`
**提交：** `{last_commit_hash}`
**会话目标：** {session_goal}

---

## 📋 完成项

- {item1}
- {item2}

## 🚧 未完成 / 待办

- [ ] {item1}
- [ ] {item2}

## 🔧 修改的文件

### 飞控固件 (C)
- `path/file.c` — 改动说明

### 上位机 (Python)
- `path/file.py` — 改动说明

## 💡 下一步建议

1. {建议一}
2. {建议二}

## 🔗 相关上下文

- 设计文档：{link}
- 踩坑记录：{note}

## 📊 Git 状态

- 当前分支：{branch}
- 相比上次总结的新提交：{count}
- 未提交改动：{yes/no}
```

---

## 7. Skill 目录结构

```
.agents/skills/session-summary/
├── SKILL.md              ← 指令 + 内嵌模板
└── references/
    └── template.md       ← 独立的模板文件（供模型参考）
```

注：当前采用方案 A（纯 Prompt 模板），不包含辅助脚本。后续可根据实际使用反馈决定是否增加 `scripts/` 目录。

---

## 8. SKILL.md 内容规划

### Frontmatter

```yaml
name: session-summary
description: >
  会话总结与任务交接 Skill。
  在会话结束时使用 `save` 参数保存当前工作状态为结构化总结文件；
  在新会话开始时无参数调用可读取最近总结，快速恢复上下文。
  当用户提到"总结"、"交接"、"保存进度"、"上次做到哪"、"handoff" 或输入 /session-summary 时触发。
```

### Body 关键指令

- **保存模式**：git log 对比、文件变更扫描、用户备注收集、模板填充、写入文件
- **读取模式**：查找最新总结文件、摘要展示
- **列表模式**：遍历输出目录、按时间排序、列出摘要
- **模板定义**：内嵌完整 Markdown 模板
- **约束**：
  - 不自动 git add/commit
  - 输出到 `docs/session-summaries/` 目录
  - 文件名使用 `YYYY-MM-DD-session-summary.md`

---

## 9. 与现有生态的集成

- 项目已有 `.claude/` 配置结构，但本 Skill 存放在 `.agents/skills/` 下，与 `.claude/` 解耦
- 与已有的 `docs/superpowers/specs/` 和 `docs/superpowers/plans/` 互补：spec → plan → session-summary 形成完整工作流
- 无冲突，纯增量

---

## 10. 未涵盖的范围（YAGNI）

- ❌ 不自动触发（无 SessionStart/SessionEnd hook）
- ❌ 不包含辅助脚本（当前阶段）
- ❌ 不自动 git commit/push
- ❌ 不涉及跨设备同步（用户通过 git 自行同步）
- ❌ 不自动解析 issue tracker
