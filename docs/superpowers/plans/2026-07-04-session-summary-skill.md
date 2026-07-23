# Session Summary Skill — 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 创建 `session-summary` Skill，实现会话总结保存与读取功能

**架构:** 纯 Prompt 模板方案。一个 SKILL.md 包含完整指令 + 内嵌模板，references/template.md 作为独立模板参考文件。无外部脚本依赖。

**Tech Stack:** ZCode Skill (Markdown + YAML frontmatter)

---

## 文件结构

```
.agents/skills/session-summary/
├── SKILL.md              ← Skill 定义：frontmatter + 指令 + 内嵌模板
└── references/
    └── template.md       ← 纯模板文件，供模型参考
```

### Task 1: 创建目录结构

**Files:**
- Create: `.agents/skills/session-summary/`
- Create: `.agents/skills/session-summary/references/`

- [ ] **Step 1: 创建目录**

```bash
mkdir -p ".agents/skills/session-summary/references"
```

- [ ] **Step 2: 验证目录已创建**

```bash
ls -la ".agents/skills/session-summary/"
```

---

### Task 2: 编写 references/template.md

**Files:**
- Create: `.agents/skills/session-summary/references/template.md`

- [ ] **Step 1: 写入模板文件**

模板内容就是设计文档中确认的 Markdown 模板，作为独立文件供模型读取参考。

```markdown
# 会话总结 — {日期}

**分支：** `{branch}`
**提交：** `{last_commit_hash}`
**会话目标：** {session_goal}

---

## \u1f4cb 完成项

- {item1}
- {item2}

## \u1f6a7 未完成 / 待办

- [ ] {item1}
- [ ] {item2}

## \u1f527 修改的文件

### 飞控固件 (C)
- `path/file.c` — 改动说明

### 上位机 (Python)
- `path/file.py` — 改动说明

## \u1f4a1 下一步建议

1. {建议一}
2. {建议二}

## \u1f517 相关上下文

- 设计文档：{link}
- 踩坑记录：{note}

## \u1f4ca Git 状态

- 当前分支：{branch}
- 相比上次总结的新提交：{count}
- 未提交改动：{yes/no}
```

- [ ] **Step 2: 验证文件写入**

```bash
cat ".agents/skills/session-summary/references/template.md"
```

---

### Task 3: 编写 SKILL.md

**Files:**
- Create: `.agents/skills/session-summary/SKILL.md`

这是核心文件。包含：
1. YAML frontmatter（name, description）
2. 三种模式的定义和触发方式
3. 保存模式的完整工作流程
4. 读取模式的工作流程
5. 列表模式的工作流程
6. 内嵌的 Markdown 模板
7. 约束条件

- [ ] **Step 1: 写入 SKILL.md**

```markdown
---
name: session-summary
description: >
  会话总结与任务交接 Skill。在会话结束时用 `save` 参数保存当前工作状态；
  在新会话开始时无参数调用可读取最近总结恢复上下文。
  当用户提到"总结"、"交接"、"保存进度"、"上次做到哪"、"handoff" 或输入 /session-summary 时触发。
  也适合作为每日工作记录的工具。
---

# Session Summary Skill

## 概述

本 Skill 提供两种核心功能：

1. **保存模式 (`save`)** — 扫描当前会话的工作状态，生成结构化 Markdown 总结文件
2. **读取模式 (无参数)** — 展示最近的总结，快速恢复上下文
3. **列表模式 (`--list`)** — 列出所有历史总结文件

## 触发方式

通过 `/skill session-summary` 调用，第一个参数区分模式：

| 命令 | 行为 |
|------|------|
| `/skill session-summary save` | 保存当前会话状态 |
| `/skill session-summary` | 读取最近一份总结 |
| `/skill session-summary --list` | 列出所有总结文件 |

## 保存模式流程

当你收到 `/session-summary save` 时，按以下步骤执行：

### 步骤 1: 确认 Git 状态
- 运行 `git branch` 获取当前分支
- 运行 `git status --short` 检查未提交改动
- 运行 `git rev-parse --short HEAD` 获取最新提交 hash

### 步骤 2: 查找上次总结节点
- 检查 `docs/session-summaries/` 目录是否存在
- 如果存在，找到最近的总结文件，获取其日期作为 git log 的起始点
- 如果不存在，使用 `git log --oneline -10` 获取最近的提交历史

### 步骤 3: 获取变更信息
- 运行 `git log --oneline --since="<上次总结日期>"` 或 `git log --oneline -10` 获取提交记录
- 运行 `git diff --stat` 获取文件变更概览
- 如果有未跟踪文件，运行 `git status --short` 获取列表

### 步骤 4: 询问用户
向用户提问（一次一个问题）：
1. "本次会话的主要目标是什么？"（如果用户没有主动说明）
2. "有什么备注或踩坑记录想加进总结？"（可选）

### 步骤 5: 生成总结文件
- 使用下方内嵌的 Markdown 模板填充内容
- 写入 `docs/session-summaries/YYYY-MM-DD-session-summary.md`
- 确保目录存在（如不存在则创建）

### 步骤 6: 输出结果
告知用户文件已生成：
> "总结已保存到 `docs/session-summaries/2026-07-04-session-summary.md`"
> "你可以 review 一下内容，需要修改的话告诉我。"

## 读取模式流程

当你收到 `/session-summary`（无参数）时：

### 步骤 1: 查找最新总结
- 检查 `docs/session-summaries/` 下所有 `.md` 文件
- 按文件名降序排列，取第一个（最近的）

### 步骤 2: 展示内容
输出以下内容给用户：
- **文件名** 和 **日期**
- **分支** 和 **会话目标**
- **完成项** 列表
- **未完成/待办** 列表
- **下一步建议**
- 末尾提示：`输入 /session-summary save 来保存本次会话`

### 步骤 3: 无总结文件时的处理
如果 `docs/session-summaries/` 不存在或为空：
> "暂无历史会话总结。输入 `/session-summary save` 来创建第一份总结。"

## 列表模式流程

当你收到 `/session-summary --list` 时：

### 步骤 1: 扫描总结目录
- 列出 `docs/session-summaries/` 下所有 `.md` 文件
- 按文件名降序排列

### 步骤 2: 展示列表
对每个文件提取第一行（# 标题）作为摘要，展示：
```
📋 历史会话总结：
1. 2026-07-04 — 会话总结 — 稳定性重构第三阶段
2. 2026-07-03 — 会话总结 — K230 视觉调试
```

## 模板

直接使用以下模板填充。`{}` 中的内容由你根据收集到的信息替换。

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

## 约束

- ❌ **不自动 git add/commit** — 总结文件由用户自行管理版本
- ✅ **输出目录固定为 `docs/session-summaries/`**
- ✅ **文件名格式固定为 `YYYY-MM-DD-session-summary.md`**
- ✅ **同一天多次 save 会覆盖当天的文件**（同名文件）
- ❌ **不跨 session 自动触发** — 需要用户手动调用
- ❌ **不涉及外部服务或 API**
```

- [ ] **Step 2: 验证 SKILL.md**

```bash
head -10 ".agents/skills/session-summary/SKILL.md"
```

---

### Task 4: 功能测试

- [ ] **Step 1: 测试保存模式**

```bash
# 确保输出目录存在
mkdir -p docs/session-summaries/

# 验证 skill 文件结构完整
ls -laR .agents/skills/session-summary/
```

预期输出：
```
.agents/skills/session-summary/:
total 2
drwxr-xr-x  ... .
drwxr-xr-x  ... ..
drwxr-xr-x  ... references
-rw-r--r--  ... SKILL.md

.agents/skills/session-summary//references:
total 1
-rw-r--r--  ... template.md
```

- [ ] **Step 2: Git 提交**

```bash
git add .agents/skills/session-summary/
git add docs/superpowers/plans/2026-07-04-session-summary-skill.md
git commit -m "skill: 创建 session-summary skill（会话总结与任务交接）"
```
