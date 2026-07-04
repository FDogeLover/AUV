---
name: session-summary
description: >
  会话总结与任务交接 Skill。
  在会话结束时用 `save` 参数保存当前工作状态为结构化 Markdown 总结文件；
  在新会话开始时无参数调用可读取最近总结，快速恢复上下文。
  当用户提到"总结"、"交接"、"保存进度"、"上次做到哪"、"handoff"、"会话总结"、
  或输入 /session-summary 时触发。
  也适合作为每日工作记录或跨会话任务交接的工具。
---

# Session Summary Skill

## 概述

本 Skill 提供三种操作模式：

| 命令 | 行为 |
|------|------|
| `/skill session-summary save` | **保存模式** — 扫描当前会话状态，生成总结文件 |
| `/skill session-summary` | **读取模式** — 展示最近一份总结 |
| `/skill session-summary --list` | **列表模式** — 列出所有历史总结文件 |

---

## 保存模式流程

当你收到 `save` 参数时，按以下步骤执行：

### 步骤 1：确认 Git 状态

并发运行以下命令获取当前 git 状态：

```bash
git branch --show-current
git rev-parse --short HEAD
git status --short
```

记录输出结果。

### 步骤 2：查找上次总结节点

检查 `docs/session-summaries/` 目录：

- **如果存在**该目录且有 `.md` 文件：找到最近的文件（按文件名降序取第一个），从中提取日期作为 git log 的起始点，并读取已有的完成/待办项作为参考
- **如果不存在**或为空：使用 `git log --oneline -10` 获取最近 10 条提交

### 步骤 3：获取变更信息

运行以下命令收集变更数据：

```bash
# 如有上次总结日期，获取自该日期以来的提交
git log --oneline --since="<上次总结日期>"

# 或如无上次总结，获取最近提交
git log --oneline -10

# 获取文件变更统计
git diff --stat

# 获取未跟踪文件
git status --short
```

### 步骤 4：询问用户（1-2 个问题，每个单独问）

**问题 1（如用户未主动说明）：**
> "本次会话的主要目标/主题是什么？"

**问题 2（可选）：**
> "有什么备注、踩坑记录或决策说明想加进总结？"

### 步骤 5：生成总结文件

1. 确保 `docs/session-summaries/` 目录存在（不存在则创建）
2. 使用下方内嵌的模板填充内容
3. 写入 `docs/session-summaries/<YYYY-MM-DD>-session-summary.md`

**文件名规则：** 使用当天的日期。同一天多次 `save` 会覆盖当天的文件。

### 步骤 6：输出结果

告知用户：

> "✅ 总结已保存到 `docs/session-summaries/2026-07-04-session-summary.md`"
> "你可以 review 一下内容，需要修改的话告诉我。"

**不要自动 git add/commit。** 让用户自行决定是否版本控制。

---

## 读取模式流程

当你收到无参数的 `/skill session-summary` 时：

### 步骤 1：查找最新总结

```bash
ls docs/session-summaries/*.md 2>/dev/null | sort -r | head -1
```

### 步骤 2：有总结文件 → 展示

输出以下内容：

- 📄 **文件名** 和 **日期**
- 🌿 **分支**
- 🎯 **会话目标**
- ✅ **完成项**（完整的列表）
- 🚧 **未完成/待办**（完整的列表）
- 💡 **下一步建议**
- 末尾提示：*"输入 `/skill session-summary save` 保存本次会话"*

### 步骤 3：无总结文件 → 提示

如果目录不存在或无 `.md` 文件：

> "📭 暂无历史会话总结。"
> "输入 `/skill session-summary save` 来创建第一份总结。"

---

## 列表模式流程

当你收到 `--list` 参数时：

### 步骤 1：扫描总结目录

```bash
ls docs/session-summaries/*.md 2>/dev/null | sort -r
```

### 步骤 2：展示列表

```markdown
📋 历史会话总结：
1. 2026-07-04 — 会话总结 — <第一行标题摘要>
2. 2026-07-03 — 会话总结 — <第一行标题摘要>
...
```

对每个文件，提取第一行 `# 会话总结 — {日期}` 作为摘要展示。

---

## 模板

使用以下 Markdown 模板填充内容。`{}` 中的占位符由你根据收集到的信息替换。

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

参考文件：`references/template.md` 也包含了同样的模板，可读取使用。

---

## 约束

- ✅ **输出目录固定为 `docs/session-summaries/`**
- ✅ **文件名格式固定为 `YYYY-MM-DD-session-summary.md`**
- ✅ **同一天多次 save 会覆盖当天的文件**
- ❌ **不自动 git add/commit/push** — 由用户自行管理
- ❌ **不跨会话自动触发** — 需用户手动调用
- ❌ **不涉及外部服务或 API**
- ❌ **不解析 issue tracker 或外部系统**
