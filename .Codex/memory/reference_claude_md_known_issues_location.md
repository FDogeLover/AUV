---
name: reference-claude-md-known-issues-location
description: "CLAUDE.md的'已知未解决问题'详情已迁移到docs/known_issues.md，CLAUDE.md本体只保留状态速查表"
metadata: 
  node_type: memory
  type: reference
  originSessionId: 97e88646-fcce-4d53-8f85-3594a274091f
---

2026-07-13起，`.claude/CLAUDE.md` 里原来又长又详细的"已知未解决问题"整节（25条，含完整时间线/数据表格/原始数据路径）已原样迁移到项目仓库的 `docs/known_issues.md`，CLAUDE.md 本体只保留一张紧凑状态表（编号/一句话现状/标签🔴🟡✅⏸🟢/链接）。

**How to apply**：
- 想知道某个问题"现在是什么状态"，先看 CLAUDE.md 里的状态表。
- 想看某个问题的完整调试历史、数据、原始文件路径，去 `docs/known_issues.md` 对应编号。
- 以后新增/更新问题记录时，状态表只改一行摘要，完整叙事写进 `docs/known_issues.md`——不要再把长叙事直接堆回 CLAUDE.md 本体，否则又会重新膨胀。
