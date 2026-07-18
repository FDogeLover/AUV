---
name: feedback-verify-known-issues-detail-before-citing
description: 引用CLAUDE.md已知问题表的一行摘要来支撑设计决策前，要去docs/known_issues.md读完整条目，摘要可能省略了后续推翻/修正
metadata: 
  node_type: memory
  type: feedback
  originSessionId: ece4fa35-8cac-41a6-bdaa-2358804118db
---

CLAUDE.md「已知未解决问题」表格里每行只是一句摘要，指向`docs/known_issues.md`的完整条目。完整条目里经常包含"复测后发现结论下早了"这类后续修正，但摘要行不一定会同步更新这层nuance。

**Why:** 2026-07-13会话讨论绕障飞行器环绕航点设计时，直接引用CLAUDE.md问题15的摘要"安全步长上限约0.2m"来论证环绕航点弦长(0.5m)有风险，建议加密到16个点。用户指出"就算每步走0.5m也没有问题"。回去查`docs/known_issues.md`完整条目才发现：0.2m→0.4m的"硬阈值"结论在2026-07-08反向复测时**没有复现**，作者自己写了"说明之前那条结论下早了"，唯一站得住的是"扰动随步长单调增大"这个趋势（无硬上限）；而且问题21用大步长大范围测试精度反而更好。CLAUDE.md摘要行保留了"安全步长上限约0.2m"这个已经被推翻的表述，没有反映完整条目里的修正过程。

**How to apply:** 凡是要用某条已知问题的结论去支撑一个新设计决策或论证风险时，不要只看CLAUDE.md那一行摘要——去`docs/known_issues.md`读完整条目，特别注意条目里"XX真机验证/复测"这类后续段落，很可能包含对前面结论的修正甚至推翻。摘要行本身也可能滞后于完整条目的最新结论，参考[[feedback_claude_md_sync_lag]]（那条讲的是pi↔本机同步滞后，这条讲的是同一份本机文档内部"摘要vs详情"的滞后）。
