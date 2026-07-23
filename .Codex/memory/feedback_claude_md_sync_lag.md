---
name: feedback-claude-md-sync-lag
description: CLAUDE.md已知问题列表容易跟不上ubuntu-pi上的实际测试进展，每次涉及飞行测试的会话结束前要主动核对补齐
metadata: 
  node_type: memory
  type: feedback
  originSessionId: fad5c2a9-8042-44f5-b3d1-efab1a9b716b
---

飞行测试的真实发现经常先落在 `ubuntu-pi` 上 `FJJ/.git` 的提交信息里（因为测试往往是独立进行、事后才在Claude Code会话里复盘），而不是第一时间写回本机仓库的项目 CLAUDE.md「已知未解决问题」列表。如果没人主动同步，CLAUDE.md 会长期停留在几天前的状态，下次会话直接读 CLAUDE.md 会得出过时甚至被推翻的结论。

**Why:** 2026-07-07 会话里，CLAUDE.md 只记录到"起飞/降落水平位移"这条问题，完全没提到 2026-07-06 晚上（commit `d11c354`）实际做的六次X/Y轴平移测试、发现的"减速阶段Y轴漂移9-12cm"、"降落成功与触发时机(是否静止)强相关"这两个新问题。导致本次会话一开始基于 CLAUDE.md 给出的"今天测什么"建议是错的（用户纠正："昨天最后的问题不是这个吧"），只能临时SSH上ubuntu-pi翻git log和原始数据才拼出真实情况。根本原因是两边"同步"目前只覆盖了代码文件（[[feedback_auto_sync_pi]] 本机→pi）和测试数据本身（[[feedback_sync_test_data_to_local]] pi→本机），唯独 **CLAUDE.md 里的"已知问题"叙述本身没有被纳入任何自动同步机制**，全靠人工事后补记，很容易漏。

**How to apply:** 每次涉及真实飞行测试的会话（不管是本次会话内做的测试，还是复盘之前独立发生的测试）：
1. 开始阶段如果 CLAUDE.md 的"已知问题"提到的最新日期早于最近一次 `ssh ubuntu-pi "cd ~/Desktop/FJJ && git log --oneline -5"` 看到的提交日期，主动提醒可能有未同步的发现，先看 pi 上的 git log/commit message 补齐上下文，不要直接假设 CLAUDE.md 是最新的。
2. 任何一次会话如果新确认/推翻了一个问题的结论（不管是通过分析已有数据还是新飞行），当场就把 CLAUDE.md 对应条目更新掉，不要留到"以后"——参考本次会话对问题2/8/11的更新方式（关闭已解决的、新增新发现的、标注决策）。
3. 如果用户提到"昨天/上次做了什么"而 CLAUDE.md 里没有对应记录，第一反应应该是"去pi上翻git log核实"，而不是"基于CLAUDE.md现有内容推测"。
