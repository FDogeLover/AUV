---
name: feedback-push-after-commit
description: 本机提交后要主动push；板子FJJ独立git要及时commit(但不push)
metadata: 
  node_type: memory
  type: feedback
  originSessionId: ed312006-7b6d-4d6e-b12d-5a75f1d59077
---

两条并行的git workflow规则（2026-07-09由用户明确建立）：

1. **本机仓库(Project2)提交后要主动`git push`到远程**，不用像之前那样每次等用户明确要求才推送。
2. **`ubuntu-pi`上`FJJ/.git`（板载独立历史）也要及时commit**——每次同步文件到板子后，如果板子git有改动，应该主动`git add -A && git commit`，不要让改动堆积成未提交状态。

**Why**：用户在准备真机测试前主动提出这条要求，目的是保持两边的版本历史都干净、可追溯，避免本机commit了却忘记推送、或板子上改动堆积没有提交记录。

**How to apply**：
- 本机：每次`git commit`完，紧接着`git push origin main`（或当前分支），不用单独确认。这跟原先"不自动推送，等待用户明确要求"的默认行为不同，是2026-07-09起的新约定，已同步更新到 CLAUDE.md 的"Git 约定"一节。
- 板子：`scp`同步完文件后，如果涉及有意义的改动（不是临时/试验性文件），检查`ssh ... "cd FJJ && git status"`，有改动就提交。**板子的commit依然不push**——`FJJ/.git`跟本机仓库没有push/pull关联，这条没有变，只是本地commit要更及时。
- 如果commit信息可能包含敏感内容或改动范围不确定，仍应按常规安全协议先检查再操作，这条约定不豁免"review before push"的基本谨慎。
