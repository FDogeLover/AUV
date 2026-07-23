---
name: feedback-sync-test-data-to-local
description: 飞行测试后 ubuntu-pi 上归档的 flight_data 测试数据要同步回本地仓库，不要只留在板子上
metadata: 
  node_type: memory
  type: feedback
  originSessionId: fad5c2a9-8042-44f5-b3d1-efab1a9b716b
---

`ubuntu-pi`（`~/Desktop/FJJ/basic/test_data_*`）上归档的飞行测试数据（`flight_data_*.jsonl.bak` 等）以后要同步一份到本机仓库对应目录（`drone_control/tools/data_archive/test_data_*`），不要只留在板子上。

**Why:** 2026-07-07 会话里发现 2026-07-06 那次做了大量测试（X/Y轴平移、降落可靠性等），但数据只提交到了板载的 `FJJ/.git`，本机仓库完全没有——导致复盘时要先 SSH 上板子现拉数据，还差点因为板子断电/离线拉不到。跟 [[feedback_auto_sync_pi]]（本地改动同步到 pi）是反方向的同一类问题：单向同步会导致两边状态不一致。

**How to apply:** 飞行测试结束、板子上产生新的 `test_data_*` 归档 commit 后，主动把新增的测试数据文件 scp 回本机仓库对应路径，不用等用户要求。跟 [[feedback_auto_sync_pi]] 一样，这一步本身不用先问，照常做完汇报即可；涉及本机 git commit 仍按项目 CLAUDE.md 的 Git 约定，等用户明确要求再提交。
