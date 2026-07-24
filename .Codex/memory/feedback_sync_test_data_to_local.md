---
name: feedback-sync-test-data-to-local
description: 飞行数据用 pull_flight_log.sh 自动归档到本地，不再手动 scp
metadata: 
  node_type: memory
  type: feedback
  originSessionId: fad5c2a9-8042-44f5-b3d1-efab1a9b716b
---

## 2026-07-24 更新：pull_flight_log.sh 自动化

以前需要手动 `scp` 数据回本地。现在 `tools/pull_flight_log.sh` 自动完成：
  1. 从板子 scp `flight_data.jsonl` + `router.txt`
  2. 归档到 `drone_control/tools/data_archive/test_data_YYYYMMDD/`
  3. 自动按 router.txt 路径 + 日期命名，不覆盖
  4. 清空板子工作日志
  5. 数据只存在本地，板子不留副本

**用法**：飞完在本机执行 `./tools/pull_flight_log.sh`

**2026-07-24 同步历史数据**：从板子 `~/Desktop/FJJ/test_data/` 拉取了全部历史数据 (~416MB, 2026-07-05 至 2026-07-22)，归档到本地后已清理板子空间。

## 旧记录（历史参考）

`ubuntu-pi`（`~/Desktop/FJJ/basic/test_data_*`）上归档的飞行测试数据（`flight_data_*.jsonl.bak` 等）以后要同步一份到本机仓库对应目录（`drone_control/tools/data_archive/test_data_*`），不要只留在板子上。

**Why:** 2026-07-07 会话里发现 2026-07-06 那次做了大量测试（X/Y轴平移、降落可靠性等），但数据只提交到了板载的 `FJJ/.git`，本机仓库完全没有——导致复盘时要先 SSH 上板子现拉数据，还差点因为板子断电/离线拉不到。跟 [[feedback_auto_sync_pi]]（本地改动同步到 pi）是反方向的同一类问题：单向同步会导致两边状态不一致。
