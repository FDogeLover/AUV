---
name: project-flight-log-backup
description: 飞行日志归档由 pull_flight_log.sh 自动完成，不再手动备份
metadata: 
  node_type: memory
  type: project
  originSessionId: cda04a56-bdf1-42df-badc-36ac67856d0a
---

## 2026-07-24 更新：自动化归档

以前需要手动 `mv` 备份 `flight_data.jsonl`。现在 `tools/pull_flight_log.sh` 一键完成：
  1. 从板子 scp `flight_data.jsonl` 到本地 `data_archive/test_data_YYYYMMDD/`
  2. 以 router.txt 路径概要 + 日期自动命名，不覆盖
  3. 清空板子上的 `flight_data.jsonl`（为下次测试准备）
  4. 数据只存在本地，板子不留副本

**用法**：飞完后在本机执行 `./tools/pull_flight_log.sh`

## 旧记录（历史参考）

`drone_control/basic/Mission_GPT.py` 里日志文件以 `"a"`（追加）模式打开（`self._log_file = open(path + "/flight_data.jsonl", "a")`），不会自动清空。多次飞行/测试的数据会混在同一个文件里，只能靠 `t` 时间戳字段自己切分。

**Why:** 用户明确要求，每次实际飞行前先把 `ubuntu-pi:~/Desktop/FJJ/basic/flight_data.jsonl` 挪走备份（比如改名加时间戳后缀），这样飞行后拷回的日志是干净的单次数据，便于用 [[of-t265-correlation-analysis]] 工具分析，不用手动按时间戳过滤。

**2026-07-07 教训**：同一会话里连续飞了好几次（比如复测同一个改动），只在第一次飞行前备份了一次，后面几次飞行忘了再备份，导致连续两次都把两次飞行的数据混进了同一个文件，事后要靠时间戳间隔手动切分才能分析。**每一次**触发飞行(不管是第一次还是同一会话里的第N次复测)之前都要检查/备份一次，不能假设"这个session已经备份过了就够了"——启动`main.py`本身就是触发条件，跟"是不是同一批测试"无关。
