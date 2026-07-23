---
name: feedback-auto-sync-pi
description: 修改 drone_control 相关 Python 文件后，自动 scp 同步到 ubuntu-pi，不用每次询问
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 3767b980-1dc2-400b-8a6f-54179837a7f7
---

修改 `drone_control/`（包括 `basic/` 精简版和全功能版）下的 Python 文件之后，直接 `scp` 同步到 `ubuntu-pi:~/Desktop/FJJ/`对应路径（`basic/` 对应 `~/Desktop/FJJ/basic/`，全功能版对应 `~/Desktop/FJJ/original/`），同步后 `chown sunrise:sunrise`，不用每次都先问"要不要同步"。

**Why:** 2026-07-05 用户明确说"后面相关修改自动同步过去"——本机改完不同步等于没用，之前每次都要额外问一遍、等确认，显得多余。

**How to apply:** 只要编辑了 `drone_control/basic/*.py` 或 `drone_control/*.py`（全功能版）里已经部署到 pi 上的文件，编辑完就直接 scp 过去并 chown，照常汇报"已同步"即可，不必等用户确认这一步本身（其他风险更高的操作，比如启动真实飞行、kill 进程、改配置常量之外的破坏性操作，仍然要按 [[project_motor_unlock_test_safety]] 等既有安全规范单独确认）。仅同步部署到板子上的文件，不涉及 git commit（那个仍需用户明确要求，见项目 CLAUDE.md 的 Git 约定）。

**例外（2026-07-08细化）**：改动同时涉及 `basic`/`basic_radar`/`original` 多个版本、且当次会话接下来要做真机测试时，见 [[feedback_sync_test_version_first]]——先只同步当次要测试的那个版本，其他版本等测试通过后再同步，不要一次性全部同步。纯代码修复、当次不安排真机飞行的情况，仍然可以三个版本一起同步。
