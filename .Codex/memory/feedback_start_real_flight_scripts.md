---
name: feedback-start-real-flight-scripts
description: 实机飞行测试时，由Claude通过SSH启动main.py等脚本，不是用户自己启动
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 3e206fa0-fe8a-47e7-a368-627b97bcf0c9
---

真机测试（`main.py`/`manual_flight_logger.py` 等）由 Claude 通过 SSH 在 `ubuntu-pi` 上启动，用户负责物理操作（站在飞机旁、遥控器在手、随时接管），不是用户自己去板子上敲命令启动。

**Why:** 2026-07-08 用户明确说"为了方便数据和日志读取，都是你替我启动"——Claude 启动脚本能直接看到实时输出/日志，省得用户念给它听或者事后传日志文件，配合效率更高。

**How to apply:** 用户说"准备好了"/"可以启动"这类话时，直接 SSH 上去启动对应脚本（一般用 `run_in_background`，方便持续读输出）。**但涉及解锁/起飞这种有物理风险的动作，启动前仍然要按 [[feedback_flight_test_safety_confirmation]] 做一次明确的起飞前安全确认**（人在不在旁边、遥控器在不在手），这条记忆只是说"谁来敲命令"，不改变安全确认这一步本身。
