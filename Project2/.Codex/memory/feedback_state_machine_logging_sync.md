---
name: feedback-state-machine-logging-sync
description: 无人机任务状态机(Mission_GPT.py)新增/修改nav_mode子状态时，必须同步给该状态补JSON飞行日志，不能只顾业务逻辑
metadata: 
  node_type: memory
  type: feedback
  originSessionId: b60bc9e1-90b8-4bba-a23e-19268b2fd903
---

给`Mission_GPT.py`的任务状态机加新状态(或新的`nav_mode`子状态)时，必须同步检查该状态在`navigate()`主循环里有没有写JSON飞行日志(`self._log_file`)，不能只写业务逻辑就完事。

**Why**：fire_patrol开发时，`APPROACH`/`CONFIRM_WARN`/`HOVER_DROP`这几个新增子状态最初都在`navigate()`里提前`return`，完全跳过了原有PATROL态才有的JSON日志写入代码块。2026-07-16第一次真机测试时想诊断"为什么恢复PATROL后航点全部超时跳过"，结果这段时间(APPROACH+HOVER_DROP的十几秒)日志里完全是空的，只能靠终端print摘要还原大概时间点，没法拿到逐帧位置/速度数据做真正的定量分析。后续测试(第3次)想诊断"火情为什么在返程才识别到"、"APPROACH视觉伺服是否发散"，同样发现PATROL态的检测结果(`dx_px`/`dy_px`)和APPROACH阶段的逐帧数据从来没被记录过，两次都是先测试完才发现日志缺口，只能留到下次测试。

**How to apply**：状态机加新状态时，检查清单：
1. 这个状态会运行多久？如果超过几个tick(不是单次触发就转移到下一状态那种)，就需要跟其它主循环状态一样的节流JSON日志(参照`FLIGHT_LOG_INTERVAL`节流写法)
2. 这个状态依赖的关键判断量(比如视觉检测的`dx_px`/`dy_px`、传感器读数)有没有被记下来——事后光看"进入了这个状态"/"超时了"这两条摘要日志，做不了定量诊断
3. 多个状态各自的日志节流时间戳最好独立(不要共用同一个`_last_log_time`)，否则不同状态的日志块会互相抢节流窗口、变相拉低各自的采样率(fire_patrol里为此专门加了`_last_detect_log_time`区分于原有的`_last_log_time`)
4. 日志写入本身要有try/except包裹，不能因为日志失败影响飞行主循环

不要等到真机测试完事后才发现"这段时间完全没数据"，加状态的同时就该问自己"这段时间出问题了我拿什么诊断"。
