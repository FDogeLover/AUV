---
name: project-board-resource-monitor
description: circle_pole新增ResourceMonitor(CPU/内存/温度)后台线程记录板载负载，已合并main并在真机验证
metadata: 
  node_type: memory
  type: project
  originSessionId: 747bf7b8-8b34-4a89-afd0-aa51c1b880fc
---

2026-07-15：给`drone_control/circle_pole`新增了板载资源监控功能，走完整的brainstorming→设计文档→实现计划→subagent-driven-development流程，合并进`main`。设计文档`docs/superpowers/specs/2026-07-15-board-resource-monitor-design.md`，实现计划`docs/superpowers/plans/2026-07-15-board-resource-monitor.md`。

**架构**：新增`Lcode/resource_monitor.py`的`ResourceMonitor`类，独立后台daemon线程，每1秒用`psutil`采样系统CPU%、本进程CPU%、系统内存、本进程RSS、CPU温度，写入`flight_data.jsonl`的`{"event": "resource", ...}`行——跟现有的位置/状态遥测共用同一个文件。因为两个线程会并发写同一文件，顺带给`Mission_GPT.py`里全部8处已有的日志写入点加了共享锁(`self._log_lock`)。

**接入`mission`生命周期时发现并修复的安全问题**：最初把阻塞式的`ResourceMonitor.stop()`(最多2秒)放在`stop_all()`最前面，但`stop_all()`是紧急停止路径(飞控超时/T265丢失)唯一入口，会让电机上锁指令延迟最多2秒——代码质量审查发现后已修复：`se_fc`上锁指令改成最先发出，`resource_monitor.stop()`挪到之后但仍在`_log_file.close()`之前(保证监控线程真正停止后才关文件，避免"写已关闭文件"竞态)。

**真机验证**：8核板子上，circle_pole本进程CPU占用均值213%(超2核，符合视觉+雷达+T265+主循环多线程并发预期)，最高235.7%，负载正常。

**2026-07-15当天连续4次真机测试(13:23~13:56)的负载趋势**：CPU占用高度稳定(208~215%，不随任务成功/失败变化)；本进程内存RSS稳定(113~118MB，4次测试波动仅几MB，**没有内存泄漏**)；但系统整体内存占用率在第2、3次测试之间跳了一个台阶(11.7%→15.7%，约260~280MB)且之后维持高位，时间点跟同期做的板子git仓库损坏修复(`git add -A`暂存1399个文件等大批量操作)吻合，**猜测是page cache效应，未验证**；CPU温度随4次连续测试从48~53℃一路涨到58~62℃，怀疑是缺少测试间隔导致的累积热效应，62℃仍在安全范围但值得留意——如果以后做更长时间连续测试，可能需要测试间隙留降温时间。

**How to apply**：以后分析`flight_data.jsonl`性能相关问题时，直接过滤`event=="resource"`的行，`t`字段跟位置遥测同一时间基准，可以按时间戳对齐叠加分析"哪个飞行阶段CPU/温度在飙"。

[[project_circle_pole_vision_servo_stage2_design]]
