# 计划：返航推进状态机修复与飞行循环异常保护

## 问题描述 & 目标

2026-07-20 15:28 A1实飞已经证明首返航点cruise到达生效：终端打印“航点0掠过”。随后所有NAVIGATE/LAND日志消失，仅ResourceMonitor继续写入，说明飞行daemon线程在航点回调中发生未捕获异常。

代码静态追踪已经定位确定根因：`InventoryFlightMission._advance_waypoint()`调用`coordinator.on_waypoint_arrived()`；返航TRANSIT到达时，`on_waypoint_arrived()`无条件调用`_go(InventoryState.TRANSIT)`。此时业务状态已经是`RETURN`，而允许转移表仅允许`RETURN → LAND/FAULT`，因此抛出：

```text
ValueError: 非法状态转移: RETURN -> TRANSIT
```

异常越过`navigate()`和`loop()`，杀死daemon飞行线程；串口发送线程继续重复最后指令，所以现场表现为悬停/卡住。本轮实际上没有进入LAND，新的`land_start`事件为0。

目标：

1. RETURN期间到达TRANSIT绕行点时保持业务状态为RETURN，只记录采样，不尝试非法转回TRANSIT。
2. 完整桌面走通：扫码失败→安装返航路线→两个TRANSIT点→LAND_APPROACH→LAND。
3. 主飞行循环任何状态tick异常都必须记录完整traceback、清零水平速度并进入受控LAND，不能再次静默死亡。
4. 若异常发生在LAND本身，避免无限重复异常，转为现有emergency/stop安全终止路径并记录原因。

## 方案选择

| 方案 | 优点 | 缺点/风险 |
|------|------|----------|
| A. 只修`RETURN→TRANSIT`非法转移 | 最小改动，解决已知根因 | 未来其他回调异常仍可静默杀死飞行线程 |
| B. 修状态机根因 + loop顶层异常边界 + 子类同步业务LAND | 同时消除本次根因和同类静默失效；日志可定位 | 涉及基类与盘点子类的异常契约，需要严格测试 |
| C. 把RETURN→TRANSIT加入允许转移表 | 改动最少 | 语义错误：安全返航途中业务状态不应退出RETURN，会破坏LAND_APPROACH判断 |

**选择方案B。** 不放宽状态机合法转移；RETURN是持续状态，经过中间通道点时保持RETURN。

## 改动范围

- `drone_control/warehouse_inventory/Lcode/inventory_controller.py`
  - `InventoryMissionCoordinator.on_waypoint_arrived()`：TRANSIT到达时，仅当当前业务状态不是RETURN才`_go(TRANSIT)`；RETURN期间只记录sample并返回ADVANCE。RETURN路线意外出现TAKEOFF kind时同样不做非法转移，但记录warning便于追溯规划器异常。
  - 覆盖飞行循环异常hook：先取消活跃扫码；在不影响飞行中性指令的前提下，尝试将业务状态同步到LAND。若业务状态已是FAULT则按允许表转LAND；已是RETURN则直接转LAND；已是LAND/END不重复转移。业务同步异常向上抛给基类第三级裸写保底，不吞掉。
  - 保持LAND_APPROACH在业务状态RETURN时返回LAND，不改现有语义。

- `drone_control/warehouse_inventory/Mission_GPT.py`
  - 将每轮状态分发包在顶层`try/except Exception`中，使用`logger.exception`保存完整traceback和状态。
  - 新增可覆盖的`on_flight_loop_exception(exc, failed_state)`，执行顺序固定：
    1. 先从`se_fc[5]`和当前heading状态取得安全快照，不依赖异常栈局部变量；
    2. 先调用`set_speed(0,0,bounded_yaw,current_z)`清零水平速度；
    3. 若`failed_state != LAND`，再切`state="LAND"`，下一tick进入正常受控降落；
    4. 若`failed_state == LAND`，先在lock内补写`se_fc[2]=0`及中性XY/yaw，确保一键降落已请求，再设置`emergency_stop=True`，下一tick进入现有stop路径。
  - loop捕获hook二次异常后执行**不依赖set_speed/coordinator/stop_all/T265/ResourceMonitor的第三级裸写保底**：
    - `logger.critical(..., exc_info=True)`记录hook失败；
    - 直接在lock内写`se_fc[2]=0`、XY/yaw中性、`se_fc[7]=101`；
    - lock/数组写本身也包独立try/except，失败后不再调用任何封装；
    - `finally`中无条件`task_running=False`，确保daemon不再以假活状态运行。
  - 日志新增结构化`flight_loop_exception`事件，记录原状态、异常类型/文本、导航purpose、target_index/target；日志写失败不得影响中性指令。

- `drone_control/warehouse_inventory/test_inventory_controller.py`
  - 根因回归：业务状态RETURN时到达TRANSIT，返回ADVANCE且状态仍为RETURN，不抛异常；RETURN期间意外TAKEOFF记录warning但不非法转移。
  - 完整动态返航集成必须使用真实`InventoryFlightMission + InventoryMissionCoordinator`（仅硬件依赖使用fake），通过直接驱动真实`navigate(pos,yaw)`而不是直接调用coordinator：测试显式配置`NavigationProfileConfig(profile="precision", cruise_confirm_cycles=2, cruise_radius_m=0.15)`；对每个TRANSIT目标连续喂入2帧距离<=0.15m且Z误差<0.20m的位置，确认触发真实cruise到达；最终LAND_APPROACH按precision所需窗口/停留时间使用受控时钟或直接构造已确认窗口后再由`navigate()`触发。索引0→1→2且业务状态始终RETURN，最终飞行状态LAND、索引不越界。
  - 子类loop异常hook测试：业务状态同步LAND、扫码取消、水平指令清零；业务同步再次抛异常时由基类第三级保底接管。

- `drone_control/warehouse_inventory/test_navigation_modes.py`
  - 普通NAVIGATE tick抛异常时：记录异常、先清零XY再转LAND，任务保持运行以进入下一tick受控降落。
  - LAND tick在一键降落写入前抛异常时：验证先补写`task=0`和中性指令，再触发emergency_stop。
  - 异常hook自身再次失败时：验证不调用stop_all/T265封装，直接裸写disarm且`task_running=False`。

- `.codex/CLAUDE.md`、`.codex/memory/project_warehouse_inventory_async_return.md`、`docs/known_issues.md`
  - 记录已确认根因`RETURN→TRANSIT`非法转移、修复和真机待验证状态。

## 风险点

- 安全隐患：异常后直接LAND前必须先清零XY，避免发送线程继续重放最后一次巡航速度；顺序固定为中性指令→state切换→业务同步。
- 安全隐患：不能把RETURN→TRANSIT加入允许转移，否则LAND_APPROACH会按非RETURN分支推进而不触发LAND。
- 安全隐患：LAND异常可能发生在一键降落指令写入前，必须先补写task=0及中性指令，再进入emergency，避免停T265前固件仍未接管降落。
- 安全隐患：异常处理器自己也可能失败；第三级保底不得调用coordinator、set_speed、stop_all、T265或资源监控，只允许尝试lock内裸写中性/disarm，并在finally无条件`task_running=False`。
- 边界条件：异常可能发生在TAKEOFF、NAVIGATE、SCAN、LAND任一状态；SCAN需取消worker，LAND异常不能无限重试。
- 边界条件：业务状态可能已是FAULT/LAND/END，异常hook不能再次执行非法状态转移。
- 回退方案：状态机根因修复可独立保留；若loop异常自动LAND真机表现异常，可回退hook策略为立即emergency_stop，但不能移除traceback记录和中性指令保底。

## 验证方式

- 单元测试：覆盖RETURN中间点状态保持、完整3点返航、普通tick异常、LAND异常、hook二次异常。
- 桌面测试：`python -m pytest drone_control/warehouse_inventory -q`全部通过。
- 日志回放：复现本轮首返航点`cruise_arrival`，验证不再抛`RETURN→TRANSIT`，产生waypoint_advance并切到第二目标。
- 真机验证：A1扫码超时后观察两个中间点均推进；最终必须出现`land_start`及明确`land_exit`或`land_wait_manual`。若发生任何新异常，必须在fc_log/flight_data中看到完整异常事件，不得只剩resource日志。
