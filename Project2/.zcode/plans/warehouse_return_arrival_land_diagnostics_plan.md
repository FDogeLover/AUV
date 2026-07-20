# 计划：安全返航到达判定与 LAND 诊断

## 问题描述 & 目标

2026-07-20 A1 真机日志证明扫码超时结果已被消费并安装返航路线，无人机实际飞到首返航点 1.52 cm 内，但该点仍按 precision 的速度窗口/停留条件等待，最终以 timeout 结束。当前 `purpose=return` 的任意 timeout 都立即切换 `LAND`，因此第二返航点和 `LAND_APPROACH` 没有执行。

进入 `LAND` 后任务约一分钟仍未结束，随后人工中断。现有 LAND 日志包含位置、`unlock_sta`、`motor_pwm_mask` 等采样，但缺少明确的“进入原因、降落命令是否持续发送、字段新鲜度、确认进度、退出/永久等待原因”事件，且 `land_timeout_gaveup=True` 会有意跳过 Python 25 秒超时并永久等待人工介入，这可能解释一分钟不结束，但当前证据不足以确认。

目标：

1. 返航中间通道点使用巡航到达语义，不因 T265 速度噪声卡住。
2. 返航中间点超时时，若位置/高度已经足够近则推进下一点；只有仍明显偏离路线时才原地 LAND。
3. 最终 `LAND_APPROACH` 保持 precision，到达后由协调器切换 LAND；其超时仍按安全策略原地 LAND。
4. 为 LAND 增加可直接回答“为什么未结束”的结构化诊断，不改变已有解锁确认安全条件，也不擅自缩短人工介入等待。

## 方案选择

| 方案 | 优点 | 缺点/风险 |
|------|------|----------|
| 方案A：所有返航点继续 precision，仅在 timeout 时判断近/远 | 改动小 | 每个中间点仍可能白等完整 timeout，飞行效率差 |
| 方案B：返航中间点强制 cruise，末点 precision；timeout 再做近/远分流 | 中间点及时推进，末点仍精确；同时覆盖到达检测噪声和兜底 | 需要让航点模式、事件日志和 timeout 策略共享同一判断，避免语义不一致 |
| 方案C：返航全部 cruise | 最简单、最快 | `LAND_APPROACH` 可能带速度触发 LAND，不符合降落安全要求 |

**选择方案B。** 返航中间点本质是绕障通道点，适合圆形半径+连续周期确认；最后的降落接近点必须保持 precision。timeout 兜底使用同一个巡航半径并额外检查 Z，避免“XY很近但高度不安全”时推进。

## 改动范围

- `drone_control/warehouse_inventory/Mission_GPT.py`
  - 新增统一的 `_waypoint_mode()`：当模块级安全开关 `RETURN_CRUISE_ENABLED=True`、`_navigation_purpose == "return"` 且不是最后一个目标时返回 `cruise`，最后一个返航目标返回 `precision`；普通导航保持现有 profile 行为。该开关不开放为环境变量，避免飞行时被隐式配置；回退版本只需改为False。
  - `navigate()`、`_log_waypoint_event()` 改用统一模式，事件增加 `navigation_purpose`，避免控制与日志模式不一致。
  - 返航中间点的 cruise 到达始终要求 Z 合格，不受普通 profile 的 `cruise_require_z=False` 影响；XY半径和连续周期数复用 `NavigationProfileConfig.cruise_radius_m/cruise_confirm_cycles`，不增加另一套隐藏参数。
  - 新增返航 timeout 分类函数，但不直接修改索引：
    - 仅中间点、当前 T265 `confidence >= 2`、`arrival_distance <= cruise_radius_m` 且 `abs(z-target_z) < Mission_GPT.py` 现有模块常量 `posthreshold_z` 时，将原因转换为 `return_timeout_near` 并继续调用多态 `_advance_waypoint()`；
    - 中间点仍远、Z不合格或confidence<2时，以 `return_timeout_far` 切换 LAND；
    - 最后返航点 timeout 仍切换 LAND。
  - cruise正常到达和`return_timeout_near`都不绕过子类/coordinator私自修改`target_index`，索引推进仍只发生在基类 `_advance_waypoint()` 的正常 ADVANCE 路径中。
  - 用单个显式策略门控同时控制“返航中间点 cruise”和“timeout-near 推进”；回退时两项一起恢复为全 precision + 任意返航 timeout LAND。
  - 在 `land()` 第一次执行时统一记录一次结构化 `land_start` 事件，从而覆盖返航超时、LAND_APPROACH、扫码追踪丢失、SCAN异常、起飞中止和航点耗尽等所有LAND入口。
  - LAND 周期日志补充：基于 `time.monotonic()` 的 `land_elapsed_s`、`laser_height_m`、`laser_height_valid`、`unlock_confirm_count`、`motor_pwm_ok`、`motor_pwm_age_s`、`land_timeout_gaveup`、当前发送的 task/z/yaw 命令。时间戳缺失或异常时age为None且不影响确认。
  - LAND 检测到固件 gaveup 时记录一次结构化 `land_wait_manual` 事件；正常确认与 Python 超时分别记录 `land_exit` 事件及 reason。
  - 不修改 `unlock_sta == 0 && motor_pwm_mask == 0/None` 的确认逻辑，不修改 `land_timeout_gaveup` 后保持串口/T265并等待人工介入的安全行为。

- `drone_control/warehouse_inventory/Lcode/inventory_controller.py`
  - 保持 `InventoryFlightMission._advance_waypoint()` 为返航到达动作的协调入口；`return_timeout_near` 必须原样传入 `coordinator.on_waypoint_arrived()`。
  - `on_waypoint_arrived()` 当前只把reason写入sample，不按reason分支；因此无需新增特殊白名单。`return_timeout_near`作为审计标签原样透传，返航TRANSIT仍按`WaypointKind.TRANSIT`返回ADVANCE。
  - 返航 TRANSIT 由 coordinator 返回 ADVANCE 后，再调用基类原子推进索引；不得由基类判定函数绕过 coordinator。若coordinator返回LAND或其他非ADVANCE动作，索引必须保持不变并执行该动作。
  - 确认 LAND_APPROACH 仍由 coordinator 在 RETURN 状态下返回 LAND，不能因 cruise override 被提前推进。

- `drone_control/warehouse_inventory/test_navigation_modes.py`
  - 新增返航中间点使用 cruise、末点保持 precision 的测试。
  - 新增首返航点在 1.52 cm 内能够快速推进的测试。
  - 新增中间点 timeout 近距离推进、距离0.16m时LAND、Z偏差0.21m时LAND、confidence=1时LAND、末点 timeout LAND 的边界测试。
  - 验证 waypoint 事件记录的模式、`navigation_purpose`与实际控制一致。

- `drone_control/warehouse_inventory/test_inventory_controller.py`
  - 新增集成测试：返航中间点 `return_timeout_near` 只调用 coordinator 一次，coordinator 返回ADVANCE后索引只增加一次，且 `inventory_route`、`coordinator.route`、`targets` 保持一致。
  - 新增负路径测试：coordinator返回LAND或其他非ADVANCE动作时索引不变、不二次调用，并执行对应安全动作。
  - 验证最终LAND_APPROACH仍由coordinator切换LAND。

- `drone_control/warehouse_inventory/test_land_logging.py`
  - 扩展 fake serial 支持 `motor_pwm_mask_t`、`land_timeout_gaveup`。
  - 测试 `land_start`、周期诊断字段、正常确认退出原因、Python timeout 退出原因和 gaveup 人工等待事件。
  - 验证 gaveup 状态仍不会走 Python 强制退出，防止诊断改动破坏安全语义。

- `.codex/CLAUDE.md`、`.codex/memory/project_warehouse_inventory_async_return.md`、`docs/known_issues.md`
  - 实现和测试完成后更新问题状态、诊断字段与下一轮真机验证要求。


## 风险点

- 安全隐患：返航中间点过早推进可能切角靠近货架。控制使用已有 `cruise_radius_m=0.15m` 和连续周期确认；timeout近距离兜底还要求当前confidence>=2、XY半径和Z均合格，不放宽到更大范围。
- 安全隐患：最终降落点不能使用 cruise，必须明确保留 precision，并继续经由coordinator识别LAND_APPROACH后切LAND。
- 安全隐患：LAND 的 `land_timeout_gaveup=True` 表示固件认为高度仍偏高并放弃自动锁桨；Python 必须继续维持通信等待人工介入，不能为了“任务能退出”而强制 END。
- 边界条件：只有一个返航目标时，它既是首点也是末点，应使用 precision。
- 边界条件：`motor_pwm_mask_t` 缺失、未来时间或时钟域不兼容时 age 记录为 None，不能影响确认逻辑。
- 边界条件：日志文件不可用/写失败不能阻塞 LAND。
- 回退方案：用一个显式策略门控同时控制返航中间点 cruise 与 timeout-near 推进；关闭后完整恢复“返航全precision + 任意timeout原地LAND”。诊断字段均为只读附加，可独立保留。

## 验证方式

- 单元测试：运行新增的返航边界测试和 LAND 诊断测试。
- 全量桌面测试：`python -m pytest drone_control/warehouse_inventory -q`，要求现有测试全部通过。
- 日志回放：用 A1 实飞终点 `pos=(-2.6369,0.0703,1.40)`、`target=(-2.65,0.0625,1.40)` 验证中间点能够推进；同时用距离0.16m、Z偏差0.21m和confidence=1的反例验证仍进入LAND。
- 真机验证分两阶段：
  1. 不带二维码变量的最小动态返航测试，确认两个通道点连续推进、最终接近点 precision 后进入 LAND；
  2. A1 扫码失败返航测试，检查 `land_start`、LAND周期字段和明确的 `land_exit`/`land_wait_manual` 原因。
- 真机安全判据：中间点距离仍大于巡航半径或高度不合格时 timeout 必须原地 LAND；任何异常均不得跳过最终 precision 约束。
