# 2026-07-22 warehouse_inventory 通用改进同步到 basic

## 同步范围

本次按风险从低到高，将货架盘点版本中与赛题业务无关的通用改进同步到
`drone_control/basic/`：

1. `Lcode/Lprotocol.py`
   - 保存监听、T265发送和命令发送线程引用。
   - `listen_end()`使用`cancel_read()`唤醒阻塞读取并限时等待线程退出。
   - `close()`先停止全部工作线程，再关闭串口，且可重复调用。
   - 串口读写兼容处理`SerialException`、`OSError`和关闭竞态的`TypeError`。
2. `Lcode/global_variable.py`与`Lprotocol.py`
   - 新增`fc_last_rx_monotonic`和`fc_frame_counter`。
   - 飞控yaw、墙钟时间、单调时间戳和帧号在同一把锁内提交，避免读取到
     “新yaw + 旧时间戳”的撕裂快照。
3. `Lcode/navigation_profile.py`与`Mission_GPT.py`
   - 新增可选环境变量`DRONE_CRUISE_REQUIRE_Z`。
   - 开启时，中间巡航航点除满足水平半径外，还必须满足现有Z轴到达门槛。
   - 默认值为关闭，不改变`basic`原有掠过式巡航行为。
4. `Lcode/heading_hold.py`
   - runaway只在航向修正持续达到配置上限、误差仍继续增长时触发。
   - 比例控制尚处于低于上限的输出阶段不再误判为控制失效。

## 明确未同步

- 不同步货架QR、云台、激光指示、地面站和A/B/C/D路线逻辑。
- 不同步货架任务的航向恢复状态机、自动重锁或“故障后继续盘点”策略。
- 不启用FC yaw反馈源；`basic`继续使用T265航向。
- 不改变默认航向修正上限，仍为`1°/s`。

## 验证

- 新增`test_lprotocol_shutdown.py`，覆盖阻塞监听安全关闭和幂等关闭。
- 新增`test_lprotocol_atomic_rx.py`，覆盖飞控yaw、单调时间戳及帧计数更新。
- 补充巡航Z门槛和航向runaway饱和条件测试。
- `python -m pytest -q`：`98 passed, 1 skipped`。
- `python -m compileall -q Lcode Mission_GPT.py`通过。
- `git diff --check`通过。

本次只完成桌面回归，没有因为参数性质或通用基础设施同步额外安排真机飞行。
