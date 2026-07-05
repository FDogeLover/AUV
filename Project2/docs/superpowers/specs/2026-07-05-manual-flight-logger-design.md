# 手动遥控飞行 + T265 数据记录脚本 — 设计

## 背景

需要支持"遥控器手动描杆飞行，T265 只作为凌霄IMU悬停抗漂移的外部速度参考"这种飞行方式，同时把飞行过程记录下来供后续分析（尤其是光流坐标对齐问题，人工手动飞行可能比自动导航更容易做出干净的单轴动作）。

飞控固件已支持这种模式：`User_Task.c:102` 里 `CH_7 > 1700 || received_data.task_sta==1` 触发"定位任务"状态机；如果 Python 侧完全不发指令帧（`AA 02`），`received_data.task_sta` 就保持默认值0，飞控不会进入这个状态机，完全由遥控器 CH_7/CH_8 物理开关控制飞行模式。`pi_ctrl_mode` 开机强制=1（`User_Task.c:26`），只要 Python 持续发送 T265 速度帧（`AA 01`），凌霄IMU的悬停抗漂移就能用上这个参考，跟飞控是否处于"定位任务"状态机无关。

## 目标

新增一个独立脚本，只做"接入T265 + 发送速度帧 + 记录日志"，不碰任何自动导航逻辑。

## 文件

`drone_control/basic/manual_flight_logger.py`（新文件，不修改 `Mission_GPT.py`/`main.py`/固件）

## 行为

1. 启动 T265（复用 `t265.py` 的 `t265_class`）
2. 打开飞控串口（复用 `Lcode/Lprotocol.py` 的 `Serial_fc`），启动下行监听线程 `listen_start(re_fc)`
3. 只启动 T265 速度发送线程：`serial_fc.send_start(t265_obj=realsense, vel_freq=100)`（不传 `comlist` 参数，所以 `_send_command_loop` 线程不会启动，飞控收不到任何 `AA 02` 指令帧）
4. 主循环（30ms 一次，跟现有 `Mission_GPT.py` 控制周期一致）：
   - 读取 T265 位置/速度、`re_fc` 里的 roll/pitch/of1速度/of状态/激光高度
   - 按 `FLIGHT_LOG_INTERVAL=0.05`（复用相同常量值，直接写死在这个新文件里，不从 `Mission_GPT.py` 导入，保持文件独立）节流写入日志
   - 日志字段跟 `flight_data.jsonl` 保持一致的命名风格：`t`, `pos`, `t265_vel`, `of1_vel_cms`, `roll_pitch`, `of_status`，但不需要 `state`/`target_idx`/`vx,vy,vyaw`/`target`/`height_setpoint_cm`（这些是任务状态机特有字段，手动飞行没有）
   - 写入独立文件 `flight_data_manual.jsonl`（不跟自动导航的 `flight_data.jsonl` 混用同一文件名，避免追加混淆）
   - 终端打印同样字段，方便实时看数据是否正常
5. `Ctrl+C` 停止：停 T265、关闭日志文件、`serial_fc.send_end()` + `serial_fc.close()`，正常退出（复用 `main.py` 里已经验证过的退出方式）

## 非目标

- 不发送任何指令帧（`AA 02`），不涉及 `task_sta`/`com_x/y/z/yaw`
- 不做起飞/降落确认、不检查T265置信度阈值（纯记录工具，安全飞行判断交给飞行员和遥控器本身）
- 不复用 `Mission_GPT.py` 的类结构（避免引入用不到的状态机字段），独立成一个简单的过程式脚本

## 验证

- 本地 `python -m py_compile manual_flight_logger.py` 确认语法正确
- 同步到 `ubuntu-pi` 后实际用遥控器飞一次验证：T265速度帧能正常发送（凌霄IMU悬停不漂移）、`flight_data_manual.jsonl` 正常写入、`Ctrl+C` 能正常退出
