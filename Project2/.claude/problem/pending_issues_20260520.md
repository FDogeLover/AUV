# 代码检查 - 待后续处理问题汇总

> 更新时间：2026-05-20
> ✅ 已修复并提交的变更（commit range）：
> - TIM8 PWM预装载 bug → 飞控固件 Drv_PwmOut.c
> - ESC_Output 浮点→整数运算优化 → FcSrc/ANO_LX.c
> - t265 重复实例化移除 + 串口看门狗启用 → drone_control/Mission_GPT.py
> - AnimalDetectApp 公共基类提取 (detector_base.py) → k230/animal_detect_visual.py + animal_detect_yolov8n.py
> - UART 线程安全修复 (exit_event + write_lock) → k230/animal_detect_yolov8n.py
> - Serial_gpio 死代码删除 → drone_control/Lcode/Lprotocol.py
> - test_simulation.py 废弃 → drone_control/test_simulation.py
> - fc_log.log 清理 → drone_control/fc_log.log
> 📌 以下为**仍未处理**的剩余问题。

---

## 📍 ANO_LX_FC_倾角保护版（飞控固件）

### 中等问题

**M1. 全局变量 pi_ctrl_mode 分散引用**
- 定义在 `FcSrc/ANO_LX.c`（L314），被 `LX_FC_EXT_Sensor.c`、`User_Task.c` 等多处引用
- 影响：修改困难，缺乏集中管理
- 建议：统一为枚举类型或结构体管理
- 涉及文件：`FcSrc/ANO_LX.c`、`FcSrc/LX_FC_EXT_Sensor.c`、`FcSrc/User_Task.c`

**M2. 串口数据轮询效率（DrvUartDataCheck）**
- 在1ms中断中轮询5个UART缓冲区
- 数据量大时可能影响中断响应
- 涉及文件：`DriversMcu/<平台>/Drivers/Drv_Uart.c`

**M3. SBUS帧校验码轮询方式效率较低**
- 直接轮询数组比较 0x00/0x04/0x14/0x24/0x34
- 可直接位运算判断
- 涉及文件：`DriversMcu/<平台>/Drivers/Drv_RcIn.c`

### 架构建议

**A1. Mycode 与 FcSrc 耦合**
- `Mycode/my_protocol.c` 直接修改 `ANO_LX.c` 的全局变量
- 建议将 `angle_protect.c` 整合进 FcSrc，减少目录间耦合
- 涉及文件：`Mycode/`、`FcSrc/`

**A2. 飞控状态机缺少异常恢复路径**
- 当前状态机缺少异常复位逻辑
- 建议增加超时/错误恢复路径
- 涉及文件：`FcSrc/LX_FC_State.c`

---

## 📍 drone_control

### 严重

**~D-Sev1. test_simulation.py 引用已移除功能，完全无法运行~~ ~~✅ 已废弃删除~~
- 文件引用了 `m.detecting`, `m.grid_results`, `_grid_from_real()`, `ANIMAL_LABELS` 等已删除的变量/方法
- **处理：已直接删除该文件**
- 涉及文件：`drone_control/test_simulation.py` — ~~已删除~~

### 中等

**D-Mid1. 串口路径硬编码**
- `/dev/ttyS6`, `/dev/ttyS7`, `/dev/ttyS3` 均写死在 `main.py` 和 `Lprotocol.py`
- Windows上无法测试，也缺乏配置文件机制
- 建议：通过环境变量或配置文件传入
- 涉及文件：`drone_control/main.py`、`drone_control/Lcode/Lprotocol.py`

**~D-Mid2. Serial_gpio 类完全未使用（死代码）~~ ~~✅ 已删除~~
- `Lprotocol.py` L188-L238 定义了完整功能但在任何生产代码中未被引用
- **处理：已直接删除该类的全部代码**
- 涉及文件：`drone_control/Lcode/Lprotocol.py` — ~~已删除~~

**D-Mid3. 缺少串口断线重连机制**
- 任何串口断开导致永久性故障
- 建议：加入自动重连逻辑

~~**D-Mid4. 锁使用不统一**~~ ✅已修复
- 混用 `lock.acquire()/release()` 和 `with lock:`
- 建议：全部改为 `with lock:` 语法

~~**D-Mid5. 缺少飞行数据日志**~~ ✅已修复
- 只有普通日志，没有位置/PID输出/状态转换的结构化记录
- 建议：在 mission.loop 中定期记录结构化飞行数据

~~**D-Mid6. Logger.py fileHdl/consoleHdl 污染 logger 命名空间**~~ ✅已修复
- 直接作为属性赋值给 logger 对象
- 建议使用局部变量

~~**D-Mid7. 多线程竞争 re_fc 读取**~~ ✅已修复
- 主线程读 `re_fc` 时无锁保护
- 可能导致读到部分更新的缓冲区
- 涉及文件：`drone_control/main.py`、`Lcode/Lprotocol.py`

---

## 📍 T265 定位精确度分析

> 分析时间：2026-05-20
> 整体评分：XY 7/10 | Z轴 3/10 | 速度 5/10 | 鲁棒性 4/10 | 整体 5/10

### 核心问题

**T1. Z轴高度伪问题（最严重）**
- T265 是视觉 SLAM，只在水平面建图，**不提供垂直方向定位能力**
- `t265.py` L36-41 坐标变换仅处理 XY 平面
- `Mission_GPT.py` pos[2] 实际是 T265.y 值，不是真实高度
- 飞行高度由飞控气压计隐式控制，上位机不知道真实高度值

**T2. 航点到达阈值偏短**
- arrival_confirm_need = 5（~150ms）太短，受 T265 高频噪声影响易误判"到达"
- T265 追踪置信度 < 2 时直接丢弃数据使用旧值，长时间低置信度会导致定位"冻结"

**T3. 低通滤波过激进造成相位滞后**
- t265.py alpha=0.3 等效截止频率 ~4Hz，位置跟踪有明显滞后
- PID 响应不够跟手，速度积分漂移明显（即使速度噪声 0.02m/s，积分 10s 产生 0.2m 偏差）

**T4. 环境依赖性强**
- 最低照度约 2 lux，暗室失效
- 每视野至少 2000 个特征点，平滑地面（白瓷砖、水面）完全丢失
- 重复纹理（格子地板）会引起地图混淆
- 飞行振动导致图像模糊，降低特征点数量

### P0 — 已实施 ✅

| # | 改动 | 原值 → 新值 | 说明 |
|---|---|---|------|
| T-P0a | 增大确认窗口 | `arrival_confirm_need: 5 → 15` (~450ms) | 避免单帧噪声误判，已在 `Mission_GPT.py` L20 完成 |
| T-P0b | 动态阈值 | 置信度3→0.10m, 2→0.15m, 1→0.30m | 按追踪置信度自适应，已在 `Mission_GPT.py` L258-264 完成 |
| T-P0c | 降低滤波系数 | `t265.py alpha: 0.3 → 0.15` | 减少相位滞后，截止频率~8Hz，已完成 |
| T-P0d | 缩短超时 | `arrival_timeout_max: 10s → 5s` | 定位丢失快速恢复，已在 `Mission_GPT.py` L21 完成 |

### P1 — 部分实施

| # | 改动 | 状态 | 说明 |
|---|---|------|------|
| **T-P1a** | **融合飞控激光测距高度** ⭐最关键 | ✅ **已实施** | 协议扩展：pi_send() 帧增加4字节激光高度（小端序，cm单位），Lprotocol.py 解析并缓存，loop() 中用激光高度覆盖 pos[2]（替代 T265 伪 Z 数据）；涉及文件 `my_protocol.c`、`Lprotocol.py`、`Mission_GPT.py`、`main.py` |
| T-P1b | EKF替代低通滤波 | 📌 延后 | 高复杂度，先观察 P0+P1a 效果 |
| **T-P1c** | **置信度降级策略** | ✅ **已实施** | 置信度==0时悬停（速度清零不等超时），已在 `Mission_GPT.py` L236-242 完成；新增 `t265_class.get_tracking_confidence()` 方法 |

### P2 — 可选升级（硬件层面）

| # | 改动 | 成本 | 效果 |
|---|---|---|---|
| T-P2a | ~~底部 TOF 测距模块~~ ✅ **已解决**：飞控固件已通过光流串口接收激光测距高度，经 `ano_of.of_alt_cm` 回传给上位机 | — | — |
| T-P2b | 减振优化（弹簧减振垫） | ¥10-30 | 提升追踪置信度 1+ 级 |

### 关键文件

- T265 数据采集/坐标变换：`drone_control/t265.py`
- 导航参数/到达判断：`drone_control/Mission_GPT.py`
- 飞控速度帧发送：`drone_control/Lcode/Lprotocol.py`
- 飞控PID参数：`drone_control/Lcode/Lpid.py`

### P1 — 建议优先修复

**K1. 文件名误导**
- `animal_detect_yolov8n.py` 实际是 Pi 双向通信专用版
- 建议改名：`animal_detect_yolov8n.py` → `animal_detect_pi_protocol.py`

**K2. animal_detect_yolov8n.py 缺少 os.exitpoint()**
- 主循环中没有调用，IDE 可能无法优雅中断
- dataset_capture.py 和 animal_detect_visual.py 都有

**K3. UART write() 无完整性检查**
- 未校验写入字节数是否等于帧长度
- Pi端可能收到格式错误的帧
- 涉及文件：`k230/animal_detect_yolov8n.py` L326

**K4. get_uart_data 仅保留最高置信度类别**
- 多种类同时出现时合并到最高置信度类别
- 多动物场景信息丢失

**K5. kmodel 路径硬编码且不一致**
- visual版: `/sdcard/examples/mycode/animal_yolov8n_v2_best.kmodel`
- yolov8n版: `new_animal_v2.kmodel`
- 建议统一为可配置参数或环境变量

### P2 — 可选改进

**K6. dataset_capture.py save_jpg() 无写入校验**
- TF卡已满时会静默写入空文件
- 后续索引扫描可能出错

**K7. animal_detect_visual.py 缺少 RGB LED 错误指示**
- dataset_capture.py 有完整的 `led_pulse_n` 错误指示模式
- 建议复用该模式

**K8. draw_result 坐标映射精度问题**
- `int(round(v, 0))` 等价于 `int(v)`
- 负坐标时 `int(-0.6)=0` 而非 `-1`
- 影响极小（1像素偏差），但需注意

**K9. k230_client.py 在 drone_control/Lcode/ 而非 k230/ 下**
- 虽然是合理设计（Pi端代码归Pi端），但不熟悉项目的人可能找不到
- 建议在 k230/ 目录下加 README 说明协议定义位置
