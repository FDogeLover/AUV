# 03 关键流程与接口

## 3.1 固件：启动链路

- **入口 `main()`**
  - [main.c](file:///workspace/飞控固件/FcSrc/main.c#L22-L33)
  - 调用顺序：`All_Init()` → `Scheduler_Setup()` → `while(1) Scheduler_Run()`

- **整机初始化 `All_Init()`**
  - T265版：[Drv_BSP.c](file:///workspace/飞控固件/DriversBsp/Drv_BSP.c#L21-L89)
  - 典型动作：`DrvSysInit()`、LED/PWM/串口(1~5)/RC输入/ADC/数传/定时器/GPIO等

## 3.2 固件：周期运行与调度

### 3.2.1 软调度器（主循环）

调度器核心数据结构：

- `sched_task_t`：函数指针 + 频率 + 周期tick + 上次运行时间
  - [Ano_Scheduler.h](file:///workspace/飞控固件/FcSrc/Ano_Scheduler.h#L4-L10)
- `Scheduler_Setup()`：把 `rate_hz` 转换为 `interval_ticks = TICK_PER_SECOND / rate_hz`
  - [Scheduler_Setup](file:///workspace/飞控固件/FcSrc/Ano_Scheduler.c#L81-L95)
- `Scheduler_Run()`：读取 `GetSysRunTimeMs()`，到期则执行任务
  - [Scheduler_Run](file:///workspace/飞控固件/FcSrc/Ano_Scheduler.c#L97-L116)

系统时基：

- `TICK_PER_SECOND = 1000`（1ms tick）：[SysConfig.h](file:///workspace/飞控固件/FcSrc/SysConfig.h#L13-L16)

### 3.2.2 1ms 主任务（更偏实时）

在 STM32F407 平台上，通过 TIM7 中断直接驱动 `ANO_LX_Task()`：

- [stm32f4xx_it.c](file:///workspace/飞控固件/DriversMcu/STM32F407/Drivers/stm32f4xx_it.c#L59-L69)

`ANO_LX_Task()` 内部按 1ms/10ms/100ms 分层执行（T265版）：

- [ANO_LX_Task](file:///workspace/飞控固件/FcSrc/ANO_LX.c#L289-L341)

## 3.3 固件：核心数据结构（“关键类”）

### 3.3.1 遥控输入 `rc_in`

- `_rc_input_st`：PPM/SBUS 原始通道、信号频率、failsafe等
  - [Drv_BSP.h](file:///workspace/飞控固件/DriversBsp/Drv_BSP.h#L7-L33)

### 3.3.2 实时目标 `rt_tar`（下发给飞控控制器）

`_rt_tar_st`（0x41帧语义）：

- `rol/pit/thr/yaw_dps`
- `vel_x/vel_y/vel_z`：实时速度控制量（厘米/秒）
- 定义见 [ANO_LX.h](file:///workspace/飞控固件/FcSrc/ANO_LX.h#L37-L53)

典型赋值路径：

- RC摇杆 → `RC_Data_Task()` → `rt_tar`（并标记 `dt.fun[0x41].WTS = 1`）
  - T265版示例：[ANO_LX.c](file:///workspace/飞控固件/FcSrc/ANO_LX.c#L64-L185)

### 3.3.3 飞控状态 `fc_sta`

`_fc_state_st` 记录：

- 模式命令/状态、解锁命令/状态
- IMU就绪/起飞/在空/降落标记
- 定义见 [LX_FC_State.h](file:///workspace/飞控固件/FcSrc/LX_FC_State.h#L22-L41)

状态机执行入口：

- `LX_FC_State_Task(dT_s)`：[LX_FC_State.c](file:///workspace/飞控固件/FcSrc/LX_FC_State.c#L164-L174)

### 3.3.4 业务态聚合 `LX_FC`（T265版）

串口拓展板/上位机联动用的“飞行态快照”：

- `FC_data`：飞行状态、控制模式、位置/航向、电池等
  - 定义：[User_ComBoard.h](file:///workspace/飞控固件/FcSrc/User_ComBoard.h#L29-L55)

## 3.4 固件：通信链路（匿名协议 + 自定义帧）

### 3.4.1 匿名协议任务：`ANO_LX_Data_Exchange_Task()`

该函数被设计为“1ms调用一次”，内部做两件事：

1. CMD 校验回包重发（50ms周期、最多重发5次）
2. 检查各功能帧的定频发送（如 0x30/0x33/0x34/0x40/0x41/0x0d 等）

- [ANO_LX_Data_Exchange_Task](file:///workspace/飞控固件/FcSrc/ANO_DT_LX.c#L478-L525)

### 3.4.2 T265版：自定义上行帧 `t265_data_send()`

T265版在匿名协议基础上，构造 ID=0xF1 的“灵活数据帧”，用于把任务/视觉/控制输出等信息回传上位机：

- [t265_data_send](file:///workspace/飞控固件/FcSrc/ANO_DT_LX.c#L527-L580)

### 3.4.3 倾角保护版：`my_protocol` 灵活帧与Pi下行

倾角保护版提供 `Mycode/my_protocol.*`：

- **灵活帧发送**（Fletcher-8校验）
  - `flex_send()`：[my_protocol.c](file:///workspace/飞控固件/Mycode/my_protocol.c#L37-L61)
  - `flex_send_t265_vel()`（0xF1）与 `flex_send_guangliu_vel()`（0xF2）：[my_protocol.c](file:///workspace/飞控固件/Mycode/my_protocol.c#L63-L79)
- **Pi→FC 解析入口**
  - `pi_receive(u8 data)`：支持两类帧
    - 0x01：T265速度帧（vx/vy/yaw）
    - 0x02：飞行指令帧（task_sta、com_x/y/z/yaw、next_task、sp_side）
  - 见：[my_protocol.c](file:///workspace/飞控固件/Mycode/my_protocol.c#L92-L161)

## 3.5 Python：关键流程与协议

### 3.5.1 Pi 端主流程

- [main.py](file:///workspace/drone_control/main.py#L21-L46)
  - 初始化 T265：`t265_class()`
  - 初始化飞控串口 `/dev/ttyS6`，并启动：
    - 监听线程：解析飞控回传帧（任务模式 + x/y积分）
    - 发送线程：
      - 速度帧 100Hz
      - 指令帧 50Hz
  - 初始化地面站串口 `/dev/ttyS7`
  - 创建并启动任务状态机 `mission`

### 3.5.2 Pi ↔ 飞控：双帧协议（与倾角保护版 `pi_receive` 对齐）

飞控串口模块：`Serial_fc`：[Lprotocol.py](file:///workspace/drone_control/Lcode/Lprotocol.py#L8-L115)

- **速度帧（0x01）**：`AA 01 vx_h vx_l vy_h vy_l yaw_h yaw_l CK FF`
  - 生成逻辑：[Lprotocol.py](file:///workspace/drone_control/Lcode/Lprotocol.py#L57-L77)
  - CK 为从字节1开始 XOR
- **指令帧（0x02）**：`AA 02 task_sta com_x+sp com_y+sp com_z com_yaw+sp next_task sp_side CK FF`
  - 发送逻辑：[Lprotocol.py](file:///workspace/drone_control/Lcode/Lprotocol.py#L79-L91)
- **飞控回传帧**：`AA mode xH xL yH yL FF`（其中x/y为积分值偏移0x4000）
  - 解析逻辑：[Lprotocol.py](file:///workspace/drone_control/Lcode/Lprotocol.py#L30-L56)

### 3.5.3 Pi ↔ 地面站：协议 v2.0

地面站协议在文档中定义：

- [地面站通信协议.md](file:///workspace/drone_control/地面站通信协议.md#L1-L34)

实现侧：

- 下行（Pi→地面站）：`Serial_dmz.send_dmz()` 固定 100Hz 发送 5字节帧：[Lprotocol.py](file:///workspace/drone_control/Lcode/Lprotocol.py#L174-L183)
- 上行（地面站→Pi）：解析禁飞区坐标（AA + 3组(A,B) + FF）：[Lprotocol.py](file:///workspace/drone_control/Lcode/Lprotocol.py#L145-L173)

### 3.5.4 任务状态机（起飞→导航→检测→降落）

- 状态机实现：[Mission_GPT.py](file:///workspace/drone_control/Mission_GPT.py#L168-L209)
- 安全策略示例：
  - 飞控串口超时（2秒无回传）触发紧急降落：[Mission_GPT.py](file:///workspace/drone_control/Mission_GPT.py#L175-L179)
  - T265 数据采集线程停止触发紧急降落：[Mission_GPT.py](file:///workspace/drone_control/Mission_GPT.py#L182-L185)

