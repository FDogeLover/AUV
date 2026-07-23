# 02 目录与模块

## 2.1 飞控固件（C）目录结构

两套固件工程均采用相同的三层结构：

- `FcSrc/`：飞控业务层（调度/状态机/通信/外设业务适配）
- `DriversBsp/`：板级组合层（整机初始化与“跨外设”封装）
- `DriversMcu/`：MCU层（不同芯片平台的外设驱动与第三方库）
- `ProjectSTM32F407/ProjectMSP432/ProjectTM4C123/`：工程文件（Keil uVision）

以下以 T265 版为例列举核心模块（倾角保护版同名文件职责一致，但可能缺失某些 `User_*` 模块）：

### 2.1.1 `FcSrc/`（业务层）

- **启动入口**
  - `main.c`：入口 `main()`，调用 `All_Init()` + `Scheduler_*()`：[main.c](file:///workspace/ANO_LX_FC_T265代替光流/FcSrc/main.c)
- **任务调度**
  - `Ano_Scheduler.h/.c`：软调度器与任务表（多频任务：1000/500/200/100/50/20/2Hz）：[Ano_Scheduler.c](file:///workspace/ANO_LX_FC_倾角保护版/FcSrc/Ano_Scheduler.c#L67-L116)
- **飞控主任务与数据结构**
  - `ANO_LX.h/.c`：RC→目标量、通信交换触发、电机输出、LED驱动；包含协议数据结构（RC/实时目标/姿态/速度/电池等）：[ANO_LX.h](file:///workspace/ANO_LX_FC_T265代替光流/FcSrc/ANO_LX.h#L23-L140)
- **飞控状态机**
  - `LX_FC_State.h/.c`：解锁/上锁、校准触发、起飞/在空/降落等状态标记与处理：[LX_FC_State.h](file:///workspace/ANO_LX_FC_T265代替光流/FcSrc/LX_FC_State.h#L22-L51)
- **飞控命令集合（对外能力）**
  - `LX_FC_Fun.h/.c`：一键起飞/降落/返航、模式切换、各类校准命令封装：[LX_FC_Fun.h](file:///workspace/ANO_LX_FC_T265代替光流/FcSrc/LX_FC_Fun.h#L15-L27)
- **外部传感器抽象**
  - `LX_FC_EXT_Sensor.h/.c`：统一封装速度/位置/测距/GPS等“外部传感器数据”，供飞控内部消费与对外发送：[LX_FC_EXT_Sensor.h](file:///workspace/ANO_LX_FC_T265代替光流/FcSrc/LX_FC_EXT_Sensor.h#L8-L85)
- **匿名通信协议（数据交换/命令/参数/校验）**
  - `ANO_DT_LX.h/.c`：功能帧管理、CMD/CK/PAR交互、定频发送调度；T265版还实现 `t265_data_send()` 自定义上行帧：[ANO_DT_LX.h](file:///workspace/ANO_LX_FC_T265代替光流/FcSrc/ANO_DT_LX.h#L9-L68)

#### T265版专有 `User_*` 模块（业务定制）

- `User_T265.h/.c`：T265 串口数据获取与坐标转换、向T265回传数据：[User_T265.h](file:///workspace/ANO_LX_FC_T265代替光流/FcSrc/User_T265.h#L20-L61)
- `User_Ctrl.h/.c`：位置/高度/角度/视觉对齐/绕杆/巡线等控制律与参数（多PID参数与输出叠加）：[User_Ctrl.h](file:///workspace/ANO_LX_FC_T265代替光流/FcSrc/User_Ctrl.h#L21-L183)
- `User_Task.h/.c`：航点数组、标定点、任务状态机（“一键任务”）：[User_Task.c](file:///workspace/ANO_LX_FC_T265代替光流/FcSrc/User_Task.c#L35-L81)
- `User_ComBoard.h/.c`：串口拓展板数据（飞控状态/货物/舵机/播报等）与收发：[User_ComBoard.h](file:///workspace/ANO_LX_FC_T265代替光流/FcSrc/User_ComBoard.h#L29-L55)
- `User_Opmv.h/.c`：OpenMV 视觉数据协议与结构体（杆/线/呼啦圈等）：[User_Opmv.h](file:///workspace/ANO_LX_FC_T265代替光流/FcSrc/User_Opmv.h#L30-L70)
- `User_RC.h/.c`：用户级遥控处理（急停、模式切换、一键降落等）：[User_RC.h](file:///workspace/ANO_LX_FC_T265代替光流/FcSrc/User_RC.h#L15-L19)
- `User_Oled.h/.c`、`User_Gpio.h/.c`：显示与IO（如保留IO、激光/舵机/蜂鸣器等控制，具体见实现文件）

### 2.1.2 `DriversBsp/`（板级组合层）

- `Drv_BSP.c/.h`：整机初始化 `All_Init()`、RC输入（PPM/SBUS）解码、信号质量检测等：[Drv_BSP.c](file:///workspace/ANO_LX_FC_T265代替光流/DriversBsp/Drv_BSP.c#L21-L89)
- `Drv_AnoOf.c/.h`：匿名光流相关数据与状态（在 `ANO_LX_Task()` 中 `AnoOF_Check_State()` 调用）
- `Drv_UbloxGPS.c/.h`：Ublox GPS 数据处理（由 `GPS_Data_Prepare_Task()` 驱动）
- `Ano_Math.c/.h`：数学函数（如 `my_deadzone/my_sqrt/my_cos` 等被上层大量使用）

### 2.1.3 `DriversMcu/`（MCU层）

按平台拆分，整体思路是“统一上层接口 + 不同MCU的具体实现”：

- `STM32F407/Drivers/Drv_*.c`：PWM/RC/UART/ADC/Timer/LED等外设驱动（并含 `stm32f4xx_it.c` 中断向量）。
- `STM32F407/Libraries/`：CMSIS、StdPeriph、USBStack 等第三方库源码。
- `MSP432P401/Drivers/ti/.../driverlib/`：TI driverlib（同时提供 Keil/CCS/GCC/IAR 的构建入口）。
- `TM4C123/`：TI TM4C 的库与驱动封装。

## 2.2 `Mycode/`（倾角保护版专有）

倾角保护版额外提供自定义协议与保护逻辑：

- `my_protocol.h/.c`：两类串口协议
  - Pi→FC：T265速度帧（0x01）与飞行指令帧（0x02）解析入口 `pi_receive()`：[my_protocol.c](file:///workspace/ANO_LX_FC_倾角保护版/Mycode/my_protocol.c#L92-L161)
  - FC→外设/观察：灵活帧 `flex_send_*()`（Fletcher-8校验）：[flex_send](file:///workspace/ANO_LX_FC_倾角保护版/Mycode/my_protocol.c#L37-L61)
- `angle_protect.h/.c`：倾角补偿/保护（被 `my_protocol` 引用：[my_protocol.h](file:///workspace/ANO_LX_FC_倾角保护版/Mycode/my_protocol.h#L1-L5)）

## 2.3 `drone_control/`（Python 控制端）

该目录是树莓派侧“任务系统 + 通信系统”：

- `main.py`：串口初始化（飞控+地面站）、创建并启动任务：[main.py](file:///workspace/drone_control/main.py#L21-L46)
- `Lcode/`
  - `Lprotocol.py`：串口协议（飞控/地面站/GPIO），并发线程发送与监听：[Lprotocol.py](file:///workspace/drone_control/Lcode/Lprotocol.py#L57-L115)
  - `Lpid.py`：PID 控制器（用于航点导航速度控制）
  - `coverage_planner.py`：覆盖规划（结合禁飞区生成航线）
  - `k230_client.py`：与K230视觉板串口交互
  - `global_variable.py`：全局共享变量（含锁、偏移量、串口超时等）
- `Mission_GPT.py`：核心任务状态机（起飞→导航→检测→降落）与安全策略（飞控超时/T265停止则急停）：[Mission_GPT.py](file:///workspace/drone_control/Mission_GPT.py#L168-L186)
- `t265.py`：T265 读取与坐标变换，支持真实设备与模拟模式（缺失 `pyrealsense2` 时降级）：[t265.py](file:///workspace/drone_control/t265.py#L7-L19)
- `地面站通信协议.md`：地面站链路协议 v2.0（帧格式、去重逻辑、上行禁飞区格式等）：[地面站通信协议.md](file:///workspace/drone_control/地面站通信协议.md)

