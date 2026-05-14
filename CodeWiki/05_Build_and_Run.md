# 05 构建与运行

## 5.1 固件构建（Keil uVision）

仓库提供三套目标MCU工程（两套固件工程目录内各一份）：

- STM32F407：[ProjectSTM32F407/ANO_LX_STM32F407.uvprojx](file:///workspace/ANO_LX_FC_T265代替光流/ProjectSTM32F407/ANO_LX_STM32F407.uvprojx)
- MSP432：[ProjectMSP432/ANO_LX_MSP432.uvprojx](file:///workspace/ANO_LX_FC_T265代替光流/ProjectMSP432/ANO_LX_MSP432.uvprojx)
- TM4C123：[ProjectTM4C123/ANO_LX_TM4C123.uvprojx](file:///workspace/ANO_LX_FC_T265代替光流/ProjectTM4C123/ANO_LX_TM4C123.uvprojx)

### 5.1.1 STM32F407（推荐从这里开始）

1. 打开工程：`ProjectSTM32F407/ANO_LX_STM32F407.uvprojx`
2. 目标芯片：`STM32F407VGTx`（工程内声明）：
   - [ANO_LX_STM32F407.uvprojx](file:///workspace/ANO_LX_FC_T265代替光流/ProjectSTM32F407/ANO_LX_STM32F407.uvprojx#L17-L23)
3. Build
4. 产物：
   - 工程配置 AfterMake 会把 `.axf` 转为 bin：`fromelf.exe --bin -o ./ANO-LX.bin ...`
   - 见：[ANO_LX_STM32F407.uvprojx](file:///workspace/ANO_LX_FC_T265代替光流/ProjectSTM32F407/ANO_LX_STM32F407.uvprojx#L82-L86)

### 5.1.2 MSP432 / TM4C123

使用方式与 STM32 类似，打开对应 `uvprojx` 即可。

注意 MSP432 的第三方库 driverlib 同时提供命令行 Makefile（更偏“构建库”用途）：

- GCC Makefile：[driverlib/gcc/Makefile](file:///workspace/ANO_LX_FC_倾角保护版/DriversMcu/MSP432P401/Drivers/ti/devices/msp432p4xx/driverlib/gcc/Makefile#L28-L47)

## 5.2 固件下载与运行（硬件侧）

固件属于嵌入式裸机程序，运行方式为“烧录到目标板后上电运行”：

1. 连接调试器/下载器（常见为 ST-Link / J-Link / TI XDS 等，取决于目标 MCU）
2. 在 Keil 中配置 Debug/Flash Download（工程已配置基础 FlashDriver）
3. Download 到板卡
4. 上电后 `main()` 进入主循环/中断驱动任务

运行时关键外设与端口角色（以 T265版 `All_Init()` 为准）：

- UART1：T265：[Drv_BSP.c](file:///workspace/ANO_LX_FC_T265代替光流/DriversBsp/Drv_BSP.c#L61-L66)
- UART3：串口拓展板：[Drv_BSP.c](file:///workspace/ANO_LX_FC_T265代替光流/DriversBsp/Drv_BSP.c#L65-L67)
- UART4：匿名光流（若启用）：[Drv_BSP.c](file:///workspace/ANO_LX_FC_T265代替光流/DriversBsp/Drv_BSP.c#L67-L69)
- UART5：IMU：[Drv_BSP.c](file:///workspace/ANO_LX_FC_T265代替光流/DriversBsp/Drv_BSP.c#L69-L70)
- RC 输入：PPM/SBUS（`DrvRcInputInit()` 选择初始化方式）：[Drv_BSP.c](file:///workspace/ANO_LX_FC_T265代替光流/DriversBsp/Drv_BSP.c#L72-L101)

## 5.3 Python 端运行（树莓派）

### 5.3.1 依赖安装（建议方式）

仓库未提供锁定依赖文件，建议在虚拟环境中手动安装最小依赖：

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install pyserial numpy
```

若需要真实 T265：

```bash
pip install pyrealsense2
```

`transformations` 为可选（缺失会自动降级）：见 [t265.py](file:///workspace/drone_control/t265.py#L14-L19)

### 5.3.2 启动

入口脚本：

- [drone_control/main.py](file:///workspace/drone_control/main.py#L21-L46)

启动前需要确认串口设备名与波特率（默认写死在代码中）：

- 飞控：`/dev/ttyS6`，`460800`：[main.py](file:///workspace/drone_control/main.py#L25-L28)
- 地面站：`/dev/ttyS7`，`115200`：[main.py](file:///workspace/drone_control/main.py#L30-L33)
- K230 视觉板：`/dev/ttyS3`，`115200`：[main.py](file:///workspace/drone_control/main.py#L35-L37)

运行：

```bash
cd /workspace/drone_control
python3 main.py
```

### 5.3.3 航点与任务

- 静态航点来自 `router.txt`：[Mission_GPT.py](file:///workspace/drone_control/Mission_GPT.py#L70-L101)
- 若地面站提供禁飞区，则会触发覆盖规划并替换航点列表：[Mission_GPT.py](file:///workspace/drone_control/Mission_GPT.py#L127-L154)

## 5.4 地面站与K230

- 地面站串口协议定义见：[地面站通信协议.md](file:///workspace/drone_control/地面站通信协议.md)
- K230 动物识别脚本运行在 CanMV/K230 环境（非Python venv），脚本说明与UART命令格式见：[animal_detect_yolov8n.py](file:///workspace/drone_control/animal_detect_yolov8n.py#L1-L18)

