# Code Wiki（仓库级）

本仓库主要包含两部分：

- **飞控固件（C，裸机）**：两套工程，代码同源但面向不同传感器/业务定制。
  - [ANO_LX_FC_倾角保护版](file:///workspace/ANO_LX_FC_倾角保护版)
  - [ANO_LX_FC_T265代替光流](file:///workspace/ANO_LX_FC_T265代替光流)
- **上位机/地面端控制（Python）**：树莓派端串口通信、T265定位、任务状态机、与地面站/视觉板联动。
  - [drone_control](file:///workspace/drone_control)

## 文档导航

- [01_总体架构](file:///workspace/CodeWiki/01_Architecture.md)
- [02_目录与模块](file:///workspace/CodeWiki/02_Modules.md)
- [03_关键流程与接口](file:///workspace/CodeWiki/03_Key_Flows_and_APIs.md)
- [04_依赖关系](file:///workspace/CodeWiki/04_Dependencies.md)
- [05_构建与运行](file:///workspace/CodeWiki/05_Build_and_Run.md)

## 快速入口（建议阅读顺序）

1. 固件整体入口：[main.c](file:///workspace/ANO_LX_FC_T265代替光流/FcSrc/main.c#L22-L37)（两工程一致）
2. 板级初始化：`All_Init()`：[Drv_BSP.c](file:///workspace/ANO_LX_FC_T265代替光流/DriversBsp/Drv_BSP.c#L21-L89)
3. 运行骨架：
   - 裸机调度器：`Scheduler_Setup/Run()`：[Ano_Scheduler.c](file:///workspace/ANO_LX_FC_倾角保护版/FcSrc/Ano_Scheduler.c#L65-L116)
   - STM32F407 定时中断驱动主任务：`TIM7_IRQHandler -> ANO_LX_Task()`：[stm32f4xx_it.c](file:///workspace/ANO_LX_FC_T265代替光流/DriversMcu/STM32F407/Drivers/stm32f4xx_it.c#L59-L69)
4. 飞控 1ms 主任务（T265版）：`ANO_LX_Task()`：[ANO_LX.c](file:///workspace/ANO_LX_FC_T265代替光流/FcSrc/ANO_LX.c#L289-L341)
5. 树莓派端串口协议与任务入口：
   - 串口双线程（速度帧+指令帧）：[Lprotocol.py](file:///workspace/drone_control/Lcode/Lprotocol.py#L57-L115)
   - 任务状态机：[Mission_GPT.py](file:///workspace/drone_control/Mission_GPT.py#L23-L209)
   - 运行入口：[main.py](file:///workspace/drone_control/main.py#L21-L46)

