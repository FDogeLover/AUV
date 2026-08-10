# 参考资料索引

本页列出仓库内附带的全部参考资料。**所有文件已随仓库一起分发**，克隆仓库后直接在 `docs/reference-materials/` 目录下查阅即可，无需额外下载。

!!! info "资料位置"
    所有资料位于仓库的 `docs/reference-materials/` 目录下，按分类整理为 9 个子目录。

---

## 目录结构总览

```
docs/reference-materials/
├── 01-protocols/           # 通信协议与飞控手册
├── 02-schematics/          # 硬件原理图与 PCB
├── 03-datasheets/          # 芯片 Datasheet
├── 04-firmware/             # 飞控固件源码工程
│   ├── ANO_LX_FC/          #   主工程（FcSrc + DriversBsp + .bin）
│   ├── 例程1_一键起飞_降落/
│   └── 例程2_一键任务/
├── 05-imu-firmware/        # 凌霄 IMU 固件
├── 06-competition-problems/ # 历年竞赛题目（2013-2025）
├── 07-d-problem/           # 2024 电赛 D 题资料
├── 08-uav-2023/            # 2023 开源 UAV 方案
│   ├── mechanical/         #   机械图纸（SLDPRT + STL）
│   └── code/               #   Python/Arduino 代码
└── 09-theory/              # 理论资料与教材
```

---

## 1. 通信协议与飞控手册

`docs/reference-materials/01-protocols/`

| 文件 | 说明 |
|------|------|
| `匿名--凌霄--飞控手册.V1.07pdf.pdf` | 飞控功能、参数配置、通信协议总览 |
| `匿名通信协议V7.pdf` | 串口数据帧格式定义（本项目协议的基础） |
| `匿名--凌霄到手飞手册.pdf` | 首次组装与起飞流程 |
| `匿名凌霄FC姿态单参数控制参考配置.txt` | PID 调参参考值 |

**建议阅读顺序**：先看「到手飞手册」了解组装 → 看「通信协议 V7」理解数据帧格式 → 看「飞控手册」深入参数配置。

---

## 2. 硬件原理图与 PCB

`docs/reference-materials/02-schematics/`

| 文件 | 说明 |
|------|------|
| `STM32F407核心板原理图.pdf` | 本项目飞控 MCU 板 |
| `凌霄整机底板ANO-LX-PCB2-20200716.pdf` | 底板布局与接口定义 |
| `开源飞控平台底板原理图-typec.pdf` | Type-C 版底板 |
| `MSP432核心板原理图.pdf` | 备选 MCU 板 |
| `TM4C123核心板原理图.pdf` | 备选 MCU 板 |

---

## 3. 芯片 Datasheet

`docs/reference-materials/03-datasheets/`

| 文件 | 说明 |
|------|------|
| `STM32F405xx,STM32F407xx.pdf` | 飞控 MCU 主芯片 Datasheet |
| `STM32F4xx中文参考手册.pdf` | 寄存器与外设说明（20MB） |
| `ICM-20602-v1.0.pdf` | 六轴 IMU（凌霄内置） |
| `MPU-6000.6050中文资料.pdf` | 六轴 IMU 备选 |
| `MPU-60X0寄存器中文版V4.0.pdf` | 寄存器级开发参考 |
| `MS5611-01BA03气压计(高度计)中文资料(最详细的).pdf` | 气压高度计 |
| `气压传感器SPL06-001.pdf` | 气压传感器 |
| `AK8975.pdf` | 三轴磁力计 |

---

## 4. 飞控固件源码工程

!!! warning "参考资料 vs 烧录代码"
    本目录（`docs/reference-materials/04-firmware/`）中的代码仅供**阅读理解**使用，不是烧录代码。

    **烧录用的固件源码**位于仓库根目录的 `飞控固件/` 文件夹中（含倾角保护功能）。如需修改固件，**人工用 Keil uVision 编辑，AI Agent 用 `edit_firmware.py` 脚本**。详见 [固件编辑规范](../09-workflow/firmware-edit.md)。

!!! warning "编码注意"
    凌霄飞控 `.c/.h` 文件使用 **GB2312/GBK** 编码，请勿用普通编辑器直接修改。

### 4.1 主工程 ANO_LX_FC

`docs/reference-materials/04-firmware/ANO_LX_FC/`

| 目录/文件 | 说明 |
|----------|------|
| `FcSrc/` | 飞控核心源码（状态机、调度器、传感器融合等 16 个文件） |
| `DriversBsp/` | 驱动层（光流、GPS、BSP、数学库等 8 个文件） |
| `ANO-LX.bin` | STM32F407 编译好的固件二进制文件 |

### 4.2 例程

| 目录 | 说明 |
|------|------|
| `例程1_一键起飞_降落/` | 使用 **IMU 自带的一键起飞**功能，最简起飞降落流程 |
| `例程2_一键任务/` | 通过 **实时帧控制锁桨**实现起飞和降落，可在单个任务流程中多次起飞-降落-再起飞-降落 |

教程内也可直接查看例程代码：

- [:octicons-file-code-24: 例程1 — `User_Task.c`](07-firmware-examples/例程1_一键起飞_降落/User_Task.c)
- [:octicons-file-code-24: 例程2 — `User_Task.c`](07-firmware-examples/例程2_一键任务/User_Task.c)

---

## 5. 凌霄 IMU 固件

`docs/reference-materials/05-imu-firmware/`

| 文件 | 说明 |
|------|------|
| `ANO_LX-hw120-sw123.ano` | IMU 固件（版本 123） |
| `ANO_LX-hw122-sw131.ano` | IMU 固件（版本 131） |
| `ANO_LX-hw122-sw132.ano` | IMU 固件（版本 132） |
| `ANO_LX-hw122-sw133.ano` | IMU 固件（版本 133） |
| `ANO_LX-hw122-sw135.ano` | IMU 固件（版本 135） |
| `匿名-V7版本上位机固件升级使用方法.pdf` | 固件升级教程 |

!!! warning "版本匹配"
    119 以上版本固件必须搭配新的外部 MCU 程序，注意实时控制帧协议变化。

---

## 6. 历年竞赛题目（2013-2025）

`docs/reference-materials/06-competition-problems/`

| 文件 | 年份 | 主题 |
|------|------|------|
| `2013年四旋翼无人机拾取加定点飞行.pdf` | 2013 | 拾取+定点 |
| `2014年四旋翼无人机.pdf` | 2014 | 基础飞行 |
| `2015飞行后定点拾取+航拍.pdf` | 2015 | 拾取+航拍 |
| `2017无人机跟随小车.pdf` | 2017 | 跟随小车 |
| `2018年TI杯大学生电子设计竞赛题B-灭火飞行器.docx` | 2018 | 灭火 |
| `2019巡线机器人.pdf` | 2019 | 巡线 |
| `2020绕障飞行器.pdf` | 2020 | 绕障 |
| `2021植保飞行器（G题）.pdf` | 2021 | 植保 |
| `2022送货无人机.pdf` | 2022 | 送货 |
| `2023空地协同智能消防系统.pdf` | 2023 | 空地协同消防 |
| `2024立体货架盘点无人机系统.pdf` | 2024 | 货架盘点 |
| `2025野生动物巡查系统.pdf` | 2025 | 野生动物巡查 |
| `空地协调智能消防系统（G题）测试评分表（非正式版）.docx` | 2023 | 评分参考 |

---

## 7. 2024 电赛 D 题资料

`docs/reference-materials/07-d-problem/`

| 文件 | 说明 |
|------|------|
| `2024_D题设计报告.docx` | 2024 年 D 题完整设计方案 |
| `D题_立体货架盘点无人机系统.pdf` | 官方题目文件 |
| `24年提问汇 D题总.txt` | 官方答疑汇总 |
| `README.md` | 资料说明 |
| `debug.txt` | 调试记录 |
| `基础部分完成3分钟.txt` | 基础部分完成记录 |

---

## 8. 2023 开源 UAV 方案

`docs/reference-materials/08-uav-2023/`

| 目录/文件 | 说明 |
|----------|------|
| `mechanical/` | SolidWorks 零件图（.SLDPRT）+ 3D 打印件（.STL），共 12 个文件 |
| `code/` | 2023 年参赛 Python/Arduino 代码（main1.py、protocol.ino 等） |
| `README.md` | B站作者德布罗意波波丷的开源方案说明 |
| `先看本文档，动手前再看readme.txt` | 电赛准备路线图 |
| `LICENSE` | 开源协议 |
| `../09-theory/2023代码解释.docx` | 2023 年代码详细说明文档 |

---

## 9. 理论资料

`docs/reference-materials/09-theory/`

| 文件 | 说明 |
|------|------|
| `《多旋翼飞行器设计与控制》新版中文PPT合集.pdf` | 系统级理论教材（24MB） |
| `FS-i6S遥控器说明书20161026.pdf` | 遥控器使用手册（18MB） |
| `MotionDriver_Tutorial_12212018 CB.pdf` | InvenSense 官方驱动教程 |
| `2023代码解释.docx` | 2023 年代码详细说明 |

---

!!! tip "如何使用这些资料"
    1. **通信协议**是本项目串口协议的基础，建议优先阅读
    2. **原理图**用于硬件排线和接口确认
    3. **Datasheet**在需要深入理解某个传感器或 MCU 外设时查阅
    4. **历年赛题**帮助理解电赛题目演变和考察重点
    5. **固件源码**在需要修改飞控行为时参考（注意 GB2312 编码）
    6. **D 题资料**是 2024 年参赛的完整设计方案，对理解项目背景很有帮助
