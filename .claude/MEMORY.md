# Project2 无人机工程 - 项目记忆

## fc-firmware GB2312 编码约束

飞控固件（`ANO_LX_FC_倾角保护版/` 和 `ANO_LX_FC_T265代替光流/`）的所有 C 源文件使用 **GB2312 编码**，因为必须在 Keil MDK (uVision) 中编译烧录，Keil 在中文 Windows 上原生使用 GB2312。

**How to apply：** 在检查、修改或创建飞控固件 C 文件时：
1. 不得改变现有文件的编码格式（保持 GB2312）
2. 不把 GB2312 编码当作 bug 或问题列出
3. 新增文件也使用 GB2312 编码
4. **禁止对 C 源文件使用 Read + Edit 工具链** — 会在写回时将 GB2312 转为 UTF-8，导致 Keil 中文注释乱码
5. **C 文件的小改动一律用 Bash/sed 执行**，如 `sed -i 's/pattern/replacement/g' file.c`，二进制安全
6. Python 文件不受此限制（UTF-8），可以正常使用 Read/Edit

## 自动保存问题记录

代码检查、审查或调试过程中发现的问题（bug、潜在风险、架构建议等），主动保存到 `.claude/problem/` 目录，按日期和主题命名。

**How to apply：**
1. 每次代码检查/审查完成后，整理问题为结构化文档
2. 文件命名：`{主题}_{日期}.md`
3. 按严重程度分级（严重/中等/低/建议）
4. 内容包含：文件路径、行号、问题描述、影响分析
5. 存入 `.claude/problem/`

## 项目架构概览

### 项目组成

| 组件 | 路径 | 语言/平台 | 说明 |
|------|------|-----------|------|
| 飞控固件 | `ANO_LX_FC_倾角保护版/` | C / Keil / STM32F407 | 姿态控制、导航、PID |
| Python上位机 | `drone_control/` | Python / 树莓派 | 任务规划、T265定位、K230通信 |
| K230视觉板 | `k230/` | Python / Canaan K230 | YOLOv8目标检测、UART通信 |

### 三平台MCU架构

飞控固件采用**共享核心 + 独立MCU驱动层**架构：
- `FcSrc/` — 核心飞控逻辑（共享）
- `DriversBsp/` — 板级驱动（共享）
- `Mycode/` — 自定义协议/功能（共享）
- `DriversMcu/<平台>/` — MCU特定驱动（STM32F407 / MSP432P401 / TM4C123 三选一）

### 飞控调度器

7级频率调度（2~1000Hz），基于tick计数的非抢占式调度：
- 1000Hz: IMU数据更新
- 500Hz: 姿态解算
- 200Hz: 光流/T265速度融合
- 100Hz: PID控制、ESC输出
- 50Hz: 状态机、串口通信
- 10Hz: 遥测发送
- 2Hz: 系统监控

### 通信协议

**上位机 ↔ 飞控（串口 460800bps）**

上行（Pi → FC）：
- `AA 01` T265速度帧: vx(cm/s) + vy(cm/s) + yaw(0.01°) — 100Hz
- `AA 02` 指令帧: task_sta + com_x + com_y + com_z + com_yaw + next_task + sp_side — 50Hz

下行（FC → Pi）— **19字节扩展帧**：
- `AA` + task_sta + roll/pitch/yaw(x100,各2B) + state + x_int/y_int(各2B,偏移0x4000) + laser_height_cm(4B,小端序) + CK + `FF`
- CK = 字节1~16累加和

**上位机 ↔ 地面站（串口 115200bps）**：
- 禁飞区坐标: `AA` + 3×(A,B) + `FF`
- 进度回传: `AA` + idx + cls + cnt + `FF`

**上位机 ↔ K230（串口 115200bps）**：
- 请求帧: Pi发送图像帧 → K230 YOLOv8检测 → 返回类别/置信度/位置
- 协议定义在 `drone_control/Lcode/k230_client.py`

### 线程模型（上位机 7+并发线程）

| 线程 | 职责 | 频率 |
|------|------|------|
| T265采集 | pyrealsense2 pose stream | 200Hz |
| 飞控接收 | listen_fc() 解析19字节帧 | 事件驱动 |
| 速度帧发送 | _send_t265_loop() T265速度→飞控 | 100Hz |
| 指令帧发送 | _send_command_loop() 指令→飞控 | 50Hz |
| 地面站接收 | listen_dmz() 禁飞区 | 事件驱动 |
| 地面站发送 | send_dmz() 进度 | 10Hz |
| K230通信 | K230Client UART收发 | 按需 |
| 任务主循环 | mission.loop() 状态机 | ~30Hz |

**线程同步**：`threading.Lock()`（global_variable.py: `lock`）保护所有共享状态读写

### T265定位状态

- **XY平面**: 5-15cm精度，已优化（alpha=0.15低通滤波 + 置信度动态阈值）
- **Z轴**: 已融合飞控激光测距高度（替代T265伪Z值）
- **置信度**: 3级降级策略（0→悬停, 1→0.30m阈值, 2→0.15m, 3→0.10m）
- **到达判断**: 15次确认(~450ms) + 5s超时

### 关键约定

- 飞控固件 **必须使用 GB2312 编码**（Keil MDK 限制）
- 编辑 `.c` 文件时 **不能使用 Read/Edit 工具**（会破坏编码），改用 sed
- 串口路径通过环境变量配置：`DRONE_FC_PORT`, `DRONE_DMZ_PORT`, `DRONE_K230_PORT`
- Git 仓库根目录与工作目录均为 `Project2/`
