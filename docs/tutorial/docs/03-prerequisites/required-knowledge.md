# 你需要会什么

本教程假设你具备以下基础知识。不会的部分建议先学习对应链接，否则后续操作可能遇到困难。

## 必须掌握

| 知识点 | 要求程度 | 学习资源 |
|--------|---------|---------|
| **Python 3** | 能读懂类、线程、装饰器、上下文管理器 | [官方教程](https://docs.python.org/zh-cn/3/tutorial/) |
| **Linux 命令行** | 熟练使用 cd / ls / ssh / pip / 基本文件操作 | [Linux 命令大全](https://www.runoob.com/linux/linux-command-manual.html) |
| **Git** | 会 clone / commit / push / pull | [Git 教程](https://www.liaoxuefeng.com/wiki/896043488029600) |
| **串口通信概念** | 知道波特率、数据帧、校验位是什么 | 教程内补充 |
| **PID 控制概念** | 知道 P / I / D 各自的作用和基本调参思路 | [PID 控制原理](https://zh.wikipedia.org/wiki/PID控制器) |

## 最好了解

| 知识点 | 用途 |
|--------|------|
| IMU / 陀螺仪 / 加速度计 | 理解飞控姿态融合 |
| EKF（扩展卡尔曼滤波） | 理解凌霄IMU内部融合（进阶） |
| I2C / SPI / UART 协议 | 理解飞控传感器通信 |
| Markdown 语法 | 阅读和编写项目文档 |

## 不需要会的

以下知识**不需要**提前学习，教程中会讲到：

- T265 视觉里程计的工作原理
- 匿名数传 AA 帧协议格式
- 凌霄 IMU 内部 EKF 参数
- Keil 固件编译烧录流程（除非你要改固件）

---

[环境搭建 →](../04-setup/local-env.md)
