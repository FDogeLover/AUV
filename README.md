# Project2 — 自主无人机飞控系统

面向全国大学生电子设计竞赛的自主无人机系统，采用「裸机飞控 + 上位机智能决策 + 视觉感知」三层架构。

## 架构概览

```
┌─────────────────────────────────────────────────┐
│  视觉板（核桃派 CyberCamera）                     │
│  目标检测：AprilTag / 蓝色方块 / 火情            │
│  UART 115200 ↓                                  │
├─────────────────────────────────────────────────┤
│  上位机（地瓜派 RDK X5 · Python 3）               │
│  状态机控制 · T265定位 · PID闭环 · 航线导航       │
│  UART 460800 ↓                                  │
├─────────────────────────────────────────────────┤
│  飞控（STM32F407 + 凌霄IMU）                      │
│  姿态自稳 · 电机PWM · 解锁/降落 · 倾角保护       │
└─────────────────────────────────────────────────┘
```

## 快速开始

```bash
# 1. 克隆仓库
git clone https://github.com/FDogeLover/AUV.git
cd AUV

# 2. 进入教程目录，启动本地文档
cd docs/tutorial
pip install mkdocs mkdocs-material
mkdocs serve
# 浏览器打开 http://127.0.0.1:8000
```

**新手请从 [MkDocs 教程](docs/tutorial/docs/index.md) 开始**，跟随 5 步学习路径完成首次飞行。

## 版本说明

| 版本 | 目录 | 状态 | 说明 |
|------|------|:----:|------|
| 基础飞行 | `drone_control/basic/` | ✅ 已验证 | **新手必从这里开始** |
| D题陆空协同 | `drone_control/competition_2026_d/` | ✅ 已实飞 | 空地DCP + 双T265 + 视觉伺服 |
| 消防巡逻 | `drone_control/fire_patrol/` | ✅ 已实飞 | G题火情检测+抛投 |
| 货架盘点 | `drone_control/warehouse_inventory/` | ✅ 已验收 | QR视觉+异步扫码 |
| 圆杆环绕 | `drone_control/circle_pole/` | ✅ 阶段1验证 | circle_planner + 雷达避障 |
| 2026通用版 | `drone_control/competition_2026/` | 🟡 待实飞 | 事件总线+视频+UDP |

所有版本共享 `Lcode/` 核心库（串口协议、PID、航向保持等），`basic/` 是所有版本的基础。

## 目录结构

```
Project2/
├── drone_control/          Python上位机（核心代码）
│   ├── basic/               ★ 新手起点
│   ├── circle_pole/         圆杆环绕（雷达避障）
│   ├── competition_2026_d/  D题陆空协同（含视觉伺服）
│   ├── fire_patrol/         消防巡逻（G题）
│   └── ...                  其他赛题版本
├── 飞控固件/                飞控固件源码（烧录用，含倾角保护）
├── docs/
│   ├── tutorial/            MkDocs 教程（人类面向）
│   ├── known_issues.md      46条已知问题
│   ├── architecture/        架构文档
│   └── reference-materials/ 参考资料（协议/原理图/Datasheet/赛题）
├── tools/                  工具脚本（同步/日志分析）
├── CodeWiki/               代码级文档
├── edit_firmware.py        固件安全编辑脚本（AI Agent 专用）
└── sync-to-ubuntu-pi.sh    一键同步代码到板子
```

## 硬件需求

| 硬件 | 型号 |
|------|------|
| 飞控板 | STM32F407 + 凌霄IMU（集成模块） |
| 上位机 | 地瓜派 RDK X5（ARM Linux） |
| 视觉里程计 | Intel RealSense T265 |
| 视觉板 | 核桃派 CyberCamera（AprilTag/蓝块检测） |
| 电池 | 3S 3300mAh / 3S 4000mAh / 4S 5300mAh |

## 安全须知

- **降落必须人工目视确认电机停转**（land() 存在假阳性）
- **飞行区域清空**，起飞前5秒红灯警示期内撤离到3米以外
- **遥控器随时准备接管**，异常时立即切手动模式
- 详细安全规范见 [教程 · 安全须知](docs/tutorial/docs/02-safety/critical-rules.md)

## 文档导航

| 需要什么 | 去哪里 |
|---------|--------|
| 新手教程 | `docs/tutorial/`（`mkdocs serve` 启动） |
| 已知问题 | `docs/known_issues.md` |
| 架构设计 | `docs/architecture/` |
| 参考资料索引 | `docs/tutorial/docs/reference/index.md` |
| 代码文档 | `CodeWiki/` |
| AI Agent 指南 | `docs/AGENTS_GUIDE.md` |

## 许可

本项目代码仅供学习和竞赛使用。
