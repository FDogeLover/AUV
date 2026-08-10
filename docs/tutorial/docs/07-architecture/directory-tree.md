# 目录结构

## 顶层结构

```
Project2/
├── drone_control/              ★ Python上位机（核心代码）
│   ├── basic/                   ★ 基础飞行版（新手从这里开始）
│   ├── circle_pole/             圆杆环绕飞行
│   ├── competition_2026/         2026备赛版
│   ├── competition_2026_d/       D题陆空协同（最新活跃版本）
│   ├── fire_patrol/              消防巡逻赛题(G题)
│   ├── warehouse_inventory/      立体货架盘点（已验收）
│   └── tools/                    数据分析工具
├── 飞控固件/                    飞控固件源码（烧录用，含倾角保护）
├── docs/                        项目文档体系（核心事实源）
├── CodeWiki/                   仓库级代码文档
├── tools/                      项目级工具脚本（flight_log_analyzer.py、pull_flight_log.sh 等）
├── .Codex/memory/              AI Agent记忆体系（AI内部使用，人工一般不需要）
├── .zcode/plans/               功能计划文档（AI辅助设计产物）
├── edit_firmware.py            固件安全编辑脚本（AI Agent专用，人工用Keil编辑）
└── pull_flight_log.sh           一键拉取飞行日志（人工操作）
```

## basic/ 内部结构

```
drone_control/basic/
├── main.py                      程序入口
├── Mission_GPT.py               状态机主体
├── t265.py                      T265位姿读取
├── router.txt                   航点文件
├── Lcode/                       核心库
│   ├── Lprotocol.py             串口协议（三线程收发）
│   ├── Lpid.py                   PID控制器
│   ├── heading_hold.py           航向保持外环
│   ├── navigation_profile.py     航点到达策略
│   ├── global_variable.py        跨线程全局状态
│   ├── gpio_button.py            一键起飞按键
│   ├── gpio_led.py               RGB警示灯
│   ├── resource_monitor.py       CPU/内存/温度监控
│   └── Logger.py                 日志模块
├── test_*.py                    18+个单元测试
└── router_tests/                预设测试航线
```

## 航点文件格式

`router.txt` 每行一个航点，格式 `x,y,z`（单位：米）：

```
0.0,0.0,1.0    # 悬停起飞点正上方，高度1m
-0.6,0.0,1.0   # 向前飞0.6m（T265坐标系-X为前方）
-0.6,0.0,0.2   # 原地下降至0.2m，准备降落
```

---

[核心模块 →](modules/lprotocol.md)
