---
name: project-flight-tools-workflow
description: 飞行日志分析工具 + 数据归档流程
metadata: 
  node_type: memory
  type: project
  originSessionId: sess_444a2a84-dc30-4300-83d4-a618a8ba533e
---

# 飞行日志工具与归档流程（2026-07-24）

## 工具

### `tools/flight_log_analyzer.py`
一键分析 `flight_data.jsonl`，输出：
- 每次飞行的状态机流转
- 航点到达精度 (XY 偏差、Z 偏差)
- 航向保持状态（正常 / 故障锁存）
- 降落点位置 + 锁桨确认（unlock_sta / motor_pwm_mask）
- 快速摘要行（适合粘贴到聊天记录）

用法：
```bash
python3 tools/flight_log_analyzer.py                          # 自动搜索最新
python3 tools/flight_log_analyzer.py path/to/flight_data.jsonl # 指定文件
```

### `tools/pull_flight_log.sh`
从板子拉取日志到本地归档，自动清理板子工作日志：

```bash
./tools/pull_flight_log.sh               # basic 版本
./tools/pull_flight_log.sh basic_radar   # 其他版本
```

## 归档结构

数据只存在本地，板子不留副本：

```
drone_control/tools/data_archive/
└── test_data_YYYYMMDD/
    ├── flight_data_<路径概要>_YYYYMMDD.jsonl
    └── router_YYYYMMDD.txt
```

## 完整工作流

```
飞完测试
    │
    ▼
板子上快速看结果:
  python3 flight_log_analyzer.py
    │
    ▼
本机拉取归档:
  ./tools/pull_flight_log.sh
    ├── scp 到本地 data_archive/
    ├── 自动命名（不覆盖）
    └── 清空板子 flight_data.jsonl
    │
    ▼
本地分析：
  python3 tools/flight_log_analyzer.py
```

## 注意

- 每次飞完跑一次 `pull_flight_log.sh`，避免多次飞行数据混在一个文件里
- 航向保持状态需要 `heading_hold_enabled=true` 的日志才有数据
