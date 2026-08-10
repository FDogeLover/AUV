# 日志分析

## 飞行日志格式

每次飞行自动追加写入 `flight_data.jsonl`（不清空），每行一条 JSON 记录：

| 字段 | 内容 |
|------|------|
| `timestamp` | 时间戳 |
| `state` | 状态机状态 |
| `t265` | T265位姿与速度（pos_x/y/z, vel_x/y/z） |
| `fc` | 飞控帧（姿态角、解锁状态、激光高度） |
| `cpu` / `mem` / `temp` | 系统资源（来自resource_monitor） |

!!! warning "飞行前备份旧日志"
    飞行前请先备份并移走旧的 `flight_data.jsonl`，否则新数据追加进旧文件难以区分。

## 分析工具

```bash
# 一键分析日志，生成轨迹图和误差统计
python tools/flight_log_analyzer.py path/to/flight_data.jsonl
```

## 手动分析

```python
import json

with open('flight_data.jsonl') as f:
    for line in f:
        data = json.loads(line)
        state = data['state']   # 状态机状态
        t265 = data['t265']     # pos_x, pos_y, pos_z, vel_x, ...
        fc = data['fc']         # 飞控帧数据
```

---

← [代码同步](code-sync.md) | [故障排查速查 →](../10-troubleshoot/quick-reference.md)
