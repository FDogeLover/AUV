# 日志分析

## 飞行日志格式

每次飞行自动追加写入 `flight_data.jsonl`（不清空），每行一条 JSON 记录。日志字段由 `Mission_GPT.py` 在不同状态下写入，主要字段如下：

### 导航阶段（NAVIGATE）日志字段

| 字段 | 类型 | 内容 |
|------|------|------|
| `t` | float | 时间戳（Unix epoch，秒） |
| `state` | str | 状态机状态（IDLE/TAKEOFF/NAVIGATE/DESCEND/LAND/HOVER_WAIT） |
| `target_idx` | int | 当前航点索引 |
| `pos` | [x, y, z] | T265 位置（米，已校准+激光覆盖Z） |
| `target` | [x, y, z] | 当前航点目标坐标（米） |
| `vx`, `vy` | int | 发送给飞控的速度指令（cm/s，含偏移） |
| `yaw_cmd_sent` | int | 航向保持输出的 Yaw 角速度指令（°/s） |
| `t265_yaw_deg` | float | T265 当前 Yaw 角（度） |
| `fc_yaw_deg` | float | 飞控上报的 Yaw 角（度，×100 整数还原） |
| `t265_vel` | [vx, vy] | T265 速度（m/s） |
| `of1_vel_cms` | [dx, dy] | 光流速度（cm/s，飞控帧1字段） |
| `roll_pitch` | [roll, pitch] | 飞控上报的 Roll/Pitch 角（度） |
| `height_setpoint_cm` | float | 当前高度设定值（cm，ramp 值） |
| `of_status` | [quality, link, work] | 光流质量/连接/工作状态 |
| `nav_profile` | str | 导航策略（precision/cruise） |
| `waypoint_mode` | str | 当前航点模式 |
| `arrival_distance_m` | float | 到达距离（米） |
| `laser_height_m` | float | 激光高度（米） |
| `t265_raw_z_m` | float | T265 原始 Z（未滤波，米） |
| `t265_filtered_z_m` | float | T265 滤波后 Z（米） |
| `t265_confidence` | int | T265 跟踪置信度（0-3） |

### 事件日志

| `event` 值 | 触发时机 |
|-----------|---------|
| `task_start` | 任务启动 |
| `waypoint_advance` | 航点推进（含 reason: precision_arrival/cruise_arrival/timeout） |

### 系统资源日志

由 `resource_monitor.py` 独立写入，包含 `cpu`、`mem`、`temp` 字段。

!!! warning "人工操作：飞行前备份旧日志"
    飞行前请先备份并移走旧的 `flight_data.jsonl`，否则新数据追加进旧文件难以区分。

## 分析工具

!!! example "人工操作：一键分析日志"
    飞行后可使用分析工具生成轨迹图和误差统计。`flight_log_analyzer.py` 位于项目根目录的 `tools/` 下（非 `drone_control/tools/`）：

    ```bash
    # 从项目根目录执行
    python tools/flight_log_analyzer.py drone_control/basic/flight_data.jsonl
    ```

## 手动分析

如需深入分析特定字段，可用 Python 手动解析：

```python
import json

with open('flight_data.jsonl') as f:
    for line in f:
        data = json.loads(line)
        state = data.get('state')       # 状态机状态
        pos = data.get('pos')           # [x, y, z] 位置（米）
        t265_vel = data.get('t265_vel') # [vx, vy] 速度（m/s）
        laser = data.get('laser_height_m')  # 激光高度（米）
        conf = data.get('t265_confidence')  # 置信度
```

---

← [匿名上位机使用](ground-station.md) | [故障排查速查 →](../10-troubleshoot/quick-reference.md)
