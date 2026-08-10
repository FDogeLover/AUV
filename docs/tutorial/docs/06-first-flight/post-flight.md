# 飞后归档

## 四步飞后流程

### Step 1：断电

确认电机关停后：**先拔电池，再关遥控器**。

### Step 2：拉取飞行日志到本地

```bash
# 在本地电脑项目根目录执行
./tools/pull_flight_log.sh
```

这个脚本会自动通过 scp 拉取板子上的 `flight_data.jsonl` 到本地 `data_archive/` 目录，并清空板端数据。

!!! warning "数据只存本地"
    板子存储空间有限，飞完必须归档。详见 [铁律五](../02-safety/critical-rules.md)。

### Step 3：分析飞行数据

```bash
# 使用日志分析工具查看飞行轨迹、误差等
python tools/flight_log_analyzer.py data_archive/test_data_YYYYMMDD/flight_data.jsonl
```

### Step 4：记录飞行结果

建议记录：

- 飞行日期、航线、天气/光照条件
- 飞行时长
- 有无异常
- 精度误差（XY/Z）
- 电池消耗

方便后续调参和问题追溯。

---

← [起飞流程](takeoff-flow.md) | [状态机流程 →](../07-architecture/state-machine.md)
