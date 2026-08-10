# 事件总线架构

## 概述

参考 `competition_2026/Lcode/mission_events.py`。线程安全有界队列事件总线，publish 永不阻塞，支持13种事件类型。

## 设计原则

- **不阻塞飞控**：事件发布是异步的，不会卡住控制环路
- **线程安全**：有界队列(256)，publish永不阻塞
- **解耦模块**：各模块通过事件通信，不直接调用

## 事件类型

```
WAYPOINT_REACHED / WAYPOINT_START / WAYPOINT_COMPLETE
ACTION_START / ACTION_COMPLETE
SNAPSHOT_REQUEST / SNAPSHOT_COMPLETE
TASK_START / TASK_COMPLETE
...
```

## 使用方式

```python
from Lcode.mission_events import EventBus, EventType

bus = EventBus()

# 发布事件（非阻塞）
bus.publish(EventType.WAYPOINT_REACHED, {'index': 1, 'pos': [0.5, 0.0, 1.0]})

# 订阅事件
bus.subscribe(EventType.WAYPOINT_REACHED, on_waypoint_reached)
```

---

[PID调参指南 →](pid-tuning.md)
