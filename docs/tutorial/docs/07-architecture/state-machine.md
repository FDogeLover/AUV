# 状态机流程

所有飞行任务由 `Mission_GPT.py` 中的有限状态机驱动。

## 状态流转

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> TAKEOFF: 按键触发
    TAKEOFF --> NAVIGATE: T265置信度OK<br/>爬升至目标高度
    NAVIGATE --> NAVIGATE: 逐航点飞行<br/>到达确认→悬停→下一个
    NAVIGATE --> DESCEND: 最后航点到达<br/>且高度≤20cm
    DESCEND --> LAND: 贴地确认(10帧)<br/>或固件抢先锁桨
    DESCEND --> HOVER_WAIT: 超时20s
    LAND --> END: 连续5帧确认上锁
    LAND --> HOVER_WAIT: 超时25s未确认
    HOVER_WAIT --> END: 人工遥控接管
    TAKEOFF --> END: 异常/急停
    NAVIGATE --> END: 异常/急停
    DESCEND --> END: 异常/急停
```

## 各状态详解

### IDLE — 等待
- 等待 `start()` 调用（按键触发）
- 同时完成 T265 初始化和串口连接

### TAKEOFF — 盲飞离地
- 盲飞离地至 35cm（不依赖T265）
- 等待 T265 置信度 ≥ 2，超时 8s
- 激活航向保持，开始 PID 爬升至目标高度

### NAVIGATE — 逐航点飞行
- 每个航点：PID控制 → 滑动窗口到达确认 → 悬停1.5s → 下一个
- 两种到达策略：`precision`（精确） / `cruise`（巡航）
- 首尾航点自动降级为 `precision` 保证精度

### DESCEND — 两级缓降
- 水平开环控制，ramp 方式降高（每周期 -0.45cm）
- 贴地检测：激光高度 < 8cm 连续 10 帧 → 转 LAND
- 固件可能通过近地强制保护抢先锁桨 → 转 LAND
- 超时 20s → 转 HOVER_WAIT

### LAND — 等待锁桨
- 循环发送 FC_Lock 指令（101），持续清零速度
- 双确认去抖：连续 5 帧 `unlock_sta==0` 且 `motor_pwm_mask==0` → 转 END
- 超时 25s 未确认 → 转 HOVER_WAIT

### HOVER_WAIT — 等待人工介入
- **不会自动退出**：保持悬停，等待人工遥控器接管
- 持续发送控制帧和 T265 速度参考，维持飞控悬停
- 人工遥控器上锁或断电后退出

## 安全保护机制

| 机制 | 触发条件 | 动作 |
|------|---------|------|
| 飞控帧超时 | 2s 无飞控帧 | 急停 |
| T265 位姿丢失 | 置信度异常 | 急停 |
| 激光高度异常 | 高度>10m（错误码） | 滤除 |
| 航向保持跑飞 | yaw误差>8°立即锁存<br/>或1秒内增长>15° | 锁存故障，输出0°/s |
| 近地强制锁定 | 固件层保护 | 锁桨 |

---

[目录结构 →](directory-tree.md)
