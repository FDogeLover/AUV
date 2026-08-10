# 桌面模拟飞行

不需要任何硬件，就可以在本地电脑上看到完整的状态机运行流程。T265 和飞控都会自动使用模拟数据。

## 运行

```bash
cd drone_control/basic

# Linux/macOS
DRONE_DRY_RUN=1 python main.py

# Windows PowerShell
$env:DRONE_DRY_RUN=1; python main.py
```

## 预期输出

```
[INFO] Logger initialized
[INFO] DRY_RUN mode enabled - hardware will be simulated
[INFO] GPIO Button initialized (dry-run: virtual button)
[INFO] RGB LED initialized (dry-run: console output)
[INFO] Waiting for takeoff button press...
[INFO] Button pressed! Starting takeoff sequence...
[WARN] RED LED - 5 seconds warning, CLEAR THE AREA!
[INFO] T265 initialized (simulated)
[INFO] Serial port opened (simulated)
[INFO] State: IDLE -> TAKEOFF
[INFO] Lifting off to 35cm...
[INFO] T265 confidence OK, starting PID climb to target altitude
[INFO] Heading hold activated, initial yaw locked
[INFO] State: TAKEOFF -> NAVIGATE
[INFO] Navigating to waypoint 1: (0.0, 0.0, 1.0)
[INFO] Waypoint 1 reached, holding 1.5s
[INFO] Navigating to waypoint 2: (-0.6, 0.0, 1.0)
...
[INFO] State: NAVIGATE -> DESCEND
[INFO] Descending to landing height...
[INFO] State: DESCEND -> LAND
[INFO] Motors locked (simulated)
[INFO] State: LAND -> END
[INFO] Flight complete! Flight log saved.
```

!!! success "恭喜！"
    如果看到状态机完整跑完 IDLE → TAKEOFF → NAVIGATE → DESCEND → LAND → END，说明你的环境完全正常，可以进入真机飞行环节了。

## 状态机流程图

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> TAKEOFF: 按键触发
    TAKEOFF --> NAVIGATE: T265置信度OK
    NAVIGATE --> DESCEND: 最后航点到达<br/>高度≤20cm
    DESCEND --> LAND: 贴地确认/固件抢先锁桨
    LAND --> END: 连续5帧确认上锁
    LAND --> HOVER_WAIT: 超时25s未确认
    DESCEND --> HOVER_WAIT: 超时20s
    HOVER_WAIT --> END: 人工遥控接管
    TAKEOFF --> END: 异常急停
    NAVIGATE --> END: 异常急停
```

---

[运行单元测试 →](run-tests.md)
