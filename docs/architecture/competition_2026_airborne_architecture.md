# competition_2026 无人机端架构

> 2026 年电子设计竞赛无人机赛题备赛版本。基线来自 `drone_control/basic`，
> 所有比赛特定功能在此目录独立开发，不修改稳定基础版本。

---

## 目录

1. [设计原则](#1-设计原则)
2. [模块总览](#2-模块总览)
3. [启动与关闭时序](#3-启动与关闭时序)
4. [航点事件总线](#4-航点事件总线)
5. [可选后台服务](#5-可选后台服务)
6. [通信协议](#6-通信协议)
7. [飞行安全边界](#7-飞行安全边界)
8. [配置参考](#8-配置参考)
9. [验证与验收](#9-验证与验收)
10. [已知限制](#10-已知限制)

---

## 1. 设计原则

### 核心约束

| 原则 | 说明 |
|------|------|
| **不阻塞飞控** | 所有可选服务（截图、动作、视频、UDP）运行在独立线程，使用有界队列通信，故障不修改航线 |
| **完赛优先** | 航点超时、动作失败、截图失败、视频断流、通信断开只记录 warning，继续后续航点 |
| **安全底线保留** | 飞控串口超时、T265 完全失联、人工急停和现有降落/上锁路径不受新模块影响 |
| **默认关闭** | 所有可选服务默认 `enabled: false`，硬件确定后再启用 |
| **配置驱动** | `competition_config.json` 统一管理所有行为，无需修改代码即可切换方案 |

### 不实现（用户明确要求）

- 航线安全校验 / 电子围栏 / 偏航触发的返航或降落
- 场地坐标变换（初始位置和朝向固定，直接使用 T265 坐标系）
- 复杂桌面仿真（只保留单元测试，板载真实硬件验证）

---

## 2. 模块总览

```
drone_control/competition_2026/
├── competition_main.py          # 入口编排器
├── main.py                      # 飞行控制入口（被编排器调用）
├── Mission_GPT.py               # 飞控状态机
├── competition_config.json      # 任务与各服务配置
├── README.md                    # 用户文档
│
├── Lcode/
│   ├── mission_events.py        # 核心：非阻塞事件总线
│   ├── mission_session.py       # 会话持久化（JSON 文件）
│   ├── mission_outcome.py       # 任务结果跟踪器
│   ├── competition_plan.py      # 航线规划与校验
│   │
│   ├── video_source.py          # 视频抽象接口 + 工厂
│   ├── video_backends.py        # OpenCV 采集 / UDP-JPEG 发布
│   ├── airborne_video.py        # 机载视频生命周期管理
│   ├── waypoint_snapshot.py     # 到达点位自动截图
│   │
│   ├── action_executor.py       # 航点动作异步执行
│   ├── gpio_led.py              # RGB LED（含优先级租约）
│   ├── gpio_button.py           # 一键起飞按钮
│   │
│   ├── preflight.py             # 静态飞行前检查
│   ├── drone_link.py            # HMAC 认证 UDP 通信链路
│   │
│   ├── Lprotocol.py             # 飞控串口协议
│   ├── heading_hold.py          # 航向保持 PID
│   ├── navigation_profile.py    # 导航参数配置
│   └── resource_monitor.py      # 板载资源监控
│
└── test_*.py                    # 单元测试
```

### 模块职责

#### 核心框架

| 模块 | 职责 | 线程模型 |
|------|------|----------|
| `competition_main.py` | 编排器：加载配置 → 预检 → 可选服务启动 → 调度 `main.py` → 后台停止 → 汇总结果 | 主线程串行 |
| `main.py` | 飞行入口：GPIO 门禁 → T265 初始化 → 飞控串口 → 创建 `Mission_GPT` 状态机 | 主线程 |
| `Mission_GPT.py` | 飞控状态机：起飞 → 导航 → 到达确认 → 动作事件 → 降落/返航 | 飞控主循环 (~30ms) |

#### 事件与数据

| 模块 | 职责 | 关键设计 |
|------|------|----------|
| `mission_events.py` | 线程安全事件总线，`publish()` 永不阻塞 | 有界队列 (256)；消费者异常隔离；`dropped_events` 计数器 |
| `mission_session.py` | 两次飞行会话的 JSON 文件持久化 | `sessions/<timestamp>/` 含 `session.json`、`<phase>_plan.json`、`<phase>_result.json`、`events.jsonl`、`snapshots/` |
| `mission_outcome.py` | 线程安全结果跟踪器，只做内存操作 | 单向状态收敛；`snapshot()` 超时兜底返回最后已知结果；异常不抛出 |
| `competition_plan.py` | 从 JSON 配置加载航点序列，生成 `scout`/`execute` 阶段路线 | 独立于飞控，可在无硬件时 `--dry-plan` |

#### 可选后台服务

| 模块 | 职责 | 默认状态 | 熔断机制 |
|------|------|----------|----------|
| `waypoint_snapshot.py` | 订阅 `ACTION_REQUESTED`，在后台线程调用 `VideoSource.snapshot()` | 关闭 | `max_consecutive_failures` 次连续失败 → `SNAPSHOT_CIRCUIT_OPEN` |
| `airborne_video.py` | 管理 `VideoSource → VideoPublisher` 生命周期 | 关闭 | `max_consecutive_failures` 次连续失败停止发布线程 |
| `video_backends.py` | `OpenCvVideoSource`（UVC/RTSP）+ `UdpJpegPublisher`（UDP 分片） | 后端未注册 | 启动失败不影响飞行（`required=false`）|
| `action_executor.py` | 消费 `ACTION_REQUESTED` 有界队列，异步执行 `depart/return/noop/observe/signal` | 开启（仅内置动作） | 队列满立即发布 `ACTION_FAILED(queue_full)`，不阻塞飞控 |
| `drone_link.py` | HMAC-SHA256 认证 UDP 链路，事件上报 + `execute_plan` 接收 | 关闭 | `execute` 阶段可选等待认证的航线计划；飞行中仅上报 |

#### 飞行安全

| 模块 | 职责 |
|------|------|
| `preflight.py` | 静态检查：航线完整性、动作合法性、磁盘空间、会话可写、可选服务就绪状态 |
| `gpio_led.py` | RGB LED 控制 + 优先级租约系统（`SAFETY > STARTUP > ACTION`），防止多模块争抢冲突 |
| `gpio_button.py` | BCM5（物理Pin29）一键起飞按钮，物理门禁 |

---

## 3. 启动与关闭时序

### 启动顺序

```text
competition_main.py
│
├─ 1. 加载 competition_config.json
│      ├─ load_competition_config() → 航线规划
│      ├─ load_video_catalog() → 视频配置
│      ├─ load_snapshot_policy() → 截图策略
│      ├─ load_action_policy() → 动作策略
│      ├─ load_preflight_config() → 预检配置
│      └─ load_drone_link_config() → 链路配置
│
├─ 2. 创建 MissionSession
│
├─ 3. 创建并启动 MissionEventBus
│      └─ event_bus.subscribe(session.record_event)
│
├─ 4. 可选：等待认证 execute_plan（--phase execute 且 drone_link 启用时）
│      └─ 收到有效 plan 后原子切换为 REPORT_ONLY
│
├─ 5. 启动可选后台服务（按顺序，每个有硬超时）
│      ├─ WaypointSnapshotConsumer ──── 若 auto_snapshot.enabled
│      ├─ AirborneVideoManager ──────── 若 airborne_video.enabled
│      ├─ WaypointActionExecutor ────── 若 actions.enabled
│      ├─ DroneLinkServer ───────────── 若 drone_link.enabled
│      └─ event_bus.subscribe(各服务 handler)
│
├─ 6. 运行静态预检 CompetitionPreflight.run()
│      └─ 失败时：required 服务失败 → 终止；非 required → warning 继续
│
├─ 7. 导入 main.main() ─── 飞控阶段
│      ├─ wait_for_start_button() → GPIO BCM5（物理Pin29）
│      ├─ T265 初始化
│      ├─ 飞控串口启动
│      ├─ Mission_GPT.start() → 红灯5s → 状态机循环
│      └─ 返回时已降落上锁
│
└─ 8. 关闭阶段（finally）
       ├─ 切断新动作接受 / 命令接收
       ├─ 按依赖逆序停止后台服务（带超时）
       ├─ 收集各服务统计
       ├─ 汇总 MissionOutcomeTracker.finalize()
       ├─ session.finish(status, result)
       └─ 写入 <phase>_result.json
```

### 关闭路径

| 触发条件 | 飞控行为 | 后台服务 | 最终状态 |
|----------|----------|----------|----------|
| 全部航点完成 → 降落 | `ROUTE_COMPLETED` | 停止接受新动作 | `COMPLETED` / `COMPLETED_WITH_WARNINGS` |
| 用户 KeyboardInterrupt | `mission.emergency()` → `stop_all()` | 全部停止 | `INTERRUPTED` |
| 起飞前取消 | 不进入飞控 | 全部停止 | `CANCELLED` |
| 预检失败（required） | 不进入飞控 | 已启动的服务停止 | `PREFLIGHT_FAILED` |
| 硬件故障（飞控/T265） | `HARDWARE_FAILED` | 全部停止 | `HARDWARE_FAILED` |
| 人工急停 | `EMERGENCY_STOPPED` | 全部停止 | `EMERGENCY_STOPPED` |

**注意事项：**

- `stop_all()` 保持严格顺序：先写零速/上锁指令，再停资源监控/飞行日志/T265，最后清除 `task_running`
- 结果跟踪器在硬件清理之后才汇总，不阻塞安全路径
- 所有后台线程为守护线程，`stop()` 有限等待，不持有飞控资源锁

---

## 4. 航点事件总线

### 事件类型

```python
WAYPOINT_APPROACHING  # 开始飞向航点
WAYPOINT_ARRIVED      # 确认到达航点（位置+高度容差内）
HOLD_STARTED          # 开始定点停留
ACTION_REQUESTED      # 请求执行航点动作（非阻塞）
ACTION_COMPLETED      # 动作执行成功
ACTION_FAILED         # 动作执行失败
WAYPOINT_LEFT         # 离开航点（正常到达或超时）
SNAPSHOT_SAVED        # 截图保存成功
SNAPSHOT_FAILED       # 截图失败
SNAPSHOT_CIRCUIT_OPEN # 连续截图失败 → 熔断
TASK_STARTED          # 任务开始
TASK_FINISHED         # 任务结束
SERVICE_STATUS        # 后台服务状态更新
```

### 数据流

```text
Mission_GPT.navigate()
    │
    ├─ event_bus.publish(ACTION_REQUESTED)  ← 非阻塞，队列满直接丢弃
    │       │
    │       ├─ [action_executor] handle_event → 有界队列 → 守护线程执行
    │       │       ├─ depart/return/noop/observe → ActionResult(True)
    │       │       ├─ signal → acquire_rgb_led() → sleep → release
    │       │       └─ 未知动作 → ACTION_FAILED(unsupported)
    │       │
    │       └─ [snapshot_consumer] handle_event → 有界队列 → 守护线程
    │               └─ video_source.snapshot() → SNAPSHOT_SAVED / FAILED
    │
    ├─ event_bus.publish(WAYPOINT_LEFT)
    │       └─ [mission_session] record_event → events.jsonl 追加写入
    │
    └─ navigate() 继续下一航点（永不等待动作/截图结果）
```

### 线程模型

```text
┌─────────────────────────────────────────────────┐
│                 主线程                            │
│  competition_main → main → Mission_GPT           │
│  状态机循环 (~30ms)    publish(event)            │
└────────────────────┬────────────────────────────┘
                     │ 有界队列
                     ▼
┌─────────────────────────────────────────────────┐
│          事件总线线程 (mission-events)            │
│  逐个 dispatch 到已注册 handler                  │
│  异常隔离：一个 handler 失败不影响其他             │
└────┬────────┬────────┬────────┬─────────────────┘
     │        │        │        │
     ▼        ▼        ▼        ▼
  session    action    snap     drone_link
  (JSONL)    (守护)    (守护)    (守护)
```

---

## 5. 可选后台服务

### 5.1 航点动作执行器

```text
ActionPolicy
├── enabled: true                     # 默认启用
├── allowed_actions: [depart,return,noop,observe,signal]
├── signal_color: B                   # LED 信号颜色
├── signal_duration_s: 0.3            # LED 闪烁时长
├── queue_size: 16                    # 有界队列上限
└── stop_timeout_s: 0.5              # 停止等待上限

WaypointActionExecutor
├── 注册处理器: depart/return/noop/observe → _acknowledge
│              signal → _signal (GPIO LED)
├── 统计: accepted, completed, failed, dropped, unknown
└── 安全: 队列满 → ACTION_FAILED(queue_full)
          未知动作 → ACTION_FAILED(unsupported)
          stop() → 丢弃全部待处理事件 → 发布 ACTION_FAILED(shutdown)
```

### 5.2 自动截图

```text
SnapshotPolicy
├── enabled: false
├── required: false                    # required=true 时启动失败终止任务
├── trigger_actions: [observe,snapshot,inspect]
├── timeout_s: 1.0                     # 单次截图超时
├── max_snapshots: 32                  # 总数上限
├── max_consecutive_failures: 3        # 连续失败熔断
├── queue_size: 8
└── stop_timeout_s: 0.5

WaypointSnapshotConsumer
├── 收到 ACTION_REQUESTED → 入队（非阻塞写）
├── 后台线程: video_source.snapshot()
├── 临时文件 → 原子替换（同目录 .tmp → 最终）
	└── 连续 max_consecutive_failures 次失败 → SNAPSHOT_CIRCUIT_OPEN 停止
	```

### 5.3 视觉伺服精准降落

视觉伺服精准降落是 Mission_GPT 的原生状态（`VISUAL_SERVO`），
在 30ms 主循环内同步执行，不经过 ActionExecutor。

#### 核心架构

```
CyberCAM（核桃派）                       Pi（sunrise 板）
─────────────────                       ────────────────
捕获 1920×1080 @ 30fps           UART    cyber_cam_reader.py
↓                              ──────→  ├── 后台线程读 UART
OpenCV 黑色方块检测                     └── 解析 → Detection(dx,dy,found)
↓                                                ↓
计算 dx/dy(像素中心偏移)                   servo_controller.tick(detection, alt)
↓                                                ↓
encode → AA{dx},{dy},{found}              set_speed(vx_cm_s, vy_cm_s, yaw, z)
     (ASCII, 115200 baud)
```

#### 状态机

```
NAVIGATE →到达 + action="visual_servo_land" → VISUAL_SERVO
VISUAL_SERVO → 每30ms：读UART → tick() → set_speed()
VISUAL_SERVO → 对中成功/超时 → _advance_waypoint() → LAND
```

#### 模块文件

```
drone_control/competition_2026/vision/
├── servo_controller.py       # VisualServoController — tick-based IBVS
├── cyber_cam_reader.py       # UART 读取 + 协议解析
├── square_detector.py        # OpenCV 检测（桌面调试/USB 备用）
├── test_servo_controller.py  # 10 个控制器测试
└── test_square_detector.py   # 9 个检测器测试

CyberCamera/boards/cybercam/（部署到 CyberCAM 板）
├── main.py                   # 入口：捕获→检测→UART 发送
├── detector.py               # 黑色方块检测（1920×1080）
├── protocol.py               # ASCII 协议编解码
└── calib.py                  # 焦距标定工具
```

#### 安全保护

| 场景 | 行为 |
|------|------|
| UART 无数据 | 5s SEARCHING 超时 → failed → LAND（坐标降落）|
| 检测丢失（方块出画面）| CENTERING 丢帧 → 保持速度 0 → 超时 → LAND |
| 高度低于 0.3m | 停止修正，立即转 LAND（盲降）|
| CyberCAM 未启动/未安装 | `video_src=None` → 超时兜底，与正常降落无异 |
| 紧急停止触发 | loop() 顶部捕获，与任何状态相同 |

### 5.3 机载视频

```text
AirborneVideoConfig
├── enabled: false
├── required: false
├── source: VideoSourceConfig          # 摄像头或网络流
├── publisher: VideoPublisherConfig    # UDP-JPEG 发布
├── max_fps: 5.0
├── read_timeout_s: 0.5
├── start_timeout_s: 3.0
├── stop_timeout_s: 0.5
└── max_consecutive_failures: 5

AirborneVideoManager
├── start() → source.start() → publisher.start() → 启动采集线程
├── 采集循环: read_frame → publish_frame → sleep(1/fps)
├── 熔断: max_consecutive_failures 次失败 → circuit_open → 线程退出
└── stop() → 停止采集线程 → source.stop() → publisher.stop()
```

**视频后端：**

```text
OpenCvVideoSource (VideoSource)
├── 支持: UVC 摄像头 (/dev/videoX), RTSP 网络流, 本地文件
├── 异步捕获线程 + Condition 传递帧
├── snapshot() → 临时文件 → 原子替换 JPEG
└── 延迟导入 cv2，未安装时明确失败

UdpJpegPublisher (VideoPublisher)
├── 分割帧为 UDP-JPEG 分片
├── 头部: Magic(DJPG) + Version + FrameID + ChunkIndex + ChunkCount + Size + CRC32
├── 默认分片 1200 bytes (MTU 安全)
└── 纯发送端，不实现地面接收
```

### 5.4 无人机链路

```text
DroneLinkConfig
├── enabled: false
├── required: false
├── bind_host: "0.0.0.0", bind_port: 5601
├── remote_host: "127.0.0.1", remote_port: 5602
├── heartbeat_interval_s: 1.0
├── max_datagram: 1200
├── allowed_sources: []               # 空 = 允许所有
├── message_timeout_s: 10.0           # 消息时间窗口
├── execute_plan_wait_s: 30.0         # 等待地面站下发 plan
└── mode: "REPORT_ONLY" / "ACCEPT_PLAN"

协议安全:
├── 遥测: Version + Type + SeqNum + Timestamp + RunID + Payload + CRC32
├── execute_plan 命令: 额外 HMAC-SHA256(psk, nonce + payload)
├── PSK 仅从 DRONE_LINK_PSK 环境变量读取
├── 严格递增 SeqNum 防重放
└── 飞行中仅上报，不接受改航命令
```

---

## 6. 通信协议

### 6.1 UDP 遥测帧

```
Offset  Size  Field
0       1     Protocol Version (1)
1       1     Message Type
2       4     Sequence Number (big-endian, monotonic)
6       8     Unix Timestamp (double, big-endian)
14      4     CRC32 (payload, big-endian)
18      N     JSON Payload
```

### 6.2 UDP-JPEG 视频帧

```
Offset  Size  Field
0       4     Magic "DJPG"
4       1     Version (1)
5       4     Frame ID (big-endian)
9       2     Chunk Index
11      2     Chunk Count
13      2     Payload Size
15      4     CRC32 (full JPEG, big-endian)
19      N     JPEG Chunk Payload
```

### 6.3 串口协议（飞控 ↔ 树莓派）

沿用 `basic` 版本协议，`Lcode/Lprotocol.py` 实现。详见源码 `ANO_LX_FC_*/FcSrc/` 固件端定义。新增帧 2 的 `land_timeout_gaveup` bit 用于降落超时放弃状态上报。

---

## 7. 飞行安全边界

### 保留的安全机制（不受新模块影响）

| 机制 | 触发条件 | 行为 |
|------|----------|------|
| 飞控串口超时 | 串口无数据超过阈值 | `HARDWARE_FAILED` → 降落 |
| T265 完全失联 | T265 停止输出位姿 | `HARDWARE_FAILED` → 降落 |
| 人工急停 | KeyboardInterrupt | `EMERGENCY_STOPPED` → `stop_all()` |
| 近地强制锁定 | 高度 < 10cm 持续约 1s | `FC_Lock()` + 清零 PWM |
| 落地超时 | 降落触发后约 10s | `FC_Lock()`（有高度门槛） |
| 一键起飞门禁 | BCM5（物理Pin29）物理按钮 | 按前不初始化 T265/飞控 |

### 板载硬件测试脚本

| 脚本 | 测试项 | 风险 |
|------|--------|------|
| `hardware_preflight.py` | Python 版本、GPIO、T265 枚举、飞控串口收帧、磁盘空间、可执行文件 | 零（只读，不控制飞行器） |
| `link_hardware_check.py` | UDP 回环、CRC 校验、JPEG 分片、事件序列化、HMAC 签名、局域网对端 | 零（socket 操作，不连飞控） |
| `video_hardware_check.py` | OpenCV 可用性、摄像头枚举、帧读取、帧率统计、JPEG 截图 | 零（不启动飞控） |

**板载验证顺序**：`hardware_preflight` → `link_hardware_check` → （可选）`video_hardware_check` → `pytest`（139 passed on board）→ 拆桨台架 → 真机飞行。

### 不新增的安全触发

- ⛔ 位置偏差不触发自动返航或降落
- ⛔ 动作执行失败不修改航线
- ⛔ 截图/视频断流不触发急停
- ⛔ UDP 通信断开不影响飞行

---

## 8. 配置参考

完整配置见 `competition_config.json`。关键段：

```json
{
  "name": "2026 competition preparation field",
  "cruise_height_m": 1.0,
  "home_hold_s": 1.0,

  "scout_order": ["P1","P2","P3","P4","P5","P6"],
  "points": [
    {"id": "P1", "x": -0.6, "y": 0.0, "hold_s": 2.0, "action": "observe"}
  ],

  "auto_snapshot": {
    "enabled": false,
    "required": false,
    "trigger_actions": ["observe", "snapshot", "inspect"],
    "timeout_s": 1.0,
    "max_snapshots": 32,
    "max_consecutive_failures": 3
  },

  "video": {
    "active_profile": "none",
    "profiles": {
      "capture_device": { "receiver": { "backend": "capture_device",
        "source": "/dev/video0", "options": {...} },
        "publisher": { "enabled": false } },
      "board_network": { "receiver": { "backend": "network_stream",
        "source": "rtsp://...", "options": {...} },
        "publisher": { "enabled": true,
        "target": "rtsp://...", "options": {...} } }
    }
  },

  "actions": {
    "enabled": true,
    "allowed_actions": ["depart","return","noop","observe","signal"],
    "signal_color": "B",
    "signal_duration_s": 0.3
  },

  "preflight": {
    "min_free_mb": 128
  },

  "drone_link": {
    "enabled": false,
    "required": false,
    "bind_port": 5601,
    "mode": "REPORT_ONLY"
  }
}
```

---

## 9. 验证与验收

### 验证层次

| 层次 | 工具 | 目标 |
|------|------|------|
| **单元测试** | `pytest -q` | 确定性逻辑：队列、状态转换、协议编码、配置加载、入口编排 |
| **静态检查** | `compileall -q`, `git diff --check` | 语法正确、无阻塞 `import`、不新增地理围栏逻辑 |
| **桌面测试** | `--dry-plan`, UDP 回环, 内存帧 | 快速排错，不依赖硬件 |
| **板载只读测试** | `hardware_preflight.py` | GPIO 模块导入、T265 枚举、飞控串口开帧 |
| **板载台架验证** | 拆桨后全链路 | 按钮门禁、动作 LED、服务启停、中断恢复 |
| **真机飞行** | 安全确认后 | 完整航线执行、多轮验收 |

### 验收清单

- [ ] `138 passed, 1 skipped`（全部单元测试通过）
- [ ] `compileall -q` 无错误
- [ ] 未新增地理围栏 / 偏航返航 / 自动降落分支
- [ ] 所有可选服务默认关闭
- [ ] `--dry-plan` 输出与配置一致
- [ ] 板载非解锁脚本正常运行
- [ ] 拆桨台架验证：按钮 → 绿灯 → T265 → 红灯 → 状态机 → 降落
- [ ] 用户最终安全确认后真机验证

---

## 10. 已知限制

1. **OpenCV 依赖**：`video_backends.py` 延迟导入 `cv2`，未安装时相关服务失败，不影响默认关闭模式。
2. **Hobot.GPIO 依赖**：板载 GPIO 操作只在树莓派 / 香橙派等 Linux 板上有实际效果，本机运行返回 fallback 实现。
3. **UDP JPEG 丢包**：基于 UDP 分片的视频流不保证有序或可靠到达，适用局域网场景，不适用跨互联网。
4. **单次任务截图上限**：`max_snapshots: 32` 限制总截图数，超限后新截图静默丢弃（不熔断）。
5. **事件队列溢出**：事件总线 `max_queue_size: 256`，瞬时峰值超过时丢弃新事件、递增 `dropped_events` 计数器，不影响飞控。
6. **结果跟踪器超时**：`snapshot()` 调用有 10ms 锁超时，极端争抢下返回最后已知快照，不阻塞调用方。
7. **LED 租约非持久**：`acquire_rgb_led()` 返回的 token 仅用于进程内仲裁，不跨进程。SIGKILL 后再次初始化会先全部拉低。
8. **板载测试初始状态**：首轮不上解锁脚本，预检记录电池 / 固件字段可用时上报，不可用时 `not_available`。
