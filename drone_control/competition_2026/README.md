# competition_2026

本目录是基于 `basic` 开发的 2026 年无人机比赛备赛版本，原有 `basic`
目录继续作为稳定的基础飞行版本保留，不在其中直接叠加比赛任务功能。

当前第一阶段实现了两种可配置任务：

- `scout`：依次访问配置中的全部观察点，然后返回起降点；
- `execute`：按照指定顺序复访选中的关键点，然后返回起降点。

## 航线预览

不连接飞控、T265 和 GPIO 硬件时，可以使用以下命令预览生成的任务航线：

```powershell
python competition_main.py --phase scout --dry-plan
python competition_main.py --phase execute --points P2,P5 --dry-plan
```

只有在确认以下内容后，才可以删除 `--dry-plan` 进行真机测试：

- `competition_config.json` 中的坐标和高度正确；
- T265 坐标轴方向与任务场地一致；
- 全部航点位于安全飞行边界内；
- 点位访问顺序正确；
- 急停、失联和降落保护已经完成检查。

配置文件中的点位坐标只是用于台架和小范围测试的保守占位值，不代表正式比赛场地。

后续计划增加实时视频、点位截图、地面端关键点选择，以及可选的小车执行端。
这些扩展功能应使用统一的点位编号和任务动作，不应改变已经稳定的飞控通信协议。

## 预留图传接口

`Lcode/video_source.py` 定义了与具体硬件无关的 `VideoSource` 接口，用于接收
解码后的视频帧和保存点位截图。视频功能默认关闭，当前的 `none` 后端只用于
航线规划和无硬件测试，不会连接摄像头或接收视频。

配置文件已经预留两个可选方案，当前 `active_profile` 为 `none`，两种方案都不会启动：

- `capture_device`：成品或模拟图传接收器经过 UVC 采集卡接入地面开发板；
- `board_network`：无人机开发板发布网络视频流，地面开发板接收和解码。

确定硬件后，将 `active_profile` 改为对应方案，并注册其接收端实现。网络开发板
方案还需要注册 `VideoPublisher` 发布端实现。任务和飞控代码不需要随之修改。

接收端可根据实际接口增加以下后端：

- 显示为 `/dev/videoX` 的 UVC 摄像头或 UVC 图传接收器；
- RTSP、RTP/UDP 或 HTTP/MJPEG 网络视频流；
- 通过 UVC 采集卡接入的 HDMI/CVBS 图传接收器；
- 机载开发板本地摄像头和本地截图。

模拟图传接收器如果直接连接显示屏，程序只能让人观看画面，无法获得视频帧或
自动截图。此时需要增加 USB 视频采集卡，或者由无人机上的摄像头在到达点位时
本地保存截图。

## 开发板数字图传备选方案

无人机端和地面端可以使用不同型号的开发板传输视频。只要两端网络互通，并支持
相同的视频协议和编码格式，就可以构成自制数字图传：

```text
无人机端摄像头
    → H.264/MJPEG 编码
    → Wi-Fi 网络视频流
    → 地面端开发板解码
    → 6 吋显示屏、截图或图像处理
```

建议使用固定 IP 和独立的比赛局域网，优先采用 5GHz Wi-Fi。视频链路与任务指令
应使用不同端口；条件允许时，关键控制和急停应保留独立的短距离通信链路，避免
视频拥塞影响飞行安全。

推荐的初始视频参数为：

- 分辨率：720p；
- 帧率：10～20fps；
- H.264 码率：1～3Mbps；
- 端到端延迟：尽量低于 300ms；
- 到达观察点后等待画面稳定，再保存截图。

无论使用成品图传还是开发板网络视频流，后续都应通过统一的 `VideoSource`
接口接入任务系统。

## 航点事件与两次飞行会话

备赛入口现在会把点位编号和动作一并传入飞行状态机，并通过后台事件队列发布以下事件：

- `WAYPOINT_APPROACHING`：开始飞向点位；
- `WAYPOINT_ARRIVED`：确认到达点位；
- `HOLD_STARTED`：开始定点停留；
- `ACTION_REQUESTED`：请求观察、截图或其他点位动作；
- `ACTION_COMPLETED`：动作完成；
- `WAYPOINT_LEFT`：离开点位，包含正常到达或超时原因。

事件消费者运行在飞控主循环之外。后续接入截图、识别或地面站时，不应在飞控线程中直接执行耗时操作。

执行真实 `scout` 任务时会在 `sessions` 下创建时间戳目录，并输出其绝对路径：

```text
sessions/<timestamp>/
├── session.json
├── scout_plan.json
├── scout_result.json
├── events.jsonl
└── snapshots/
```

第二次飞行使用 `--session` 复用该目录，使侦察和执行记录归入同一次比赛会话：

```powershell
python competition_main.py --phase execute --points P2,P5 `
  --session "D:\完整路径\competition_2026\sessions\20260723_103000_000000"
```

执行后还会生成 `execute_plan.json` 和 `execute_result.json`。每次 `scout/execute` 启动都会生成独立 `run_id`，用于隔离航点截图去重状态；程序重启或重新执行任务不会继承上一次的去重缓存。

## 到达点位自动截图

`Lcode/waypoint_snapshot.py` 已实现硬件无关的自动截图消费者。收到匹配动作的 `ACTION_REQUESTED` 后，它只在事件线程中执行非阻塞入队，再由独立守护线程调用活动 `VideoSource`。截图成功、失败和熔断分别记录为：

- `SNAPSHOT_SAVED`；
- `SNAPSHOT_FAILED`；
- `SNAPSHOT_CIRCUIT_OPEN`。

功能默认关闭。确定并注册真实图传后端后，需要同时选择活动图传方案并启用自动截图：

```json
{
  "auto_snapshot": {
    "enabled": true,
    "required": false,
    "trigger_actions": ["observe", "snapshot", "inspect"],
    "timeout_s": 1.0,
    "queue_size": 8,
    "max_snapshots": 32,
    "max_consecutive_failures": 3
  },
  "video": {
    "active_profile": "capture_device"
  }
}
```

`capture_device` 和 `board_network` 使用同一个消费者，仅 `VideoSource` 后端不同。当前仓库仍未包含具体 UVC/RTSP 解码实现，因此仅修改 `active_profile` 和 `enabled` 还不能获得真实图片，程序会明确报告后端未注册。

安全策略如下：

- `required=false`：后端不存在或普通初始化失败时关闭截图并继续飞行；
- `required=true`：截图初始化失败时，在导入飞控入口、初始化 GPIO/T265/串口之前终止任务；
- 视频启动调用超时：无论 `required` 取值如何都终止任务，因为可能残留未知状态的底层调用；
- 运行中断流、超时或连续失败：只记录失败并熔断截图，不修改航线，不触发急停或降落；
- 队列、截图总数和连续失败次数都有上限，避免视频故障消耗无限内存或磁盘空间。

具体后端实现 `snapshot(timeout_s)` 时，必须把超时传入底层读取接口，并在会话 `snapshots/` 内先写临时文件、完成编码后原子替换为最终图片。真机启用前先拆桨验证拔线、断流、超时和写盘失败。

## 机载视频流（Airborne Video）

`Lcode/airborne_video.py` 实现了可选的机载摄像头 → 网络视频流生命周期管理，包括
`AirborneVideoConfig` 配置验证和 `AirborneVideoManager` 启动/停止/熔断监控。

`Lcode/video_backends.py` 提供两个参考后端：

- **OpenCvVideoSource**：基于 OpenCV `VideoCapture` 的视频源，支持 UVC 摄像头、RTSP
  网络流和本地视频文件。启动时异步捕获，通过条件变量传递帧，支持带超时的截图。
- **UdpJpegPublisher**：将视频帧分割为 UDP-JPEG 数据报发送到指定 `host:port`，每帧
  携带帧 ID、分片索引、总分片数和 CRC32 校验和。

后端通过 `register_builtin_video_backends()` 统一注册到 `video_source` 的工厂方法。
默认关闭，硬件确定后设置 `competition_config.json` 中的 `video.active_profile`。

## 航点动作执行器（Action Executor）

`Lcode/action_executor.py` 提供 `ActionPolicy` 配置和异步执行器，在飞控主循环之外的
独立线程中执行到达点位的动作（如 GPIO LED 闪烁信号）。支持以下动作类型：

- `depart`：起点出发（默认动作）；
- `return`：返回起降点；
- `noop`：无操作；
- `observe`：观察（配合截图或人工判断）；
- `signal`：通过 GPIO LED 发出彩色信号（红灯常亮或指定颜色闪烁）。

`ActionPolicy` 通过 `allowed_actions` 限制可执行的动作列表。

## 飞行前检查（Preflight）

`Lcode/preflight.py` 包含不控制飞行器的静态飞行前检查：

- Python 版本检查（≥3.10）；
- 磁盘剩余空间检查（默认 ≥128 MB，可通过 `competition_config.json` 的
  `preflight.min_free_mb` 配置）；
- 可执行文件可用性检查（`pytest`, `git`, `python3`）；
- 可选 GPIO 访问检查（BCM 编号）。

检查通过 `run_preflight()` 运行，返回成功/失败列表。失败项通过 `--ignore-preflight`
命令行标志可跳过。

## 任务执行结果（Mission Outcome）

`Lcode/mission_outcome.py` 定义了任务生命周期中的状态枚举和结果跟踪：

| 状态 | 说明 |
|------|------|
| `NOT_STARTED` | 尚未开始 |
| `RUNNING` | 正在执行 |
| `ROUTE_COMPLETED` | 所有航点已访问完 |
| `COMPLETED` | 正常完成（含返航和降落） |
| `COMPLETED_WITH_WARNINGS` | 完成但有警告 |
| `CANCELLED` | 被取消 |
| `PREFLIGHT_FAILED` | 飞行前检查未通过 |
| `HARDWARE_FAILED` | 硬件故障 |
| `EMERGENCY_STOPPED` | 紧急停止 |
| `INTERRUPTED` | 被中断 |

追踪器是线程安全的，支持监听器回调，可在状态变化时触发外部动作（如事件通知）。

## 无人机-地面链路（Drone Link）

`Lcode/drone_link.py` 实现了带认证的 UDP 通信链路，用于地面站向无人机下发指令和
接收任务事件：

- **HMAC-SHA256 认证**：共享密钥防止未授权指令；
- **`execute_plan` 指令**：地面站可向无人机下发 `execute` 阶段的航线计划；
- **事件上报**：无人机通过链路将 `MissionEvent` 实时发送给地面站；
- **双模式**：`ACCEPT_PLAN` 模式下接受地面站下发的航线计划，`REPORT_ONLY` 模式仅
  上报事件不接收指令。

默认关闭（`enabled: false`），启用时需要设置 `competition_config.json` 中
`drone_link` 部分的端口和共享密钥。

## 板载硬件测试脚本

三个独立硬件测试脚本，不发送任何解锁/起飞指令，零风险：

### `hardware_preflight.py` — 只读硬件预检

```bash
python3 hardware_preflight.py                       # 默认5秒
python3 hardware_preflight.py --duration 10         # 延长监听
python3 hardware_preflight.py --skip-t265 --skip-fc # 本机无硬件
```

检查项：Python 版本、GPIO 模块、T265 枚举、飞控串口收帧、磁盘空间、可执行文件。
退出码 0 = 全部通过，非 0 = 至少一项失败但 **绝不尝试解锁或起飞**。

### `link_hardware_check.py` — UDP 链路测试

```bash
python3 link_hardware_check.py                          # 本机回环
python3 link_hardware_check.py --peer --mode listen     # 等待对端
python3 link_hardware_check.py --peer --mode send --remote-host 192.168.x.x
DRONE_LINK_PSK=secret python3 link_hardware_check.py --hmac  # HMAC 验证
```

测试 UDP 收发、CRC 校验、JPEG 分片编解码、事件序列化和 HMAC 签名。

### `video_hardware_check.py` — 摄像头测试

```bash
python3 video_hardware_check.py                           # 自动检测
python3 video_hardware_check.py --source 0 --duration 10  # 帧率统计
python3 video_hardware_check.py --snapshot-dir ./test     # 截图验证
```

检测可用摄像头、读取帧、统计帧率、可选保存 JPEG 截图。

## 模块结构一览

```text
competition_2026/
├── competition_main.py        # 入口：scout/execute 双阶段任务
├── main.py                    # 飞行控制入口
├── competition_config.json    # 任务配置（点位、视频、截图等）
├── Lcode/
│   ├── video_source.py        # 视频源抽象接口
│   ├── video_backends.py      # OpenCV / UDP-JPEG 后端实现
│   ├── airborne_video.py      # 机载视频流生命周期管理
│   ├── waypoint_snapshot.py   # 到达点位自动截图
│   ├── mission_events.py      # 非阻塞航点事件总线
│   ├── mission_session.py     # 任务会话管理
│   ├── mission_outcome.py     # 任务状态与结果跟踪
│   ├── competition_plan.py    # 航线规划
│   ├── action_executor.py     # 航点动作异步执行
│   ├── preflight.py           # 飞行前静态检查
│   ├── drone_link.py          # 无人机-地面 UDP 链路
│   ├── gpio_button.py         # GPIO 按键
│   ├── gpio_led.py            # GPIO LED 控制
│   ├── heading_hold.py        # 航向保持
│   ├── navigation_profile.py  # 导航参数配置
│   ├── Lprotocol.py           # 飞控串口通信协议
│   ├── resource_monitor.py    # 资源监控
│   └── ...                    # 其他核心库
├── hardware_preflight.py      # 板载只读硬件预检脚本
├── link_hardware_check.py     # UDP 通信链路测试脚本
├── video_hardware_check.py    # 摄像头硬件测试脚本
├── sessions/                  # 任务会话目录（运行时生成）
└── test_*.py                  # 单元测试
```

