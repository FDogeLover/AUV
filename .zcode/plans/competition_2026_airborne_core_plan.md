# 计划：2026 备赛无人机端核心模块补全

## 问题描述 & 目标

`drone_control/competition_2026` 已具备稳定基础飞行、`scout/execute` 两阶段航线、航点事件、会话记录、双图传抽象和异步截图。当前仍缺少可验证的航点动作执行、准确任务结果、统一预检、机载视频生命周期以及无人机端通信协议，导致 `action` 仍只是字符串、取消任务也可能被记录为 `finished`，可选视频发布器没有运行管理，远端也无法可靠接收无人机状态或在第二次起飞前提交执行点位。

本计划只构建无人机端代码，不实现地面站、小车或复杂自动识别。根据用户明确要求：

- **不增加航线安全校验、电子围栏或偏航触发的返航/降落。** 即使实际轨迹偏离规划，也继续访问后续航点以争取完赛；
- **不增加场地坐标变换。** 无人机初始位置和朝向固定，配置坐标直接使用现有 T265 坐标系；
- **不建设复杂桌面仿真。** 只保留能防止接口回归的轻量单元测试，主要通过板载真实 GPIO、T265、飞控串口和可用摄像头做非解锁验证；
- 现有飞控串口超时、T265 完全失联、人工急停和降落保护属于硬件安全底线，保持原样，不能因“继续完赛”要求而删除。

最终目标是：无人机端可独立完成配置预检、动作分发、任务结果汇总、机载视频采集/发布、事件上报和第二次任务接收；所有新增后台功能失效时不阻塞 30 ms 飞控主循环、不改变既定航线、不触发新的返航/降落逻辑。完成代码、测试、板载非解锁验证、多轮审查和文档后，统一提交并推送。

## 方案选择

| 方案 | 优点 | 缺点/风险 |
|------|------|----------|
| 方案A：继续把动作、结果、通信和视频逻辑直接写入 `Mission_GPT.py` | 文件少，能直接访问状态机字段 | 安全关键文件膨胀；网络、GPIO或视频阻塞会拖慢飞控；难以单独测试和关闭 |
| 方案B：以本地事件总线为核心，增加独立有界后台模块，`Mission_GPT.py` 只负责准确发布状态与结果 | 与飞控循环隔离；模块可默认关闭和独立熔断；便于板载逐项验证 | 生命周期和结果汇总需要统一协调，入口改动较多 |
| 方案C：引入 ROS/Foxglove 或完整消息中间件 | 可视化和生态完整 | 部署复杂、资源占用高、超出当前单机无人机需求 |

**选择方案B。** 飞控状态机继续只处理起飞、导航和降落；动作、UDP通信和视频全部使用有界队列/独立线程，故障只形成事件和告警。`competition_main.py` 作为无人机任务编排器，在导入飞控入口和等待起飞按钮前完成静态预检与可选服务启动。

## 改动范围

### 1. 航点动作执行

- `drone_control/competition_2026/Lcode/action_executor.py` — 新增 `ActionPolicy`、`ActionResult`、处理器注册表和 `WaypointActionExecutor`。消费 `ACTION_REQUESTED`，采用有界队列；内置 `depart/return/noop/observe` 确认处理和 `signal` RGB LED 动作；发布 `ACTION_COMPLETED`、`ACTION_FAILED`，统计接受/成功/失败/丢弃/未知动作。动作请求与航点推进彻底解耦：队列满时立即回注 `ACTION_FAILED(queue_full)`，但 `Mission_GPT.navigate()` 永不等待动作结果，仍按既有停留时间推进下一航点。
- `drone_control/competition_2026/Mission_GPT.py` — 保留 `ACTION_REQUESTED` 触发，移除当前“未真正执行就直接发布 `ACTION_COMPLETED`”的错误语义；航点超时仍推进下一点，不新增偏航中止逻辑。
- `drone_control/competition_2026/Lcode/mission_events.py` — 增加动作失败、任务启动/结束和服务状态事件常量。
- `drone_control/competition_2026/competition_config.json` — 增加默认启用且仅包含安全内置动作的 `actions` 配置；未知动作在起飞前预检失败，防止赛中静默跳过。

### 2. 任务总控与准确结果

- `drone_control/competition_2026/Lcode/mission_outcome.py` — 新增线程安全 `MissionOutcomeTracker`、状态枚举和不可变结果对象。至少区分 `NOT_STARTED/RUNNING/ROUTE_COMPLETED/COMPLETED/COMPLETED_WITH_WARNINGS/CANCELLED/PREFLIGHT_FAILED/HARDWARE_FAILED/EMERGENCY_STOPPED/INTERRUPTED`，记录原因、航点超时数、动作失败数和是否完成全航线。跟踪器方法只做内存字段更新，内部吞掉诊断性异常并提供无锁快照兜底，不执行日志、JSON或磁盘I/O。
- `drone_control/competition_2026/Mission_GPT.py` — 在启动取消、正常完成、航点超时、硬件失联和急停路径更新结果；`start()` 明确返回是否真正启动。`stop_all()` 保持严格顺序：第一步写上锁/零速指令 → 停资源监控/关飞行日志/停T265 → 尝试非抛出式结果收敛 → 在 `finally` 中清除 `task_running`。结果跟踪器失败只丢诊断结果，绝不能阻断硬件清理。
- `drone_control/competition_2026/main.py` — 返回结构化飞行结果；按钮失败、串口初始化异常、用户中断和任务未启动不再统一表现为 `None/finished`。异常路径仍先执行现有急停/资源释放。
- `drone_control/competition_2026/competition_main.py` — 使用真实飞行结果写入 `session.json` 和 `<phase>_result.json`；飞控返回 `ROUTE_COMPLETED` 后，先停止接收新动作、有限等待/停止动作执行器并读取统计，再由编排器最终确定 `COMPLETED` 或 `COMPLETED_WITH_WARNINGS`。这样最后一条延迟动作失败不会在 `COMPLETED` 之后反向修改终态。结果JSON构造和写盘位于飞控串口/T265/GPIO安全清理之后，并有独立异常隔离。
- `drone_control/competition_2026/Lcode/mission_session.py` — 保存结构化任务结果、各后台模块统计和 `run_id`，继续使用临时文件原子替换。

### 3. 无人机端预检

- `drone_control/competition_2026/Lcode/preflight.py` — 新增 `PreflightCheck/PreflightReport/CompetitionPreflight`。检查配置可解析、航线非空、首尾为 `HOME`、高度为正、动作均有处理器、会话/日志目录可写、剩余磁盘空间、必需视频/通信服务是否启动。只做配置和依赖完整性检查，**不检查地理边界、路线偏差、总距离或坐标变换**。
- `drone_control/competition_2026/main.py` — 板载硬件阶段验证飞控串口已收到数据、T265启动结果和按钮/GPIO结果，并映射为结构化结果；不重复实现 `Mission_GPT` 已有的置信度人工确认。
- `drone_control/competition_2026/competition_main.py` — 在导入 `main` 和等待按钮前运行静态预检；失败时飞控不会解锁。可选服务失败按配置降级，必需服务失败终止启动。
- `drone_control/competition_2026/hardware_preflight.py` — 新增板载只读检查脚本：检测 GPIO模块导入、T265枚举/启动、飞控串口打开及短时间收帧、磁盘空间；默认绝不发送解锁/起飞指令。

### 4. 机载视频生命周期与可选真实后端

- `drone_control/competition_2026/Lcode/airborne_video.py` — 新增 `AirborneVideoConfig/Manager`，管理机载 `VideoSource → VideoPublisher` 的启动、取帧、发布、连续失败熔断、统计和有限等待停止。source/publisher 启动和停止均包装硬超时；后台线程不持有飞控资源；视频失败不影响航线。默认CPU保守参数为 640×360、5fps、JPEG质量55，队列只保留最新帧，编码循环每帧主动让出调度。
- `drone_control/competition_2026/Lcode/video_backends.py` — 提供可选、延迟导入 OpenCV 的 `OpenCvVideoSource`（UVC/RTSP、本地原子截图）和 `UdpJpegPublisher`（JPEG编码、UDP分片、序号和帧完整性字段）；未安装 OpenCV 时明确失败但不影响默认关闭模式。只实现无人机发送端，不实现地面显示程序。
- `drone_control/competition_2026/Lcode/video_source.py` — 增补机载 source/publisher 配置加载与内置后端注册入口，保持现有接口兼容。
- `drone_control/competition_2026/competition_config.json` — 增加默认关闭的 `airborne_video`，预留本地摄像头、UDP目标、帧率、JPEG质量、分片大小和失败阈值。
- `drone_control/competition_2026/video_hardware_test.py` — 板载摄像头测试：读取真实帧、可选保存一张临时图、统计发布帧；默认不启动飞控。

### 5. 无人机端短距离通信

- `drone_control/competition_2026/Lcode/drone_link.py` — 新增默认关闭的 UDP 无人机端链路。普通遥测消息使用版本、类型、递增序号、时间戳、`run_id`、payload 和 CRC32；任何能够提交 `execute_plan` 的命令还必须包含当前无人机随机 `session_nonce` 和 HMAC-SHA256。密钥只从 `DRONE_LINK_PSK` 环境变量读取，不写配置/日志；校验来源、长度、版本、HMAC、nonce、时间窗口和严格递增序号，防止伪造与重放。发送有界队列、心跳、事件转发。运行中只上报，不接受改变航线的命令。
- `drone_control/competition_2026/competition_main.py` — 可选在 `execute` 阶段、飞控初始化之前等待一个经过认证的 `execute_plan` 点位列表。接收器使用单一线程和同一socket，在锁内把模式从 `ACCEPT_PLAN` 原子切换为 `REPORT_ONLY` 后才返回点位；切换后所有改航消息只记录拒绝，不写共享计划。点位仍通过 `plan_mission()` 白名单校验。
- `drone_control/competition_2026/competition_config.json` — 增加 `drone_link` 配置：启用/必需、绑定地址、远端地址、心跳、最大报文、允许来源、第二次任务等待时间。若允许接收第二次任务但环境变量中无PSK，静态预检直接失败；纯遥测模式可不配置PSK。
- `drone_control/competition_2026/link_hardware_test.py` — 板载 UDP 回环/局域网测试，不接飞控也不解锁。

### 6. 入口编排、测试和文档

- `drone_control/competition_2026/competition_main.py` — 统一服务启动顺序：加载配置 → 可选认证接收第二次任务 → 创建会话/run_id → 创建事件总线 → 对每个可选服务执行带硬超时的启动 → 动作执行器/截图/机载视频/无人机链路状态预检 → 导入并运行飞控 → 先切断新动作/命令再按依赖逆序有限停止 → 汇总后台统计 → 最后写结果。任何结果构造或序列化异常都在独立 `try/except` 中处理，不得回跳飞控急停/清理路径。
- `drone_control/competition_2026/test_action_executor.py` — 轻量测试动作过滤、未知动作、队列、LED处理失败和结果事件。
- `drone_control/competition_2026/test_mission_outcome.py` — 测试状态只能合法收敛、警告累计和紧急结果不被正常完成覆盖。
- `drone_control/competition_2026/test_preflight.py` — 测试配置/动作/磁盘/服务必需性，不构造复杂飞行仿真。
- `drone_control/competition_2026/test_airborne_video.py` — 仅用最小内存帧验证生命周期和熔断；真实画面交给板载脚本。
- `drone_control/competition_2026/test_drone_link.py` — 测试消息编码、CRC、重复序号、非法来源/类型和本机 UDP 收发。
- `drone_control/competition_2026/test_competition_entry.py` — 使用立即返回的最小飞行入口替身验证编排和最终状态，不模拟 T265动力学。
- 现有与新测试文件 — 根据接口返回值同步调整，保持原有飞控安全回归测试全部通过。
- `drone_control/competition_2026/README.md` — 更新中文运行方式、模块状态、配置、服务失败策略、板载测试顺序和已知限制。
- `docs/competition_2026_airborne_architecture.md` — 新增无人机端架构、启动/停止时序、消息格式、任务结果含义和真机验收记录。

## 风险点

- **不得改变用户要求的完赛策略：** 航点到达超时、动作失败、截图失败、视频断流和地面通信断开都只形成 warning，继续后续航点；不新增地理围栏、位置偏差返航或自动降落。
- **保留硬件安全底线：** 飞控串口超时、T265完全停止、人工中断及已有降落/上锁路径仍可触发现有急停流程。新增结果记录不得把 `stop_all()` 第一时间写上锁指令的操作后移。
- **动作线程阻塞：** 动作执行器只运行内置短操作，使用单一守护线程和有界队列；处理器卡住不阻塞事件总线/飞控，停止有上限。未知动作在静态预检阶段失败。
- **LED状态冲突：** 在 `Lcode/gpio_led.py` 增加进程内优先级所有权仲裁：`SAFETY > STARTUP > ACTION`。每次设置返回所有权token，关闭操作必须携带相同token；动作线程只能释放自己的 `ACTION` 状态，不能覆盖起飞警示或安全状态。`main.py/Mission_GPT.py` 的启动警示使用更高优先级，正常退出显式清理GPIO。底层GPIO锁只保护短写操作，不由动作线程跨sleep持有。
- **结果竞态：** 结果跟踪器采用锁和单向状态转换；紧急/硬件失败等终态不能被随后 `stop_all()` 的正常清理覆盖。飞控只收敛到 `ROUTE_COMPLETED`，最终 `COMPLETED(_WITH_WARNINGS)` 由所有后台统计收齐后的编排器一次性确定。航点超时和动作失败只累计 warning。
- **UDP错误或恶意数据：** 限定来源、长度、版本、CRC、序号和消息类型；可改任务命令强制HMAC-SHA256、随机nonce、时间窗口和重放保护。第二次任务只在起飞前接收并再次通过 `plan_mission()` 点位白名单校验；飞行中命令永不修改航线。CRC只用于随机错误检测，安全认证依赖HMAC。
- **UDP线程/视频线程阻塞：** socket使用超时，发送队列有界；视频读取必须使用底层超时。所有可选线程为守护线程，停止有限等待，不持有串口/T265/GPIO控制锁。
- **视频带宽：** UDP JPEG只是通用备选，默认关闭；限制分辨率、帧率、质量和分片大小。丢帧优先于排队，网络拥塞不影响控制链路。
- **OpenCV可用性：** 延迟导入，未安装时只让机载视频服务失败；`required=false` 继续飞行，`required=true` 在飞控导入前终止。
- **后台清理：** 正常退出逐个显式停止线程、关闭socket/视频/GPIO并删除临时文件；守护线程只是异常退出兜底。SIGKILL/OOM无法执行Python清理属于已知限制，板载测试必须确认进程重启后LED初始化会先全部拉低。
- **板载测试安全：** 首轮只运行只读/非解锁脚本；预检在飞控遥测可用时记录电池/固件字段，不可用时明确 `not_available` 而非伪造通过。涉及 `main.py` 或真实起飞必须另行确认拆桨/净空/电池/航线，不因本计划自动执行解锁。
- **回退方案：** `actions` 可退回仅 `observe/noop`；`airborne_video.enabled=false`、`drone_link.enabled=false` 可完全关闭新后台服务；任务结果/预检模块不改变控制指令。必要时可整体回退最终提交，不触碰稳定 `basic` 目录。

## 验证方式

- **单元测试：** 运行 `drone_control/competition_2026` 全部 pytest；新增测试只覆盖队列、协议、状态转换和入口编排等确定性逻辑，不搭建复杂动力学模拟。
- **静态检查：** `py_compile`、`git diff --check`，确认没有导入时硬依赖 OpenCV/Hobot GPIO；搜索确认未新增 geofence/boundary/deviation 自动返航或降落分支。
- **桌面测试：** `--dry-plan`、UDP本机回环、内存视频帧和临时会话目录；只用于快速排错。
- **板载非解锁验证：** 同步到开发板后运行 `hardware_preflight.py`、`link_hardware_test.py`、`video_hardware_test.py`（有摄像头时）和完整 pytest；确认 GPIO/T265/飞控收帧但不发送解锁指令。
- **板载台架验证：** 拆桨后验证按钮门禁、动作 LED、服务启停、任务取消/硬件失败结果和飞控串口资源释放。只有用户再次确认安全条件后才进行真实起飞。
- **审查：** 使用明确模型 `Qwen3.8-Max-Preview` 做计划审查、实现 diff 核查；修复偏差后再做一次上板前安全审查，重点核对 `Mission_GPT.py/main.py` 的急停、降落和资源释放没有退化。
- **交付：** 更新 README 与架构/验收文档；代码全部完成并验证后按“任务结果与动作 / 预检与通信 / 视频与文档”拆成可独立回退的本地提交，最后统一推送远端。根据项目约定同步无人机端 Python 文件到板子，但板子仓库只做本地提交、不推送。

## 第一轮 Qoder 高风险处理（持续目标要求修订后复审）

1. **`stop_all()` 结果记录干扰安全清理：** 上锁指令和硬件资源释放保持最前，结果跟踪只做非抛出内存更新并置于资源清理之后；`task_running` 在 `finally` 清除，JSON写盘完全移到外层编排器。
2. **动作队列丢弃导致航点等待：** 明确飞控永不等待动作结果；队列满立即发布失败事件，航点仍按既有停留/超时逻辑推进。
3. **UDP命令伪造：** `execute_plan` 强制使用环境变量PSK、HMAC-SHA256、无人机随机nonce、时间窗口、来源限制和严格序号防重放；无PSK不得开启命令接收。
4. **RGB LED覆盖安全指示：** 增加 `SAFETY/STARTUP/ACTION` 优先级与token所有权，动作线程无法关闭或覆盖高优先级状态。
5. **中风险同步处理：** 服务启动/停止硬超时；飞控先收敛 `ROUTE_COMPLETED`、后台停止后再生成最终结果；链路模式原子切换；视频默认640×360@5fps/JPEG55；结果构造与安全清理隔离；正常退出显式清理全部资源。
6. **协议与回退补充：** 文档定义UDP JPEG重组超时、乱序和丢帧规则；结果状态明确 `CANCELLED` 为起飞前主动取消、`INTERRUPTED` 为进程/用户运行中中断；按模块拆分最终提交以支持独立回退。
