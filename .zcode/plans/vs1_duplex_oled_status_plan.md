# 计划：VS1 双向通信与 RDK OLED 状态显示

## 问题描述与目标

现有 VS1 链路只由 Cyber Camera 持续向 RDK 发送视觉帧。RDK 能判断 `Cyber Camera -> RDK` 是否收到合法帧，但仅凭本机串口 `write()` 成功，无法证明 `RDK -> Cyber Camera` 的数据确实到达。

目标是在不让 OLED 服务占用 `/dev/ttyS7`、不影响视觉伺服串口接收的前提下：

- OLED 显示 `CAM>RDK:OK/LOST`：最近是否收到合法 VS1 帧；
- OLED 显示 `RDK>CAM:OK/LOST`：RDK 发出的 PING 是否获得 Cyber Camera 的匹配 PONG；
- 任一进程退出、链路拔断或状态长时间不更新时，显示必须自动变为 `LOST`，不能保留假在线；
- 状态显示只用于诊断，不改变解锁、起飞和视觉伺服的安全判据。

## 方案选择

| 方案 | 优点 | 缺点/风险 |
|---|---|---|
| A. 视觉串口进程负责 PING/PONG，并通过 tmpfs 状态快照供 OLED 读取 | 不争抢 UART；能证明双向传输；改动边界清晰 | 视觉串口进程未运行时显示 LOST；需维护轻量状态文件 |
| B. OLED 服务直接打开 UART7 做探测 | 启动后可独立探测 | 会与视觉伺服进程争抢串口字节，存在严重误判和丢帧风险 |
| C. 新建 UART 独占 broker，再通过 IPC 向视觉伺服和 OLED 分发 | 架构最完整，可常驻监测 | 当前阶段改动过大，引入新的单点故障和部署复杂度 |

**选择方案 A。** OLED 是观察者，不接触 UART；`CyberCamReader` 仍是 RDK 侧串口唯一所有者。

## 协议与状态定义

### 双向控制帧

- 保留现有 `VS1,...,CRC\n` 视觉帧不变；
- 增加独立的 ASCII 行协议 `VC1,PING,<seq>,<crc>\n` 和 `VC1,PONG,<seq>,<crc>\n`；
- VC1 使用与 VS1 完全相同的 CRC-16-CCITT：`binascii.crc_hqx(body, 0xFFFF)`，CRC 字段为 4 位大写十六进制；
- RDK 每 1 秒发送一次递增序号 PING；
- Cyber Camera 非阻塞读取控制帧，仅对合法 PING 返回相同序号的 PONG；
- RDK 维护最近 4 个尚在 3 秒有效窗口内的已发序号；只接受与该窗口中任一序号匹配且 CRC 正确的 PONG。乱序的有效 PONG可以刷新在线时间，但重复、未知、过期或损坏帧不能刷新；
- RDK 的 PING 写入必须有独立 `try/except` 异常域。写超时或其他写异常只将发送方向标为 LOST并累计错误，禁止传播到接收循环的总异常处理，禁止清除 `_running`；
- Cyber Camera 只在完成当帧 VS1 写入之后尝试处理控制输入；每个图像循环最多读取固定 128 字节、最多解析并响应 1 条完整 VC1 帧，禁止循环排空积压数据；余下完整行留在有上限的接收缓冲区供后续帧处理；
- Cyber 控制接收缓冲区上限为 512 字节；超限时丢弃最旧的完整行并累计解析错误，若没有完整行则清空旧碎片，优先保留最新输入，避免恢复后应答已经离开 RDK 有效窗口的陈旧 PING；
- Cyber 的 PONG 写入异常只影响本次应答，不允许退出相机采集/检测/VS1发送主循环。

### OLED 状态

- `CAM>RDK:OK`：最近 1.5 秒内收到合法 VS1 帧；否则 `LOST`；
- `RDK>CAM:OK`：最近 2.5 秒内收到匹配 PONG；否则 `LOST`；
- RDK 串口进程以不高于 2 Hz 的频率，将单调时钟时间戳、计数、写入者 PID 和本次进程启动单调时间写入 `/dev/shm/competition_2026_d_vs1_status.json`；
- OLED 每 1 秒读取快照；快照缺失、格式错误或超过 3 秒未更新时，两行均显示 `LOST`；
- OLED 检测到 PID 或进程启动时间变化时显示一轮 `RESTART`，随后按新进程数据判断；正常运行中须连续两轮读到超时状态才从 OK 切换为 LOST，启动时或快照缺失/损坏时不做去抖、直接显示 LOST；
- 使用临时文件加原子替换，避免 OLED 读到半份 JSON；状态写入失败只记录诊断，不中断视觉接收线程。

## 改动范围

- `CyberCamera/boards/cybercam_d/protocol.py`
  - 增加 VC1 PING/PONG 编码、CRC 校验与解析。
- `CyberCamera/boards/cybercam_d/main.py`
  - 在现有串口对象上非阻塞接收 PING，并立即返回 PONG；限制每轮处理量，避免拖慢图像检测。
- `drone_control/competition_2026_d/vision/cybercam_protocol.py`
  - 增加与 Cyber Camera 一致的 VC1 控制帧解析/编码。
- `drone_control/competition_2026_d/vision/cybercam_reader.py`
  - 复用同一个串口周期发送 PING、解析 PONG并维护双向状态；继续解析 VS1；发布限频状态快照。
- `drone_control/competition_2026_d/vision/link_status.py`
  - 集中实现 tmpfs 状态快照的原子写入、读取、超时判断与 OLED 去抖，避免串口线程和OLED脚本重复定义状态语义。
- `drone_control/competition_2026_d/rdk_oled_monitor.py`
  - 将板上现有 T265 OLED 监控脚本纳入项目；保留 T265、VIDEO0、IP 三行，并增加两行双向链路状态。
- `drone_control/competition_2026_d/test_vs1_duplex.py`
  - 覆盖协议 CRC、分片/混合行、序号匹配、超时、状态快照损坏和串口异常。

部署时将 `rdk_oled_monitor.py` 精确同步为 `/home/sunrise/Desktop/auto-boot/t265_monitor.py`，重启 `t265-monitor.service`；不覆盖 RDK 远端仓库的其他脏改动。

RDK 实际端口唯一来源为 `drone_control/competition_2026_d/config.json` 的 `cybercam.port`，当前部署值为 `/dev/ttyS7`；类构造函数的默认端口不得用于正式入口。板端检查必须针对配置解析后的实际端口。

## 风险点

- **串口竞争：** OLED 禁止打开 `/dev/ttyS7`，只有 `CyberCamReader` 持有串口。
- **发送阻塞：** PING/PONG 使用短写超时或非阻塞写，并分别处于独立异常域；异常只更新发送错误状态，不得停止视觉接收或相机主循环。
- **协议互扰：** VS1 与 VC1 共用换行分帧，按前缀分流；未知行只计错误，不污染下一帧。
- **假在线：** 只有合法 VS1、匹配 PONG分别刷新两个方向；快照和心跳均有硬超时。
- **相机推理延迟：** Cyber 每个图像循环最多读取128字节且最多处理一条VC1，不排空积压；RDK 接受最近4个有效PING序号，PONG超时留出推理与调度抖动余量。
- **老版本兼容：** 新 RDK 配旧 Cyber 时接收方向仍可 OK，发送方向保持 LOST，不影响现有 VS1 解析。
- **快速重启：** 状态快照包含 PID 和启动标识；OLED 至少显示一轮 RESTART，不把旧进程遗留快照连续呈现为在线。
- **飞行安全：** OLED 与心跳状态是诊断信息，不接入速度输出，也不放宽现有“起飞前收到 VS1”和视觉丢失归零规则。
- **回退方案：** 停止 OLED 服务并恢复原 `t265_monitor.py`；RDK 关闭 PING 后，现有单向 VS1 数据格式仍完全兼容。Cyber 侧 VC1 逻辑是非阻塞的被动响应器，没有 PING 时不产生 PONG且每帧只做一次空缓冲检查，因此可保留；如需完全回退则同步恢复 Cyber 的 `protocol.py` 和 `main.py`。

## 验证方式

- 单元测试：VC1 正常/坏 CRC/坏序号/分片/与 VS1 交错；最近4个PING的延迟/乱序PONG；两个方向独立超时；PID变化RESTART；状态文件损坏和写入失败。
- 异常隔离测试：模拟 RDK `serial.write()` 抛出 `SerialTimeoutException` 后继续接收合法 VS1；模拟 Cyber PONG 写失败后继续采集并发送 VS1；验证积压控制帧每个图像循环最多处理1条。
- 板端静态测试：不连接飞控、不解锁，连续运行至少 30 秒，要求 VS1 无解析错误、PING/PONG连续匹配。
- 断连测试：分别断开 Cyber TX→RDK RX、RDK TX→Cyber RX，确认对应 OLED 状态在超时内转为 LOST；恢复后自动转 OK。
- 资源测试：确认只有一个进程打开 `/dev/ttyS7`，OLED 刷新不降低视觉帧接收率。
- 回归测试：现有 VS1 检测、静态方块控制器与基础飞控专项测试全部通过。
