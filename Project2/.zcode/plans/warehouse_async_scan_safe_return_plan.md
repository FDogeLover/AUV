# 仓库盘点异步扫码定点与安全返航设计

日期：2026-07-20
状态：待 Qoder Plan Review

## 问题描述与目标

### 已证实问题

2026-07-20 真机数据表明，`InventoryMissionCoordinator._inspect_slot()` 在飞行主线程中同步执行扫码循环。扫码期间 `Mission_GPT.loop()` 无法继续调用 `navigate()`，现有 T265 XY 位置 PID 停止更新。`hold_position()` 实际只发送水平零速度，不锁定固定 XY 坐标，扫码9秒期间位置从约 `(-1.666, 0.080)` 漂移到 `(-1.704, 0.196)`，用户观察到明显左右晃动。

同时，`qr_timeout` 当前通过 `_abort()` 立即触发原地 `LAND`。最新测试在约1.27m高度进入LAND后64秒仍未下降，固件报告高空降落超时放弃自动锁桨，最终由用户人工接管。当前RETURN只是日志状态，没有实际返航导航。

### 目标

1. QR解码与图片存档不得阻塞约30ms飞行控制循环。
2. 扫码期间继续使用现有T265 XY PID锁定到达货位时的航点坐标。
3. 扫码worker不得写入飞行指令或修改飞行状态。
4. QR超时/视觉失败时，优先沿安全路线返回降落区，再进入LAND。
5. 返航航点超时则在当前位置受控降落；若固件拒绝高空降落，继续维持T265通信等待人工接管。
6. 单货位只扫描一轮，默认8秒，不在货架旁重复扫码。

## 方案选择

| 方案 | 优点 | 缺点/风险 |
|------|------|-----------|
| 受限扫码worker + 保留当前航点 | 改动较集中；复用navigate() PID | 到达窗口去确认后无法稳定轮询worker，Qoder审查判定为高风险，不采用 |
| 新增显式SCAN飞行状态 | 主线程每tick位置闭环和worker轮询都有稳定入口；航点超时语义清楚 | 需要正式修改Mission_GPT核心调度和测试 |
| 主线程每tick解一帧 | 无线程同步问题 | pyzbar单帧0.5~1.4秒，仍阻塞控制循环 |

选择第二种方案：显式`SCAN`飞行状态。第一种方案在Qoder Plan Review中被发现存在到达窗口去确认后worker结果无人消费、航点超时误推进的高风险，不再采用。

## 架构设计

### 1. 扫码任务模型

Coordinator新增私有扫码任务状态，至少包含：

- `IDLE`：没有扫码任务；
- `RUNNING`：worker正在处理当前货位；
- `SUCCEEDED`：worker已产出稳定QR结果；
- `FAILED`：timeout、camera、decode、capture或cancel失败；
- `CONSUMED`：主飞行线程已处理结果，防止重复副作用。

每次任务带单调递增`generation`。worker输出必须包含generation、waypoint_index、slot_label、状态、检测结果或错误码。主线程只消费与当前任务generation完全匹配的结果；取消任务后到达的旧结果必须丢弃。

### 2. worker职责边界

worker允许：

- 调用`CameraSource.read_with_sequence()`获取最新帧；
- 跳过重复sequence；
- 在解码前保存调试图片；
- 调用target-aware快速pyzbar ROI路径；
- 使用worker私有`QRConsensus`做多帧共识；
- 写入线程安全的不可变结果对象；
- 响应`threading.Event`取消信号。

worker禁止：

- `set_speed()`、`hold_position()`或任何飞行指令；
- 修改`Mission_GPT.state`、InventoryState或`target_index`；
- 调用激光、store.add、ground_link；
- 调用`_abort()`、`abort_to_land()`、LAND；
- 复用跨任务共享且无锁的consensus实例。

### 3. 飞行主线程与显式SCAN状态

第一次满足INSPECT航点到达条件时：

1. Coordinator锁存waypoint index、slot、scan deadline与generation；
2. 启动且仅启动一个扫码worker；
3. `InventoryFlightMission`从`NAVIGATE`切换到显式`SCAN`状态；
4. 锁存INSPECT航点的固定XYZ目标，正常推进逻辑和arrival timeout暂停；
5. SCAN不依赖后续到达窗口继续成立。

`Mission_GPT.loop()`增加SCAN调度扩展点。每个约30ms tick由主飞行线程：

- 读取T265位置、航向和tracking confidence；
- 使用锁存的INSPECT X/Y目标持续执行现有位置PID，避免零速度漂移；
- 保持与navigate()一致的Z ramp/目标高度和heading hold输出，避免通道切换跳变；
- O(1)轮询worker结果；
- 持续记录控制周期、位置误差与loop jitter；
- tracking confidence降为0时立即取消worker并升级为不可导航故障策略。

worker结果处理：

- RUNNING：留在SCAN；
- SUCCEEDED：主线程执行激光、冲突检查、store.add、广播和状态转换，原子恢复正常targets并进入NAVIGATE；
- FAILED：主线程取消并join worker，安装安全返航targets并进入RETURN语义下的NAVIGATE；
- stale generation：丢弃并记录诊断。

所有状态机副作用只由主飞行线程执行一次。

### 4. 生命周期与取消

以下情况必须设置cancel event并有界join：

- QR成功并完成结果消费；
- QR timeout或worker异常；
- emergency stop、T265丢失、FC通信超时；
- 主任务停止或CameraSource关闭；
- 切换到其他货位或generation。

join必须有超时，不能阻塞飞行控制；未及时退出的worker标记为诊断故障，旧结果仍由generation隔离。

### 5. 安全返航

#### 故障分类

可导航故障：

- `qr_timeout`；
- `qr_duplicate`；
- camera/decode/capture异常（前提是T265与FC通信仍健康）；
- laser或地面广播失败。

不可继续导航故障：

- T265追踪丢失；
- 飞控通信异常；
- 用户紧急停止；
- 明确的硬件安全故障。

#### 可导航故障流程

1. 停止并取消扫码worker；
2. 从当前货位/当前位置生成到landing approach的安全返航路线；
3. 安全路线必须复用`InventoryPlanner._safe_transit()`同等规则，跨货架平面时包含bypass点；
4. 通过Mission层原子API `replace_navigation_targets(new_targets, current_pos)`安装返航路线；该API必须同时替换targets、设置target_index、重置`last_target_index`、arrival窗口、速度窗口、PID、到达确认和航点计时，不触发旧航点完成回调；
5. 到达`LAND_APPROACH`后才进入LAND；
6. 返航中任一航点超时，升级为当前位置受控LAND；
7. LAND若收到`land_timeout_gaveup`，不关闭串口/T265，持续等待人工接管。

#### 不可导航故障流程

保持现有紧急策略：立即进入受控LAND或等待人工接管，不尝试依赖失效定位继续返航。

### 6. 状态语义

InventoryState需要让RETURN成为真实持续状态，而不是`FAULT -> RETURN -> LAND`零时长过渡。建议流程：

```text
VERIFY_QR --success--> ILLUMINATE --> REPORT --> TRANSIT
VERIFY_QR --navigable failure--> FAULT --> RETURN --arrived--> LAND
RETURN --waypoint timeout--> LAND
RETURN --T265/FC failure--> LAND/emergency policy
```

FlightMission层仍可保持`NAVIGATE`执行返航targets；InventoryState用RETURN描述任务语义。只有到达landing approach或返航失败升级时，driver.state才切换为LAND。

## 数据流

```text
CameraSource采集线程
    ↓ 最新帧副本 + sequence
ScanWorker（只读图像）
    ↓ ScanResult(generation, status, detection/error)
Mission主线程轮询
    ├─ RUNNING → 继续navigate()位置PID
    ├─ SUCCESS → 激光/保存/广播/推进
    └─ FAILURE → 安全返航targets → navigate() → LAND
```

## 错误处理

- worker所有异常转为结构化错误，不允许异常逃逸结束进程；
- 图片保存失败不应停止飞行控制，但应标记worker失败或诊断事件，具体按配置决定；本设计默认图片存档是调试功能，保存失败仅记录，不终止扫码；
- decoder异常终止本轮扫码并进入安全返航；
- worker超时只由单调时钟判定；
- 状态机副作用只能在主线程发生一次；
- stale result必须记录generation但不得触发激光、存储或返航。

## 改动范围

预计影响：

- `drone_control/warehouse_inventory/Lcode/inventory_controller.py`
  - 扫码worker、结果对象、任务生命周期；
  - `_inspect_slot()`改为非阻塞启动/轮询；
  - `InventoryFlightMission`扫码轮询与返航切换；
  - worker关闭与driver停止联动。
- `drone_control/warehouse_inventory/Lcode/inventory_planner.py`
  - 暴露从当前点/货位到landing approach的安全返航规划API，避免调用私有方法。
- `drone_control/warehouse_inventory/Lcode/inventory_state.py`
  - RETURN持续状态与转换语义。
- `drone_control/warehouse_inventory/Mission_GPT.py`
  - 正式增加SCAN调度扩展点；
  - 抽取可复用的位置保持tick（XY PID + Z ramp + heading hold）；
  - 增加原子`replace_navigation_targets()`，完整重置到达跟踪状态；
  - 区分正常航点、SCAN和RETURN航点超时策略。
- `drone_control/warehouse_inventory/main.py`
  - 注入planner/return route依赖或配置。
- 测试：`test_inventory_controller.py`、`test_inventory_state.py`、`test_warehouse_model.py`或新增异步扫码测试文件。

## 风险点与控制

### 高风险

1. worker晚到结果误作用于下一货位：generation严格隔离。
2. worker与主线程同时修改状态/指令：worker权限边界禁止任何飞行副作用。
3. 同一到达条件重复启动worker：原子任务状态与单实例保护。
4. 可导航故障错误地立即LAND：新增恢复分类，测试断言qr_timeout进入RETURN而非直接LAND。
5. 返航路径穿越货架：复用安全transit规划并测试每个货架面。

### 中风险

1. worker线程的CPU/GIL/内存带宽影响控制周期：这是部署硬门控，不是后续优化。桌面和板端必须记录SCAN期间loop tick间隔；**最大jitter超过100ms则线程方案禁止真机部署，改为独立进程**；
2. CameraSource关闭竞态：先cancel/join worker，再close camera；join超时≤2s；worker必须daemon；旧worker未退出时拒绝启动新任务；
3. worker线程遗留：任务结束断言无活跃worker，join有界；
4. 调试存图I/O影响worker吞吐：会话目录在worker启动前预创建，保存失败只记录诊断，不影响飞行控制；
5. timeout唯一权威：worker正常超时为8s；主线程只设置worker timeout + 2s后备deadline，后备触发时立即取消generation并进入失败流程；
6. T265健康判据：SCAN每tick检查tracking confidence；confidence=0立即取消扫码并进入不可导航故障策略。

## 回退方案

- 环境变量提供`DRONE_ASYNC_QR_SCAN`，但**真机模式必须为1**；为0时在preflight拒绝起飞并记录`async_scan_required`，不允许隐式回退同步扫码或起飞后立即返航。
- Git可回退到当前已验证提交`0325992`，但该版本真机已证实扫码阻塞位置PID，不作为长期飞行版本。
- 安全返航若桌面测试未通过，先只部署异步扫码定点，qr_timeout仍由人工接管测试，不部署未验证返航路线。

## 验证方式

### 单元测试

1. 显式SCAN状态只启动一个worker，不依赖到达窗口持续确认；
2. 慢decoder阻塞时主线程仍执行多个SCAN位置保持tick；
3. SCAN期间T265漂移产生指向锁存INSPECT目标的PID命令，而非恒零；
4. 位置短暂超出arrival范围再返回时，SCAN不中断、不触发普通航点超时、结果仍能消费；
5. SCAN的Z ramp和heading hold与NAVIGATE切换前后连续；
6. worker成功结果只消费一次，激光/store/publish/推进各一次；
7. timeout/cancel后的旧generation结果不得影响新货位；
8. camera/decode/capture异常结构化返回；
9. shutdown先cancel/join worker，再close camera；旧worker未退出则拒绝新任务；
10. qr_timeout进入真实RETURN且不立即LAND；
11. 原子替换返航targets后arrival窗口、PID和航点计时均已重置；
12. 各货架面安全返航包含必要bypass；
13. RETURN航点超时进入LAND；
14. SCAN/RETURN中T265 confidence=0或FC失败升级紧急策略；
15. `DRONE_ASYNC_QR_SCAN=0`在真机preflight拒绝起飞。

### 桌面集成测试

- 使用阻塞Event模拟2秒decoder，验证飞行loop tick仍约30ms；
- 记录最大loop jitter、worker数量、generation；线程方案部署硬门控为最大jitter≤100ms；
- 模拟扫码成功、超时、迟到结果、异常、取消；
- 模拟从A/B/C/D不同货位故障并检查返航路径不穿越货架。

### 真机分阶段验证

第一阶段（不放QR结果也可）：

- 单A1测试；视觉伺服关闭；异步扫码开启；
- 验证扫码8秒期间flight log持续约30ms记录；
- XY最大漂移目标不超过现有precision悬停基线约15cm；
- 图片数量明显增加且不影响控制循环；
- 人工保持随时接管。

第二阶段：

- QR成功路径：解码→激光→结果保存→正常返航降落；
- QR失败路径：8秒后沿安全路线返回landing approach，再降落；
- 返航航点超时采用受控LAND升级策略；
- 每次飞行前独立安全确认，测试数据同步回本机。

## 明确不在本次范围

- 恢复或调优视觉伺服；
- 更换二维码算法或训练模型；
- 修复凌霄IMU/飞控一键降落底层固件故障；
- 多货位并行扫码；
- 将worker改为独立进程（除非GIL测量证明线程不可用）。
