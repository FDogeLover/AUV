# D题统一通信协议

状态：DCP v1冻结；CAR_STATE 13字节扩展已冻结
唯一维护位置：本文件

## 1. 适用链路

| 链路 | 端口/传输 | 波特率 | 协议 |
|------|-----------|--------|------|
| Cyber Camera → Pi/RDK | UART，设备名实施时配置 | 115200，必要时460800 | VS1观测帧 |
| 小车 ↔ 无人机 | 无人机 `/dev/bt_serial`，小车配对端 | 115200 8N1 | DCP v1 |
| 无人机 ↔ STM32 | 现有飞控串口 | 保持现状 | 现有速度/命令/遥测帧，不并入DCP |
| 无人机/小车 → 地面站 | 待硬件连接方式确认 | 与适配器一致 | DCP v1，只读遥测 |

板端 `/dev/bt_serial` 当前指向 CP2102 `/dev/ttyUSB0`。代码打开端口时显式设置115200；不得依赖系统空闲时的 `stty` 显示值。

## 2. DCP v1帧格式

小车、无人机和地面站使用长度驱动的二进制流协议：

| 字段 | 类型 | 说明 |
|------|------|------|
| magic | u8 | 固定 `0xAA` |
| version | u8 | 固定 `0x01` |
| type | u8 | 消息类型 |
| flags | u8 | ACK/事件/错误标志 |
| source | u8 | 发送设备 |
| dest | u8 | 目标设备或广播 |
| session_id | u32 | 每次任务唯一编号 |
| seq | u16 | 发送端单调递增，回绕允许 |
| sender_ms | u32 | 发送端单调毫秒计时 |
| payload_len | u16 | 载荷长度，v1上限256字节 |
| payload | bytes | 按消息类型定义 |
| crc16 | u16 | CRC-CCITT，初值 `0xFFFF`，覆盖version至payload |
| tail | u8 | 固定 `0xFF` |

- 所有多字节整数采用小端序。
- 接收器必须按长度取帧，不把payload内部的 `0xAA/0xFF` 当分隔符。
- 长度越界、CRC错误、版本错误、来源错误和过期session全部拒绝。
- 流解析器遇到坏帧后搜索下一个magic恢复，不阻塞接收线程。

## 3. 设备编号

| 设备 | 值 |
|------|----|
| UAV | `0x01` |
| CAR | `0x02` |
| GROUND | `0x03` |
| BROADCAST | `0xFF` |

## 4. 标志位

| bit | 名称 | 含义 |
|-----|------|------|
| 0 | ACK_REQUIRED | 需要确认 |
| 1 | IS_ACK | 本帧为确认 |
| 2 | EVENT | 状态事件，不是周期采样 |
| 3 | ERROR | 故障或拒绝结果 |

## 5. 消息类型与载荷

所有位置单位为毫米，速度为毫米/秒，角度为0.01度，高度为毫米，时间为毫秒。

| type | 名称 | 方向 | 可靠性 | v1载荷 |
|------|------|------|--------|--------|
| `0x01` | HEARTBEAT | 任意→任意 | 最新值 | `state:u8, fault_bits:u16` |
| `0x02` | UAV_READY | UAV→CAR/GROUND | 周期广播 | `task_mask:u8, ready_bits:u16, config_hash:u32` |
| `0x03` | CAR_START | CAR→UAV | 必须ACK、幂等 | `task_mode:u8, car_config_hash:u32` |
| `0x04` | ACK | 任意→任意 | 不再ACK | `acked_type:u8, acked_seq:u16, result:u8` |
| `0x10` | CAR_STATE | CAR→UAV/GROUND | 10 Hz最新值 | `segment:u8, track_s_mm:u16, speed_mm_s:i16, heading_cdeg:i16, state_flags:u16, vx_mm_s:i16, vy_mm_s:i16` |
| `0x11` | UAV_STATE | UAV→CAR/GROUND | 10 Hz最新值 | `x_mm:i32, y_mm:i32, z_mm:i16` |
| `0x12` | FUSED_POSITION | CAR→GROUND | 每帧UAV_STATE触发 | `car_x_mm:i32, car_y_mm:i32, uav_x_mm:i32, uav_y_mm:i32, uav_z_mm:i16, uav_seq:u16, uav_sender_ms:u32, car_pose_age_ms:u16, flags:u16` |
| `0x13` | CAR_POSITION | CAR→UAV | 10 Hz最新值 | `car_x_mm:i32, car_y_mm:i32, car_pose_age_ms:u16, flags:u16` |
| `0x20` | UAV_EVENT | UAV→CAR/GROUND | 事件、必须ACK | `phase:u8, elapsed_ms:u32` |

后续增加字段只能新增消息版本或追加可判长的尾部字段，不得改变v1已有字段语义。

### CAR_STATE冻结定义

`CAR_STATE`的新载荷为13字节，Python解包格式为`<BHhhHhh`。前9字节的字段顺序和
语义保持不变，尾部追加世界坐标速度分量：

| payload偏移 | 类型 | 字段 | 说明 |
|---:|---|---|---|
| 0 | `u8` | `segment` | 当前赛道分段 |
| 1 | `u16` | `track_s_mm` | 从A点横线起累计路径长度，mm |
| 3 | `i16` | `speed_mm_s` | 车体中心有符号合速度，mm/s |
| 5 | `i16` | `heading_cdeg` | 从小车世界`+X`起逆时针为正，0.01° |
| 7 | `u16` | `state_flags` | 小车状态位 |
| 9 | `i16` | `vx_mm_s` | 小车世界X轴速度，mm/s |
| 11 | `i16` | `vy_mm_s` | 小车世界Y轴速度，mm/s |

`state_flags`定义：

- bit0：已进入正常循迹阶段。
- bit1：循迹控制器确认处于弯道。
- bit2：已通过C点并切换速度档。
- bit3：已越过终点识别使能距离。
- bit4 `0x0010`：本帧编码器速度有效；未置位时无人机不得使用速度前馈。
- bit5..15：保留，发送端必须置0。

小车和无人机后续将统一世界速度坐标系，因此接收端当前原样保存`vx/vy`，不进行坐标旋转。
在两端坐标定义完成联合校准前，`speed/vx/vy`都只用于通信记录和诊断，不参与任务一飞行
控制，也不直接写入飞控水平速度。

无人机按同一会话的u16半区规则只接受更新seq，允许`65535→0`回绕；
接收超过300 ms未更新、session不匹配、bit4无效、保留位非零或数值越界时，速度查询
立即返回不可用，不能无限复用上一帧。

旧9字节载荷允许兼容读取基础字段，但没有`vx/vy`，不得伪造世界速度；正式联调应统计并
提示旧帧，以便发现小车固件未升级。

`UAV_STATE`只传T265坐标，正常按`10 Hz`发送。`x/y/z`均为以本次T265初始化后的
H点为原点的世界坐标，单位毫米；不传速度、视觉质量或状态位。正式任务默认使用T265
高度，因此`z`表达T265世界Z坐标。

无人机阶段变化时立即发送一帧可靠`UAV_EVENT`。`phase`使用下节统一枚举，
`elapsed_ms`为本次无人机任务启动后的毫秒数。起飞、伴飞、抛投、返航、降落和完成等
状态都通过该消息上报；中文文字由小车/地面站按枚举映射，不传自由文本。

地面站超过`500 ms`未收到新`UAV_STATE`时必须把坐标显示为“已过期/链路异常”；
阶段文字保持最近一次已ACK的`UAV_EVENT`，直到收到下一阶段事件。

`CAR_POSITION`使用场地左下角为原点、向右为`+X`、向上为`+Y`的全局坐标。
任务建立前允许`session_id=0`；收到并ACK非零session的`CAR_START`后，后续坐标必须使用
同一session。flags bit0为车辆坐标有效、bit2为车辆坐标年龄不超过200 ms、bit3表示
session非零，其余保留位必须为0。无人机当前只记录该坐标，不参与任务一飞行控制。

`FUSED_POSITION`由小车在每次收到新`UAV_STATE`后生成并只发往地面站，无人机不接收。
小车把H点相对无人机坐标和平移后的车辆T265坐标统一转换为场地全局坐标。

## 6. 状态枚举

### task_mode

- `1`：抛投任务。
- `2`：动态起降任务。

### segment

- `0`：UNKNOWN。
- `1`：A_B。
- `2`：B_C。
- `3`：C_D。
- `4`：D_A。

### UAV phase

- `0`：BOOT。
- `1`：WAIT_T265。
- `2`：READY。
- `3`：TAKEOFF。
- `4`：HOLD_3S。
- `5`：INTERCEPT。
- `6`：FORMATION_FOLLOW。
- `7`：DROP。
- `8`：DESCEND。
- `9`：TOUCHDOWN。
- `10`：DECK_RIDE。
- `11`：RETAKEOFF。
- `12`：RETURN_H。
- `13`：LAND_H。
- `14`：COMPLETE。
- `15`：FAULT。
- `16`：TERMINAL_PREDICT（近地短时预测下降）。
- `17`：CONTROLLED_ABORT（受控中止）。
- `18`：SEARCH_TARGET（搜索小车）。

## 7. 启动握手

```text
 UAV先完成蓝牙、Cyber Camera、飞控串口和投放舵机预检
 -> T265暂不初始化，程序停在原本的本地一键启动阻塞位置
 -> 绿灯常亮，表示无人机正在等待小车一键启动
 -> 周期发送 UAV_READY(session_id=0)
 CAR收到连续有效READY并允许按钮
 -> 按键后生成新session_id并立即行进
 -> 重发 CAR_START(session_id)
 UAV只在READY状态接受一次
 -> 任务绑定session_id
 -> 返回ACK(CAR_START)
 -> 收到CAR_START立即由绿灯切换为红灯
 -> CAR_START直接解除T265拔插阻塞，红灯警示与T265初始化/校准并行
 -> 红灯累计满5秒且T265严格预检通过后开始起飞
 -> T265失败则取消任务，不等待终端人工输入
```

重复 `CAR_START` 不得重复起飞；session不匹配的旧包不得改变状态。

## 8. ACK与重发

- `CAR_START`和`UAV_EVENT`设置ACK_REQUIRED。
- 发送端按有限间隔重复，收到匹配的 `acked_type + acked_seq + session_id` 后停止。
- 接收端处理事件必须幂等：重复帧只重发ACK，不重复执行动作。
- 周期状态不重传；接收端只保留最新有效seq。
- 重发和超时参数在各端配置中可调，但语义由本规范统一。

## 9. 广播与双向通信

- `dest=BROADCAST` 表示逻辑广播；是否能被多个物理接收端同时收到取决于蓝牙模块能力。
- 车机安全关键事件使用定向双向可靠模式。
- 地面站遥测使用广播/旁路监听模式，发送失败不得阻塞车机链路。
- 若现有蓝牙模块只支持单个SPP连接，车机连接优先；地面站改用独立接收适配器或由一端转发，不能以地面显示换取车机链路不稳定。

## 10. 链路失效

- 小车状态超时但视觉正常：无人机继续视觉闭环并标记降级。
- VS1观测陈旧：暂停视觉动作，不复用最后一帧。
- 动态降落任务中未收到新的降落阶段事件时，小车维持既定低速策略；无可靠链路时保持低速至A点停车。
- 地面站断开：任务继续。
- Pi/RDK与STM32失联：执行既有飞行安全策略。

## 11. VS1视觉观测帧

Cyber Camera输出短ASCII帧：

```text
VS1,stream_id,seq,capture_ms,found,cx,cy,outer_px,inner_px,angle_cdeg,quality,flags,crc16\n
```

- `stream_id` 每次Cyber Camera进程启动时改变；接收端连续确认3帧CRC正确、流标识一致且seq递增后才能切换到新流。
- `seq` 每个采集帧递增；重复seq不能重复更新控制器。
- Pi记录本地接收时间并判断新鲜度。
- `quality` 范围0~100。
- `flags` 定义 `OUTER_VALID/INNER_VALID/CROSS_VALID/PARTIAL/TOO_CLOSE/AMBIGUOUS/SURROGATE_SQUARE/APRILTAG_VALID/TEMPORAL_TRACKED/COLOR_SHAPE_TRACKED`。`APRILTAG_VALID` 为 bit 7；`TEMPORAL_TRACKED` 为 bit 8，表示该观测由最近一次正确ID直接解码初始化后的短时光流续跟产生；`COLOR_SHAPE_TRACKED` 为 bit 9，表示已由正确ID Tag 建立身份锁、当前中心来自与Tag几何一致且连续的蓝色外框。光流和颜色均不得独立确认身份；当前移动目标专项中可靠完整中心观测最高限速0.15 m/s，`PARTIAL_COARSE`仍限速0.06 m/s。VS1字段数和CRC范围保持不变。
- `SURROGATE_SQUARE` 仅用于器材未齐时的25 cm蓝色方形静态控制测试；此时 `outer_px` 表示方形平均像素边长。正式任务默认拒绝该flag，只有显式测试配置才允许进入视觉伺服。
- 当前单Tag方案只接受配置的Tag ID，Tag贴在降落圆盘中心；VS1中`cx/cy`表示Tag中心，`outer_px`表示四边平均像素长度，`inner_px`置0，`angle_cdeg`表示Tag平面航向。Tag ID只在Cyber Camera本地完成白名单校验，不增加VS1字段。
- RDK的 `vision_target_source` 只允许 `apriltag` 或 `blue_square`，两种来源互斥且只在进程启动时读取。融合模式仍归入经过身份确认的 `apriltag` 来源，使用 `APRILTAG_VALID|COLOR_SHAPE_TRACKED`，不得置 `SURROGATE_SQUARE`。`APRILTAG_VALID` 与 `SURROGATE_SQUARE` 同时置位、`APRILTAG_VALID|PARTIAL`、错误ID或多Tag场景均不得进入闭环。
- 部署顺序固定为先RDK、后Cyber Camera；两端完成VS1和PING/PONG验证后才允许装桨。版本不确定时按无视觉能力处理。
- 若以后增加多Tag或Tag不再位于圆盘中心，必须定义VS2并显式传递`target_kind/target_id`；不得改变VS1现有字段语义，也不得由各模块私自扩展不同格式。
- 正式目标特征来源切换需要连续多帧确认；来源冲突或中心突跳必须设置 `AMBIGUOUS`，控制端不得使用。
- CRC16覆盖 `VS1` 至 `flags` 的ASCII字节。
- UART不传原始图像。

## 12. 统一实现与测试向量

计划提供一个Python参考编解码实现和固定测试向量。四组不得复制后自行改动：

- Python端共享同一模块。
- 小车MCU/其他语言实现必须通过相同黄金帧测试。
- 每次协议升级同时提交正常帧、CRC错误帧、截断帧、重复事件和旧session测试向量。

冻结的`CAR_STATE`黄金帧字段为
`session=0x12345678, seq=42, sender_ms=1000, segment=A_B, track_s=1500 mm,
speed=130 mm/s, heading=0, state_flags=0x0011, vx=130 mm/s, vy=0`，完整34字节为：

```text
AA 01 10 00 02 01 78 56 34 12 2A 00 E8 03 00 00
0D 00 01 DC 05 82 00 00 00 11 00 82 00 00 00 B9
42 FF
```

CRC为`0x42B9`，在线路中按小端顺序发送`B9 42`。
